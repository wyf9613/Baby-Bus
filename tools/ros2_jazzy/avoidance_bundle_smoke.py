#!/usr/bin/env python3
"""Exercise the actual bundled ROS node with synthetic DDS messages only."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, Int8, String, UInt16
from dream_interfaces.msg import ConeDetection, ConeDetections, DriveAndSteer
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--road-width',type=float,default=2.)
    parser.add_argument('--obstacle-offset',type=float,default=0.)
    parser.add_argument('--obstacle-width',type=float,default=.20)
    parser.add_argument('--obstacle-distance',type=float,default=2.8)
    parser.add_argument('--roadside-returns',action='store_true')
    parser.add_argument('--scan-step-deg',type=float,default=3.)
    args=parser.parse_args()
    if not 0.1 <= args.scan_step_deg <= 3.:parser.error('scan-step-deg must be in [0.1, 3]')
    rays_per_half=round(180./args.scan_step_deg)
    args.output.mkdir(parents=True,exist_ok=False)
    params=yaml.safe_load((args.package/'ai4r_policy.yaml').read_text())['/**/ai4r_policy']['ros__parameters']
    namespace='/avoidance_bundle_check'
    rclpy.init();node=Node('synthetic_avoidance_peers')
    pubs={key:node.create_publisher(typ,namespace+'/'+topic,10) for key,typ,topic in (
        ('cones',ConeDetections,'cone_detections'),('scan',LaserScan,'scan'),
        ('wheel',Float32,'wheel_speed_m_per_sec'),('imu',Imu,'imu/data'),
        ('request',UInt16,'policy_fsm_transition_request'))}
    actions=[];states=[];execution=[];worker=[]
    static_scan_cache={};scan_publish_times=[]
    node.create_subscription(DriveAndSteer,namespace+'/drive_and_steer_set_point_normalized',
        lambda m:actions.append(dict(t=time.monotonic(),drive=float(m.drive),steer=float(m.steer))),10)
    node.create_subscription(Int8,namespace+'/policy_fsm_state_value',lambda m:states.append(m.data),10)
    node.create_subscription(String,namespace+'/planner_execution',lambda m:execution.append(json.loads(m.data)),10)
    node.create_subscription(String,namespace+'/shadow_planning',lambda m:worker.append(json.loads(m.data)),10)
    log=(args.output/'node.log').open('w')
    process=subprocess.Popen([sys.executable,str(args.package/'policy_node.py'),'--ros-args',
        '--params-file',str(args.package/'ai4r_policy.yaml'),'-r','__ns:='+namespace],stdout=log,stderr=log)

    def run(seconds, distance=None, *, rear_return=False):
        end=time.monotonic()+seconds;motion=scan=0.
        while time.monotonic()<end:
            if process.poll() is not None:raise AssertionError('Bundled node exited')
            now=time.monotonic();stamp=node.get_clock().now().to_msg()
            if now>=motion:
                pubs['wheel'].publish(Float32(data=0.))
                imu=Imu();imu.header.stamp=stamp;imu.header.frame_id='base_link'
                imu.orientation_covariance[0]=-1.;imu.linear_acceleration_covariance[0]=-1.
                imu.angular_velocity_covariance[0]=0.;pubs['imu'].publish(imu)
                motion=now+.05
            if now>=scan:
                batch=ConeDetections();batch.header.stamp=stamp;batch.header.frame_id='base_link'
                for color,side in ((params['estimation']['left_colour'],args.road_width/2),
                                   (params['estimation']['right_colour'],-args.road_width/2)):
                    for i in range(11):
                        cone=ConeDetection();cone.position.x=.3+.28*i;cone.position.y=side
                        cone.color=color;cone.classification_confidence=.99;batch.detections.append(cone)
                pubs['cones'].publish(batch)
                msg=LaserScan();msg.header.stamp=stamp;msg.header.frame_id='base_link'
                msg.angle_min=-math.pi/2;msg.angle_increment=math.pi/rays_per_half;msg.angle_max=math.pi/2
                msg.range_min=.05;msg.range_max=12.;msg.ranges=[math.inf]*(rays_per_half+1)
                if rear_return:
                    msg.angle_min=-math.pi;msg.angle_max=math.pi;msg.ranges=[math.inf]*(2*rays_per_half+1)
                    index=round((math.atan2(.1946268196016643,-.22286008138405408)-msg.angle_min)/msg.angle_increment)
                    msg.ranges[index]=math.hypot(.1946268196016643,.22286008138405408)
                # This DDS fixture has stationary geometry and zero wheel speed.
                # Cache ray/box intersections; stamps are fresh on every publish.
                key=(distance,rear_return)
                if key in static_scan_cache:
                    msg.ranges=static_scan_cache[key]
                else:
                    if distance is not None:
                        boxes=[(distance,distance+.25,args.obstacle_offset-args.obstacle_width/2,
                            args.obstacle_offset+args.obstacle_width/2)]
                    else:boxes=[]
                    if args.roadside_returns:
                        boxes.extend((.3+.28*k-.08,.3+.28*k+.08,y-.08,y+.08)
                            for k in range(11) for y in (-args.road_width/2,args.road_width/2))
                    for i in range(len(msg.ranges)):
                        angle=msg.angle_min+i*msg.angle_increment
                        for x0,x1,y0,y1 in boxes:
                            lo,hi=0.,12.
                            for direction,a,b in ((math.cos(angle),x0,x1),(math.sin(angle),y0,y1)):
                                if abs(direction)<1e-10:
                                    if not a<=0.<=b:hi=-1.
                                else:
                                    u,v=sorted((a/direction,b/direction));lo=max(lo,u);hi=min(hi,v)
                            if lo<=hi and lo>=msg.range_min:msg.ranges[i]=min(msg.ranges[i],lo)
                    static_scan_cache[key]=list(msg.ranges)
                scan_publish_times.append(time.monotonic())
                pubs['scan'].publish(msg);scan=now+.1
            rclpy.spin_once(node,timeout_sec=.005)

    report={'front_proximity_stop_enabled':params['obstacle_stop'].get('proximity_stop_enabled',True)}
    try:
        for _ in range(20):
            run(.5,2.8)
            if states and actions:break
        assert states and states[-1]==2 and actions and all(a['drive']==a['steer']==0. for a in actions)
        report['startup_zero']=True
        pubs['request'].publish(UInt16(data=3));index=len(actions)
        if params['lidar_avoidance'].get('maneuver_enabled'):
            run(4.,rear_return=True)
            assert states[-1]==3,(states,(args.output/'node.log').read_text()[-2000:])
            assert any(a['drive']>0 for a in actions[index:])
            assert any(x.get('lidar_avoidance',{}).get('protection_mode')=='baseline_forward_stop' for x in execution)
            report['normal_following_with_rear_return_over_four_seconds']=True
            if params.get('learned_policy',{}).get('enabled'):
                assert any((x.get('policy_selection') or {}).get('status')=='SELECTED' for x in worker)
                assert any(x.get('selection_backend')=='learned_policy' and
                    x.get('learned_policy',{}).get('state_commits',0)>0 for x in execution)
                report['rnn_selected_and_execution_memory_committed']=True
            index=len(actions)
        run(3.,args.obstacle_distance)
        assert states[-1]==3,(states,(args.output/'node.log').read_text()[-2000:])
        assert any(a['drive']>0 and abs(a['steer'])>.01 for a in actions[index:])
        assert any(x.get('mode')=='planner' and x.get('lidar_avoidance') is not None for x in execution)
        assert any((x.get('obstacle_stop') or {}).get('proximity_stop_enabled') ==
                   report['front_proximity_stop_enabled'] for x in execution)
        expected_format = ('mvp21_worker_rnn_maneuver_v1' if params.get('learned_policy',{}).get('enabled')
            else 'mvp21_worker_lidar_avoidance_v1')
        assert any(x.get('source_sha256_format')==expected_format for x in worker)
        report['actual_worker_and_guarded_native_control']=True
        if params['lidar_avoidance'].get('maneuver_enabled'):
            assert any((x.get('lidar_avoidance',{}).get('maneuver') or {}).get('phase')=='SHIFT' for x in execution)
            assert any((x.get('selected') or {}).get('family')=='maneuver_shift' for x in worker)
            report['dedicated_maneuver_mode']=True
            if params.get('learned_policy',{}).get('enabled'):
                tasks=[x for x in worker if (x.get('avoidance') or {}).get('maneuver_task_id') is not None]
                assert tasks and all('policy_selection' not in x for x in tasks)
                assert any(x.get('selection_backend')=='maneuver' for x in execution)
                report['task_planner_bypasses_rnn']=True
        run(.6,.35)
        assert states[-1]==2 and actions[-1]['drive']==actions[-1]['steer']==0.
        report['close_obstacle_stops']=True
        index=len(actions);run(.5)
        assert states[-1]==2 and all(a['drive']==a['steer']==0. for a in actions[index:])
        report['clear_scan_does_not_resume']=True
        pubs['request'].publish(UInt16(data=3));index=len(actions);run(1.5)
        assert states[-1]==3 and any(a['drive']>0 for a in actions[index:])
        report['explicit_resume']=True
        if params.get('learned_policy',{}).get('enabled'):
            index=len(actions);run(.5)
            # Stop synthetic sensor publication after explicit resume.
            # Watchdog supervision must still latch zero actions.
            deadline=time.monotonic()+.55
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
            assert states[-1]==2 and actions[-1]['drive']==actions[-1]['steer']==0.
            report['sensor_expiry_stops']=True
        pubs['request'].publish(UInt16(data=2));run(.2)
        report.update(passed=True,hardware_used=False,vehicle_enabled=False,
                      namespace=namespace,profile_enabled=params['lidar_avoidance']['enabled'],
                      road_width_m=args.road_width,obstacle_width_m=args.obstacle_width,
                      obstacle_offset_m=args.obstacle_offset,obstacle_distance_m=args.obstacle_distance,
                      roadside_returns=args.roadside_returns,scan_step_deg=args.scan_step_deg)
    finally:
        pubs['request'].publish(UInt16(data=2))
        process.terminate()
        try:process.wait(timeout=5.)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();node.destroy_node();rclpy.shutdown()
        (args.output/'RESULTS.json').write_text(json.dumps(report,indent=2)+'\n')
        (args.output/'states.json').write_text(json.dumps(states)+'\n')
        (args.output/'actions.json').write_text(json.dumps(actions)+'\n')
        (args.output/'execution.json').write_text(json.dumps(execution)+'\n')
        (args.output/'worker.json').write_text(json.dumps(worker)+'\n')
        intervals=[b-a for a,b in zip(scan_publish_times,scan_publish_times[1:])]
        (args.output/'sender-timing.json').write_text(json.dumps(dict(
            scan_period_requested_s=.1,scan_publish_times=scan_publish_times,
            maximum_scan_publication_interval_s=max(intervals,default=None),
            geometry='stationary zero-speed DDS fixture; ray intersections cached'))+'\n')
    print(json.dumps(report),flush=True)


if __name__=='__main__':
    main()
