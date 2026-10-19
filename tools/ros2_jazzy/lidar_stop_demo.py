"""Isolated DDS/timer acceptance with synthetic sensors and current MVP2.1.

No vehicle dynamics or physical braking is represented here. Drive values are
normalized software commands; synthetic wheel telemetry is held at zero.
"""
import argparse
import hashlib
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, UInt16
from tf2_ros import StaticTransformBroadcaster
from dream_interfaces.msg import ConeDetection, ConeDetections, DriveAndSteer

sys.path.insert(0,str(Path(get_package_prefix('ai4r_policy'))/'lib/ai4r_policy'))
from mvp21_policy.node import PolicyNode
from mvp21_policy.provenance import source_fingerprint


class Scenario:
    def __init__(self,name):
        self.name=name;self.origin=time.monotonic();self.records=[];self.actions=[]
        self.gap=None;self.y=0.;self.frame='synthetic_laser';self.send_scan=True
        self.next_send=0.;self.planner_seen=False;self.control_timer_cancelled=False
        self.context=Context();rclpy.init(context=self.context)
        namespace='lidar_stop_'+name
        config=Path(get_package_share_directory('ai4r_policy'))/'config/ai4r_policy.yaml'
        self.policy=PolicyNode(context=self.context,namespace=namespace,
            cli_args=['--ros-args','--params-file',str(config)],
            parameter_overrides=[Parameter('obstacle_stop.enabled',value=True),
                Parameter('obstacle_stop.front_extent_m',value=.297),
                Parameter('obstacle_stop.rear_extent_m',value=.198),
                Parameter('obstacle_stop.width_m',value=.25)])
        self.peer=Node('synthetic_sensors',context=self.context,namespace=namespace)
        self.executor=SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.policy);self.executor.add_node(self.peer)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        self.scan_pub=self.peer.create_publisher(LaserScan,'scan',qos)
        self.wheel_pub=self.peer.create_publisher(Float32,'wheel_speed_m_per_sec',qos)
        self.imu_pub=self.peer.create_publisher(Imu,'imu/data',qos)
        self.cone_pub=self.peer.create_publisher(ConeDetections,'cone_detections',qos)
        self.request_pub=self.peer.create_publisher(UInt16,'policy_fsm_transition_request',10)
        self.peer.create_subscription(DriveAndSteer,'drive_and_steer_set_point_normalized',self.action,10)
        self.tf=StaticTransformBroadcaster(self.peer)
        transform=TransformStamped()
        transform.header.frame_id='base_link';transform.child_frame_id='synthetic_laser'
        transform.header.stamp=self.peer.get_clock().now().to_msg()
        transform.transform.translation.x=.05;transform.transform.rotation.w=1.
        self.tf.sendTransform(transform)

    def action(self,msg):
        mode=self.policy.planner_execution.diagnostics.get('mode')
        self.planner_seen |= mode=='planner'
        record=dict(t_s=time.monotonic()-self.origin,kind='action',drive=float(msg.drive),
            steer=float(msg.steer),fsm=self.policy.fsm_state,reason=self.policy.state_reason,
            execution_mode=mode,monitor=deepcopy(self.policy.obstacle_stop_diagnostics))
        self.records.append(record);self.actions.append(record)

    def feed(self):
        stamp=self.peer.get_clock().now().to_msg()
        self.wheel_pub.publish(Float32(data=0.))
        gyro=Imu();gyro.header.stamp=stamp;gyro.header.frame_id='base_link'
        gyro.orientation_covariance[0]=-1.;gyro.linear_acceleration_covariance[0]=-1.
        gyro.angular_velocity_covariance[0]=0.
        self.imu_pub.publish(gyro)
        cones=ConeDetections();cones.header.stamp=stamp;cones.header.frame_id='base_link'
        for colour,y in ((self.policy.estimation_settings.left_colour,.5),
                          (self.policy.estimation_settings.right_colour,-.5)):
            for index in range(10):
                cone=ConeDetection();cone.position.x=-.3+.3*index;cone.position.y=y
                cone.color=colour;cone.classification_confidence=.95;cones.detections.append(cone)
        self.cone_pub.publish(cones)
        if self.send_scan:
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id=self.frame
            x=.297+(self.gap if self.gap is not None else 1.)-.05
            scan.angle_min=math.atan2(self.y,x);scan.angle_increment=.1
            scan.angle_max=scan.angle_min+scan.angle_increment
            scan.range_min=.15;scan.range_max=16.;scan.scan_time=.05
            scan.ranges=[math.hypot(x,self.y) if self.gap is not None else math.inf,math.inf]
            self.scan_pub.publish(scan)
        self.records.append(dict(t_s=time.monotonic()-self.origin,kind='synthetic_input',
            front_gap_m=self.gap,lateral_m=self.y,scan_published=self.send_scan,
            scan_frame=self.frame,wheel_speed_mps=0.))

    def spin(self,condition=None,timeout=3.):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            now=time.monotonic()
            if now>=self.next_send:
                self.feed();self.next_send=now+.05
            self.executor.spin_once(timeout_sec=.005)
            if condition is not None and condition():return
        if condition is not None:
            raise AssertionError(self.name+': deadline; '+self.policy.state_reason)

    def resume(self):
        self.request_pub.publish(UInt16(data=3))
        self.records.append(dict(t_s=time.monotonic()-self.origin,kind='explicit_start_request'))

    def stopped(self,reason):
        self.spin(lambda:self.policy.fsm_state==2 and reason in self.policy.state_reason
                  and bool(self.actions) and self.actions[-1]['drive']==0.)

    def run(self):
        self.spin(lambda:self.policy.observations['lidar_cartesian'] is not None
                  and self.request_pub.get_subscription_count()>0)
        if self.name=='startup_obstacle':self.gap=.14;self.spin(timeout=.15)
        start=len(self.actions);self.resume()
        if self.name=='startup_obstacle':
            self.stopped('lidar_obstacle')
            assert not any(r['drive']>0. for r in self.actions[start:])
        else:
            self.spin(lambda:any(r['drive']>0. for r in self.actions[start:]))
            if self.name=='central':
                self.spin(lambda:self.planner_seen,timeout=5.)
                self.gap=.16;self.spin(timeout=.20)
                assert self.policy.fsm_state==3
                self.gap=.14;self.stopped('lidar_obstacle')
            elif self.name=='outside':
                self.gap=.14;self.y=.30;self.spin(timeout=.3)
                assert self.policy.fsm_state==3 and self.actions[-1]['drive']>0.
            elif self.name in ('dropout','watchdog'):
                if self.name=='watchdog':
                    self.policy.policy_timer.cancel();self.control_timer_cancelled=True
                self.send_scan=False;self.stopped('lidar_stale_or_bad_timestamp')
            elif self.name=='missing_tf':
                self.frame='missing_transform';self.stopped('lidar_unavailable')
            elif self.name=='empty':
                self.spin(timeout=.3)
                assert self.policy.fsm_state==3 and self.actions[-1]['drive']>0.
        trigger=deepcopy(self.policy.obstacle_stop_monitor.trigger)
        if self.name not in ('empty','outside'):
            self.gap=None;self.y=0.;self.frame='synthetic_laser';self.send_scan=True
            mark=len(self.actions);self.spin(timeout=.25)
            assert self.policy.fsm_state==2
            assert all(r['drive']==0. and r['steer']==0. for r in self.actions[mark:])
            assert self.policy.obstacle_stop_monitor.trigger==trigger
            if self.name=='central':
                mark=len(self.actions);self.resume()
                self.spin(lambda:any(r['drive']>0. for r in self.actions[mark:]))
                assert not self.policy.obstacle_stop_monitor.latched
        near_input=next((r['t_s'] for r in self.records if r['kind']=='synthetic_input'
            and r['front_gap_m']==.14 and r['lateral_m']==0.),None)
        zero_event=next((r['t_s'] for r in self.actions if trigger is not None
            and trigger['reason'] in r['reason'] and r['drive']==0. and r['fsm']==2),None)
        return dict(passed=True,trigger=trigger,planner_seen=self.planner_seen,
            control_timer_cancelled=self.control_timer_cancelled,
            retained_zero_without_auto_resume=self.name not in ('empty','outside'),
            explicit_resume_verified=self.name=='central',commands=len(self.actions),
            target_speed_mps=self.policy.planning_settings.cruise_speed_mps,
            stop_settings=vars(self.policy.obstacle_stop_monitor.settings),
            obstacle_input_to_zero_s=(zero_event-near_input if near_input is not None and zero_event is not None else None),
            physical_stop_verified=False)

    def close(self):
        self.executor.remove_node(self.peer);self.executor.remove_node(self.policy)
        self.policy.destroy_node();self.peer.destroy_node()
        self.executor.shutdown();self.context.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Use a new evidence directory')
    if os.environ.get('ROS_DOMAIN_ID')!='218' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        parser.error('This demo requires isolated domain 218 and LOCALHOST discovery')
    source=source_fingerprint()
    root=Path(__file__).resolve().parents[2]
    expected={p.name:hashlib.sha256(p.read_bytes().replace(b'\r\n',b'\n')).hexdigest()
              for p in (root/'mvp21_policy').glob('*.py')}
    if source['files']!=expected:
        parser.error('Installed modules differ from this checkout; run the ROS check/build first')
    args.output.mkdir(parents=True)
    summaries={}
    for name in ('empty','central','outside','dropout','missing_tf','watchdog','startup_obstacle'):
        scene=Scenario(name)
        try:
            summaries[name]=scene.run()
        finally:
            (args.output/(name+'.jsonl')).write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in scene.records))
            scene.close()
        print(name+': PASS',flush=True)
    config=Path(get_package_share_directory('ai4r_policy'))/'config/ai4r_policy.yaml'
    unchanged=source==source_fingerprint()
    result=dict(all_passed=unchanged and all(r['passed'] for r in summaries.values()),scenarios=summaries,
        source=source,source_unchanged_during_run=unchanged,
        common_config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),
        physical_vehicle_used=False,synthetic_sensors=True,normalized_zero_command_only=True,
        scope='real ROS messages, TF, current MVP2.1 estimator/planner/control and FSM; no vehicle plant')
    (args.output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return 0 if result['all_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
