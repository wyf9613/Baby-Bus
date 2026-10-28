"""Actual PolicyNode/ROS types, worker subprocess, fail-safe state boundaries."""
from copy import deepcopy
import time
from types import SimpleNamespace

import pytest
from test_policy_node import make_node, request
from test_ros_obstacle_stop import stop_node, feed, start


def avoidance_node(make_node, **extra):
    settings={'planner_control.enabled':True,'shadow.enabled':True,
        'shadow.rate_hz':5.,'shadow.length_m':2.5,'shadow.fusion_enabled':False,
        'shadow.collision_coverage':'generated','shadow.score_coverage':'generated',
        'lidar_avoidance.enabled':True}
    settings.update(extra)
    return stop_node(make_node,**settings)


def wait_result(node):
    deadline=time.monotonic()+8.
    while time.monotonic()<deadline:
        node._shadow_poll()
        if node.planner_execution.last_result_status is not None or node.fsm_state!=3:
            return
        time.sleep(.01)
    raise AssertionError('No actual worker result before bounded test timeout')


def test_actual_ros_node_executes_avoidance_worker_reference(make_node):
    node=avoidance_node(make_node)
    start(node,1.5)
    wait_result(node)
    assert node.fsm_state==3,node.state_reason
    assert node.planner_execution.last_result_status=='VALID'
    assert node.planner_execution.active is not None
    node.test_clock.advance(.05);feed(node,1.5);node.run_policy_step()
    assert node.fsm_state==3,node.state_reason
    assert node.planner_execution.diagnostics['mode']=='planner'
    assert node.action_publisher.messages[-1].drive>0.
    assert abs(node.action_publisher.messages[-1].steer)>.01
    assert node.lidar_avoidance_guard.side in (-1,1)


def fake_worker(node,result):
    node.shadow_worker.close()
    node.shadow_worker=SimpleNamespace(poll=lambda:deepcopy(result),close=lambda:None,submit=lambda _:None)


def test_no_path_latches_zero_and_only_explicit_admitted_resume_clears(make_node):
    node=avoidance_node(make_node);start(node)
    result=dict(status='NO_AVOIDANCE_PATH',generation=node.shadow_generation,job_id=100,
        input_stamp_s=node.get_clock().now().nanoseconds/1e9,
        avoidance=dict(enabled=True,blocking_obstacle=True))
    fake_worker(node,result);node._shadow_poll()
    assert node.fsm_state==2 and node.lidar_avoidance_guard.latched
    node._publish_actions(.3,.5,None)
    assert node.action_publisher.messages[-1].drive==0.
    assert node.action_publisher.messages[-1].steer==0.
    node.test_clock.advance(.05);feed(node);node.supervision_callback()
    assert node.fsm_state==2 and node.lidar_avoidance_guard.latched
    request(node,1);request(node,2)
    assert node.lidar_avoidance_guard.latched
    request(node,3)
    assert node.fsm_state==3 and not node.lidar_avoidance_guard.latched


def test_old_run_cannot_latch_new_run_and_missing_guard_result_stops(make_node):
    node=avoidance_node(make_node);start(node)
    result=dict(status='VALID',generation=node.shadow_generation-1,job_id=99,input_stamp_s=100.05)
    fake_worker(node,result);node._shadow_poll()
    assert node.fsm_state==3 and not node.lidar_avoidance_guard.latched
    result['generation']=node.shadow_generation
    fake_worker(node,result);node._shadow_poll()
    assert node.fsm_state==2 and node.lidar_avoidance_guard.stop_reason=='avoidance_worker_result_invalid'


def test_worker_transport_fault_stops_without_automatic_resume(make_node):
    node=avoidance_node(make_node);start(node)
    fake_worker(node,dict(status='TRANSPORT_ERROR',error='synthetic worker loss'))
    node._shadow_poll()
    assert node.fsm_state==2 and node.lidar_avoidance_guard.latched
    assert node.action_publisher.messages[-1].drive==0.


@pytest.mark.parametrize('extra',[
    {'obstacle_stop.enabled':False}, {'planner_control.enabled':False},
    {'lidar_avoidance.width_m':.9}, {'control.end_straight_distance_m':.5},
    {'control.robustness_enabled':True}])
def test_incompatible_enable_is_refused(make_node,extra):
    with pytest.raises(ValueError):
        avoidance_node(make_node,**extra)


def feed_maneuver(node,gap=2.5,*,rear_point=False,width=1.8,obstacle_y=0.):
    from dream_interfaces.msg import ConeDetection,ConeDetections
    import math
    from test_policy_node import stamp,imu,scan
    from std_msgs.msg import Float32
    node.wheel_speed_callback(Float32(data=0.))
    gyro=imu(node,orientation=False);gyro.angular_velocity_covariance[0]=0.;node.imu_callback(gyro)
    message=scan(node,frame='base_link');message.angle_min=0.;message.angle_max=.1
    front=node.obstacle_stop_monitor.settings.front_extent_m
    message.ranges=[front+gap,math.inf]
    if obstacle_y:
        message.angle_min=math.atan2(obstacle_y,front+gap)
        message.angle_max=message.angle_min+.1
        message.ranges=[math.hypot(obstacle_y,front+gap),math.inf]
    if rear_point:
        message.angle_min=math.atan2(.1946268196016643,-.22286008138405408)
        message.angle_max=message.angle_min+.1
        message.ranges=[math.hypot(.1946268196016643,.22286008138405408),math.inf]
    node.lidar_callback(message)
    batch=stamp(node,ConeDetections())
    for colour,side in ((ConeDetection.COLOR_BLUE,width/2),(ConeDetection.COLOR_YELLOW,-width/2)):
        for index in range(19):
            cone=ConeDetection();cone.position.x=-.4+.3*index;cone.position.y=side
            cone.color=colour;cone.classification_confidence=.95;batch.detections.append(cone)
    node.cone_detection_callback(batch)


def test_normal_following_survives_held_plan_and_rear_return_then_front_stop_still_latches(make_node):
    node=avoidance_node(make_node,**{'lidar_avoidance.maneuver_enabled':True,
        'shadow.rate_hz':1.,'shadow.length_m':1.,
        'shadow.collision_coverage':'prefix','shadow.score_coverage':'prefix'})
    feed_maneuver(node,10.,rear_point=True);request(node,3)
    node.test_clock.advance(.05);feed_maneuver(node,10.,rear_point=True);node.run_policy_step()
    wait_result(node)
    assert node.fsm_state==3,node.state_reason
    assert node.planner_execution.last_result_status=='VALID'
    fake_worker(node,None)  # Hold a real selected path; state/road/LiDAR stay fresh.
    for _ in range(70):
        node.test_clock.advance(.05);feed_maneuver(node,10.,rear_point=True);node.run_policy_step()
        assert node.fsm_state==3,node.state_reason
        assert node.action_publisher.messages[-1].drive>0.
        assert not node.lidar_avoidance_guard.maneuver.active
    assert node.planner_execution.diagnostics['source_age_s']>3.
    node.test_clock.advance(.05);feed_maneuver(node,.14);node.run_policy_step()
    assert node.fsm_state==2 and node.obstacle_stop_monitor.latched
    assert node.action_publisher.messages[-1].drive==0.
    node.test_clock.advance(.05);feed_maneuver(node,10.);node.run_policy_step()
    assert node.fsm_state==2 and node.action_publisher.messages[-1].drive==0.


def test_maneuver_ros_node_uses_task_reference_and_stops_on_new_near_obstacle(make_node):
    node=avoidance_node(make_node,**{'lidar_avoidance.maneuver_enabled':True,
        'lidar_avoidance.max_curvature_1pm':1.,'shadow.rate_hz':2.})
    feed_maneuver(node);request(node,3)
    node.test_clock.advance(.05);feed_maneuver(node);node.run_policy_step()
    wait_result(node)
    assert node.fsm_state==3,node.state_reason
    assert node.planner_execution.active is not None
    assert node.lidar_avoidance_guard.maneuver.task['id']==1
    node.test_clock.advance(.05);feed_maneuver(node);node.run_policy_step()
    assert node.fsm_state==3,node.state_reason
    assert node._trajectory_reference['maneuver_task_id']==1
    assert node.action_publisher.messages[-1].drive>0
    node.test_clock.advance(.05);feed_maneuver(node,.14);node.run_policy_step()
    assert node.fsm_state==2 and node.action_publisher.messages[-1].drive==0


def test_narrow_road_right_obstacle_is_admitted_to_actual_worker(make_node):
    node=avoidance_node(make_node,**{
        'lidar_avoidance.maneuver_enabled':True,'lidar_avoidance.max_curvature_1pm':1.,
        'lidar_avoidance.obstacle_margin_m':.03,'lidar_avoidance.tracking_margin_m':.02,
        'lidar_avoidance.target_lateral_allowance_m':.02,'lidar_avoidance.maneuver_clearance_m':.01,
        'lidar_avoidance.road_edge_exclusion_m':.10,
        'lidar_avoidance.front_extent_m':.275,'lidar_avoidance.rear_extent_m':.198,
        'lidar_avoidance.width_m':.30,'obstacle_stop.front_extent_m':.275,
        'obstacle_stop.rear_extent_m':.198,'obstacle_stop.width_m':.30,
        'shadow.rate_hz':1.,'shadow.length_m':1.,'shadow.collision_coverage':'prefix',
        'shadow.score_coverage':'prefix','shadow.body_front_m':.275,
        'shadow.body_rear_m':.198,'shadow.body_width_m':.30})
    feed_maneuver(node,width=.8,obstacle_y=-.13);request(node,3)
    node.test_clock.advance(.05);feed_maneuver(node,width=.8,obstacle_y=-.13);node.run_policy_step()
    wait_result(node)
    assert node.fsm_state==3,node.state_reason
    assert node.planner_execution.active is not None
    task=node.lidar_avoidance_guard.maneuver.task
    assert task is not None and 0.<task['goal_d']<.2
    assert node.lidar_avoidance_guard.maneuver.admission['available_channel_m']>0.
    node.test_clock.advance(.05);feed_maneuver(node,width=.8,obstacle_y=-.13);node.run_policy_step()
    assert node._trajectory_reference['maneuver_task_id']==task['id']
    assert node.action_publisher.messages[-1].drive>0.
    node.test_clock.advance(.05);feed_maneuver(node,.14,width=.8);node.run_policy_step()
    assert node.fsm_state==2 and node.obstacle_stop_monitor.latched
    assert node.action_publisher.messages[-1].drive==0.
