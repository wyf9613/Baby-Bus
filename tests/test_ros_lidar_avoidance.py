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
