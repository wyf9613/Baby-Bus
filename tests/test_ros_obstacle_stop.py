"""Real ROS callbacks and DDS acceptance for the opt-in straight stop gate."""
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from dream_interfaces.msg import ConeDetection, ConeDetections
from test_policy_node import make_node, stamp, scan, imu, request


def stop_node(make_node, **extra):
    settings = {'control.enabled':True, 'control.mode':'mvp',
        'planning.algorithm':'road_centerline', 'obstacle_stop.enabled':True,
        'policy_update_rate_hz':20., 'planning.allow_forward_only':True}
    settings.update(extra)
    return make_node(required=('cone_detections','wheel_speed','imu_angular_velocity'), **settings)


def feed(node, gap=None, y=0., frame='base_link', empty_road=False):
    node.wheel_speed_callback(Float32(data=0.))
    gyro = imu(node, orientation=False)
    gyro.angular_velocity_covariance[0] = 0.
    node.imu_callback(gyro)
    batch = stamp(node, ConeDetections())
    if not empty_road:
        for color, side in ((ConeDetection.COLOR_BLUE,.5),(ConeDetection.COLOR_YELLOW,-.5)):
            for index in range(10):
                cone = ConeDetection()
                cone.position.x, cone.position.y = -.3+.3*index, side
                cone.color, cone.classification_confidence = color, .95
                batch.detections.append(cone)
    node.cone_detection_callback(batch)
    message = scan(node, frame=frame)
    x = .297+(gap if gap is not None else 1.)
    message.angle_min = math.atan2(y,x)
    message.angle_max = message.angle_min+.1
    message.ranges = [math.hypot(x,y) if gap is not None else math.inf, math.inf]
    node.lidar_callback(message)


def start(node, gap=None, **kwargs):
    feed(node, gap, **kwargs)
    request(node, 3)
    # A post-start frame supplies motion history without inventing odometry.
    node.test_clock.advance(.05)
    feed(node, gap, **kwargs)
    node.run_policy_step()


@pytest.mark.parametrize('gap,y,stops', [(None,0.,False),(.16,0.,False),(.14,0.,True),
                                        (.14,.3,False),(.02,0.,True)])
def test_current_ros_estimation_to_front_bumper_gate(make_node, gap, y, stops):
    node = stop_node(make_node)
    start(node, gap, y=y)
    assert node.fsm_state == (2 if stops else 3), node.state_reason
    action = node.action_publisher.messages[-1]
    assert (action.drive==0.) if stops else (action.drive>0.)
    assert action.steer==0. if stops else True
    if stops:
        assert node.obstacle_stop_monitor.trigger['reason']=='lidar_obstacle'
        assert node.obstacle_stop_monitor.trigger['front_gap_m']<=.15


def test_stop_survives_clear_disable_and_requires_explicit_fresh_resume(make_node):
    node = stop_node(make_node)
    start(node, .14)
    first = dict(node.obstacle_stop_monitor.trigger)
    node.test_clock.advance(.05);feed(node)
    node.supervision_callback();node.run_policy_step()
    assert node.fsm_state==2 and node.obstacle_stop_monitor.trigger==first
    request(node,1);request(node,2)
    assert node.obstacle_stop_monitor.trigger==first
    request(node,3)
    node.test_clock.advance(.05);feed(node);node.run_policy_step()
    assert node.fsm_state==3 and not node.obstacle_stop_monitor.latched
    assert node.action_publisher.messages[-1].drive>0.


def test_stale_resume_is_refused_without_clearing_first_trigger(make_node):
    node=stop_node(make_node);start(node,.14)
    first=dict(node.obstacle_stop_monitor.trigger)
    node.test_clock.advance(.16);request(node,3)
    assert node.fsm_state==2 and 'start refused' in node.state_reason
    assert node.obstacle_stop_monitor.trigger==first


def test_late_calculation_cannot_publish_a_fresh_driving_command(make_node,monkeypatch):
    node=stop_node(make_node);start(node)
    original=node.calculate_policy_actions
    def late(*args):
        result=original(*args)
        node.test_clock.advance(.16)
        return result
    monkeypatch.setattr(node,'calculate_policy_actions',late)
    mark=len(node.action_publisher.messages)
    node.test_clock.advance(.05);feed(node);node.run_policy_step()
    assert node.fsm_state==2 and 'lidar_stale' in node.state_reason
    assert all(m.drive==0. for m in node.action_publisher.messages[mark:])


def test_missing_scan_or_tf_refuses_start(make_node):
    node = stop_node(make_node)
    feed(node, frame='missing_transform')
    request(node,3)
    assert node.fsm_state==2 and 'lidar_unavailable' in node.state_reason


def test_source_watchdog_works_without_policy_or_sensor_callbacks(make_node):
    node = stop_node(make_node)
    start(node)
    node.test_clock.advance(.151)
    node.supervision_callback()
    assert node.fsm_state==2
    assert 'lidar_stale_or_bad_timestamp' in node.state_reason
    assert node.action_publisher.messages[-1].drive==0.
    assert node.obstacle_stop_monitor.latched


def test_lost_tf_invalidates_previous_points_and_stops(make_node):
    node = stop_node(make_node)
    start(node)
    node.test_clock.advance(.05);feed(node, frame='missing_transform')
    node.supervision_callback()
    assert node.fsm_state==2 and 'lidar_unavailable' in node.state_reason
    assert node.action_publisher.messages[-1].drive==0.


def test_actual_mount_transform_is_used_before_gate(make_node):
    node = stop_node(make_node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link','laser'
    transform.transform.translation.x = .1
    transform.transform.rotation.w = 1.
    node.tf_buffer.set_transform_static(transform,'synthetic_test')
    start(node,.20,frame='laser')  # raw range .497 + mount .1 is safely outside.
    assert node.fsm_state==3
    node.test_clock.advance(.05);feed(node,.04,frame='laser')
    node.run_policy_step()
    assert node.fsm_state==2
    assert node.obstacle_stop_monitor.trigger['front_gap_m']==pytest.approx(.14,abs=1e-6)


def test_startup_alignment_priming_cannot_publish_positive_drive(make_node):
    node = stop_node(make_node)
    feed(node,.14)
    node.test_clock.advance(.01)
    node.wheel_speed_callback(Float32(data=0.))
    gyro=imu(node,orientation=False);gyro.angular_velocity_covariance[0]=0.
    node.imu_callback(gyro)
    request(node,3)
    # The scan predates the latest cached telemetry by 10 ms.
    node.test_clock.advance(.01)
    node.run_policy_step()
    assert node.action_publisher.messages[-1].drive==0.
    assert node.obstacle_stop_diagnostics['reason']=='lidar_alignment_priming'
    node.test_clock.advance(.03);feed(node,.14);node.run_policy_step()
    assert node.fsm_state==2 and node.obstacle_stop_monitor.latched


@pytest.mark.parametrize('branch', ['planner', 'terminal'])
def test_stop_precedes_native_selection_baseline_and_terminal_command(make_node,monkeypatch,branch):
    node = stop_node(make_node)
    start(node)
    if branch=='terminal':
        node._end_straight_start_distance_m=0.
        node._end_straight_started_at_s=node.test_clock.monotonic
        # Isolate the terminal branch's action, not its distance budget.
        monkeypatch.setattr(node,'_control_problem',lambda *_:None)
    else:
        from types import SimpleNamespace
        node.planner_control_settings=SimpleNamespace(enabled=True)
        monkeypatch.setattr(node,'_select_planner_reference',lambda *_:pytest.fail('Gate must precede planning'))
        monkeypatch.setattr(node.planner,'plan',lambda *_args,**_kw:pytest.fail('Gate must precede planning'))
    node.test_clock.advance(.05);feed(node,.14);node.run_policy_step()
    assert node.fsm_state==2 and node.action_publisher.messages[-1].drive==0.
    assert node.obstacle_stop_monitor.trigger['reason']=='lidar_obstacle'


def test_default_feature_is_off_and_threshold_is_retained(make_node):
    node = make_node()
    assert not node.obstacle_stop_monitor.settings.enabled
    assert node.obstacle_stop_monitor.settings.trigger_distance_m==.15


def test_real_dds_timer_and_common_mvp21_policy(tmp_path):
    script=Path(__file__).resolve().parents[1]/'tools/ros2_jazzy/lidar_stop_demo.py'
    subprocess.run([sys.executable,str(script),'--output',str(tmp_path/'dds')],check=True,timeout=80)
    summary=json.loads((tmp_path/'dds/summary.json').read_text())
    assert summary['all_passed']
    assert summary['scenarios']['central']['planner_seen']
    assert summary['scenarios']['watchdog']['control_timer_cancelled']
    assert summary['physical_vehicle_used'] is False
