"""One bounded, logged workshop run; uses the policy's existing limits."""
import json
import math
import argparse
import signal
import statistics
import hashlib
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import Log
from diagnostic_msgs.msg import DiagnosticArray
from dream_interfaces.msg import ConeDetections, DriveAndSteer
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int8, String, UInt8, UInt16

parser = argparse.ArgumentParser()
parser.add_argument('--dry', action='store_true')
parser.add_argument('--label', default='')
args = parser.parse_args()
mode = 'dry' if args.dry else 'drive'
root = Path.home() / 'mvp2-test-20261013' / (mode + ('-' + args.label if args.label else ''))
root.mkdir(parents=True, exist_ok=True)
log = (root / 'run.jsonl').open('w', buffering=1)
started = time.monotonic()
latest = {}
received = {}
max_speed = 0.0
max_drive = 0.0
max_distance = 0.0
subscriptions = []
last_logged = {}
result = {'result': 'not_started'}
latest_policy_text = [None]

def interrupted(signum, frame):
    raise KeyboardInterrupt('signal ' + str(signum))
signal.signal(signal.SIGTERM, interrupted)

def event(kind, value):
    log.write(json.dumps({'elapsed_s': round(time.monotonic()-started, 4), 'time_unix_s': time.time(), 'kind': kind, 'value': value}, ensure_ascii=False) + '\n')

def store(name, value):
    global max_speed, max_drive, max_distance
    now = time.monotonic()
    latest[name] = value
    received[name] = now
    if name == 'wheel': max_speed = max(max_speed, abs(value))
    if name == 'actions': max_drive = max(max_drive, abs(value['drive']))
    if name == 'policy_text' and value != latest_policy_text[0]:
        print(value, flush=True)
        latest_policy_text[0] = value
    if name == 'distance': max_distance = max(max_distance, value)
    if now-last_logged.get(name, -1e9) >= (0.1 if name in ('wheel','gyro','actions','applied','distance') else 0.0):
        event(name, value)
        last_logged[name] = now

def cone_callback(msg):
    counts = {}
    for d in msg.detections:
        if d.classification_confidence >= 0.5 and d.position.x > 0:
            counts[d.color] = counts.get(d.color, 0)+1
    store('cones', {'counts': counts, 'stamp_s': msg.header.stamp.sec+msg.header.stamp.nanosec/1e9, 'latency_s': msg.acquisition_to_publish_latency_s, 'positions': [[d.position.x,d.position.y,d.color] for d in msg.detections]})

def imu_callback(msg):
    if msg.angular_velocity_covariance[0] != -1 and all(math.isfinite(x) for x in (msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z)):
        store('gyro', {'z':msg.angular_velocity.z, 'stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9})

def diagnostics(msg):
    for s in msg.status:
        if s.name == '/car/traxxas_vehicle_interface':
            v = {x.key:x.value for x in s.values}
            store('vehicle_diagnostics', {k:v.get(k) for k in ('state','reason','connected','ready','connection_attempts','session_id','rc_drive_status','rc_steer_status','selected_source','host_seq','host_fresh','host_permitted','applied_drive_normalized','drive_pwm_us','rc_drive_us','last_positive_vehicle_ack','last_written_drive_normalized','last_written_steer_normalized','host_permission_reason')})

def bind(cls, topic, name, transform=lambda m:m.data, qos=10):
    subscriptions.append(node.create_subscription(cls, topic, lambda msg:store(name, transform(msg)), qos))

def spin_until(predicate, seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
        if predicate(): return True
    return False

def fresh(name, seconds):
    return name in received and time.monotonic()-received[name] < seconds

def candidate_unchanged():
    workspace = Path.home() / 'ai4r_student_workspace'
    expected = {workspace/'src/ai4r_policy/scripts/policy_node.py':'fbc27b7e9b7b556cca32c035516240aa649fb6711642c1603369c5bf4e14bbed',
        workspace/'install/ai4r_policy/lib/ai4r_policy/policy_node.py':'fbc27b7e9b7b556cca32c035516240aa649fb6711642c1603369c5bf4e14bbed',
        workspace/'src/ai4r_policy/config/ai4r_policy.yaml':'59147ecf791f1e74f8c85c4996ee6c63a3b44a1d2aecce4e27822822b4213f5d'}
    return all(hashlib.sha256(path.read_bytes()).hexdigest() == digest for path,digest in expected.items())

def healthy():
    v = latest.get('vehicle_diagnostics', {})
    cones = latest.get('cones', {})
    gyro = latest.get('gyro', {})
    ros_now = node.get_clock().now().nanoseconds/1e9
    return (fresh('vehicle_diagnostics',2) and v.get('connected')=='1' and v.get('ready')=='1'
            and v.get('rc_drive_status')=='1' and v.get('rc_steer_status')=='1'
            and fresh('cones',0.5) and 0 <= ros_now-cones.get('stamp_s',0) < 0.4
            and cones.get('counts',{}).get(1,0) >= 2 and cones.get('counts',{}).get(2,0) >= 2
            and statistics.median(v[1] for v in cones['positions'] if v[2] == 2) > statistics.median(v[1] for v in cones['positions'] if v[2] == 1)
            and fresh('wheel',0.5) and math.isfinite(latest['wheel'])
            and fresh('gyro',0.5) and 0 <= ros_now-gyro.get('stamp_s',0) < 0.5)

rclpy.init()
node = Node('mvp2_workshop_test')
be = QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
subscriptions.append(node.create_subscription(ConeDetections,'/car/cone_detections',cone_callback,10))
subscriptions.append(node.create_subscription(Imu,'/car/imu/data',imu_callback,be))
subscriptions.append(node.create_subscription(DiagnosticArray,'/car/diagnostics',diagnostics,10))
bind(Float32,'/car/wheel_speed_m_per_sec','wheel')
bind(Float32,'/car/debug2','distance')
bind(Float32,'/car/debug1','path_error')
bind(Int8,'/car/policy_fsm_state_value','policy_state')
bind(String,'/car/policy_fsm_state_string','policy_text')
bind(String,'/car/traxxas_state','vehicle_state')
for topic,name in (('/car/drive_and_steer_set_point_normalized','actions'),('/car/drive_and_steer_applied_normalized','applied')):
    bind(DriveAndSteer,topic,name,lambda m:{'units':m.units,'drive':m.drive,'steer':m.steer})
policy = node.create_publisher(UInt16,'/car/policy_fsm_transition_request',10)
vehicle = node.create_publisher(UInt8,'/car/request',10)
subscriptions.append(node.create_subscription(Log, '/rosout', lambda msg:event('policy_log', {'level':msg.level,'message':msg.msg}) if 'ai4r_policy' in msg.name else None, 100))
client = node.create_client(GetParameters,'/car/ai4r_policy/get_parameters')
try:
    if not candidate_unchanged(): raise RuntimeError('Candidate files changed; test refused')
    if not spin_until(lambda: policy.get_subscription_count()>0 and vehicle.get_subscription_count()>0,5):
        raise RuntimeError('Policy or vehicle command subscription not discovered')
    policy.publish(UInt16(data=2))
    vehicle.publish(UInt8(data=0))
    spin_until(lambda: latest.get('policy_state')==2 and latest.get('vehicle_state') in ('Disabled','Disarmed'),3)
    if not client.wait_for_service(timeout_sec=2): raise RuntimeError('Policy parameter service unavailable')
    names=['control.enabled','control.mode','control.max_distance_m','control.max_run_time_s','planning.cruise_speed_mps','control.drive_max','control.steering_max_normalized','control.mvp_drive_feedforward','control.mvp_steering_direction','control.speed_kp','control.speed_ki','planning.algorithm','planning.lattice.async_enabled','planning.lattice.direct_sample_output','control.robustness_enabled','planning.lattice.hard_margin_m','estimation.continuous_road_output','estimation.max_source_age_s','planning.lattice.recovery_enabled','planning.lattice.max_candidates','planning.lattice.centerline_first_enabled','control.startup_grace_s','control.recovery_confirm_frames','planning.lattice.budget_s','planning.lattice.update_rate_hz','policy_update_rate_hz']
    request=GetParameters.Request(names=names)
    future=client.call_async(request)
    if not spin_until(future.done,3): raise RuntimeError('Policy parameter query timeout')
    values=future.result().values
    params={name:({1:v.bool_value,2:v.integer_value,3:v.double_value,4:v.string_value}.get(v.type)) for name,v in zip(names,values)}
    event('parameters',params)
    if len(node.get_publishers_info_by_topic('/car/drive_and_steer_set_point_normalized')) != 1:
        raise RuntimeError('Expected exactly one policy action publisher')
    if not (params.get('planning.algorithm') == 'lattice_v2'
            and params.get('planning.lattice.async_enabled') is True
            and params.get('planning.lattice.recovery_enabled') is True
            and params.get('planning.lattice.centerline_first_enabled') is False
            and params.get('planning.lattice.max_candidates') == 16
            and params.get('control.startup_grace_s') == 3.0
            and params.get('control.recovery_confirm_frames') == 3
            and params.get('planning.lattice.budget_s') == 0.1
            and params.get('planning.lattice.update_rate_hz') == 10.0
            and params.get('policy_update_rate_hz') == 20.0
            and params.get('planning.lattice.direct_sample_output') is True
            and params.get('control.robustness_enabled') is True
            and params.get('planning.lattice.hard_margin_m') == 0.02
            and params.get('estimation.continuous_road_output') is True
            and params.get('estimation.max_source_age_s') == 1.5
            and params.get('control.enabled') is True and params.get('control.mode')=='mvp'
            and 0 < params.get('control.max_distance_m',999) <= 3.0
            and 0 < params.get('control.max_run_time_s',999) <= 30
            and 0 < params.get('planning.cruise_speed_mps',999) <= 0.2
            and 0 < params.get('control.drive_max',999) <= 0.35
            and 0 < params.get('control.steering_max_normalized',999) <= 0.5
            and params.get('control.mvp_drive_feedforward') == 0.30
            and params.get('control.mvp_steering_direction') == -1.0):
        raise RuntimeError('MVP bounds not confirmed')
    if not spin_until(lambda: healthy() and abs(latest.get('wheel',999)) < 0.005 and latest.get('policy_state')==2 and fresh('actions',0.3) and latest['actions']['drive']==0 and latest['actions']['steer']==0,8):
        raise RuntimeError('Preflight sensors/RC/connection/zero commands not healthy')
    spin_until(lambda: False, 1.5)
    if not healthy() or abs(latest.get('wheel',999)) >= 0.005:
        raise RuntimeError('Stationary priming/health lost')
    if not candidate_unchanged(): raise RuntimeError('Candidate changed during preflight')
    if args.dry:
        event('phase','Dry run; vehicle remains Disabled/Disarmed')
        if latest.get('vehicle_state') not in ('Disabled','Disarmed'):
            raise RuntimeError('Dry run requires disabled vehicle')
    else:
        event('phase','Enable vehicle')
        vehicle.publish(UInt8(data=1))
        if not spin_until(lambda: fresh('vehicle_state', 1) and latest.get('vehicle_state')=='Enabled',6):
            raise RuntimeError('Vehicle did not become Enabled')
    event('phase','Start policy once')
    initial_text=latest.get('policy_text')
    max_speed = max_drive = max_distance = 0.0
    launch_time=time.monotonic()
    print('Starting ' + mode + ' policy test now', flush=True)
    policy.publish(UInt16(data=3))
    seen_active=False
    stop_reason='Dry run completed 5 seconds' if args.dry else 'Test deadline reached'
    run_limit = 5 if args.dry else 32
    last_hash_check = launch_time
    while time.monotonic()-launch_time < run_limit:
        rclpy.spin_once(node,timeout_sec=0.03)
        if time.monotonic()-last_hash_check >= 0.25:
            last_hash_check = time.monotonic()
            if not candidate_unchanged():
                stop_reason='Candidate files changed during run'; break
        if latest.get('policy_state')==3: seen_active=True
        if max_speed > 0.5:
            stop_reason='Measured speed exceeded test bound 0.5 m/s'; break
        if not args.dry and time.monotonic()-launch_time > 4 and max_speed == 0:
            stop_reason='No wheel motion after 4 seconds'; break
        if (received.get('policy_text',0)>launch_time and latest.get('policy_text','').startswith('Publishing zero actions:')
                and received.get('policy_state',0)>launch_time and latest.get('policy_state')==2):
            if seen_active or (time.monotonic()-launch_time>0.25 and latest.get('policy_text')!=initial_text):
                stop_reason=latest['policy_text']; break
        if args.dry and (abs(latest.get('wheel',0)) > 0.005 or abs(latest.get('applied',{}).get('drive',0)) > 0):
            stop_reason='Unexpected motion/applied drive during disabled dry test'; break
        if not fresh('wheel',0.5) or not fresh('vehicle_diagnostics',2):
            stop_reason='Test monitor lost wheel/vehicle feedback'; break
        if time.monotonic()-launch_time>1 and latest.get('vehicle_state') not in (('Disabled','Disarmed') if args.dry else ('Enabled',)):
            stop_reason='Vehicle left Enabled: '+str(latest.get('vehicle_state')); break
        if time.monotonic()-launch_time>2 and not seen_active:
            stop_reason='Policy did not enter active state'; break
    result={'result':'completed' if ('Distance limit reached' in stop_reason or stop_reason == 'Dry run completed 5 seconds') else 'stopped', 'stop_reason':stop_reason,'run_seconds':time.monotonic()-launch_time,'seen_active':seen_active}
except BaseException as exc:
    result={'result':'aborted','reason':str(exc),'exception':type(exc).__name__}
    event('error',result)
finally:
    event('phase','Stop policy and Disable vehicle')
    for _ in range(3):
        policy.publish(UInt16(data=2))
        vehicle.publish(UInt8(data=0))
        spin_until(lambda:False,0.2)
    confirmed_stop = spin_until(lambda:latest.get('policy_state')==2 and fresh('policy_state',1) and latest.get('vehicle_state') in ('Disabled','Disarmed') and fresh('vehicle_state',1) and fresh('actions',0.3) and latest['actions']['drive']==0 and latest['actions']['steer']==0 and fresh('wheel',0.3) and abs(latest['wheel']) < 0.005,5)
    result['confirmed_stop'] = confirmed_stop
    result.update(mode=mode, candidate='ae74be4+road-v2@6552033+lattice-only+margin-0.02+startup-retry+planning-budget100+cruise-floor0.2', final_actions=latest.get('actions'),final_applied=latest.get('applied'),final_wheel_speed_mps=latest.get('wheel'),max_wheel_speed_mps=max_speed,max_requested_drive=max_drive,max_distance_debug_m=max_distance,final_policy_state=latest.get('policy_state'),final_vehicle_state=latest.get('vehicle_state'),final_policy_text=latest.get('policy_text'),vehicle_diagnostics=latest.get('vehicle_diagnostics'),latest_cones=latest.get('cones'))
    event('result',result)
    (root/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False),flush=True)
    node.destroy_node()
    rclpy.shutdown()
    log.close()
    if result.get('result') != 'completed' or not result.get('confirmed_stop'):
        raise SystemExit(1)
