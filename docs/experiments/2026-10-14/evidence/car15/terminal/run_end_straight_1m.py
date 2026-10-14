"""Logged workshop run of the learned-width profile with live trajectory verification; no distance/time budget."""
import json
import math
import statistics
import signal
import sys
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from diagnostic_msgs.msg import DiagnosticArray
from dream_interfaces.msg import ConeDetections, DriveAndSteer
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path as TrajectoryPath
from std_msgs.msg import Float32, Int8, String, UInt8, UInt16

if '--run' not in sys.argv:
    raise SystemExit('Pass --run for the authorized continuous trial; use probe.py for read-only observation')
root = Path.home() / 'car15-test-20261014/evidence/end-straight2-1m'
root.mkdir(parents=True, exist_ok=False)
log = (root / 'run.jsonl').open('w', buffering=1)
started = time.monotonic()
latest = {}
received = {}
max_speed = 0.0
max_drive = 0.0
max_distance = 0.0
nonempty_path_messages = 0
max_path_points = 0
subscriptions = []
last_logged = {}
result = {'result': 'not_started'}

def event(kind, value):
    log.write(json.dumps({'elapsed_s': round(time.monotonic()-started, 4), 'time_unix_s': time.time(), 'kind': kind, 'value': value}, ensure_ascii=False) + '\n')

def store(name, value):
    global max_speed, max_drive, max_distance, nonempty_path_messages, max_path_points
    now = time.monotonic()
    latest[name] = value
    received[name] = now
    if name == 'wheel': max_speed = max(max_speed, abs(value))
    if name == 'actions': max_drive = max(max_drive, abs(value['drive']))
    if name == 'distance': max_distance = max(max_distance, value)
    if name == 'trajectory':
        nonempty_path_messages += int(value['point_count'] > 0)
        max_path_points = max(max_path_points, value['point_count'])
    if name == 'path_error':
        event('control_sample', {'path_error_m':value, 'actions':latest.get('actions'), 'applied':latest.get('applied'), 'wheel_mps':latest.get('wheel'), 'gyro_raw':latest.get('gyro'), 'distance_m':latest.get('distance'), 'policy_state':latest.get('policy_state')})
    if now-last_logged.get(name, -1e9) >= (0.1 if name in ('wheel','gyro','actions','applied','distance') else 0.0):
        event(name, value)
        last_logged[name] = now

def cone_callback(msg):
    near = {c: [d.position.y for d in msg.detections if d.color == c and 0 < d.position.x < 2.5 and d.classification_confidence >= 0.5] for c in (1, 2)}
    store('colour_layout', all(len(near[c]) >= 2 for c in near) and statistics.median(near[1]) > statistics.median(near[2]))
    counts = {}
    for d in msg.detections:
        if d.classification_confidence >= 0.5 and d.position.x > 0:
            counts[d.color] = counts.get(d.color, 0)+1
    store('cones', {'counts': counts, 'stamp_s': msg.header.stamp.sec+msg.header.stamp.nanosec/1e9, 'latency_s': msg.acquisition_to_publish_latency_s, 'positions': [[d.position.x,d.position.y,d.position.z,d.color,d.classification_confidence] for d in msg.detections]})

def imu_callback(msg):
    if msg.angular_velocity_covariance[0] != -1 and all(math.isfinite(x) for x in (msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z)):
        store('gyro', {'z':msg.angular_velocity.z, 'stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9})

def diagnostics(msg):
    for s in msg.status:
        if s.name == '/car/bno08x_imu_interface':
            store('imu_diagnostics', {'message':s.message, **{x.key:x.value for x in s.values if x.key in ('reason','firmware.lifecycle_state','host_publication_ready')}})
        if s.name == '/car/traxxas_vehicle_interface':
            v = {x.key:x.value for x in s.values}
            store('vehicle_diagnostics', {k:v.get(k) for k in ('state','reason','connected','ready','connection_attempts','session_id','rc_drive_status','rc_steer_status','selected_source','host_seq','host_fresh','host_permitted','applied_drive_normalized','applied_steer_normalized','steering_trim','rc_steer_us','drive_pwm_us','rc_drive_us','last_positive_vehicle_ack','last_written_vehicle_command')})

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

def healthy():
    v = latest.get('vehicle_diagnostics', {})
    cones = latest.get('cones', {})
    gyro = latest.get('gyro', {})
    ros_now = node.get_clock().now().nanoseconds/1e9
    return (fresh('vehicle_diagnostics',2) and v.get('connected')=='1' and v.get('ready')=='1'
            and v.get('rc_drive_status')=='1' and v.get('rc_steer_status')=='1'
            and fresh('colour_layout',0.5) and latest['colour_layout']
            and fresh('cones',0.5) and 0 <= ros_now-cones.get('stamp_s',0) < 0.4
            and sum(c >= 2 for c in cones.get('counts',{}).values()) >= 2
            and fresh('wheel',0.5) and math.isfinite(latest['wheel'])
            and fresh('gyro',0.5) and 0 <= ros_now-gyro.get('stamp_s',0) < 0.5)

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
def interrupted(signum, frame):
    raise KeyboardInterrupt(f"Signal {signum}")
signal.signal(signal.SIGINT, interrupted)
signal.signal(signal.SIGTERM, interrupted)
node = Node('car15_end_straight_1m_trial')
be = QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
subscriptions.append(node.create_subscription(ConeDetections,'/car/cone_detections',cone_callback,10))
subscriptions.append(node.create_subscription(Imu,'/car/imu/data',imu_callback,be))
subscriptions.append(node.create_subscription(DiagnosticArray,'/car/diagnostics',diagnostics,10))
bind(Float32,'/car/wheel_speed_m_per_sec','wheel')
bind(Float32,'/car/steering_trim_current_normalized','steering_trim')
bind(Float32,'/car/steering_trim_set_normalized','steering_trim_set_command')
bind(Float32,'/car/steering_trim_increment_normalized','steering_trim_increment_command')
bind(Float32,'/car/debug2','distance')
bind(Float32,'/car/debug1','path_error')
bind(TrajectoryPath,'/car/policy_trajectory','trajectory',lambda m:{'frame_id':m.header.frame_id,'stamp_s':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'point_count':len(m.poses),'positions_xy':[[p.pose.position.x,p.pose.position.y] for p in m.poses]})
bind(Int8,'/car/policy_fsm_state_value','policy_state')
bind(String,'/car/policy_fsm_state_string','policy_text')
bind(String,'/car/traxxas_state','vehicle_state')
for topic,name in (('/car/drive_and_steer_set_point_normalized','actions'),('/car/drive_and_steer_applied_normalized','applied')):
    bind(DriveAndSteer,topic,name,lambda m:{'units':m.units,'drive':m.drive,'steer':m.steer})
policy = node.create_publisher(UInt16,'/car/policy_fsm_transition_request',10)
vehicle = node.create_publisher(UInt8,'/car/request',10)
client = node.create_client(GetParameters,'/car/ai4r_policy/get_parameters')
vehicle_client = node.create_client(GetParameters,'/car/traxxas_vehicle_interface/get_parameters')
try:
    if not spin_until(lambda: policy.get_subscription_count()>0 and vehicle.get_subscription_count()>0,5):
        raise RuntimeError('Policy or vehicle command subscription not discovered')
    policy.publish(UInt16(data=2))
    vehicle.publish(UInt8(data=0))
    spin_until(lambda: latest.get('policy_state')==2 and latest.get('vehicle_state') in ('Disabled','Disarmed'),3)
    if not client.wait_for_service(timeout_sec=2): raise RuntimeError('Policy parameter service unavailable')
    names=['control.enabled','control.mode','control.max_distance_m','control.max_run_time_s','control.end_straight_distance_m','control.end_straight_drive_normalized','control.end_straight_max_time_s','planning.cruise_speed_mps','control.drive_max','control.steering_max_normalized','control.mvp_lateral_kp','control.mvp_heading_kp','control.mvp_steering_slew_rate_per_s','control.mvp_drive_feedforward','control.mvp_steering_direction','control.speed_kp','control.speed_ki','estimation.left_colour','estimation.right_colour','planning.algorithm','estimation.road_width_min_m','estimation.road_forward_max_m','estimation.road_max_abs_slope','estimation.huber_delta_m','estimation.single_side_from_learned_width','visualization.enabled','policy_update_mode','policy_update_rate_hz','required_sensors']
    request=GetParameters.Request(names=names)
    future=client.call_async(request)
    if not spin_until(future.done,3): raise RuntimeError('Policy parameter query timeout')
    values=future.result().values
    params={name:({1:v.bool_value,2:v.integer_value,3:v.double_value,4:v.string_value,9:list(v.string_array_value)}.get(v.type)) for name,v in zip(names,values)}
    event('parameters',params)
    if not (params.get('control.enabled') is True and params.get('control.mode')=='mvp'
            and params.get('control.max_distance_m') == 0.0
            and params.get('control.max_run_time_s') == 0.0
            and params.get('control.end_straight_distance_m') == 1.0
            and params.get('control.end_straight_drive_normalized') == 0.30
            and params.get('control.end_straight_max_time_s') == 8.0
            and 0 < params.get('planning.cruise_speed_mps',999) <= 0.2
            and 0 < params.get('control.drive_max',999) <= 0.35
            and params.get('control.steering_max_normalized') == 1.0
            and params.get('control.mvp_lateral_kp') == 2.5
            and params.get('control.mvp_heading_kp') == 1.0
            and params.get('control.mvp_steering_slew_rate_per_s') == 2.0
            and params.get('control.mvp_drive_feedforward') == 0.30
            and params.get('control.mvp_steering_direction') == -1.0
            and params.get('estimation.left_colour') == 1
            and params.get('estimation.right_colour') == 2
            and params.get('planning.algorithm') == 'road_centerline'
            and params.get('estimation.road_width_min_m') == 0.15
            and params.get('estimation.road_forward_max_m') == 4.0
            and params.get('estimation.road_max_abs_slope') == 3.0
            and params.get('estimation.huber_delta_m') == 0.2
            and params.get('estimation.single_side_from_learned_width') is True
            and params.get('visualization.enabled') is True
            and params.get('policy_update_mode') == 'timer'
            and params.get('policy_update_rate_hz') == 20.0
            and params.get('required_sensors') == ['cone_detections','wheel_speed','imu_angular_velocity']):
        raise RuntimeError('MVP bounds not confirmed')
    if not vehicle_client.wait_for_service(timeout_sec=3): raise RuntimeError('Vehicle parameter service unavailable')
    vnames=['steering_min_normalized','steering_max_normalized']
    vf=vehicle_client.call_async(GetParameters.Request(names=vnames))
    if not spin_until(vf.done,3): raise RuntimeError('Vehicle steering interval query timeout')
    vparams={k:v.double_value for k,v in zip(vnames,vf.result().values)}
    event('vehicle_parameters',vparams)
    if vparams != {'steering_min_normalized':-0.8,'steering_max_normalized':0.8}: raise RuntimeError('Vehicle steering interval not confirmed: '+str(vparams))
    if not spin_until(lambda:fresh('steering_trim',1.5),3):
        raise RuntimeError('Live steering trim telemetry unavailable')
    if abs(latest['steering_trim']) > 1e-6:
        raise RuntimeError('Live steering trim differs from tested zero baseline: '+str(latest['steering_trim']))
    print('Waiting for fresh sensors, valid RC signals and zero commands (up to 90 seconds)', flush=True)
    if not spin_until(lambda: healthy() and abs(latest.get('wheel',999)) < 0.005 and latest.get('policy_state')==2 and fresh('actions',0.3) and latest['actions']['drive']==0 and latest['actions']['steer']==0,90):
        raise RuntimeError('Preflight failed: '+json.dumps({'imu':latest.get('imu_diagnostics'), 'gyro_received': 'gyro' in received, 'colour_layout':latest.get('colour_layout'), 'vehicle':latest.get('vehicle_diagnostics')}))
    event('phase','Enable vehicle')
    vehicle.publish(UInt8(data=1))
    if not spin_until(lambda: latest.get('vehicle_state')=='Enabled',6):
        raise RuntimeError('Vehicle did not become Enabled')
    event('phase','Start policy once')
    initial_text=latest.get('policy_text')
    max_speed = max_drive = max_distance = 0.0
    launch_time=time.monotonic()
    print('Starting continuous drive; distance/time test budgets disabled', flush=True)
    policy.publish(UInt16(data=3))
    seen_active=False
    stop_reason='Test deadline reached'
    next_progress = launch_time
    while rclpy.ok():
        if time.monotonic() >= next_progress:
            print(json.dumps({'run_seconds':round(time.monotonic()-launch_time,1),'distance_m':latest.get('distance'),'path_error_m':latest.get('path_error'),'wheel_mps':latest.get('wheel'),'steer':latest.get('actions',{}).get('steer'),'trajectory_points':latest.get('trajectory',{}).get('point_count'),'trajectory_frame':latest.get('trajectory',{}).get('frame_id'),'steering_trim':latest.get('steering_trim'),'policy_text':latest.get('policy_text')}),flush=True)
            next_progress=time.monotonic()+5
        rclpy.spin_once(node,timeout_sec=0.03)
        if latest.get('policy_state')==3: seen_active=True
        if abs(latest.get('steering_trim',0.0)) > 1e-6:
            stop_reason='Live steering trim changed from tested zero baseline: '+str(latest['steering_trim']); break
        if max_speed > 0.5:
            stop_reason='Measured speed exceeded test bound 0.5 m/s'; break
        if time.monotonic()-launch_time > 3 and max_speed == 0:
            stop_reason='No wheel motion after 3 seconds'; break
        if received.get('policy_text',0)>launch_time and latest.get('policy_text','').startswith('Publishing zero actions:'):
            if seen_active or (time.monotonic()-launch_time>0.25 and latest.get('policy_text')!=initial_text):
                stop_reason=latest['policy_text']; break
        if time.monotonic()-launch_time>1 and latest.get('vehicle_state')!='Enabled':
            stop_reason='Vehicle left Enabled: '+str(latest.get('vehicle_state')); break
        if time.monotonic()-launch_time>2 and not seen_active:
            stop_reason='Policy did not enter active state'; break
    result={'result':'completed' if 'End straight distance reached' in stop_reason else 'stopped', 'stop_reason':stop_reason,'run_seconds':time.monotonic()-launch_time,'seen_active':seen_active}
except BaseException as exc:
    result={'result':'aborted','reason':str(exc),'exception':type(exc).__name__}
    event('error',result)
finally:
    event('phase','Stop policy, confirm zero actions, then Disable vehicle')
    policy.publish(UInt16(data=2))
    spin_until(lambda: fresh('actions',0.3) and latest['actions']['drive']==0 and latest['actions']['steer']==0,1.0)
    for _ in range(3):
        policy.publish(UInt16(data=2))
        vehicle.publish(UInt8(data=0))
        spin_until(lambda:False,0.2)
    spin_until(lambda:latest.get('policy_state')==2 and latest.get('vehicle_state') in ('Disabled','Disarmed'),3)
    stopped = spin_until(lambda: latest.get('policy_state')==2 and latest.get('vehicle_state')=='Disabled' and fresh('wheel',0.3) and abs(latest['wheel'])<0.005 and fresh('applied',0.3) and latest['applied']['drive']==0,3)
    trajectory_cleared=spin_until(lambda:fresh('trajectory',3.0) and latest['trajectory']['point_count']==0,1.0)
    result.update(final_steering_trim=latest.get('steering_trim'),nonempty_path_messages=nonempty_path_messages,max_path_points=max_path_points,trajectory_cleared=trajectory_cleared,stop_telemetry_confirmed=stopped, imu_diagnostics=latest.get('imu_diagnostics'))
    result.update(max_wheel_speed_mps=max_speed,max_requested_drive=max_drive,max_distance_debug_m=max_distance,final_policy_state=latest.get('policy_state'),final_vehicle_state=latest.get('vehicle_state'),final_policy_text=latest.get('policy_text'),vehicle_diagnostics=latest.get('vehicle_diagnostics'),latest_cones=latest.get('cones'))
    event('result',result)
    (root/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'latest_cones'},indent=2,ensure_ascii=False),flush=True)
    node.destroy_node()
    rclpy.shutdown()
    log.close()
