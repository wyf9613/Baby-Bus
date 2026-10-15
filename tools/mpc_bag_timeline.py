"""Timeline of an MPC exec/shadow bag: state changes, MPC branch/phase, speed, drive.

Run where ROS and dream_interfaces are sourced (WSL or the car):
    python3 tools/mpc_bag_timeline.py <bag dir>
Prints every state change and every MPC branch/phase/reject change, then the last 12 MPC steps.
"""
import json, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = []
    while r.has_next():
        topic, data, t = r.read_next()
        out.append((t/1e9, topic, deserialize_message(data, get_message(types[topic]))))
    return out

msgs = read(sys.argv[1])
t0 = msgs[0][0]
speed = applied = setp = None
last_state = {}
rows = []
prev_key = None
for t, topic, m in msgs:
    if topic == "/car/wheel_speed_m_per_sec": speed = m.data
    elif topic == "/car/drive_and_steer_applied_normalized": applied = (m.drive, m.steer)
    elif topic == "/car/drive_and_steer_set_point_normalized": setp = (m.drive, m.steer)
    elif topic in ("/car/policy_fsm_state_string", "/car/traxxas_state"):
        if m.data != last_state.get(topic):
            rows.append((True, f"{t-t0:7.2f} [{topic.split('/')[-1]}] {m.data}"))
            last_state[topic] = m.data
    elif topic == "/car/mpc_debug":
        d = json.loads(m.data)
        f = lambda k, n=3: "-" if d.get(k) is None else (f"{d[k]:.{n}f}" if isinstance(d[k], float) else d[k])
        line = (f"{t-t0:7.2f} mpc {f('branch'):11s} {f('phase'):9s} why={f('reject_reason')} v={f('speed_mps')} "
              f"cand_drive={f('candidate_drive')} app_drive={f('applied_drive')} steer={f('applied_steer_action',2)} "
              f"e_y={f('e_y_m')} bias={f('drive_bias')} | wheel={speed if speed is None else round(speed,3)} "
              f"setp={None if setp is None else tuple(round(x,3) for x in setp)} applied={None if applied is None else tuple(round(x,3) for x in applied)}")
        key = (d.get('branch'), d.get('phase'), d.get('reject_reason'))
        rows.append((key != prev_key, line)); prev_key = key

mpc_idx = [i for i, (_, l) in enumerate(rows) if ' mpc ' in l]
tail = set(mpc_idx[-12:])
for i, (important, line) in enumerate(rows):
    if important or i in tail:
        print(line)
