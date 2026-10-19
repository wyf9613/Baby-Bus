"""Export car MPC exec bags to the compact per-tick fixtures read by tools/car05_run_stats.py.

    pip install mcap mcap-ros2-support          # no ROS needed; run it in its own venv
    python tools/export_car_run.py tests/fixtures/car05_20261019_runs  path/to/bag_dir/*.mcap

One row per /car/mpc_debug message, with the latest raw wheel speed, IMU yaw rate and set
point at that tick, plus the FSM / Traxxas state changes. Writes <bag dir name>.json.gz.
"""
import gzip
import sys, json
from pathlib import Path
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
out = Path(sys.argv[1])
KEEP = ("branch","phase","reject_reason","speed_mps","speed_raw_mps","wheel_fresh","wheel_held_s","speed_override",
        "v_ref_mps","e_y_m","e_psi_rad","candidate_drive","applied_drive","applied_steer_action","drive_bias",
        "overspeed_state","overspeed_s","reference_selection","reference_selection_reason","reference_gate",
        "restarts","drive_cap","dt")
for p in sys.argv[2:]:
    t0=None; wheel=yaw=sp=fsm=None; ticks=[]; events=[]
    with open(p,"rb") as f:
        for s,ch,msg,m in make_reader(f,decoder_factories=[DecoderFactory()]).iter_decoded_messages():
            t=msg.log_time/1e9; t0=t0 or t; rt=round(t-t0,3)
            tp=ch.topic
            if tp=="/car/wheel_speed_m_per_sec": wheel=round(float(m.data),4)
            elif tp=="/car/imu/data": yaw=round(float(m.angular_velocity.z),4)
            elif tp=="/car/drive_and_steer_set_point_normalized": sp=(round(m.drive,4),round(m.steer,4))
            elif tp in ("/car/policy_fsm_state_string","/car/traxxas_state"):
                if not events or events[-1][1:]!=[tp.split("/")[-1],m.data]: events.append([rt,tp.split("/")[-1],m.data])
            elif tp=="/car/mpc_debug":
                d=json.loads(m.data)
                row={"t":rt,"wheel_raw":wheel,"yaw_rps":yaw,"set_point":sp}
                for k in KEEP:
                    v=d.get(k)
                    row[k]=round(v,4) if isinstance(v,float) else v
                ticks.append(row)
    name=Path(p).parent.name
    doc={"bag":name,"note":"t from the first message in the bag; wheel_raw/yaw_rps/set_point are the latest values at the mpc_debug tick",
         "events":events,"ticks":ticks}
    path = out/f"{name}.json.gz"
    with gzip.open(path, "wt", encoding="utf-8") as g:
        json.dump(doc, g, separators=(",", ":"))
    print(name, len(ticks), path.stat().st_size)
