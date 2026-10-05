#!/usr/bin/env python3
"""只订阅并保存 JSONL；不发送控制或状态请求。需要车上的 ROS 2 环境。"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time


def clean(value):
    """JSON 不保存 NaN/Inf；无效值留空，不替换为测量零。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", required=True, help="实际车辆命名空间，例如 car")
    parser.add_argument("--car-id", required=True)
    parser.add_argument("--notes", default="", help="电池、载荷、地面、配置快照路径")
    parser.add_argument("--output", required=True, type=Path, help="新 JSONL 文件；不覆盖")
    args = parser.parse_args()
    namespace = args.namespace.strip("/")
    if not namespace or any(c.isspace() for c in namespace):
        parser.error("请提供非空车辆命名空间")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float32, Int8, String
    from dream_interfaces.msg import DriveAndSteer

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8", buffering=1) as stream:
        rclpy.init()
        node = Node("vehicle_identification_recorder", namespace=namespace)
        sequence = 0

        def write(event, data):
            nonlocal sequence
            record = {"schema": 1, "sequence": sequence, "event": event,
                      "receive_monotonic_s": time.monotonic(),
                      "receive_ros_ns": node.get_clock().now().nanoseconds,
                      "data": clean(data)}
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            sequence += 1

        def stamp_ns(msg):
            return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        def vector(v):
            return [v.x, v.y, v.z]

        def imu(msg):
            # Raw sensor frame, not the transformed body-frame yaw rate in samples.
            write("imu_raw", {
                "source_stamp_ns": stamp_ns(msg), "frame_id": msg.header.frame_id,
                "angular_velocity": (None if msg.angular_velocity_covariance[0] == -1
                                     else vector(msg.angular_velocity)),
                "specific_force": (None if msg.linear_acceleration_covariance[0] == -1
                                   else vector(msg.linear_acceleration)),
            })

        def sample(msg):
            try:
                data = json.loads(msg.data)
            except ValueError:
                write("invalid_sample", {"text": msg.data})
            else:
                write("sample", data)

        reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions = [
            node.create_subscription(String, "identification_sample", sample, reliable),
            node.create_subscription(DriveAndSteer, "drive_and_steer_set_point_normalized",
                lambda m: write("command", {"drive": m.drive, "steer": m.steer,
                                           "units": m.units}), reliable),
            node.create_subscription(Float32, "wheel_speed_m_per_sec",
                lambda m: write("wheel_speed", {"speed_mps": m.data,
                                               "source_stamp_ns": None}), reliable),
            node.create_subscription(Imu, "imu/data", imu, best_effort),
            node.create_subscription(Int8, "policy_fsm_state_value",
                lambda m: write("state", {"value": m.data}), reliable),
            node.create_subscription(String, "policy_fsm_state_string",
                lambda m: write("status", {"text": m.data}), reliable),
        ]
        write("metadata", {"car_id": args.car_id, "namespace": namespace,
              "notes": args.notes, "utc": datetime.now(timezone.utc).isoformat(),
              "command_meaning": "published request, not measured actuator output",
              "wheel_speed_unsigned": True})
        print(f"Recording /{namespace} -> {args.output}; Ctrl+C stops recording ONLY.", flush=True)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                write("recorder_end", {"subscriptions": len(subscriptions)})
            finally:
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == "__main__":
    main()
