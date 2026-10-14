#!/usr/bin/env python3
"""Which cone colour is on the LEFT of the road? Prints the make_deploy_yaml --cones value.

    python3 tools/cone_layout_check.py [--seconds 3]      # on the car, ROS environment sourced

Put the car centred between the two cone rows, facing along the road, Disabled.
Reads /car/cone_detections (dream_interfaces/ConeDetections, base_link: +y is LEFT)
and reports the median y of each colour over cones 0.3-2.5 m ahead. ROS colour IDs:
yellow=1, blue=2. The estimator assumes one fixed pair (estimation.left_colour /
right_colour); a swapped pair makes every road invalid_width_or_order.
"""
import argparse
import statistics
import sys
import time

TOPIC = "/car/cone_detections"
COLOURS = {1: "yellow", 2: "blue"}
LAYOUTS = {"yellow": "yellow-left", "blue": "blue-left"}


def layout(cones, min_count=3):
    """cones: iterable of (x, y, colour, confidence). Returns (layout or None, per-colour stats)."""
    ys = {name: [] for name in COLOURS.values()}
    for x, y, colour, confidence in cones:
        if colour in COLOURS and 0.3 <= x <= 2.5 and confidence >= 0.5:
            ys[COLOURS[colour]].append(y)
    stats = {name: (len(v), statistics.median(v) if v else None) for name, v in ys.items()}
    if any(count < min_count for count, _ in stats.values()):
        return None, stats
    left = [name for name, (_, median) in stats.items() if median > 0]
    right = [name for name, (_, median) in stats.items() if median < 0]
    if len(left) != 1 or len(right) != 1:
        return None, stats      # both on one side: the car is not between the rows
    return LAYOUTS[left[0]], stats


def collect(seconds):
    import rclpy
    from dream_interfaces.msg import ConeDetections
    rclpy.init()
    node = rclpy.create_node("cone_layout_check")
    cones, frames = [], set()

    def callback(msg):
        frames.add(msg.header.frame_id)
        cones.extend((c.position.x, c.position.y, c.color, c.classification_confidence) for c in msg.detections)

    node.create_subscription(ConeDetections, TOPIC, callback, 10)
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return cones, frames


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()
    cones, frames = collect(args.seconds)
    if frames != {"base_link"}:
        print(f"cone frames {sorted(frames) or 'none received'}: expected base_link only")
        return 1
    result, stats = layout(cones)
    for name, (count, median) in stats.items():
        print(f"{name:6s} {count:4d} detections, median y {'-' if median is None else f'{median:+.3f} m'}")
    if result is None:
        print("UNDECIDED: need >= 3 of each colour, one colour at y > 0 and the other at y < 0")
        return 1
    print(f"--cones {result}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
