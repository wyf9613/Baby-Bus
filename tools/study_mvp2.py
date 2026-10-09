#!/usr/bin/env python3
"""Actual YAML/estimation/planning/control geometry rollout, without hardware.

Wheel speed follows the requested target with a labelled first-order lag. This
does not model ESC effort, grip, camera errors or qualify physical completion.
The deterministic worker port releases results after a configured delay;
installed ROS tests exercise the actual subprocess and callback scheduling.
"""
import argparse
from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
import runpy
import statistics
from types import SimpleNamespace
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]
api = runpy.run_path(str(ROOT/"tests/test_control.py"))
ns = api["ns"]
params = yaml.safe_load((ROOT/"config/ai4r_policy.yaml").read_text())["/**/ai4r_policy"]["ros__parameters"]


def settings(cls, prefix):
    values = params[prefix] if prefix != "lattice" else params["planning"]["lattice"]
    return ns[cls](**{k: values[k] for k in vars(ns[cls]()) if k in values})


class StudyWorker:
    ready = True
    process = SimpleNamespace(poll=lambda: None)

    def __init__(self, node):
        self.node, self.pending_at, self.result, self.planner = node, None, None, None
        self.runtimes = []
        self.generation = None
        self.delay_s = 0.05

    def submit(self, job, now):
        if self.pending_at is not None:
            return False
        if self.generation != job["generation"]:
            self.planner = ns["FrenetLatticePlanner"](ns["PlanningSettings"](**job["planning"]),
                ns["LatticeSettings"](**job["lattice"]), job["frame_id"])
            self.generation = job["generation"]
        started = time.perf_counter()
        ref, diag = ns["_mvp_plan"](self.planner, ns["ControlSettings"](**job["control"]), job["args"])
        self.runtimes.append(time.perf_counter()-started)
        self.pending_at = now
        self.result = {"generation": job["generation"], "reference": ref,
                       "diagnostics": diag, "road": deepcopy(job["args"][0])}
        return True

    def poll(self):
        if self.pending_at is not None and self.node._monotonic()-self.pending_at+1e-9 >= self.delay_s:
            result = self.result
            self.pending_at, self.result = None, None
            return result
        return None


def scene(bend=0.0, angle_deg=0.0, offset_m=0.0, dropout=False, steps=600):
    node = api["make_node"](simulated=False, mvp=True)
    node.estimation_settings = settings("EstimationSettings", "estimation")
    node.motion_history = ns["EstimationMotionHistory"](node.estimation_settings)
    node.planning_settings = settings("PlanningSettings", "planning")
    node.lattice_settings = settings("LatticeSettings", "lattice")
    node.control_settings = settings("ControlSettings", "control")
    node.controller = ns["PolicyController"](node.control_settings)
    node.distance_limiter = ns["RunDistanceLimiter"](node.control_settings.max_distance_m, node.control_settings.max_dt_s)
    worker = node.async_planner = StudyWorker(node)
    x, y, heading, speed, delta = 0.0, offset_m, math.radians(angle_deg), 0.0, 0.0
    trace, controls = [], []
    for step in range(steps):
        now, mono = node.test_clock.ros_ns, node.test_clock.monotonic
        beta = math.atan(0.4*math.tan(delta))
        yaw = speed*math.cos(beta)*math.tan(delta)/0.33
        node._store("wheel_speed", speed, None, mono, now)
        node._store("imu_angular_velocity", (0, 0, yaw), now, mono, now)
        cones = []
        # A 1.2 m wide, fully observed analytic corridor; preserve true frame.
        for side, colour in ((0.6, 2), (-0.6, 1)):
            for j in range(32):
                wx = x-0.7+j*0.14
                wy, slope = bend*wx*wx, 2*bend*wx
                norm = math.hypot(1, slope)
                dx, dy = wx-side*slope/norm-x, wy+side/norm-y
                px = math.cos(heading)*dx+math.sin(heading)*dy
                py = -math.sin(heading)*dx+math.cos(heading)*dy
                if -0.5 <= px <= 3.0:
                    cones.append((px, py, 0, colour, 0.95))
        empty = dropout and (70 <= step < 78 or 130 <= step < 138)
        batch = {"detections": [] if empty else cones, "acquisition_to_publish_latency_s": 0.0}
        node._store("cone_detections", batch, now, mono, now)
        if not empty:
            node.last_nonempty_cones = node.observations["cone_detections"]
        if step == 0:
            api["start"](node)
        worker.delay_s = 0.2 if dropout and 100 <= step < 105 else 0.05
        before = time.perf_counter()
        old_total = sum(worker.runtimes)
        node.run_policy_step()
        # Remove worker work from parent time: real execution is a process.
        controls.append(max(0.0, time.perf_counter()-before-(sum(worker.runtimes)-old_total)))
        action = node.action_publisher.messages[-1]
        if node.fsm_state == 2:
            break
        target = node.control_reference_manager.last_target_speed_mps if hasattr(node, "control_reference_manager") else None
        target = target or 0.0
        speed += (target-speed)*(1-math.exp(-0.05/0.2))
        # Same-car negative steering sign is explicitly represented here.
        requested = action.steer*node.control_settings.mvp_steering_direction*math.pi/4
        delta += max(-math.pi/2*0.05, min(math.pi/2*0.05, requested-delta))
        beta = math.atan(0.4*math.tan(delta))
        x += speed*math.cos(heading+beta)*0.05
        y += speed*math.sin(heading+beta)*0.05
        heading += speed*math.cos(beta)*math.tan(delta)/0.33*0.05
        trace.append({"time_s": step*0.05, "x_m": x, "y_m": y, "heading_rad": heading,
                      "error_m": (y-bend*x*x)/math.hypot(1, 2*bend*x), "speed_mps": speed,
                      "mode": (node.control_diagnostics or {}).get("tracking_mode"),
                      "distance_m": node.distance_limiter.distance_m})
        node.test_clock.advance(0.05)
    def timing(values):
        if not values:
            return None
        values = sorted(values)
        return {"median_ms": statistics.median(values)*1000, "p95_ms": values[int(.95*(len(values)-1))]*1000,
                "max_ms": max(values)*1000, "over_35ms": sum(v >= .035 for v in values)}
    return {"bend": bend, "angle_deg": angle_deg, "offset_m": offset_m, "dropout": dropout,
            "completed": node.fsm_state == 2 and "Distance limit reached" in node.state_reason,
            "stop_reason": node.state_reason, "distance_m": node.distance_limiter.distance_m,
            "final_error_m": trace[-1]["error_m"] if trace else None,
            "planning_calls": len(worker.runtimes), "worker_timing": timing(worker.runtimes),
            "control_timing": timing(controls), "trace": trace}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT/".verification/mvp2-study.json")
    args = parser.parse_args()
    scenes = [(0, 0, .15, False), (0, 30, .15, False), (0, -30, -.15, False),
              (0, 45, 0, False), (.03, 0, .1, False), (-.03, 0, -.1, False),
              (.08, 0, .1, False), (.03, 30, .1, True)]
    results = [scene(*s) for s in scenes]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"model": "approximate bicycle, ideal target-speed lag; synthetic corridor",
                                      "results": results}, indent=2)+"\n")
    for result in results:
        print(json.dumps({k: v for k, v in result.items() if k != "trace"}))
    if not all(r["completed"] for r in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
