"""Run a sequence of explicit commands; no policy is implemented here."""
import argparse
import json
import math
from pathlib import Path

from .environment import PlannerSimulationEnv, SensorTiming


def _json_value(value):
    if isinstance(value, dict):
        return {k: _json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--drive", type=float, default=0.1)
    parser.add_argument("--steering", type=float, default=0.0)
    parser.add_argument("--cone-period-steps", type=int, default=1)
    parser.add_argument("--cone-delay-steps", type=int, default=0)
    parser.add_argument("--without-obstacles", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    timing = SensorTiming(period_steps={"cone_detections": args.cone_period_steps},
                          delay_steps={"cone_detections": args.cone_delay_steps})
    with PlannerSimulationEnv(with_obstacles=not args.without_obstacles,
                              max_steps=args.steps, sensor_timing=timing) as env:
        observation, info = env.reset(seed=args.seed)
        configuration = info["configuration"]
        trace = []
        for _ in range(args.steps):
            observation, reward, terminated, truncated, info = env.step(
                (args.drive, args.steering, None, None, None))
            cones = observation["sensors"]["cone_detections"]
            trace.append({"time_s": observation["elapsed_time_s"], "reward": reward,
                          "cone_count": None if cones is None else len(cones["detections"]),
                          "road_valid": observation["estimation"]["road"]["valid"],
                          "road_status": observation["estimation"]["road"]["status"],
                          "collision": info["evaluation"]["collision"],
                          "progress_m": info["evaluation"]["ground_truth"]["reference_line_progress_m"],
                          "terminated": terminated, "truncated": truncated})
            if terminated or truncated:
                break
        report = {"configuration": configuration, "trace": trace,
                  "final_observation": observation, "final_evaluation": info["evaluation"]}
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(_json_value(report), ensure_ascii=False,
                                              indent=2, allow_nan=False), encoding="utf-8")
        print(json.dumps({"steps": len(trace), "elapsed_time_s": env.elapsed_time_s,
                          "final": trace[-1], "output": str(args.output) if args.output else None},
                         ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
