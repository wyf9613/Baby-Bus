#!/usr/bin/env python3
"""离线工具：预览实验、几何计算、转向拟合、导出日志。不会连接车辆。"""

import argparse
import ast
import csv
import json
import math
from numbers import Real
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[2]


def load_experiment():
    """测试实际策略中的纯逻辑，不导入 ROS，也不复制另一份控制逻辑。"""
    source = ROOT / "scripts/policy_node.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    names = {"finite_number", "IdentificationExperiment"}
    nodes = [n for n in tree.body if
             isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in names or
             isinstance(n, ast.Assign) and any(
                 isinstance(t, ast.Name) and t.id == "IDENTIFICATION_DEFAULTS" for t in n.targets)]
    scope = {"math": math, "Real": Real}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), scope)
    return scope["IdentificationExperiment"], scope["IDENTIFICATION_DEFAULTS"]


def number(value, name):
    if isinstance(value, bool):
        raise ValueError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def geometry(wheelbase, front_load, rear_load, front_overhang, rear_overhang, left, right):
    values = [number(v, "geometry") for v in
              (wheelbase, front_load, rear_load, front_overhang, rear_overhang, left, right)]
    length, front, rear, front_end, rear_end, left, right = values
    if min(length, front, rear, left, right) <= 0 or min(front_end, rear_end) < 0:
        raise ValueError("Lengths/loads must be positive; axle-to-body overhangs nonnegative")
    lr = length * front / (front + rear)
    return {"wheelbase_m": length, "rear_axle_from_cg_m": lr,
            "body_front_extent_from_cg_m": length - lr + front_end,
            "body_rear_extent_from_cg_m": lr + rear_end,
            "body_width_m": 2 * max(left, right)}


def equivalent_angle(left_deg, right_deg):
    left, right = (math.radians(number(x, "wheel angle")) for x in (left_deg, right_deg))
    if max(abs(left), abs(right)) >= math.pi / 2:
        raise ValueError("Wheel angles must be strictly between -90 and 90 degrees")
    if left == right == 0:
        return 0.0
    if left * right <= 0:
        raise ValueError("Near-zero/mixed-sign wheel angles need manual geometric review")
    a, b = math.tan(left), math.tan(right)
    return math.atan(2 * a * b / (a + b))


def steering_fit(rows):
    points = []
    for row in rows:
        u = number(row["steering_action"], "steering_action")
        if not -1 <= u <= 1:
            raise ValueError("steering_action must lie in [-1, 1]")
        # An independently established equivalent angle overrides Ackermann conversion.
        if row.get("delta_equivalent_deg", "").strip():
            angle = math.radians(number(row["delta_equivalent_deg"], "delta_equivalent_deg"))
            if abs(angle) >= math.pi / 2:
                raise ValueError("Equivalent angle must be between -90 and 90 degrees")
        else:
            angle = equivalent_angle(row["delta_left_deg"], row["delta_right_deg"])
        points.append((u, angle))
    if len({u for u, _ in points}) < 3 or not (min(u for u, _ in points) < 0 < max(u for u, _ in points)):
        raise ValueError("Need >=3 distinct requests covering both left and right")
    mean_u = statistics.mean(u for u, _ in points)
    mean_a = statistics.mean(a for _, a in points)
    gain = sum((u - mean_u) * (a - mean_a) for u, a in points) / sum(
        (u - mean_u) ** 2 for u, _ in points)
    offset = mean_a - gain * mean_u
    residuals = [a - (gain * u + offset) for u, a in points]
    return {
        "parameters": {"steering_gain_rad": gain, "steering_offset_rad": offset,
                       "steering_min_rad": min(a for _, a in points),
                       "steering_max_rad": max(a for _, a in points)},
        "fit_rmse_rad": math.sqrt(statistics.mean(r * r for r in residuals)),
        "max_abs_residual_rad": max(abs(r) for r in residuals),
        "request_range": [min(u for u, _ in points), max(u for u, _ in points)],
        "points": len(points),
        "limits_meaning": "observed angles within tested request range, not mechanical limits",
        "validation": "not performed; check repeatability, hysteresis and independent runs",
    }


def save_json(path, result):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def steering_dynamics(rows, threshold_deg, hold_samples):
    """已同步外部角度数据的响应摘要；不自动认定物理速率上限或纯延迟。"""
    threshold = math.radians(number(threshold_deg, "threshold_deg"))
    if threshold <= 0 or hold_samples < 2:
        raise ValueError("Positive noise threshold and at least 2 hold samples are required")
    data = [(number(r["time_s"], "time_s"), number(r["steering_action"], "steering_action"),
             math.radians(number(r["delta_equivalent_deg"], "delta_equivalent_deg"))) for r in rows]
    if len(data) < 2 or any(data[i][0] <= data[i-1][0] for i in range(1, len(data))):
        raise ValueError("Need strictly increasing synchronized timestamps")
    if any(abs(u) > 1 or abs(a) >= math.pi / 2 for _, u, a in data):
        raise ValueError("Invalid request or equivalent wheel angle")
    changes = [i for i in range(1, len(data)) if data[i][1] != data[i-1][1]]
    results = []
    for index, start in enumerate(changes):
        end = changes[index+1] if index+1 < len(changes) else len(data)
        baseline = statistics.median(a for _, _, a in data[max(0, start-5):start])
        onset = None
        for j in range(start, end - hold_samples + 1):
            offsets = [data[k][2] - baseline for k in range(j, j + hold_samples)]
            if min(offsets) > threshold or max(offsets) < -threshold:
                onset = j
                break
        slopes = []
        if onset is not None:
            final = statistics.median(a for _, _, a in data[max(onset, end-3):end])
            change = final - baseline
            if abs(change) > 2 * threshold:
                for k in range(onset+1, end):
                    progress = (data[k][2] - baseline) / change
                    previous = (data[k-1][2] - baseline) / change
                    if 0.2 <= previous <= 0.8 and 0.2 <= progress <= 0.8:
                        slopes.append(abs((data[k][2] - data[k-1][2]) / (data[k][0] - data[k-1][0])))
        results.append({"command_time_s": data[start][0], "steering_action": data[start][1],
                        "observed_delay_s": None if onset is None else data[onset][0] - data[start][0],
                        "mid_response_rate_rad_s": statistics.median(slopes) if slopes else None})
    return {"transitions": results, "threshold_deg": threshold_deg, "hold_samples": hold_samples,
            "sample_interval_max_s": max(data[i][0] - data[i-1][0] for i in range(1, len(data))),
            "meaning": "Observed onset and middle-response slope; not verified pure delay or rate limit."}


def export_log(path, output_dir):
    output_dir.mkdir(parents=True, exist_ok=False)
    sample_fields = ["run_id", "receive_monotonic_s", "policy_monotonic_s", "policy_ros_ns",
                     "elapsed_s", "dt_s", "mode", "phase", "stage", "drive", "steer",
                     "speed_mps", "body_yaw_rate_rad_s", "wheel_age_s", "gyro_age_s",
                     "gyro_stamp_ns", "terminal", "reason"]
    event_fields = ["sequence", "event", "receive_monotonic_s", "receive_ros_ns", "data_json"]
    runs, warnings = {}, []
    with path.open(encoding="utf-8") as source, \
            (output_dir / "samples.csv").open("x", newline="", encoding="utf-8") as samples, \
            (output_dir / "events.csv").open("x", newline="", encoding="utf-8") as events:
        sw, ew = csv.DictWriter(samples, sample_fields), csv.DictWriter(events, event_fields)
        sw.writeheader()
        ew.writeheader()
        for line_no, line in enumerate(source, 1):
            try:
                event = json.loads(line)
            except ValueError:
                warnings.append(f"Invalid/truncated JSON at line {line_no}")
                continue
            data = event.get("data", {})
            ew.writerow({**{k: event.get(k) for k in event_fields[:-1]},
                         "data_json": json.dumps(data, ensure_ascii=False)})
            if event.get("event") != "sample":
                continue
            row = {key: data.get(key) for key in sample_fields}
            row["receive_monotonic_s"] = event.get("receive_monotonic_s")
            ages, stamps = data.get("sensor_age_s", {}), data.get("sensor_stamp_ns", {})
            row.update(wheel_age_s=ages.get("wheel_speed"), gyro_age_s=ages.get("imu_angular_velocity"),
                       gyro_stamp_ns=stamps.get("imu_angular_velocity"))
            sw.writerow(row)
            run = runs.setdefault(data["run_id"], {"samples": 0, "max_speed_mps": None,
                "config": None, "terminal": False, "last_reason": "", "last_phase": ""})
            run["samples"] += 1
            if data.get("config") is not None:
                run["config"] = data["config"]
            speed = data.get("speed_mps")
            if speed is not None:
                run["max_speed_mps"] = max(speed, run["max_speed_mps"] or 0)
            run.update(terminal=data.get("terminal", False), last_reason=data.get("reason", ""),
                       last_phase=data.get("phase", ""))
    save_json(output_dir / "summary.json", {
        "source": str(path), "runs": runs, "warnings": warnings,
        "notes": ["Samples contain planned requests; events contain actual published commands.",
                  "A missing terminal sample means interrupted/incomplete, not successful completion.",
                  "Repeated cached speeds are not independent measurements; inspect wheel events.",
                  "No automatic calibration or pure-delay estimate is inferred from these logs."]})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    preview = sub.add_parser("preview", help="校验并打印 YAML 实验计划，不连接 ROS")
    preview.add_argument("--config", type=Path, default=ROOT / "config/ai4r_policy.yaml")
    geo = sub.add_parser("geometry", help="由尺量和轴荷生成 5 项几何候选值")
    for key in ("wheelbase", "front-load", "rear-load", "front-overhang", "rear-overhang",
                "left-extent", "right-extent"):
        geo.add_argument("--" + key, required=True, type=float)
    geo.add_argument("--output", type=Path, required=True)
    steer = sub.add_parser("steering", help="由实测 CSV 拟合等效转向角映射")
    steer.add_argument("--csv", type=Path, required=True)
    steer.add_argument("--output", type=Path, required=True)
    dynamic = sub.add_parser("steering-dynamics", help="分析已同步的指令/等效轮角时间序列")
    dynamic.add_argument("--csv", type=Path, required=True)
    dynamic.add_argument("--threshold-deg", type=float, required=True, help="由静止角度噪声选定")
    dynamic.add_argument("--hold-samples", type=int, default=3)
    dynamic.add_argument("--output", type=Path, required=True)
    export = sub.add_parser("export", help="JSONL 导出为样本/事件 CSV 和运行摘要")
    export.add_argument("--log", type=Path, required=True)
    export.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command == "preview":
            import yaml
            params = yaml.safe_load(args.config.read_text(encoding="utf-8"))["/**/ai4r_policy"]["ros__parameters"]
            cls, defaults = load_experiment()
            config = {**defaults, **params.get("id_test", {})}
            if config.keys() != defaults.keys():
                raise ValueError("Unknown id_test configuration key")
            experiment = cls(config)
            if experiment.mode != "off":
                if params["policy_update_mode"] != "timer" or "wheel_speed" not in params["required_sensors"]:
                    raise ValueError("Test requires timer mode and required wheel_speed")
                if 1.0 / params["policy_update_rate_hz"] >= config["max_dt_s"]:
                    raise ValueError("Timer interval must be shorter than max_dt_s")
            print(json.dumps({"config": config, "total_s": getattr(experiment, "total_s", 0),
                              "preview_only": True}, indent=2))
        elif args.command == "geometry":
            params = geometry(args.wheelbase, args.front_load, args.rear_load,
                              args.front_overhang, args.rear_overhang, args.left_extent, args.right_extent)
            save_json(args.output, {"status": "candidate_not_validated", "parameters": params,
                      "measurements": {k: v for k, v in vars(args).items() if k not in ("output", "command")},
                      "units": "lengths in m; axle loads in one consistent unit",
                      "width_meaning": "CG-centred conservative envelope; no obstacle margin"})
        elif args.command == "steering":
            with args.csv.open(encoding="utf-8-sig", newline="") as stream:
                result = steering_fit(list(csv.DictReader(stream)))
            save_json(args.output, {"status": "candidate_not_validated", "source": str(args.csv), **result})
        elif args.command == "steering-dynamics":
            with args.csv.open(encoding="utf-8-sig", newline="") as stream:
                result = steering_dynamics(list(csv.DictReader(stream)), args.threshold_deg, args.hold_samples)
            save_json(args.output, {"status": "candidate_not_validated", "source": str(args.csv), **result})
        elif args.command == "export":
            export_log(args.log, args.output_dir)
    except ModuleNotFoundError as exc:
        parser.exit(2, f"Missing dependency: {exc.name}. YAML preview requires PyYAML.\n")
    except (ValueError, KeyError, OSError, ZeroDivisionError) as exc:
        parser.exit(2, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
