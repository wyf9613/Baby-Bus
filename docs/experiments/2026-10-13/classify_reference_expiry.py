"""Classify archived rejection logs; does not execute policy or contact vehicle."""
import ast
import csv
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
rows = []
other = Counter()
for run in sorted((ROOT / "evidence/runs").iterdir()):
    for line_no, line in enumerate((run / "run.jsonl").read_text(encoding="utf-8").splitlines(), 1):
        event = json.loads(line)
        if event.get("kind") != "policy_log":
            continue
        message = event["value"].get("message", "")
        prefix = "Planning rejection details: "
        if not message.startswith(prefix):
            continue
        detail = ast.literal_eval(message[len(prefix):])
        diag = detail.get("planning_diagnostics", {})
        reason = detail.get("reason")
        if reason != "source_deadline_during_planning":
            other[reason] += 1
            continue
        ages, limits = diag.get("source_ages_s", {}), diag.get("source_age_limits_s", {})
        remaining = {name: limits[name] - ages[name] for name in limits if name in ages}
        remaining["trajectory_window"] = 0.2
        if diag.get("road_degraded"):
            remaining["prediction"] = diag["prediction_remaining_s"]
        minimum = min(remaining.values())
        binders = [key for key, value in remaining.items() if abs(value - minimum) < 1e-9]
        stamp = diag.get("result_timestamp_s")
        row = {"run": run.name, "raw_line": line_no, "trajectory_id": diag.get("trajectory_id"),
               "stage": "parent_transport" if diag.get("discarded_result") == "reference_expired" else "worker",
               "limiter": "+".join(binders), "road_remaining_ms": remaining.get("road", 0)*1000,
               "wheel_remaining_ms": remaining.get("wheel_speed", 0)*1000,
               "imu_remaining_ms": remaining.get("imu_angular_velocity", 0)*1000,
               "trajectory_window_ms": 200.0, "binding_lifetime_ms": minimum*1000,
               "planner_elapsed_ms": diag.get("elapsed_s", 0)*1000,
               "reference_timestamp_s": stamp,
               "absolute_deadline_s_reconstructed": None if stamp is None else stamp+minimum,
               "accurate_parent_poll_time_s": None,
               "raw_path": f"evidence/runs/{run.name}/run.jsonl"}
        rows.append(row)

out = ROOT / "REFERENCE_EXPIRY_DETAILS.csv"
with out.open("w", newline="", encoding="utf-8-sig") as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)

counts = Counter((r["stage"], r["limiter"]) for r in rows)
print(json.dumps({"source_deadline_records": len(rows),
                  "counts": {"/".join(k): v for k, v in counts.items()},
                  "other_rejections": dict(other)}, ensure_ascii=False, indent=2))
labels = {"road": "道路", "wheel_speed": "旧轮速", "imu_angular_velocity": "旧 IMU", "trajectory_window": "轨迹窗口"}
md = ["# 2026-10-13 逐条参考失效分类", "",
      "从全部 15 个归档 run.jsonl 解析 Planning rejection details。只分类顶层 source_deadline_during_planning；其他原因另计。没有运行规划器或连接车辆。", "",
      "有效期按部署规则还原为 min(0.2, 各输入年龄上限−worker 输入年龄)，而不是取外层当前道路年龄。输入限制来自每条诊断；轨迹上限 0.2 s 来自当天部署配置。", "",
      "分类表示决定合并截止时刻的最早限制。原代码只有一个 reference_expired 判断，没有独立报告哪类源过期。准确主进程取用时刻未记录，因此不能判断该时刻还有哪些非最早限制也同时过期。worker 行表示其 elapsed_s 达到还原的输入寿命；交付前排队时间不包含在这个耗时比较中。", "",
      "## 汇总", "", "| 拒绝阶段 | 最早限制 | 条数 |", "| --- | --- | ---: |"]
for (stage, limiter), count in sorted(counts.items()):
    md.append(f"| {'主进程取用 reference_expired' if stage == 'parent_transport' else 'worker 内期限检查'} | {labels.get(limiter, limiter)} | {count} |")
md += ["", "## 逐条记录", "",
       "剩余时间和规划耗时单位 ms。任务编号仅在同一轮内有意义；链接定位原始日志。", "",
       "| 轮次 | 任务 | 阶段 | 道路剩余 | 旧轮速剩余 | 旧 IMU 剩余 | 轨迹窗口 | 最早限制 | 规划耗时 | 原始行 |",
       "| --- | ---: | --- | ---: | ---: | ---: | ---: | --- | ---: | --- |"]
for r in rows:
    short = r["run"].removeprefix("drive-")
    stage = "取用过期" if r["stage"] == "parent_transport" else "worker 到期"
    nums = [f'{r[k]:.3f}' for k in ("road_remaining_ms", "wheel_remaining_ms", "imu_remaining_ms", "trajectory_window_ms")]
    md.append(f'| {short} | {r["trajectory_id"]} | {stage} | ' + " | ".join(nums) +
              f' | {labels.get(r["limiter"], r["limiter"])} | {r["planner_elapsed_ms"]:.3f} | [{r["raw_line"]}]({r["raw_path"]}#L{r["raw_line"]}) |')
md += ["", "## 未归为参考到期的拒绝", ""]
for reason, count in sorted(other.items()):
    md.append(f"- {reason}: {count} 条")
md += ["", "这些计数只覆盖打印的拒绝日志，不是全部规划任务的失败率。无明细的早期几何/监测中止未纳入期限分类。", "",
       "## 代码位置", "",
       "- scripts/policy_node.py:2066：逐源计算剩余寿命。",
       "- scripts/policy_node.py:2341：lifetime=min(reference_lifetime_s,*deadlines)，worker 耗时达到 lifetime 时拒绝。",
       "- scripts/policy_node.py:2866：transport_control_reference 计算 timestamp_s+valid_for_s-now_s；小于等于零抛 reference_expired。",
       "- scripts/policy_node.py:4689：主进程把上述异常统一标记为 source_deadline_during_planning。", ""]
(ROOT / "REFERENCE_EXPIRY_CLASSIFICATION_CN.md").write_text("\n".join(md), encoding="utf-8")
