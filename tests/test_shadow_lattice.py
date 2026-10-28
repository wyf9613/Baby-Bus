"""Fresh shadow definitions from actual selected policy source, without ROS.

Includes real subprocess protocol and public run-loop isolation tests; no
archived lattice imports, vehicle nodes or actuators are involved.

Ported from origin/mvp2.1 3652e9b (tests/test_shadow_lattice.py) on 2026-10-16:
ShadowChecks and ShadowTemporalChecks verbatim. Its ShadowIntegrationChecks
drive the MVP PID node and are replaced on this branch by the MPC checks in
test_mpc_planner_control.py.
"""
import ast
from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
import runpy
import subprocess
import sys
import queue
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

api = runpy.run_path(str(Path(__file__).with_name("test_estimation.py")))
ns = dict(api["namespace"])
source = api["SOURCE"]
tree = ast.parse(source.read_text(encoding="utf-8"))
definitions = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
    and (n.name in ("ShadowSettings", "ShadowFrame", "ShadowLatticePlanner", "ShadowWorker",
                   "PlannerControlSettings", "PlannerExecutionManager")
         or n.name.startswith("_shadow_")) and n.name != "_shadow_worker_entry"]
# mpc-car-run adaptation: the planner and PlannerExecutionManager share the
# module constant EXECUTION_TAIL_M (2026-10-20).
constants = [n for n in tree.body if isinstance(n, ast.Assign)
             and [getattr(t, "id", None) for t in n.targets] == ["EXECUTION_TAIL_M"]]
# mpc-car-run adaptation: this branch's test_estimation does not extract the
# ordered_xy road helpers (mvp2.1's does); add the ones it lacks.
definitions += [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name not in ns
                and (n.name.startswith("_path_") or n.name == "_estimation_order_boundary")]
ns["time"] = time
ns.update(queue=queue, threading=threading, subprocess=subprocess, sys=sys, json=json)
exec(compile(ast.Module(body=constants+definitions, type_ignores=[]), str(source), "exec"), ns)
# The estimator classes resolve globals in test_estimation's own namespace.
api["namespace"].update({n.name: ns[n.name] for n in definitions if n.name not in api["namespace"]})
Settings, Planner = ns["ShadowSettings"], ns["ShadowLatticePlanner"]


def job(center=0.0, speed=0.2):
    return {"settings": vars(Settings(sample_count=12)), "stamp_s": 100.0,
        "generation": 1, "job_id": 1, "submitted_monotonic_s": 10.0,
        "control_settings": {"enabled": True, "mode": "mvp", "mvp_lateral_kp": 2.5,
                             "mvp_heading_kp": 1.5, "mvp_steering_direction": -1.0},
        "preview_m": 0.35, "control_period_s": 0.05, "target_speed_mps": 0.2,
        "state": {"speed_mps": speed, "speed_valid": True},
        "road": {"centerline_xy": [(0.0, center), (0.5, center), (1.0, center)],
            "left_boundary_xy": [(0.0, 0.5), (1.0, 0.5)],
            "right_boundary_xy": [(0.0, -0.5), (1.0, -0.5)],
            "boundary_source": {"left": "observed", "right": "observed"},
            "lane_width_m": 1.0, "measurement_timestamp_s": 99.9}}


def plan(record, **overrides):
    return Planner(Settings(**{**record["settings"], **overrides})).plan(record)


class ShadowChecks(unittest.TestCase):
    def test_actual_speed_and_rate_determine_prefix_not_target_speed(self):
        record = job(speed=0.3)
        record["target_speed_mps"] = 0.01
        for rate in (1.0, 2.0, 10.0):
            result = plan(record, rate_hz=rate)
            self.assertAlmostEqual(result["check_length_m"], 0.3*(1/rate+0.1))
            self.assertGreaterEqual(result["generated_length_m"], result["control_read_length_m"])
            self.assertEqual(result["status"], "VALID")

    def test_short_single_point_and_boundary_only_are_not_admission_gates(self):
        for center in ([(0.6, 0.03)], [(0.6, 0.03), (0.62, 0.03)], []):
            record = job()
            record["road"].update(valid=False, status="near_field_unobserved", centerline_xy=center,
                                  right_boundary_xy=None, lane_width_m=None)
            record["road"]["left_boundary_xy"] = [(0.6, 0.5), (0.62, 0.5)]
            result = plan(record)
            self.assertEqual(result["candidate_count"], 12)
            self.assertEqual(result["status"], "VALID")
            self.assertGreaterEqual(result["generated_length_m"], 1.0)
            self.assertEqual(result["known_boundary_sides"], ["left"])
            self.assertEqual(result["selected"]["path"][0], (0.0, 0.0, 0.0))

    def test_observed_handoff_delay_expands_checked_execution_window(self):
        record = job(speed=0.2)
        record["handoff_estimate_s"] = 0.4
        result = plan(record, rate_hz=10.0)
        self.assertAlmostEqual(result["hold_time_s"], 0.5)
        self.assertAlmostEqual(result["check_length_m"], 0.1)

    def test_empty_geometry_is_the_only_road_admission_gate(self):
        record = job()
        record["road"] = {}
        self.assertEqual(plan(record)["status"], "NO_ROAD")

    def test_inferred_and_predicted_boundaries_never_hard_reject(self):
        record = job()
        record["road"]["left_boundary_xy"] = [(0.0, -0.1), (0.3, -0.1)]
        for source_label in ("width_inferred", "predicted_history", "absent"):
            record["road"]["boundary_source"]["left"] = source_label
            result = plan(record)
            self.assertEqual(result["status"], "VALID")
            self.assertEqual(result["known_boundary_sides"], ["right"])
        record["road"]["boundary_source"]["left"] = "observed"
        self.assertEqual(plan(record)["status"], "ALL_COLLIDE")

    def test_far_contact_does_not_reject_prefix_but_body_front_counts(self):
        record = job(speed=0.2)
        record["road"]["right_boundary_xy"] = [(0.6, 0.0), (0.8, 0.0)]
        self.assertEqual(plan(record, rate_hz=10.0)["status"], "VALID")
        record["road"]["right_boundary_xy"] = [(0.24, -0.1), (0.24, 0.1)]
        self.assertEqual(plan(record, rate_hz=10.0)["status"], "ALL_COLLIDE")

    def test_sweep_detects_crossing_between_path_grid_points(self):
        cfg = Settings(body_front_m=0.005, body_rear_m=0.005, body_width_m=0.01)
        hit = ns["_shadow_collision"]([(0.0, 0.0, 0.0), (0.2, 0.0, 0.0)], 0.2,
                                    {"left": [(0.1, -0.03), (0.1, 0.03)]}, cfg)
        self.assertIsNotNone(hit)

    def test_controller_limits_and_models_cannot_influence_candidate_ranking(self):
        record = job(center=0.2)
        record["road"].update(left_boundary_xy=None, right_boundary_xy=None)
        record["control_settings"].update(steering_max_normalized=0.01,
                                         mvp_steering_slew_rate_per_s=0.001)
        original = plan(record)
        record["control_settings"].update(mvp_lateral_kp=1e9, mvp_heading_kp=1e9)
        result = plan(record)
        self.assertEqual(result["feasible_count"], 12)
        self.assertEqual(result["selected"]["id"], original["selected"]["id"])
        self.assertEqual([c["cost"] for c in result["candidates"]], [c["cost"] for c in original["candidates"]])
        self.assertTrue(all("execution" not in c and "execution" not in c["costs"] for c in result["candidates"]))

    def test_curvature_cost_is_off_by_default_and_prefers_wider_paths_when_on(self):
        # 2026-10-28 experiment: car 01's left lock sits at the MPC model's limit.
        record = job(center=0.3)
        off = plan(record)
        self.assertTrue(all("curvature" not in c["costs"] for c in off["candidates"]))
        on = plan(record, weight_curvature=1e6, curvature_limit_1pm=0.5)
        excess = {c["id"]: c["costs"]["curvature"] for c in on["candidates"]}
        self.assertGreater(max(excess.values()), 0.0)
        feasible = [c["id"] for c in on["candidates"] if c["collision"] is None]
        self.assertEqual(excess[on["selected"]["id"]], min(excess[i] for i in feasible))
        self.assertLessEqual(excess[on["selected"]["id"]], excess[off["selected"]["id"]])
        for bad in (dict(curvature_limit_1pm=0.0), dict(curvature_window_m=0.0), dict(weight_curvature=-1.0)):
            with self.assertRaises(ValueError):
                Settings(**bad)

    def test_stage_timing_accounts_for_total_including_no_road(self):
        for record in (job(), {**job(), "road": {}}):
            result = plan(record)
            timing = result["timing_s"]
            self.assertEqual(timing["total"], result["elapsed_s"])
            self.assertAlmostEqual(sum(v for k, v in timing.items() if k != "total"), timing["total"], places=8)
            self.assertTrue(all(v >= 0 for v in timing.values()))

    def test_real_car15_continuous_curve_masks_each_side_before_reconstruction(self):
        root = Path(__file__).resolve().parents[1]
        # mpc-car-run adaptation: this branch's base YAML is the V2 y_of_x road;
        # the lattice runs on the deployed centerline layers' ordered_xy estimation.
        deploy = runpy.run_path(str(root/"tools/make_deploy_yaml.py"))
        config = deploy["build"]("mpc-shadow-centerline", 1.0, cones="yellow-left")[0][
            "/**/ai4r_policy"]["ros__parameters"]
        rows = [json.loads(line) for line in (root/"docs/experiments/2026-10-14/evidence/car15/visualization4/run.jsonl").read_text().splitlines()]
        frames = [r["value"] for r in rows if r["kind"] == "cones"]
        for side, colour in (("left", 1), ("right", 2)):
            pipeline = ns["EstimationPipeline"](ns["EstimationSettings"](**config["estimation"]))
            seed = frames[0]
            pipeline._history_road({"detections": seed["positions"]}, round(seed["stamp_s"]*1e9), 0, seed["stamp_s"])
            self.assertIsNotNone(pipeline._learned_lane_width_m)
            for frame in frames[150:156]:
                masked = [p for p in frame["positions"] if p[3] == colour]
                road = pipeline.road({"detections": masked}, round(frame["stamp_s"]*1e9), 0)
                hidden = "right" if side == "left" else "left"
                self.assertIsNone(road[hidden+"_boundary_xy"])
                self.assertEqual(road["visibility"], side+"_only")
                result = plan({**job(), "road": road})
                self.assertEqual(result["status"], "VALID")
                self.assertEqual(result["known_boundary_sides"], [side])
                self.assertIsNone(result["selected"]["collision"])

    def test_recenter_prefix_and_turn_preparation_have_explicit_candidates(self):
        record = job(center=0.10)
        result = plan(record)
        candidates = {c["family"]: c for c in result["candidates"] if c["family"] != "halton"}
        self.assertLess(candidates["fast_center"]["costs"]["lateral"], candidates["hold"]["costs"]["lateral"])
        for sign in (-1, 1):
            record = job()
            record["road"].update(left_boundary_xy=None, right_boundary_xy=None,
                centerline_xy=[(0.6*math.sin(i*.08), sign*.6*(1-math.cos(i*.08))) for i in range(20)])
            result = plan(record)
            self.assertLess(sign*result["preparation_offset_m"], 0)
            prep = next(c for c in result["candidates"] if c["family"] == "prepare")
            self.assertLess(sign*prep["offsets_m"][0], 0)

    def test_vertical_returning_road_preserves_order_and_generates(self):
        record = job()
        record["road"].update(centerline_xy=[(0.1, 0), (.4, 0), (.4, .3), (.1, .3), (-.1, .3)],
                              left_boundary_xy=None, right_boundary_xy=None)
        result = plan(record)
        self.assertEqual(result["status"], "VALID")
        self.assertTrue(all(math.isfinite(v) for c in result["candidates"] for p in c["path"] for v in p))

    def test_worker_protocol_executes_without_importing_ros(self):
        record = job()
        process = subprocess.run([sys.executable, str(source), "--shadow-worker"],
            input=json.dumps(record)+"\n", capture_output=True, text=True, timeout=15)
        self.assertEqual(process.returncode, 0, process.stderr)
        result = json.loads(process.stdout)
        self.assertEqual(result["status"], "VALID")
        self.assertEqual(result["job_id"], 1)
        self.assertEqual(result["generation"], 1)

    def test_real_background_transport_does_not_wait_on_submit_and_closes(self):
        worker = ns["ShadowWorker"](str(source))
        try:
            record = job()
            record["submitted_monotonic_s"] = time.monotonic()
            worker.submit(record)
            deadline = time.monotonic()+10
            result = None
            while result is None and time.monotonic() < deadline:
                result = worker.poll()
                time.sleep(0.01)
            self.assertIsNotNone(result)
            self.assertEqual(result["status"], "VALID", result)
            self.assertGreaterEqual(result["received_monotonic_s"], record["submitted_monotonic_s"])
        finally:
            worker.close()
            worker.thread.join(timeout=3)
        self.assertFalse(worker.thread.is_alive())


class ShadowTemporalChecks(unittest.TestCase):
    def record(self):
        record = job(center=0.05)
        record.update(shadow_pose=[0.0, 0.0, 0.0], shadow_odometry_epoch=7,
            road_history=[{"road": deepcopy(record["road"]), "stamp_s": record["stamp_s"],
                "pose": [0.0, 0.0, 0.0], "odometry_epoch": 7}])
        return record

    def test_fit_preserves_hard_boundaries_and_does_not_mutate_inputs(self):
        record = self.record()
        before = deepcopy(record)
        result = plan(record)
        self.assertEqual(record, before)
        self.assertEqual(result["status"], "VALID")
        self.assertEqual(result["fusion"]["status"], "FUSED")
        self.assertEqual(result["planning_input"]["road"]["left_boundary_xy"], record["road"]["left_boundary_xy"])
        self.assertEqual(result["planning_input"]["road"]["right_boundary_xy"], record["road"]["right_boundary_xy"])
        self.assertEqual(result["planning_input"]["road"]["boundary_source"], record["road"]["boundary_source"])
        self.assertIsNone(result["reference_observed_range_s_m"])
        self.assertFalse(result["fusion"]["reference_is_observed_boundary"])
        timing = result["timing_s"]
        self.assertAlmostEqual(sum(v for k, v in timing.items() if k != "total"), timing["total"], places=6)

    def test_fusion_exception_retries_exact_raw_planner_and_reports_reason(self):
        record = self.record()
        expected = Planner(Settings(**record["settings"]))._plan_geometry(record)
        with patch.dict(ns, _shadow_fuse_road=lambda *_: (_ for _ in ()).throw(RuntimeError("injected fit fault"))):
            result = plan(record)
        self.assertEqual(result["selected"], expected["selected"])
        self.assertEqual(result["status"], "VALID")
        self.assertTrue(result["fusion"]["raw_fallback_used"])
        self.assertEqual(result["fusion"]["reason"], "fusion_error")
        self.assertIn("injected fit fault", result["fusion"]["error"])
        json.dumps(result, allow_nan=False)

    def test_reference_crop_stays_local_across_repeated_forward_motion(self):
        record = self.record()
        record["road"]["centerline_xy"] = [(0., 0.), (1., 0.)]
        record["previous_reference"] = {"points": [(-5., 0.), (0., 0.), (1., 0.)],
            "pose": [0., 0., 0.], "stamp_s": 99.5, "odometry_epoch": 7}
        for step in range(8):
            record.update(stamp_s=100.+step*.2, shadow_pose=[step*.2, 0., 0.])
            record["road"]["measurement_timestamp_s"] = record["stamp_s"]-.05
            record["road_history"] = [{"road": deepcopy(record["road"]),
                "stamp_s": record["stamp_s"], "pose": record["shadow_pose"][:], "odometry_epoch": 7}]
            result = plan(record)
            self.assertEqual(result["status"], "VALID")
            fusion = result["fusion"]
            self.assertEqual(fusion["status"], "FUSED")
            crop, points = fusion["reference_crop"], fusion["guide"]["points"]
            self.assertAlmostEqual(crop["retained_rear_m"], .3, places=6)
            self.assertGreaterEqual(points[0][0], -.300001)
            self.assertAlmostEqual(points[-1][0], 1., places=6)
            self.assertLessEqual(crop["output_length_m"], 1.300001)
            for field in ("left_boundary_xy", "right_boundary_xy", "boundary_source"):
                self.assertEqual(result["planning_input"]["road"][field], record["road"][field])
            if step == 0:
                self.assertAlmostEqual(crop["trimmed_rear_m"], 4.7, places=6)
            record["previous_reference"] = fusion["guide"]

    def test_reference_crop_preserves_future_returning_leg(self):
        record = self.record()
        returning = [(0.,0.),(.25,0.),(.4,.15),(.25,.3),(-.2,.3)]
        record["road"].update(centerline_xy=returning, left_boundary_xy=None, right_boundary_xy=None)
        record["road_history"][0]["road"] = deepcopy(record["road"])
        record["previous_reference"] = {"points": [(-5.,0.)]+returning,
            "pose": [0.,0.,0.], "stamp_s": 99.5, "odometry_epoch": 7}
        result = plan(record)
        self.assertEqual(result["status"], "VALID")
        fusion = result["fusion"]
        self.assertGreater(fusion["reference_crop"]["trimmed_rear_m"], 4.)
        end = fusion["guide"]["points"][-1]
        self.assertLess(end[0], -.15)
        self.assertGreater(end[1], .25)

    def test_reference_crop_keeps_inferred_connection_to_short_far_support(self):
        record = self.record()
        record["road"].update(centerline_xy=[(.6,0.),(.62,0.)],
                              left_boundary_xy=None, right_boundary_xy=None)
        record["road_history"][0]["road"] = deepcopy(record["road"])
        result = plan(record)
        self.assertEqual(result["status"], "VALID")
        self.assertEqual(result["fusion"]["status"], "FUSED")
        self.assertLess(result["fusion"]["reference_crop"]["window_s_m"][0], 0.)
        self.assertAlmostEqual(result["fusion"]["guide"]["points"][0][0], 0., places=6)

    def test_infeasible_or_faulting_fused_plan_cannot_remove_raw_success(self):
        record = self.record()
        original = Planner._plan_geometry
        forced_road = {**record["road"], "injected_fused_road": True}
        for status in ("ALL_COLLIDE", "NO_ROAD", "exception"):
            def calculate(subject, data):
                if data["road"].get("injected_fused_road"):
                    if status == "exception":
                        raise RuntimeError("injected candidate failure")
                    return {"status": status, "selected": None, "candidate_count": 12, "feasible_count": 0}
                return original(subject, data)
            with self.subTest(status=status), patch.dict(ns, _shadow_fuse_road=lambda *_:
                    (forced_road, {"status": "FUSED"})), patch.object(Planner, "_plan_geometry", calculate):
                result = plan(record)
            self.assertEqual(result["status"], "VALID")
            self.assertEqual(result["fusion"]["status"], "RAW_FALLBACK")
            self.assertEqual(result["fusion"]["raw_status"], "VALID")
            self.assertTrue(result["fusion"]["raw_fallback_used"])
            self.assertNotIn("guide", result["fusion"])
            timing = result["timing_s"]
            self.assertAlmostEqual(sum(v for k, v in timing.items() if k != "total"), timing["total"], places=6)
            if status != "exception":
                self.assertEqual(result["fusion"]["failed_fused_attempt"]["feasible_count"], 0)

    def test_alignment_reset_uses_raw_and_serializes_fault_events(self):
        record = self.record()
        record["temporal_collection"] = {"reset": True, "reason": "motion_history_gap",
            "events": [{"id": 1, "stamp_s": 100.0, "reason": "motion_history_gap"}]}
        with patch.dict(ns, _shadow_fuse_road=lambda *_: self.fail("fusion must not use unaligned history")):
            result = plan(record)
        self.assertEqual(result["status"], "VALID")
        self.assertEqual(result["fusion"]["reason"], "motion_alignment_reset")
        self.assertEqual(result["fusion"]["collection"]["events"][0]["reason"], "motion_history_gap")
        self.assertIn("road_history", result["temporal_input"])

    def test_measurement_epoch_dedup_and_original_age_are_retained(self):
        record = self.record()
        record["road"]["measurement_timestamp_s"] = 99.5
        record["road_history"] = [{"road": deepcopy(record["road"]), "stamp_s": 99.6+i*.02,
            "pose": [0.0, 0.0, 0.0], "odometry_epoch": 7} for i in range(21)]
        result = plan(record)
        self.assertEqual(result["fusion"]["history_frames"], 1)
        self.assertEqual(result["fusion"]["measurement_epochs"], 1)
        self.assertAlmostEqual(result["fusion"]["oldest_measurement_age_s"], .5)

    def test_cold_short_boundary_only_and_single_point_keep_raw_availability(self):
        for center in ([(.6,.1)], [], [(0,.1),(.02,.1)]):
            record = self.record()
            record["road"].update(centerline_xy=center, left_boundary_xy=[(.6,.5)],
                                  right_boundary_xy=None, boundary_source={"left": "observed", "right": "absent"})
            record["road_history"][0]["road"] = deepcopy(record["road"])
            result = plan(record)
            self.assertEqual(result["status"], "VALID")
            self.assertEqual(result["candidate_count"], 12)

    def test_self_crossing_guidance_is_repaired_but_real_u_bend_is_retained(self):
        broken = [(0.,0.),(.5,.5),(0.,.5),(.5,0.),(1.,0.)]
        repaired, count = ns["_shadow_erase_reference_loops"](broken)
        self.assertGreater(count, 0)
        self.assertEqual(repaired[0], broken[0])
        self.assertEqual(repaired[-1], broken[-1])
        returning = [(0.,0.),(.5,0.),(.7,.2),(.5,.4),(0.,.4)]
        self.assertEqual(ns["_shadow_erase_reference_loops"](returning), (returning, 0))
        record = self.record()
        record["road"].update(centerline_xy=returning, left_boundary_xy=None, right_boundary_xy=None)
        record["road_history"][0]["road"] = deepcopy(record["road"])
        self.assertEqual(plan(record)["status"], "VALID")

    def test_disabled_and_absent_history_always_publish_explicit_mode(self):
        record = job()
        a = plan(record)
        b = plan(record, fusion_enabled=False)
        self.assertEqual(a["selected"], b["selected"])
        self.assertEqual(a["fusion"]["reason"], "history_unavailable")
        self.assertEqual(b["fusion"]["status"], "DISABLED")
        self.assertFalse(b["fusion"]["degraded"])

    def test_actual_worker_carries_temporal_input_and_diagnostics(self):
        record = self.record()
        process = subprocess.run([sys.executable, str(source), "--shadow-worker"],
            input=json.dumps(record)+"\n", capture_output=True, text=True, timeout=15)
        self.assertEqual(process.returncode, 0, process.stderr)
        result = json.loads(process.stdout)
        self.assertEqual(result["status"], "VALID")
        self.assertEqual(result["fusion"]["status"], "FUSED")
        self.assertEqual(len(result["temporal_input"]["road_history"]), 1)
        self.assertEqual(result["generation"], record["generation"])

    def test_malformed_history_does_not_poison_raw_planning(self):
        record = self.record()
        record["road_history"][0]["pose"] = [0.0, 0.0]
        result = plan(record)
        self.assertEqual(result["status"], "VALID")
        self.assertEqual(result["fusion"]["reason"], "no_aligned_history")
        self.assertTrue(result["fusion"]["raw_fallback_used"])
        json.dumps(result, allow_nan=False)

    def test_fusion_settings_bound_memory_without_road_admission_thresholds(self):
        for options in ({"fusion_enabled": 1}, {"fusion_max_samples": 0}, {"fusion_max_samples": 129},
                        {"fusion_max_samples": 2.0}, {"fusion_window_s": 0}, {"fusion_spacing_m": 0},
                        {"fusion_rear_support_m": -0.1}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                Settings(**options)

    def test_recorded_20_inputs_preserve_success_and_improve_aggregate_continuity(self):
        root = Path(__file__).resolve().parents[1]
        replay = runpy.run_path(str(root/"offline/shadow_lattice/replay_temporal.py"))
        fixture = json.loads((root/"tests/fixtures/shadow_temporal_recorded_run.json").read_text())
        results = replay["evaluate"](ns, fixture)
        before, after = (replay["metrics"](ns, results, mode)["summary"] for mode in ("raw", "fused"))
        self.assertEqual((before["valid"], after["valid"]), (20, 20))
        self.assertLess(after["continuity_rms_m"]["mean"], before["continuity_rms_m"]["mean"])
        self.assertLess(after["shape_rms_m"]["mean"], before["shape_rms_m"]["mean"])
        self.assertTrue(all(not r["fused"]["fusion"]["raw_fallback_used"] for r in results))

    def test_all_20_recorded_inputs_keep_original_paths_under_fusion_failure(self):
        fixture = json.loads((Path(__file__).parent/"fixtures/shadow_temporal_recorded_run.json").read_text())
        def fail(*_):
            raise RuntimeError("injected fusion failure across recorded inputs")
        with patch.dict(ns, _shadow_fuse_road=fail):
            for frame in fixture["frames"]:
                result = plan(frame["job"])
                self.assertEqual(result["status"], "VALID")
                self.assertTrue(result["fusion"]["raw_fallback_used"])
                self.assertEqual(result["selected"]["id"], frame["recorded_selected"]["id"])
                self.assertLess(max(abs(a-b) for p,q in zip(result["selected"]["path"],
                    frame["recorded_selected"]["path"]) for a,b in zip(p,q)), 1e-12)


def bump_job():
    """Straight road whose left boundary juts in to y=0.10 at x 0.60-0.70.

    The straight "follow" candidate's body first touches it at about 0.35 m of
    path: past the 0.22 m "travel" check, inside the execution window.
    """
    record = job()
    record["road"].update(centerline_xy=[(0.0, 0.0), (0.75, 0.0), (1.5, 0.0)],
        left_boundary_xy=[(0.0, 0.5), (0.55, 0.5), (0.6, 0.1), (0.7, 0.1), (0.75, 0.5), (1.5, 0.5)],
        right_boundary_xy=[(0.0, -0.5), (1.5, -0.5)])
    return record


class CheckLengthModeChecks(unittest.TestCase):
    """mpc-car-run, 2026-10-20: shadow.check_length_mode (docs/LATTICE_CHECK_LENGTH_PLAN_CN.md)."""

    def test_default_travel_is_mvp21_speed_times_hold(self):
        self.assertEqual(Settings().check_length_mode, "travel")
        self.assertEqual(ns["EXECUTION_TAIL_M"], 0.15)
        record = job(speed=0.2)
        default = plan(record)
        self.assertEqual(default["check_length_m"], 0.2*(1/1.0+0.10))
        self.assertAlmostEqual(default["check_length_m"], 0.22)
        self.assertEqual((default["check_length_mode"], default["travel_check_m"]),
                         ("travel", default["check_length_m"]))
        self.assertAlmostEqual(default["execution_window_m"], 0.50)
        explicit = plan(record, check_length_mode="travel")
        for key in ("check_length_m", "score_length_m", "generated_length_m", "selected", "candidates"):
            self.assertEqual(explicit[key], default[key], key)

    def test_mode_lengths_including_standstill_and_missing_preview(self):
        expected = {0.2: (0.22, 0.50, 0.72), 0.0: (0.0, 0.50, 0.50)}
        for speed, lengths in expected.items():
            for mode, length in zip(("travel", "max", "travel_plus_window"), lengths):
                with self.subTest(speed=speed, mode=mode):
                    result = plan(job(speed=speed), check_length_mode=mode)
                    self.assertAlmostEqual(result["check_length_m"], length)
                    self.assertEqual(result["status"], "VALID")
                    self.assertGreaterEqual(result["generated_length_m"], result["control_read_length_m"])
        # Invalid preview falls back to score_min_length_m (0.2), as the horizon already did.
        for preview in (None, 0.0, -1.0, float("nan"), "0.35"):
            record = job(speed=0.2)
            record["preview_m"] = preview
            for mode, length in (("travel", 0.22), ("max", 0.35), ("travel_plus_window", 0.57)):
                with self.subTest(preview=preview, mode=mode):
                    result = plan(record, check_length_mode=mode)
                    self.assertAlmostEqual(result["check_length_m"], length)
                    self.assertAlmostEqual(result["execution_window_m"], 0.35)
        record = job(speed=0.2)
        del record["preview_m"]
        self.assertAlmostEqual(plan(record, check_length_mode="travel_plus_window")["check_length_m"], 0.57)

    def test_window_modes_reject_a_candidate_that_hits_beyond_travel(self):
        record = bump_job()
        travel = plan(record)
        window = plan(record, check_length_mode="travel_plus_window")
        self.assertEqual((travel["status"], window["status"]), ("VALID", "VALID"))
        self.assertEqual(travel["candidates"][0]["family"], "follow")
        self.assertIsNone(travel["candidates"][0]["collision"])
        self.assertEqual(travel["selected"]["family"], "follow")
        hit = window["candidates"][0]["collision"]
        self.assertEqual(hit["side"], "left")
        self.assertTrue(0.3 <= hit["pose"][0] <= 0.6, hit)
        self.assertLess(window["feasible_count"], travel["feasible_count"])
        self.assertNotEqual(window["selected"]["family"], "follow")

    def test_travel_plus_window_path_passes_execution_while_in_use(self):
        record = bump_job()
        selected_family = {}
        for mode in ("travel", "travel_plus_window"):
            result = plan(record, check_length_mode=mode)
            result.update(job_id=1, execution_anchor={"pose": [0.0, 0.0, 0.0], "odometry_epoch": 1})
            selected_family[mode] = result["selected"]["family"]
            path = result["selected"]["path"]
            arc = ns["_shadow_lengths"]([p[:2] for p in path])
            cfg = Settings(**{**record["settings"], "check_length_mode": mode})
            manager = ns["PlannerExecutionManager"]()
            manager.accept(result)
            travelled = result["travel_check_m"]
            for tick, distance in enumerate((0.0, travelled/2, travelled), 1):
                # The car on the planned path; the same boundaries in its current frame.
                pose = path[min(range(len(arc)), key=lambda i: abs(arc[i]-distance))]
                stamp = record["stamp_s"]+0.05*tick
                road = {"timestamp_s": stamp, "frame_id": "base_link", "time_aligned": True,
                        "boundary_source": record["road"]["boundary_source"]}
                for side in ("left", "right"):
                    road[side+"_boundary_xy"] = ns["_shadow_pose_points"](
                        record["road"][side+"_boundary_xy"], (0.0, 0.0, 0.0), pose)
                state = {"timestamp_s": stamp, "frame_id": "base_link", "speed_valid": True,
                         "yaw_rate_valid": True, "speed_mps": 0.2, "yaw_rate_rps": 0.0}
                reference = manager.select(road, state, pose, 1, cfg, record["preview_m"], 0.2, 0.05, 0.25)
                with self.subTest(mode=mode, distance=distance):
                    if mode == "travel_plus_window":
                        self.assertIsNotNone(reference, manager.diagnostics.get("reason"))
                        self.assertEqual(manager.diagnostics["reason"], "planner_tracking")
                    else:       # mvp2.1's check: the execution check rejects the planned path
                        self.assertIsNone(reference)
                        self.assertEqual(manager.diagnostics["reason"], "known_boundary_collision")
        self.assertEqual(selected_family["travel"], "follow")

    def test_mode_must_be_one_of_the_three_strings(self):
        for mode in ("travel", "max", "travel_plus_window"):
            self.assertEqual(Settings(check_length_mode=mode).check_length_mode, mode)
        for mode in ("", "Travel", "window", "travel+window", None, 0, 0.5, True, ["travel"]):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                Settings(check_length_mode=mode)

    def test_worker_and_summary_carry_the_mode(self):
        record = job()
        record["settings"] = vars(Settings(sample_count=12, check_length_mode="travel_plus_window"))
        process = subprocess.run([sys.executable, str(source), "--shadow-worker"],
            input=json.dumps(record)+"\n", text=True, capture_output=True, timeout=15)
        self.assertEqual(process.returncode, 0, process.stderr)
        response = json.loads(process.stdout)
        self.assertEqual(response["status"], "VALID", response.get("error"))
        self.assertEqual(response["check_length_mode"], "travel_plus_window")
        self.assertAlmostEqual(response["check_length_m"], 0.72)
        self.assertIn('"check_length_mode"', source.read_text(encoding="utf-8").split(
            '"speed_mps", "hold_time_s", "check_length_m",', 1)[1].split(")}", 1)[0])


def far_bump_job():
    """Straight road whose left boundary juts in to y=0.10 at x 1.10-1.20: beyond the
    0.72 m travel_plus_window check (+0.25 m body front), inside the 1.07 m path."""
    record = job()
    record["road"].update(centerline_xy=[(0.0, 0.0), (0.8, 0.0), (1.6, 0.0)],
        left_boundary_xy=[(0.0, 0.5), (1.05, 0.5), (1.1, 0.1), (1.2, 0.1), (1.25, 0.5), (1.6, 0.5)],
        right_boundary_xy=[(0.0, -0.5), (1.6, -0.5)])
    return record


class CoverageChecks(unittest.TestCase):
    """mpc-car-run, 2026-10-21: shadow.collision_coverage / score_coverage (sim-branch 8f1f01d)."""

    def test_default_prefix_is_unchanged(self):
        self.assertEqual((Settings().collision_coverage, Settings().score_coverage), ("prefix", "prefix"))
        for mode in ("travel", "travel_plus_window"):
            record = job(speed=0.2)
            default = plan(record, check_length_mode=mode)
            explicit = plan(record, check_length_mode=mode, collision_coverage="prefix", score_coverage="prefix")
            for key in ("check_length_m", "score_length_m", "generated_length_m", "control_read_length_m",
                        "selected", "candidates"):
                self.assertEqual(explicit[key], default[key], (mode, key))
            self.assertEqual((default["collision_coverage"], default["score_coverage"]), ("prefix", "prefix"))

    def test_generated_scores_and_checks_the_whole_path(self):
        record = job(speed=0.2)
        prefix = plan(record, check_length_mode="travel_plus_window")
        self.assertAlmostEqual(prefix["score_length_m"], 0.72)
        self.assertAlmostEqual(prefix["generated_length_m"], 1.07)
        score = plan(record, check_length_mode="travel_plus_window", score_coverage="generated")
        self.assertEqual(score["score_length_m"], score["generated_length_m"])
        self.assertEqual(score["check_length_m"], prefix["check_length_m"])
        self.assertEqual(score["control_read_length_m"], prefix["control_read_length_m"])
        for candidate in score["candidates"]:
            self.assertGreater(candidate["scoring_samples"][-1][0], 1.0)
        for candidate in prefix["candidates"]:
            self.assertLessEqual(candidate["scoring_samples"][-1][0], 0.72+1e-9)
        collision = plan(record, check_length_mode="travel_plus_window", collision_coverage="generated")
        self.assertEqual(collision["check_length_m"], collision["generated_length_m"])
        self.assertEqual(collision["score_length_m"], prefix["score_length_m"])
        # The controller still reads check+preview of the prefix-mode check.
        self.assertEqual(collision["control_read_length_m"], prefix["control_read_length_m"])

    def test_generated_collision_rejects_a_far_bump_prefix_does_not_see(self):
        record = far_bump_job()
        prefix = plan(record, check_length_mode="travel_plus_window", sample_count=48)
        generated = plan(record, check_length_mode="travel_plus_window", sample_count=48,
                         collision_coverage="generated")
        free = {c["id"] for c in prefix["candidates"] if c["collision"] is None}
        free_generated = {c["id"] for c in generated["candidates"] if c["collision"] is None}
        self.assertLess(len(free_generated), len(free))
        self.assertLessEqual(free_generated, free)      # a longer check only removes candidates
        self.assertNotIn(generated["selected"]["id"], free-free_generated)

    def test_values_must_be_prefix_or_generated(self):
        for name in ("collision_coverage", "score_coverage"):
            for value in ("prefix", "generated"):
                self.assertEqual(getattr(Settings(**{name: value}), name), value)
            for value in ("", "Prefix", "full", None, 0, True, ["prefix"]):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    Settings(**{name: value})

    def test_worker_and_summary_carry_the_coverage(self):
        record = job()
        record["settings"] = vars(Settings(sample_count=12, check_length_mode="travel_plus_window",
                                           score_coverage="generated"))
        process = subprocess.run([sys.executable, str(source), "--shadow-worker"],
            input=json.dumps(record)+"\n", text=True, capture_output=True, timeout=15)
        self.assertEqual(process.returncode, 0, process.stderr)
        response = json.loads(process.stdout)
        self.assertEqual(response["status"], "VALID", response.get("error"))
        self.assertEqual((response["collision_coverage"], response["score_coverage"]), ("prefix", "generated"))
        self.assertAlmostEqual(response["score_length_m"], response["generated_length_m"])
        summary_keys = source.read_text(encoding="utf-8").split(
            '"speed_mps", "hold_time_s", "check_length_m",', 1)[1].split(")}", 1)[0]
        self.assertIn('"collision_coverage", "score_coverage"', summary_keys)


def legacy_shadow_at(points, lengths, s):
    """_shadow_at before 2026-10-21 (linear search), kept as the reference."""
    if len(points) == 1:
        return points[0][0]+s, points[0][1]
    if s <= 0:
        i = 0
    elif s >= lengths[-1]:
        i = len(points)-2
    else:
        i = next(i for i in range(len(points)-1) if s <= lengths[i+1])
    u = (s-lengths[i])/(lengths[i+1]-lengths[i])
    return tuple(points[i][k]+u*(points[i+1][k]-points[i][k]) for k in (0, 1))


class LookupOptimisationChecks(unittest.TestCase):
    """mpc-car-run, 2026-10-21: binary arc lookup and per-job knot cache (sim-branch 92ef303).

    Whole-planner equivalence (28 jobs x 3 check_length_mode values against the
    bb0bef2 source) is recorded in docs/MPC_PROGRESS_LOG.md 2026-10-21.
    """

    def test_arc_lookup_matches_legacy_at_knots_neighbours_and_tails(self):
        for points in ([(.6, .03)], [(0., 0.), (1., 0.)], [(0., 0.), (.3, .2), (.7, .5), (1., .4)],
                       [(0., 0.), (0., .5), (-.3, .5), (-.3, -.3)]):
            lengths = ns["_shadow_lengths"](points)
            stations = [-.4, 0., .04, lengths[-1]+.4]
            for knot in lengths:
                stations.extend((math.nextafter(knot, -math.inf), knot, math.nextafter(knot, math.inf)))
            for station in stations:
                with self.subTest(points=points, station=station):
                    self.assertEqual(ns["_shadow_at"](points, lengths, station),
                                     legacy_shadow_at(points, lengths, station))

    def test_worker_plans_a_curve_with_the_binary_lookup(self):
        record = job()
        record["road"].update(centerline_xy=[(0.0, 0.0), (0.5, 0.05), (1.0, 0.2), (1.4, 0.45)],
                              left_boundary_xy=[(0.0, 0.5), (0.5, 0.55), (1.0, 0.7), (1.3, 0.95)],
                              right_boundary_xy=[(0.0, -0.5), (0.5, -0.45), (1.0, -0.3), (1.5, -0.05)])
        record["settings"] = vars(Settings(sample_count=48, check_length_mode="travel_plus_window"))
        process = subprocess.run([sys.executable, str(source), "--shadow-worker"],
            input=json.dumps(record)+"\n", text=True, capture_output=True, timeout=15)
        self.assertEqual(process.returncode, 0, process.stderr)
        response = json.loads(process.stdout)
        self.assertNotEqual(response["status"], "WORKER_ERROR", response.get("error"))
        expected = plan(record, check_length_mode="travel_plus_window", sample_count=48)
        self.assertEqual(response["status"], expected["status"])
        self.assertEqual(response["selected"]["id"], expected["selected"]["id"])


def path_turn(path):
    """Heading change from the first to the last segment, rad (left positive)."""
    a = math.atan2(path[1][1]-path[0][1], path[1][0]-path[0][0])
    b = math.atan2(path[-1][1]-path[-2][1], path[-1][0]-path[-2][0])
    return math.atan2(math.sin(b-a), math.cos(b-a))


class FusionPriorGuideChecks(unittest.TestCase):
    """fusion-prior-reset (2026-10-28): car 01 left U-turns seen right-only.

    Each cycle's fusion guide was the previous reference; no new point joined it ahead,
    so it shrank (2.0 -> 0.55 m) and kept its old direction, which pointed right once the
    car had turned. These four recorded plans turned the car right; without the prior
    guide the same inputs plan left.
    """
    fixture = json.loads((Path(__file__).parent/"fixtures/car01_20261028_uturn_prior_guide.json").read_text())

    def run_frame(self, frame, **overrides):
        settings = {k: v for k, v in frame["job"]["settings"].items() if k in vars(Settings())}
        return Planner(Settings(**{**settings, **overrides})).plan(frame["job"])

    def test_default_reproduces_the_recorded_right_turns(self):
        for frame in self.fixture["frames"]:
            with self.subTest(source=frame["source"]):
                result = self.run_frame(frame)
                self.assertEqual(result["status"], "VALID")
                self.assertEqual(result["selected"]["id"], frame["recorded_selected"]["id"])
                self.assertLess(path_turn(result["selected"]["path"]), -0.5)

    def test_without_prior_guide_the_same_inputs_plan_left(self):
        for frame in self.fixture["frames"]:
            with self.subTest(source=frame["source"]):
                result = self.run_frame(frame, fusion_use_prior=False)
                self.assertEqual(result["status"], "VALID")
                self.assertEqual(result["fusion"]["status"], "FUSED")
                self.assertFalse(result["fusion"]["prior_used"])
                self.assertGreater(path_turn(result["selected"]["path"]), 0.2)

    def test_fusion_use_prior_must_be_boolean(self):
        with self.assertRaises(ValueError):
            Settings(fusion_use_prior=0)


if __name__ == "__main__":
    unittest.main()
