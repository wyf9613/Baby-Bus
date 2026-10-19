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
# mpc-car-run adaptation: this branch's test_estimation does not extract the
# ordered_xy road helpers (mvp2.1's does); add the ones it lacks.
definitions += [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name not in ns
                and (n.name.startswith("_path_") or n.name == "_estimation_order_boundary")]
ns["time"] = time
ns.update(queue=queue, threading=threading, subprocess=subprocess, sys=sys, json=json)
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(source), "exec"), ns)
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

    def test_stage_timing_accounts_for_total_including_no_road(self):
        for record in (job(), {**job(), "road": {}}):
            result = plan(record)
            timing = result["timing_s"]
            self.assertEqual(timing["total"], result["elapsed_s"])
            self.assertAlmostEqual(sum(v for k, v in timing.items() if k != "total"), timing["total"], places=8)
            self.assertTrue(all(v >= 0 for v in timing.values()))

    def test_real_car15_continuous_curve_masks_each_side_before_reconstruction(self):
        root = Path(__file__).resolve().parents[1]
        # Local port: use the user's actual estimator settings and the exact
        # original seed + six masked frames, stored as a bounded test fixture.
        import yaml
        config = yaml.safe_load((root/"config/ai4r_policy_speed_feedback.yaml").read_text(encoding="utf-8"))[
            "/**/ai4r_policy"]["ros__parameters"]
        frames = json.loads((root/"tests/fixtures/car15_shadow_single_side_frames.json").read_text(encoding="utf-8"))
        for side, colour in (("left", 1), ("right", 2)):
            pipeline = ns["EstimationPipeline"](ns["EstimationSettings"](**config["estimation"]))
            seed = frames[0]
            pipeline._history_road({"detections": seed["positions"]}, round(seed["stamp_s"]*1e9), 0, seed["stamp_s"])
            self.assertIsNotNone(pipeline._learned_lane_width_m)
            for frame in frames[1:]:
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


if __name__ == "__main__":
    unittest.main()
