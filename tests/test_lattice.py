"""Fresh shadow definitions from actual selected policy source, without ROS.

Includes real subprocess protocol and public run-loop isolation tests; no
archived lattice imports, vehicle nodes or actuators are involved.
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
from mvp21_policy import lattice,transport
ns.update(vars(lattice));ns.update(vars(transport))
tree=ast.parse(source.read_text(encoding='utf-8'))
worker_source=Path(__file__).resolve().parents[1]/'scripts/mvp21_policy_node.py'
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
        import yaml
        root = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((root/"config/ai4r_policy.yaml").read_text(encoding="utf-8"))["/**/ai4r_policy"]["ros__parameters"]
        frames=json.loads((root/'tests/fixtures/car15_shadow_curve_sequence.json').read_text())
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
        process = subprocess.run([sys.executable, str(worker_source), "--shadow-worker"],
            input=json.dumps(record)+"\n", capture_output=True, text=True, timeout=15)
        self.assertEqual(process.returncode, 0, process.stderr)
        result = json.loads(process.stdout)
        self.assertEqual(result["status"], "VALID")
        self.assertEqual(result["job_id"], 1)
        self.assertEqual(result["generation"], 1)

    def test_real_background_transport_does_not_wait_on_submit_and_closes(self):
        worker = ns["ShadowWorker"]()
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


control = runpy.run_path(str(Path(__file__).with_name("test_control.py")))
node_definition = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "PolicyNode")
methods = [n for n in node_definition.body if isinstance(n, ast.FunctionDef)
           and n.name in ("_shadow_offer", "_shadow_poll", "_shadow_clear", "stop_before_shutdown")]
method_ns = {**ns, "json": json, "String": SimpleNamespace, "FSM_STATE_PUBLISHING_POLICY_ACTION": 3,
             "FSM_STATE_NOT_PUBLISHING_ACTIONS": 1}
exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), method_ns)
method_ns["_shadow_offer"]=transport.ShadowJobSource._shadow_offer


class ShadowIntegrationChecks(unittest.TestCase):
    def node(self):
        node = control["make_node"](simulated=False, mvp=True)
        node.shadow_settings = Settings(enabled=True)
        node.shadow_generation = 2
        node.policy_update_rate_hz = 20.0
        node.shadow_job_id = 0
        node.shadow_last_submit_s = -math.inf
        node.shadow_handoff_estimate_s = node.shadow_settings.handoff_allowance_s
        node.shadow_worker = SimpleNamespace(submit=lambda job: None, poll=lambda: None)
        node.shadow_displays = []
        node._shadow_publish_path = lambda pub, points, stamp: node.shadow_displays.append((pub, points, stamp))
        for name in ("path", "prefix", "reference", "diagnostics"):
            setattr(node, "shadow_"+name+"_publisher", control["Recorder"]())
        for name in ("_shadow_offer", "_shadow_poll", "_shadow_clear", "stop_before_shutdown"):
            setattr(node, name, method_ns[name].__get__(node))
        return node

    def baseline_state(self, node):
        def values(value):
            if isinstance(value, dict):
                return {k: values(v) for k, v in value.items()}
            if isinstance(value, (tuple, list)):
                return [values(v) for v in value]
            if hasattr(value, "__dict__"):
                return values(vars(value))
            return value
        return deepcopy({name: getattr(node, name) for name in (
            "fsm_state", "state_reason", "estimation_output", "planning_output",
            "planning_diagnostics", "control_diagnostics", "last_control_reference_deadline_s",
            "control_has_run", "_trajectory_reference")}) | {
                "controller": values(node.controller),
                "distance": values(node.distance_limiter),
                "actions": [vars(msg).copy() for msg in node.action_publisher.messages]}

    def configure_shipped_baseline(self, node):
        import yaml
        cfg = yaml.safe_load((source.parents[1]/"config/ai4r_policy.yaml").read_text(
            encoding="utf-8"))["/**/ai4r_policy"]["ros__parameters"]
        node.estimation_settings = ns["EstimationSettings"](**cfg["estimation"])
        node.motion_history = ns["EstimationMotionHistory"](node.estimation_settings)
        node.planning_settings = ns["PlanningSettings"](**cfg["planning"])
        node.control_settings = ns["ControlSettings"](**cfg["control"])
        node.controller = ns["PolicyController"](node.control_settings)
        node.distance_limiter = ns["RunDistanceLimiter"](
            node.control_settings.max_distance_m, node.control_settings.max_dt_s)
        node.sensor_timeout_s.update(cfg["sensor_timeout_s"])

    def feed_shipped_baseline(self, node, center=0.0, speed=0.2, yaw=0.0):
        now, mono = node.test_clock.ros_ns, node.test_clock.monotonic
        node._store("wheel_speed", speed, None, mono, now)
        node._store("imu_angular_velocity", (0.0, 0.0, yaw), now, mono, now)
        # Ordered-road fixture: observed support ahead, with no reversing rear chain.
        xs = [0.15+0.15*i for i in range(15)]
        batch = api["cones"]([(x, center+0.35) for x in xs], [(x, center-0.35) for x in xs])
        batch["detections"] = [(x, y, z, node.estimation_settings.left_colour if y > center
            else node.estimation_settings.right_colour, confidence)
            for x, y, z, _, confidence in batch["detections"]]
        node._store("cone_detections", batch, node.test_clock.ros_ns,
                    node.test_clock.monotonic, node.test_clock.ros_ns)
        node.last_nonempty_cones = node.observations["cone_detections"]

    def test_faulting_shadow_offer_cannot_change_baseline_action_or_fsm(self):
        baseline = control["make_node"](simulated=False, mvp=True)
        shadow = self.node()
        def fail():
            raise RuntimeError("shadow injected fault")
        shadow._shadow_offer = fail
        for node in (baseline, shadow):
            control["feed"](node)
            control["start"](node)
            node.run_policy_step()
        self.assertEqual(baseline.fsm_state, 3)
        self.assertEqual(shadow.fsm_state, baseline.fsm_state)
        self.assertEqual(vars(shadow.action_publisher.messages[-1]), vars(baseline.action_publisher.messages[-1]))

    def test_scheduler_uses_latest_road_without_touching_real_controller(self):
        node = self.node()
        record = job()
        node.estimation_output = {"road": record["road"], "state": record["state"]}
        submitted = []
        node.shadow_worker.submit = submitted.append
        node._shadow_offer()
        node.test_clock.advance(0.05)
        node._shadow_offer()
        self.assertEqual(len(submitted), 1)
        node.test_clock.advance(1.0)
        node.estimation_output["state"]["speed_mps"] = 0.3
        node._shadow_offer()
        self.assertEqual(len(submitted), 2)
        self.assertEqual(submitted[-1]["state"]["speed_mps"], 0.3)
        self.assertEqual(node.controller._mvp_previous_steer, 0.0)
        node.estimation_output["road"]["centerline_xy"][0] = (9.0, 9.0)
        self.assertEqual(submitted[-1]["road"]["centerline_xy"][0], (0.0, 0.0))

    def test_stop_generation_fences_old_output_and_all_collision_clears_selected_line(self):
        node = self.node()
        result = plan(job())
        result["generation"] = 1
        node.fsm_state = 3
        node.shadow_worker.poll = lambda: result
        node._shadow_poll()
        self.assertEqual(node.shadow_displays, [])
        result["generation"] = 2
        result["status"] = "ALL_COLLIDE"
        node._shadow_poll()
        self.assertEqual(node.shadow_displays[0][1], [])
        self.assertEqual(node.shadow_displays[1][1], [])
        count = len(node.shadow_displays)
        node.fsm_state = 2
        result["status"] = "VALID"
        node._shadow_poll()
        self.assertEqual(len(node.shadow_displays), count)

    def test_shadow_results_never_mutate_live_reference_controller_fsm_or_actions(self):
        node = self.node()
        control["feed"](node)
        control["start"](node)
        node.run_policy_step()
        before = self.baseline_state(node)
        result = plan(job())
        result["generation"] = node.shadow_generation
        result["selected"]["path"] = [(0.0, 0.0, 0.0), (0.01, 10.0, 3.0)]
        for status in ("VALID", "NO_ROAD", "ALL_COLLIDE", "WORKER_ERROR"):
            with self.subTest(status=status):
                result["status"] = status
                node.shadow_worker.poll = lambda: result
                node._shadow_poll()
                self.assertEqual(self.baseline_state(node), before)
        # A malformed diagnostic also cannot corrupt the active controller.
        node.shadow_worker.poll = lambda: {"generation": node.shadow_generation,
            "status": "VALID", "selected": {"path": None}}
        node._shadow_poll()
        self.assertEqual(self.baseline_state(node), before)

    def test_shipped_baseline_commands_match_with_shadow_results_across_40_ticks(self):
        baseline, shadow = control["make_node"](simulated=False, mvp=True), self.node()
        for node in (baseline, shadow):
            self.configure_shipped_baseline(node)
            self.feed_shipped_baseline(node)
            control["start"](node)
        result = plan(job())
        for tick in range(40):
            for node in (baseline, shadow):
                node.test_clock.advance(0.05)
                self.feed_shipped_baseline(node, center=0.12*math.sin(tick*0.3),
                    speed=0.15+0.03*math.cos(tick*0.2), yaw=0.1*math.sin(tick*0.2))
                node.supervision_callback()
                node.run_policy_step()
            result["generation"] = shadow.shadow_generation
            result["status"] = ("VALID", "ALL_COLLIDE", "NO_ROAD", "WORKER_ERROR")[tick % 4]
            shadow.shadow_worker.poll = lambda: result
            shadow._shadow_poll()
            self.assertEqual(shadow.fsm_state, 3, shadow.state_reason)
            self.assertEqual(self.baseline_state(shadow), self.baseline_state(baseline))
        # Independent watchdog must still stop even if shadow says VALID.
        for node in (baseline, shadow):
            node.test_clock.advance(0.5)
            node.supervision_callback()
        result["status"] = "VALID"
        shadow._shadow_poll()
        self.assertEqual(shadow.fsm_state, 2)
        self.assertEqual(self.baseline_state(shadow), self.baseline_state(baseline))

    def test_transport_failure_restarts_only_shadow_worker(self):
        node = self.node()
        control["feed"](node)
        control["start"](node)
        node.run_policy_step()
        before = self.baseline_state(node)
        closed = []
        node.shadow_worker.close = lambda: closed.append(True)
        node.shadow_worker.poll = lambda: {"status": "TRANSPORT_ERROR", "error": "injected crash"}
        replacement = SimpleNamespace(poll=lambda: None, submit=lambda _: None)
        with patch.dict(method_ns, ShadowWorker=lambda: replacement, __file__=str(source)):
            node._shadow_poll()
        self.assertEqual(closed, [True])
        self.assertIs(node.shadow_worker, replacement)
        self.assertEqual(self.baseline_state(node), before)

    def test_operator_and_watchdog_zero_precede_shadow_display_clearing(self):
        for stop_source in ("operator", "watchdog"):
            with self.subTest(stop_source=stop_source):
                node = self.node()
                control["feed"](node)
                control["start"](node)
                node.run_policy_step()
                self.assertGreater(node.action_publisher.messages[-1].drive, 0)
                cleared_after_zero = []
                def clear():
                    command = node.action_publisher.messages[-1]
                    cleared_after_zero.append((node.fsm_state, command.drive, command.steer))
                node._shadow_clear = clear
                if stop_source == "operator":
                    node.fsm_transition_request_callback(SimpleNamespace(data=2))
                else:
                    node.test_clock.advance(1.0)
                    node.supervision_callback()
                self.assertEqual(cleared_after_zero, [(2, 0.0, 0.0)])

    def test_shutdown_zero_precedes_worker_close(self):
        node = self.node()
        control["feed"](node)
        control["start"](node)
        node.run_policy_step()
        node.context = SimpleNamespace(ok=lambda: True)
        closed_after_zero = []
        node.shadow_worker.close = lambda: closed_after_zero.append(
            vars(node.action_publisher.messages[-1]).copy())
        node.stop_before_shutdown()
        self.assertEqual(len(closed_after_zero), 1)
        self.assertEqual((closed_after_zero[0]["drive"], closed_after_zero[0]["steer"]), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
