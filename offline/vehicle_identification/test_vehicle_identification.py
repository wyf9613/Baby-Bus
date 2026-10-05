"""Hardware-free tests of the actual policy scheduler and offline measurements."""

import csv
import ast
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import traceback
from types import SimpleNamespace
import unittest
import uuid

from record_vehicle_test import clean
from vehicle_test_tools import (equivalent_angle, export_log, geometry, load_experiment,
                                save_json, steering_dynamics, steering_fit)


Experiment, DEFAULTS = load_experiment()


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def policy_harness():
    """Actual node methods with only transport/clock replaced; NOT a ROS graph test."""
    source = Path(__file__).resolve().parents[2] / "scripts/policy_node.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    selected = {"_age", "_fresh", "health_problem", "run_policy_step", "calculate_policy_actions",
                "fsm_transition_request_callback", "_change_state", "supervision_callback",
                "publish_zero_actions", "_publish_actions"}
    constants = {"SENSORS", "STATE_NAMES", "FSM_STATE_NOT_PUBLISHING_ACTIONS",
                 "FSM_STATE_PUBLISHING_ZERO_ACTIONS", "FSM_STATE_PUBLISHING_POLICY_ACTION"}
    nodes = [n for n in tree.body if isinstance(n, ast.Assign) and any(
        isinstance(t, ast.Name) and t.id in constants for t in n.targets)]
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "PolicyNode")
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in selected]
    nodes.append(cls)
    message = type("Message", (SimpleNamespace,), {"UNITS_NORMALIZED": 1})
    scope = {**Experiment.__init__.__globals__, "deepcopy": deepcopy, "json": json,
             "uuid": uuid, "traceback": traceback, "String": message,
             "Float32": message, "DriveAndSteer": message}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), scope)
    node = scope["PolicyNode"]()
    node.now = 10.0
    node._times = lambda: (node.now, round(node.now * 1e9))
    node._monotonic = lambda: node.now
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(node.now * 1e9)))
    node.errors = []
    node.get_logger = lambda: SimpleNamespace(info=lambda text: None, warning=node.errors.append,
                                             error=node.errors.append)
    node._warn = lambda key, text: node.errors.append(text)
    node.publish_state = lambda: None
    node._last_status_at, node.status_period_s = 0, 0.5
    node.identification_config = config()
    node.identification = Experiment(node.identification_config)
    node.fsm_state = 2
    node.required_sensors = ["wheel_speed"]
    node.sensor_timeout_s = {name: 0.5 for name in scope["SENSORS"]}
    node.observations = {name: None for name in scope["SENSORS"]}
    for name in ("action", "pan", "debug1", "debug2", "identification"):
        setattr(node, name + "_publisher", Publisher())
    return node


def harness_tick(node, advance=0, speed=0.0, fresh=True):
    node.now += advance
    if fresh:
        node.observations["wheel_speed"] = SimpleNamespace(value=speed, received_at=node.now, stamp_ns=None)
    node.run_policy_step()


def config(**overrides):
    return {**DEFAULTS, "mode": "drive", "confirmed": True, "durations_s": [0.5, 0.5],
            "drive_values": [0.1, 0.0], "steering_values": [0.0, 0.0],
            "drive_max": 0.2, "speed_limit_mps": 0.5, "settle_s": 0.5,
            "max_dt_s": 0.15, "max_run_s": 5.0, **overrides}


class SequenceTests(unittest.TestCase):
    def test_shipped_defaults_never_request_motion(self):
        experiment = Experiment(DEFAULTS)
        for time in (0, 1, 100):
            step = experiment.step(time, 0, None)
            self.assertEqual((step["drive"], step["steer"]), (0, 0))

    def test_sequence_is_timed_and_finishes_latched(self):
        experiment = Experiment(config())
        outputs = [experiment.step(tick / 20, 0.05 if tick else 0, 0.0) for tick in range(41)]
        self.assertEqual(outputs[9]["phase"], "settle")
        self.assertEqual(outputs[10]["drive"], 0.1)
        self.assertEqual(outputs[19]["drive"], 0.1)
        self.assertEqual(outputs[20]["drive"], 0.0)
        self.assertEqual(outputs[30]["phase"], "tail")
        self.assertTrue(outputs[40]["terminal"])
        self.assertEqual(experiment.step(0, 0, 0)["drive"], 0)
        self.assertTrue(experiment.step(0, 0, 0)["terminal"])

    def test_invalid_configurations_fail_closed(self):
        cases = [dict(confirmed=False), dict(mode="typo"), dict(durations_s=[]),
                 dict(drive_values=[0.1]), dict(durations_s=[0.1, 0.5]),
                 dict(drive_max=0.05), dict(steering_values=[0.2, 0]),
                 dict(speed_limit_mps=0), dict(start_speed_max_mps=0.6),
                 dict(settle_s=0), dict(max_run_s=121), dict(max_run_s=1.5),
                 dict(drive_values=[math.nan, 0]), dict(drive_min=True),
                 dict(mode="steering"), dict(steering_abs_max=2)]
        for change in cases:
            with self.subTest(change=change), self.assertRaises(ValueError):
                Experiment(config(**change))

    def test_negative_drive_needs_both_brake_mode_and_confirmation(self):
        for mode, confirmed in (("drive", True), ("brake", False)):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                Experiment(config(mode=mode, drive_min=-0.2, drive_values=[-0.1, 0],
                                  negative_drive_confirmed=confirmed))
        experiment = Experiment(config(mode="brake", drive_min=-0.2, drive_values=[-0.1, 0],
                                       negative_drive_confirmed=True))
        for tick in range(11):
            step = experiment.step(tick / 20, 0.05 if tick else 0, 0)
        self.assertEqual(step["drive"], -0.1)

    def test_steering_mode_cannot_drive(self):
        experiment = Experiment(config(mode="steering", drive_values=[0, 0],
                                       steering_values=[0.2, -0.2], steering_abs_max=0.2))
        for tick in range(41):
            step = experiment.step(tick / 20, 0.05 if tick else 0, 0)
            self.assertEqual(step["drive"], 0)
            self.assertLessEqual(abs(step["steer"]), 0.2)

    def test_missing_invalid_or_fast_speed_aborts(self):
        for speed in (None, math.nan, math.inf, -0.1, True, 0.5, 1):
            with self.subTest(speed=speed):
                experiment = Experiment(config())
                experiment.step(0, 0, 0)
                step = experiment.step(0.05, 0.05, speed)
                self.assertTrue(step["terminal"])
                self.assertEqual(step["drive"], 0)

    def test_start_requires_stationary_and_timely_call(self):
        for elapsed, speed in ((0, 0.04), (0.2, 0)):
            self.assertTrue(Experiment(config()).step(elapsed, 0, speed)["terminal"])

    def test_long_gap_or_backward_clock_never_skips_to_driving(self):
        for elapsed, dt in ((0.6, 0.6), (-0.01, 0.01), (0.05, 0.2), (math.nan, 0.05)):
            with self.subTest(elapsed=elapsed):
                experiment = Experiment(config())
                experiment.step(0, 0, 0)
                self.assertTrue(experiment.step(elapsed, dt, 0)["terminal"])
                self.assertEqual(experiment.step(0.7, 0.05, 0)["drive"], 0)


class MeasurementTests(unittest.TestCase):
    def test_geometry_matches_static_moment_balance(self):
        result = geometry(0.33, 1.2, 1.8, 0.1, 0.06, 0.12, 0.13)
        self.assertAlmostEqual(result["rear_axle_from_cg_m"], 0.132)
        self.assertAlmostEqual(result["body_front_extent_from_cg_m"], 0.298)
        self.assertAlmostEqual(result["body_rear_extent_from_cg_m"], 0.192)
        self.assertAlmostEqual(result["body_width_m"], 0.26)
        with self.assertRaises(ValueError):
            geometry(0.33, 0, 0, 0.1, 0.06, 0.12, 0.13)

    def test_ackermann_conversion_and_bad_angles(self):
        self.assertAlmostEqual(equivalent_angle(20, 20), math.radians(20))
        self.assertAlmostEqual(equivalent_angle(-20, -20), -math.radians(20))
        for pair in ((0, 5), (-1, 1), (90, 90), (math.nan, 2)):
            with self.subTest(pair=pair), self.assertRaises(ValueError):
                equivalent_angle(*pair)

    def test_recover_known_steering_map(self):
        rows = [{"steering_action": str(u), "delta_equivalent_deg": str(math.degrees(0.6*u+0.02))}
                for u in (-0.5, -0.25, 0, 0.25, 0.5)]
        result = steering_fit(rows)
        self.assertAlmostEqual(result["parameters"]["steering_gain_rad"], 0.6)
        self.assertAlmostEqual(result["parameters"]["steering_offset_rad"], 0.02)
        self.assertLess(result["fit_rmse_rad"], 1e-12)
        with self.assertRaises(ValueError):
            steering_fit(rows[:2])

    def test_dynamic_analysis_observes_delay_without_claiming_rate_limit(self):
        rows = [{"time_s": tick*0.01, "steering_action": 0 if tick < 10 else 0.2,
                 "delta_equivalent_deg": max(0, min(10, tick-15))} for tick in range(40)]
        result = steering_dynamics(rows, 0.5, 3)
        transition = result["transitions"][0]
        self.assertAlmostEqual(transition["observed_delay_s"], 0.06)
        self.assertAlmostEqual(transition["mid_response_rate_rad_s"], math.radians(100))
        with self.assertRaises(ValueError):
            steering_dynamics(rows[::-1], 0.5, 3)

    def test_outputs_never_overwrite_and_export_preserves_incomplete_run(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            target = directory / "result.json"
            save_json(target, {"test": 1})
            with self.assertRaises(FileExistsError):
                save_json(target, {"test": 2})
            log = directory / "run.jsonl"
            sample = {"event": "sample", "receive_monotonic_s": 10, "data": {
                "run_id": "r1", "speed_mps": 0.2, "phase": "stage", "drive": 0.1,
                "sensor_age_s": {"wheel_speed": 0.01}, "sensor_stamp_ns": {}}}
            log.write_text(json.dumps(sample) + '\n{"broken":', encoding="utf-8")
            export_log(log, directory / "export")
            summary = json.loads((directory / "export/summary.json").read_text())
            self.assertFalse(summary["runs"]["r1"]["terminal"])
            self.assertEqual(len(summary["warnings"]), 1)
            with (directory / "export/samples.csv").open(newline="") as stream:
                self.assertEqual(next(csv.DictReader(stream))["wheel_age_s"], "0.01")
            with self.assertRaises(FileExistsError):
                export_log(log, directory / "export")

    def test_recorder_keeps_invalid_values_missing(self):
        self.assertEqual(clean({"gyro": [1, math.nan, math.inf], "wheel": None}),
                         {"gyro": [1, None, None], "wheel": None})


class PolicyBoundaryTests(unittest.TestCase):
    def start(self):
        node = policy_harness()
        harness_tick(node)
        node.fsm_transition_request_callback(SimpleNamespace(data=3))
        node.run_policy_step()
        return node

    def test_actual_policy_stages_finish_and_restart_from_zero(self):
        node = self.start()
        first_id = json.loads(node.identification_publisher.messages[0].data)["run_id"]
        for _ in range(41):
            harness_tick(node, 0.05)
        self.assertEqual(node.fsm_state, 2)
        self.assertTrue(any(m.drive > 0 for m in node.action_publisher.messages))
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        node.fsm_transition_request_callback(SimpleNamespace(data=3))
        node.run_policy_step()
        sample = json.loads(node.identification_publisher.messages[-1].data)
        self.assertNotEqual(sample["run_id"], first_id)
        self.assertEqual(sample["phase"], "settle")
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        self.assertFalse(node.pan_publisher.messages)
        self.assertFalse(node.errors)

    def test_actual_policy_stale_sensor_latches_zero(self):
        node = self.start()
        for _ in range(11):
            harness_tick(node, 0.05)
        self.assertGreater(node.action_publisher.messages[-1].drive, 0)
        harness_tick(node, 0.6, fresh=False)
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        harness_tick(node, 0.05)
        self.assertEqual(node.fsm_state, 2)

    def test_actual_policy_overspeed_and_operator_stop(self):
        node = self.start()
        harness_tick(node, 0.05, speed=0.6)
        self.assertEqual(node.fsm_state, 2)
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        self.assertTrue(json.loads(node.identification_publisher.messages[-1].data)["terminal"])
        node = self.start()
        for _ in range(11):
            harness_tick(node, 0.05)
        node.fsm_transition_request_callback(SimpleNamespace(data=2))
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)
        node.supervision_callback()
        self.assertEqual(node.action_publisher.messages[-1].drive, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
