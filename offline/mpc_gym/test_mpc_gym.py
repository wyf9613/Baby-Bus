"""Checks that the MPC runs under the PID experiment's exact conditions (needs dreamgym 0.4.0)."""
import json
import math
import unittest

try:
    import harness
except ImportError as exc:   # dreamgym not installed: nothing here can run
    harness = None
    IMPORT_ERROR = exc


@unittest.skipIf(harness is None, "Dream Gym environment not available")
class SharedLoop(unittest.TestCase):
    def test_pid_through_the_shared_loop_reproduces_tune_simulate(self):
        gains = harness.Gains(**json.loads(
            (harness.ROOT / "offline/control_pid/results/selected_gains.json").read_text(encoding="utf-8")))
        for scenario in (harness.tune.Scenario('notebook_05', speed=0.5),
                         harness.tune.Scenario('command_delay_100ms', 's_bend', delay_steps=2),
                         harness.tune.Scenario('noisy_reference', 's_bend', reference_noise_m=0.015,
                                               heading_noise_rad=0.015, speed_noise_mps=0.02)):
            expected, expected_trace = harness.tune.simulate(gains, scenario, 101)
            row, trace = harness.run(harness.PIDAdapter(gains), scenario, 101)
            self.assertEqual(row.pop("controller"), "pid")
            self.assertEqual(row, expected, scenario.name)
            self.assertEqual(trace, expected_trace, scenario.name)


@unittest.skipIf(harness is None, "Dream Gym environment not available")
class Adapters(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.api = harness.load_mpc()

    def test_notebook_vehicle_is_the_simulator_and_consistent(self):
        vehicle = harness.notebook_vehicle(self.api)
        config = harness.tune.CONFIG["bicycle_model_config"]
        self.assertIsNone(vehicle.problem())
        self.assertAlmostEqual(vehicle.wheelbase_m, 0.33)
        self.assertAlmostEqual(vehicle.rear_axle_from_cg_m, 0.132)
        self.assertEqual(vehicle.steering_gain_rad,
                         config["steering"]["requested_front_wheel_angle_magnitude_limit_rad"])
        self.assertEqual((vehicle.mass_kg, vehicle.motor_gain_n, vehicle.drag_kg_per_m), (3.0, 10.0, 1.0))
        self.assertTrue(vehicle.source.startswith("sim:"))

    def test_polynomial_reference_becomes_points_on_the_curve(self):
        coeffs = [0.1, 0.05, 0.2, -0.03]
        reference = {"timestamp_s": 4.0, "valid": True, "path_coeffs": coeffs,
                     "x_range_m": [-0.5, 2.0], "target_speed_mps": 0.8}
        traj = harness.poly_to_trajectory(reference, pad_m=1.5)
        self.assertTrue(traj["valid"])
        points = traj["points"]
        on_curve = [p for p in points if p["x_m"] <= 2.0 + 1e-9]
        for p in on_curve:
            y = sum(c*p["x_m"]**i for i, c in enumerate(coeffs))
            self.assertAlmostEqual(p["y_m"], y, places=12)
        self.assertGreater(on_curve[0]["curvature_1pm"], 0.0)          # y'' > 0 near the car: left bend
        self.assertTrue(all(b["s_m"] > a["s_m"] for a, b in zip(points, points[1:])))
        self.assertGreaterEqual(points[-1]["s_m"] - on_curve[-1]["s_m"], 1.5 - 1e-9)
        self.assertTrue(all(p["curvature_1pm"] == 0.0 for p in points[len(on_curve):]))
        info, why = self.api["mpc_reference_errors"](traj, 10, 0.1, 0.0, 0.6)
        self.assertIsNone(why)
        invalid = harness.poly_to_trajectory(dict(reference, valid=False), 1.5)
        self.assertFalse(invalid["valid"])

    def test_mpc_completes_the_notebook_road_within_budget(self):
        adapter = harness.MPCAdapter(self.api)
        row, _ = harness.run(adapter, harness.tune.Scenario('notebook_05', speed=0.5), 101)
        self.assertTrue(row["complete"], row["failure"])
        self.assertEqual(row["mpc_stops"], 0)
        self.assertEqual(row["mpc_over_budget"], 0)
        self.assertLess(row["lateral_max_m"], 0.3)
        self.assertTrue(math.isfinite(row["lateral_rmse_m"]))


if __name__ == "__main__":
    unittest.main()
