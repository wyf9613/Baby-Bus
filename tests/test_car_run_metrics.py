"""tools/car_run_metrics.py on synthetic series (no bags, no mcap)."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import car_run_metrics as m  # noqa: E402

ZERO = "Publishing zero actions: Policy calculation or output was invalid"
GO = "Publishing policy actions: Operator request"


def series(t0, t1, value, dt=0.05):
    n = int(round((t1-t0)/dt))
    return [(t0+i*dt, value(t0+i*dt) if callable(value) else value) for i in range(n+1)]


def road(t, y_left=0.45, y_right=-0.45, left=m.BLUE):
    right = m.YELLOW if left == m.BLUE else m.BLUE
    return [(x, y_left, left) for x in (0.3, 0.6, 0.9, 1.2)] + [(x, y_right, right) for x in (0.3, 0.6, 0.9, 1.2)]


def bag(cones=None, yaw=0.0, steer=0.0, speed=0.2, end=ZERO, vehicle_off_at=None, mpc=()):
    s = {"state": [(0.0, ZERO), (1.0, GO), (11.0, end)],
         "steer": series(0, 12, steer), "speed": series(0, 12, speed), "yaw": series(0, 12, yaw),
         "cones": [(t, (cones or road)(t)) for t, _ in series(0, 12, 0, dt=0.1)],
         "vehicle": [(0.0, "Enabled")] + ([(vehicle_off_at, "Disabled")] if vehicle_off_at else []),
         "mpc": list(mpc)}
    return s


class CarRunMetricsChecks(unittest.TestCase):
    def test_runs_split_on_the_policy_state_strings(self):
        states = [(0, ZERO), (1, GO), (5, ZERO), (6, ZERO), (7, GO), (9, ZERO), (10, GO)]
        self.assertEqual(m.split_runs(states), [(1, 5, ZERO), (7, 9, ZERO), (10, 10, "bag ended")])

    def test_imu_rotation_flips_yaw_for_the_upside_down_mount(self):
        self.assertAlmostEqual(m.rotate(m.DEFAULT_IMU_MOUNT, (0.0, 0.0, 0.3))[2], -0.3)
        self.assertAlmostEqual(m.rotate((0.0, 0.0, 0.0, 1.0), (0.1, 0.2, 0.3))[2], 0.3)

    def test_a_straight_centred_run(self):
        r = m.run_metrics(bag(), 1.0, 11.0, ZERO)
        self.assertAlmostEqual(r["distance_m"], 2.0, places=1)
        self.assertEqual((r["bends"], r["crosses"], r["touches"]), ([], 0, 0))
        self.assertEqual(r["offset_p95_m"], 0.0)
        self.assertEqual((r["left_colour"], r["cones_source"]), (m.BLUE, "auto"))
        self.assertTrue(r["vehicle_enabled"])
        self.assertEqual(r["full_lock_share"], 0.0)

    def test_one_bend_is_one_bend_despite_noise(self):
        # 90 deg left over 4 s with a +-2 deg wobble, then straight
        yaw = lambda t: (math.pi/2/4 if 3 <= t < 7 else 0.0) + 0.6*math.sin(25*t)*math.radians(2)
        r = m.run_metrics(bag(yaw=yaw), 1.0, 11.0, ZERO)
        self.assertEqual(len(r["bends"]), 1, r["bends"])
        self.assertAlmostEqual(r["bends"][0]["deg"], 90, delta=6)
        self.assertGreater(r["heading_max_deg"], 85)

    def test_crossing_the_right_boundary_is_one_event_and_a_short_glitch_is_none(self):
        # car drifts right: the right (yellow) cones end up left of the car from t = 6 to 8
        cones = lambda t: road(t, y_left=1.0, y_right=0.1 if 6 <= t < 8 else -0.45)
        r = m.run_metrics(bag(cones=cones), 1.0, 11.0, ZERO)
        self.assertEqual((r["crosses"], r["events"][0]["kind"]), (1, "cross_right"))
        self.assertAlmostEqual(r["events"][0]["start"], 6.0, delta=0.11)
        self.assertEqual(r["touches"], 0)                    # superseded by the cross
        glitch = lambda t: road(t, y_right=0.1 if abs(t-6.0) < 0.05 else -0.45)
        self.assertEqual(m.run_metrics(bag(cones=glitch), 1.0, 11.0, ZERO)["crosses"], 0)

    def test_touch_and_offset(self):
        cones = lambda t: road(t, y_left=0.70, y_right=-0.07)  # car 0.31 m right of centre... of the pair
        r = m.run_metrics(bag(cones=cones), 1.0, 11.0, ZERO)
        self.assertEqual((r["touches"], r["crosses"]), (1, 0))
        self.assertAlmostEqual(r["offset_p50_m"], 0.315, places=3)
        self.assertAlmostEqual(r["min_gap_right_m"], 0.07, places=3)

    def test_colours_are_taken_from_the_road_or_the_option(self):
        yellow_left = lambda t: road(t, left=m.YELLOW)
        self.assertEqual(m.run_metrics(bag(cones=yellow_left), 1.0, 11.0, ZERO)["left_colour"], m.YELLOW)
        r = m.run_metrics(bag(cones=yellow_left), 1.0, 11.0, ZERO, cones_mode="blue-left")
        self.assertEqual((r["left_colour"], r["crosses"]), (m.BLUE, 2))   # wrong layout: both sides "crossed"

    def test_end_reasons(self):
        stop = [(10.98, {"branch": "stop", "reject_reason": "invalid_road_insufficient_geometry",
                         "reference_selection": "planner", "longitudinal_mode": "joint"})]
        r = m.run_metrics(bag(mpc=stop), 1.0, 11.0, ZERO)
        self.assertEqual((r["end"], r["mpc_stop"], r["lattice_share"]), ("invalid_road_insufficient_geometry",
                                                                         "invalid_road_insufficient_geometry", 1.0))
        r = m.run_metrics(bag(vehicle_off_at=10.9), 1.0, 11.0, ZERO)
        self.assertEqual(r["end"], "manual_disable")
        self.assertNotIn("mpc_stop", r)                     # PID bags: no mpc_debug

    def test_speed_and_steering_statistics(self):
        speed = lambda t: 0.45 if 4 <= t < 5 else 0.2
        steer = lambda t: 1.0 if (int(t) % 2) else -1.0
        r = m.run_metrics(bag(speed=speed, steer=steer), 1.0, 11.0, ZERO)
        self.assertAlmostEqual(r["s_above_0_40"], 1.0, delta=0.06)
        self.assertEqual(r["speed_max"], 0.45)
        self.assertEqual(r["full_lock_share"], 1.0)
        self.assertEqual(r["steer_swings"], 10)       # a flip at t = 2, 3, ..., 11


if __name__ == "__main__":
    unittest.main()
