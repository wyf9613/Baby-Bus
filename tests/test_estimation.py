"""Offline numerical/contract checks of the single-file estimator, without ROS.

Extract the actual pure estimator definitions from the deployed policy source;
do not copy its algorithms or install ROS mocks. ROS lifecycle/transport checks
remain in test_policy_node.py and the pinned CONTRIBUTING fast gate.
"""
import ast
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "policy_node.py"
try:
    from ament_index_python.packages import get_package_prefix
except ImportError:
    pass  # Portable offline run: verify the working-tree source.
else:
    # In the ROS gate verify the INSTALLED single-file policy, like the existing
    # framework suite, rather than accidentally testing a different source copy.
    SOURCE = Path(get_package_prefix("ai4r_policy")) / "lib" / "ai4r_policy" / "policy_node.py"
tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
names = {"finite_number", "Observation", "EstimationSettings", "FirstOrderSampleFilter", "EstimationMotionHistory",
         "_estimation_median", "_estimation_solve", "_estimation_fit_boundary", "EstimationPipeline",
         "PlanningSettings", "CenterlinePlanner", "evaluate_planning_path", "planning_vehicle_limits",
         "ControlSettings", "VehicleCalibrationSettings", "ControlPID", "PolicyController", "wrap_angle",
         "PolicyStopRequest", "RunDistanceLimiter",
         "_control_poly_eval", "_control_poly_roots", "control_path_geometry"}
names.update({"LatticeSettings", "LatticeCurve", "FrenetLatticePlanner", "_lattice_interp",
              "_lattice_derivative", "_lattice_lateral", "_lattice_geometry", "_lattice_steering"})
names.add("evaluate_frenet_path")
names.add("_planning_control_geometry")
definitions = [item for item in tree.body if isinstance(item, (ast.FunctionDef, ast.ClassDef))
               and item.name in names]
policy_class = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "PolicyNode")
policy_method = next(item for item in policy_class.body if isinstance(item, ast.FunctionDef)
                     and item.name == "calculate_policy_actions")
store_method = next(item for item in policy_class.body if isinstance(item, ast.FunctionDef)
                    and item.name == "_store")
namespace = {"dataclass": dataclass, "math": math, "deepcopy": deepcopy,
             "time": __import__("time"),
             "Real": __import__("numbers").Real,
             "ConeDetection": SimpleNamespace(COLOR_BLUE=2, COLOR_YELLOW=1)}
exec(compile(ast.Module(body=definitions+[policy_method, store_method], type_ignores=[]), str(SOURCE), "exec"), namespace)
Settings = namespace["EstimationSettings"]
Filter = namespace["FirstOrderSampleFilter"]
Pipeline = namespace["EstimationPipeline"]
Motion = namespace["EstimationMotionHistory"]


def cones(left, right):
    return {"detections": [(x, y, 0.0, colour, 0.95)
                           for colour, points in ((2, left), (1, right)) for x, y in points],
            "acquisition_to_publish_latency_s": 0.05}


def straight(slope=0.0, center=0.0, width=1.0):
    offset = width*math.hypot(1, slope)/2
    xs = [0.5, 0.8, 1.1, 1.4, 1.7, 2.0, 2.3, 2.6]
    return ([(x, center+slope*x+offset) for x in xs],
            [(x, center+slope*x-offset) for x in xs])


def input_record(speed=0.0, yaw=0.0, lidar=None):
    left, right = straight()
    # This fixture observes both sides across the car origin; forward-only
    # observations are tested separately and cannot qualify near-field support.
    left = [(-0.3, 0.5), (0.0, 0.5), (0.3, 0.5)] + left
    right = [(-0.3, -0.5), (0.0, -0.5), (0.3, -0.5)] + right
    obs = {"cone_detections": cones(left, right), "wheel_speed": speed,
           "imu_angular_velocity": (0.0, 0.0, yaw) if yaw is not None else None,
           "lidar_cartesian": lidar}
    ages = {key: 0.05 for key in obs}
    stamps = {"cone_detections": 10_000_000_000, "wheel_speed": None,
              "imu_angular_velocity": 10_000_000_000, "lidar_cartesian": 10_000_000_000}
    return obs, ages, stamps, {"wheel_speed": 1.0}


class EstimationChecks(unittest.TestCase):
    def test_cached_sample_is_not_reassimilated(self):
        f = Filter(0.15, 0.5)
        f.update(0.0, 1, 0.0)
        first = f.update(1.0, 2, 0.1)
        for _ in range(25):
            self.assertEqual(f.update(1.0, 2, 0.1), first)
        # Equal numerical values with NEW source identity are valid updates.
        self.assertGreater(f.update(1.0, 3, 0.2), first)

    def test_filter_respects_actual_spacing_and_discontinuities(self):
        fast, slow = Filter(0.15, 0.5), Filter(0.15, 0.5)
        fast.update(0.0, 1, 0.0)
        slow.update(0.0, 1, 0.0)
        self.assertGreater(slow.update(1.0, 2, 0.2), fast.update(1.0, 2, 0.02))
        self.assertEqual(fast.update(2.0, 3, 1.0), 2.0)
        self.assertEqual(fast.update(0.5, 4, 0.0), 0.5)

    def test_filter_reduces_noise_and_exhibits_finite_step_lag(self):
        f = Filter(0.15, 0.5)
        values = [1.0+0.2*(-1)**i for i in range(100)]
        smoothed = [f.update(v, i, i*0.05) for i, v in enumerate(values)]
        raw_error = sum((v-1)**2 for v in values[20:])
        filtered_error = sum((v-1)**2 for v in smoothed[20:])
        self.assertLess(filtered_error, raw_error/4)
        first_step = f.update(2.0, 100, 5.0)
        self.assertGreater(first_step, 1.0)
        self.assertLess(first_step, 2.0)

    def test_measured_zero_is_distinct_from_missing_and_stale(self):
        p = Pipeline(Settings())
        obs, ages, stamps, receipts = input_record()
        valid_zero = p.update(obs, ages, stamps, receipts, 10.05)["state"]
        self.assertTrue(valid_zero["valid"])
        self.assertEqual(valid_zero["speed_mps"], 0.0)
        obs["wheel_speed"] = None
        missing = p.update(obs, ages, stamps, receipts, 10.05)["state"]
        self.assertFalse(missing["valid"])
        self.assertIsNone(missing["speed_mps"])
        obs["wheel_speed"], ages["wheel_speed"] = 0.0, 0.5
        self.assertFalse(p.update(obs, ages, stamps, receipts, 10.5)["state"]["speed_valid"])

    def test_draft_fields_aliases_and_source_times(self):
        out = Pipeline(Settings()).update(*input_record(speed=0.5, yaw=0.1), 10.05)
        state, road = out["state"], out["road"]
        self.assertEqual(state["speed_mps"], state["v_mps"])
        self.assertEqual(state["yaw_rate_rps"], state["yaw_rate_radps"])
        self.assertIsNone(state["source_stamp_ns"]["wheel_speed"])
        self.assertEqual(state["timestamp_s"], road["timestamp_s"])
        self.assertEqual(road["timestamp_s"], 10.05)
        self.assertEqual(road["measurement_timestamp_s"], 10.0)
        self.assertEqual(road["source_age_s"], 0.05)
        self.assertTrue(road["motion_compensated"])
        self.assertTrue(out["alignment"]["valid"])
        self.assertFalse(out["vehicle_limits"]["valid"])
        self.assertFalse(out["vehicle_params"]["valid"])
        json.dumps(out, allow_nan=False)

    def test_straight_offset_and_sloped_width(self):
        road = Pipeline(Settings()).road(cones(*straight(0.3, 0.2)), 10_000_000_000, 0.05)
        self.assertTrue(road["valid"])
        self.assertAlmostEqual(road["lane_width_m"], 1.0, places=6)
        for x, y in road["centerline_xy"]:
            self.assertAlmostEqual(y, 0.2+0.3*x, places=6)
        self.assertNotEqual(road["centerline_xy"][0][1], 0.0)

    def test_curved_road_recovers_geometry(self):
        xs = [0.5+i*0.25 for i in range(9)]
        left, right = [(x, 0.08*x*x+0.5) for x in xs], [(x, 0.08*x*x-0.5) for x in xs]
        road = Pipeline(Settings()).road(cones(left, right), 10_000_000_000, 0.05)
        self.assertTrue(road["valid"])
        self.assertGreater(road["local_curvature_1pm"], 0)
        for x, y in road["centerline_xy"]:
            self.assertAlmostEqual(y, 0.08*x*x, places=5)

    def test_robust_fit_resists_one_bad_cone(self):
        left, right = straight()
        left[3] = (left[3][0], 1.8)
        road = Pipeline(Settings()).road(cones(left, right), 10_000_000_000, 0.05)
        self.assertTrue(road["valid"])
        self.assertLess(max(abs(y) for _, y in road["centerline_xy"]), 0.04)
        self.assertLess(road["confidence"], 1.0)

    def test_single_side_needs_known_width_and_offsets_along_normal(self):
        left, _ = straight(slope=0.4)
        batch = cones(left, [])
        self.assertFalse(Pipeline(Settings()).road(batch, 10_000_000_000, 0.05)["valid"])
        road = Pipeline(Settings(known_lane_width_m=1.0)).road(batch, 10_000_000_000, 0.05)
        self.assertTrue(road["valid"])
        self.assertEqual(road["visibility"], "left_only")
        self.assertIsNone(road["right_boundary_xy"])
        for x, y in road["centerline_xy"]:
            self.assertAlmostEqual(y, 0.4*x, places=5)

    def test_empty_stale_degenerate_crossing_and_gapped_roads_fail(self):
        pipeline = Pipeline(Settings())
        left, right = straight()
        cases = [cones([], []), cones([(1, 0.5)]*5, [(1, -0.5)]*5),
                 cones(right, left), cones([(0.1, 0.5), (2.0, 0.5)], [(0.1, -0.5), (2.0, -0.5)])]
        for batch in cases:
            with self.subTest(batch=batch):
                self.assertFalse(pipeline.road(batch, 10_000_000_000, 0.05)["valid"])
        self.assertFalse(pipeline.road(cones(left, right), 10_000_000_000, 0.5)["valid"])

    def test_lidar_empty_available_differs_from_unavailable_and_copies_points(self):
        pipeline = Pipeline(Settings())
        args = input_record(lidar={"points_xyz": [], "scan_indices": []})
        empty = pipeline.update(*args, 10.05)["obstacles"]
        self.assertTrue(empty["available"])
        args[0]["lidar_cartesian"]["points_xyz"] = [(1, 0, 0.12)]
        args[0]["lidar_cartesian"]["scan_indices"] = [5]
        out = pipeline.update(*args, 10.05)["obstacles"]
        out["points_xyz_m"].clear()
        self.assertEqual(len(args[0]["lidar_cartesian"]["points_xyz"]), 1)
        args[0]["lidar_cartesian"] = None
        self.assertFalse(pipeline.update(*args, 10.05)["obstacles"]["available"])

    def test_unusable_numbers_are_not_geometry_or_motor_values(self):
        left, right = straight()
        batch = cones(left, right)
        batch["detections"] += [(1, float("nan"), 0, 2, 0.9), (1, 1e308, 0, 2, 0.9)]
        self.assertTrue(Pipeline(Settings()).road(batch, 10_000_000_000, 0.05)["valid"])
        f = Filter(0.15, 0.5)
        self.assertIsNone(f.update(float("nan"), 1, 0.0))

    def test_settings_reject_invalid_ranges_and_unbounded_work(self):
        for kwargs in ({"speed_tau_s": 0.0}, {"road_sample_spacing_m": 1e-6},
                       {"known_lane_width_m": -1.0}, {"road_width_min_m": 3.0},
                       {"min_cone_confidence": float("nan")}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Settings(**kwargs)

    def test_shipped_estimation_configuration_is_valid(self):
        # The subtree consists only of double scalar defaults. Parse that
        # restricted subset without adding a YAML dependency to the offline run.
        import re
        config = Path(__file__).resolve().parents[1] / "config" / "ai4r_policy.yaml"
        scalars = {}
        inside = False
        for line in config.read_text(encoding="utf-8").splitlines():
            if line == "    estimation:":
                inside = True
                continue
            if not inside or not line.strip() or line.lstrip().startswith("#"):
                continue
            if not line.startswith("      "):
                break
            match = re.fullmatch(r"      ([a-z_]+): (-?[0-9]+(?:\.[0-9]+)?)", line)
            self.assertIsNotNone(match, line)
            self.assertNotIn(match[1], scalars)
            scalars[match[1]] = float(match[2])
        settings = Settings(**scalars)
        self.assertLessEqual(settings.motion_max_interval_s, settings.max_source_age_s)

    def test_actual_student_entry_exports_estimates_and_returns_zero_actions(self):
        obs, ages, stamps, receipts = input_record()
        for key in ("fiducial_detections", "lidar_scan", "imu_orientation", "imu_specific_force"):
            obs[key] = None
        node = SimpleNamespace(estimation_settings=Settings(), policy_frame_id="base_link",
                               planning_settings=namespace["PlanningSettings"](),
                               sensor_timeout_s={key: 0.5 for key in obs},
                               heading_reference=None,
                               motion_history=Motion(Settings()),
                               observations={k: SimpleNamespace(received_at=receipts.get(k),
                                                               received_ros_ns=10_000_000_000) for k in obs},
                               get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_050_000_000)))
        method = namespace["calculate_policy_actions"]
        result = method(node, obs, ages, stamps, 0.0, 0.0, True)
        self.assertEqual(result, (0.0, 0.0, None, None, None))
        self.assertTrue(node.estimation_output["road"]["valid"])
        obs["wheel_speed"] = None
        method(node, obs, ages, stamps, 0.05, 0.05, False)
        self.assertFalse(node.estimation_output["state"]["valid"])

    def test_delayed_straight_road_moves_into_current_body_frame(self):
        pipeline = Pipeline(Settings())
        args = input_record(speed=0.5, yaw=0.0)
        raw = pipeline.road(args[0]["cone_detections"], args[2]["cone_detections"], 0.05)
        out = pipeline.update(*args, 10.05)
        self.assertTrue(out["alignment"]["valid"])
        self.assertEqual(out["road"]["timestamp_s"], out["state"]["timestamp_s"])
        for old, new in zip(raw["centerline_xy"], out["road"]["centerline_xy"]):
            self.assertAlmostEqual(new[0], old[0]-0.025)
            self.assertAlmostEqual(new[1], old[1])

    def test_left_turn_uses_inverse_rigid_transform(self):
        motion = Motion(Settings())
        motion.observe("wheel_speed", 1.0, 1, 1.0, 10.0)
        motion.observe("imu_angular_velocity", 0.5, 10_000_000_000, 10.0, 10.0)
        pose, reason = motion.pose_between(10.0, 10.1)
        self.assertIsNone(reason)
        self.assertAlmostEqual(pose["dx_m"], math.sin(0.05)/0.5)
        self.assertAlmostEqual(pose["dy_m"], (1-math.cos(0.05))/0.5)
        x, y = motion.transform_xy((2.0, 0.0), pose)
        self.assertAlmostEqual(x, math.cos(0.05)*(2-pose["dx_m"])-math.sin(0.05)*pose["dy_m"])
        self.assertAlmostEqual(y, -math.sin(0.05)*(2-pose["dx_m"])-math.cos(0.05)*pose["dy_m"])
        self.assertLess(y, 0)

    def test_asynchronous_motion_knots_are_integrated_causally(self):
        settings = Settings(speed_tau_s=1e-6, yaw_rate_tau_s=1e-6)
        motion = Motion(settings)
        motion.observe("wheel_speed", 1.0, 1, 1.0, 10.0)
        motion.observe("imu_angular_velocity", 0.0, 10_000_000_000, 10.0, 10.0)
        motion.observe("wheel_speed", 2.0, 2, 1.05, 10.05)
        motion.observe("imu_angular_velocity", 0.0, 10_080_000_000, 10.08, 10.08)
        pose, reason = motion.pose_between(10.0, 10.1)
        self.assertIsNone(reason)
        self.assertAlmostEqual(pose["dx_m"], 0.15)
        self.assertAlmostEqual(pose["dy_m"], 0.0)
        # A future speed sample must not alter an earlier interval.
        motion.observe("wheel_speed", 20.0, 3, 1.2, 10.2)
        earlier, reason = motion.pose_between(10.0, 10.1)
        self.assertIsNone(reason)
        self.assertEqual(earlier, pose)

    def test_fresh_latest_motion_does_not_fill_missing_history(self):
        args = input_record(speed=0.5, yaw=0.3)
        args[1].update(cone_detections=0.2, wheel_speed=0.04, imu_angular_velocity=0.02)
        args[2]["imu_angular_velocity"] = 10_180_000_000
        out = Pipeline(Settings()).update(*args, 10.2)
        self.assertTrue(out["state"]["valid"])
        self.assertFalse(out["road"]["valid"])
        self.assertFalse(out["alignment"]["valid"])
        self.assertEqual(out["road"]["status"], "missing_motion_history")
        self.assertIsNone(out["road"]["timestamp_s"])
        self.assertEqual(out["road"]["measurement_timestamp_s"], 10.0)
        self.assertEqual(out["road"]["centerline_xy"], [])

    def test_accepted_samples_between_camera_triggers_align_delayed_road(self):
        settings = Settings()
        motion = Motion(settings)
        node = SimpleNamespace(motion_history=motion,
            sensor_timeout_s={"wheel_speed": 0.5, "imu_angular_velocity": 0.5},
            timestamp_tolerance_s=0.05,
            observations={"wheel_speed": None, "imu_angular_velocity": None},
            _warn=lambda *_: None)
        store = namespace["_store"]
        # Actual _store entry is used at each accepted telemetry callback;
        # there is no policy calculation between these motion measurements.
        for dt in (0.0, 0.05, 0.1, 0.15):
            stamp = round((10.0+dt)*1e9)
            self.assertTrue(store(node, "wheel_speed", 0.5, None, 1.0+dt, stamp))
            self.assertTrue(store(node, "imu_angular_velocity", (0, 0, 0.3), stamp, 1.0+dt, stamp))
        self.assertTrue(store(node, "wheel_speed", 0.5, None, 1.16, 10_160_000_000))
        self.assertTrue(store(node, "imu_angular_velocity", (0, 0, 0.3), 10_180_000_000, 1.18, 10_180_000_000))
        self.assertEqual(node.observations["wheel_speed"].received_ros_ns, 10_160_000_000)
        count = len(motion.samples["imu_angular_velocity"])
        self.assertFalse(store(node, "imu_angular_velocity", (0, 0, 99), 10_180_000_000, 1.19, 10_190_000_000))
        self.assertEqual(len(motion.samples["imu_angular_velocity"]), count)
        args = input_record(speed=0.5, yaw=0.3)
        args[1].update(cone_detections=0.2, wheel_speed=0.04, imu_angular_velocity=0.02)
        args[2]["imu_angular_velocity"] = 10_180_000_000
        args[3]["wheel_speed"] = 1.16
        pipeline = Pipeline(settings, motion_history=motion)
        raw = pipeline.road(args[0]["cone_detections"], 10_000_000_000, 0.2)
        out = pipeline.update(*args, 10.2, {"wheel_speed": 10_160_000_000})
        self.assertTrue(out["alignment"]["valid"])
        self.assertEqual(out["road"]["timestamp_s"], 10.2)
        self.assertEqual(out["state"]["timestamp_s"], 10.2)
        self.assertEqual(out["road"]["measurement_timestamp_s"], 10.0)
        # Independent analytic constant-turn reference, not a copied integrator.
        angle = 0.3*0.2
        dx, dy = 0.5/0.3*math.sin(angle), 0.5/0.3*(1-math.cos(angle))
        for (x, y), transformed in zip(raw["centerline_xy"], out["road"]["centerline_xy"]):
            self.assertAlmostEqual(transformed[0], math.cos(angle)*(x-dx)+math.sin(angle)*(y-dy))
            self.assertAlmostEqual(transformed[1], -math.sin(angle)*(x-dx)+math.cos(angle)*(y-dy))

    def test_stop_reset_removes_previous_run_history(self):
        motion = Motion(Settings())
        motion.observe("wheel_speed", 1.0, 1, 1.0, 10.0)
        motion.observe("imu_angular_velocity", 0.0, 1, 10.0, 10.0)
        motion.clear()
        pose, reason = motion.pose_between(10.0, 10.05)
        self.assertIsNone(pose)
        self.assertEqual(reason, "missing_motion_history")

    def test_transformed_road_behind_vehicle_is_rejected(self):
        args = input_record(speed=30.0, yaw=0.0)
        args[1].update(cone_detections=0.1, wheel_speed=0.1, imu_angular_velocity=0.1)
        out = Pipeline(Settings()).update(*args, 10.1)
        self.assertFalse(out["road"]["valid"])
        self.assertEqual(out["road"]["status"], "motion_transformed_geometry_invalid")

    def test_future_camera_stamp_cannot_be_retrospectively_aligned(self):
        args = input_record(speed=0.5, yaw=0.0)
        args[2]["cone_detections"] = 10_080_000_000
        args[1]["cone_detections"] = 0.0
        out = Pipeline(Settings()).update(*args, 10.05)
        self.assertFalse(out["road"]["valid"])
        self.assertEqual(out["road"]["status"], "invalid_motion_time")

    def test_motion_gap_is_not_hidden_by_new_samples(self):
        motion = Motion(Settings())
        for name in ("wheel_speed", "imu_angular_velocity"):
            motion.observe(name, 0.5 if name == "wheel_speed" else 0.0, 1, 1.0, 10.0)
            motion.observe(name, 0.5 if name == "wheel_speed" else 0.0, 2, 1.2, 10.2)
        pose, reason = motion.pose_between(10.0, 10.25)
        self.assertIsNone(pose)
        self.assertEqual(reason, "motion_history_gap")

    def test_clock_rollback_clears_motion_and_future_data_is_not_used(self):
        motion = Motion(Settings())
        motion.check_clock(10.0)
        motion.observe("wheel_speed", 0.5, 1, 1.0, 10.0)
        motion.observe("imu_angular_velocity", 0.1, 1, 10.0, 10.0)
        motion.check_clock(9.0)
        self.assertIsNone(motion.current("wheel_speed", 9.0))
        motion.observe("wheel_speed", 0.5, 2, 2.0, 9.1)
        self.assertIsNone(motion.current("wheel_speed", 9.0))

    def test_interval_limit_and_negative_speed_reject_motion(self):
        motion = Motion(Settings())
        self.assertEqual(motion.pose_between(10.0, 10.4)[1], "motion_interval_too_long")
        motion.observe("wheel_speed", -0.5, 1, 1.0, 10.0)
        self.assertIsNone(motion.current("wheel_speed", 10.0))
        self.assertEqual(motion.pose_between(10.1, 10.0)[1], "invalid_motion_time")

    def test_lidar_points_share_epoch_and_keep_scan_metadata(self):
        out = Pipeline(Settings()).update(*input_record(speed=0.5, yaw=0.0,
            lidar={"points_xyz": [(1.0, 0.0, 0.12)], "scan_indices": [7]}), 10.05)
        obstacle = out["obstacles"]
        self.assertTrue(obstacle["available"])
        self.assertEqual(obstacle["timestamp_s"], out["state"]["timestamp_s"])
        self.assertEqual(obstacle["measurement_timestamp_s"], 10.0)
        self.assertAlmostEqual(obstacle["points_xyz_m"][0][0], 0.975)
        self.assertEqual(obstacle["points_xyz_m"][0][2], 0.12)
        self.assertEqual(obstacle["scan_indices"], [7])
        self.assertFalse(obstacle["deskewed"])

    def test_cached_motion_has_bounded_memory_and_no_duplicate_samples(self):
        motion = Motion(Settings())
        motion.observe("wheel_speed", 0.5, 1, 1.0, 10.0)
        for _ in range(20):
            motion.observe("wheel_speed", 0.5, 1, 1.0, 10.0)
        self.assertEqual(len(motion.samples["wheel_speed"]), 1)
        for index in range(1, 800):
            motion.observe("wheel_speed", 0.5, index+1, 1+index*0.001, 10+index*0.001)
        self.assertLessEqual(len(motion.samples["wheel_speed"]), 512)


class NearFieldHistoryChecks(unittest.TestCase):
    @staticmethod
    def step(pipeline, time_s, xs, speed=0.0, yaw=0.0, offset=0.0):
        batch = cones([(x, offset+0.5) for x in xs], [(x, offset-0.5) for x in xs])
        obs = {"cone_detections": batch, "wheel_speed": speed,
               "imu_angular_velocity": (0.0, 0.0, yaw), "lidar_cartesian": None}
        stamp = round(time_s*1e9)
        return pipeline.update(obs, {key: 0.0 for key in obs},
                               {"cone_detections": stamp, "wheel_speed": None,
                                "imu_angular_velocity": stamp}, {"wheel_speed": time_s}, time_s)

    def test_front_only_start_is_not_a_driving_corridor(self):
        out = self.step(Pipeline(Settings()), 10.0, [0.9+0.3*i for i in range(7)])
        road = out["road"]
        self.assertFalse(road["valid"])
        self.assertTrue(road["geometry_valid"])
        self.assertEqual(road["status"], "near_field_unobserved")
        self.assertAlmostEqual(road["near_field"]["coverage_x_m"][0], 0.9)
        self.assertFalse(out["alignment"]["valid"])
        self.assertEqual(road["centerline_xy"], [])
        self.assertFalse(road["near_field"]["footprint_validated"])

    def test_old_observations_fill_a_moving_camera_blind_zone(self):
        pipeline = Pipeline(Settings())
        world_xs = [0.9+0.3*i for i in range(20)]
        first_usable = None
        for frame in range(81):
            distance = 0.2*frame*0.1
            visible = [x-distance for x in world_xs if 0.9-1e-9 <= x-distance <= 3.0]
            out = self.step(pipeline, 10+frame*0.1, visible, speed=0.2)
            if out["road"]["valid"]:
                first_usable = frame if first_usable is None else first_usable
                self.assertTrue(out["road"]["history_used"])
                self.assertTrue(out["road"]["near_field"]["valid"])
                self.assertLessEqual(out["road"]["x_range_m"][0], 1e-9)
                self.assertTrue(all(abs(y) < 1e-8 for x, y in out["road"]["centerline_xy"]))
                self.assertIsNotNone(out["road"]["local_curvature_1pm"])
        self.assertIsNotNone(first_usable)
        self.assertGreaterEqual(first_usable, 45)  # No invented early road.
        self.assertTrue(out["road"]["valid"])

    def test_unchanged_camera_batch_is_not_reassimilated(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, [-0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1])
        out = self.step(pipeline, 10.1, [0.9, 1.2, 1.5, 1.8, 2.1])
        saved = list(pipeline._road_cones)
        for _ in range(25):
            repeat = self.step(pipeline, 10.1, [0.9, 1.2, 1.5, 1.8, 2.1])
            self.assertEqual(pipeline._road_cones, saved)
            self.assertEqual(repeat["road"], out["road"])

    def test_fresh_far_frames_never_renew_unseen_near_points(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, [-0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1])
        for frame in range(1, 52):
            out = self.step(pipeline, 10+frame*0.1, [0.9, 1.2, 1.5, 1.8, 2.1])
        self.assertFalse(out["road"]["valid"])
        self.assertEqual(out["road"]["status"], "near_field_unobserved")
        self.assertFalse(out["road"]["history_used"])

    def test_overlap_conflict_discards_old_near_corridor(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, [-0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1])
        out = self.step(pipeline, 10.1, [0.9, 1.2, 1.5, 1.8, 2.1], offset=0.3)
        self.assertFalse(out["road"]["valid"])
        self.assertFalse(out["road"]["history_used"])
        self.assertTrue(all(p[5] == 10.1 for p in pipeline._road_cones))

    def test_missing_motion_interval_cannot_transport_near_history(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, [-0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1])
        out = self.step(pipeline, 10.2, [0.9, 1.2, 1.5, 1.8, 2.1])
        self.assertFalse(out["road"]["valid"])
        self.assertFalse(out["road"]["history_used"])

    def test_stop_and_clock_reset_clear_near_history(self):
        for reset in ("stop", "clock"):
            pipeline = Pipeline(Settings())
            self.step(pipeline, 10, [-0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1])
            if reset == "stop":
                pipeline.motion.clear()
            out = self.step(pipeline, 9.9 if reset == "clock" else 10.1,
                            [0.9, 1.2, 1.5, 1.8, 2.1])
            self.assertFalse(out["road"]["valid"])
            self.assertFalse(out["road"]["history_used"])

    def test_empty_frame_bridges_briefly_then_expires_without_new_support(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, [-0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1])
        out = self.step(pipeline, 10.1, [])
        self.assertTrue(out["road"]["valid"])
        self.assertTrue(out["road"]["degraded"])
        self.assertEqual(out["road"]["current_frame"]["visibility"], "none")
        self.assertTrue(self.step(pipeline, 10.2, [])["road"]["valid"])
        out = self.step(pipeline, 10.3, [])
        self.assertFalse(out["road"]["valid"])
        self.assertEqual(pipeline._road_cones, [])
        out = self.step(pipeline, 10.4, [0.9, 1.2, 1.5, 1.8, 2.1])
        self.assertFalse(out["road"]["valid"])

    def test_history_is_bounded_and_contains_no_internal_public_fields(self):
        pipeline = Pipeline(Settings())
        for frame in range(8):
            xs = [-0.3+0.03*i+0.003*frame for i in range(101)]
            out = self.step(pipeline, 10+frame*0.1, xs)
            self.assertLessEqual(len(pipeline._road_cones), 128)
            for colour in (1, 2):
                self.assertLessEqual(sum(p[3] == colour for p in pipeline._road_cones), 64)
            self.assertLessEqual(len(out["road"]["centerline_xy"]), 201)
            self.assertFalse(any(key.startswith("_") for key in out["road"]))
            json.dumps(out, allow_nan=False)

    def test_cached_cones_turn_with_the_vehicle(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, [-0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1], yaw=0.2)
        pose, reason = pipeline.motion.pose_between(10, 10.1)
        self.assertIsNone(reason)
        old = list(pipeline._road_cones)
        self.step(pipeline, 10.1, [0.9, 1.2, 1.5, 1.8, 2.1], yaw=0.2)
        for point in old:
            if point[0] >= 0.6:
                continue
            x, y = pipeline.motion.transform_xy(point, pose)
            self.assertTrue(any(abs(p[0]-x) < 1e-9 and abs(p[1]-y) < 1e-9
                                for p in pipeline._road_cones))

    def test_curved_noisy_corridor_keeps_near_support_after_warmup(self):
        import random
        rng = random.Random(42)
        pipeline = Pipeline(Settings(speed_tau_s=1e-6, yaw_rate_tau_s=1e-6))
        usable = 0
        for frame in range(70):
            elapsed = frame*0.1
            angle = 0.02*elapsed
            pose = {"dx_m": 20*math.sin(angle), "dy_m": 20*(1-math.cos(angle)), "yaw_rad": angle}
            left, right = [], []
            for side, radius in ((left, 19.5), (right, 20.5)):
                for i in range(35):
                    theta = (0.9+0.3*i)/20
                    x, y = pipeline.motion.transform_xy((radius*math.sin(theta), 20-radius*math.cos(theta)), pose)
                    if 0.9 <= x <= 3.0:
                        side.append((x, y+rng.gauss(0, 0.01)))
            stamp = round((10+elapsed)*1e9)
            obs = {"cone_detections": cones(left, right), "wheel_speed": 0.4,
                   "imu_angular_velocity": (0, 0, 0.02)}
            out = pipeline.update(obs, {k: 0.0 for k in obs},
                                  {"cone_detections": stamp, "imu_angular_velocity": stamp},
                                  {"wheel_speed": 10+elapsed}, 10+elapsed)
            if frame >= 30:
                self.assertTrue(out["road"]["valid"], out["road"]["status"])
                usable += 1
                error = max(abs(y-(20-math.sqrt(400-x*x))) for x, y in out["road"]["centerline_xy"])
                self.assertLess(error, 0.03)
                self.assertTrue(out["road"]["near_field"]["valid"])
        self.assertEqual(usable, 40)

    def test_missing_ego_state_does_not_store_an_unaligned_near_cache(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, [-0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1])
        obs, ages, stamps, receipts = input_record()
        obs["wheel_speed"] = None
        pipeline.update(obs, ages, stamps, receipts, 10.05)
        self.assertEqual(pipeline._road_cones, [])


class RoadDropoutChecks(unittest.TestCase):
    XS = [-0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1]

    @classmethod
    def step(cls, pipeline, now, left=None, right=None, stamp=None,
             speed=0.5, yaw=0.2):
        left = [(x, 0.5) for x in cls.XS] if left is None else left
        right = [(x, -0.5) for x in cls.XS] if right is None else right
        stamp = now if stamp is None else stamp
        obs = {"cone_detections": cones(left, right), "wheel_speed": speed,
               "imu_angular_velocity": None if yaw is None else (0, 0, yaw)}
        return pipeline.update(obs, {"cone_detections": max(0, now-stamp),
                                    "wheel_speed": 0.0, "imu_angular_velocity": 0.0},
                               {"cone_detections": round(stamp*1e9),
                                "imu_angular_velocity": round(now*1e9)},
                               {"wheel_speed": now}, now)

    def test_single_side_failure_transports_last_good_geometry_and_keeps_source_time(self):
        pipeline = Pipeline(Settings())
        good = self.step(pipeline, 10)
        cached = list(pipeline._road_cones)
        out = self.step(pipeline, 10.1, left=[(0.6, 0.5)])
        road = out["road"]
        self.assertTrue(road["valid"])
        self.assertTrue(road["degraded"])
        self.assertEqual(road["status"], "predicted_history")
        self.assertEqual(road["measurement_timestamp_s"], 10.0)
        self.assertEqual(road["timestamp_s"], out["state"]["timestamp_s"])
        self.assertAlmostEqual(road["source_age_s"], 0.1)
        self.assertEqual(road["current_frame"]["timestamp_s"], 10.1)
        self.assertEqual(road["current_frame"]["status"], "single_side_width_unknown")
        self.assertEqual(road["current_frame"]["visibility"], "right_only")
        self.assertEqual(road["current_frame"]["boundary_diagnostics"]["left"]["raw_count"], 1)
        self.assertFalse(road["current_frame"]["boundary_diagnostics"]["left"]["fit_valid"])
        self.assertEqual(road["recommended_speed_scale"], 0.5)
        self.assertGreater(road["prediction_remaining_s"], 0)
        self.assertEqual(pipeline._road_cones, cached)
        self.assertEqual(pipeline._trusted_road["timestamp_s"], 10.0)
        pose, reason = pipeline.motion.pose_between(10, 10.1)
        self.assertIsNone(reason)
        for actual, before in zip(road["centerline_xy"], good["road"]["centerline_xy"]):
            expected = pipeline.motion.transform_xy(before, pose)
            self.assertAlmostEqual(actual[0], expected[0])
            self.assertAlmostEqual(actual[1], expected[1])
        self.assertFalse(any(key.startswith("_") for key in road))
        json.dumps(out, allow_nan=False)

    def test_next_good_frame_recovers_near_history_and_clears_degraded_status(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, speed=0.0, yaw=0.0)
        self.assertTrue(self.step(pipeline, 10.1, left=[], speed=0.0, yaw=0.0)["road"]["degraded"])
        out = self.step(pipeline, 10.2, left=[(x, 0.5) for x in self.XS if x >= 0.9],
                        right=[(x, -0.5) for x in self.XS if x >= 0.9], speed=0.0, yaw=0.0)
        self.assertTrue(out["road"]["valid"])
        self.assertFalse(out["road"]["degraded"])
        self.assertTrue(out["road"]["history_used"])
        self.assertEqual(out["road"]["measurement_timestamp_s"], 10.2)
        self.assertIsNone(pipeline._failure_since_s)

    def test_repeated_bad_batch_does_not_renew_either_deadline(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, speed=0.0, yaw=0.0)
        for now in (10.05, 10.1, 10.15, 10.2):
            out = self.step(pipeline, now, left=[], right=[], stamp=10.05, speed=0.0, yaw=0.0)
            self.assertTrue(out["road"]["valid"])
            self.assertEqual(out["road"]["measurement_timestamp_s"], 10.0)
            self.assertAlmostEqual(out["road"]["prediction_elapsed_s"], now-10.05)
            self.assertAlmostEqual(out["road"]["source_age_s"], now-10.0)
        out = self.step(pipeline, 10.25, left=[], right=[], stamp=10.05, speed=0.0, yaw=0.0)
        self.assertFalse(out["road"]["valid"])
        self.assertEqual(out["road"]["prediction_status"], "prediction_deadline_expired")
        self.assertEqual(out["road"]["centerline_xy"], [])
        self.assertIsNone(pipeline._trusted_road)

    def test_bad_batch_is_fitted_once_even_with_timer_reuse(self):
        from unittest.mock import patch
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, speed=0.0, yaw=0.0)
        original, calls = Pipeline.road, []
        def counted(obj, batch, stamp, age):
            calls.append(stamp)
            return original(obj, batch, stamp, age)
        with patch.object(Pipeline, "road", counted):
            for now in (10.05, 10.1, 10.15, 10.2):
                self.step(pipeline, now, left=[], right=[], stamp=10.05, speed=0.0, yaw=0.0)
        self.assertEqual(calls, [10_050_000_000])

    def test_failure_without_a_trusted_near_corridor_cannot_initialize(self):
        for startup in ("empty", "forward_only"):
            with self.subTest(startup=startup):
                pipeline = Pipeline(Settings())
                if startup == "forward_only":
                    self.step(pipeline, 10, left=[(x, 0.5) for x in self.XS if x >= 0.9],
                              right=[(x, -0.5) for x in self.XS if x >= 0.9])
                out = self.step(pipeline, 10.1, left=[], right=[])
                self.assertFalse(out["road"]["valid"])
                self.assertFalse(out["road"]["degraded"])
                self.assertEqual(out["road"]["prediction_status"], "no_trusted_road")

    def test_crossing_boundaries_are_not_hidden_by_history(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10)
        out = self.step(pipeline, 10.1, left=[(x, -0.5) for x in self.XS],
                        right=[(x, 0.5) for x in self.XS])
        self.assertFalse(out["road"]["valid"])
        self.assertEqual(out["road"]["status"], "invalid_width_or_order")
        self.assertIsNone(pipeline._trusted_road)

    def test_observed_single_side_conflict_rejects_the_old_corridor(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, speed=0.0, yaw=0.0)
        out = self.step(pipeline, 10.1, left=[(x, 0.8) for x in self.XS], right=[],
                        speed=0.0, yaw=0.0)
        self.assertFalse(out["road"]["valid"])
        self.assertEqual(out["road"]["status"], "observed_history_conflict")
        self.assertIsNone(pipeline._trusted_road)

    def test_missing_motion_and_motion_gap_cannot_predict(self):
        for mode in ("speed", "gyro", "gap"):
            with self.subTest(mode=mode):
                pipeline = Pipeline(Settings())
                self.step(pipeline, 10)
                out = self.step(pipeline, 10.2 if mode == "gap" else 10.1, left=[], right=[],
                                speed=None if mode == "speed" else 0.5,
                                yaw=None if mode == "gyro" else 0.2)
                self.assertFalse(out["road"]["valid"])
                self.assertFalse(out["road"]["degraded"])
                self.assertIsNone(pipeline._trusted_road)
                if mode == "gap":
                    self.assertEqual(out["road"]["prediction_status"], "motion_history_gap")

    def test_source_unavailable_or_stale_never_uses_prediction(self):
        for batch, age in ((None, 0.0), (cones([], []), 0.5)):
            pipeline = Pipeline(Settings())
            self.step(pipeline, 10)
            obs = {"cone_detections": batch, "wheel_speed": 0.5, "imu_angular_velocity": (0, 0, 0.2)}
            out = pipeline.update(obs, {"cone_detections": age, "wheel_speed": 0.0,
                                        "imu_angular_velocity": 0.0},
                                  {"cone_detections": 10_100_000_000, "imu_angular_velocity": 10_100_000_000},
                                  {"wheel_speed": 10.1}, 10.1)
            self.assertFalse(out["road"]["valid"])
            self.assertIsNone(pipeline._trusted_road)

    def test_stop_clock_and_out_of_order_camera_clear_prediction(self):
        for event in ("stop", "clock", "camera"):
            with self.subTest(event=event):
                pipeline = Pipeline(Settings())
                self.step(pipeline, 10)
                if event == "stop":
                    pipeline.motion.clear()
                elif event == "camera":
                    self.step(pipeline, 10.05, left=[], right=[])
                now = 9.9 if event == "clock" else 10.1
                out = self.step(pipeline, now, left=[], right=[],
                                stamp=10.02 if event == "camera" else now)
                self.assertFalse(out["road"]["valid"])
                self.assertIsNone(pipeline._trusted_road)
                if event == "camera":
                    self.assertEqual(out["road"]["status"], "out_of_order_camera")

    def test_real_camera_latency_consumes_motion_budget_without_retimestamping(self):
        pipeline = Pipeline(Settings(motion_max_interval_s=0.4))
        for t in (9.8, 9.9):
            for name, value in (("wheel_speed", 0.0), ("imu_angular_velocity", 0.0)):
                pipeline.motion.observe(name, value, t, t, t)
        self.assertTrue(self.step(pipeline, 10, stamp=9.8, speed=0.0, yaw=0.0)["road"]["valid"])
        out = self.step(pipeline, 10.1, left=[], right=[], stamp=9.9, speed=0.0, yaw=0.0)
        self.assertTrue(out["road"]["valid"])
        self.assertEqual(out["road"]["measurement_timestamp_s"], 9.8)
        self.assertAlmostEqual(out["road"]["source_age_s"], 0.3)
        self.assertAlmostEqual(out["road"]["current_frame"]["source_age_s"], 0.2)
        out = self.step(pipeline, 10.2, left=[], right=[], stamp=10, speed=0.0, yaw=0.0)
        self.assertFalse(out["road"]["valid"])
        self.assertEqual(out["road"]["prediction_status"], "prediction_deadline_expired")

    def test_near_observation_expiry_still_applies_during_prediction(self):
        pipeline = Pipeline(Settings())
        self.step(pipeline, 10, speed=0.0, yaw=0.0)
        for frame in range(1, 50):
            out = self.step(pipeline, 10+frame*0.1,
                            left=[(x, 0.5) for x in self.XS if x >= 0.9],
                            right=[(x, -0.5) for x in self.XS if x >= 0.9], speed=0.0, yaw=0.0)
            self.assertTrue(out["road"]["valid"])
        out = self.step(pipeline, 15, left=[], right=[], speed=0.0, yaw=0.0)
        self.assertFalse(out["road"]["valid"])
        self.assertIsNone(pipeline._trusted_road)

    def test_hold_can_be_disabled_and_cannot_be_unbounded(self):
        pipeline = Pipeline(Settings(road_hold_s=0.0))
        self.step(pipeline, 10)
        self.assertFalse(self.step(pipeline, 10.1, left=[], right=[])["road"]["valid"])
        for value in (-0.01, 0.251, float("nan")):
            with self.assertRaises(ValueError):
                Settings(road_hold_s=value)


if __name__ == "__main__":
    unittest.main()
