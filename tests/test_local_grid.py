"""Portable geometry/temporal checks of the actual single-file grid implementation."""
import ast
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from numbers import Real
from pathlib import Path
from types import SimpleNamespace
from types import MethodType
import time
import unittest

SOURCE = Path(__file__).resolve().parents[1]/"scripts/policy_node.py"
try:
    from ament_index_python.packages import get_package_prefix
except ImportError:
    pass
else:
    SOURCE = Path(get_package_prefix("ai4r_policy"))/"lib/ai4r_policy/policy_node.py"
tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
names = {"LocalGridSettings", "LocalOccupancyGrid", "_local_grid_ray", "finite_number",
         "EstimationMotionHistory", "_local_grid_rgb", "_local_grid_scan_summary"}
definitions = [node for node in tree.body
               if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in names]
namespace = dict(dataclass=dataclass, math=math, Real=Real)
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(SOURCE), "exec"), namespace)
Settings, Grid = namespace["LocalGridSettings"], namespace["LocalOccupancyGrid"]
Motion = namespace["EstimationMotionHistory"]


class KnownMotion:
    """An independently specified synthetic pose at the NEW scan time."""
    def __init__(self, dx=0.0, dy=0.0, yaw=0.0, missing=False):
        self.dx, self.dy, self.yaw, self.missing = dx, dy, yaw, missing

    def pose_between(self, start, end):
        if self.missing:
            return None, "missing_motion_history"
        moved = start < end-1e-9
        return dict(dx_m=self.dx if moved else 0.0, dy_m=self.dy if moved else 0.0,
                    yaw_rad=self.yaw if moved else 0.0), None

    transform_xy = staticmethod(Motion.transform_xy)


def value(grid, result, x, y, inflated=False):
    col = math.floor(x/grid.settings.resolution_m)
    row = math.floor((y+1.5)/grid.settings.resolution_m)
    return result["inflated_data" if inflated else "data"][row*grid.cells+col]


class LocalGridChecks(unittest.TestCase):
    def test_forward_rectangle_ray_free_hit_and_occlusion(self):
        grid = Grid(Settings())
        grid.add_scan(10.0, (0.2, 0.025), [(1.025, 0.025), (-1.0, 0.025)])
        result = grid.render(10.0, KnownMotion())
        self.assertEqual(len(result["data"]), 60*60)
        self.assertEqual(value(grid, result, 0.525, 0.025), 0)
        self.assertEqual(value(grid, result, 1.025, 0.025), 100)
        self.assertEqual(value(grid, result, 1.525, 0.025), -1)
        self.assertEqual(result["occupied_cells"], 1)  # Rear hit is outside the map.

    def test_rays_start_at_lidar_not_vehicle_centre(self):
        grid = Grid(Settings())
        grid.add_scan(10.0, (0.225, 0.025), [(1.025, 0.025)])
        result = grid.render(10.0, KnownMotion())
        self.assertEqual(value(grid, result, 0.025, 0.025), -1)
        self.assertEqual(value(grid, result, 0.325, 0.025), 0)

    def test_outside_hit_clears_to_edge_without_inventing_boundary_wall(self):
        grid = Grid(Settings())
        grid.add_scan(10.0, (0.2, 0.025), [(8.0, 0.025), (1.0, 5.0)])
        result = grid.render(10.0, KnownMotion())
        self.assertEqual(value(grid, result, 2.975, 0.025), 0)
        self.assertEqual(result["occupied_cells"], 0)

    def test_new_free_ray_clears_old_hit_without_clearing_behind_new_hit(self):
        grid = Grid(Settings())
        grid.add_scan(10.0, (0.2, 0.025), [(1.025, 0.025)])
        grid.add_scan(10.1, (0.2, 0.025), [(2.025, 0.025)])
        result = grid.render(10.1, KnownMotion())
        self.assertEqual(value(grid, result, 1.025, 0.025), 0)
        self.assertEqual(value(grid, result, 2.025, 0.025), 100)
        self.assertEqual(value(grid, result, 2.525, 0.025), -1)

    def test_same_scan_hit_wins_over_other_clearing_rays(self):
        grid = Grid(Settings())
        grid.add_scan(10.0, (0.2, 0.025), [(1.025, 0.025), (2.025, 0.025)])
        result = grid.render(10.0, KnownMotion())
        self.assertEqual(value(grid, result, 1.025, 0.025), 100)

    def test_static_obstacle_moves_back_with_forward_vehicle_motion(self):
        grid = Grid(Settings())
        grid.add_scan(10.0, (0.2, 0.025), [(1.025, 0.025)])
        grid.add_scan(10.1, (0.2, 0.025), [])
        result = grid.render(10.1, KnownMotion(dx=0.2))
        self.assertEqual(value(grid, result, 0.825, 0.025), 100)
        self.assertNotEqual(value(grid, result, 1.025, 0.025), 100)

    def test_static_obstacle_rotates_opposite_vehicle_yaw(self):
        grid = Grid(Settings())
        grid.add_scan(10.0, (0.2, 0.0), [(1.025, 0.525)])
        grid.add_scan(10.1, (0.2, 0.0), [])
        result = grid.render(10.1, KnownMotion(yaw=math.pi/2))
        self.assertEqual(value(grid, result, 0.525, -1.025), 100)

    def test_inflation_is_separate_and_does_not_include_car_size(self):
        grid = Grid(Settings(inflation_radius_m=0.10))
        grid.add_scan(10.0, (0.2, 0.025), [(1.025, 0.025)])
        result = grid.render(10.0, KnownMotion())
        self.assertEqual(value(grid, result, 1.025, 0.025, True), 100)
        self.assertEqual(value(grid, result, 1.025, 0.125, True), 90)
        self.assertEqual(value(grid, result, 1.025, 0.125), -1)
        self.assertEqual(value(grid, result, 1.025, 0.225, True), -1)

    def test_repeated_and_out_of_order_scans_do_not_add_evidence(self):
        grid = Grid(Settings())
        self.assertTrue(grid.add_scan(10.0, (0.2, 0), [(1.0, 0)]))
        self.assertFalse(grid.add_scan(10.0, (0.2, 0), [(2.0, 0)]))
        self.assertFalse(grid.add_scan(9.9, (0.2, 0), [(2.0, 0)]))
        first = grid.render(10.0, KnownMotion())
        second = grid.render(10.0, KnownMotion())
        self.assertEqual(first["data"], second["data"])
        self.assertEqual(second["scan_count"], 1)

    def test_old_unobserved_hit_expires_to_unknown(self):
        grid = Grid(Settings(history_s=0.3))
        grid.add_scan(10.0, (0.2, 0), [(1.025, 0.025)])
        grid.add_scan(10.4, (0.2, 0), [])
        result = grid.render(10.4, KnownMotion())
        self.assertEqual(value(grid, result, 1.025, 0.025), -1)
        self.assertEqual(result["scan_count"], 1)

    def test_no_motion_uses_only_latest_acquisition_frame_not_fake_current_time(self):
        grid = Grid(Settings())
        grid.add_scan(10.0, (0.2, 0), [(1.025, 0.025)])
        grid.add_scan(10.1, (0.2, 0), [(2.025, 0.525)])
        result = grid.render(10.15, KnownMotion(missing=True))
        self.assertFalse(result["time_aligned"])
        self.assertEqual(result["timestamp_s"], 10.1)
        self.assertEqual(result["scan_count"], 1)
        self.assertNotEqual(value(grid, result, 1.025, 0.025), 100)
        self.assertEqual(value(grid, result, 2.025, 0.525), 100)

    def test_invalid_and_no_return_points_never_clear_unknown(self):
        grid = Grid(Settings())
        grid.add_scan(10.0, (0.2, 0), [(math.inf, 0), (math.nan, 1)])
        result = grid.render(10.0, KnownMotion())
        self.assertTrue(all(cell == -1 for cell in result["data"]))

    def test_stale_or_reset_output_clears_both_layers(self):
        grid = Grid(Settings())
        grid.add_scan(10.0, (0.2, 0), [(1.0, 0)])
        grid.clear()
        result = grid.empty(10.5, "SCAN_OR_TF_UNAVAILABLE")
        self.assertFalse(result["valid"])
        self.assertEqual(set(result["data"]), {-1})
        self.assertEqual(set(result["inflated_data"]), {-1})

    def test_sustained_input_has_bounded_storage(self):
        grid = Grid(Settings(history_s=0.5))
        for i in range(100):
            grid.add_scan(10+i/100, (0.2, 0), [(1.0, 0)])
        self.assertLessEqual(len(grid.scans), 16)


class Envelope:
    """Only message field containers; no claim of ROS middleware validation."""
    def __init__(self):
        self.header = SimpleNamespace()
        self.info = SimpleNamespace(origin=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)))


class Stamp:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        return SimpleNamespace(sec=self.nanoseconds//10**9, nanosec=self.nanoseconds % 10**9)


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class LocalGridAdapterChecks(unittest.TestCase):
    def node(self):
        policy_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "PolicyNode")
        methods = deepcopy([n for n in policy_class.body if isinstance(n, ast.FunctionDef)
                            and n.name in ("_local_grid_tick", "_local_grid_update", "_age")])
        scope = dict(namespace, time=time, json=json, OccupancyGrid=Envelope, Image=Envelope, Time=Stamp,
                     String=lambda **kw: SimpleNamespace(**kw))
        # _age is static inside PolicyNode, but plain when extracted here.
        for method in methods:
            method.decorator_list = []
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), "exec"), scope)
        settings = Settings(enabled=True)
        motion = KnownMotion(missing=True)
        motion.reset_id = 0
        node = SimpleNamespace(local_grid_settings=settings, local_grid=Grid(settings),
            motion_history=motion, _local_grid_motion_reset_id=0,
            _lidar_grid_origin=(0.2, 0.025), policy_frame_id="base_link", fresh=True,
            observations={"lidar_cartesian": SimpleNamespace(stamp_ns=10_000_000_000,
                received_at=5.0, value={"points_xyz": [(1.025, 0.025, 0.12)]})},
            _times=lambda: (5.1, 10_100_000_000), warnings=[], actions=[])
        node._fresh = lambda *args: node.fresh
        node._age = scope["_age"]
        node._warn = lambda *args: node.warnings.append(args)
        node.run_policy_step = lambda: node.actions.append("run")
        node._change_state = lambda *args: node.actions.append("state")
        for name in ("local_grid", "local_inflated_grid", "local_grid_status", "local_grid_image"):
            setattr(node, name+"_publisher", Recorder())
        node._local_grid_tick = MethodType(scope["_local_grid_tick"], node)
        node._local_grid_update = MethodType(scope["_local_grid_update"], node)
        return node

    def test_scan_only_envelope_has_correct_frame_stamp_origin_shape_and_no_actions(self):
        node = self.node()
        node._local_grid_tick()
        message = node.local_grid_publisher.messages[-1]
        self.assertEqual(message.header.frame_id, "base_link")
        self.assertEqual(message.header.stamp.sec, 10)
        self.assertEqual(message.header.stamp.nanosec, 0)  # Not fake current time 10.1.
        self.assertEqual((message.info.width, message.info.height), (60, 60))
        self.assertEqual(message.info.origin.position.y, -1.5)
        self.assertEqual(message.info.origin.orientation.w, 1.0)
        self.assertEqual(len(message.data), 3600)
        self.assertIn(100, message.data)
        status = json.loads(node.local_grid_status_publisher.messages[-1].data)
        self.assertEqual(status["status"], "SCAN_ONLY_NO_MOTION")
        self.assertFalse(status["inflation_includes_vehicle_size"])
        self.assertEqual(node.actions, [])
        image = node.local_grid_image_publisher.messages[-1]
        self.assertEqual((image.width, image.height, image.step, image.encoding), (120, 120, 360, "rgb8"))
        self.assertEqual(len(image.data), image.height*image.step)
        self.assertEqual(image.header.stamp.sec, message.header.stamp.sec)

    def test_expired_sensor_clears_published_maps_and_history(self):
        node = self.node()
        node._local_grid_tick()
        node._local_grid_tick()
        self.assertEqual(len(node.local_grid.scans), 1)
        node.fresh = False
        node._local_grid_tick()
        self.assertEqual(set(node.local_grid_publisher.messages[-1].data), {-1})
        self.assertEqual(set(node.local_inflated_grid_publisher.messages[-1].data), {-1})
        self.assertFalse(node.local_grid_output["valid"])
        self.assertEqual(node.local_grid.scans, [])
        image = node.local_grid_image_publisher.messages[-1]
        self.assertEqual(set(zip(*(iter(image.data),)*3)), {(216, 223, 230)})

    def test_publication_error_does_not_enter_control_or_escape_callback(self):
        node = self.node()
        def unavailable(message):
            raise RuntimeError("synthetic publication failure")
        node.local_grid_publisher.publish = unavailable
        node._local_grid_tick()
        self.assertEqual(node.actions, [])
        self.assertEqual(len(node.warnings), 1)

    def test_palette_preserves_unknown_free_hit_and_margin_and_orientation(self):
        # Row-major map: y=-left row first, x forward columns.
        rgb = namespace["_local_grid_rgb"]([-1, 100, 0, 90], 2)
        def pixel(x, y):
            offset = 3*(y*4+x)
            return tuple(rgb[offset:offset+3])
        self.assertEqual(pixel(0, 0), (251, 146, 60))  # Front-left margin.
        self.assertEqual(pixel(3, 0), (239, 68, 68))  # Front-right hit.
        self.assertEqual(pixel(0, 3), (250, 252, 254))  # Near-left free.
        self.assertEqual(pixel(3, 3), (216, 223, 230))  # Near-right UNKNOWN.

    def test_raw_scan_diagnostics_distinguish_missing_returns(self):
        summary = namespace["_local_grid_scan_summary"]({"ranges": [1.0, math.inf, math.nan, 0, 20],
                                                       "range_min": 0.05, "range_max": 12.0})
        self.assertEqual(summary, {"total_rays": 5, "valid_returns": 1,
                                   "positive_inf_rays": 1, "other_invalid_rays": 3})

    def test_current_raw_scan_counters_and_grid_coverage_are_published(self):
        node = self.node()
        node.observations["lidar_scan"] = SimpleNamespace(value={"ranges": [1.0, math.inf],
            "range_min": 0.05, "range_max": 12.0})
        node._local_grid_tick()
        status = json.loads(node.local_grid_status_publisher.messages[-1].data)
        self.assertEqual(status["scan"]["positive_inf_rays"], 1)
        self.assertEqual(status["free_cells"]+status["unknown_cells"]+status["occupied_cells"], 3600)
        node.fresh = False
        node._local_grid_tick()
        status = json.loads(node.local_grid_status_publisher.messages[-1].data)
        self.assertIsNone(status["scan"])
        self.assertEqual(status["unknown_cells"], 3600)


if __name__ == "__main__":
    unittest.main()
