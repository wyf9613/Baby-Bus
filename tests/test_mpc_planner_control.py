"""MVP2.1 lattice -> MPC consumer (planner_control), without ROS.

Runs the actual run_policy_step / calculate_policy_actions source of this
branch (test_control's stand-in node) on the deployed car 05 centerline layers,
with the MPC on and the MVP PID off. The shadow worker is replaced by a
synchronous stand-in that runs the REAL ShadowLatticePlanner on each submitted
job and adds the same envelope fields as _shadow_worker_entry/ShadowWorker; the
real subprocess protocol itself is covered by test_shadow_lattice.py.

These checks cover reference selection, fallback and run fencing. They are not
a closed-loop, timing or physical collision result.
"""
import ast
from copy import deepcopy
import json
import math
from pathlib import Path
import runpy
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
control = runpy.run_path(str(Path(__file__).with_name("test_control.py")))
shadow = runpy.run_path(str(Path(__file__).with_name("test_shadow_lattice.py")))
deploy = runpy.run_path(str(ROOT/"tools"/"make_deploy_yaml.py"))
cns = control["ns"]
estimation_ns = control["api"]["namespace"]
source = control["api"]["SOURCE"]
tree = ast.parse(source.read_text(encoding="utf-8"))

# Definitions test_control's namespace lacks: road_centerline + ordered_xy,
# the MPC, and the MVP2.1 lattice. Estimator classes resolve their helpers in
# test_estimation's namespace, so those helpers go there too.
EXTRA = {"RoadCenterlinePlanner", "_estimation_order_boundary", "_path_points", "_path_lengths",
         "_path_sample", "_path_tangent", "_path_project",
         "MPCStop", "VehicleParamsSettings", "course_simulation_vehicle_params", "course_model_step",
         "MPCSettings", "mpc_vehicle_params", "reference_trajectory_from_planning", "_ordered_lead_in",
         "_estimation_solve", "reference_trajectory_from_road", "mpc_reference_errors",
         "mpc_path_projector", "VehicleModelMPC", "MPCController", "mpc_debug_json",
         "transport_control_reference", "_control_sample_points", "mpc_planning_reference",
         "ShadowSettings", "ShadowFrame", "ShadowLatticePlanner", "PlannerControlSettings",
         "PlannerExecutionManager"}
definitions = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
               and (n.name in EXTRA or (n.name.startswith("_shadow_") and n.name != "_shadow_worker_entry"))]
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(source), "exec"), cns)
for n in definitions:
    estimation_ns.setdefault(n.name, cns[n.name])
METHODS = {"_planner_preview_m", "_select_planner_reference", "_select_mpc_planner_trajectory",
           "_publish_planner_execution", "_shadow_clear", "_shadow_collect_road", "_shadow_offer",
           "_shadow_poll", "_mpc_uses_planning"}
policy_node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "PolicyNode")
methods = [n for n in policy_node.body if isinstance(n, ast.FunctionDef) and n.name in METHODS]
assert {n.name for n in methods} == METHODS
method_ns = cns
exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), method_ns)
Recorder, start = control["Recorder"], control["start"]
ShadowSettings, Planner = cns["ShadowSettings"], cns["ShadowLatticePlanner"]
PlannerControl, Manager = cns["PlannerControlSettings"], cns["PlannerExecutionManager"]
TIMING = ("step_s", "solve_s", "step_p95_s")


class SyncWorker:
    """Synchronous stand-in for ShadowWorker running the real planner."""
    def __init__(self, deliver=True):
        self.deliver, self.jobs, self.results = deliver, [], []

    def submit(self, job):
        job = json.loads(json.dumps(job, allow_nan=False))    # the real wire format
        self.jobs.append(job)
        if not self.deliver:
            return
        result = Planner(ShadowSettings(**job["settings"])).plan(job)
        result.update(generation=job["generation"], job_id=job["job_id"],
                      execution_anchor={"pose": job.get("shadow_pose"),
                                        "odometry_epoch": job.get("shadow_odometry_epoch")},
                      shadow_only=not job.get("planner_control_enabled"),
                      submitted_monotonic_s=job["submitted_monotonic_s"])
        self.results.append(json.loads(json.dumps(result, allow_nan=False)))

    def poll(self):
        return self.results.pop(0) if self.results else None

    def close(self):
        pass


def deployed(mode="mpc-shadow-centerline"):
    params = deploy["build"](mode, 3.0, v_exec_max=0.12, cones="yellow-left", car="05")[0]
    return params["/**/ai4r_policy"]["ros__parameters"]


def make_node(shadow_enabled=False, planner=False, worker=None, mode="mpc-shadow-centerline", **planner_cfg):
    p = deployed(mode)
    node = control["make_node"](simulated=False, mvp=True)
    for name in METHODS:
        setattr(node, name, method_ns[name].__get__(node))
    node.estimation_settings = cns["EstimationSettings"](**p["estimation"])
    node.motion_history = cns["EstimationMotionHistory"](node.estimation_settings)
    planning = dict(p["planning"])
    lattice = planning.pop("lattice")
    node.planning_settings = cns["PlanningSettings"](**planning)
    node.lattice_settings = cns["LatticeSettings"](**lattice)
    node.control_settings = cns["ControlSettings"](**p["control"])
    node.controller = cns["PolicyController"](node.control_settings)
    node.distance_limiter = cns["RunDistanceLimiter"](p["control"]["max_distance_m"],
                                                       node.control_settings.max_dt_s)
    node.sensor_timeout_s.update(p["sensor_timeout_s"])
    node.vehicle_settings = cns["VehicleParamsSettings"](**p["vehicle"])
    node.mpc_settings = cns["MPCSettings"](**p["mpc"])
    node.mpc_controller = node.mpc_held_reference = None
    node.mpc_debug_publisher = Recorder()
    node.policy_update_rate_hz = p["policy_update_rate_hz"]
    node.shadow_settings = ShadowSettings(**{**p.get("shadow", {}), "enabled": shadow_enabled})
    node.planner_control_settings = PlannerControl(enabled=planner, **planner_cfg)
    node.planner_preview_m = node._planner_preview_m()
    node.planner_execution = Manager(node.policy_frame_id)
    node._last_execution_mode = node._mpc_trajectory = None
    node.shadow_generation = node.shadow_job_id = 0
    node.shadow_last_submit_s = -math.inf
    node.shadow_handoff_estimate_s = node.shadow_settings.handoff_allowance_s
    node.shadow_last_result_s = node.shadow_pose_stamp_s = node.shadow_previous_reference = None
    node.shadow_road_history, node.shadow_collection_events = [], []
    node.shadow_pose, node.shadow_odometry_epoch, node.shadow_collection_reset_id = [0.0, 0.0, 0.0], 0, 0
    node.shadow_worker = None
    if shadow_enabled:
        node.shadow_worker = worker or SyncWorker()
        node.shadow_displays = []
        node._shadow_publish_path = lambda pub, points, stamp: node.shadow_displays.append((pub, points, stamp))
        for name in ("path", "prefix", "reference", "diagnostics"):
            setattr(node, "shadow_"+name+"_publisher", Recorder())
    if planner:
        node.planner_execution_publisher = Recorder()
    return node


def feed(node, center=0.0, speed=0.12, yaw=0.0):
    """Straight two-sided yellow-left / blue-right road, 0.7 m wide."""
    now, mono = node.test_clock.ros_ns, node.test_clock.monotonic
    node._store("wheel_speed", speed, None, mono, now)
    node._store("imu_angular_velocity", (0.0, 0.0, yaw), now, mono, now)
    xs = [0.15+0.15*i for i in range(15)]
    batch = control["api"]["cones"]([(x, center+0.35) for x in xs], [(x, center-0.35) for x in xs])
    batch["detections"] = ([(x, center+0.35, 0.0, node.estimation_settings.left_colour, 0.95) for x in xs]
                           + [(x, center-0.35, 0.0, node.estimation_settings.right_colour, 0.95) for x in xs])
    node._store("cone_detections", batch, now, mono, now)
    node.last_nonempty_cones = node.observations["cone_detections"]


def run(node, ticks, poll=True, **feed_args):
    """Start (first call only) and step 20 Hz control ticks; returns mpc_debug records."""
    if node.fsm_state != 3:
        feed(node, **feed_args)
        start(node)
        node.run_policy_step()
        ticks -= 1
    for _ in range(ticks):
        node.test_clock.advance(0.05)
        feed(node, **feed_args)
        node.run_policy_step()
        if poll and node.shadow_worker is not None:
            node._shadow_poll()
    assert node.fsm_state == 3, node.state_reason
    return [json.loads(m.data) for m in node.mpc_debug_publisher.messages]


def without_timing(record):
    return {k: v for k, v in record.items() if k not in TIMING}


def offset_result(node, y, length=1.2, job_id=99):
    """A VALID planner result anchored at the CURRENT pose/epoch, offset by y."""
    return {"status": "VALID", "input_stamp_s": node.estimation_output["state"]["timestamp_s"],
            "measurement_stamp_s": None, "job_id": job_id, "generation": node.shadow_generation,
            "execution_anchor": {"pose": list(node.shadow_pose), "odometry_epoch": node.shadow_odometry_epoch},
            "selected": {"path": [(length*i/48, y, 0.0) for i in range(49)]}}


class DisabledEquivalenceChecks(unittest.TestCase):
    def test_shadow_alone_does_not_change_any_mpc_output(self):
        baseline = run(make_node(), 24)
        node = make_node(shadow_enabled=True)
        with_shadow = run(node, 24)
        self.assertGreater(len(node.shadow_worker.jobs), 0)
        self.assertTrue(all(r["status"] == "VALID" for r in node.shadow_diagnostics_publisher.messages
                            for r in [json.loads(r.data)] if "status" in r and r["status"] != "IDLE"))
        self.assertEqual([without_timing(r) for r in baseline], [without_timing(r) for r in with_shadow])
        self.assertIn(baseline[-1]["branch"], ("shadow",))

    def test_planner_without_any_result_is_the_same_baseline_tick(self):
        baseline = run(make_node(), 12)
        node = make_node(shadow_enabled=True, planner=True, worker=SyncWorker(deliver=False))
        waiting = run(node, 12)
        for a, b in zip(baseline, waiting):
            self.assertEqual(b.pop("reference_selection"), "baseline")
            self.assertEqual(b.pop("reference_selection_reason"), "waiting_for_planner")
            self.assertEqual(without_timing(a), without_timing(b))


class PlannerSelectionChecks(unittest.TestCase):
    def test_real_lattice_result_is_aligned_cropped_and_tracked(self):
        node = make_node(shadow_enabled=True, planner=True)
        records = run(node, 30)
        planner = [r for r in records if r.get("reference_selection") == "planner"]
        self.assertGreater(len(planner), 20, [r.get("reference_selection_reason") for r in records])
        self.assertTrue(all(r["branch"] == "shadow" for r in planner))
        last = planner[-1]
        self.assertEqual(last["path_id"], node.planner_execution.last_job_id)
        self.assertLess(abs(last["e_y_m"]), 0.05)
        info = json.loads(node.planner_execution_publisher.messages[-1].data)
        self.assertEqual((info["mode"], info["fallback"]), ("planner", False))
        self.assertGreater(info["progress_m"], 0.0)       # passed arc was cropped
        self.assertGreater(info["source_age_s"], 0.0)     # original input time retained
        self.assertGreaterEqual(info["remaining_length_m"], node.planner_preview_m)
        green = node._trajectory_reference["path"]["samples_xy"]
        self.assertLess(math.hypot(*green[0]), 0.05)

    def test_offset_lattice_path_moves_the_mpc_error_not_the_baseline(self):
        node = make_node(shadow_enabled=True, planner=True, worker=SyncWorker(deliver=False))
        before = run(node, 6)[-1]
        self.assertEqual(before["reference_selection"], "baseline")
        node.planner_execution.accept(offset_result(node, 0.1))
        after = run(node, 2)[-1]
        self.assertEqual(after["reference_selection"], "planner")
        self.assertLess(abs(before["e_y_m"]), 0.02)
        self.assertAlmostEqual(after["e_y_m"], -0.1, delta=0.02)   # car is right of the path
        self.assertEqual(after["path_id"], 99)
        self.assertNotEqual(after["candidate_steer_action"], before["candidate_steer_action"])

    def test_baseline_rejection_blocks_the_lattice_unless_explicitly_allowed(self):
        for allowed in (False, True):
            node = make_node(shadow_enabled=True, planner=True, worker=SyncWorker(deliver=False),
                             require_valid_baseline=not allowed)
            run(node, 4)
            node.planner_execution.accept(offset_result(node, 0.0))
            estimates = deepcopy(node.estimation_output)
            node.planning_output = {"valid": False, "reason": "invalid_road_near_field_unobserved",
                                    "stop_requested": True}
            reference = node._select_planner_reference(estimates)
            if allowed:
                self.assertIsNotNone(reference)
            else:
                self.assertIsNone(reference)
                self.assertEqual(node.planner_execution.diagnostics["reason"],
                                 "baseline_invalid: invalid_road_near_field_unobserved")

    def test_path_too_short_for_the_mpc_horizon_falls_back_in_the_same_tick(self):
        node = make_node(shadow_enabled=True, planner=True, worker=SyncWorker(deliver=False))
        run(node, 4)
        stamp = node.estimation_output["state"]["timestamp_s"]+0.05
        short = {"schema_version": "planning_reference_v1", "planner_version": "lattice_execution",
                 "trajectory_id": 5, "timestamp_s": stamp, "valid_for_s": 0.25, "frame_id": "base_link",
                 "vehicle_reference_point": "cg_ground_projection", "valid": True, "status": "VALID",
                 "path": {"type": "ORDERED_CENTERLINE", "independent_variable": "s_m",
                          "samples_xy": [(0.0, 0.0), (0.04, 0.0), (0.08, 0.0)], "range": [0.0, 0.08]},
                 "target_speed_mps": 0.2, "stop_requested": False, "simulation_only": False}
        node.planner_execution.select = lambda *args: short
        record = run(node, 1)[-1]
        self.assertEqual(record["reference_selection"], "baseline")
        self.assertEqual(record["reference_selection_reason"], "mpc_reference_rejected: reference_too_short")
        self.assertEqual(record["branch"], "shadow")      # the baseline was solved this tick

    def test_motion_epoch_change_drops_the_path(self):
        node = make_node(shadow_enabled=True, planner=True, worker=SyncWorker(deliver=False))
        run(node, 4)
        node.planner_execution.accept(offset_result(node, 0.0))
        self.assertEqual(run(node, 1)[-1]["reference_selection"], "planner")
        node.shadow_odometry_epoch += 1
        record = run(node, 1)[-1]
        self.assertEqual((record["reference_selection"], record["reference_selection_reason"]),
                         ("baseline", "motion_epoch_changed"))


class RunFencingChecks(unittest.TestCase):
    def test_stop_clears_the_path_and_fences_late_results(self):
        worker = SyncWorker(deliver=False)
        node = make_node(shadow_enabled=True, planner=True, worker=worker)
        run(node, 4)
        node.planner_execution.accept(offset_result(node, 0.0))
        self.assertEqual(run(node, 1)[-1]["reference_selection"], "planner")
        late = offset_result(node, 0.0)
        generation, epoch = node.shadow_generation, node.shadow_odometry_epoch
        node._change_state(2, "Operator request")
        self.assertIsNone(node.planner_execution.active)
        self.assertEqual(json.loads(node.planner_execution_publisher.messages[-1].data)["mode"], "idle")
        self.assertEqual((node.shadow_generation, node.shadow_odometry_epoch), (generation+1, epoch+1))
        node.test_clock.advance(0.05)
        feed(node)
        start(node)
        node.run_policy_step()
        worker.results.append(late)                # previous run's generation
        node._shadow_poll()
        self.assertIsNone(node.planner_execution.active)
        node.test_clock.advance(0.05)
        feed(node)
        node.run_policy_step()
        record = json.loads(node.mpc_debug_publisher.messages[-1].data)
        self.assertEqual(record["reference_selection"], "baseline")


class PreviewDistanceChecks(unittest.TestCase):
    def test_preview_covers_the_mpc_horizon_travel(self):
        node = make_node(shadow_enabled=True, planner=True)
        vehicle, mpc = node.vehicle_settings, node.mpc_settings
        delay = max(vehicle.drive_delay_s, vehicle.steering_delay_s)
        for cap in (mpc.v_exec_max_mps, 0.5):
            stand_in = SimpleNamespace(planning_settings=node.planning_settings, vehicle_settings=vehicle,
                                       mpc_settings=cns["MPCSettings"](**{**vars(mpc), "v_exec_max_mps": cap}))
            expected = max(node.planning_settings.preview_m, cap*(mpc.horizon_n*mpc.dt_pred_s+delay))
            self.assertAlmostEqual(method_ns["_planner_preview_m"](stand_in), expected)
        self.assertGreater(method_ns["_planner_preview_m"](stand_in), node.planning_settings.preview_m)


if __name__ == "__main__":
    unittest.main()
