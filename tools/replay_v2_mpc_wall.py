#!/usr/bin/env python3
"""Replay Planning V2's synthetic obstacle wall through the local MPC.

This reads a *local Git ref* in memory; it does not fetch, merge, modify the
checkout, use ROS, or command a vehicle. The MPC vehicle is an offline model,
not measured car calibration. Run with a Python that has numpy installed:

    MPC_TEST_QP_SOLVER=dense python3 -B tools/replay_v2_mpc_wall.py

The default Planning ref is origin/mvp2.0. --planning-ref accepts any local
commit/ref containing scripts/policy_node.py; the tested snapshot was ae74be4.
"""

import argparse
import ast
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from numbers import Real
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tests import test_mpc as mpc_test  # Extracts the actual local policy definitions.


# This selector mirrors the pure-definition loader in Planning V2's
# tests/test_estimation.py. No Planning algorithm is copied into this tool.
PLANNING_NAMES = {
    "finite_number", "Observation", "EstimationSettings", "FirstOrderSampleFilter",
    "EstimationMotionHistory", "_estimation_median", "_estimation_solve",
    "_estimation_fit_boundary", "EstimationPipeline", "PlanningSettings",
    "CenterlinePlanner", "evaluate_planning_path", "planning_vehicle_limits",
    "ControlSettings", "VehicleCalibrationSettings", "ControlPID", "PolicyController",
    "wrap_angle", "PolicyStopRequest", "RunDistanceLimiter", "ControlReferenceManager",
    "_control_poly_eval", "_control_poly_roots", "control_path_geometry", "LatticeSettings",
    "LatticeCurve", "FrenetLatticePlanner", "_lattice_interp", "_lattice_derivative",
    "_lattice_lateral", "_lattice_geometry", "_lattice_steering",
    "evaluate_frenet_path", "_planning_control_geometry", "_control_cached_geometry",
    "_control_sample_points", "_control_sample_at", "MvpRecoveryPlanner", "_mvp_plan",
    "transport_control_reference", "control_reference_snapshot", "AsyncLatticePlanner",
}


def planning_namespace(revision):
    source_name = f"{revision}:scripts/policy_node.py"
    source = subprocess.check_output(["git", "show", source_name], cwd=ROOT, text=True)
    tree = ast.parse(source, filename=source_name)
    definitions = [node for node in tree.body
                   if isinstance(node, (ast.FunctionDef, ast.ClassDef))
                   and node.name in PLANNING_NAMES]
    names_found = {node.name for node in definitions}
    required = {"PlanningSettings", "LatticeSettings", "FrenetLatticePlanner"}
    if not required <= names_found:
        raise RuntimeError(f"Planning ref lacks V2 definitions: {sorted(required-names_found)}")
    namespace = {
        "dataclass": dataclass, "math": math, "deepcopy": deepcopy, "json": json,
        "Path": Path, "__file__": source_name, "queue": queue, "subprocess": subprocess,
        "sys": sys, "threading": threading, "time": time, "Real": Real,
        "ConeDetection": SimpleNamespace(COLOR_BLUE=2, COLOR_YELLOW=1),
    }
    exec(compile(ast.Module(body=definitions, type_ignores=[]), source_name, "exec"), namespace)
    return namespace


def wall_inputs():
    """Same 1.5 m obstacle-wall geometry as V2 test_lattice.py (synthetic)."""
    xs = [i*0.1 for i in range(31)]
    road = {"valid": True, "frame_id": "base_link", "timestamp_s": 10.0,
            "source_age_s": 0.0, "visibility": "both", "local_curvature_1pm": 0.0,
            "centerline_xy": [(x, 0.0) for x in xs],
            "left_boundary_xy": [(x, 0.5) for x in xs],
            "right_boundary_xy": [(x, -0.5) for x in xs]}
    state = {"speed_valid": True, "yaw_rate_valid": True, "frame_id": "base_link",
             "timestamp_s": 10.0, "speed_mps": 0.1, "yaw_rate_rps": 0.0,
             "source_age_s": {"wheel_speed": 0.01, "imu_angular_velocity": 0.01}}
    obstacles = {"available": True, "timestamp_s": 10.0, "source_age_s": 0.01,
                 "frame_id": "base_link", "points_xyz_m":
                 [(1.5, y, 0) for y in (-0.45, -0.3, -0.15, 0, 0.15, 0.3, 0.45)]}
    limits = {"valid": True, "source": "offline_test_assumption",
              "vehicle_reference_point": "cg_ground_projection",
              "footprint_xy_m": [(-0.15, -0.1), (0.2, -0.1), (0.2, 0.1), (-0.15, 0.1)],
              "speed_max_mps": 0.5, "braking_deceleration_mps2": 0.5,
              "actuation_delay_s": 0.1, "safety_margin_m": 0.05}
    return road, state, obstacles, limits, 10.0


def replay(namespace, direct):
    planner = namespace["FrenetLatticePlanner"](
        namespace["PlanningSettings"](algorithm="lattice_v2", max_source_age_s=0.4,
                                      reference_lifetime_s=0.2),
        namespace["LatticeSettings"](budget_s=5, clear_start_assumed=True,
                                    direct_sample_output=direct), clock=lambda: 0.0)
    reference, diagnostic = planner.plan(*wall_inputs())
    assert reference["valid"] and reference["planned_stop"]
    assert diagnostic["behavior"] == "obstacle_stop"
    assert reference["speed_profile"][-1][1] == 0.0
    trajectory = mpc_test.adapt(reference, 0.05)
    assert trajectory["valid"] and not trajectory["stop_required"], trajectory["reason"]

    # A deliberately synthetic vehicle model: useful for software replay only.
    vehicle = mpc_test.measured(drive_deadband=0.3, brake_gain_n=5.0)
    settings = mpc_test.Settings(enabled=True, shadow=True, vehicle_params_source="vehicle",
                                 qp_solver="dense", max_step_time_s=1.0)
    controller = mpc_test.Controller(settings, vehicle)
    shadow = controller.step(trajectory, 0.1, 0.0, 10.0, True)
    debug = shadow["debug"]
    assert debug["branch"] == "shadow", debug["reject_reason"]
    assert (shadow["drive"], shadow["steer"]) == (0.0, 0.0)
    assert debug["candidate_drive"] < vehicle.drive_deadband

    # A one-horizon prediction, not a full 3 s stopping simulation. This wall
    # fixture is straight, so CG x is comparable with stop_at_path_m.
    z, _ = controller.initial_state(0.1)
    for drive, steer in controller.plan[1]:
        z = mpc_test.model_step(z, (drive, steer), settings.dt_pred_s, vehicle)
    assert math.isclose(z[3], debug["pred_v_end_mps"], abs_tol=1e-8)
    remaining_m = reference["stop_at_path_m"] - z[0]
    assert remaining_m > 0, "predicted first horizon crossed the Planning stop point"
    required_decel = z[3]**2/(2*remaining_m)
    model_neutral_decel = vehicle.brake_gain_n*vehicle.drive_deadband/vehicle.mass_kg

    no_brake_model = mpc_test.measured(drive_deadband=0.3, brake_gain_n=0.0)
    refused = mpc_test.Controller(settings, no_brake_model).step(
        trajectory, 0.1, 0.0, 10.0, True)
    assert refused["debug"]["branch"] == "stop"
    assert refused["debug"]["reject_reason"] == "braking_model_unavailable"

    executing = mpc_test.verified(vehicle=vehicle)
    try:
        executing.step(trajectory, 0.1, 0.0, 10.0, True)
    except mpc_test.Stop:
        pass
    else:
        raise AssertionError("planned stop unexpectedly reached MPC execution")
    assert executing.last_debug["branch"] == "stop"

    return {"encoding": reference["path"]["type"],
            "planning_stop_at_path_m": reference["stop_at_path_m"],
            "planning_profile_end_s": reference["speed_profile"][-1][0],
            "mpc_prediction_end_s": settings.horizon_n*settings.dt_pred_s,
            "mpc_predicted_x_m": z[0], "mpc_predicted_speed_mps": z[3],
            "remaining_to_planned_stop_m": remaining_m,
            "minimum_deceleration_after_horizon_mps2": required_decel,
            "offline_model_neutral_deceleration_mps2": model_neutral_decel,
            "mpc_target_end_speed_mps": debug["v_ref_end_mps"],
            "candidate_drive": debug["candidate_drive"],
            "applied_drive": shadow["drive"],
            "execution_branch": executing.last_debug["branch"],
            "missing_brake_model_branch": refused["debug"]["branch"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planning-ref", default="origin/mvp2.0",
                        help="local Planning V2 Git ref; no fetch is performed")
    args = parser.parse_args()
    revision = subprocess.check_output(["git", "rev-parse", args.planning_ref],
                                       cwd=ROOT, text=True).strip()
    namespace = planning_namespace(revision)
    reports = [replay(namespace, direct) for direct in (False, True)]
    print(json.dumps({"planning_commit": revision, "offline_model_only": True,
                      "full_stop_position_verified": False, "results": reports},
                     indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
