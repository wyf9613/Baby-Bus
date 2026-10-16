"""Upstream 3652e9b optional runtime path selection; ROS defaults remain local."""
from dataclasses import dataclass
import math
import time
from .geometry import finite_number, _path_lengths, _path_project, _path_sample, _path_tangent
from .control import control_path_geometry
from .lattice import _shadow_pose_points, _shadow_collision, _shadow_points


@dataclass
class PlannerControlSettings:
    """Opt-in consumer of the fresh lattice; baseline remains the default.

    There is no fixed source-trajectory TTL or steering/curvature admission
    threshold. Fresh feedback, rolling known-boundary checks and remaining
    preview coverage decide whether to use the path or this cycle's baseline.
    """
    enabled: bool = False

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("planner_control.enabled must be boolean")


class PlannerExecutionManager:
    """Geometry-only high-rate adapter; never publishes actions or requests Stop.

    A result's input pose/epoch survives arbitrarily long computation. The
    short execution reference is rebuilt at each CURRENT state timestamp;
    source age is retained separately and cannot be renewed by reception.
    Only the selected path is checked, without a controller rollout or cost.
    """
    def __init__(self, frame_id="base_link"):
        self.frame_id = frame_id
        self.clear()

    def clear(self, reason="waiting_for_planner"):
        self.active = None
        self.reason = reason
        self.last_job_id = None
        self.last_result_stamp_s = None
        self.last_result_status = None
        self.diagnostics = {}

    def accept(self, result):
        # A completed planning failure immediately selects baseline, rather
        # than silently continuing the previous successful trajectory.
        self.active = None
        self.last_job_id = result.get("job_id")
        self.last_result_stamp_s = result.get("input_stamp_s")
        self.last_result_status = result.get("status")
        self.reason = "planner_"+str(result.get("status", "missing_status")).lower()
        if result.get("status") != "VALID":
            return
        anchor = result.get("execution_anchor") or {}
        pose = anchor.get("pose")
        selected = result.get("selected") or {}
        path = selected.get("path")
        if (not isinstance(pose, (list, tuple)) or len(pose) != 3
                or not all(finite_number(v) for v in pose)
                or not isinstance(anchor.get("odometry_epoch"), int)
                or not finite_number(self.last_result_stamp_s)
                or not isinstance(path, (list, tuple)) or len(path) < 2
                or any(not isinstance(p, (list, tuple)) or len(p) != 3
                       or not all(finite_number(v) for v in p) for p in path)
                or any(math.hypot(b[0]-a[0], b[1]-a[1]) <= 1e-9
                       for a, b in zip(path, path[1:]))):
            self.reason = "planner_invalid_execution_payload"
            return
        self.active = {"path": [tuple(p) for p in path], "pose": list(pose),
            "epoch": anchor["odometry_epoch"], "stamp_s": self.last_result_stamp_s,
            "measurement_stamp_s": result.get("measurement_stamp_s"), "progress_m": 0.0}
        self.reason = "planner_ready"

    def select(self, road, state, pose, epoch, cfg, preview_m, target_speed_mps,
               control_period_s, execution_lifetime_s):
        started = time.perf_counter()
        now = state.get("timestamp_s")
        self.diagnostics = {"requested_source": "planner", "mode": "baseline",
            "fallback": True, "reason": self.reason, "timestamp_s": now,
            "job_id": self.last_job_id, "planner_status": self.last_result_status,
            "input_stamp_s": self.last_result_stamp_s,
            "source_age_s": now-self.last_result_stamp_s if finite_number(now)
                and finite_number(self.last_result_stamp_s) else None}

        def fallback(reason):
            self.diagnostics.update(reason=reason, elapsed_s=time.perf_counter()-started)
            return None

        active = self.active
        if active is None:
            return fallback(self.reason)
        if epoch != active["epoch"]:
            self.active = None
            self.reason = "motion_epoch_changed"
            return fallback(self.reason)
        if (state.get("frame_id") != self.frame_id or state.get("speed_valid") is not True
                or state.get("yaw_rate_valid") is not True or not finite_number(now)
                or now < active["stamp_s"] or not all(finite_number(v) for v in pose)):
            return fallback("execution_motion_unavailable")
        points = _shadow_pose_points(active["path"], active["pose"], pose)
        arc = _path_lengths(points)
        # Remove previously passed arc BEFORE projecting: a returning leg may
        # remain ahead; deleting by x sign would destroy genuine U bends.
        start = active["progress_m"]
        local = [_path_sample(points, arc, start)]
        local.extend(p for p, s in zip(points, arc) if s > start+1e-6)
        if len(local) < 2:
            return fallback("preview_coverage_exhausted")
        local_arc = _path_lengths(local)
        nearest = _path_project(local, local_arc)
        if nearest is None:
            return fallback("execution_branch_unavailable")
        progress = min(arc[-1], start+nearest[1])
        active["progress_m"] = progress
        remaining = arc[-1]-progress
        speed = abs(state["speed_mps"])
        needed = preview_m+speed*control_period_s
        self.diagnostics.update(progress_m=progress, remaining_length_m=remaining,
            preview_m=preview_m, required_coverage_m=needed,
            remaining_preview_time_s=max(0.0, remaining-preview_m)/speed if speed > 1e-9 else None,
            measurement_stamp_s=active["measurement_stamp_s"], odometry_epoch=epoch,
            current_pose=list(pose), execution_lifetime_s=execution_lifetime_s)
        if remaining+1e-9 < needed:
            return fallback("preview_coverage_exhausted")
        cropped = [_path_sample(points, arc, progress)]
        cropped.extend(p for p, s in zip(points, arc) if s > progress+1e-6)
        lengths = _path_lengths(cropped)
        # Use exactly the same geometric reader as the actual PID, including
        # its tangent smoothing beyond the preview. This is NOT a rollout.
        path = {"type": "ORDERED_CENTERLINE", "independent_variable": "s_m",
                "samples_xy": cropped, "range": [0.0, lengths[-1]], "preview_m": preview_m}
        try:
            geometry = control_path_geometry(path)
        except (ValueError, TypeError, ArithmeticError) as error:
            return fallback("execution_geometry: "+str(error))
        # Projecting again inside the controller may select a later segment.
        # Check coverage at ITS actual preview position as well.
        if lengths[-1]-geometry["closest_s_m"]+1e-9 < needed:
            return fallback("preview_coverage_exhausted")
        check = min(lengths[-1], max(needed,
            geometry["preview_s_m"]+0.15))  # Existing PID tangent window.
        sources = road.get("boundary_source") or {}
        boundaries = {side: _shadow_points(road.get(side+"_boundary_xy"))
            for side in ("left", "right") if sources.get(side) in ("observed", "observed_with_history")
            and road.get(side+"_boundary_xy")}
        if boundaries and (road.get("time_aligned") is not True
                or road.get("frame_id") != self.frame_id
                or not finite_number(road.get("timestamp_s"))
                or abs(road["timestamp_s"]-now) > 1e-6):
            return fallback("known_boundary_alignment_unavailable")
        sweep = []
        for i, p in enumerate(cropped):
            heading = _path_tangent(cropped, lengths, lengths[i], window=cfg.path_spacing_m/2)
            sweep.append((*p, math.atan2(heading[1], heading[0])))
        # Include the actual vehicle footprint and the geometric connection to
        # the closest reference point when tracking has a lateral error.
        collision = _shadow_collision([(0.0, 0.0, 0.0)], 0.0, boundaries, cfg)
        connector = math.hypot(*cropped[0])
        if connector > 1e-9:
            sweep.insert(0, (0.0, 0.0, 0.0))
        collision = collision or _shadow_collision(sweep, check+connector, boundaries, cfg)
        self.diagnostics.update(check_length_m=check, connector_length_m=connector,
            known_boundary_sides=list(boundaries), known_boundaries_xy=boundaries,
            collision=collision, remaining_path_xy=cropped,
            checked_through_preview=True)
        if collision:
            return fallback("known_boundary_collision")
        self.diagnostics.update(mode="planner", fallback=False, reason="planner_tracking",
            elapsed_s=time.perf_counter()-started)
        return {"schema_version": "planning_reference_v1", "planner_version": "lattice_execution",
            "trajectory_id": self.last_job_id, "timestamp_s": now, "valid_for_s": execution_lifetime_s,
            "frame_id": self.frame_id, "vehicle_reference_point": "cg_ground_projection",
            "valid": True, "status": "VALID", "reason": "planner_tracking", "path": path,
            "target_speed_mps": target_speed_mps, "stop_requested": False, "stop_reason": None,
            "simulation_only": False, "original_input_stamp_s": active["stamp_s"]}
