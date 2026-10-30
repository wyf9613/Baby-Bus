"""Shared ROS/simulation execution with MVP2.1 geometry and baseline fallback."""
from copy import deepcopy
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
    def __init__(self, frame_id="base_link", generation=None):
        self.frame_id = frame_id
        self.clear(generation=generation)

    def clear(self, reason="waiting_for_planner", generation=None):
        self.generation = generation
        self.active = None
        self.reason = reason
        self.last_job_id = None
        self.last_result_stamp_s = None
        self.last_result_status = None
        self.diagnostics = {}
        self.latest_input_stamp_s = None
        self.last_rejected_result = None

    def accept(self, result):
        # Reject obsolete transport results before touching the current path.
        # This does not add a road/geometry admission condition.
        identifier, stamp = result.get("job_id"), result.get("input_stamp_s")
        reason = None
        transport_failure = result.get("status") == "TRANSPORT_ERROR"
        if not transport_failure and self.generation is not None and result.get("generation") != self.generation:
            reason = "old_generation"
        elif identifier is not None and (type(identifier) is not int or identifier < 0):
            reason = "invalid_job_id"
        elif identifier is not None and self.last_job_id is not None and identifier <= self.last_job_id:
            reason = "old_job"
        elif (finite_number(stamp) and finite_number(self.latest_input_stamp_s)
                and stamp < self.latest_input_stamp_s):
            reason = "old_input_stamp"
        if reason:
            self.last_rejected_result = dict(reason=reason, generation=result.get("generation"), job_id=identifier)
            return False
        if self.generation is None and result.get("generation") is not None:
            self.generation = result["generation"]
        if finite_number(stamp):
            self.latest_input_stamp_s = stamp
        # A completed planning failure immediately selects baseline, rather
        # than silently continuing the previous successful trajectory.
        self.active = None
        if identifier is not None:
            self.last_job_id = identifier
        self.last_result_stamp_s = result.get("input_stamp_s")
        self.last_result_status = result.get("status")
        self.reason = "planner_"+str(result.get("status", "missing_status")).lower()
        if result.get("status") != "VALID":
            return True
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
            return True
        self.active = {"path": [tuple(p) for p in path], "pose": list(pose),
            "job_id": identifier, "maneuver": dict(result.get("avoidance") or {}),
            "epoch": anchor["odometry_epoch"], "stamp_s": self.last_result_stamp_s,
            "measurement_stamp_s": result.get("measurement_stamp_s"), "progress_m": 0.0}
        self.reason = "planner_ready"
        return True

    def prepare(self, estimates, baseline, source, cfg, planning, control, update_rate_hz):
        """Same path selection for both callers; baseline geometry is not a gate."""
        try:
            road = estimates.get("road") or {}
            if not any(road.get(k) for k in ("centerline_xy", "left_boundary_xy", "right_boundary_xy")):
                road = road.get("forward_geometry") or {}
            state = estimates["state"]
            source._shadow_collect_road(road, state["timestamp_s"])
            if baseline.get("valid") and baseline.get("stop_requested"):
                self.diagnostics = dict(mode="baseline", fallback=True,
                    reason="baseline_stop_request", requested_source="planner")
                return None
            period = 1/update_rate_hz
            return self.select(road, state, source.shadow_pose, source.shadow_odometry_epoch,
                cfg, planning.preview_m, planning.cruise_speed_mps, period, control.max_dt_s+period)
        except Exception as error:
            self.diagnostics = dict(mode="baseline", fallback=True, requested_source="planner",
                reason="execution_exception: "+str(error))
            return None

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
        self.diagnostics.update(job_id=active.get('job_id', self.last_job_id),
            latest_result_job_id=self.last_job_id, input_stamp_s=active['stamp_s'],
            source_age_s=now-active['stamp_s'] if finite_number(now) else None)
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
        reference = {"schema_version": "planning_reference_v1", "planner_version": "lattice_execution",
            "trajectory_id": active.get('job_id', self.last_job_id), "timestamp_s": now, "valid_for_s": execution_lifetime_s,
            "frame_id": self.frame_id, "vehicle_reference_point": "cg_ground_projection",
            "valid": True, "status": "VALID", "reason": "planner_tracking", "path": path,
            "target_speed_mps": target_speed_mps, "stop_requested": False, "stop_reason": None,
            "simulation_only": False, "original_input_stamp_s": active["stamp_s"]}
        marker = active.get("maneuver", {})
        if marker.get("maneuver_task_id") is not None:
            reference.update(maneuver_task_id=marker['maneuver_task_id'],
                             maneuver_revision=marker['maneuver_revision'])
        return reference


def priming_motion_history(baseline, estimates, has_control, elapsed_s, control):
    """Preserve the upstream bounded neutral start before the first control step."""
    return (not baseline['valid'] and not has_control and estimates['state']['valid']
        and estimates['road']['status'] in ('missing_motion_history',
            'boundary_colour_assignment_pending', 'boundary_colour_assignment_ambiguous')
        and elapsed_s < control.startup_grace_s)


def calculate_selected_control(controller, baseline, planner_reference, state, vehicle_params,
                               now_s, dt, execution=None, baseline_provider=None,
                               obstacle_stop=None, obstacles=None, lidar_avoidance=None):
    """Run the existing PID with MVP2.1 same-tick rollback/fallback semantics."""
    reference = planner_reference if planner_reference is not None else baseline
    # A body-forward stop is independent of reference choice. It must bypass
    # the normal planner-controller rollback and same-tick baseline fallback.
    decision = obstacle_stop.evaluate(obstacles, now_s) if obstacle_stop is not None else None
    if decision is not None and decision['stop_required']:
        controller.clear()
        diagnostics = dict(valid=True, stop_requested=True, reason=decision['reason'],
                           obstacle_stop=decision, reference_source='obstacle_stop')
        if execution is not None:
            execution.diagnostics.update(mode='obstacle_stop', fallback=False,
                reason=decision['reason'], drive_action=0., steering_action=0., control_dt_s=dt)
        return controller, 0., 0., diagnostics, reference

    def avoidance_stop(ref):
        result = lidar_avoidance.evaluate(ref, now_s) if lidar_avoidance is not None else None
        if result is None or not result['stop_required']:
            return None
        controller.clear()
        diagnostics = dict(valid=True, stop_requested=True, reason=result['reason'],
            lidar_avoidance=result, reference_source='avoidance_stop')
        if execution is not None:
            execution.diagnostics.update(mode='avoidance_stop', fallback=False,
                reason=result['reason'], drive_action=0., steering_action=0., control_dt_s=dt)
        return controller, 0., 0., diagnostics, ref

    stopped = avoidance_stop(reference)
    if stopped is not None:
        return stopped
    previous = deepcopy(controller) if planner_reference is not None else None
    try:
        drive, steer, diagnostics = controller.calculate(reference, state, vehicle_params, now_s, dt)
    except Exception as error:
        if planner_reference is None:
            raise
        diagnostics = dict(valid=False, stop_requested=True, reason="planner_controller_exception: "+str(error))
    if planner_reference is not None and (not diagnostics["valid"] or diagnostics["stop_requested"]):
        controller = previous
        execution.diagnostics.update(mode="baseline", fallback=True,
            reason="planner_control_rejected: "+str(diagnostics.get("reason")))
        reference = baseline_provider() if baseline_provider is not None else baseline
        # The old fallback is also a motion request. A blocked centerline must
        # not bypass the same rolling LiDAR admission after native rejection.
        stopped = avoidance_stop(reference)
        if stopped is not None:
            return stopped
        drive, steer, diagnostics = controller.calculate(reference, state, vehicle_params, now_s, dt)
    if execution is not None:
        diagnostics["reference_source"] = execution.diagnostics.get("mode")
        diagnostics["reference_selection_reason"] = execution.diagnostics.get("reason")
        execution.diagnostics.update(drive_action=drive, steering_action=steer, control_dt_s=dt)
    if decision is not None:
        diagnostics['obstacle_stop'] = decision
    if lidar_avoidance is not None:
        diagnostics['lidar_avoidance'] = dict(lidar_avoidance.diagnostics)
    return controller, drive, steer, diagnostics, reference
