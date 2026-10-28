"""Opt-in static LiDAR avoidance experiment using native MVP2.1 candidates.

Only measured points/road and measured-motion odometry enter decisions. This
is a numerical geometric check, not braking or real-time qualification.
ROS and the offline runtime use the same guard and worker-side selector.
"""
from copy import deepcopy
from dataclasses import dataclass, replace
import math
import numpy as np

from .geometry import finite_number, wrap_angle, _path_lengths, _path_tangent
from .frenet import ShadowFrame
from .lattice import ShadowLatticePlanner, ShadowSettings, _shadow_collision
from .obstacle_stop import ObstacleStopMonitor, ObstacleStopSettings


@dataclass(frozen=True)
class LidarAvoidanceSettings:
    enabled: bool = False
    front_extent_m: float = .297
    rear_extent_m: float = .198
    width_m: float = .25
    obstacle_margin_m: float = .08
    tracking_margin_m: float = .08
    boundary_margin_m: float = .02
    max_source_age_s: float = .30
    max_curvature_1pm: float = 2.5
    max_straight_fit_residual_m: float = .02
    rolling_check_m: float = .65
    memory_resolution_m: float = .025
    memory_capacity: int = 2048
    sensing_radius_m: float = 3.5
    # Two 2-Hz planning cycles plus a bounded worker handoff. Sensor source
    # and receipt deadlines remain independently much shorter.
    max_plan_age_s: float = 1.5
    # Legacy A2 stays available for immutable replays; new deliveries opt in.
    maneuver_enabled: bool = False
    maneuver_plan_length_m: float = 2.5
    maneuver_plan_rate_hz: float = 2.0
    maneuver_side: int = 1
    trigger_lookahead_m: float = 3.0
    target_cluster_gap_m: float = .15
    target_match_distance_m: float = .15
    target_depth_allowance_m: float = .30
    target_lateral_allowance_m: float = .10
    road_edge_exclusion_m: float = .12
    road_support_max_age_s: float = .65
    maneuver_straight_residual_m: float = .05
    maneuver_clearance_m: float = .04
    shift_length_m: float = 2.0
    return_length_m: float = 2.0
    pass_margin_m: float = .15
    lateral_tolerance_m: float = .06
    maneuver_heading_tolerance_rad: float = .15

    def __post_init__(self):
        if not isinstance(self.maneuver_enabled, bool) or type(self.maneuver_side) is not int or self.maneuver_side not in (-1, 1):
            raise ValueError('Boolean maneuver enable and side -1/+1 required')
        if self.maneuver_enabled and not self.enabled:
            raise ValueError('Maneuver requires enabled LiDAR avoidance')
        if not isinstance(self.enabled, bool):
            raise ValueError('avoidance.enabled must be boolean')
        for name, value in vars(self).items():
            if name in ('enabled', 'memory_capacity', 'maneuver_enabled', 'maneuver_side'):
                continue
            if not finite_number(value) or value <= 0:
                raise ValueError('Positive finite avoidance '+name+' required')
        if type(self.memory_capacity) is not int or not 1 <= self.memory_capacity <= 4096:
            raise ValueError('Bounded integer memory capacity required')


def obstacle_fault(obstacles, now, cfg):
    # Reuse the established observation admission, without resetting the actual
    # emergency-stop latch. A close point is valid data, not a source failure.
    probe = ObstacleStopMonitor(ObstacleStopSettings(enabled=True,
        front_extent_m=cfg.front_extent_m, rear_extent_m=cfg.rear_extent_m,
        width_m=cfg.width_m, max_source_age_s=cfg.max_source_age_s))
    result = probe.evaluate(obstacles, now)
    return result['reason'] if result['reason'] not in ('clear', 'lidar_obstacle') else None


def path_poses(points):
    if (not isinstance(points, (list, tuple)) or len(points) < 2
            or any(not isinstance(p, (list, tuple)) or len(p) < 2
                   or not all(finite_number(v) for v in p[:2]) for p in points)
            or any(math.dist(a[:2], b[:2]) <= 1e-9 for a, b in zip(points, points[1:]))):
        raise ValueError('Finite nondegenerate sampled path required')
    xy = [p[:2] for p in points]
    arc = _path_lengths(xy)
    xy_array = np.asarray(xy, dtype=float); stations = np.asarray(arc)
    before = np.clip(stations-.0125, 0., arc[-1]); after = np.clip(stations+.0125, 0., arc[-1])
    dx = np.interp(after, stations, xy_array[:, 0])-np.interp(before, stations, xy_array[:, 0])
    dy = np.interp(after, stations, xy_array[:, 1])-np.interp(before, stations, xy_array[:, 1])
    if np.any(np.hypot(dx,dy) <= 1e-12):
        raise ValueError('Degenerate path tangent')
    return np.column_stack((xy_array, np.arctan2(dy,dx))).tolist()


def curvature_peak(path):
    return max((abs(wrap_angle(b[2]-a[2]))/math.dist(a[:2], b[:2])
                for a, b in zip(path, path[1:])), default=0.)


def collision(path, points, cfg, length=math.inf, boundaries=None, *, plan_tracking=False):
    # Same adaptive samples as the established numerical sweep; vectorize the
    # point and segment tests instead of rebuilding a rectangle for every hit.
    arc = _path_lengths([p[:2] for p in path])
    end = min(length, arc[-1]); poses = [path[0]]
    for i, (a, b) in enumerate(zip(path, path[1:])):
        if arc[i] >= end:
            break
        fraction = min(1., (end-arc[i])/(arc[i+1]-arc[i]))
        angle = wrap_angle(b[2]-a[2])*fraction
        distance = (arc[i+1]-arc[i])*fraction
        count = max(1, math.ceil(distance/.01), math.ceil(abs(angle)/.025))
        for j in range(1, count+1):
            u = fraction*j/count
            poses.append((a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]),
                          a[2]+wrap_angle(b[2]-a[2])*u))
    poses = np.asarray(poses, dtype=float)
    c, s = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])

    def local(values):
        xy = np.asarray(values, dtype=float)[:, :2]
        dx, dy = xy[None, :, 0]-poses[:, None, 0], xy[None, :, 1]-poses[:, None, 1]
        return c*dx+s*dy, -s*dx+c*dy

    def inside(x, y, margin):
        return ((x >= -cfg.rear_extent_m-margin) & (x <= cfg.front_extent_m+margin)
                & (np.abs(y) <= cfg.width_m/2+margin))

    if points:
        margin = cfg.obstacle_margin_m
        if plan_tracking:
            travelled = np.r_[0., np.cumsum(np.linalg.norm(np.diff(poses[:, :2],axis=0),axis=1))]
            # Present measured pose has no accumulated future tracking error.
            # Grow the extra tracking tube over the checked forward distance;
            # the sensing margin is never reduced, including at the start.
            margin = margin+cfg.tracking_margin_m*np.minimum(1.,travelled[:, None]/cfg.rolling_check_m)
        x, y = local(points); hits = np.argwhere(inside(x, y, margin))
        if hits.size:
            pose, point = map(int, hits[0])
            return dict(kind='lidar_point', side=str(point), point_xy_m=list(points[point][:2]),
                        pose=poses[pose].tolist(), pose_index=pose)
    for side, values in (boundaries or {}).items():
        if not values:
            continue
        x, y = local(values); margin = cfg.boundary_margin_m
        hits = np.argwhere(inside(x, y, margin))
        if hits.size:
            pose, point = map(int, hits[0])
            return dict(kind='road_boundary', side=side, point_xy_m=list(values[point][:2]),
                        pose=poses[pose].tolist(), pose_index=pose)
        if len(values) < 2:
            continue
        lower = np.zeros_like(x[:, :-1]); upper = np.ones_like(lower)
        for start, finish, lo, hi in (
                (x[:, :-1], x[:, 1:], -cfg.rear_extent_m-margin, cfg.front_extent_m+margin),
                (y[:, :-1], y[:, 1:], -cfg.width_m/2-margin, cfg.width_m/2+margin)):
            delta = finish-start; parallel = np.abs(delta) < 1e-15
            safe = np.where(parallel, 1., delta)
            a, b = (lo-start)/safe, (hi-start)/safe
            low, high = np.minimum(a, b), np.maximum(a, b)
            low = np.where(parallel, -np.inf, low)
            high = np.where(parallel, np.inf, high)
            high = np.where(parallel & ((start < lo) | (start > hi)), -np.inf, high)
            lower = np.maximum(lower, low); upper = np.minimum(upper, high)
        hits = np.argwhere(lower <= upper)
        if hits.size:
            pose, segment = map(int, hits[0])
            return dict(kind='road_boundary', side=side, segment_xy_m=values[segment:segment+2],
                        pose=poses[pose].tolist(), pose_index=pose)
    return None


def lateral_hold_candidates(result, job, cfg, points):
    """Add smooth lateral holds in the native Frenet chart for a static bypass.

    The upstream road-following pool remains intact. These explicit experiment
    families use a quintic lane shift and keep the offset until later measured
    replanning; no unseen obstacle depth or return-to-center truth is assumed.
    """
    # Current measured road chart, not the temporally fused/previous-reference
    # guide: a lateral hold is measured from the road, not yesterday's offset.
    chart = job['road'].get('centerline_xy')
    if not chart:
        return []
    chart_source = 'observed_centerline'
    fit_residual = None
    if len(chart) >= 2:
        observed = np.asarray(chart, dtype=float)[:, :2]
        center = observed.mean(axis=0); shifted = observed-center
        _, axes = np.linalg.eigh(shifted.T@shifted)
        axis = axes[:, -1]
        if float(axis@(observed[-1]-observed[0])) < 0:
            axis = -axis
        stations = shifted@axis
        normal = np.array([-axis[1],axis[0]])
        fit_residual = float(np.max(np.abs(shifted@normal)))
        # A tightly supported near-straight observation should not acquire
        # unrealizable curvature from tiny polyline tangent discontinuities.
        # This uses measured samples only; current hard boundaries stay raw.
        if (fit_residual <= cfg.max_straight_fit_residual_m
                and float(stations.max()-stations.min()) > .2
                and np.all(np.diff(stations) > 0)):
            chart = [(center+axis*stations.min()).tolist(),(center+axis*stations.max()).tolist()]
            chart_source = 'observed_near_straight_fit'
    frame = ShadowFrame(chart)
    s0, d0 = frame.project((0., 0.))
    horizon = min(job['settings']['length_m'], frame.end-s0)
    if horizon <= .4:
        return []
    _, tangent, _ = frame.at(s0)
    slope = math.tan(-math.atan2(tangent[1], tangent[0]))
    if abs(slope) > .8:
        return []
    forward = [p[0] for p in points if p[0] >= 0.]
    available = min(forward, default=horizon)-cfg.front_extent_m-cfg.obstacle_margin_m-.10
    candidates = []
    count = math.ceil(horizon/job['settings']['path_spacing_m'])
    for extra_length in (0., .15, .30, .50):
        shift = max(.5, min(.9*horizon, available+extra_length))
        for target in (-.50, -.46, -.42, -.38, -.34, -.30, -.26, -.22,
                       .22, .26, .30, .34, .38, .42, .46, .50):
            delta = target-d0
            a3, a4, a5 = ((10*delta-6*slope*shift)/shift**3,
                         (-15*delta+8*slope*shift)/shift**4,
                         (6*delta-3*slope*shift)/shift**5)
            xy = []; lateral_offsets = []
            for i in range(count+1):
                distance = horizon*i/count
                u = min(shift, distance)
                offset = d0+slope*u+a3*u**3+a4*u**4+a5*u**5
                p, _, normal = frame.at(s0+distance)
                xy.append([p[0]+normal[0]*offset, p[1]+normal[1]*offset])
                lateral_offsets.append(offset)
            path = path_poses(xy)
            costs = dict(lateral=sum(d*d for d in lateral_offsets)/len(path),
                         heading=sum(p[2]**2 for p in path)/len(path))
            candidates.append(dict(id=len(result.get('candidates', []))+len(candidates),
                family='avoidance_lateral_hold', path=path, offsets_m=[target]*3,
                cost=6*costs['lateral']+1.5*costs['heading'], costs=costs, collision=None,
                chart_source=chart_source, observed_fit_residual_m=fit_residual))
    return candidates


def filter_candidates(job):
    """Native generation + full-path hard filters, then native deterministic cost."""
    context = job['lidar_avoidance']
    cfg = LidarAvoidanceSettings(**context['settings'])
    if cfg.maneuver_enabled:
        from .maneuver import plan_maneuver
        return plan_maneuver(job, cfg)
    if not cfg.enabled:
        raise ValueError('Avoidance planner requires explicit enable')
    result = ShadowLatticePlanner(ShadowSettings(**job['settings'])).plan(job)
    result = deepcopy(result)
    road = job['road']
    sources = road.get('boundary_source') or {}
    boundaries = {side: road.get(side+'_boundary_xy') for side in ('left', 'right')
                  if sources.get(side) in ('observed', 'observed_with_history')
                  and road.get(side+'_boundary_xy')}
    reason = obstacle_fault(context['obstacles'], job['stamp_s'], cfg)
    if (road.get('frame_id') != 'base_link' or road.get('time_aligned') is not True
            or not finite_number(road.get('timestamp_s'))
            or abs(road['timestamp_s']-job['stamp_s']) > 1e-6):
        reason = reason or 'avoidance_road_frame_or_time'
    if len(boundaries) != 2:
        reason = reason or 'avoidance_requires_two_observed_boundaries'
    points = context['points_xy_m']
    if points:
        result.setdefault('candidates', []).extend(lateral_hold_candidates(result, job, cfg, points))
    result['candidate_count'] = len(result['candidates'])
    preference = context.get('side', 0)
    ahead = [p for p in points if p[0] >= -cfg.rear_extent_m-cfg.obstacle_margin_m]
    blocking = collision([(0., 0., 0.), (job['settings']['length_m'], 0., 0.)], ahead, cfg)
    obstacle = blocking['point_xy_m'] if blocking else None
    feasible = []
    planning_cfg = replace(cfg, obstacle_margin_m=cfg.obstacle_margin_m+cfg.tracking_margin_m)
    for candidate in result.get('candidates', []):
        path = candidate['path']
        rejection = reason
        peak = curvature_peak(path)
        if peak > cfg.max_curvature_1pm:
            rejection = rejection or 'curvature_limit'
        radius = math.hypot(max(cfg.front_extent_m, cfg.rear_extent_m)+planning_cfg.obstacle_margin_m,
                            cfg.width_m/2+planning_cfg.obstacle_margin_m)
        if any(math.hypot(p[0], p[1])+radius > cfg.sensing_radius_m for p in path):
            rejection = rejection or 'outside_lidar_horizon'
        hit = collision(path, points, cfg, boundaries=boundaries, plan_tracking=True) if rejection is None else None
        if hit:
            rejection = rejection or hit['kind']
        side = 0
        if obstacle is not None:
            closest = min(path, key=lambda p: abs(p[0]-obstacle[0]))
            delta = closest[1]-obstacle[1]
            side = 1 if delta > .02 else -1 if delta < -.02 else 0
            if preference and side != preference:
                rejection = rejection or 'committed_side'
        candidate['avoidance'] = dict(rejected=rejection, collision=hit,
            peak_curvature_1pm=peak, side=side, checked_length_m='full_generated_path')
        if rejection is None:
            feasible.append(candidate)
    bypassing = bool(blocking or (preference and points))
    selected = min(feasible, key=lambda c: (
        int(bypassing and c['family'] != 'avoidance_lateral_hold'), c['cost'], c['id'])) if feasible else None
    result.update(status='VALID' if selected else 'NO_AVOIDANCE_PATH', selected=selected,
        feasible_count=len(feasible), avoidance=dict(enabled=True, reason=reason,
            blocking_obstacle=bool(blocking), committed_side=preference,
            selected_side=(selected['avoidance']['side'] if selected else preference),
            memory_point_count=len(points), settings=vars(cfg)))
    return result


class LidarAvoidanceGuard:
    """Per-run memory/side commitment and final reference admission.

    Memory assumes static obstacles. Fresh live scans are still mandatory; a
    missing return never renews a remembered point's measurement timestamp.
    Stop is latched until a caller explicitly starts a new run/component.
    """
    def __init__(self, settings):
        if not settings.enabled:
            raise ValueError('Avoidance guard requires explicit enable')
        self.settings = settings
        self.memory = {}
        self.side = 0
        self.epoch = None
        self.stamp = None
        self.pose = None
        self.stop_reason = None
        self.diagnostics = {}
        self.obstacles = None
        self.points = []
        self.first_failure = None
        self.maneuver = None
        if settings.maneuver_enabled:
            from .maneuver import AvoidanceManeuverManager
            self.maneuver = AvoidanceManeuverManager(settings)

    @property
    def latched(self):
        return self.stop_reason is not None

    def stop(self, reason):
        if self.first_failure is None:
            self.first_failure = dict(reason=reason, timestamp_s=self.stamp,
                maneuver=None if self.maneuver is None else deepcopy(self.maneuver.diagnostics))
        self.stop_reason = self.stop_reason or reason
        self.diagnostics = dict(enabled=True, stop_required=True, latched=True,
            reason=self.stop_reason, committed_side=self.side, memory_point_count=len(self.memory),
            first_failure=self.first_failure)
        return self.diagnostics

    def update(self, obstacles, pose, epoch, now, road=None):
        cfg = self.settings
        self.obstacles = deepcopy(obstacles)
        fault = obstacle_fault(obstacles, now, cfg)
        if fault:
            return self.stop(fault)
        if (not isinstance(pose, (list, tuple)) or len(pose) != 3
                or not all(finite_number(v) for v in pose) or type(epoch) is not int
                or (self.stamp is not None and now < self.stamp)):
            return self.stop('avoidance_motion_invalid')
        if self.epoch is not None and epoch != self.epoch and self.memory:
            return self.stop('avoidance_motion_epoch_changed')
        self.epoch, self.stamp, self.pose = epoch, now, list(pose)
        if self.maneuver is not None:
            problem = self.maneuver.update(obstacles, road, pose, epoch, now)
            self.points = self.maneuver.collision_points(obstacles['points_xyz_m'])
            self.side = self.maneuver.task['side'] if self.maneuver.active else 0
            self.memory = {}  # Target task, not environmental map, owns progress.
            if problem or self.maneuver.error:
                return self.stop(problem or self.maneuver.error)
            if len(self.points) > self.settings.memory_capacity:
                return self.stop('maneuver_point_capacity')
            return self.evaluate(None, now, check_reference=False)
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        for px, py, _ in obstacles['points_xyz_m']:
            if math.hypot(px, py) > cfg.sensing_radius_m:
                continue
            wx, wy = x+c*px-s*py, y+s*px+c*py
            key = (round(wx/cfg.memory_resolution_m), round(wy/cfg.memory_resolution_m))
            self.memory[key] = (wx, wy, obstacles['measurement_timestamp_s'])
        self.points = []
        for key, (wx, wy, stamp) in list(self.memory.items()):
            bx, by = c*(wx-x)+s*(wy-y), -s*(wx-x)+c*(wy-y)
            if (bx < -cfg.rear_extent_m-cfg.obstacle_margin_m-.10
                    or math.hypot(bx, by) > cfg.sensing_radius_m):
                del self.memory[key]
            else:
                self.points.append([bx, by])
        if len(self.memory) > cfg.memory_capacity:
            return self.stop('avoidance_memory_capacity')
        # Release only after the measured-motion rear footprint passed all
        # remembered points. Empty scans alone cannot release the commitment.
        if not self.memory:
            self.side = 0
        return self.evaluate(None, now, check_reference=False)

    def context(self):
        result = dict(settings=vars(self.settings), obstacles=deepcopy(self.obstacles),
                    points_xy_m=deepcopy(self.points), side=self.side)
        if self.maneuver is not None:
            result['maneuver'] = self.maneuver.context()
        return result

    def accept(self, result):
        info = result.get('avoidance') or {}
        if self.maneuver is not None:
            if result.get('status') == 'VALID' and self.maneuver.active:
                self.maneuver.admitted = True
            elif self.maneuver.active:
                self.stop(info.get('reason') or 'maneuver_no_path')
            return
        if result.get('status') == 'VALID' and info.get('selected_side'):
            self.side = self.side or info['selected_side']
        elif info.get('blocking_obstacle'):
            self.stop('no_feasible_avoidance_path')

    def evaluate(self, reference, now, *, check_reference=True):
        if self.latched:
            return self.stop(self.stop_reason)
        if self.stamp is None or abs(self.stamp-now) > 1e-6:
            return self.stop('avoidance_snapshot_not_current')
        result = dict(enabled=True, stop_required=False, latched=False, reason='clear',
            committed_side=self.side, memory_point_count=len(self.memory))
        if self.maneuver is not None:
            result['maneuver'] = deepcopy(self.maneuver.diagnostics)
        if check_reference:
            try:
                if not reference or reference.get('valid') is not True:
                    return self.stop('avoidance_reference_unavailable')
                if (reference.get('frame_id') != 'base_link'
                        or not finite_number(reference.get('timestamp_s'))
                        or abs(reference['timestamp_s']-now) > 1e-6):
                    return self.stop('avoidance_reference_frame_or_time')
                if self.maneuver is not None and not self.maneuver.active:
                    # NORMAL retains the successful MVP execution admission and
                    # independent forward ObstacleStopMonitor. A surrounding
                    # margin halo and a task-source TTL must not become new
                    # stop conditions for ordinary lane following. Active tasks
                    # below still check the complete swept footprint, including
                    # the rear when turning, and retain their source deadline.
                    result.update(protection_mode='baseline_forward_stop', collision=None)
                    self.diagnostics = result
                    return result
                if (self.maneuver is not None and self.maneuver.active and self.maneuver.admitted
                        and reference.get('maneuver_task_id') != self.maneuver.task['id']):
                    return self.stop('maneuver_blocked_fallback')
                poses = path_poses(reference['path']['samples_xy'])
                if reference.get('planner_version') == 'lattice_execution':
                    source = reference.get('original_input_stamp_s')
                    if not finite_number(source) or not 0 <= now-source < self.settings.max_plan_age_s:
                        result = self.stop('avoidance_planner_stale')
                        result.update(plan_age_s=now-source if finite_number(source) else None,
                                      max_plan_age_s=self.settings.max_plan_age_s)
                        return result
                # Include current body and the connection from actual pose to
                # the cropped reference. Checking only the centerline is unsafe.
                if math.hypot(*poses[0][:2]) > 1e-9:
                    poses.insert(0, (0., 0., 0.))
                boundaries = (self.maneuver.context()['boundaries']
                    if self.maneuver is not None and self.maneuver.active else None)
                hit = collision(poses, self.points, self.settings, self.settings.rolling_check_m,
                                boundaries=boundaries)
                result['collision'] = hit
                if hit:
                    result = self.stop('avoidance_execution_collision')
                    result['collision'] = hit
                    return result
            except (ValueError, TypeError, KeyError, ArithmeticError):
                return self.stop('avoidance_reference_invalid')
        self.diagnostics = result
        return result


def accept_avoidance_result(execution, guard, result):
    """Retain a still-checked bypass across a temporary nonblocking plan miss.

    The original pose/epoch/progress/source stamp remain unchanged. Execution
    still applies current road/LiDAR checks and the original plan-age deadline.
    A newly blocked corridor or worker fault never takes this path.
    """
    if guard.maneuver is not None:
        info = result.get('avoidance') or {}
        if not info.get('enabled') or 'maneuver_revision' not in info:
            guard.stop('avoidance_worker_result_invalid')
            return True
        if not guard.maneuver.result_current(info):
            return False  # Late normal/old task/old return geometry cannot replace a task.
    previous = execution.active
    if not execution.accept(result):
        return False
    info = result.get('avoidance') or {}
    if not info.get('enabled'):
        guard.stop('avoidance_worker_result_invalid')
        return True
    if result.get('status') == 'VALID' and execution.active is None:
        guard.stop('avoidance_worker_payload_invalid')
        return True
    guard.accept(result)
    if (guard.maneuver is None and previous is not None and not guard.latched and guard.side
            and result.get('status') == 'NO_AVOIDANCE_PATH'
            and not info.get('blocking_obstacle') and info.get('reason') is None
            and previous['epoch'] == guard.epoch):
        execution.active = previous
        execution.reason = 'avoidance_retained_checked_reference'
    return True
