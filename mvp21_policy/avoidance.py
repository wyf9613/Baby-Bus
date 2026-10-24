"""Opt-in static LiDAR avoidance experiment using native MVP2.1 candidates.

Only measured points/road and measured-motion odometry enter decisions. This
is a numerical geometric check, not braking or real-time qualification. ROS
does not construct this component; the first consumer is the offline runtime.
"""
from copy import deepcopy
from dataclasses import dataclass, replace
import math

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
    rolling_check_m: float = .65
    memory_resolution_m: float = .025
    memory_capacity: int = 512

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('avoidance.enabled must be boolean')
        for name, value in vars(self).items():
            if name in ('enabled', 'memory_capacity'):
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
    return [(p[0], p[1], math.atan2(*reversed(_path_tangent(xy, arc, s, window=.0125))))
            for p, s in zip(xy, arc)]


def curvature_peak(path):
    return max((abs(wrap_angle(b[2]-a[2]))/math.dist(a[:2], b[:2])
                for a, b in zip(path, path[1:])), default=0.)


def collision(path, points, cfg, length=math.inf, boundaries=None):
    # Each LiDAR return is an independent point. Connecting successive angular
    # returns would invent solid walls across free gaps.
    settings = ShadowSettings(body_front_m=cfg.front_extent_m+cfg.obstacle_margin_m,
        body_rear_m=cfg.rear_extent_m+cfg.obstacle_margin_m,
        body_width_m=cfg.width_m+2*cfg.obstacle_margin_m)
    hits = {str(i): [p[:2]] for i, p in enumerate(points)}
    hit = _shadow_collision(path, length, hits, settings) if hits else None
    if hit:
        return dict(kind='lidar_point', **hit)
    if boundaries:
        settings = replace(settings,
            body_front_m=cfg.front_extent_m+cfg.boundary_margin_m,
            body_rear_m=cfg.rear_extent_m+cfg.boundary_margin_m,
            body_width_m=cfg.width_m+2*cfg.boundary_margin_m)
        hit = _shadow_collision(path, length, boundaries, settings)
        if hit:
            return dict(kind='road_boundary', **hit)
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
            xy = []
            for i in range(count+1):
                distance = horizon*i/count
                u = min(shift, distance)
                offset = d0+slope*u+a3*u**3+a4*u**4+a5*u**5
                p, _, normal = frame.at(s0+distance)
                xy.append([p[0]+normal[0]*offset, p[1]+normal[1]*offset])
            path = path_poses(xy)
            costs = dict(lateral=sum(frame.project(p[:2])[1]**2 for p in path)/len(path),
                         heading=sum(p[2]**2 for p in path)/len(path))
            candidates.append(dict(id=len(result.get('candidates', []))+len(candidates),
                family='avoidance_lateral_hold', path=path, offsets_m=[target]*3,
                cost=6*costs['lateral']+1.5*costs['heading'], costs=costs, collision=None))
    return candidates


def filter_candidates(job):
    """Native generation + full-path hard filters, then native deterministic cost."""
    context = job['lidar_avoidance']
    cfg = LidarAvoidanceSettings(**context['settings'])
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
        hit = collision(path, points, planning_cfg, boundaries=boundaries)
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

    @property
    def latched(self):
        return self.stop_reason is not None

    def stop(self, reason):
        self.stop_reason = self.stop_reason or reason
        self.diagnostics = dict(enabled=True, stop_required=True, latched=True,
            reason=self.stop_reason, committed_side=self.side, memory_point_count=len(self.memory))
        return self.diagnostics

    def update(self, obstacles, pose, epoch, now):
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
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        for px, py, _ in obstacles['points_xyz_m']:
            wx, wy = x+c*px-s*py, y+s*px+c*py
            key = (round(wx/cfg.memory_resolution_m), round(wy/cfg.memory_resolution_m))
            self.memory[key] = (wx, wy, obstacles['measurement_timestamp_s'])
        self.points = []
        for key, (wx, wy, stamp) in list(self.memory.items()):
            bx, by = c*(wx-x)+s*(wy-y), -s*(wx-x)+c*(wy-y)
            if bx < -cfg.rear_extent_m-cfg.obstacle_margin_m-.10:
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
        return dict(settings=vars(self.settings), obstacles=deepcopy(self.obstacles),
                    points_xy_m=deepcopy(self.points), side=self.side)

    def accept(self, result):
        info = result.get('avoidance') or {}
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
        if check_reference:
            try:
                if not reference or reference.get('valid') is not True:
                    return self.stop('avoidance_reference_unavailable')
                if (reference.get('frame_id') != 'base_link'
                        or not finite_number(reference.get('timestamp_s'))
                        or abs(reference['timestamp_s']-now) > 1e-6):
                    return self.stop('avoidance_reference_frame_or_time')
                poses = path_poses(reference['path']['samples_xy'])
                # Include current body and the connection from actual pose to
                # the cropped reference. Checking only the centerline is unsafe.
                if math.hypot(*poses[0][:2]) > 1e-9:
                    poses.insert(0, (0., 0., 0.))
                hit = collision(poses, self.points, self.settings, self.settings.rolling_check_m)
                result['collision'] = hit
                if hit:
                    result = self.stop('avoidance_execution_collision')
                    result['collision'] = hit
                    return result
            except (ValueError, TypeError, KeyError, ArithmeticError):
                return self.stop('avoidance_reference_invalid')
        self.diagnostics = result
        return result
