"""Latched LiDAR stop request for a straight, body-forward corridor.

This module decides whether to stop, never how to brake. Points must already
be aligned to the current base_link frame. Curved swept-body checks are outside
this first version. Construct/reset the monitor only on explicit run recovery.
"""
from dataclasses import dataclass
from .geometry import finite_number


@dataclass(frozen=True)
class ObstacleStopSettings:
    enabled: bool = False
    trigger_distance_m: float = 0.15  # gap beyond the FRONT bumper, not lidar range
    front_extent_m: float = 0.297
    rear_extent_m: float = 0.198
    width_m: float = 0.25
    lateral_margin_m: float = 0.0
    max_source_age_s: float = 0.15
    frame_id: str = 'base_link'

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('Obstacle stop enabled must be bool')
        for name in ('trigger_distance_m', 'front_extent_m', 'rear_extent_m',
                     'width_m', 'max_source_age_s'):
            if not finite_number(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError('Positive finite obstacle stop '+name+' required')
        if not finite_number(self.lateral_margin_m) or self.lateral_margin_m < 0:
            raise ValueError('Nonnegative lateral margin required')
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError('Obstacle frame required')


class ObstacleStopMonitor:
    def __init__(self, settings=None):
        self.settings = settings or ObstacleStopSettings()
        self.reset()

    def reset(self):
        """Explicit new run only; disappearing returns never release a stop."""
        self.trigger = None
        self.diagnostics = {}

    @property
    def latched(self):
        return self.trigger is not None

    def request_fault(self, reason, now_s, source_age_s=None):
        """Independent source watchdog; no invented points/alignment record."""
        if reason not in ('lidar_unavailable', 'lidar_stale_or_bad_timestamp',
                          'lidar_frame_or_alignment'):
            raise ValueError('Unsupported LiDAR source fault')
        if not self.settings.enabled:
            return self.evaluate(None, now_s)
        if not self.latched:
            self.trigger = dict(reason=reason, timestamp_s=now_s,
                                front_gap_m=None, source_age_s=source_age_s)
        self.diagnostics = dict(enabled=True, stop_required=True, latched=True,
            reason=self.trigger['reason'], trigger=dict(self.trigger),
            nearest_front_gap_m=None, source_age_s=source_age_s, point_count=0)
        return self.diagnostics

    def evaluate(self, obstacles, now_s):
        cfg = self.settings
        result = dict(enabled=cfg.enabled, stop_required=False, latched=False,
                      reason='disabled' if not cfg.enabled else 'clear',
                      trigger_distance_m=cfg.trigger_distance_m,
                      front_extent_m=cfg.front_extent_m, width_m=cfg.width_m,
                      nearest_front_gap_m=None, point_count=0)
        if not cfg.enabled:
            self.diagnostics = result
            return result
        reason = None
        if not finite_number(now_s):
            reason = 'lidar_clock_invalid'
        elif not isinstance(obstacles, dict) or obstacles.get('available') is not True:
            reason = 'lidar_unavailable'
        elif obstacles.get('frame_id') != cfg.frame_id or obstacles.get('time_aligned') is not True:
            reason = 'lidar_frame_or_alignment'
        else:
            stamp, source, age = (obstacles.get(k) for k in
                                  ('timestamp_s', 'measurement_timestamp_s', 'source_age_s'))
            if (not all(finite_number(v) for v in (stamp, source, age))
                    or abs(stamp-now_s) > 1e-6 or source > now_s+1e-6
                    or age < 0 or age >= cfg.max_source_age_s
                    or now_s-source >= cfg.max_source_age_s
                    or abs((now_s-source)-age) > 1e-5):
                reason = 'lidar_stale_or_bad_timestamp'
            else:
                points = obstacles.get('points_xyz_m')
                if not isinstance(points, (list, tuple)):
                    reason = 'lidar_points_invalid'
                else:
                    gaps = []
                    for point in points:
                        if (not isinstance(point, (list, tuple)) or len(point) != 3
                                or not all(finite_number(v) for v in point)):
                            reason = 'lidar_points_invalid'
                            break
                        x, y, _ = point
                        # Also reject a return already inside the current body.
                        if x >= -cfg.rear_extent_m and abs(y) <= cfg.width_m/2+cfg.lateral_margin_m:
                            gaps.append(x-cfg.front_extent_m)
                    result['point_count'] = len(points)
                    result['source_age_s'] = age
                    result['nearest_front_gap_m'] = min(gaps, default=None)
                    if reason is None and gaps and min(gaps) <= cfg.trigger_distance_m+1e-9:
                        reason = 'lidar_obstacle'
        if reason is not None and not self.latched:
            self.trigger = dict(reason=reason, timestamp_s=now_s,
                                front_gap_m=result['nearest_front_gap_m'],
                                source_age_s=result.get('source_age_s'))
        if self.latched:
            result.update(stop_required=True, latched=True, reason=self.trigger['reason'],
                          trigger=dict(self.trigger))
        self.diagnostics = result
        return result
