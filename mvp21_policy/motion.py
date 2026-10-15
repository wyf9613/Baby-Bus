"""MVP2.1 motion; relocated from pinned f11428b, without algorithm tuning."""

import math
from .geometry import (
    finite_number,
)


class FirstOrderSampleFilter:
    """Causal low-pass; a cached sample is assimilated exactly once.

    sample_key identifies an accepted source message, not its numerical value.
    sample_time_s is ROS sample time for gyro, monotonic receipt for wheel speed;
    each filter stays in its own time domain. Source freshness is checked by the
    caller. New runs and discontinuities reset rather than bridge stale history.
    """
    def __init__(self, tau_s, reset_gap_s):
        self.tau_s = tau_s
        self.reset_gap_s = reset_gap_s
        self.clear()

    def clear(self):
        self.value = self.sample_key = self.sample_time_s = None

    def update(self, value, sample_key, sample_time_s):
        if not finite_number(value) or sample_key is None or not finite_number(sample_time_s):
            self.clear()
            return None
        if sample_key == self.sample_key:
            return self.value
        elapsed = None if self.sample_time_s is None else sample_time_s - self.sample_time_s
        if self.value is None or elapsed is None or elapsed <= 0 or elapsed >= self.reset_gap_s:
            self.value = float(value)
        else:
            alpha = -math.expm1(-elapsed / self.tau_s)
            self.value += alpha * (value - self.value)
        self.sample_key, self.sample_time_s = sample_key, sample_time_s
        return self.value


class EstimationMotionHistory:
    """Bounded, causal filtered wheel/gyro history on the ROS clock.

    Wheel time is ROS receipt (its message has no acquisition stamp). Motion
    assumes FORWARD, no-slip planar driving; unsigned wheel speed cannot support
    reverse compensation. Never substitute missing history with current speed.
    """
    def __init__(self, settings):
        self.settings = settings
        self.filters = {
            "wheel_speed": FirstOrderSampleFilter(settings.speed_tau_s, settings.max_source_age_s),
            "imu_angular_velocity": FirstOrderSampleFilter(settings.yaw_rate_tau_s, settings.max_source_age_s)}
        self.clear()

    def clear(self):
        self.reset_id = getattr(self, "reset_id", 0) + 1
        self.samples = {name: [] for name in self.filters}
        for filter_ in self.filters.values():
            filter_.clear()
        self.last_ros_now = None

    def check_clock(self, now_s):
        if not finite_number(now_s):
            self.clear()
            return False
        if self.last_ros_now is not None and now_s < self.last_ros_now:
            self.clear()
        self.last_ros_now = now_s
        return True

    def clear_source(self, name):
        self.samples[name] = []
        self.filters[name].clear()

    def observe(self, name, value, key, sample_time_s, ros_time_s):
        filter_ = self.filters[name]
        if (not finite_number(value) or not finite_number(ros_time_s)
                or (name == "wheel_speed" and value < 0)):
            self.clear_source(name)
            return
        if key == filter_.sample_key and key is not None:
            return
        if self.samples[name] and ros_time_s <= self.samples[name][-1][0]:
            self.clear_source(name)
        filtered = filter_.update(value, key, sample_time_s)
        if not finite_number(filtered):
            self.clear_source(name)
            return
        samples = self.samples[name]
        samples.append((ros_time_s, filtered))
        # Bounded memory, retaining enough history for the admitted interval.
        cutoff = ros_time_s - 2*self.settings.max_source_age_s
        while len(samples) > 2 and samples[1][0] < cutoff:
            samples.pop(0)
        del samples[:-512]

    def sample_at(self, name, time_s):
        return next((item for item in reversed(self.samples[name]) if item[0] <= time_s), None)

    def current(self, name, now_s):
        sample = self.sample_at(name, now_s)
        return (None if sample is None or now_s-sample[0] > self.settings.motion_max_gap_s
                else sample[1])

    def pose_between(self, start_s, end_s):
        if not all(finite_number(v) for v in (start_s, end_s)) or end_s < start_s:
            return None, "invalid_motion_time"
        if end_s-start_s > self.settings.motion_max_interval_s + 1e-9:
            return None, "motion_interval_too_long"
        x = y = yaw = travelled = 0.0
        knots = sorted({start_s, end_s} | {
            t for samples in self.samples.values() for t, _ in samples if start_s < t < end_s})
        indices = {name: -1 for name in self.samples}
        for begin, end in zip(knots, knots[1:]):
            held = {}
            for name, samples in self.samples.items():
                index = indices[name]
                while index+1 < len(samples) and samples[index+1][0] <= begin:
                    index += 1
                indices[name] = index
                held[name] = samples[index] if index >= 0 else None
            speed, rate = held["wheel_speed"], held["imu_angular_velocity"]
            if speed is None or rate is None:
                return None, "missing_motion_history"
            if max(end-speed[0], end-rate[0]) > self.settings.motion_max_gap_s + 1e-9:
                return None, "motion_history_gap"
            dt = end-begin
            angle, distance = rate[1]*dt, speed[1]*dt
            travelled += abs(distance)
            if not all(math.isfinite(v) for v in (angle, distance)):
                return None, "nonfinite_motion"
            if abs(angle) < 1e-6:
                dx = distance*(1-angle*angle/6)
                dy = distance*(angle/2-angle**3/24)
            else:
                dx, dy = distance*math.sin(angle)/angle, distance*(1-math.cos(angle))/angle
            c, s = math.cos(yaw), math.sin(yaw)
            x, y, yaw = x+c*dx-s*dy, y+s*dx+c*dy, yaw+angle
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                return None, "nonfinite_motion"
        return {"dx_m": x, "dy_m": y, "yaw_rad": yaw, "interval_s": end_s-start_s,
                "travelled_distance_m": travelled}, None

    @staticmethod
    def transform_xy(point, pose):
        x, y = point[0]-pose["dx_m"], point[1]-pose["dy_m"]
        c, s = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
        return (c*x+s*y, -s*x+c*y)

    def pose_between_segments(self, start_s, end_s):
        """Compose admitted short intervals; retain all gap/history checks.

        A delayed planner result can predate the single-interval limit. Splitting
        the interval does not synthesize samples or relax source freshness.
        """
        if not all(finite_number(v) for v in (start_s, end_s)) or end_s < start_s:
            return None, 'invalid_motion_time'
        total = dict(dx_m=0., dy_m=0., yaw_rad=0., travelled_distance_m=0., interval_s=end_s-start_s)
        cursor = start_s
        while cursor < end_s:
            following = min(end_s, cursor+self.settings.motion_max_interval_s)
            if following <= cursor:
                return None, 'invalid_motion_time'
            pose, problem = self.pose_between(cursor, following)
            if problem:
                return None, problem
            c, s = math.cos(total['yaw_rad']), math.sin(total['yaw_rad'])
            total['dx_m'] += c*pose['dx_m']-s*pose['dy_m']
            total['dy_m'] += s*pose['dx_m']+c*pose['dy_m']
            total['yaw_rad'] += pose['yaw_rad']
            total['travelled_distance_m'] += pose['travelled_distance_m']
            cursor = following
        return total, None

    @staticmethod
    def inverse_transform_xy(point, pose):
        """A point in the frame at the END of pose -> the frame at its START."""
        c, s = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
        return (c*point[0]-s*point[1]+pose["dx_m"], s*point[0]+c*point[1]+pose["dy_m"])
