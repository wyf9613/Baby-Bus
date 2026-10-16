"""MVP2.1 estimation; relocated from pinned f11428b, without algorithm tuning."""

from dataclasses import dataclass
from copy import deepcopy
import math
from .geometry import (
    _path_lengths,
    _path_points,
    _path_project,
    _path_sample,
    _path_tangent,
    finite_number,
)
from .motion import (
    EstimationMotionHistory,
)


@dataclass(frozen=True)
class EstimationSettings:
    """Prototype tuning, not a calibration or permission to drive.

    Course framework contracts take priority; the Word meeting draft only
    supplies additional internal EstimatedState/RoadState records.
    Screenshot names v_mps/yaw_rate_radps are additive aliases. Distances are
    metres, angles radians, and all output stamps use the node's ROS clock.
    Parameters live in the EXISTING ai4r_policy.yaml under estimation.*.
    """
    speed_tau_s: float = 0.15
    yaw_rate_tau_s: float = 0.08
    max_source_age_s: float = 0.5
    motion_max_gap_s: float = 0.15
    motion_max_interval_s: float = 0.3
    # Local observed cones only, propagated incrementally between camera frames.
    road_history_s: float = 5.0
    road_rear_support_m: float = 0.5
    # Brief quality-drop bridge, not the lifetime of near-field observations.
    # Zero disables prediction; source and motion deadlines still apply.
    road_hold_s: float = 0.2
    # ROS cone colour IDs: yellow=1, blue=2. YAML selects the course layout.
    left_colour: int = 2
    right_colour: int = 1
    min_cone_confidence: float = 0.5
    road_forward_max_m: float = 3.0
    road_min_coverage_m: float = 0.5
    road_sample_spacing_m: float = 0.1
    road_max_gap_m: float = 1.5
    huber_delta_m: float = 0.08
    road_width_min_m: float = 0.4
    road_width_max_m: float = 2.0
    road_max_abs_slope: float = 1.0
    # 0 means unknown: single-sided geometry stays invalid until measured or
    # explicitly supplied for a known simulated road. Never guess a lane width.
    known_lane_width_m: float = 0.0
    # Remember the last valid TWO-SIDED observed width for this policy run.
    # A fresh single boundary then defines an inward normal-offset centreline.
    # This is a constant-track-width assumption, not a hold of an old path.
    single_side_from_learned_width: bool = False
    # Fraction of the FULL learned/known lane width to offset inward from a
    # single observed boundary. 0.5 is geometric centre; smaller hugs that side.
    # This does not change width learning or the two-boundary centreline.
    single_side_offset_fraction: float = 0.5
    # Ordered XY curves support vertical and returning road segments. The
    # legacy y(x) estimator remains selectable for the archived workshop run.
    road_geometry: str = "y_of_x"
    # Opposite-boundary arc backtrack allowed during ordered pairing. The
    # default retains the existing 3 cm rule; source: mpc-car-run d155a80.
    ordered_pairing_backtrack_m: float = 0.03
    # Imported mpc-car-run switches (docs/ESTIMATION_V1_CN.md); defaults keep
    # the original behaviour. "continuous" moves the cone history to every
    # policy step with short wheel/gyro intervals, so a camera gap no longer
    # needs one long motion interval and does not clear the history.
    history_propagation: str = "camera"
    # "distance" keeps a cone until it falls behind road_rear_support_m (the
    # local window bounds the dead-reckoned travel to ~3.5 m), with
    # road_history_max_s as a stationary backstop instead of road_history_s.
    road_history_mode: str = "time"
    road_history_max_s: float = 20.0
    # Merge the visible side of a one-sided frame with the history of both
    # sides instead of returning the one-sided frame unfused.
    history_merge_single_side: bool = False
    # A fresh cone replaces history cones within this radius (0 = the original
    # 2*huber_delta_m). It must stay below half the cone spacing, or a fresh
    # detection of cone k also deletes cone k-1 and leaves holes near the car.
    history_merge_radius_m: float = 0.0
    # Opt-in alternative: reconstruct an interrupted camera output from the
    # incrementally moved, observed cone cache. No unseen-road extrapolation.
    continuous_road_output: bool = False
    road_prediction_max_distance_m: float = 0.2

    def __post_init__(self):
        if (type(self.left_colour) is not int or type(self.right_colour) is not int
                or {self.left_colour, self.right_colour} != {1, 2}):
            raise ValueError("estimation left/right colours must be distinct yellow=1 and blue=2")
        if self.history_propagation not in ("camera", "continuous"):
            raise ValueError("estimation.history_propagation must be camera or continuous")
        if self.road_history_mode not in ("time", "distance"):
            raise ValueError("estimation.road_history_mode must be time or distance")
        if self.road_geometry not in ("y_of_x", "ordered_xy"):
            raise ValueError("estimation.road_geometry must be y_of_x or ordered_xy")
        if not isinstance(self.history_merge_single_side, bool):
            raise ValueError("estimation.history_merge_single_side must be bool")
        if not isinstance(self.continuous_road_output, bool):
            raise ValueError("estimation.continuous_road_output must be bool")
        if not isinstance(self.single_side_from_learned_width, bool):
            raise ValueError("estimation.single_side_from_learned_width must be bool")
        if self.continuous_road_output and self.history_propagation != "continuous":
            raise ValueError("continuous road output requires continuous history propagation")
        for name, value in vars(self).items():
            if name in ("history_propagation", "road_history_mode", "road_geometry", "history_merge_single_side",
                        "continuous_road_output", "single_side_from_learned_width"):
                continue
            if not finite_number(value):
                raise ValueError(f"estimation.{name} must be finite")
            if name == "min_cone_confidence":
                if not 0.0 <= value <= 1.0:
                    raise ValueError("min_cone_confidence must be in [0,1]")
            elif name == "known_lane_width_m":
                if value != 0.0 and not self.road_width_min_m <= value <= self.road_width_max_m:
                    raise ValueError("known width must be 0 (unknown) or inside width bounds")
            elif name == "single_side_offset_fraction":
                if not 0.0 < value < 1.0:
                    raise ValueError("single-side offset fraction must be in (0, 1)")
            elif name == "ordered_pairing_backtrack_m":
                if not 0.0 <= value <= 0.3:
                    raise ValueError("estimation.ordered_pairing_backtrack_m must be in [0, 0.3]")
            elif name == "history_merge_radius_m":
                if value < 0.0:
                    raise ValueError("estimation.history_merge_radius_m must be >= 0")
            elif name == "road_hold_s":
                # The continuous version integrates each short motion segment;
                # admit the requested wider hold there, preserving legacy limits.
                hold_limit = 1.0 if self.continuous_road_output else 0.25
                if not 0.0 <= value <= hold_limit:
                    raise ValueError(f"road prediction hold must be in [0, {hold_limit}] seconds")
            elif value <= 0.0:
                raise ValueError(f"estimation.{name} must be positive")
        if self.road_width_min_m >= self.road_width_max_m:
            raise ValueError("road width bounds must increase")
        if self.road_min_coverage_m > self.road_forward_max_m:
            raise ValueError("road coverage exceeds the forward range")
        if self.road_sample_spacing_m > self.road_min_coverage_m:
            raise ValueError("road spacing exceeds minimum coverage")
        if (self.road_forward_max_m + self.road_rear_support_m) / self.road_sample_spacing_m > 200:
            raise ValueError("road sampling is limited to 201 points")
        if self.road_history_s > 5.0 or self.road_rear_support_m > 1.0:
            raise ValueError("local road history is limited to 5 s and 1 m behind the car")
        if not self.road_history_s <= self.road_history_max_s <= 30.0:
            raise ValueError("road_history_max_s must be in [road_history_s, 30] s")
        if self.road_prediction_max_distance_m > 0.5:
            raise ValueError("road prediction travel is limited to 0.5 m")

        if max(self.motion_max_gap_s, self.motion_max_interval_s) > self.max_source_age_s:
            raise ValueError("motion timing limits cannot exceed the source-age limit")

    @property
    def history_lifetime_s(self):
        """Cone lifetime: road_history_s, or the stationary backstop in distance mode."""
        return self.road_history_max_s if self.road_history_mode == "distance" else self.road_history_s


def _estimation_median(values):
    ordered = sorted(values)
    n = len(ordered)
    return ordered[n // 2] if n % 2 else (ordered[n // 2 - 1] + ordered[n // 2]) / 2.0


def _estimation_solve(matrix, rhs):
    """Pivoted solve of a maximum 3x3 scaled normal system; no extra runtime dependency."""
    rows = [list(row) + [value] for row, value in zip(matrix, rhs)]
    n = len(rows)
    for i in range(n):
        pivot = max(range(i, n), key=lambda j: abs(rows[j][i]))
        rows[i], rows[pivot] = rows[pivot], rows[i]
        divisor = rows[i][i]
        if abs(divisor) < 1e-10 or not math.isfinite(divisor):
            return None
        rows[i] = [v / divisor for v in rows[i]]
        for j in range(n):
            if j != i:
                factor = rows[j][i]
                rows[j] = [a - factor*b for a, b in zip(rows[j], rows[i])]
    result = [row[-1] for row in rows]
    return result if all(math.isfinite(v) for v in result) else None


def _estimation_fit_boundary(points, settings):
    """Finite, bounded Huber IRLS followed by an inlier refit.

    Fits y(x) only for modest local slopes. Ordered points, measured support and
    gaps determine validity. This reconstructs a road, not a driving reference.
    """
    points = sorted(points)
    # Keep bounded work, retaining points across the visible range.
    if len(points) > 64:
        points = [points[round(i*(len(points)-1)/63)] for i in range(64)]
    if len(points) < 2:
        return None
    x0, x1 = points[0][0], points[-1][0]
    if x1 - x0 < settings.road_min_coverage_m:
        return None
    origin, scale = (x0+x1)/2.0, (x1-x0)/2.0
    degree = 2 if len(points) >= 5 and len({p[0] for p in points}) >= 3 else 1
    basis = [[((p[0]-origin)/scale)**j for j in range(degree+1)] for p in points]
    # Three fixed baselines keep robust initialization linear in point count;
    # do not build all N*(N-1)/2 pairs for each side of every camera frame.
    strides = sorted({1, max(1, len(points)//4), max(1, len(points)//2)})
    slopes = [(points[i+d][1]-points[i][1])/(points[i+d][0]-points[i][0])
              for d in strides for i in range(len(points)-d) if points[i+d][0]-points[i][0] > 1e-6]
    if not slopes:
        return None
    slope = _estimation_median(slopes)
    intercept = _estimation_median([p[1]-slope*p[0] for p in points])
    coeffs = [intercept+slope*origin, slope*scale] + ([0.0] if degree == 2 else [])
    residuals = []
    for _ in range(8):
        residuals = [sum(c*b for c, b in zip(coeffs, row))-p[1]
                     for row, p in zip(basis, points)]
        weights = [p[2]*min(1.0, settings.huber_delta_m/max(abs(r), 1e-12))
                   for p, r in zip(points, residuals)]
        matrix = [[sum(w*row[i]*row[j] for w, row in zip(weights, basis))
                   for j in range(degree+1)] for i in range(degree+1)]
        rhs = [sum(w*row[i]*p[1] for w, row, p in zip(weights, basis, points))
               for i in range(degree+1)]
        updated = _estimation_solve(matrix, rhs)
        if updated is None:
            return None
        converged = max(abs(a-b) for a, b in zip(coeffs, updated)) < 1e-7
        coeffs = updated
        if converged:
            break
    residuals = [sum(c*b for c, b in zip(coeffs, row))-p[1] for row, p in zip(basis, points)]
    inliers = [i for i, r in enumerate(residuals) if abs(r) <= 2*settings.huber_delta_m]
    if len(inliers) < degree+1 or len(inliers) < 0.6*len(points):
        return None
    # Restrict support to a near contiguous inlier segment; never bridge a large gap.
    segment = []
    for i in inliers:
        if segment and points[i][0]-points[segment[-1]][0] > settings.road_max_gap_m:
            break
        segment.append(i)
    if len(segment) < degree+1:
        return None
    inliers = segment
    matrix = [[sum(points[k][2]*basis[k][i]*basis[k][j] for k in inliers)
               for j in range(degree+1)] for i in range(degree+1)]
    rhs = [sum(points[k][2]*basis[k][i]*points[k][1] for k in inliers)
           for i in range(degree+1)]
    coeffs = _estimation_solve(matrix, rhs)
    if coeffs is None:
        return None
    lo, hi = points[inliers[0]][0], points[inliers[-1]][0]
    if hi-lo < settings.road_min_coverage_m:
        return None
    c2 = coeffs[2] if degree == 2 else 0.0
    a2 = c2/(scale*scale)
    a1 = coeffs[1]/scale - 2*a2*origin
    a0 = coeffs[0] - coeffs[1]*origin/scale + a2*origin*origin
    if max(abs(a1+2*a2*x) for x in (lo, hi)) > settings.road_max_abs_slope:
        return None
    rms = math.sqrt(sum((a0+a1*points[i][0]+a2*points[i][0]**2-points[i][1])**2
                        for i in inliers)/len(inliers))
    if rms > settings.huber_delta_m:
        return None
    return {"coeffs": (a0, a1, a2), "range": (lo, hi), "rms_m": rms,
            "support_points": [points[i] for i in inliers],
            "inlier_fraction": len(inliers)/len(points)}


def _estimation_order_boundary(points, settings):
    """Bounded local nearest-neighbour chain with direction continuity.

    Confidence filters run before this function. Near duplicates are removed;
    growth never bridges a gap larger than road_max_gap_m, or makes an abrupt
    reverse. A sparse/ambiguous cone layout is not a complete track map.
    """
    unique = []
    for p in sorted(points, key=lambda p: (-p[2], p[0], p[1])):
        if not any(math.hypot(p[0]-q[0], p[1]-q[1]) < 0.03 for q in unique):
            unique.append(p)
    unique = sorted(unique, key=lambda p: (p[0]**2+p[1]**2, p[0], p[1]))[:64]
    if len(unique) < 2:
        return None
    nearest_spacings = [min(math.hypot(p[0]-q[0], p[1]-q[1])
                            for j, q in enumerate(unique) if j != i)
                        for i, p in enumerate(unique)]
    # Do not close a hairpin by jumping across its open end after ordinary
    # adjacent cones run out. The absolute configured gap remains an upper
    # bound, with a local spacing bound for densely populated tracks.
    chain_gap = min(settings.road_max_gap_m,
                    max(3*settings.road_sample_spacing_m, 3*_estimation_median(nearest_spacings)))
    def grow(direction, seed, used):
        chain, current = [], unique[seed]
        while len(used) < len(unique):
            choices = []
            for i, p in enumerate(unique):
                if i in used:
                    continue
                dx, dy = p[0]-current[0], p[1]-current[1]
                distance = math.hypot(dx, dy)
                cosine = (dx*direction[0]+dy*direction[1])/distance
                if distance <= chain_gap+1e-9 and cosine >= -0.25:
                    choices.append((distance*(1+0.5*(1-cosine)), distance, i, dx, dy))
            if not choices:
                break
            _, distance, i, dx, dy = min(choices)
            used.add(i)
            current = unique[i]
            chain.append(current)
            direction = (dx/distance, dy/distance)
        return chain

    # The nearest same-colour point can be a misclassification on the OTHER
    # boundary. Compare at most six near seeds and retain the best supported
    # contiguous chain, rather than letting an isolated short cluster win.
    # Near-seed bounds keep a long but distant branch from replacing entry.
    choices = []
    nearest_range = math.hypot(*unique[0][:2])
    for seed in range(min(6, len(unique))):
        if math.hypot(*unique[seed][:2]) > nearest_range+chain_gap:
            continue
        used = {seed}
        ahead = grow((1.0, 0.0), seed, used)
        behind = grow((-1.0, 0.0), seed, used)
        chain = list(reversed(behind))+[unique[seed]]+ahead
        lengths = _path_lengths(chain)
        if len(chain) >= 2 and lengths[-1] >= settings.road_min_coverage_m:
            near = min(math.hypot(*p[:2]) for p in chain)
            support = sum(p[2] for p in chain)/(1+max(0.0, near-nearest_range)/chain_gap)
            choices.append((support, lengths[-1], -near, -seed, chain, lengths))
    if not choices:
        return None
    _, _, _, _, chain, lengths = max(choices, key=lambda c: c[:4])
    return {"support_points": chain, "range": (0.0, lengths[-1]),
            "lengths": lengths, "rms_m": None,
            "inlier_fraction": len(chain)/len(unique), "encoding": "ordered_xy"}


class EstimationPipeline:
    """Local geometry aligned to the state epoch with bounded planar odometry.

    Draft names remain canonical; aliases support the supplied screenshots.
    Raw road() reconstructs acquisition-frame geometry. update() transports it
    to the current body frame, or rejects it if historical motion is missing.
    This is approximate forward odometry, not SLAM or calibrated localization.
    """
    QUALITY_FAILURES = ("insufficient_geometry", "single_side_width_unknown",
                        "single_side_inferred", "insufficient_common_range")
    # A stale or missing camera frame while wheel/gyro stay continuous. With
    # history_propagation=continuous these end the output, not the history.
    CAMERA_GAP_FAILURES = ("unavailable", "motion_interval_too_long")

    def __init__(self, settings, frame_id="base_link", left_colour=None, right_colour=None,
                 motion_history=None):
        self.settings, self.frame_id = settings, frame_id
        self.left_colour = settings.left_colour if left_colour is None else left_colour
        self.right_colour = settings.right_colour if right_colour is None else right_colour
        self.motion = motion_history or EstimationMotionHistory(settings)
        self._clear_road_history()

    def _clear_road_history(self, *, keep_learned_width=False):
        if not keep_learned_width:
            self._learned_lane_width_m = None
            self._learned_lane_width_stamp_s = None
        self._road_cones = []  # (x, y, z, colour, confidence, original_seen_s)
        self._road_stamp = None
        self._fused_source_s = None  # camera time of the last fused frame
        self._road_frame = None
        self._observation_stamp = None
        self._observation_road = None
        self._trusted_road = None  # Unaligned acquisition-frame geometry only.
        self._failure_since_s = None
        self._prediction_motion_stamp = None
        self._prediction_distance_m = 0.0
        self._prediction_pose = None
        self._prediction_expired_source_s = None
        self._road_reset_id = self.motion.reset_id

    def _advance_prediction_distance(self, now_s):
        """Count travel once per policy epoch, independent of camera rebasing.

        Integrated speed counts turning as travel, rather than using a net
        displacement that could hide a manoeuvre. The unsigned wheel contract
        still supports forward motion only. A missing motion segment discards
        both the prediction anchor and the cone cache.
        """
        if self._prediction_motion_stamp is None:
            return
        pose, problem = self.motion.pose_between(self._prediction_motion_stamp, now_s)
        if problem is not None:
            self._clear_road_history()
            return
        self._prediction_distance_m += pose["travelled_distance_m"]
        total = self._prediction_pose
        c, s = math.cos(total["yaw_rad"]), math.sin(total["yaw_rad"])
        total["dx_m"] += c*pose["dx_m"]-s*pose["dy_m"]
        total["dy_m"] += s*pose["dx_m"]+c*pose["dy_m"]
        total["yaw_rad"] += pose["yaw_rad"]
        total["interval_s"] += pose["interval_s"]
        total["travelled_distance_m"] = self._prediction_distance_m
        self._prediction_motion_stamp = now_s

    def _advance_history(self, now_s):
        """Continuous mode: move the cone history to now_s over one short interval.

        Called every policy step, so no single motion interval spans a camera
        gap. Cones behind road_rear_support_m or past their lifetime drop out.
        A motion problem still clears the history, as in camera mode.
        """
        if self._road_reset_id != self.motion.reset_id:
            self._clear_road_history()
            return
        if not self._road_cones or self._road_stamp is None or now_s <= self._road_stamp:
            return
        pose, problem = self.motion.pose_between(self._road_stamp, now_s)
        if problem is not None:
            self._clear_road_history()
            return
        cones = []
        for x, y, z, colour, confidence, seen in self._road_cones:
            x, y = self.motion.transform_xy((x, y), pose)
            if x >= -self.settings.road_rear_support_m and now_s-seen <= self.settings.history_lifetime_s:
                cones.append((x, y, z, colour, confidence, seen))
        self._road_cones, self._road_stamp = cones, now_s

    def _history_at(self, source_s):
        """History cones in the camera frame at source_s: (cones, None) or (None, problem).

        Camera mode stores history at the last fused camera time (earlier than
        source_s); continuous mode has already moved it past source_s.
        """
        if self._road_stamp is None:
            return [], None
        if source_s >= self._road_stamp:
            pose, problem = self.motion.pose_between(self._road_stamp, source_s)
            move = self.motion.transform_xy
        else:
            pose, problem = self.motion.pose_between(source_s, self._road_stamp)
            move = self.motion.inverse_transform_xy
        if problem is not None:
            return None, problem
        return [(*move(p[:2], pose), *p[2:]) for p in self._road_cones], None

    @staticmethod
    def _observed_support_conflicts(x, y, fit, tolerance):
        """Check real inlier support, not a global quadratic's model residual.

        A changing view of a bend changes its best quadratic even when the
        observed cones agree. That must not discard all unseen near history.
        Actual contradictions within overlapping measured support still clear
        history; no comparison or extension is made outside the observed span.
        """
        if fit.get("encoding") == "ordered_xy":
            points = [p[:2] for p in fit["support_points"]]
            projection = _path_project(points, _path_lengths(points), (x, y), forward=False)
            _, _, _, i, u = projection
            if (i == 0 and u == 0) or (i == len(points)-2 and u == 1):
                return False  # Beyond observed endpoints is not a contradiction.
            return math.sqrt(projection[0]) > tolerance
        if not fit["range"][0] <= x <= fit["range"][1]:
            return False
        points = fit["support_points"]
        at_x = [p for p in points if abs(p[0]-x) <= 1e-9]
        if at_x:
            return min(abs(y-p[1]) for p in at_x) > tolerance
        for a, b in zip(points, points[1:]):
            if a[0] < x < b[0]:
                observed_y = a[1]+(b[1]-a[1])*(x-a[0])/(b[0]-a[0])
                return abs(y-observed_y) > tolerance
        return False

    def _history_road(self, batch, stamp_ns, age, now_s):
        """Merge bounded, validated observations; never extrapolate a blind start.

        At most 64 cones per side, with latest observations taking precedence
        within the Huber residual radius. Stored ages refer to real detections,
        not predictions. A fresh two-sided fit seeds observed history; when
        enabled, valid partial fits can extend existing two-sided support.
        A bounded quality-drop bridge may reuse the last fully valid corridor.
        """
        if self._road_reset_id != self.motion.reset_id:
            self._clear_road_history()
        source_s = stamp_ns / 1e9 if stamp_ns is not None else None
        if (not self._fresh(batch, age) or source_s is None or source_s > now_s
                or (self._observation_stamp is not None and source_s < self._observation_stamp)):
            out_of_order = (self._observation_stamp is not None and source_s is not None
                            and source_s < self._observation_stamp)
            # A camera gap (stale or missing batch) keeps a continuously moved
            # history; out-of-order or future stamps still clear it.
            if not (self.settings.history_propagation == "continuous" and not out_of_order
                    and (source_s is None or source_s <= now_s)):
                self._clear_road_history()
            result = self.road(batch, stamp_ns, age)
            if out_of_order:
                result.update(valid=False, status="out_of_order_camera", centerline_xy=[],
                              left_boundary_xy=None, right_boundary_xy=None)
            return result
        expired = any(now_s - p[5] > self.settings.history_lifetime_s for p in self._road_cones)
        if source_s == self._fused_source_s and self._road_frame is not None and not expired:
            result = deepcopy(self._road_frame)
            result["source_age_s"] = age
            return result

        if source_s != self._observation_stamp or self._observation_road is None:
            self._observation_stamp = source_s
            self._observation_road = self.road(batch, stamp_ns, age)
            # Learn ONLY from a fresh raw frame that independently fits BOTH
            # sides. Merged history and inferred/predicted roads never teach
            # a width or refresh its acquisition timestamp.
            if (self.settings.single_side_from_learned_width
                    and self._observation_road["valid"]
                    and self._observation_road["visibility"] == "both"):
                self._learned_lane_width_m = self._observation_road["lane_width_m"]
                self._learned_lane_width_stamp_s = source_s
        current = deepcopy(self._observation_road)
        current["source_age_s"] = age
        if current.get("single_side_inferred_from_learned_width"):
            # Use today's visible boundary with the remembered HALF width.
            # Old opposite-side cones must not veto or distort that trajectory.
            # Width remains fixed until both boundaries are observed again;
            # current camera and wheel/gyro deadlines still govern every step.
            if self.settings.road_geometry == "ordered_xy":
                # Retain only the actually observed visible chain for a
                # bounded camera dropout. Do not fabricate opposite cones.
                self._road_cones = [(x, y, 0.0, colour, confidence, source_s)
                    for colour, fit in current["_boundary_fits"].items()
                    if (colour == self.left_colour) == (current["visibility"] == "left_only")
                    for x, y, confidence in fit["support_points"]]
                self._road_stamp = self._fused_source_s = source_s
                self._road_frame = deepcopy(current)
            else:
                self._clear_road_history(keep_learned_width=True)
            return current
        if not current["valid"] or current["visibility"] != "both":
            if current["status"] not in self.QUALITY_FAILURES:
                self._clear_road_history()
                return current
            # Even a one-sided fit can contradict an old corridor. Never hide
            # an observed change behind a prediction from the other boundary.
            placed, _ = self._history_at(source_s)
            for x, y, _, colour, _, _ in placed or []:
                fit = current.get("_boundary_fits", {}).get(colour)
                if fit is None:
                    continue
                if self._observed_support_conflicts(x, y, fit, 2*self.settings.huber_delta_m):
                    self._clear_road_history()
                    current.update(valid=False, status="observed_history_conflict")
                    return current
            # history_merge_single_side: the visible side(s) of a one-sided or
            # short-overlap frame join the history of both sides below. A failed
            # merge returns this frame unchanged and leaves the history alone.
            if not (self.settings.history_merge_single_side and self._road_cones
                    and current.get("_boundary_fits")):
                return current
            partial = True
        else:
            partial = False
        fits = current.pop("_boundary_fits")
        placed, _ = self._history_at(source_s)
        previous = [p for p in placed or []
                    if 0 <= now_s-p[5] <= self.settings.history_lifetime_s
                    and -self.settings.road_rear_support_m <= p[0] <= self.settings.road_forward_max_m]
        # An observed change in the overlapping corridor invalidates old support.
        radius = 2*self.settings.huber_delta_m
        for colour, fit in fits.items():
            if any(p[3] == colour and self._observed_support_conflicts(p[0], p[1], fit, radius)
                   for p in previous):
                previous = []
                break
        fresh = [(x, y, 0.0, colour, confidence, source_s)
                 for colour, fit in fits.items() for x, y, confidence in fit["support_points"]]
        merge_radius = self.settings.history_merge_radius_m or radius
        for point in fresh:
            previous = [p for p in previous if p[3] != point[3]
                        or math.hypot(p[0]-point[0], p[1]-point[1]) > merge_radius]
        merged = previous + fresh
        bounded = []
        for colour in (self.left_colour, self.right_colour):
            side = sorted((p for p in merged if p[3] == colour), key=lambda p: p[0])
            if len(side) > 64:
                side = [side[round(i*(len(side)-1)/63)] for i in range(64)]
            bounded.extend(side)
        result = self.road({"detections": [p[:5] for p in bounded]}, stamp_ns, age)
        if not result["valid"] or (partial and result["visibility"] != "both"):
            if partial:
                current.pop("_support_cones", None)
                return current
            # Never let a history-induced bad fit poison subsequent frames.
            bounded, result = fresh, current
        else:
            result.pop("_boundary_fits", None)
            # Keep only inliers of the combined fit, including their original age.
            supported = result.pop("_support_cones")
            bounded = [p for p in bounded if p[:5] in supported]
        current.pop("_support_cones", None)
        result.pop("_support_cones", None)
        result["history_used"] = any(p[5] < source_s for p in bounded)
        if result["history_used"]:
            result["status"] = "observed_with_history"
            result["boundary_source"] = {"left": "observed_with_history", "right": "observed_with_history"}
            if partial:
                # The side without fresh support in this frame is history only.
                result["boundary_source"] = {
                    side: "observed_with_history" if colour in fits else "history"
                    for side, colour in (("left", self.left_colour), ("right", self.right_colour))}
                result["merged_partial_frame"] = True
        result["oldest_observation_timestamp_s"] = min(p[5] for p in bounded)
        self._road_cones, self._road_stamp = bounded, source_s
        self._fused_source_s = source_s
        self._road_frame = deepcopy(result)
        return result

    def _predict_road(self, current_frame, now_s):
        """Transport one last fully valid observation; never promote prediction.

        Geometry remains anchored at its real acquisition timestamp. Repeated
        failed batches do not renew either the observation or the hold budget.
        """
        if self._trusted_road is None or self.settings.road_hold_s == 0:
            return None, "no_trusted_road"
        if self.settings.continuous_road_output:
            return self._predict_continuous_road(current_frame, now_s)
        if self._failure_since_s is None:
            self._failure_since_s = now_s
        elapsed = now_s-self._failure_since_s
        source_s = self._trusted_road["timestamp_s"]
        age = now_s-source_s
        oldest = self._trusted_road.get("oldest_observation_timestamp_s", source_s)
        remaining = min(self.settings.road_hold_s-elapsed,
                        self.settings.max_source_age_s-age,
                        self.settings.motion_max_interval_s-age,
                        self.settings.history_lifetime_s-(now_s-oldest))
        if elapsed < 0 or age < 0 or remaining <= 1e-9:
            return None, "prediction_deadline_expired"
        predicted = deepcopy(self._trusted_road)
        predicted.update(source_age_s=age, history_used=True)
        predicted = self._align_road(predicted, now_s, True)
        if not predicted["valid"]:
            return None, predicted["status"]
        predicted.update(status="predicted_history", degraded=True,
                         current_frame=current_frame, prediction_status="active",
                         prediction_elapsed_s=elapsed, prediction_age_s=age,
                         prediction_remaining_s=remaining, recommended_speed_scale=0.5,
                         confidence=0.5*predicted["confidence"],
                         boundary_source={"left": "predicted_history", "right": "predicted_history"})
        return predicted, "active"

    def _predict_continuous_road(self, current_frame, now_s):
        """Fit moved REAL observations, keeping the last accepted camera epoch.

        Incremental wheel/gyro segments each obey the usual motion limits. The
        total prediction still has independent source-age, hold and travel
        deadlines. Neither a predicted fit nor a repeated frame renews them.
        """
        if self._failure_since_s is None:
            self._failure_since_s = now_s
        source_s = self._trusted_road["timestamp_s"]
        age, elapsed = now_s-source_s, now_s-self._failure_since_s
        remaining = min(self.settings.road_hold_s-elapsed,
                        self.settings.max_source_age_s-age)
        distance_left = self.settings.road_prediction_max_distance_m-self._prediction_distance_m
        if age < 0 or elapsed < 0 or remaining <= 1e-9 or distance_left <= 1e-9:
            return None, "prediction_deadline_expired"
        speed = self.motion.current("wheel_speed", now_s)
        if speed is not None and speed > 1e-9:
            # Advisory time at current filtered speed; acceleration can consume
            # distance faster, so consumers must still recheck every update.
            remaining = min(remaining, distance_left/speed)
        placed, problem = self._history_at(now_s)
        if problem is not None:
            return None, problem
        if not placed:
            return None, "no_observed_history"
        oldest = min(p[5] for p in placed)
        remaining = min(remaining, self.settings.history_lifetime_s-(now_s-oldest))
        if remaining <= 1e-9:
            return None, "near_field_history_expired"
        predicted = self.road({"detections": [p[:5] for p in placed]}, round(now_s*1e9), 0.0)
        predicted.update(timestamp_s=now_s, history_used=True, oldest_observation_timestamp_s=oldest)
        predicted = self._align_road(predicted, now_s, True)
        if (self.settings.road_geometry == "ordered_xy" and not predicted["valid"]
                and (predicted.get("forward_geometry") or {}).get("single_side_inferred_from_learned_width")):
            predicted = predicted["forward_geometry"]
            predicted["valid"] = True
        if not predicted["valid"]:
            return None, predicted["status"]
        # The fit above is in today's body frame; it is not a new camera sample.
        # Preserve acquisition provenance separately from the aligned epoch.
        predicted.update(measurement_timestamp_s=source_s, source_age_s=age,
                         motion_compensated=age > 0, motion_compensation=deepcopy(self._prediction_pose),
                         fusion_mode="observed_cones_wheel_gyro_incremental",
                         status="predicted_history", degraded=True,
                         current_frame=current_frame, prediction_status="active",
                         prediction_elapsed_s=elapsed, prediction_age_s=age,
                         prediction_remaining_s=remaining,
                         prediction_distance_m=self._prediction_distance_m,
                         prediction_remaining_distance_m=distance_left,
                         recommended_speed_scale=0.5,
                         confidence=0.5*predicted["confidence"],
                         boundary_source={side: "predicted_history" if predicted.get(side+"_boundary_xy")
                                          else "inferred_width" for side in ("left", "right")})
        return predicted, "active"

    def _fresh(self, value, age):
        return value is not None and finite_number(age) and 0 <= age < self.settings.max_source_age_s

    def road(self, batch, stamp_ns, age):
        timestamp = stamp_ns/1e9 if stamp_ns is not None and stamp_ns > 0 else None
        output = {"timestamp_s": timestamp, "frame_id": self.frame_id, "valid": False,
                  "centerline_xy": [], "left_boundary_xy": None, "right_boundary_xy": None,
                  "lane_width_m": None, "local_curvature_1pm": None, "x_range_m": None,
                  "confidence": 0.0, "visibility": "none", "status": "unavailable",
                  "source_age_s": age, "motion_compensated": False,
                  "boundary_source": {"left": "absent", "right": "absent"}}
        output["boundary_diagnostics"] = {
            side: {"raw_count": 0, "filtered_count": 0, "fit_valid": False,
                   "fit_status": "unavailable", "inlier_count": 0,
                   "fit_rms_m": None, "x_range_m": None}
            for side in ("left", "right")}
        if not self._fresh(batch, age) or timestamp is None:
            return output
        sides = {self.left_colour: [], self.right_colour: []}
        for x, y, z, colour, confidence in batch["detections"]:
            if colour in sides:
                side = "left" if colour == self.left_colour else "right"
                output["boundary_diagnostics"][side]["raw_count"] += 1
            if (all(finite_number(v) for v in (x, y, z, confidence))
                    and -self.settings.road_rear_support_m <= x <= self.settings.road_forward_max_m
                    and abs(y) <= self.settings.road_width_max_m + self.settings.road_forward_max_m*self.settings.road_max_abs_slope
                    and self.settings.min_cone_confidence <= confidence <= 1.0
                    and confidence > 0 and colour in sides):
                sides[colour].append((x, y, confidence))
        if self.settings.road_geometry == "ordered_xy":
            return self._ordered_road(output, sides)
        left = _estimation_fit_boundary(sides[self.left_colour], self.settings)
        right = _estimation_fit_boundary(sides[self.right_colour], self.settings)
        output["_boundary_fits"] = {colour: fit for colour, fit in
                                    ((self.left_colour, left), (self.right_colour, right)) if fit is not None}
        for side, colour, fit in (("left", self.left_colour, left), ("right", self.right_colour, right)):
            diag = output["boundary_diagnostics"][side]
            diag["filtered_count"] = len(sides[colour])
            points = sides[colour]
            diag["fit_status"] = ("insufficient_points" if len(points) < 2 else
                                  "insufficient_x_coverage" if max(p[0] for p in points)-min(p[0] for p in points)
                                  < self.settings.road_min_coverage_m else "fit_rejected")
            if fit is not None:
                diag.update(fit_valid=True, fit_status="accepted", inlier_count=len(fit["support_points"]),
                            fit_rms_m=fit["rms_m"], x_range_m=list(fit["range"]))
        output["status"] = "insufficient_geometry"
        if left is None and right is None:
            return output
        lane_width = self.settings.known_lane_width_m
        learned_single_side = False
        if left is None or right is None:
            output["visibility"] = "left_only" if left is not None else "right_only"
            if (self.settings.single_side_from_learned_width
                    and self._learned_lane_width_m is not None):
                lane_width = self._learned_lane_width_m
                learned_single_side = True
            if lane_width == 0:
                output["status"] = "single_side_width_unknown"
                return output
        fits = [fit for fit in (left, right) if fit is not None]
        lo, hi = max(f["range"][0] for f in fits), min(f["range"][1] for f in fits)
        if hi-lo < self.settings.road_min_coverage_m:
            if (len(fits) == 2 and self.settings.single_side_from_learned_width
                    and self._learned_lane_width_m is not None):
                # A few returning FAR cones do not yet define a two-sided
                # corridor beside the visible near boundary. Keep the near
                # side with the last learned width until common support returns.
                chosen = min(fits, key=lambda f: (f["range"][0],
                                                  -(f["range"][1]-f["range"][0])))
                left, right = (chosen, None) if chosen is left else (None, chosen)
                output["visibility"] = "left_only" if left is not None else "right_only"
                output["single_side_fallback_reason"] = "insufficient_common_range"
                lane_width = self._learned_lane_width_m
                learned_single_side = True
                fits = [chosen]
                lo, hi = chosen["range"]
            else:
                output["status"] = "insufficient_common_range"
                return output
        count = min(200, math.ceil((hi-lo)/self.settings.road_sample_spacing_m))
        center, widths, samples = [], [], {"left": [], "right": []}
        for i in range(count+1):
            x = lo+(hi-lo)*i/count
            evaluated = []
            for side, fit in (("left", left), ("right", right)):
                if fit is None:
                    evaluated.append(None)
                    continue
                a0, a1, a2 = fit["coeffs"]
                y, slope = a0+a1*x+a2*x*x, a1+2*a2*x
                samples[side].append((x, y))
                evaluated.append((y, slope))
            if all(v is not None for v in evaluated):
                (yl, sl), (yr, sr) = evaluated
                slope = (sl+sr)/2
                width = (yl-yr)/math.hypot(1, slope)
                center.append((x, (yl+yr)/2))
            else:
                y, slope = next(v for v in evaluated if v is not None)
                width = lane_width
                # Offset along the local normal, not simply along body y.
                direction = -1 if left is not None else 1
                offset = direction*width*self.settings.single_side_offset_fraction/math.hypot(1, slope)
                center.append((x-offset*slope, y+offset))
            widths.append(width)
        if (any(not self.settings.road_width_min_m <= w <= self.settings.road_width_max_m for w in widths)
                or any(b[0] <= a[0] for a, b in zip(center, center[1:]))
                or center[0][0] < -self.settings.road_rear_support_m
                or center[-1][0]-center[0][0] < self.settings.road_min_coverage_m):
            output["status"] = "invalid_width_or_order"
            return output
        output.update(valid=True, centerline_xy=center, lane_width_m=_estimation_median(widths),
                      x_range_m=[center[0][0], center[-1][0]],
                      confidence=min(f["inlier_fraction"] for f in fits) * (1.0 if len(fits) == 2 else 0.5),
                      visibility="both" if len(fits) == 2 else output["visibility"],
                      status="observed" if len(fits) == 2 else "single_side_inferred",
                      fit_rms_m=max(f["rms_m"] for f in fits))
        if learned_single_side:
            output.update(single_side_inferred_from_learned_width=True,
                          lane_width_source="last_two_sided_observation",
                          lane_width_observed_at_s=self._learned_lane_width_stamp_s,
                          inward_offset_m=lane_width*self.settings.single_side_offset_fraction,
                          centerline_source="left_boundary_inward_offset" if left is not None
                          else "right_boundary_inward_offset")
        for side, fit in (("left", left), ("right", right)):
            output[f"{side}_boundary_xy"] = samples[side] if fit is not None else None
            output["boundary_source"][side] = "observed" if fit is not None else "absent"
        output["_boundary_fits"] = {colour: fit for colour, fit in
                                    ((self.left_colour, left), (self.right_colour, right)) if fit is not None}
        output["_support_cones"] = [(x, y, 0.0, colour, confidence)
                                    for colour, fit in output["_boundary_fits"].items()
                                    for x, y, confidence in fit["support_points"]]
        if len(fits) == 2:
            a0 = (left["coeffs"][0]+right["coeffs"][0])/2
            a1 = (left["coeffs"][1]+right["coeffs"][1])/2
            a2 = (left["coeffs"][2]+right["coeffs"][2])/2
            output["_center_coeffs"] = (a0, a1, a2)
            slope = a1+2*a2*lo
            output["local_curvature_1pm"] = 2*a2/(1+slope*slope)**1.5
        # For an offset curve, do not publish its boundary curvature as the
        # centreline's curvature. None is honest until that derivation is added.
        return output

    def _ordered_road(self, output, sides, reference_side=None):
        """Normal-paired, ordered local boundary chains and single-side offsets.

        Pairing follows the opposite chain's travel order and requires agreeing
        tangents. It never pairs a U-turn entry to its rear-facing exit. Samples
        are restricted to measured support; no endpoint tangent extension.
        """
        cfg = self.settings
        fits = {name: _estimation_order_boundary(sides[colour], cfg)
                for name, colour in (("left", self.left_colour), ("right", self.right_colour))}
        output["geometry_encoding"] = "ordered_xy"
        output["_boundary_fits"] = {}
        for name, colour in (("left", self.left_colour), ("right", self.right_colour)):
            fit = fits[name]
            diag = output["boundary_diagnostics"][name]
            diag.update(filtered_count=len(sides[colour]), fit_valid=fit is not None,
                        fit_status="accepted" if fit else "insufficient_chain_support")
            if fit:
                diag.update(inlier_count=len(fit["support_points"]),
                            arc_length_m=fit["lengths"][-1])
                output["_boundary_fits"][colour] = fit
        output["status"] = "insufficient_geometry"
        if not any(fits.values()):
            return output
        near = lambda name: min(p[0]**2+p[1]**2 for p in fits[name]["support_points"])
        side = reference_side or min((name for name in fits if fits[name]), key=near)
        fit, other = fits[side], fits["right" if side == "left" else "left"]
        boundary = [p[:2] for p in fit["support_points"]]
        total = min(fit["lengths"][-1], cfg.road_forward_max_m+cfg.road_rear_support_m)
        count = min(200, math.ceil(total/cfg.road_sample_spacing_m))
        candidates, runs, current, previous_other_s = [], [], [], -1.0
        pairing_conflict = False
        for i in range(count+1):
            distance = total*i/count
            p = _path_sample(boundary, fit["lengths"], distance)
            tx, ty = _path_tangent(boundary, fit["lengths"], distance)
            nx, ny = (ty, -tx) if side == "left" else (-ty, tx)
            matches = []
            if other:
                for j, (a, b) in enumerate(zip(other["support_points"], other["support_points"][1:])):
                    dx, dy = b[0]-a[0], b[1]-a[1]
                    length = other["lengths"][j+1]-other["lengths"][j]
                    if (tx*dx+ty*dy)/length < 0.3:
                        continue
                    denominator = nx*dy-ny*dx
                    if abs(denominator) < 1e-9:
                        continue
                    ax, ay = a[0]-p[0], a[1]-p[1]
                    width = (ax*dy-ay*dx)/denominator
                    u = (ax*ny-ay*nx)/denominator
                    if -1e-6 <= u <= 1+1e-6 and not cfg.road_width_min_m <= width <= cfg.road_width_max_m:
                        pairing_conflict = True
                    other_s = other["lengths"][j]+min(1.0, max(0.0, u))*length
                    if (cfg.road_width_min_m <= width <= cfg.road_width_max_m
                            and -1e-6 <= u <= 1+1e-6
                            and other_s >= previous_other_s-cfg.ordered_pairing_backtrack_m):
                        # Prefer the expected width once measured, otherwise
                        # the first inward boundary crossing on this branch.
                        expected = self._learned_lane_width_m
                        score = abs(width-expected) if expected is not None else width
                        matches.append((score, width, other_s))
            if matches:
                _, width, previous_other_s = min(matches)
                current.append(((p[0]+nx*width/2, p[1]+ny*width/2), width))
            elif current:
                runs.append(current)
                current = []
            candidates.append((p, nx, ny))
        if current:
            runs.append(current)
        runs = [run for run in runs if len(run) >= 2
                and _path_lengths([p for p, _ in run])[-1] >= cfg.road_min_coverage_m]
        inferred = not runs
        if (inferred and reference_side is None and other is not None
                and other["lengths"][-1] > fit["lengths"][-1]):
            # Common support has disappeared: the nearest boundary may now
            # be only a short noisy fragment. Prefer the longer actually
            # observed chain for remembered-width inference, once per frame.
            return self._ordered_road(output, sides,
                                      reference_side="right" if side == "left" else "left")
        if not inferred:
            # Prefer the nearest usable contiguous corridor over a longer far
            # branch; discarded gaps never become implicit connecting lines.
            run = min(runs, key=lambda r: min(p[0]**2+p[1]**2 for p, _ in r))
            center = [p for p, _ in run]
            width = _estimation_median([w for _, w in run])
        else:
            width = (self._learned_lane_width_m if cfg.single_side_from_learned_width
                     else None) or cfg.known_lane_width_m
            output["visibility"] = side+"_only"
            if width == 0:
                output["status"] = "single_side_width_unknown" if other is None else "insufficient_common_range"
                return output
            # A visible but contradictory opposite chain must not be hidden by
            # a width fallback. Missing/too-short chains are handled normally.
            if other is not None and pairing_conflict:
                output["status"] = "invalid_boundary_pairing"
                return output
            if other is not None:
                output["single_side_fallback_reason"] = "insufficient_common_range"
            offset = width*cfg.single_side_offset_fraction
            center = [(p[0]+nx*offset, p[1]+ny*offset) for p, nx, ny in candidates]
        # One corner-cutting pass removes polyline heading steps. Bound each
        # displacement by the local half-width, retaining measured endpoints.
        # This is geometric smoothing, not a calibrated turning-radius planner.
        smooth = [center[0]]
        for a, p, b in zip(center, center[1:], center[2:]):
            q = ((a[0]+2*p[0]+b[0])/4, (a[1]+2*p[1]+b[1])/4)
            smooth.append(q if math.hypot(q[0]-p[0], q[1]-p[1]) <= width/8 else p)
        smooth.append(center[-1])
        if _path_points(smooth) is None or _path_lengths(smooth)[-1] < cfg.road_min_coverage_m:
            output["status"] = "invalid_ordered_geometry"
            return output
        # Normal offsets change arc length, especially on an inner boundary.
        # Resample the resulting centreline by its OWN length for consumers.
        center_lengths = _path_lengths(smooth)
        count = min(200, math.ceil(center_lengths[-1]/cfg.road_sample_spacing_m))
        smooth = [_path_sample(smooth, center_lengths, center_lengths[-1]*i/count) for i in range(count+1)]
        output.update(valid=True, centerline_xy=smooth, lane_width_m=width,
                      x_range_m=[min(p[0] for p in smooth), max(p[0] for p in smooth)],
                      arc_length_m=_path_lengths(smooth)[-1], local_curvature_1pm=None,
                      visibility=side+"_only" if inferred else "both",
                      status="single_side_inferred" if inferred else "observed",
                      confidence=min(f["inlier_fraction"] for f in fits.values() if f)*(0.5 if inferred else 1.0),
                      centerline_source=side+"_boundary_inward_offset" if inferred else "two_boundaries")
        supported = []
        for name, colour in (("left", self.left_colour), ("right", self.right_colour)):
            f = fits[name]
            output[name+"_boundary_xy"] = [p[:2] for p in f["support_points"]] if f else None
            output["boundary_source"][name] = "observed" if f else "absent"
            if f:
                supported.extend((x, y, 0.0, colour, confidence) for x, y, confidence in f["support_points"])
        output["_support_cones"] = supported
        if inferred and self._learned_lane_width_m is not None:
            output.update(single_side_inferred_from_learned_width=True,
                          lane_width_source="last_two_sided_observation",
                          inward_offset_m=width*cfg.single_side_offset_fraction,
                          lane_width_observed_at_s=self._learned_lane_width_stamp_s)
        return output

    def _align_road(self, road, now_s, state_valid):
        center_coeffs = road.pop("_center_coeffs", None)
        road.pop("_boundary_fits", None)
        road.pop("_support_cones", None)
        road["geometry_valid"] = False
        road["near_field"] = {"valid": False, "status": "unavailable",
                              "coverage_x_m": None, "uses_history": road.get("history_used", False),
                              "oldest_observation_age_s": None, "footprint_validated": False}
        source_time = road["timestamp_s"]
        road.update(measurement_timestamp_s=source_time, timestamp_s=None,
                    time_aligned=False, motion_compensation=None)
        if not road["valid"]:
            return road
        if road.get("geometry_encoding") == "ordered_xy":
            return self._align_ordered_road(road, source_time, now_s, state_valid)
        pose, problem = self.motion.pose_between(source_time, now_s)
        if not state_valid:
            problem = "motion_state_unavailable"
        if problem is None:
            points = [self.motion.transform_xy(p, pose) for p in road["centerline_xy"]]
            selected = [i for i, p in enumerate(points)
                        if all(math.isfinite(v) for v in p)
                        and -self.settings.road_rear_support_m <= p[0] <= self.settings.road_forward_max_m]
            center = [points[i] for i in selected]
            if (any(not all(math.isfinite(v) for v in p) for p in points)
                    or len(center) < 2
                    or center[-1][0]-center[0][0] < self.settings.road_min_coverage_m
                    or any(b[0] <= a[0] or abs(b[1]-a[1]) > self.settings.road_max_abs_slope*(b[0]-a[0])
                           for a, b in zip(center, center[1:]))):
                problem = "motion_transformed_geometry_invalid"
            else:
                if center_coeffs is not None:
                    slope = center_coeffs[1]+2*center_coeffs[2]*road["centerline_xy"][selected[0]][0]
                    road["local_curvature_1pm"] = 2*center_coeffs[2]/(1+slope*slope)**1.5
                road["centerline_xy"] = center
                road["x_range_m"] = [center[0][0], center[-1][0]]
                for side in ("left", "right"):
                    key = f"{side}_boundary_xy"
                    if road[key] is not None:
                        boundary = [self.motion.transform_xy(p, pose) for p in road[key]]
                        road[key] = [p for p in boundary
                                     if -self.settings.road_rear_support_m <= p[0] <= self.settings.road_forward_max_m]
                        if any(not all(math.isfinite(v) for v in p) for p in boundary):
                            problem = "motion_transformed_geometry_invalid"
                # Rigid motion preserves curvature at the SAME physical point.
                # If clipping removes that first point, do not relabel its value.
                if selected[0] != 0 and center_coeffs is None:
                    road["local_curvature_1pm"] = None
                road.update(timestamp_s=now_s, time_aligned=True,
                            motion_compensated=pose["interval_s"] > 0,
                            motion_compensation=pose)
                road["geometry_valid"] = problem is None
                boundaries = [road.get(f"{side}_boundary_xy") for side in ("left", "right")]
                if (any(not points or any(b[0] <= a[0] for a, b in zip(points, points[1:]))
                        for points in boundaries)):
                    problem = "near_field_requires_both_boundaries"
                else:
                    lo = max(center[0][0], *(points[0][0] for points in boundaries))
                    hi = min(center[-1][0], *(points[-1][0] for points in boundaries))
                    oldest = road.get("oldest_observation_timestamp_s", source_time)
                    road["near_field"].update(coverage_x_m=[lo, hi],
                                              oldest_observation_age_s=max(0.0, now_s-oldest))
                    if lo > 1e-9 or hi < 0:
                        problem = "near_field_unobserved"
                    elif now_s-oldest > self.settings.history_lifetime_s:
                        problem = "near_field_history_expired"
                    else:
                        road["near_field"].update(valid=True, status="observed_support")
        if problem is not None:
            if (road["geometry_valid"] and (problem == "near_field_unobserved"
                    or (problem == "near_field_requires_both_boundaries"
                        and road.get("single_side_inferred_from_learned_width")))):
                # Retain validated, aligned FORWARD observations separately.
                # Estimator validity stays false; only an explicitly configured
                # planning extension may consume this incomplete support.
                forward = deepcopy(road)
                forward.update(valid=False, status=problem)
                road["forward_geometry"] = forward
            road["near_field"]["status"] = problem
            road.update(valid=False, status=problem, timestamp_s=None,
                        time_aligned=False, motion_compensated=False,
                        centerline_xy=[], left_boundary_xy=None, right_boundary_xy=None,
                        lane_width_m=None, x_range_m=None, local_curvature_1pm=None,
                        confidence=0.0, motion_compensation=None)
        return road

    def _align_ordered_road(self, road, source_time, now_s, state_valid):
        pose, problem = self.motion.pose_between(source_time, now_s)
        if not state_valid:
            problem = "motion_state_unavailable"
        if problem is None:
            center = [self.motion.transform_xy(p, pose) for p in road["centerline_xy"]]
            points = _path_points(center)
            if points is None:
                problem = "motion_transformed_geometry_invalid"
            else:
                lengths = _path_lengths(points)
                projection = _path_project(points, lengths)
                if projection is None:
                    problem = "ordered_branch_unavailable"
                else:
                    # Keep one contiguous local arc around this branch. Body
                    # x may turn back; clipping/sorting by x would break U turns.
                    near_s = projection[1]
                    lo = max(0.0, near_s-self.settings.road_rear_support_m)
                    hi = min(lengths[-1], near_s+self.settings.road_forward_max_m)
                    center = [_path_sample(points, lengths, lo)]
                    center.extend(p for p, s in zip(points, lengths) if lo+1e-6 < s < hi-1e-6)
                    center.append(_path_sample(points, lengths, hi))
                    if hi-lo < self.settings.road_min_coverage_m or _path_points(center) is None:
                        problem = "motion_transformed_geometry_invalid"
                    else:
                        road.update(centerline_xy=center, arc_length_m=_path_lengths(center)[-1],
                                    x_range_m=[min(p[0] for p in center), max(p[0] for p in center)],
                                    timestamp_s=now_s, time_aligned=True, geometry_valid=True,
                                    motion_compensated=pose["interval_s"] > 0, motion_compensation=pose)
                        for side in ("left", "right"):
                            key = side+"_boundary_xy"
                            if road[key] is not None:
                                road[key] = [self.motion.transform_xy(p, pose) for p in road[key]]
                        road["near_field"].update(coverage_s_m=[lo-near_s, hi-near_s],
                            oldest_observation_age_s=now_s-road.get("oldest_observation_timestamp_s", source_time))
                        if road.get("single_side_inferred_from_learned_width"):
                            problem = "near_field_requires_both_boundaries"
                        elif near_s <= 1e-6 or near_s >= lengths[-1]-1e-6:
                            problem = "near_field_unobserved"
                        elif road["near_field"]["oldest_observation_age_s"] > self.settings.history_lifetime_s:
                            problem = "near_field_history_expired"
                        else:
                            road["near_field"].update(valid=True, status="observed_support")
        if problem is not None:
            if road["geometry_valid"] and problem in ("near_field_unobserved", "near_field_requires_both_boundaries"):
                forward = deepcopy(road)
                forward.update(valid=False, status=problem)
                road["forward_geometry"] = forward
            road["near_field"]["status"] = problem
            road.update(valid=False, status=problem, timestamp_s=None, time_aligned=False,
                        centerline_xy=[], left_boundary_xy=None, right_boundary_xy=None,
                        lane_width_m=None, x_range_m=None, arc_length_m=None, confidence=0.0)
        return road

    def update(self, observations, ages, stamps, sample_receipts, now_s,
               sample_receipt_ros_ns=None):
        clock_valid = self.motion.check_clock(now_s)
        receipt_stamps = sample_receipt_ros_ns or {}
        for name, component in (("wheel_speed", None), ("imu_angular_velocity", 2)):
            value = observations.get(name)
            if not clock_valid or not self._fresh(value, ages.get(name)):
                self.motion.clear_source(name)
                continue
            if component is not None:
                value = value[component]
            stamp = stamps.get(name)
            if name == "imu_angular_velocity" and (stamp is None or stamp <= 0):
                self.motion.clear_source(name)
                continue
            key = stamp if stamp is not None else sample_receipts.get(name)
            sample_time = stamp/1e9 if stamp is not None else sample_receipts.get(name)
            # Production uses the ROS receipt saved by _store; standalone
            # numerical snapshots can map receipt age onto their supplied clock.
            receipt = receipt_stamps.get(name)
            ros_time = (stamp/1e9 if stamp is not None else
                        receipt/1e9 if receipt is not None else now_s-ages[name])
            self.motion.observe(name, value, key, sample_time, ros_time)

        speed = self.motion.current("wheel_speed", now_s) if clock_valid else None
        yaw_rate = self.motion.current("imu_angular_velocity", now_s) if clock_valid else None
        state = {"timestamp_s": now_s, "frame_id": self.frame_id,
                 "speed_mps": speed, "yaw_rate_rps": yaw_rate,
                 "v_mps": speed, "yaw_rate_radps": yaw_rate,
                 "speed_valid": speed is not None, "yaw_rate_valid": yaw_rate is not None,
                 "valid": speed is not None and yaw_rate is not None,
                 "confidence": 1.0 if speed is not None and yaw_rate is not None else 0.0,
                 "lateral_error_m": None, "heading_error_rad": None,
                 "road_relative_valid": False, "mode": "causal_filtered_hold",
                 "sample_time_basis": {"wheel_speed": "ros_receipt_proxy",
                                       "imu_angular_velocity": "ros_acquisition"},
                 "source_age_s": {k: ages.get(k) for k in ("wheel_speed", "imu_angular_velocity")},
                 "source_stamp_ns": {k: stamps.get(k) for k in ("wheel_speed", "imu_angular_velocity")}}
        continuous = self.settings.history_propagation == "continuous"
        if not state["valid"]:
            self._clear_road_history()
            road = self.road(observations.get("cone_detections"), stamps.get("cone_detections"),
                             ages.get("cone_detections"))
        else:
            if continuous:
                if self.settings.continuous_road_output:
                    self._advance_prediction_distance(now_s)
                self._advance_history(now_s)
            road = self._history_road(observations.get("cone_detections"), stamps.get("cone_detections"),
                                      ages.get("cone_detections"), now_s)
        observed = (self._observation_road if road["status"] != "unavailable"
                    and self._observation_road is not None else road)
        current_frame = {"timestamp_s": road["timestamp_s"],
                         "source_age_s": ages.get("cone_detections"),
                         "fit_valid": observed["valid"] and observed["visibility"] == "both",
                         "status": observed["status"], "visibility": observed["visibility"],
                         "boundary_diagnostics": deepcopy(observed["boundary_diagnostics"])}
        failure = road["status"]
        if (self.settings.continuous_road_output and self._failure_since_s is not None
                and self._trusted_road is not None and road["valid"]
                and road["timestamp_s"] == self._trusted_road["timestamp_s"]):
            # Re-presenting the old successful camera batch after an outage is
            # not recovery. Only a new accepted camera epoch resets the hold.
            road.update(valid=False, status="camera_frame_not_advanced", centerline_xy=[],
                        left_boundary_xy=None, right_boundary_xy=None, lane_width_m=None,
                        x_range_m=None, local_curvature_1pm=None, confidence=0.0)
            failure = road["status"]
        if (self.settings.continuous_road_output and self._prediction_expired_source_s is not None
                and road["timestamp_s"] is not None
                and road["timestamp_s"] <= self._prediction_expired_source_s):
            road.update(valid=False, status="prediction_deadline_expired", centerline_xy=[],
                        left_boundary_xy=None, right_boundary_xy=None, lane_width_m=None,
                        x_range_m=None, local_curvature_1pm=None, confidence=0.0)
            failure = road["status"]
        trusted_candidate = (deepcopy(road) if road["valid"] and
            (road["visibility"] == "both" or self.settings.road_geometry == "ordered_xy"
             and road.get("single_side_inferred_from_learned_width")) else None)
        road = self._align_road(road, now_s, state["valid"])
        road.update(current_frame=current_frame, degraded=False, prediction_status="not_used",
                    prediction_elapsed_s=None, prediction_age_s=None,
                    prediction_remaining_s=None, recommended_speed_scale=1.0 if road["valid"] else 0.0)
        if road["valid"]:
            if (self.settings.continuous_road_output and
                    (self._trusted_road is None or
                     trusted_candidate["timestamp_s"] != self._trusted_road["timestamp_s"])):
                self._prediction_motion_stamp = now_s
                self._prediction_distance_m = road["motion_compensation"]["travelled_distance_m"]
                self._prediction_pose = deepcopy(road["motion_compensation"])
                self._prediction_expired_source_s = None
            self._trusted_road = trusted_candidate
            self._failure_since_s = None
        elif (road.get("forward_geometry") or {}).get("single_side_inferred_from_learned_width"):
            # Fresh one-sided geometry is a new observation each frame, not
            # a dead-reckoned hold. Legacy y(x) only preserves width here;
            # ordered XY also retains the visible chain for bounded prediction.
            road["prediction_status"] = "fresh_single_side_geometry"
            if self.settings.road_geometry == "ordered_xy" and trusted_candidate is not None:
                forward = road["forward_geometry"]
                if self._trusted_road is None or trusted_candidate["timestamp_s"] != self._trusted_road["timestamp_s"]:
                    self._prediction_motion_stamp = now_s
                    self._prediction_distance_m = forward["motion_compensation"]["travelled_distance_m"]
                    self._prediction_pose = deepcopy(forward["motion_compensation"])
                    self._prediction_expired_source_s = None
                self._trusted_road, self._failure_since_s = trusted_candidate, None
        elif state["valid"] and (failure in self.QUALITY_FAILURES or
                (self.settings.continuous_road_output and road["status"] in
                 self.CAMERA_GAP_FAILURES + ("camera_frame_not_advanced",))):
            predicted, reason = self._predict_road(current_frame, now_s)
            if predicted is not None:
                road = predicted
            else:
                road["prediction_status"] = reason
                # Continuous mode keeps the (still conflict-checked) history;
                # only the trusted-corridor prediction ends here.
                if continuous:
                    if self.settings.continuous_road_output and self._trusted_road is not None:
                        self._prediction_expired_source_s = self._trusted_road["timestamp_s"]
                        self._prediction_motion_stamp = None
                    self._trusted_road, self._failure_since_s = None, None
                else:
                    self._clear_road_history()
        elif road["status"] == "near_field_unobserved":
            # Keep validated forward observations for near-field reconstruction,
            # but never use an incomplete startup corridor as a driving fallback.
            self._trusted_road = None
            self._failure_since_s = None
            self._prediction_motion_stamp = None
        elif continuous and road["status"] == "prediction_deadline_expired":
            # Keep measured near support for a NEW camera frame, but this old
            # camera epoch cannot re-arm an expired prediction on its own.
            self._trusted_road, self._failure_since_s = None, None
            self._prediction_motion_stamp = None
        elif continuous and road["status"] in self.CAMERA_GAP_FAILURES:
            # Camera gap: no output this step, but the history keeps moving with
            # wheel/gyro so the near field is still covered when cones return.
            self._trusted_road = None
            self._failure_since_s = None
        else:
            self._clear_road_history()
        lidar = observations.get("lidar_cartesian")
        lidar_stamp = stamps.get("lidar_cartesian")
        available = self._fresh(lidar, ages.get("lidar_cartesian")) and lidar_stamp is not None
        source_time = lidar_stamp/1e9 if available else None
        pose, problem = (self.motion.pose_between(source_time, now_s) if available
                         else (None, "unavailable"))
        available = available and state["valid"] and problem is None
        points = []
        if available:
            for point in lidar["points_xyz"]:
                x, y = self.motion.transform_xy(point, pose)
                if not all(finite_number(v) for v in (x, y, point[2])):
                    available, points = False, []
                    break
                points.append((x, y, point[2]))
        obstacle = {"timestamp_s": now_s if available else None,
                    "measurement_timestamp_s": source_time,
                    "frame_id": self.frame_id, "available": available,
                    "source_age_s": ages.get("lidar_cartesian"),
                    "points_xyz_m": points,
                    "scan_indices": list(lidar["scan_indices"]) if available else [],
                    "time_aligned": available,
                    "motion_compensated": available and pose["interval_s"] > 0,
                    "motion_compensation": pose if available else None,
                    "deskewed": False}
        # Physical calibration is not present in the project. Expose missing
        # values rather than copy simulation parameters into real-car limits.
        params = {"valid": False, "source": "unmeasured", "effective_wheelbase_m": None,
                  "steering_offset": None, "steering_map": None, "steering_limit": None,
                  "drive_response": None, "actuation_delay_s": None}
        limits = {"valid": False, "source": "unmeasured", "footprint_xy_m": None,
                  "wheelbase_m": None, "rear_axle_x_m": None, "curvature_limit_1pm": None,
                  "speed_max_mps": None, "acceleration_max_mps2": None,
                  "braking_deceleration_mps2": None, "safety_margin_m": None}
        alignment = {"valid": state["valid"] and road["valid"] and road["time_aligned"],
                     "timestamp_s": now_s,
                     "method": "forward_planar_filtered_odometry",
                     "wheel_time_basis": "ros_receipt_proxy",
                     "wheel_acquisition_time_known": False,
                     "assumptions": ["forward_motion", "planar_no_slip", "causal_filtered_hold"]}
        return {"state": state, "road": road, "obstacles": obstacle, "alignment": alignment,
                "vehicle_params": params, "vehicle_limits": limits}
