"""Shared tracking objective for the internal C* solver and policy labels."""
from dataclasses import asdict, dataclass
import math

import numpy as np

from .source import candidate_api

CSTAR_OBJECTIVE = "cstar_full_path_loss_v2"
CSTAR_TARGET = "negative_expected_full_path_cost"


@dataclass(frozen=True)
class CStarTrackingSettings:
    horizon_s: float = 3.0
    dt_s: float = 0.05
    discount: float = 1.0
    position_weight: float = 1.0
    heading_weight: float = 0.2
    speed_weight: float = 0.2
    drive_rate_weight: float = 0.01
    steering_rate_weight: float = 0.05
    position_scale_m: float = 0.1
    heading_scale_rad: float = 0.1
    speed_scale_mps: float = 0.1
    drive_rate_scale_per_s: float = 1.0
    steering_rate_scale_per_s: float = 1.0
    max_horizon_s: float = 60.0
    speed_reward_weight: float = 0.5
    speed_reward_scale_mps: float = 0.2
    terminal_position_weight: float = 2.0
    terminal_speed_weight: float = 0.2
    completion_tolerance_m: float = 0.15
    stop_speed_tolerance_mps: float = 0.02
    stop_settle_s: float = 0.5
    failure_loss: float = 1e6

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            if value < 0 or (name not in {"heading_weight", "speed_weight", "drive_rate_weight",
                                         "steering_rate_weight", "speed_reward_weight", "terminal_position_weight",
                                         "terminal_speed_weight", "stop_settle_s"} and value == 0):
                raise ValueError(f"Invalid C* objective setting: {name}")
        if not 0 < self.discount <= 1 or not 1 <= self.horizon_s/self.dt_s <= 10000:
            raise ValueError("Invalid C* discount or horizon/sample count")
        if self.max_horizon_s < self.horizon_s or self.max_horizon_s/self.dt_s > 10000:
            raise ValueError("Invalid full-path maximum horizon")
        if not math.isclose(self.horizon_s/self.dt_s, round(self.horizon_s/self.dt_s), abs_tol=1e-8):
            raise ValueError("C* horizon must be an integer multiple of dt_s")

    @property
    def steps(self):
        return round(self.horizon_s/self.dt_s)

    def specification(self):
        return {"id": CSTAR_OBJECTIVE, "direction": "maximize", "discount": self.discount,
                "horizon_s": self.horizon_s,
                "definition": {"type": CSTAR_OBJECTIVE, "settings": asdict(self),
                               "target": CSTAR_TARGET,
                               "state_columns": ["x_m", "y_m", "motion_heading_rad", "speed_mps"],
                               "control_columns": ["drive_normalized", "steer_normalized"],
                               "frame": "decision_base_link", "sampling": "interval_end_states",
                               "reference": "full_frenet_path_original_timing_then_terminal_speed_extension",
                               "horizon_policy": "complete_spatial_path_or_planned_stop",
                               "normalization": "time_average_plus_terminal_cost",
                               "speed_utility": "mean_positive_velocity_along_reference_on_moving_stages",
                               "horizon_s_meaning": "minimum_sampling_duration_not_truncation",
                               "constraints": "native_dynamics_input_and_safety_hard_constraints",
                               "zero_speed_heading": "body_heading_plus_kinematic_steering_direction",
                               "failure_encoding": "declared_failure_loss_not_a_feasible_solution"}}

    @classmethod
    def from_specification(cls, objective):
        try:
            settings = cls(**objective["definition"]["settings"])
        except (TypeError, KeyError) as error:
            raise ValueError("A complete C* tracking objective specification is required") from error
        if objective != settings.specification():
            raise ValueError("C* target/weights/units/sampling differ from the shared specification")
        return settings


def _array(value, shape, name):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"Invalid {name}") from error
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must have finite shape {shape}")
    return result


def reference_plan(candidate, settings=None):
    """Cover the entire native spatial path, preserving the authored time prefix."""
    cfg = settings or CStarTrackingSettings()
    api = candidate_api()
    reference = candidate["reference"]
    lateral = reference["frenet_path"]["lateral"]
    curve = api.LatticeCurve.from_record(reference["frenet_path"]["reference_curve"])
    longitudinal = reference["longitudinal_profile"]
    duration = float(longitudinal["duration_s"])
    coefficients = np.asarray(longitudinal["coeffs_low_to_high"], dtype=float)
    if (not math.isfinite(duration) or duration <= 0 or coefficients.ndim != 1
            or not 1 <= len(coefficients) <= 6 or not np.isfinite(coefficients).all()
            or longitudinal.get("type") != "S_REF_OF_TIME"):
        raise ValueError("Invalid original longitudinal reference")
    profile = np.asarray(reference["speed_profile"], dtype=float)
    if (profile.ndim != 2 or profile.shape[1] != 2 or len(profile) < 2 or not np.isfinite(profile).all()
            or (np.diff(profile[:,0]) <= 0).any() or profile[0,0] > 1e-8
            or profile[-1,0]+1e-8 < duration or (profile[:,1] < 0).any()):
        raise ValueError("Physical speed profile must cover its authored duration")
    lo, hi = map(float,lateral["range"])
    def geometry(s):
        return api._lattice_geometry(curve,lateral["origin"],lateral["coeffs_low_to_high"],
                                     lateral["scale"],lateral["terminal_offset_m"],float(s))
    end_s = float(api._control_poly_eval(coefficients,1.0))
    if not lo-1e-7 <= end_s <= hi+1e-7:
        raise ValueError("Timed candidate exceeds its admitted spatial support")
    stopping = reference.get("planned_stop") is True or longitudinal.get("stop") is True
    goal_s = end_s if stopping else hi
    terminal_speed = float(profile[-1,1])
    extension_s, extension_times = np.array([end_s]), np.array([0.0])
    if not stopping and goal_s-end_s > 1e-7:
        if terminal_speed <= 0.01:
            raise ValueError("Cruise needs a positive terminal speed to cover the full path")
        extension_s = np.linspace(end_s,goal_s,max(2,int(math.ceil((goal_s-end_s)/.01))+1))
        xy = np.asarray([[geometry(s)["x"],geometry(s)["y"]] for s in extension_s])
        extension_times = np.r_[0.0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))/terminal_speed]
    required = duration+float(extension_times[-1])+(cfg.stop_settle_s if stopping else 0.0)
    horizon = max(cfg.horizon_s,math.ceil(required/cfg.dt_s-1e-8)*cfg.dt_s)
    if horizon > cfg.max_horizon_s+1e-8:
        raise ValueError("Full candidate exceeds max_horizon_s; refusing a truncated score")
    steps = round(horizon/cfg.dt_s)
    times, targets = np.arange(steps+1)*cfg.dt_s, []
    for t in times:
        if t <= duration+1e-10:
            s = api._control_poly_eval(coefficients,min(1.0,float(t)/duration))
        else:
            s = float(np.interp(t-duration,extension_times,extension_s))
        if not lo-1e-7 <= s <= hi+1e-7:
            raise ValueError("Timed candidate exceeds its admitted spatial support")
        g = geometry(min(hi,max(lo,s)))
        speed = float(np.interp(t,profile[:,0],profile[:,1])) if t <= duration else (0.0 if stopping else terminal_speed)
        targets.append([g["x"],g["y"],g["heading"],speed])
    dense = np.linspace(lo,goal_s,max(2,int(math.ceil((goal_s-lo)/.01))+1))
    xy = np.asarray([[geometry(s)["x"],geometry(s)["y"]] for s in dense])
    return {"times_s": times, "states": _array(targets,(steps+1,4),"sampled reference"),
            "steps": steps, "duration_s": horizon, "source_duration_s": duration,
            "source_end_s_m": end_s, "goal_s_m": goal_s, "spatial_span_m": goal_s-lo,
            "physical_length_m": float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
            "planned_stop": stopping, "completion_tolerance_m": cfg.completion_tolerance_m}


def sample_reference(candidate, settings=None):
    plan = reference_plan(candidate,settings)
    return plan["times_s"],plan["states"]


def path_completion(candidate, actual_states, settings=None, *, plan=None):
    cfg = settings or CStarTrackingSettings()
    plan = plan or reference_plan(candidate,cfg)
    actual = np.asarray(actual_states,dtype=float)
    error = float(np.linalg.norm(actual[-1,:2]-plan["states"][-1,:2]))
    speed_ok = not plan["planned_stop"] or actual[-1,3] <= cfg.stop_speed_tolerance_mps
    return {"reached": bool(error <= cfg.completion_tolerance_m and speed_ok),
            "endpoint_error_m": error, "final_speed_mps": float(actual[-1,3]),
            "spatial_span_m": plan["spatial_span_m"], "duration_s": plan["duration_s"],
            "planned_stop": plan["planned_stop"]}


def tracking_loss(reference_states, actual_states, controls, initial_control, settings=None):
    """One stage objective for C* search, actual execution labels and evaluation."""
    cfg = settings or CStarTrackingSettings()
    shape = np.asarray(reference_states).shape
    if len(shape) != 2 or shape[1] != 4 or not 2 <= shape[0] <= 10001:
        raise ValueError("Reference states must have finite shape (N+1,4)")
    steps = shape[0]-1
    target = _array(reference_states, shape, "reference states")
    actual = _array(actual_states, target.shape, "actual states")
    if (actual[:, 3] < -1e-8).any() or (target[:, 3] < -1e-8).any():
        raise ValueError("CG trajectory speed must be a nonnegative magnitude")
    actions = _array(controls, (steps, 2), "applied controls")
    previous = _array(initial_control, (2,), "initial applied control")
    if (np.abs(actions) > 1+1e-8).any() or (np.abs(previous) > 1+1e-8).any():
        raise ValueError("C* controls must be normalized to [-1,1]")
    error = actual[1:]-target[1:]
    heading_error = np.arctan2(np.sin(error[:, 2]), np.cos(error[:, 2]))
    rate = np.diff(np.vstack((previous, actions)), axis=0)/cfg.dt_s
    stage = {
        "position": cfg.position_weight*np.sum(error[:, :2]**2, axis=1)/cfg.position_scale_m**2,
        "heading": cfg.heading_weight*(heading_error/cfg.heading_scale_rad)**2,
        "speed": cfg.speed_weight*(error[:, 3]/cfg.speed_scale_mps)**2,
        "drive_rate": cfg.drive_rate_weight*(rate[:, 0]/cfg.drive_rate_scale_per_s)**2,
        "steering_rate": cfg.steering_rate_weight*(rate[:, 1]/cfg.steering_rate_scale_per_s)**2}
    moving = target[1:,3] > 0.01
    stage["speed_utility"] = -cfg.speed_reward_weight*np.where(moving,
        np.maximum(0.0,actual[1:,3]*np.cos(heading_error))/cfg.speed_reward_scale_mps,0.0)
    weights = cfg.discount**np.arange(steps)/steps
    terms = {name: float(np.dot(weights, value)) for name, value in stage.items()}
    terms["terminal_position"] = cfg.terminal_position_weight*float(np.sum(error[-1,:2]**2))/cfg.position_scale_m**2
    terms["terminal_speed"] = cfg.terminal_speed_weight*float(error[-1,3]**2)/cfg.speed_scale_mps**2
    value = float(sum(terms.values()))
    if not math.isfinite(value) or value >= cfg.failure_loss:
        raise ValueError("Feasible tracking loss must be finite and below declared failure_loss")
    return value, terms


def cstar_trial_returns(candidate, trials, objective):
    """Recompute labels from the producer's actual C* executions and diagnostics."""
    cfg = CStarTrackingSettings.from_specification(objective)
    times, target = sample_reference(candidate, cfg)
    if not isinstance(trials, list) or not trials:
        raise ValueError("C* labels require nonempty cstar_trials with execution traces")
    returns, diagnostics, initial_states, identities = [], [], {}, set()
    for trial in trials:
        if not isinstance(trial, dict):
            raise ValueError("Each C* trial must be an execution record")
        identity = trial.get("trial_id")
        if not isinstance(identity, str) or not identity or identity in identities:
            raise ValueError("C* trial IDs must be nonempty and unique within a candidate")
        identities.add(identity)
        status = trial.get("solver_status")
        if status not in {"optimal", "approximate", "infeasible", "failed"}:
            raise ValueError("C* trial must declare its solver status")
        previous = _array(trial.get("initial_control"), (2,), "initial control")
        initial = _array(trial.get("initial_state"), (4,), "initial state")
        if not np.allclose(initial[:2], 0.0, atol=1e-7):
            raise ValueError("C* traces must start at the CG origin of the frozen decision frame")
        initial_states[identity] = (initial, previous)
        if status in {"infeasible", "failed"}:
            value, terms = cfg.failure_loss, {"failure": cfg.failure_loss}
        else:
            if trial.get("constraints_satisfied") is not True:
                raise ValueError("Successful C* executions must declare dynamics/input/safety constraints satisfied")
            supplied_times = _array(trial.get("times_s"), times.shape, "trial times")
            if not np.allclose(supplied_times, times, atol=1e-8, rtol=0):
                raise ValueError("C* executions must cover the entire shared sampling horizon")
            actual = _array(trial.get("states"), target.shape, "actual states")
            if not np.allclose(actual[0], initial, atol=1e-8, rtol=0):
                raise ValueError("C* initial state differs from its execution trace")
            value, terms = tracking_loss(target, actual, trial.get("controls"), previous, cfg)
            if not path_completion(candidate,actual,cfg)["reached"]:
                raise ValueError("Successful C* trial did not reach the full planned path endpoint")
        returns.append(-value)
        diagnostics.append({"trial_id": identity, "solver_status": status, "loss": value, "loss_terms": terms})
    return {"returns": returns, "cstar_losses": diagnostics}, initial_states
