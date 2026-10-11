"""Constrained direct shooting against the exact pinned native plant, then env replay."""
from dataclasses import asdict, dataclass
from collections import deque
import math
import time

import numpy as np

from offline.planner_sim.snapshot import clone_runtime
from dreamgym.envs.obstacle_geometry import rectangle_polygon, closed_rectangle_intersection_mask

from .objective import CStarTrackingSettings, sample_reference, reference_plan, path_completion, tracking_loss
from .roads import road_length


@dataclass(frozen=True)
class SolverSettings:
    population: int = 12
    iterations: int = 4
    max_evaluations: int = 64
    max_wall_s: float = 10.0
    drive_sigma: float = 0.025
    steering_sigma: float = 0.08
    tolerance: float = 1e-5
    safety_margin_m: float = 0.02
    speed_max_mps: float = 0.5
    acceleration_max_mps2: float = 1.0
    lateral_acceleration_max_mps2: float = 1.0

    def __post_init__(self):
        for name in ("population", "iterations", "max_evaluations"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < (4 if name != "iterations" else 1):
                raise ValueError(f"Invalid solver setting: {name}")
        for name in ("max_wall_s", "drive_sigma", "steering_sigma", "tolerance", "speed_max_mps",
                     "acceleration_max_mps2", "lateral_acceleration_max_mps2"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"Invalid solver setting: {name}")
        if not math.isfinite(self.safety_margin_m) or self.safety_margin_m < 0:
            raise ValueError("Invalid safety margin")


def frozen_state(raw, anchor, car):
    """CG position/direction/speed in the decision frame; preserve a defined rest bearing."""
    x, y, yaw, vx, vy, _, delta = raw
    dx, dy = x-anchor[0], y-anchor[1]
    cosine, sine = math.cos(anchor[2]), math.sin(anchor[2])
    speed = math.hypot(vx, vy)
    if speed > 1e-8:
        beta = math.atan2(vy, vx)
    else:
        rear = car._rear_axle_distance_from_center_of_mass_m
        length = rear+car._front_axle_distance_from_center_of_mass_m
        beta = math.atan(rear/length*math.tan(delta))
    heading = yaw+beta-anchor[2]
    return [cosine*dx+sine*dy, -sine*dx+cosine*dy,
            math.atan2(math.sin(heading), math.cos(heading)), speed]


class NativePrediction:
    """Clone all car history; integrate without costly sensor generation during optimization."""
    def __init__(self, environment, settings=None):
        self.settings = settings or SolverSettings()
        self.backend = environment._backend
        if self.backend._lane_keeping_enabled or self.backend._cruise_control_enabled:
            raise ValueError("C* requires full caller-controlled native actions")
        self.initial_car = clone_runtime(self.backend.car)
        self.anchor = self.initial_car._state_values()[:3]
        self.dt_s = environment.dt_s
        self.integration = dict(duration_s=self.dt_s, method=self.backend.integration_method,
                                substep_count=self.backend.effective_integration_substep_count,
                                update_stored_state=True, **self.backend._direction_latch_integration_kwargs)
        self.output_indices = tuple(range(1, self.integration["substep_count"]+1))
        self.polygons = self.backend._obstacle_polygons_m[self.backend._obstacle_active_mask].copy()
        self.road = self.backend.road
        self.road_length_m = road_length(environment.config.road_spec)

    def margins(self, raw, previous=None, dt_s=None):
        cfg, car = self.settings, self.initial_car
        x, y, heading, vx, vy, yaw_rate, steering = raw
        polygon = rectangle_polygon(np.asarray([x, y]), heading_rad=heading,
            forward_extent_m=car._body_front_extent_from_center_of_mass_m+cfg.safety_margin_m,
            rear_extent_m=car._body_rear_extent_from_center_of_mass_m+cfg.safety_margin_m,
            width_m=car._body_width_m+2*cfg.safety_margin_m)
        clearance = longitudinal = float("inf")
        for px, py in polygon:
            cx, cy, _, _, progress, tangent, _ = self.road.find_closest_point_to(float(px), float(py))
            bounds = self.road.get_reference_lane_bounds_at_progress(progress)
            lateral = -(px-cx)*math.sin(tangent)+(py-cy)*math.cos(tangent)
            clearance = min(clearance, lateral-bounds["right_bound_m"], bounds["left_bound_m"]-lateral)
            # find_closest_point_to clamps to a finite endpoint. Lateral bounds
            # alone would admit the infinite start/end tangent extensions.
            along = (px-cx)*math.cos(tangent)+(py-cy)*math.sin(tangent)
            distance = (along if progress <= 1e-7 else -along if progress >= self.road_length_m-1e-7
                        else min(progress,self.road_length_m-progress))
            longitudinal = min(longitudinal,distance)
        contact = bool(closed_rectangle_intersection_mask(polygon, self.polygons).any())
        margins = {"road_clearance_m": float(clearance), "obstacle_contact_margin": -1.0 if contact else 1.0,
                   "road_longitudinal_clearance_m": float(longitudinal),
                   "speed_margin_mps": cfg.speed_max_mps-math.hypot(vx, vy),
                   "forward_velocity_margin_mps": vx+1e-7,
                   "steering_margin_rad": car._requested_front_wheel_angle_magnitude_limit_rad-abs(steering)}
        if previous is not None:
            dt_s = dt_s or self.dt_s
            long_acceleration = (vx-previous[3])/dt_s-yaw_rate*vy
            lateral_acceleration = (vy-previous[4])/dt_s+yaw_rate*vx
            rate = (steering-previous[6])/dt_s
            rate_bounds = car._physical_front_wheel_angle_rate_bounds_rad_per_s
            margins.update(longitudinal_acceleration_margin_mps2=cfg.acceleration_max_mps2-abs(long_acceleration),
                           lateral_acceleration_margin_mps2=cfg.lateral_acceleration_max_mps2-abs(lateral_acceleration),
                           steering_rate_margin_radps=min(rate-rate_bounds[0], rate_bounds[1]-rate))
        return margins

    def predict(self, controls):
        car = clone_runtime(self.initial_car)
        raw = car._state_values()
        states = [frozen_state(raw, self.anchor, car)]
        minimum = self.margins(raw)
        for control in controls:
            # PlannerSimulationEnv uses float32 input before the native double plant.
            action = np.asarray(control, dtype=np.float32)
            car.set_action_request(float(action[0]), float(action[1])*car._requested_front_wheel_angle_magnitude_limit_rad)
            output = np.empty((len(self.output_indices), 7))
            car.integrate(**self.integration, state_output_substep_indices=self.output_indices, state_output=output)
            for substate in output:
                values = self.margins(substate, raw, self.dt_s/len(output))
                for name, value in values.items():
                    minimum[name] = min(minimum.get(name, float("inf")), value)
                raw = substate.copy()
            states.append(frozen_state(car._state_values(), self.anchor, car))
        return np.asarray(states), minimum


class CStarSolver:
    """Full 2H-variable CEM search with feasible elites, multi-start and bounded budget."""
    def __init__(self, objective=None, settings=None):
        self.objective = objective or CStarTrackingSettings()
        self.settings = settings or SolverSettings()

    def warm_start(self, prediction, target):
        car, dt = prediction.initial_car, self.objective.dt_s
        rear = car._rear_axle_distance_from_center_of_mass_m
        length = rear+car._front_axle_distance_from_center_of_mass_m
        distance = np.maximum((target[1:, 3]+target[:-1, 3])*dt/2, 1e-5)
        curvature = np.diff(np.unwrap(target[:, 2]))/distance
        beta = np.arcsin(np.clip(rear*curvature, -0.95, 0.95))
        delta = np.arctan(length*curvature/np.cos(beta))
        body_speed = target[1:, 3]*np.cos(beta)
        acceleration = np.diff(np.r_[car._vx, body_speed])/dt
        force = (car._mass_kg*acceleration+
                 car._quadratic_aerodynamic_drag_coefficient_kg_per_m*body_speed*np.abs(body_speed))
        gain = car._normalized_motor_command_force_gain_n
        drive = force/max(gain, 1e-9)
        return np.clip(np.column_stack((drive, delta/car._requested_front_wheel_angle_magnitude_limit_rad)), -1, 1)

    def transient_warm_start(self,prediction,target):
        """Native-plant inverse-heading initializer, including initial steering slip.

        This is only one search seed. Every seed still receives the complete
        constrained optimization/actual replay; it never bypasses C* evaluation.
        """
        car=clone_runtime(prediction.initial_car)
        rear=car._rear_axle_distance_from_center_of_mass_m
        wheelbase=rear+car._front_axle_distance_from_center_of_mass_m
        actions=[]
        for desired in target[1:]:
            raw=car._state_values()
            error=prediction.anchor[2]+desired[2]-raw[2]
            beta=float(np.clip(math.atan2(math.sin(error),math.cos(error)),-.5,.5))
            steering=math.atan(wheelbase/rear*math.tan(beta))
            velocity=desired[3]*math.cos(beta)
            acceleration=(velocity-raw[3])/self.objective.dt_s
            force=car._mass_kg*acceleration+car._quadratic_aerodynamic_drag_coefficient_kg_per_m*raw[3]*abs(raw[3])
            action=np.asarray(np.clip([force/car._normalized_motor_command_force_gain_n,
                steering/car._requested_front_wheel_angle_magnitude_limit_rad],-1,1),dtype=np.float32)
            car.set_action_request(float(action[0]),float(action[1])*car._requested_front_wheel_angle_magnitude_limit_rad)
            car.integrate(**prediction.integration)
            actions.append(action)
        return np.asarray(actions,dtype=float)

    def solve(self, environment, candidate, previous_action=(0.0, 0.0), *, seed=0):
        if not math.isclose(environment.dt_s, self.objective.dt_s, rel_tol=0, abs_tol=1e-10):
            raise ValueError("Objective dt must match the existing env integration step")
        plan = reference_plan(candidate,self.objective)
        target = plan["states"]
        steps = len(target)-1
        prediction = NativePrediction(environment, self.settings)
        cfg, started = self.settings, time.perf_counter()
        initial_margins=prediction.margins(prediction.initial_car._state_values())
        if min(initial_margins.values()) < -1e-7:
            return {'status':'failed','reason':'illegal_initial_state','margins':initial_margins,
                'diagnostics':{'method':'constrained_multistart_CEM_direct_shooting','seed':int(seed),
                    'variables':2*steps,'evaluations':0,'wall_s':time.perf_counter()-started,
                    'stop_reason':'illegal_initial_state','global_optimum_proven':False,
                    'solver_settings':asdict(cfg),'full_path_duration_s':steps*self.objective.dt_s,
                    'steps':steps,'spatial_span_m':plan['spatial_span_m']}}
        rng = np.random.default_rng(seed)
        best, evaluations, stop_reason = None, 0, "iteration_limit"
        ranked = []

        def evaluate(actions):
            nonlocal best, evaluations
            if evaluations >= cfg.max_evaluations or time.perf_counter()-started >= cfg.max_wall_s:
                return None
            actions = np.asarray(np.clip(actions, -1, 1), dtype=np.float32).astype(np.float64)
            evaluations += 1
            try:
                states, margins = prediction.predict(actions)
                violation = sum(max(0.0, -value) for value in margins.values())
                completion = path_completion(candidate,states,self.objective,plan=plan)
                violation += max(0.0,completion["endpoint_error_m"]-self.objective.completion_tolerance_m)
                if completion["planned_stop"]:
                    violation += max(0.0,completion["final_speed_mps"]-self.objective.stop_speed_tolerance_mps)
                loss, terms = tracking_loss(target, states, actions, previous_action, self.objective)
                item = {"controls": actions.copy(), "states": states, "loss": loss,
                        "loss_terms": terms, "margins": margins, "violation": violation, "completion": completion}
                if violation <= 1e-7 and (best is None or loss < best["loss"]):
                    best = item
                return item
            except (ValueError, ArithmeticError, FloatingPointError):
                return None

        curvature_warm = self.warm_start(prediction, target)
        warm = self.transient_warm_start(prediction,target)
        brake = np.zeros_like(warm)
        brake[:, 0] = -0.03
        for actions in (warm, curvature_warm, warm*0.8, np.zeros_like(warm), brake):
            item = evaluate(actions)
            if item is not None:
                ranked.append(item)
        mean = (best["controls"] if best is not None else warm).copy()
        sigma = np.tile([cfg.drive_sigma, cfg.steering_sigma], (steps, 1))
        last_loss, stable = float("inf"), 0
        for iteration in range(cfg.iterations):
            batch = list(ranked)
            for _ in range(cfg.population):
                # Correlated exploration plus independent noise gives all 2H inputs freedom.
                knots = rng.normal(size=(min(8, steps), 2))
                band = np.column_stack([np.interp(np.linspace(0, 1, steps),
                    np.linspace(0, 1, len(knots)), knots[:, axis]) for axis in range(2)])
                noise = 0.8*band+0.2*rng.normal(size=mean.shape)
                item = evaluate(mean+sigma*noise)
                if item is not None:
                    batch.append(item)
            if not batch:
                break
            batch.sort(key=lambda item: (item["violation"] > 1e-7, item["violation"], item["loss"]))
            feasible = [item for item in batch if item["violation"] <= 1e-7]
            elite = (feasible or batch)[:max(2, cfg.population//4)]
            samples = np.stack([item["controls"] for item in elite])
            mean = 0.4*mean+0.6*samples.mean(axis=0)
            sigma = np.maximum(0.65*sigma+0.35*samples.std(axis=0), np.array([0.001, 0.003]))
            ranked = batch[:2]
            if best is not None:
                stable = stable+1 if abs(last_loss-best["loss"]) < cfg.tolerance else 0
                last_loss = best["loss"]
            if stable >= 2:
                stop_reason = "loss_stagnation"
                break
            if evaluations >= cfg.max_evaluations or time.perf_counter()-started >= cfg.max_wall_s:
                stop_reason = "evaluation_budget" if evaluations >= cfg.max_evaluations else "wall_budget"
                break
        if time.perf_counter()-started >= cfg.max_wall_s:
            stop_reason = "wall_budget"
        elif evaluations >= cfg.max_evaluations:
            stop_reason = "evaluation_budget"
        diagnostics = {"method": "constrained_multistart_CEM_direct_shooting", "seed": int(seed), "variables": 2*self.objective.steps,
                       "evaluations": evaluations, "wall_s": time.perf_counter()-started, "stop_reason": stop_reason,
                       "global_optimum_proven": False, "solver_settings": asdict(cfg),
                       "full_path_duration_s": steps*self.objective.dt_s, "steps": steps,
                       "spatial_span_m": plan["spatial_span_m"]}
        diagnostics["variables"] = 2*steps
        if best is None:
            return {"status": "failed", "reason": "no_feasible_solution_found", "diagnostics": diagnostics}
        return {"status": "approximate", "reason": stop_reason, **best, "diagnostics": diagnostics}


def performance_profile(seed, *, nominal=False):
    if nominal:
        return {"seed": int(seed), "nominal": True, "drive_gain": 1.0, "steering_gain": 1.0, "steering_bias": 0.0, "delay_steps": 0}
    rng = np.random.default_rng(seed)
    return {"seed": int(seed), "nominal": False, "drive_gain": float(rng.uniform(0.9, 1.1)), "steering_gain": float(rng.uniform(0.9, 1.1)),
            "steering_bias": float(rng.uniform(-0.005, 0.005)), "delay_steps": int(rng.integers(0, 2))}


def apply_performance(controls, previous_action, profile):
    actuator = PerformanceActuator(profile, previous_action)
    return np.stack([actuator.apply(action) for action in controls])


class PerformanceActuator:
    """Delay requested commands using persistent history, then apply gain/bias."""
    def __init__(self, profile, initial_control):
        self.profile = dict(profile)
        self.pending = deque(np.asarray(initial_control, dtype=float).copy()
                             for _ in range(profile["delay_steps"]))

    def apply(self, requested):
        self.pending.append(np.asarray(requested, dtype=float).copy())
        delayed = self.pending.popleft()
        action = [delayed[0]*self.profile["drive_gain"],
                  delayed[1]*self.profile["steering_gain"]+self.profile["steering_bias"]]
        return np.asarray(np.clip(action, -1, 1), dtype=np.float32)


def execute_trial(environment, candidate, solution, previous_action, objective, safety, *, trial_id, profile):
    """Replay every solve in the full env; labels use actual, not predicted, motion."""
    checker = NativePrediction(environment, safety)
    initial = frozen_state(checker.initial_car._state_values(), checker.anchor, checker.initial_car)
    trial = {"trial_id": trial_id, "initial_state": initial, "initial_control": list(previous_action),
             "solver_status": "failed", "constraints_satisfied": False, "performance": profile,
             "solver_diagnostics": solution["diagnostics"], "nominal_predicted_loss": solution.get("loss"),
             "nominal_constraint_margins": solution.get("margins")}
    if solution["status"] != "approximate":
        trial["reason"] = solution["reason"]
        return trial
    actions = apply_performance(solution["controls"], previous_action, profile)
    times, target = sample_reference(candidate, objective)
    steps = len(target)-1
    states, margins, reason, observations = [initial], checker.margins(checker.initial_car._state_values()), None, []
    with environment.fork() as branch:
        raw = branch._backend.car._state_values()
        for action in actions:
            observation, _, terminated, truncated, info = branch.step(action)
            following = branch._backend.car._state_values()
            current = checker.margins(following, raw)
            for name, value in current.items():
                margins[name] = min(margins.get(name, float("inf")), value)
            states.append(frozen_state(following, checker.anchor, branch._backend.car))
            observations.append({"timestamp_s": observation["timestamp_s"],
                                 "sensor_available": observation["sensor_available"]})
            raw = following
            if info["evaluation"]["collision"] or min(margins.values()) < -1e-7:
                reason = "execution_constraint_violation"
                break
            if terminated or truncated:
                reason = "execution_horizon_terminated"
                break
    trial.update(states=states, controls=actions[:len(states)-1].tolist(), times_s=times[:len(states)].tolist(),
                 constraint_margins=margins, observation_diagnostics=observations)
    if reason is None and len(states) == steps+1:
        completion = path_completion(candidate,states,objective)
        trial["completion"] = completion
        if not completion["reached"]:
            reason = "full_path_endpoint_not_reached"
    if reason is None and len(states) == steps+1:
        loss, terms = tracking_loss(target, states, actions, previous_action, objective)
        nominal = profile.get("nominal", False)
        difference = float(np.max(np.abs(np.asarray(states)-solution["states"]))) if nominal else None
        if nominal and difference > 1e-7:
            reason = "native_prediction_env_replay_mismatch"
        else:
            trial.update(solver_status="approximate", constraints_satisfied=True,
                         actual_loss=loss, actual_loss_terms=terms, replay_max_state_error=difference)
    if reason:
        trial["reason"] = reason
    return trial
