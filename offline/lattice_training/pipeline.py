"""One env -> native lattice -> C* -> execution labels -> policy -> evaluation pipeline."""
from copy import deepcopy
from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path
import time

import numpy as np

from offline.planner_sim import PlannerSimulationEnv, SensorTiming, load_notebook

from .cstar import CStarSolver, NativePrediction, PerformanceActuator, SolverSettings, execute_trial, frozen_state, performance_profile
from .dataset import (DECISION_FORMAT, INTERNAL_LABEL_VERSION, import_dataset, json_safe, objective_hash,
                      write_json, load_dataset, append_training_dataset, label_source_contract)
from .features import encode_candidate, scene_features
from .model import evaluate_policy, train_model
from .objective import CStarTrackingSettings, cstar_trial_returns, sample_reference, reference_plan, path_completion, tracking_loss
from .plots import plot_training, plot_closed_loop, plot_road_coverage
from .source import planner_source_sha256
from .roads import RoadRandomizationSettings, generate_random_road_spec, road_identity, road_length, randomization_provenance, obstacle_passage_report


@dataclass(frozen=True)
class PipelineSettings:
    kinds: tuple = ("randomized",)
    train_seeds: tuple = tuple(range(8))
    validation_seeds: tuple = (100, 101)
    test_seeds: tuple = (200, 201)
    decisions: int = 3
    until_road_end: bool = False
    max_segments: int = 100
    starts_per_road: int = 2
    planner_profiles: tuple = ("standard", "conservative")
    segment_resampling_attempts: int = 6
    trials: int = 2
    evaluation_steps: int = 60
    train_only: bool = False

    def __post_init__(self):
        known = {"randomized", "straight", "left_curve", "right_curve", "obstacle", "blocked"}
        if not self.kinds or len(set(self.kinds)) != len(self.kinds) or not set(self.kinds) <= known:
            raise ValueError("Unknown or duplicate training scene kinds")
        if (not self.planner_profiles or len(set(self.planner_profiles)) != len(self.planner_profiles)
                or not set(self.planner_profiles) <= {"standard", "conservative", "wide"}):
            raise ValueError("Invalid lattice profiles")
        sets = []
        if not isinstance(self.train_only, bool):
            raise ValueError('train_only must be boolean')
        if self.train_only and (self.validation_seeds or self.test_seeds):
            raise ValueError('Train-only collection must have empty validation/test seed lists')
        for index, seeds in enumerate((self.train_seeds, self.validation_seeds, self.test_seeds)):
            if ((not seeds and (not self.train_only or index == 0)) or len(set(seeds)) != len(seeds)
                    or any(isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in seeds)):
                raise ValueError("Each split requires distinct nonnegative seeds")
            sets.append(set(seeds))
        if any(a & b for i, a in enumerate(sets) for b in sets[i+1:]):
            raise ValueError("Scene seeds must not overlap between train/validation/test")
        if not isinstance(self.until_road_end, bool):
            raise ValueError("until_road_end must be boolean")
        for name in ("decisions", "max_segments", "trials", "evaluation_steps", "starts_per_road", "segment_resampling_attempts"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"Invalid pipeline setting: {name}")


def initial_progress_intervals(spec, body, *, low, high, margin_m=.02):
    """Physical free-space support, independent of sensors, planner and rewards.

    Reserve obstacle cross-sections conservatively for all sampled lateral
    offsets/headings. This constructs the distribution before the single draw;
    it never redraws a state because a planner or perception check rejected it.
    """
    if not math.isfinite(margin_m) or margin_m < 0:
        raise ValueError("Invalid initialization safety margin")
    front = body['front_extent_from_center_of_mass_m']+margin_m
    rear = body['rear_extent_from_center_of_mass_m']+margin_m
    intervals = [(float(low),float(high))] if high>low else []
    for obstacle in spec.get('obstacles',[]):
        station=obstacle['placement']['reference_line_progress_m']
        half=obstacle['geometry']['length_m']/2
        # .10m covers the bounded heading offset and curved-road projection.
        forbidden=(station-half-front-.10,station+half+rear+.10)
        remaining=[]
        for left,right in intervals:
            if forbidden[1]<=left or forbidden[0]>=right:
                remaining.append((left,right))
            else:
                if left<forbidden[0]:remaining.append((left,min(right,forbidden[0])))
                if forbidden[1]<right:remaining.append((max(left,forbidden[1]),right))
        intervals=remaining
    if not intervals:
        raise ValueError("Initial progress stratum contains no physical free-space interval")
    return intervals


def draw_progress(rng, intervals):
    lengths=[right-left for left,right in intervals]
    coordinate=float(rng.uniform(0,sum(lengths)))
    for (left,right),length in zip(intervals,lengths):
        if coordinate<length:return left+coordinate
        coordinate-=length
    return intervals[-1][1]


def make_environment(kind, seed, *, max_steps, initial_index=0, initial_count=1,
                     initialization_attempt=0, road_settings=None, initial_safety_margin_m=.02):
    """Vary scene/initial conditions; preserve notebook vehicle and RK4 parameters."""
    if kind not in {"randomized", "straight", "left_curve", "right_curve", "obstacle", "blocked"}:
        raise ValueError("Unknown scene kind")
    if (not 0 <= initial_index < initial_count or initialization_attempt < 0
            or not math.isfinite(initial_safety_margin_m) or initial_safety_margin_m < 0):
        raise ValueError("Invalid initial state index/count/attempt")
    config = load_notebook(with_obstacles=True)
    rng = np.random.default_rng(seed)
    element = deepcopy(config.road_spec["elements"][0])
    element["geometry"] = {"type": "straight", "length_m": 10.0}
    width = float(rng.uniform(1.0, 1.15))
    element["lanes"]["reference_lane"]["width_m"].update(start=width, end=width)
    element["active_cone_paths"] = {"left_boundary": "regular", "right_boundary": "regular"}
    if kind in {"left_curve", "right_curve"}:
        sign = 1 if kind == "left_curve" else -1
        element["geometry"] = {"type": "circular_arc", "curvature_per_m": sign/float(rng.uniform(5.0, 7.0)),
                               "sweep_angle_deg": 80.0}
    config.road_spec["elements"] = [element]
    config.road_spec["obstacles"] = []
    if kind in {"obstacle", "blocked"}:
        obstacle = deepcopy(load_notebook().road_spec["obstacles"][1])
        obstacle["geometry"].update(length_m=0.25, width_m=0.28 if kind == "obstacle" else 1.2)
        # With a legal rear footprint, the old 1.4m example left less than the
        # native minimum .6m STOP transition after its obstacle/body guards.
        obstacle["placement"].update(reference_line_progress_m=1.6 if kind == "obstacle" else 0.9,
                                     lateral_offset_m=0.17 if kind == "obstacle" else 0.0)
        config.road_spec["obstacles"] = [obstacle]
    body=config.bicycle_model_config['body_geometry']
    # The original .12m CG position placed the .198m rear extent outside road.
    safe_start=(body['rear_extent_from_center_of_mass_m']+initial_safety_margin_m
                +(body['width_m']/2+initial_safety_margin_m)*math.sin(.035)+.01)
    config.initial_state_config["pose"]["progress"].update(lower=safe_start, upper=safe_start)
    offset = float(rng.uniform(-0.1, 0.1))
    heading = float(rng.uniform(-0.01, 0.01))
    speed = float(rng.uniform(0.08, 0.16))
    config.initial_state_config["pose"]["lateral_offset_m"].update(lower=offset, upper=offset)
    config.initial_state_config["pose"]["heading_error_rad"].update(lower=heading, upper=heading)
    config.initial_state_config["body_motion"]["longitudinal_velocity"]["value_mps"].update(lower=speed, upper=speed)
    if kind == "randomized":
        config.road_spec = generate_random_road_spec(seed, road_settings)
        episode_seed = int(np.random.SeedSequence([seed, initial_index, initialization_attempt]).generate_state(1)[0])
        initial_rng = np.random.default_rng(episode_seed)
        maximum = road_length(config.road_spec)-3.0
        if maximum <= safe_start:
            raise ValueError("Random road has insufficient space for the complete initial car body")
        low = safe_start+(maximum-safe_start)*initial_index/initial_count
        high = safe_start+(maximum-safe_start)*(initial_index+1)/initial_count
        intervals=initial_progress_intervals(config.road_spec,body,low=low,high=high,margin_m=initial_safety_margin_m)
        progress = draw_progress(initial_rng,intervals)
        config.initial_state_config["pose"]["progress"].update(lower=progress, upper=progress)
        offset = float(initial_rng.uniform(-0.12, 0.12))
        heading = float(initial_rng.uniform(-0.035, 0.035))
        speed = float(initial_rng.uniform(0.08, 0.25))
        config.initial_state_config["pose"]["lateral_offset_m"].update(lower=offset, upper=offset)
        config.initial_state_config["pose"]["heading_error_rad"].update(lower=heading, upper=heading)
        config.initial_state_config["body_motion"]["longitudinal_velocity"]["value_mps"].update(lower=speed, upper=speed)
    from offline.mvp21_sim.source import parameters
    estimator_settings=parameters()['estimation']
    estimator_settings.update(left_colour=2,right_colour=1)  # Notebook generated cones: blue left.
    return PlannerSimulationEnv(config, max_steps=max_steps, estimation_settings=estimator_settings, sensor_timing=SensorTiming(
        period_steps={"cone_detections": 2}, delay_steps={"cone_detections": 1}))


def make_planner(profile="standard", *, cruise_speed=.2):
    from mvp21_policy.lattice import ShadowSettings
    from mvp21_policy.planning import PlanningSettings
    from .planner import PolicyCandidatePlanner
    variants={"standard":{},"conservative":{"search_half_width_m":.3},"wide":{"sample_count":64,"search_half_width_m":.6}}
    if profile not in variants:raise ValueError("Unknown latest lattice profile")
    return PolicyCandidatePlanner(PlanningSettings(algorithm="road_centerline",allow_forward_only=True,
        max_source_age_s=1.5,reference_lifetime_s=.2,cruise_speed_mps=cruise_speed),
        ShadowSettings(enabled=True,collision_coverage="generated",score_coverage="generated",length_m=1.5,**variants[profile]))



def candidates_at(planner, observation, model=None):
    estimates=observation["estimation"]
    context=scene_features(observation) if model is not None else None
    scorer=(lambda candidate:float(model.predict(encode_candidate(observation,candidate,context=context)[None,:])[0])) if model is not None else None
    return planner.plan(estimates["road"],estimates["state"],estimates["obstacles"],estimates["vehicle_limits"],
        observation["timestamp_s"],source_timeout_s={"road":1.5,"wheel_speed":.25,"imu_angular_velocity":.25},
        collect_candidates=True,candidate_scorer=scorer)



def initialize_episode(kind, seed, initial_index, settings, objective, road_settings, solver_settings=None):
    """Draw the initial state once; perception/planner rejection never redraws it."""
    segments = settings.max_segments if settings.until_road_end else settings.decisions
    limit = (segments+1)*math.ceil(objective.max_horizon_s/objective.dt_s)+10
    environment = make_environment(kind, seed, max_steps=max(limit, settings.evaluation_steps+objective.steps+5),
        initial_index=initial_index, initial_count=settings.starts_per_road, road_settings=road_settings,
        initial_safety_margin_m=(solver_settings or SolverSettings()).safety_margin_m)
    episode_seed = int(np.random.SeedSequence([seed, initial_index, 0]).generate_state(1)[0])
    observation, _ = environment.reset(seed=episode_seed if kind == "randomized" else seed)
    observation, _, terminated, truncated, _ = environment.step((0.0, 0.0))
    return environment, observation, {"episode_seed": episode_seed, "initial_state_draws": 1,
        "warmup_steps": 1, "filtered_by_planner": False, "sampling_domain": "physical_free_space_v2",
        "warmup_terminated":bool(terminated),"warmup_truncated":bool(truncated),
        "initial_region":terminal_region(environment,solver_settings or SolverSettings())}


def road_goal_progress(environment):
    """Same finite-road, body-safe destination for collection and evaluation."""
    return road_length(environment.config.road_spec)-environment._backend.car._body_front_extent_from_center_of_mass_m-.2


def terminal_region(environment, solver_settings):
    """Spatial legality of the complete car body, independent of reward/ranking."""
    raw = np.asarray(environment._backend.car._state_values(), dtype=float)
    if raw.shape != (7,) or not np.isfinite(raw).all():
        return {"legal": False, "reason": "nonfinite_terminal_state"}
    margins = NativePrediction(environment, solver_settings).margins(raw)
    spatial = {name: margins[name] for name in ("road_clearance_m", "obstacle_contact_margin", "road_longitudinal_clearance_m")}
    legal = min(spatial.values()) >= -1e-7
    return {"legal": bool(legal), "reason": None if legal else "terminal_body_outside_legal_region",
            "spatial_margins": spatial}


def simulate_segment(environment, candidate, solution, previous_action, objective, solver_settings, *, performance=None, actuator=None):
    """Execute all C* controls on a fork and preserve the complete terminal env."""
    start = list(environment._backend.car._state_values())
    transition = {"candidate_id": candidate["candidate_id"], "start_state": start,
                  "start_timestamp_s": environment._frame["timestamp_s"],
                  "initial_control": list(previous_action), "completed": False, "steps": 0}
    if solution["status"] != "approximate":
        transition["reason"] = "cstar:"+solution["reason"]
        return None, transition
    actions = np.asarray(solution["controls"], dtype=np.float32)
    plan = reference_plan(candidate,objective)
    if actions.shape != (plan["steps"], 2) or not np.isfinite(actions).all():
        raise ValueError("C* successor controls must cover the complete shared horizon")
    actuator = actuator or PerformanceActuator(performance or performance_profile(0,nominal=True),previous_action)
    checker = NativePrediction(environment,solver_settings)
    if min(checker.margins(start).values()) < -1e-7:
        transition['reason']='illegal_initial_state'
        return None,transition
    states, trace = [frozen_state(start,checker.anchor,checker.initial_car)], []
    with environment.fork() as branch:
        terminated = truncated = False
        for step, action in enumerate(actions):
            action = actuator.apply(action)
            previous_raw=branch._backend.car._state_values()
            _, _, terminated, truncated, info = branch.step(action)
            raw = branch._backend.car._state_values()
            states.append(frozen_state(raw,checker.anchor,branch._backend.car))
            trace.append({"step": step,"timestamp_s": branch._frame["timestamp_s"],"x_m": float(raw[0]),
                "y_m": float(raw[1]),"body_heading_rad": float(raw[2]),"speed_mps": float(states[-1][3]),
                "progress_m": float(info["evaluation"]["ground_truth"]["reference_line_progress_m"]),
                "action": action.tolist(),"constraint_margins": checker.margins(raw,previous_raw)})
            transition["steps"] = step+1
            if terminated or truncated:
                break
        transition.update(end_state=list(branch._backend.car._state_values()),
                          end_timestamp_s=branch._frame["timestamp_s"],
                          final_control=trace[-1]["action"], trace=trace,
                          completed=transition["steps"] == plan["steps"], planned_duration_s=plan["duration_s"],
                          spatial_span_m=plan["spatial_span_m"],
                          environment_terminated=bool(terminated), environment_truncated=bool(truncated))
        if not transition["completed"]:
            transition["reason"] = "cstar_execution_incomplete"
            return None, transition
        region = terminal_region(branch, solver_settings)
        transition["terminal_region"] = region
        if not region["legal"]:
            transition["reason"] = "terminal_region_illegal"
            return None, transition
        if terminated or truncated:
            transition["reason"] = "environment_terminated_at_segment_end"
            return None, transition
        if any(min(row["constraint_margins"].values()) < -1e-7 for row in trace):
            transition["reason"] = "execution_constraint_violation"
            return None, transition
        transition["completion"] = path_completion(candidate,states,objective,plan=plan)
        if not transition["completion"]["reached"]:
            transition["reason"] = "full_path_endpoint_not_reached"
            return None, transition
        transition["actual_loss"], transition["loss_terms"] = tracking_loss(
            plan["states"],states,[row["action"] for row in trace],previous_action,objective)
        transition["reason"] = None
        return branch.snapshot(), transition


def sample_successor(environment, choices, solved, previous_action, objective, solver_settings,
                     chooser, attempts):
    """Uniform sampling without replacement; retry only a completed illegal endpoint."""
    transitions = []
    for index in chooser.permutation(len(choices))[:attempts]:
        chosen = choices[int(index)]
        snapshot, transition = simulate_segment(environment, chosen, solved[chosen["candidate_id"]],
                                                previous_action, objective, solver_settings)
        transitions.append(transition)
        if snapshot is not None:
            return chosen, snapshot, {"attempts": transitions, "accepted": True, "reason": None}
        # Low reward, solver failure, missing perception and incomplete execution
        # must not bias the sampler by triggering a draw of a better candidate.
        if not transition["completed"] or transition["reason"] != "terminal_region_illegal":
            return chosen, None, {"attempts": transitions, "accepted": False, "reason": transition["reason"]}
    return None, None, {"attempts": transitions, "accepted": False, "reason": "illegal_terminal_resampling_exhausted"}


def _collection_label_source(settings, objective, solver_settings, road_settings):
    specification = objective.specification()
    sampling = {"selection": "uniform_without_replacement", "segment_horizon": "complete_candidate_spatial_path",
                "state_transition": "complete_actual_env_terminal_snapshot",
                "resample_only_if": "completed_cstar_terminal_region_illegal",
                "no_policy_or_reward_selection": True, "initial_planner_filter": False,
                "initial_sampling_domain": "physical_free_space_v2"}
    return {"name": "internal_cstar_native_env", "version": INTERNAL_LABEL_VERSION, "execution_assumption": "cstar",
                    "solver_settings": asdict(solver_settings),
                    "objective_sha256": objective_hash(specification),
                    "performance_trials": "one_nominal_plus_seeded_gain_bias_delay_trials",
                    "road_randomization": randomization_provenance(road_settings),
                    "collection_settings": asdict(settings), "sampling": sampling}


def _plant_source(configuration):
    return {"version": configuration["installed_dependency"]["version"],
            "source_sha256": configuration["installed_dependency"]["source_sha256"],
            "notebook_sha256": configuration["provenance"]["notebook_sha256"],
            "bicycle_model_config": configuration["bicycle_model_config"],
            "integration_config": configuration["integration_config"]}


def collect_records(output, settings, objective, solver_settings, progress=None, *, road_settings=None):
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    road_settings = road_settings or RoadRandomizationSettings()
    solver = CStarSolver(objective, solver_settings)
    specification = objective.specification()
    label_source = _collection_label_source(settings, objective, solver_settings, road_settings)
    sampling = label_source['sampling']
    scenes, total_decisions, total_candidates, base_splits = [], 0, 0, {}
    timing_totals = dict.fromkeys(("planning_s", "solve_s", "trials_s", "successor_s", "serialization_s"), 0.0)
    total_trials = successful_trials = 0
    started = time.perf_counter()
    def collection_report(complete):
        return {"decisions": total_decisions, "candidates": total_candidates, "trials": total_trials,
            "successful_trials": successful_trials, "scenes": scenes, "base_roads": len(base_splits),
            "base_road_splits": base_splits, "sampling": sampling, "collection_complete": complete,
            "wall_s": time.perf_counter()-started, "timing_totals": dict(timing_totals),
            "pipeline_settings": asdict(settings), "solver_settings": asdict(solver_settings),
            "objective": specification, "road_randomization": randomization_provenance(road_settings)}
    with path.open("w", encoding="utf-8") as stream:
        for split, seeds in (("train", settings.train_seeds), ("validation", settings.validation_seeds), ("test", settings.test_seeds)):
            for seed in seeds:
                for kind in settings.kinds:
                    for initial_index in range(settings.starts_per_road):
                        for profile in settings.planner_profiles:
                            environment, observation, initialization = initialize_episode(
                                kind, seed, initial_index, settings, objective, road_settings, solver_settings)
                            episode_id = f"{kind}:{seed}:initial:{initial_index}:lattice:{profile}"
                            scene_config = environment.configuration()
                            base_id = road_identity(scene_config["road_spec"])
                            if base_splits.setdefault(base_id, split) != split:
                                environment.close()
                                raise ValueError("Same generated road would leak across data splits")
                            with environment:
                                episode_started = time.perf_counter()
                                planner = make_planner(profile)
                                chooser = np.random.default_rng(initialization["episode_seed"])
                                decisions, faults, previous_action, terminal_behavior = 0, [], (0.0, 0.0), None
                                initial_progress = environment._info["evaluation"]["ground_truth"]["reference_line_progress_m"]
                                initial_pose = list(environment._backend.car._state_values()[:3])
                                goal = road_goal_progress(environment)
                                if progress:
                                    progress({"stage": "collect_episode_start", "episode": episode_id, "split": split,
                                        "initial_progress_m": initial_progress, "goal_progress_m": goal,
                                        "completed_episodes": len(scenes), "total_candidates": total_candidates})
                                segments, planner_fault, scene_candidates = [], None, 0
                                label_source["plant"] = _plant_source(scene_config)
                                segment_limit = settings.max_segments if settings.until_road_end else settings.decisions
                                for segment in range(segment_limit):
                                    if (not initialization['initial_region']['legal'] or initialization['warmup_terminated']
                                            or initialization['warmup_truncated']):
                                        faults.append('illegal_initial_state' if not initialization['initial_region']['legal']
                                                      else 'environment_finished_during_initialization')
                                        terminal_behavior='initial_state_failure'
                                        break
                                    current = float(environment._info["evaluation"]["ground_truth"]["reference_line_progress_m"])
                                    if settings.until_road_end and current >= goal:
                                        terminal_behavior = "road_goal_reached"
                                        break
                                    decision_started = time.perf_counter()
                                    reference, diagnostics = candidates_at(planner, observation)
                                    timings = {"planning_s": time.perf_counter()-decision_started, "solve_s": 0.0, "trials_s": 0.0}
                                    choices = diagnostics.get("candidate_references", [])
                                    if not reference.get("valid") or not choices:
                                        timing_totals['planning_s']+=timings['planning_s']
                                        faults.append(reference.get("reason", "no_candidates"))
                                        terminal_behavior = "planning_failure"
                                        planner_fault={'reason':reference.get('reason'), 'observation':observation,
                                            'diagnostics':diagnostics,'progress_m':current}
                                        break
                                    decision_seed = (initialization["episode_seed"]+segment*100+17) % (2**32)
                                    entries, solved = [], {}
                                    for candidate_index, candidate in enumerate(choices):
                                        solve_started = time.perf_counter()
                                        solution = solver.solve(environment, candidate, previous_action, seed=decision_seed)
                                        timings["solve_s"] += time.perf_counter()-solve_started
                                        trial_started = time.perf_counter()
                                        trials = [execute_trial(environment, candidate, solution, previous_action,
                                            objective, solver_settings, trial_id=f"trial-{trial}",
                                            profile=performance_profile(decision_seed+trial, nominal=trial == 0))
                                            for trial in range(settings.trials)]
                                        computed, _ = cstar_trial_returns(candidate, trials, specification)
                                        timings["trials_s"] += time.perf_counter()-trial_started
                                        entries.append({"candidate": candidate, "cstar_trials": trials, "label": computed})
                                        solved[candidate["candidate_id"]] = solution
                                        if progress and (candidate_index+1) % 8 == 0:
                                            progress({"stage": "collect_candidate_batch", "episode": episode_id,
                                                "segment": segment, "labeled": candidate_index+1, "candidates": len(choices),
                                                "decision_wall_s": time.perf_counter()-decision_started})
                                    successor_started = time.perf_counter()
                                    chosen, snapshot, successor = sample_successor(environment, choices, solved, previous_action,
                                        objective, solver_settings, chooser, settings.segment_resampling_attempts)
                                    timings["successor_s"] = time.perf_counter()-successor_started
                                    record = {"format": DECISION_FORMAT, "planner_source_sha256": planner_source_sha256(),
                                        "group_id": f"{episode_id}:segment:{segment}", "scene_id": base_id, "split": split,
                                        "objective": specification, "label_source": label_source, "observation": observation,
                                        "candidates": entries, "rule_selected_candidate_id": choices[0]["candidate_id"],
                                        "episode": {"id": episode_id, "road_seed": seed, "kind": kind,
                                            "initial_index": initial_index, "behavior": "uniform_random", "lattice_profile": profile,
                                            "road_length_m": road_length(environment.config.road_spec),
                                            "progress_m": environment._info["evaluation"]["ground_truth"]["reference_line_progress_m"],
                                            "selected_candidate_id": chosen["candidate_id"] if chosen else None,
                                            "segment": segment, "step": round(environment.elapsed_time_s/environment.dt_s)-1,
                                            "start_state": list(environment._backend.car._state_values()),
                                            "initial_control": list(previous_action)}, "successor": successor,
                                        "timings": timings}
                                    serialization_started = time.perf_counter()
                                    stream.write(json.dumps(json_safe(record), ensure_ascii=False, allow_nan=False)+"\n")
                                    stream.flush()
                                    timings["serialization_s"] = time.perf_counter()-serialization_started
                                    for key, value in timings.items():
                                        timing_totals[key] += value
                                    decisions += 1
                                    total_decisions += 1
                                    total_candidates += len(entries)
                                    scene_candidates += len(entries)
                                    decision_successes = sum(t["constraints_satisfied"] for e in entries for t in e["cstar_trials"])
                                    total_trials += len(entries)*settings.trials
                                    successful_trials += decision_successes
                                    segments.append(successor)
                                    if progress:
                                        progress({"stage": "collect_decision", "episode": episode_id, "split": split,
                                            "segment": segment, "candidates": len(entries), "successor_accepted": successor["accepted"],
                                            "successful_trials": decision_successes, "start_progress_m": current,
                                            "end_progress_m": successor["attempts"][-1].get("trace", [{}])[-1].get("progress_m", current),
                                            "wall_s": time.perf_counter()-decision_started,
                                            "collection_wall_s": time.perf_counter()-started,
                                            "total_candidates": total_candidates, "timings": timings})
                                    if snapshot is None:
                                        faults.append(successor["reason"])
                                        terminal_behavior = "successor_failure"
                                        break
                                    observation, _ = environment.restore(snapshot)
                                    previous_action = tuple(successor["attempts"][-1]["final_control"])
                                    current = float(environment._info["evaluation"]["ground_truth"]["reference_line_progress_m"])
                                    if settings.until_road_end and current >= goal:
                                        terminal_behavior = "road_goal_reached"
                                        break
                                    if chosen["reference"]["planned_stop"]:
                                        terminal_behavior = "planned_stop_reference_complete"
                                        break
                                else:
                                    terminal_behavior = "maximum_segments_reached" if settings.until_road_end else "decision_budget_reached"
                                    if settings.until_road_end:
                                        faults.append("maximum_segments_reached")
                                final_progress = float(environment._info["evaluation"]["ground_truth"]["reference_line_progress_m"])
                                scenes.append({"episode_id": episode_id, "scene_id": base_id, "split": split,
                                    "decisions": decisions, "faults": faults, "terminal_behavior": terminal_behavior,
                                    "candidates":scene_candidates,
                                    "road_completed": terminal_behavior == "road_goal_reached",
                                    "final_progress_m": final_progress, "goal_progress_m": goal,
                                    "wall_s": time.perf_counter()-episode_started,
                                    "initialization": initialization, "behavior": "uniform_random", "lattice_profile": profile,
                                    "planner_fault":planner_fault,
                                    "initial_progress_m": initial_progress, "initial_pose": initial_pose,
                                    "segments": segments, "road_length_m": road_length(scene_config["road_spec"]),
                                    "obstacle_passage": obstacle_passage_report(scene_config["road_spec"]) if kind=="randomized" else None,
                                    "environment_configuration": scene_config})
                                write_json(path.with_suffix(".collection.json"), collection_report(False))
                                if progress:
                                    progress({"stage": "collect_episode_complete", "episode": episode_id,
                                        "decisions": decisions, "completion": terminal_behavior,
                                        "final_progress_m": final_progress, "road_length_m": road_length(scene_config["road_spec"]),
                                        "faults": faults, "wall_s": scenes[-1]["wall_s"], "completed_episodes": len(scenes)})
    report = collection_report(True)
    write_json(path.with_suffix(".collection.json"), report)
    return report


def collection_coverage(report, *, require_road_end):
    """Count actual chains and visited progress; generated roads alone are not coverage."""
    roads=[]
    for identity,split in report['base_road_splits'].items():
        scenes=[s for s in report['scenes'] if s['scene_id']==identity]
        intervals=[]
        for scene in scenes:
            previous_progress=scene['initial_progress_m']
            for segment in scene.get('segments',[]):
                if segment['accepted']:
                    transition=segment['attempts'][-1]
                    points=[p['progress_m'] for p in transition.get('trace',[])]
                    if points:
                        intervals.append((min(previous_progress,*points),max(previous_progress,*points)))
                        previous_progress=points[-1]
        merged=[]
        for low,high in sorted(intervals):
            if merged and low<=merged[-1][1]+1e-8:merged[-1][1]=max(merged[-1][1],high)
            else:merged.append([low,high])
        distance=sum(high-low for low,high in merged)
        roads.append({'scene_id':identity,'split':split,'sequences':len(scenes),
            'decisions':sum(s['decisions'] for s in scenes),
            'completed_sequences':sum(s['road_completed'] for s in scenes),
            'visited_progress_m':distance,'visited_fraction':distance/scenes[0]['road_length_m'],
            'progress_intervals_m':merged,'road_length_m':scenes[0]['road_length_m']})
    missing_labels=[r['scene_id'] for r in roads if not r['decisions']]
    missing_goals=[r['scene_id'] for r in roads if not r['completed_sequences']] if require_road_end else []
    illegal=[s['episode_id'] for s in report['scenes']
             if s.get('initialization',{}).get('initial_region',{}).get('legal') is False]
    return {'roads':roads,'missing_label_roads':missing_labels,'missing_goal_roads':missing_goals,
            'illegal_initial_episodes':illegal,'require_road_end':require_road_end,
            'coverage_complete':bool(roads) and not (missing_labels or missing_goals or illegal)}


class CollectionCoverageError(RuntimeError):
    def __init__(self,summary):
        super().__init__('Required label splits are unavailable; diagnostic artifacts saved')
        self.summary=summary


def collect_dataset(output, *, settings=None, objective=None, solver_settings=None, progress=None, road_settings=None):
    """Generate/freeze labels once using the same collection stage as run."""
    output = Path(output)
    records = output/"decisions.jsonl"
    if records.exists() or (output/"dataset"/"data.npz").exists():
        raise FileExistsError(f"Collection output already exists: {output}; choose a new directory")
    settings = settings or PipelineSettings()
    objective = objective or CStarTrackingSettings()
    solver_settings = solver_settings or SolverSettings()
    report = collect_records(records, settings, objective, solver_settings, progress, road_settings=road_settings)
    coverage=collection_coverage(report,require_road_end=settings.until_road_end)
    encoding_started = time.perf_counter()
    nonempty_splits={s['split'] for s in report['scenes'] if s['decisions']}
    metadata=None
    required_splits = {'train'} if settings.train_only else {'train','validation','test'}
    if nonempty_splits==required_splits:
        metadata = import_dataset(records, output/"dataset", required_splits=required_splits)
        metadata.update(collection_coverage=coverage,
                        road_coverage_complete=coverage['coverage_complete'],
                        dataset_scope='train_only_addition' if settings.train_only else 'three_split_training')
        write_json(output/'dataset'/'metadata.json',metadata)
    else:
        coverage['empty_splits']=sorted(required_splits-nonempty_splits)
    ready = bool(metadata and metadata['training_eligible'])
    status = ('ready_to_append' if settings.train_only else 'ready' if coverage['coverage_complete']
              else 'ready_with_incomplete_road_coverage') if metadata is not None else 'missing_label_splits'
    summary = {"scope": "fixed_cstar_dataset_collection", "base_roads": report["base_roads"],
        "episodes": len(report["scenes"]), "road_completed_episodes": sum(s["road_completed"] for s in report["scenes"]),
        "decisions": report["decisions"], "candidate_rows": report["candidates"], "trials": report["trials"],
        "successful_trials": report["successful_trials"], "collection_wall_s": report["wall_s"],
        "encoding_wall_s": time.perf_counter()-encoding_started, "timing_totals": report["timing_totals"],
        "dataset": str(output/"dataset") if metadata is not None else None, "objective": objective.specification(),
        "coverage":coverage,"training_eligible":ready,"road_coverage_complete":coverage['coverage_complete'],
        "status":status,
        "pipeline_settings": asdict(settings), "solver_settings": asdict(solver_settings)}
    write_json(output/"summary.json", summary)
    plot_road_coverage(report, output)
    if metadata is None:
        raise CollectionCoverageError(summary)
    return summary


def augmentation_preflight(dataset, settings, objective, solver_settings, road_settings):
    """Check contracts and generated road identities before any expensive labels."""
    data, metadata = load_dataset(dataset)
    if not settings.train_only or settings.kinds != ('randomized',):
        raise ValueError('Augmentation collects new randomized roads into Train only')
    if set(data['splits']) != {'train', 'validation', 'test'}:
        raise ValueError('Existing dataset must contain all three splits')
    expected = _collection_label_source(settings, objective, solver_settings, road_settings)
    with make_environment('randomized', settings.train_seeds[0], max_steps=100,
                          initial_count=settings.starts_per_road, road_settings=road_settings,
                          initial_safety_margin_m=solver_settings.safety_margin_m) as environment:
        expected['plant'] = _plant_source(environment.configuration())
    if (metadata['objective'] != objective.specification()
            or label_source_contract(metadata['label_source']) != label_source_contract(expected)):
        raise ValueError('New collection must retain the existing C* and sampling contract')
    reserved = set(metadata['scene_splits'])
    reserved.update(r['scene_id'] for r in metadata.get('collection_coverage', {}).get('roads', []))
    roads = []
    for seed in settings.train_seeds:
        specification = generate_random_road_spec(seed, road_settings)
        identity = road_identity(specification)
        if identity in reserved or any(row['scene_id'] == identity for row in roads):
            raise ValueError(f'New seed {seed} repeats an existing road, possibly a holdout')
        roads.append({'seed': seed, 'scene_id': identity, 'split': 'train',
                      'length_m': road_length(specification)})
        for initial_index in range(settings.starts_per_road):
            for profile in settings.planner_profiles:
                prefix = f'randomized:{seed}:initial:{initial_index}:lattice:{profile}:segment:'
                if any(str(group).startswith(prefix) for group in data['groups']):
                    raise ValueError('New seed/profile would repeat an existing decision group')
    return {'base_dataset': str(Path(dataset).resolve()), 'base_data_sha256': metadata['data_sha256'],
            'requested_new_roads': roads, 'requested_new_sequences': len(roads)*settings.starts_per_road*len(settings.planner_profiles),
            'new_road_ids_disjoint_from_all_existing_splits': True,
            'Cstar_plant_and_sampling_contract_unchanged': True,
            'base_splits': {split: {'rows': int(np.sum(data['splits'] == split)),
                                    'decisions': len(np.unique(data['groups'][data['splits'] == split])),
                                    'roads': len(np.unique(data['scenarios'][data['splits'] == split]))}
                            for split in ('train', 'validation', 'test')}}


def augment_training_dataset(dataset, output, *, settings, objective, solver_settings, progress=None, road_settings=None):
    """Reuse the collection stage, then append its Train rows to a new frozen dataset."""
    output = Path(output)
    if (output/'augmentation.json').exists() or (output/'dataset').exists() or (output/'addition'/'decisions.jsonl').exists():
        raise FileExistsError(f'Augmentation output already exists: {output}')
    road_settings = road_settings or RoadRandomizationSettings()
    preflight = augmentation_preflight(dataset, settings, objective, solver_settings, road_settings)
    write_json(output/'preflight.json', preflight)
    if progress:
        progress({'stage': 'augmentation_preflight_complete', **preflight})
    collection = collect_dataset(output/'addition', settings=settings, objective=objective,
                                 solver_settings=solver_settings, progress=progress, road_settings=road_settings)
    metadata = append_training_dataset(dataset, output/'addition'/'dataset', output/'dataset')
    counts = {split: {'rows': metadata['split_rows'][split],
                       'decisions': preflight['base_splits'][split]['decisions'] + (
                           metadata['augmentation']['added_train_decisions'] if split == 'train' else 0),
                       'roads': sum(value == split for value in metadata['scene_splits'].values())}
              for split in ('train', 'validation', 'test')}
    summary = {'operation': 'append_new_roads_to_train_only', 'dataset': str(output/'dataset'),
               'addition_collection': collection, 'splits': counts,
               'added_train_rows': metadata['augmentation']['added_train_rows'],
               'added_train_decisions': metadata['augmentation']['added_train_decisions'],
               'holdout_arrays_unchanged': metadata['augmentation']['holdout_arrays_unchanged'],
               'training_eligible': metadata['training_eligible'],
               'road_coverage_complete': metadata['road_coverage_complete'],
               'status': 'appended' if metadata['road_coverage_complete'] else 'appended_with_incomplete_road_coverage'}
    write_json(output/'augmentation.json', summary)
    return summary


def closed_loop_episode(kind, seed, model, objective, solver_settings, steps, *, initial_index=0,
                        settings=None, road_settings=None):
    from .evaluation import RoadEvaluation
    settings = settings or replace(PipelineSettings(), kinds=(kind,), starts_per_road=1, evaluation_steps=steps)
    environment, _, initialization = initialize_episode(kind,seed,initial_index,settings,objective,road_settings,solver_settings)
    with environment:
        evaluator=RoadEvaluation(environment, model=model, objective=objective, solver_settings=solver_settings, seed=seed, performance=performance_profile(seed + 1234))
        report=evaluator.run(max_control_steps=steps)
    report.update(scene_id=f"{kind}:{seed}:initial:{initial_index}",initialization=initialization,
                  render_seed=initialization["episode_seed"] if kind=="randomized" else seed,
                  base_road_id=report["road_id"],time_limit_reached=report["completion"]=="complete_segment_budget_reached")
    return report


def evaluate_closed_loop(model, settings, objective, solver_settings, output, progress=None, *, road_settings=None):
    episodes = []
    for seed in settings.test_seeds:
        for kind in tuple(dict.fromkeys(settings.kinds+("blocked",))):
            for initial_index in range(settings.starts_per_road if kind == "randomized" else 1):
                pair = [closed_loop_episode(kind, seed, candidate_model, objective, solver_settings, settings.evaluation_steps,
                            initial_index=initial_index, settings=settings, road_settings=road_settings)
                        for candidate_model in (None, model)]
                episodes.extend(pair)
                if progress:
                    progress({"stage": "closed_loop_scene", "scene": pair[0]["scene_id"],
                              "rule_steps": pair[0]["steps"], "learned_steps": pair[1]["steps"],
                              "rule_return": pair[0]["return"], "learned_return": pair[1]["return"]})
    summaries = {}
    for selector in ("rule", "learned"):
        rows = [item for item in episodes if item["selector"] == selector]
        summaries[selector] = {"episodes": len(rows), "mean_return": float(np.mean([row["return"] for row in rows])),
                              "mean_progress_m": float(np.mean([row["progress_m"] for row in rows])),
                              "collisions": sum(row["collision"] for row in rows), "offroad": sum(row["offroad"] for row in rows),
                              "faults": sum(row["fault"] is not None for row in rows)}
        summaries[selector]["completed_stops"] = sum(row["stop_completed"] for row in rows)
    report = {"objective": objective.specification(), "summary": summaries, "episodes": episodes,
              "runtime_qualified": False, "evaluation_type": "actual_env_complete_candidate_cstar_execution"}
    write_json(output, report)
    return report


def run_pipeline(output, *, settings=None, objective=None, solver_settings=None,
                 epochs=150, hidden=32, seed=0, learning_rate=0.005, progress=None,
                 road_settings=None):
    settings = settings or PipelineSettings()
    objective = objective or CStarTrackingSettings()
    solver_settings = solver_settings or SolverSettings()
    output = Path(output)
    collect_dataset(output,settings=settings,objective=objective,solver_settings=solver_settings,
                    progress=progress,road_settings=road_settings)
    collection=json.loads((output/'decisions.collection.json').read_text(encoding='utf-8'))
    metadata=json.loads((output/'dataset'/'metadata.json').read_text(encoding='utf-8'))
    model, training = train_model(output / "dataset", output / "model.json", epochs=epochs,
                                  hidden=hidden, seed=seed, learning_rate=learning_rate)
    if progress:
        progress({"stage": "training_complete", "splits": training["splits"]})
    ranking = evaluate_policy(model, output / "dataset", output / "evaluation.json")
    evaluation = evaluate_closed_loop(model, settings, objective, solver_settings, output / "closed_loop.json", progress,
                                      road_settings=road_settings)
    plot_training(training, output)
    plot_closed_loop(evaluation, output)
    plot_road_coverage(collection, output)
    summary = {"scope": "env_cstar_policy_training", "dataset_rows": metadata["rows"],
               "decisions": metadata["decisions"], "label_source": metadata["label_source"],
               "objective": objective.specification(), "prediction_metrics": ranking["splits"],
               "closed_loop": evaluation["summary"], "collection_wall_s": collection["wall_s"],
               "base_roads": collection["base_roads"], "road_randomization": collection["road_randomization"],
               "model": str(output / "model.json"), "runtime_qualified": False}
    write_json(output / "summary.json", summary)
    return summary
