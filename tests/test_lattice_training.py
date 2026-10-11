"""Shared C* objective, native solver/env integration and the single training pipeline."""
from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np

from offline.lattice_training.dataset import (
    DECISION_FORMAT, import_dataset, load_dataset, objective_hash, score_decisions, validate_objective)
from offline.lattice_training.model import ReturnMLP, evaluate_policy, train_model
from offline.lattice_training.objective import CSTAR_TARGET, CStarTrackingSettings, cstar_trial_returns, sample_reference, tracking_loss
from offline.lattice_training.features import FEATURE_NAMES, encode_candidate
from offline.lattice_training.source import candidate_api, planner_source_sha256


def candidate(offset=0.0):
    """Native geometry encoding for an analytic test target, not driving evidence."""
    api = candidate_api()
    curve = api.LatticeCurve([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)], 0.0, 0.025)
    lateral = {"origin": 0.0, "scale": 1.5, "range": [0.0, 2.0], "terminal_offset_m": offset,
               "coeffs_low_to_high": api._lattice_lateral(0, 0, 0, offset, 1.5)}
    profile = [[0.0, 0.1], [3.0, 0.1]]
    return {"candidate_id": str(offset), "minimum_clearance_m": 0.1,
            "descriptor": {"length_m": 1.5, "offset_m": offset, "duration_s": 3.0,
                           "terminal_speed_mps": 0.1, "score": -offset**2,
                           "costs": dict.fromkeys(("center", "clearance", "steering", "consistency", "motion"), 0.0)},
            "reference": {"valid": True, "valid_for_s": 0.2, "planned_stop": False,
                          "frenet_path": {"reference_curve": curve.record(), "lateral": lateral},
                          "longitudinal_profile": {"type": "S_REF_OF_TIME", "duration_s": 3.0,
                                                   "coeffs_low_to_high": [0.0, 0.3], "stop": False},
                          "speed_profile": profile}}


def execution(proposal, shift=0.0, identity="trial-0", settings=None):
    cfg = settings or CStarTrackingSettings()
    times, actual = sample_reference(proposal, cfg)
    actual = actual.copy()
    actual[1:, 1] += shift
    return {"trial_id": identity, "solver_status": "approximate", "constraints_satisfied": True,
            "initial_state": actual[0].tolist(), "initial_control": [0.0, 0.0],
            "times_s": times.tolist(), "states": actual.tolist(),
            "controls": np.zeros((len(times)-1, 2)).tolist()}


def decision_records():
    cfg = CStarTrackingSettings()
    specification = cfg.specification()
    available = dict.fromkeys(("cone_detections", "wheel_speed", "imu_angular_velocity", "lidar_cartesian"), True)
    observation = {"timestamp_s": 1.0, "dt_s": 0.05, "frame_id": "base_link",
                   "sensor_available": available, "sensor_age_s": dict.fromkeys(available, 0.0),
                   "estimation": {"state": {"speed_mps": 0.1, "yaw_rate_rps": 0.0,
                                             "speed_valid": True, "yaw_rate_valid": True},
                                  "road": {"valid": True, "x_range_m": [0.0, 3.0], "lane_width_m": 1.0,
                                           "centerline_xy": [(0.0, 0.0), (3.0, 0.0)],
                                           "left_boundary_xy": [(0.0, 0.5), (3.0, 0.5)],
                                           "right_boundary_xy": [(0.0, -0.5), (3.0, -0.5)]},
                                  "obstacles": {"available": True, "points_xyz_m": []}}}
    records = []
    for split in ("train", "validation", "test"):
        entries = []
        for offset in (0.0, 0.02, -0.02):
            proposal = candidate(offset)
            entries.append({"candidate": proposal,
                            "cstar_trials": [execution(proposal, abs(offset), "noise-0"),
                                             execution(proposal, 2*abs(offset), "noise-1")]})
        records.append({"format": DECISION_FORMAT, "planner_source_sha256": planner_source_sha256(),
                        "group_id": split+"-decision", "scene_id": split+"-synthetic-scene", "split": split,
                        "objective": specification,
                        "label_source": {"name": "synthetic_contract_fixture", "version": "1",
                                         "execution_assumption": "cstar", "objective_sha256": objective_hash(specification)},
                        "observation": deepcopy(observation), "candidates": entries})
    return records


class TrackingObjective(unittest.TestCase):
    def setUp(self):
        self.settings = CStarTrackingSettings(horizon_s=1.0, dt_s=0.1)
        self.target = np.zeros((11, 4))
        self.controls = np.zeros((10, 2))

    def test_zero_tracking_error_and_constant_control_have_zero_loss(self):
        value, terms = tracking_loss(self.target, self.target, self.controls, [0.0, 0.0], self.settings)
        self.assertEqual(value, 0.0)
        self.assertTrue(all(term == 0.0 for term in terms.values()))

    def test_normalized_tracking_terms_match_the_formula(self):
        actual = self.target.copy()
        actual[1:] = [0.1, 0.0, 0.1, 0.1]
        value, terms = tracking_loss(self.target, actual, self.controls, [0.0, 0.0], self.settings)
        self.assertAlmostEqual(terms["position"], 1.0)
        self.assertAlmostEqual(terms["heading"], 0.2)
        self.assertAlmostEqual(terms["speed"], 0.2)
        self.assertAlmostEqual(terms["terminal_position"],2.0)
        self.assertAlmostEqual(terms["terminal_speed"],.2)
        self.assertAlmostEqual(value, 3.6)

    def test_heading_wrap_and_initial_control_change(self):
        target, actual = self.target.copy(), self.target.copy()
        target[:, 2], actual[:, 2] = math.pi-0.01, -math.pi+0.01
        controls = self.controls.copy()
        controls[:, 0], controls[:, 1] = 0.1, 0.2
        _, terms = tracking_loss(target, actual, controls, [0, 0], self.settings)
        self.assertAlmostEqual(terms["heading"], 0.2*(0.02/0.1)**2)
        self.assertAlmostEqual(terms["drive_rate"], 0.01*(0.1/0.1)**2*0.1)
        self.assertAlmostEqual(terms["steering_rate"], 0.05*(0.2/0.1)**2*0.1)

    def test_specification_rejects_changed_semantics_or_bad_settings(self):
        specification = self.settings.specification()
        self.assertEqual(validate_objective(specification), objective_hash(specification))
        specification["definition"]["frame"] = "moving_base_link"
        with self.assertRaisesRegex(ValueError, "shared specification"):
            validate_objective(specification)
        with self.assertRaises(ValueError):
            CStarTrackingSettings(dt_s=0.07)

    def test_full_reference_uses_original_time_profile_and_spatial_curve(self):
        times, target = sample_reference(candidate(), self.settings)
        np.testing.assert_allclose(target[:, 0], 0.1*times)
        np.testing.assert_allclose(target[:, 1:3], 0.0)
        np.testing.assert_allclose(target[:, 3], 0.1)
        short = candidate()
        with self.assertRaisesRegex(ValueError, "refusing a truncated score"):
            sample_reference(short, replace(self.settings,max_horizon_s=10))
        self.assertAlmostEqual(times[-1],20.0)
        self.assertAlmostEqual(target[-1,0],2.0)

    def test_trials_use_negative_loss_and_explicit_failure_encoding(self):
        proposal = candidate()
        trials = [execution(proposal, 0.1, "a", self.settings),
                  {"trial_id": "b", "solver_status": "infeasible", "initial_state": [0, 0, 0, 0.1],
                   "initial_control": [0, 0]}]
        label, _ = cstar_trial_returns(proposal, trials, self.settings.specification())
        self.assertAlmostEqual(label["returns"][0], -2.75)
        self.assertEqual(label["returns"][1], -self.settings.failure_loss)
        trials[0]["times_s"] = trials[0]["times_s"][:-1]
        with self.assertRaisesRegex(ValueError, "trial times"):
            cstar_trial_returns(proposal, trials, self.settings.specification())

    def test_full_path_duration_and_speed_utility_penalize_standing_still(self):
        from offline.lattice_training.objective import reference_plan,path_completion
        plan=reference_plan(candidate())
        self.assertEqual(plan['spatial_span_m'],2.0)
        self.assertGreater(plan['duration_s'],3.0)
        actual=plan['states'].copy()
        zero=np.zeros((plan['steps'],2))
        moving,terms=tracking_loss(actual,actual,zero,(0,0))
        stationary=np.tile(actual[0],(len(actual),1));stationary[:,3]=0
        stopped,_=tracking_loss(actual,stationary,zero,(0,0))
        self.assertLess(terms['speed_utility'],0)
        self.assertLess(moving,stopped)
        self.assertFalse(path_completion(candidate(),stationary)['reached'])

    def test_speed_preference_uses_duration_average_and_old_objective_is_rejected(self):
        proposal=candidate();times,slow=sample_reference(proposal)
        fast=deepcopy(proposal);fast['reference']['speed_profile']=[[0,.2],[3,.2]]
        times_fast,target_fast=sample_reference(fast)
        a,_=tracking_loss(slow,slow,np.zeros((len(slow)-1,2)),(0,0))
        b,_=tracking_loss(target_fast,target_fast,np.zeros((len(target_fast)-1,2)),(0,0))
        self.assertLess(b,a)
        self.assertLess(times_fast[-1],times[-1])
        old=CStarTrackingSettings().specification();old['id']='cstar_tracking_loss_v1'
        with self.assertRaises(ValueError): validate_objective(old)


class CStarPolicyTraining(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.records = decision_records()
        self.input = self.root / "records.jsonl"

    def tearDown(self):
        self.directory.cleanup()

    def write_records(self):
        self.input.write_text("\n".join(json.dumps(record) for record in self.records)+"\n", encoding="utf-8")

    def test_policy_target_is_negative_mean_loss_and_train_defaults_to_cstar(self):
        self.write_records()
        metadata = import_dataset(self.input, self.root / "data")
        data, _ = load_dataset(self.root / "data")
        self.assertEqual(metadata["label_definition"], "negative_expected_full_path_cost")
        # Mean speed utility .25 minus mean path+terminal deviation .3.
        np.testing.assert_allclose(data["y"][:3], [.25, -.05, -.05], atol=1e-8)
        self.assertTrue((data["label_trials"] == 2).all())
        model, report = train_model(self.root / "data", self.root / "model.json", epochs=10)
        self.assertEqual(model.provenance["target"], CSTAR_TARGET)
        self.assertEqual(report["model"]["target"], CSTAR_TARGET)

    def test_wrong_return_or_inconsistent_initial_trials_are_rejected(self):
        self.records[0]["candidates"][0]["label"] = {"return": 10.0}
        self.write_records()
        with self.assertRaisesRegex(ValueError, "negative tracking loss"):
            import_dataset(self.input, self.root / "data")
        del self.records[0]["candidates"][0]["label"]
        trial = self.records[0]["candidates"][1]["cstar_trials"][0]
        trial["initial_state"][3] += 0.01
        trial["states"][0][3] += 0.01
        self.write_records()
        with self.assertRaisesRegex(ValueError, r"share C\* trial"):
            import_dataset(self.input, self.root / "data")

    def test_generic_reward_is_not_silently_used_as_the_default_target(self):
        objective = {"id": "other_reward", "direction": "maximize", "discount": 1.0,
                     "horizon_s": 3.0, "definition": "road progress"}
        for record in self.records:
            record["objective"] = objective
            record["label_source"]["objective_sha256"] = objective_hash(objective)
            for entry in record["candidates"]:
                entry["label"] = {"return": 1.0}
        self.write_records()
        with self.assertRaisesRegex(ValueError, r"complete C\* tracking"):
            import_dataset(self.input, self.root / "data")

    def test_scoring_utility_does_not_import_simulation(self):
        self.write_records()
        import_dataset(self.input, self.root / "data")
        model, _ = train_model(self.root / "data", self.root / "model.json", epochs=8)
        code = """import importlib.abc, runpy, sys
class BlockSimulation(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        blocked = ('dreamgym', 'gymnasium', 'offline.planner_sim', 'offline.control_pid', 'rclpy')
        if any(fullname == name or fullname.startswith(name+'.') for name in blocked):
            raise ImportError('Scoring imported simulation: '+fullname)
sys.meta_path.insert(0, BlockSimulation())
sys.argv = ['offline.lattice_training'] + sys.argv[1:]
runpy.run_module('offline.lattice_training', run_name='__main__')
"""
        result = subprocess.run([sys.executable, "-B", "-c", code, "score", "--records", str(self.input),
                                 "--model", str(self.root / "model.json"), "--output", str(self.root / "scores.json")],
                                cwd=ROOT, capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stderr+"\n"+result.stdout)
        report = json.loads((self.root / "scores.json").read_text(encoding="utf-8"))
        self.assertEqual(len(report["decisions"]), 3)

    def test_mixed_objectives_and_wrong_fingerprints_are_rejected(self):
        changed = replace(CStarTrackingSettings(), horizon_s=9.0).specification()
        self.records[1]["objective"] = changed
        self.write_records()
        with self.assertRaisesRegex(ValueError, "fingerprint"):
            import_dataset(self.input, self.root / "data")
        self.records[1]["label_source"]["objective_sha256"] = objective_hash(changed)
        self.write_records()
        with self.assertRaisesRegex(ValueError, "mix"):
            import_dataset(self.input, self.root / "data")

    def test_scene_leakage_duplicate_decisions_and_empty_trials_are_rejected(self):
        original = deepcopy(self.records)
        self.records[1]["scene_id"] = self.records[0]["scene_id"]
        self.write_records()
        with self.assertRaisesRegex(ValueError, "different splits"):
            import_dataset(self.input, self.root / "data")
        self.records = deepcopy(original)
        self.records[1]["group_id"] = self.records[0]["group_id"]
        self.write_records()
        with self.assertRaisesRegex(ValueError, "repeats"):
            import_dataset(self.input, self.root / "data")
        self.records = deepcopy(original)
        self.records[0]["candidates"][0]["cstar_trials"] = []
        self.write_records()
        with self.assertRaisesRegex(ValueError, "nonempty cstar_trials"):
            import_dataset(self.input, self.root / "data")

    def test_dataset_tampering_is_rejected(self):
        self.write_records()
        import_dataset(self.input, self.root / "data")
        path = self.root / "data/data.npz"
        path.write_bytes(path.read_bytes()+b"tampered")
        with self.assertRaisesRegex(ValueError, "hash"):
            load_dataset(self.root / "data")


class PolicyModelChecks(unittest.TestCase):
    def test_model_learns_sigmoid_scores_and_json_roundtrips(self):
        rng = np.random.default_rng(10)
        X = np.zeros((96, len(FEATURE_NAMES)))
        X[:, :2] = rng.uniform(-1, 1, size=(96, 2))
        y = 0.2*X[:, 0]-0.1*X[:, 1]-0.5
        model = ReturnMLP(hidden=16, seed=2)
        history = model.fit(X[:72], y[:72], np.repeat(np.arange(24), 3),
                            X[72:], y[72:], epochs=100, learning_rate=0.01)
        self.assertLess(np.sqrt(np.mean((model.predict(X[72:])-model.training_targets(y[72:]))**2)), 0.15)
        np.testing.assert_allclose(model.x_mean, X[:72].mean(axis=0))
        self.assertEqual(len(history), 100)
        model.provenance = {"objective": CStarTrackingSettings().specification(), "target": CSTAR_TARGET,
                            "training_target_transform": model.target_specification}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.json"
            model.save(path)
            restored = ReturnMLP.load(path)
            np.testing.assert_allclose(restored.predict(X), model.predict(X), atol=1e-12)
            record = json.loads(path.read_text(encoding="utf-8"))
            record["x_scale"][0] = 0.0
            path.write_text(json.dumps(record), encoding="utf-8")
            with self.assertRaises(ValueError):
                ReturnMLP.load(path)

    def test_features_ignore_truth_labels_and_controller_state(self):
        record = decision_records()[0]
        proposal = record["candidates"][0]["candidate"]
        expected = encode_candidate(record["observation"], proposal)
        changed = deepcopy(record["observation"])
        changed["ground_truth"] = {"speed": 1e9}
        changed["controller_context"] = {"speed_integral": 1e9, "lateral_integral": -1e9}
        modified = deepcopy(proposal)
        modified["label"] = {"return": 1e9}
        np.testing.assert_array_equal(encode_candidate(changed, modified), expected)
        self.assertEqual(len(expected), 140)
        self.assertFalse(hasattr(candidate_api(), "PolicyController"))

    def test_control_edits_do_not_invalidate_planner_identity(self):
        from offline.lattice_training import source
        original = source.SOURCE.read_text(encoding="utf-8")
        expected = planner_source_sha256()
        with tempfile.TemporaryDirectory() as directory:
            copy = Path(directory) / "policy.py"
            copy.write_text(original.replace("speed_kp: float = 1.4", "speed_kp: float = 1.6"), encoding="utf-8")
            with patch.object(source, "SOURCE", copy):
                self.assertEqual(planner_source_sha256(), expected)
            copy.write_text(original.replace("preview_m: float = 0.35", "preview_m: float = 0.36"), encoding="utf-8")
            with patch.object(source, "SOURCE", copy):
                self.assertNotEqual(planner_source_sha256(), expected)


class RandomRoadCollectionChecks(unittest.TestCase):
    def test_random_obstacles_leave_a_connected_inflated_body_passage(self):
        from offline.lattice_training.roads import generate_random_road_spec,obstacle_passage_report,road_identity
        for seed in range(10):
            road=generate_random_road_spec(seed)
            self.assertGreaterEqual(len(road['obstacles']),3)
            report=obstacle_passage_report(road)
            self.assertGreaterEqual(report['minimum_gap_m'],.55)
            self.assertTrue(report['passable'])
            self.assertGreater(report['road_clearance_m'],0)
            changed=deepcopy(road);changed['obstacles']=[]
            self.assertEqual(road_identity(road),road_identity(changed))

    def test_inspection_renders_native_admitted_and_rejected_candidates(self):
        from offline.lattice_training.evaluation import RoadEvaluation,road_environment
        from offline.lattice_training.pipeline import make_environment
        from offline.lattice_training.cstar import SolverSettings
        with make_environment('straight',0,max_steps=5000) as env:
            env.reset(seed=0);env.step((0,0))
            evaluator=RoadEvaluation(env,solver_settings=SolverSettings(population=4,iterations=1,max_evaluations=4))
            report=evaluator.run(max_control_steps=1)
            self.assertGreater(len(report['trace']),60)
            panel=evaluator.probe(.8)
            self.assertGreater(panel['admitted_count'],0)
            self.assertGreater(panel['rejected_count'],0)
            self.assertTrue(all(len(c['points'])>2 for c in panel['candidates']))
            self.assertEqual(sum(c['selected'] for c in panel['candidates']),1)
            self.assertEqual(next(c['score'] for c in panel['candidates'] if c['selected']),
                             max(c['score'] for c in panel['candidates'] if c['admitted']))

    def test_visualization_cli_exports_whole_trace_and_point_candidates(self):
        with tempfile.TemporaryDirectory() as directory:
            result=subprocess.run([sys.executable,'-B','-m','offline.lattice_training.visualize',
                '--output',directory,'--road-seed','0','--max-segments','1','--population','4',
                '--iterations','1','--max-evaluations','4','--inspect-progress','.8,1.5'],
                cwd=ROOT,capture_output=True,text=True,timeout=120)
            self.assertEqual(result.returncode,0,result.stderr+result.stdout)
            for file in ('evaluation.png','evaluation.json','evaluation.html','candidates.png','road.json'):
                self.assertTrue(Path(directory,file).is_file())
            report=json.loads(Path(directory,'evaluation.json').read_text(encoding='utf-8'))
            self.assertGreater(len(report['trace']),60)
            self.assertEqual(len(report['inspection_panels']),2)
            self.assertAlmostEqual(report['segments'][0]['spatial_span_m'],2.0)
    def test_random_fragment_is_reproducible_long_and_native_compatible(self):
        from offline.lattice_training.roads import generate_random_road_spec, road_identity, road_length
        from dreamgym.envs import Road
        road = generate_random_road_spec(0)
        self.assertEqual(road, generate_random_road_spec(0))
        self.assertNotEqual(road_identity(road), road_identity(generate_random_road_spec(1)))
        self.assertGreater(road_length(road), 20.0)
        self.assertEqual(road['elements'][0]['geometry']['type'], 'straight')
        self.assertEqual({e['geometry']['type'] for e in road['elements']}, {'straight', 'circular_arc'})
        native = Road(road_spec=road)
        for progress in np.linspace(0.1, road_length(road)-0.1, 20):
            bounds = native.get_reference_lane_bounds_at_progress(progress)
            self.assertGreater(bounds['left_bound_m']-bounds['right_bound_m'], 0.9)

    def test_starts_cover_same_road_and_keep_notebook_vehicle_parameters(self):
        from offline.lattice_training.pipeline import make_environment
        from offline.planner_sim import load_notebook
        from offline.lattice_training.roads import road_length, road_identity
        configs, progresses = [], []
        for index in range(2):
            with make_environment('randomized', 0, max_steps=100, initial_index=index, initial_count=2) as env:
                config = env.configuration()
                configs.append(config)
                progresses.append(config['initial_state_config']['pose']['progress']['lower'])
                self.assertEqual(config['bicycle_model_config'], load_notebook().bicycle_model_config)
                self.assertEqual(config['integration_config'], load_notebook().integration_config)
        self.assertEqual(configs[0]['road_spec'], configs[1]['road_spec'])
        self.assertEqual(road_identity(configs[0]['road_spec']), road_identity(configs[1]['road_spec']))
        body=configs[0]['bicycle_model_config']['body_geometry']
        safe_start=body['rear_extent_from_center_of_mass_m']+.02+(body['width_m']/2+.02)*math.sin(.035)+.01
        middle = safe_start+(road_length(configs[0]['road_spec'])-3.0-safe_start)/2
        self.assertLess(progresses[0], middle)
        self.assertGreater(progresses[1], middle)
        self.assertGreater(progresses[1], 2.0)

    def test_lattice_profiles_change_candidates_and_keep_local_horizon(self):
        from offline.lattice_training.pipeline import PipelineSettings, initialize_episode, make_planner, candidates_at
        env, obs, _ = initialize_episode('straight', 0, 0, PipelineSettings(), CStarTrackingSettings(), None)
        self.assertIsNotNone(env)
        with env:
            sets = []
            for profile in ('standard', 'conservative'):
                planner = make_planner(profile)
                self.assertEqual(planner.cfg.horizon_m, 2.0)
                ref, diag = candidates_at(planner, obs)
                self.assertTrue(ref['valid'])
                sets.append({c['candidate_id'] for c in diag['candidate_references']})
            self.assertNotEqual(*sets)

    def test_initialization_is_one_draw_without_a_planner_acceptance_gate(self):
        from offline.lattice_training.pipeline import PipelineSettings, initialize_episode, make_planner, candidates_at
        first = initialize_episode('randomized', 200, 0, PipelineSettings(), CStarTrackingSettings(), None)
        second = initialize_episode('randomized', 200, 0, PipelineSettings(), CStarTrackingSettings(), None)
        try:
            self.assertEqual(first[2], second[2])
            self.assertEqual(first[2]['initial_state_draws'], 1)
            self.assertFalse(first[2]['filtered_by_planner'])
            np.testing.assert_array_equal(first[0]._backend.car._state_values(), second[0]._backend.car._state_values())
            self.assertTrue(first[2]['initial_region']['legal'])
        finally:
            first[0].close()
            second[0].close()

    def test_missing_perception_or_candidates_is_reported_without_redrawing_initial_state(self):
        from offline.lattice_training.pipeline import PipelineSettings, collect_records
        from offline.lattice_training.cstar import SolverSettings
        settings = PipelineSettings(train_seeds=(0,), validation_seeds=(100,), test_seeds=(200,),
                                    starts_per_road=1, planner_profiles=('standard',))
        with tempfile.TemporaryDirectory() as directory, patch(
                'offline.lattice_training.pipeline.candidates_at', return_value=({'valid': False, 'reason': 'coverage'}, {})) as planner:
            report = collect_records(Path(directory)/'records.jsonl', settings, CStarTrackingSettings(), SolverSettings())
            self.assertEqual(planner.call_count, 3)
            self.assertEqual(report['decisions'], 0)
            self.assertTrue(all(s['faults'] == ['coverage'] for s in report['scenes']))
            self.assertTrue(all(s['initialization']['initial_state_draws'] == 1 for s in report['scenes']))

    def test_invalid_ranges_and_sampler_settings_are_rejected(self):
        from offline.lattice_training.roads import RoadRandomizationSettings
        from offline.lattice_training.pipeline import PipelineSettings, make_environment, collect_records
        import inspect
        for kwargs in ({'straight_probability': 2.0}, {'radius_min_m': -1}, {'element_count_min': True}):
            with self.assertRaises(ValueError):
                RoadRandomizationSettings(**kwargs)
        with self.assertRaises(ValueError):
            make_environment('randomized', 0, max_steps=20, initial_index=2, initial_count=2)
        with self.assertRaises(ValueError):
            PipelineSettings(segment_resampling_attempts=0)
        for kwargs in ({'max_segments': 0}, {'until_road_end': 1}):
            with self.assertRaises(ValueError):
                PipelineSettings(**kwargs)
        self.assertNotIn('behavior_model', inspect.signature(collect_records).parameters)
        self.assertNotIn('rollout_behaviors', PipelineSettings.__dataclass_fields__)

    def test_duplicate_base_geometry_across_splits_is_rejected(self):
        from offline.lattice_training.pipeline import PipelineSettings, collect_records
        from offline.lattice_training.cstar import SolverSettings
        with tempfile.TemporaryDirectory() as directory, patch(
                'offline.lattice_training.pipeline.road_identity', return_value='same-road'), patch(
                'offline.lattice_training.pipeline.candidates_at', return_value=({'valid': False}, {})):
            with self.assertRaisesRegex(ValueError, 'leak'):
                collect_records(Path(directory)/'records.jsonl', PipelineSettings(
                    train_seeds=(0,),validation_seeds=(100,),test_seeds=(200,),
                    starts_per_road=1,planner_profiles=('standard',)),
                    CStarTrackingSettings(), SolverSettings())

    def test_random_successor_does_not_select_by_policy_or_reward(self):
        from offline.lattice_training.pipeline import sample_successor
        choices = [{'candidate_id': str(i), 'reward': i*100} for i in range(8)]
        solved = {str(i): {} for i in range(8)}
        expected = int(np.random.default_rng(12).permutation(8)[0])
        with patch('offline.lattice_training.pipeline.simulate_segment',
                   return_value=(object(), {'completed': True, 'reason': None})) as simulate:
            selected, snapshot, report = sample_successor(None, choices, solved, (0,0), None, None,
                                                          np.random.default_rng(12), 6)
            self.assertEqual(selected['candidate_id'], str(expected))
            self.assertIsNotNone(snapshot)
            self.assertTrue(report['accepted'])
            self.assertEqual(simulate.call_count, 1)
            choices[expected]['reward'] = -1e12
            selected, _, _ = sample_successor(None, choices, solved, (0,0), None, None, np.random.default_rng(12), 6)
            self.assertEqual(selected['candidate_id'], str(expected))

    def test_retry_only_after_a_completed_illegal_terminal_region(self):
        from offline.lattice_training.pipeline import sample_successor
        choices = [{'candidate_id': str(i)} for i in range(3)]
        solved = {str(i): {} for i in range(3)}
        invalid = {'completed': True, 'reason': 'terminal_region_illegal', 'terminal_region': {'legal': False}}
        valid = {'completed': True, 'reason': None, 'terminal_region': {'legal': True}}
        with patch('offline.lattice_training.pipeline.simulate_segment',
                   side_effect=[(None, invalid), (object(), valid)]) as simulate:
            _, snapshot, report = sample_successor(None, choices, solved, (0,0), None, None, np.random.default_rng(3), 3)
            self.assertIsNotNone(snapshot)
            self.assertEqual(len(report['attempts']), 2)
            self.assertEqual(simulate.call_count, 2)
            self.assertNotEqual(simulate.call_args_list[0].args[1]['candidate_id'],
                                simulate.call_args_list[1].args[1]['candidate_id'])
        for reason, completed in (('cstar:failed',False), ('cstar_execution_incomplete',False),
                                  ('environment_terminated_at_segment_end',True)):
            with self.subTest(reason=reason), patch('offline.lattice_training.pipeline.simulate_segment',
                    return_value=(None, {'completed': completed, 'reason': reason})) as simulate:
                _, snapshot, report = sample_successor(None, choices, solved, (0,0), None, None, np.random.default_rng(3), 3)
                self.assertIsNone(snapshot)
                self.assertEqual(simulate.call_count, 1)
                self.assertEqual(report['reason'], reason)

    def test_illegal_endpoint_retry_exhaustion_is_explicit(self):
        from offline.lattice_training.pipeline import sample_successor
        choices = [{'candidate_id': str(i)} for i in range(3)]
        with patch('offline.lattice_training.pipeline.simulate_segment',
                   return_value=(None, {'completed': True, 'reason': 'terminal_region_illegal'})) as simulate:
            _, snapshot, report = sample_successor(None, choices, {str(i): {} for i in range(3)},
                                                   (0,0), None, None, np.random.default_rng(3), 2)
            self.assertIsNone(snapshot)
            self.assertEqual(simulate.call_count, 2)
            self.assertEqual(report['reason'], 'illegal_terminal_resampling_exhausted')

    def test_collection_connects_full_terminal_states_and_keeps_base_road_splits(self):
        from offline.lattice_training.pipeline import PipelineSettings, collect_records
        from offline.lattice_training.cstar import SolverSettings
        settings = PipelineSettings(kinds=('straight',), train_seeds=(0,), validation_seeds=(100,), test_seeds=(200,),
            starts_per_road=1, planner_profiles=('standard',), decisions=2, trials=1)
        with tempfile.TemporaryDirectory() as directory, patch.object(ReturnMLP, 'predict',
                side_effect=AssertionError('Policy must not select training transitions')):
            path = Path(directory)/'records.jsonl'
            report = collect_records(path, settings, CStarTrackingSettings(),
                SolverSettings(population=4, iterations=1, max_evaluations=4))
            records = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
            self.assertEqual(report['base_roads'], 3)
            groups, splits = {}, {}
            for record in records:
                self.assertEqual(splits.setdefault(record['scene_id'], record['split']), record['split'])
                groups.setdefault(record['episode']['id'], []).append(record)
            self.assertEqual(len(groups), 3)
            self.assertTrue(all(len(group)==2 for group in groups.values()))
            for first, second in groups.values():
                terminal = first['successor']['attempts'][-1]
                self.assertTrue(first['successor']['accepted'])
                self.assertGreater(terminal['steps'], 60)
                self.assertAlmostEqual(terminal['spatial_span_m'],2.0)
                self.assertAlmostEqual(second['observation']['timestamp_s']-first['observation']['timestamp_s'],terminal['planned_duration_s'])
                self.assertEqual(second['episode']['start_state'], terminal['end_state'])
                self.assertEqual(second['episode']['initial_control'], terminal['final_control'])
                self.assertEqual(second['episode']['step'],terminal['steps'])
                self.assertEqual(second['episode']['segment'], 1)
                self.assertGreater(second['episode']['progress_m'], first['episode']['progress_m'])
            self.assertTrue(any(
                record['episode']['selected_candidate_id'] != max(record['candidates'],
                    key=lambda c: np.mean(c['label']['returns']))['candidate']['candidate_id']
                for record in records))

    def test_until_road_end_collects_multiple_segments_and_reports_real_completion(self):
        from offline.lattice_training.pipeline import PipelineSettings, collect_records
        from offline.lattice_training.cstar import SolverSettings
        settings = PipelineSettings(kinds=('straight',), train_seeds=(0,), validation_seeds=(100,), test_seeds=(200,),
            starts_per_road=1, planner_profiles=('standard',), decisions=1, trials=1,
            until_road_end=True, max_segments=3)
        def goal(env):
            return float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])+3.7
        with tempfile.TemporaryDirectory() as directory, patch(
                'offline.lattice_training.pipeline.road_goal_progress', side_effect=goal):
            report = collect_records(Path(directory)/'records.jsonl', settings, CStarTrackingSettings(),
                SolverSettings(population=4, iterations=1, max_evaluations=4))
            self.assertTrue(report['collection_complete'])
            self.assertEqual(len(report['scenes']), 3)
            for scene in report['scenes']:
                self.assertEqual(scene['decisions'], 2)
                self.assertTrue(scene['road_completed'])
                self.assertEqual(scene['terminal_behavior'], 'road_goal_reached')
                self.assertGreaterEqual(scene['final_progress_m'], scene['goal_progress_m'])
                self.assertEqual(scene['faults'], [])
            self.assertEqual(report['trials'], report['candidates'])
            self.assertGreater(report['timing_totals']['solve_s'], 0)
            self.assertGreater(report['timing_totals']['trials_s'], 0)
        with tempfile.TemporaryDirectory() as directory:
            report = collect_records(Path(directory)/'records.jsonl', replace(settings,max_segments=1),
                CStarTrackingSettings(), SolverSettings(population=4, iterations=1, max_evaluations=4))
            self.assertTrue(all(not scene['road_completed'] for scene in report['scenes']))
            self.assertTrue(all(scene['faults']==['maximum_segments_reached'] for scene in report['scenes']))

    def test_collection_entry_refuses_to_overwrite_an_existing_frozen_dataset(self):
        from offline.lattice_training.pipeline import collect_dataset
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory)/'decisions.jsonl').write_text('preserved', encoding='utf-8')
            with self.assertRaises(FileExistsError):
                collect_dataset(directory)
            self.assertEqual((Path(directory)/'decisions.jsonl').read_text(encoding='utf-8'), 'preserved')

    def test_initial_draws_are_physically_legal_without_querying_the_planner(self):
        from offline.lattice_training.pipeline import PipelineSettings,initialize_episode
        settings=PipelineSettings(starts_per_road=2)
        with patch('offline.lattice_training.pipeline.candidates_at',side_effect=AssertionError('No planner acceptance filter')):
            for seed in (0,1,2,3,100,101,200,201,7,13,17,23):
                for initial in range(2):
                    env,_,info=initialize_episode('randomized',seed,initial,settings,CStarTrackingSettings(),None)
                    with env:
                        self.assertTrue(info['initial_region']['legal'],(seed,initial,info))
                        self.assertEqual(info['initial_state_draws'],1)
                        self.assertFalse(info['filtered_by_planner'])

    def test_candidate_query_does_not_commit_an_unexecuted_rule_path_or_reset_origin_guard(self):
        from offline.lattice_training.pipeline import PipelineSettings,initialize_episode,make_planner,candidates_at,commit_candidate
        env,obs,_=initialize_episode('straight',0,0,PipelineSettings(),CStarTrackingSettings(),None)
        with env:
            planner=make_planner()
            ref,diag=candidates_at(planner,obs)
            self.assertTrue(ref['valid'])
            self.assertIsNone(planner.previous)
            self.assertFalse(planner.stop_latched)
            chosen=diag['candidate_references'][-1]
            commit_candidate(planner,chosen)
            previous=deepcopy(planner.previous)
            candidates_at(planner,obs)
            self.assertEqual(planner.previous,previous)
            self.assertFalse(planner.stop_latched)
            planner.observed_origin_seen=True
            if obs['estimation']['road']['status']=='near_field_unobserved':
                ref,_=candidates_at(planner,obs)
                self.assertFalse(ref['valid'])
                self.assertEqual(ref['reason'],'invalid_estimates_or_boundaries')
                self.assertTrue(planner.observed_origin_seen)

    def test_coverage_counts_chains_separately_and_unions_actual_progress(self):
        from offline.lattice_training.pipeline import collection_coverage
        scenes=[]
        for index,(start,end) in enumerate(((1.,3.),(1.,3.),(7.,9.),(9.,10.))):
            scenes.append({'scene_id':'road','split':'train','episode_id':str(index),'decisions':1,
                'road_completed':False,'road_length_m':12.,'initial_progress_m':start,
                'segments':[{'accepted':True,'attempts':[{'trace':[{'progress_m':end}]}]}]})
        coverage=collection_coverage({'base_road_splits':{'road':'train'},'scenes':scenes},require_road_end=True)
        road=coverage['roads'][0]
        self.assertEqual(road['sequences'],4)
        self.assertEqual(road['decisions'],4)
        self.assertEqual(road['progress_intervals_m'],[[1.,3.],[7.,10.]])
        self.assertAlmostEqual(road['visited_progress_m'],5.)
        self.assertFalse(coverage['coverage_complete'])
        self.assertEqual(coverage['missing_goal_roads'],['road'])

    def test_incomplete_full_road_collection_keeps_valid_candidate_labels_trainable(self):
        from offline.lattice_training.pipeline import PipelineSettings,collect_dataset
        def fake_collect(path,*args,**kwargs):
            Path(path).write_text('\n'.join(json.dumps(r) for r in decision_records()),encoding='utf-8')
            scenes=[{'scene_id':split+'-synthetic-scene','split':split,'episode_id':split,
                'decisions':1,'road_completed':False,'road_length_m':10.,'initial_progress_m':1.,'segments':[]}
                for split in ('train','validation','test')]
            return {'scenes':scenes,'base_road_splits':{s['scene_id']:s['split'] for s in scenes},
                'base_roads':3,'decisions':3,'candidates':9,'trials':18,'successful_trials':18,'wall_s':0.,'timing_totals':{}}
        with tempfile.TemporaryDirectory() as directory,patch('offline.lattice_training.pipeline.collect_records',side_effect=fake_collect),patch(
                'offline.lattice_training.pipeline.plot_road_coverage'),patch('offline.lattice_training.pipeline.train_model') as train:
            collect_dataset(directory,settings=PipelineSettings(until_road_end=True))
            train.assert_not_called()
            summary=json.loads((Path(directory)/'summary.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['status'],'ready_with_incomplete_road_coverage')
            self.assertTrue(summary['training_eligible'])
            metadata=json.loads((Path(directory)/'dataset'/'metadata.json').read_text(encoding='utf-8'))
            self.assertTrue(metadata['training_eligible'])
            train_model(Path(directory)/'dataset',Path(directory)/'model.json',epochs=2)
            self.assertTrue((Path(directory)/'model.json').exists())

    def test_internal_legacy_labels_are_rejected_instead_of_reusing_incomplete_safety_checks(self):
        from offline.lattice_training.dataset import validate_label_source
        with self.assertRaisesRegex(ValueError,'finite-road'):
            validate_label_source({'name':'internal_cstar_native_env','version':'5'})
        validate_label_source({'name':'internal_cstar_native_env','version':'6'})

    def test_coverage_figure_uses_intermediate_trace_points_and_chain_labels(self):
        from offline.lattice_training.pipeline import make_environment
        from offline.lattice_training.plots import plot_road_coverage
        from matplotlib.axes import Axes
        with make_environment('straight',0,max_steps=10) as env:
            configuration=env.configuration()
        scene={'scene_id':'road','split':'train','episode_id':'straight:0:initial:0:lattice:standard',
            'initial_pose':[1.,0.,0.],'initial_progress_m':1.,'road_length_m':10.,'decisions':1,
            'road_completed':False,'lattice_profile':'standard','terminal_behavior':'planning_failure','faults':['geometry'],
            'environment_configuration':configuration,'segments':[{'accepted':True,'attempts':[{
                'start_state':[1.,0.,0.,0.,0.,0.,0.],'trace':[
                    {'x_m':1.5,'y_m':.2,'progress_m':1.5},{'x_m':2.,'y_m':0.,'progress_m':2.}]}]}]}
        lines=[];original=Axes.plot
        def capture(axis,*args,**kwargs):
            if kwargs.get('label','').startswith('I1/standard'):
                lines.append((list(args[0]),list(args[1]),kwargs['label']))
            return original(axis,*args,**kwargs)
        with tempfile.TemporaryDirectory() as directory,patch.object(Axes,'plot',capture):
            plot_road_coverage({'scenes':[scene],'base_road_splits':{'road':'train'},'pipeline_settings':{}},directory)
            self.assertTrue((Path(directory)/'road_coverage.png').exists())
        self.assertEqual(lines[0][0],[1.,1.5,2.])
        self.assertEqual(lines[0][1],[0.,.2,0.])
        self.assertIn('1 groups, FAIL',lines[0][2])


class NativeCStarIntegration(unittest.TestCase):
    def setUp(self):
        from offline.lattice_training.pipeline import make_environment, make_planner, candidates_at
        from offline.lattice_training.cstar import CStarSolver, SolverSettings
        self.env = make_environment("straight", 0, max_steps=5000)
        self.observation, _ = self.env.reset(seed=0)
        self.observation, *_ = self.env.step((0, 0))
        self.reference, diag = candidates_at(make_planner(), self.observation)
        self.proposal = diag["candidate_references"][0]
        self.solver = CStarSolver(settings=SolverSettings(population=4, iterations=1, max_evaluations=8))

    def tearDown(self):
        self.env.close()

    def test_native_prediction_matches_full_env_including_braking_history(self):
        from offline.lattice_training.cstar import NativePrediction, frozen_state
        controls = np.vstack((np.tile([0.002, 0.03], (5, 1)),
                              np.tile([-0.08, 0], (35, 1)), np.tile([0, 0], (3, 1)),
                              np.tile([-0.01, 0], (7, 1))))
        predictor = NativePrediction(self.env)
        predicted, _ = predictor.predict(controls)
        actual = [frozen_state(self.env._backend.car._state_values(), predictor.anchor, predictor.initial_car)]
        for action in controls:
            self.env.step(action)
            actual.append(frozen_state(self.env._backend.car._state_values(), predictor.anchor, self.env._backend.car))
        np.testing.assert_allclose(predicted, actual, atol=1e-10, rtol=0)

    def test_solver_improves_loss_and_actual_replay_without_mutating_parent(self):
        from offline.lattice_training.cstar import NativePrediction, execute_trial, performance_profile
        original = self.env._backend.car._state_values()
        latch = self.env._backend.car._latched_direction
        _, target = sample_reference(self.proposal)
        baseline, _ = NativePrediction(self.env).predict(np.zeros((len(target)-1, 2)))
        baseline_loss, _ = tracking_loss(target, baseline, np.zeros((len(target)-1, 2)), [0, 0])
        solution = self.solver.solve(self.env, self.proposal, seed=7)
        self.assertEqual(solution["status"], "approximate")
        self.assertLess(solution["loss"], baseline_loss)
        self.assertFalse(solution["diagnostics"]["global_optimum_proven"])
        trial = execute_trial(self.env, self.proposal, solution, (0, 0), self.solver.objective,
                              self.solver.settings, trial_id="nominal", profile=performance_profile(0, nominal=True))
        self.assertTrue(trial["constraints_satisfied"], trial.get("reason"))
        self.assertEqual(len(trial["states"]),len(solution["states"]))
        self.assertLessEqual(trial["replay_max_state_error"], 1e-10)
        self.assertAlmostEqual(trial["actual_loss"], solution["loss"], places=10)
        np.testing.assert_array_equal(self.env._backend.car._state_values(), original)
        self.assertEqual(self.env._backend.car._latched_direction, latch)
        self.assertEqual(self.env.elapsed_time_s, 0.05)

    def test_same_state_and_seed_give_same_controls(self):
        first = self.solver.solve(self.env, self.proposal, seed=12)
        second = self.solver.solve(self.env, self.proposal, seed=12)
        np.testing.assert_array_equal(first["controls"], second["controls"])
        np.testing.assert_array_equal(first["states"], second["states"])

    def test_full_segment_snapshot_retains_vehicle_and_sensor_history(self):
        from offline.lattice_training.pipeline import simulate_segment
        from offline.lattice_training.dataset import json_safe
        solution = self.solver.solve(self.env, self.proposal, seed=12)
        original = list(self.env._backend.car._state_values())
        snapshot, transition = simulate_segment(self.env, self.proposal, solution, (0,0),
                                                self.solver.objective, self.solver.settings)
        self.assertIsNotNone(snapshot)
        self.assertEqual(transition['steps'],len(solution['controls']))
        np.testing.assert_array_equal(self.env._backend.car._state_values(), original)
        with self.env.fork() as expected:
            for action in solution['controls']:
                expected.step(action)
            self.env.restore(snapshot)
            np.testing.assert_array_equal(self.env._backend.car._state_values(), expected._backend.car._state_values())
            self.assertEqual(self.env._backend.car._latched_direction, expected._backend.car._latched_direction)
            self.assertEqual(json_safe(self.env._frame), json_safe(expected._frame))
            obs, *_ = self.env.step((0,0))
            expected_obs, *_ = expected.step((0,0))
            self.assertEqual(json_safe(obs), json_safe(expected_obs))

    def test_terminal_region_checks_space_instead_of_tracking_loss(self):
        from offline.lattice_training.pipeline import terminal_region
        original = self.env._backend.car._state_values()
        self.env._backend.car._px = 1.0
        self.env._backend.car._vy = 2.0
        self.assertTrue(terminal_region(self.env, self.solver.settings)['legal'])
        self.env._backend.car._py = 0.8
        self.assertFalse(terminal_region(self.env, self.solver.settings)['legal'])
        self.env._backend.car._py = original[1]
        self.env._backend.car._px = 11.0
        self.assertFalse(terminal_region(self.env, self.solver.settings)['legal'])
        self.env._backend.car._px = original[0]
        self.env._backend.car._vy = original[4]

    def test_actual_wheel_rate_bound_and_negative_drive_braking(self):
        raw = self.env._backend.car._state_values()
        self.env.step((-0.1, 1.0))
        following = self.env._backend.car._state_values()
        self.assertLess(following[3], raw[3])
        self.assertLessEqual(abs(following[6]-raw[6]), math.pi/2*self.env.dt_s+1e-10)
        for _ in range(10):
            self.env.step((-0.1, 0.0))
        self.assertGreaterEqual(self.env._backend.car._vx, -1e-10)

    def test_full_body_obstacle_and_road_constraint_rejection(self):
        from offline.lattice_training.pipeline import make_environment
        from offline.lattice_training.cstar import NativePrediction
        with make_environment("blocked", 0, max_steps=80) as blocked:
            blocked.reset(seed=0)
            checker = NativePrediction(blocked)
            raw = list(blocked._backend.car._state_values())
            raw[:2] = [0.9, 0.0]
            self.assertLess(checker.margins(raw)["obstacle_contact_margin"], 0)
            raw[:2] = [0.1, 0.6]
            self.assertLess(checker.margins(raw)["road_clearance_m"], 0)
            blocked._backend.car._px, blocked._backend.car._py = 0.9, 0.0
            solution = self.solver.solve(blocked, self.proposal, seed=1)
            self.assertEqual(solution["status"], "failed")
            self.assertEqual(solution["reason"], "illegal_initial_state")
            self.assertEqual(solution['diagnostics']['evaluations'],0)

    def test_native_solver_rejects_body_outside_finite_start_and_end(self):
        from offline.lattice_training.cstar import NativePrediction
        from offline.lattice_training.pipeline import simulate_segment
        for progress in (.12,9.9):
            self.env._backend.car._px=progress
            margins=NativePrediction(self.env,self.solver.settings).margins(self.env._backend.car._state_values())
            self.assertLess(margins['road_longitudinal_clearance_m'],0)
            solution=self.solver.solve(self.env,self.proposal)
            self.assertEqual(solution['reason'],'illegal_initial_state')
            self.assertEqual(solution['diagnostics']['evaluations'],0)
            steps=len(sample_reference(self.proposal)[0])-1
            snapshot,transition=simulate_segment(self.env,self.proposal,
                {'status':'approximate','controls':np.zeros((steps,2))},(0,0),self.solver.objective,self.solver.settings)
            self.assertIsNone(snapshot)
            self.assertEqual(transition['reason'],'illegal_initial_state')
            self.assertEqual(transition['steps'],0)

    def test_road_evaluation_does_not_declare_illegal_initial_body_a_completed_road(self):
        from offline.lattice_training.evaluation import road_environment,RoadEvaluation
        from offline.lattice_training.roads import generate_random_road_spec,road_length
        spec=generate_random_road_spec(0)
        with road_environment(spec,progress=road_length(spec)-.05,speed=0.) as env:
            result=RoadEvaluation(env).run()
        self.assertFalse(result['road_completed'])
        self.assertEqual(result['fault'],'illegal_initial_state')
        self.assertEqual(result['steps'],0)

    def test_performance_delay_uses_requested_history(self):
        from offline.lattice_training.cstar import PerformanceActuator, apply_performance
        profile = {"drive_gain": 1.0, "steering_gain": 1.0, "steering_bias": 0.0, "delay_steps": 1}
        actuator = PerformanceActuator(profile, (0.0, 0.0))
        first = actuator.apply((0.1, 0.2))
        second = actuator.apply((0.3, 0.4))
        np.testing.assert_allclose(first, [0, 0])
        np.testing.assert_allclose(second, [.1, .2])
        np.testing.assert_allclose(apply_performance([[.1, .2], [.3, .4]], (0, 0), profile), [first, second])

    def test_budget_exhaustion_does_not_claim_infeasibility_or_optimality(self):
        from offline.lattice_training.cstar import CStarSolver, SolverSettings
        solver = CStarSolver(settings=SolverSettings(population=4, iterations=1, max_evaluations=4,
                                                      max_wall_s=1e-9))
        solution = solver.solve(self.env, self.proposal)
        self.assertEqual(solution["status"], "failed")
        self.assertEqual(solution["diagnostics"]["stop_reason"], "wall_budget")
        self.assertFalse(solution["diagnostics"]["global_optimum_proven"])

    def test_committed_stop_uses_full_reference_without_lattice_restart(self):
        from offline.lattice_training.pipeline import closed_loop_episode
        result = closed_loop_episode("obstacle", 200, None, self.solver.objective, self.solver.settings, 60)
        self.assertGreaterEqual(result["steps"],60)
        self.assertIsNone(result["fault"])
        self.assertTrue(result["planned_stop"])
        self.assertEqual(result["completion"], "planned_stop_reference_complete")
        self.assertIsInstance(result["stop_completed"], bool)

    def test_full_pipeline_cli_uses_real_env_and_generates_model_and_evaluation(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable, "-B", "-m", "offline.lattice_training", "run",
                "--output", directory, "--kinds", "straight", "--train-seeds", "0",
                "--validation-seeds", "100", "--test-seeds", "200", "--decisions", "1",
                "--starts-per-road", "1", "--planner-profiles", "standard",
                "--trials", "1", "--population", "4", "--iterations", "1", "--max-evaluations", "8",
                "--evaluation-steps", "2", "--epochs", "15"],
                cwd=ROOT, capture_output=True, text=True, timeout=300)
            self.assertEqual(result.returncode, 0, result.stderr+"\n"+result.stdout)
            root = Path(directory)
            for name in ("dataset/data.npz", "decisions.jsonl", "model.json", "model.training.json",
                         "closed_loop.json", "evaluation.json", "summary.json", "training.png", "closed_loop.png",
                         "road_coverage.png"):
                self.assertTrue((root / name).is_file(), name)
            data, metadata = load_dataset(root / "dataset")
            self.assertGreater(len(data["y"]), 0)
            self.assertEqual(metadata["label_source"]["name"], "internal_cstar_native_env")
            records = [json.loads(line) for line in (root / "decisions.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertTrue(all(trial["constraints_satisfied"] for record in records
                                for entry in record["candidates"] for trial in entry["cstar_trials"]))
            self.assertEqual(metadata["objective"]["horizon_s"], 3.0)
            model = ReturnMLP.load(root / "model.json")
            score_decisions(model, root / "decisions.jsonl", root / "scores.json")
            closed = json.loads((root / "closed_loop.json").read_text(encoding="utf-8"))
            self.assertEqual(len(closed["episodes"]), 4)
            self.assertEqual(closed["episodes"][0]["environment_configuration"],
                             closed["episodes"][1]["environment_configuration"])
            self.assertFalse(closed["runtime_qualified"])


if __name__ == "__main__":
    unittest.main()
