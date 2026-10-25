"""Exact frozen inference, asynchronous memory and isolated single-file delivery."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from unittest.mock import Mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mvp21_policy.learned import FrozenPathPolicy, LearnedPolicySettings, PolicyExecutionMemory
from mvp21_policy.lattice import ShadowLatticePlanner, ShadowSettings
from mvp21_policy.candidates import from_lattice
from offline.lattice_training.features import encode_candidate
from offline.lattice_training.model import SpatialPathPolicy
from tools.bundle_python import bundle

CHECKPOINT = ROOT/'offline/lattice_training/artifacts/2026-10-19-multistart-policy/model.json'
RESOURCE = 'mvp21_policy/models/path_policy.json'


def job(identifier=1):
    state = dict(speed_mps=.2, yaw_rate_rps=.1, speed_valid=True, yaw_rate_valid=True,
                 timestamp_s=10., frame_id='base_link')
    road = dict(centerline_xy=[(0., 0.), (1., .1), (2., .3)],
        left_boundary_xy=[(0., .7), (1., .8), (2., 1.)],
        right_boundary_xy=[(0., -.7), (1., -.6), (2., -.4)],
        boundary_source=dict(left='observed', right='observed'))
    return dict(settings=vars(ShadowSettings(enabled=True, sample_count=12)),
        learned_policy=vars(LearnedPolicySettings(enabled=True)), policy_cruise_speed_mps=.5,
        policy_context=dict(revision=0, hidden=[0.]*16),
        policy_observation=dict(frame_id='base_link', timestamp_s=10.,
            estimation=dict(state=deepcopy(state), road=deepcopy(road),
                obstacles=dict(available=True, points_xyz_m=[[1., .4, 0.]]))),
        road=road, state=state, generation=1, job_id=identifier, stamp_s=10.,
        shadow_pose=[0., 0., 0.], shadow_odometry_epoch=0, preview_m=.35,
        submitted_monotonic_s=1., planner_control_enabled=True)


class LearnedPolicyChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.settings = LearnedPolicySettings(enabled=True)
        cls.model = FrozenPathPolicy(cls.settings, .5)
        cls.original = SpatialPathPolicy.load(CHECKPOINT)

    def result(self, identifier=1, context=None):
        request = job(identifier)
        if context is not None:
            request['policy_context'] = context
        result = ShadowLatticePlanner(ShadowSettings(**request['settings'])).plan(request)
        result['job_id'], result['generation'] = identifier, 1
        return self.model.select(result, request['policy_observation'], request['policy_context'])

    def test_checkpoint_is_unchanged_and_scores_and_hidden_are_exact(self):
        self.assertEqual((ROOT/RESOURCE).read_bytes(), CHECKPOINT.read_bytes())
        request = job()
        native = ShadowLatticePlanner(ShadowSettings(**request['settings'])).plan(request)
        previous = np.linspace(-.3, .4, 16)
        for candidate in native['candidates']:
            encoded = encode_candidate(request['policy_observation'],
                dict(reference=dict(frenet_path=dict(type='SAMPLED_FRENET',
                    candidate=from_lattice(candidate, native)))))
            expected, hidden = self.original.step(encoded, previous)
            actual, next_hidden = self.model.step(encoded, previous)
            self.assertEqual(actual, expected)
            np.testing.assert_array_equal(next_hidden, hidden)

    def test_ranking_uses_same_prior_for_every_path_and_ignores_native_cost(self):
        request = job();native = ShadowLatticePlanner(ShadowSettings(**request['settings'])).plan(request)
        priors = []
        def scorer(record, hidden):
            priors.append(hidden.copy())
            return float(record['candidate_path']['samples'][-1][2]), [0.2]*16
        with patch.object(self.model, 'step', side_effect=scorer):
            first = self.model.select(deepcopy(native), request['policy_observation'], request['policy_context'])
            for c in native['candidates']:c['cost'] = 1e9-c['id']
            native['candidates'].reverse()
            second = self.model.select(native, request['policy_observation'], request['policy_context'])
        self.assertEqual(first['selected']['id'], second['selected']['id'])
        self.assertTrue(all(np.array_equal(p, np.zeros(16)) for p in priors))
        self.assertEqual(request['policy_context'], dict(revision=0, hidden=[0.]*16))

    def test_collision_candidates_never_scored_and_inference_error_has_no_native_pick(self):
        request = job();native = ShadowLatticePlanner(ShadowSettings(**request['settings'])).plan(request)
        keep = native['candidates'][-1]['id']
        for c in native['candidates']:c['collision'] = None if c['id'] == keep else {'side': 'left'}
        with patch.object(self.model, 'step', return_value=(.5, [0.]*16)) as scorer:
            selected = self.model.select(deepcopy(native), request['policy_observation'], request['policy_context'])
            self.assertEqual(scorer.call_count, 1)
        self.assertEqual(selected['selected']['id'], keep)
        with patch.object(self.model, 'step', side_effect=ValueError('bad input')):
            failed = self.model.select(native, request['policy_observation'], request['policy_context'])
        self.assertEqual(failed['status'], 'POLICY_ERROR')
        self.assertIsNone(failed['selected'])

    def test_checksum_and_training_speed_are_required(self):
        with self.assertRaisesRegex(ValueError, 'cruise-speed'):
            FrozenPathPolicy(self.settings, .2)
        with self.assertRaisesRegex(ValueError, 'checksum'):
            FrozenPathPolicy(LearnedPolicySettings(enabled=True, model_sha256='0'*64), .5)

    def test_memory_only_advances_for_admitted_executed_job_once(self):
        memory = PolicyExecutionMemory(16, self.model.sha256)
        first = self.result()
        self.assertTrue(memory.validate_result(first))
        memory.stage(first, False)
        self.assertFalse(memory.commit(1))
        memory.stage(first, True)
        self.assertFalse(memory.commit(999))
        self.assertTrue(memory.commit(1))
        self.assertFalse(memory.commit(1))
        self.assertEqual(memory.revision, 1)
        self.assertEqual(memory.hidden, first['policy_selection']['hidden_selected'])
        late = self.result(2)
        self.assertFalse(memory.validate_result(late))
        self.assertEqual(memory.last_rejection['reason'], 'stale_policy_memory')
        fresh = self.result(3, memory.snapshot())
        self.assertTrue(memory.validate_result(fresh))
        memory.stage(fresh, True)
        self.assertTrue(memory.commit(3))
        self.assertEqual(memory.commits, 2)
        memory.reset()
        self.assertEqual(memory.snapshot(), dict(revision=0, hidden=[0.]*16))

    def test_malformed_valid_payload_cannot_execute_native_choice(self):
        memory = PolicyExecutionMemory(16, self.model.sha256)
        for field in ('model_sha256', 'hidden_before', 'hidden_selected', 'selected_id'):
            result = self.result()
            result['policy_selection'][field] = None
            self.assertTrue(memory.validate_result(result))
            self.assertEqual(result['status'], 'POLICY_ERROR')
            self.assertIsNone(result['selected'])

    def test_node_commits_after_publication_and_never_for_baseline_or_failed_tick(self):
        api = runpy.run_path(str(ROOT/'tests/test_control.py'))
        for mode in ('planner', 'baseline', 'expired', 'shadow', 'plan_hold'):
            with self.subTest(mode=mode):
                node = api['make_node'](simulated=False, mvp=True)
                api['feed'](node);api['start'](node)
                memory = PolicyExecutionMemory(16, self.model.sha256)
                result = self.result(99);memory.stage(result, True)
                node.learned_policy_memory = memory
                def calculate(*args):
                    node.control_diagnostics = dict(mpc_reference_executed=mode=='planner')
                    node._trajectory_reference = dict(
                        planner_version='lattice_execution' if mode != 'baseline' else 'road_centerline',
                        trajectory_id=99)
                    if mode == 'expired':node.test_clock.advance(.6)
                    return .3, 0., None, None, None
                node.calculate_policy_actions = calculate
                with patch.object(memory, 'commit', wraps=memory.commit) as commit:
                    node.run_policy_step()
                    self.assertEqual(commit.call_count, int(mode == 'planner'))
                if mode == 'planner':
                    self.assertEqual(memory.revision, 1)
                    self.assertGreater(node.action_publisher.messages[-1].drive, 0)
                else:self.assertEqual(memory.revision, 0)

    def test_single_file_scores_without_repository_or_training_imports(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'policy_node.py'
            manifest = bundle(ROOT/'scripts/mvp21_policy_node.py', path, root=ROOT, resources=[RESOURCE])
            self.assertIn(RESOURCE, manifest['resources'])
            self.assertNotIn('offline.lattice_training.model', manifest['modules'])
            self.assertNotIn('offline.lattice_training.dataset', manifest['modules'])
            result = subprocess.run([sys.executable, '-I', '-B', str(path), '--shadow-worker'],
                input=json.dumps(job())+'\n', cwd=temp, capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stderr)
            actual = json.loads(result.stdout)
            expected = self.result()
            self.assertEqual(actual['status'], 'VALID', actual)
            self.assertEqual(actual['selected'], json.loads(json.dumps(expected['selected'])))
            self.assertEqual(actual['policy_selection']['scores'], expected['policy_selection']['scores'])
            self.assertEqual(actual['policy_selection']['hidden_selected'], expected['policy_selection']['hidden_selected'])

    def test_stale_recurrent_job_never_replaces_active_execution_path(self):
        api = runpy.run_path(str(ROOT/'tests/test_control.py'))
        node = api['make_node'](simulated=False, mvp=True)
        api['feed'](node);api['start'](node)
        memory = PolicyExecutionMemory(16, self.model.sha256)
        first = self.result();memory.stage(first, True);memory.commit(1)
        node.learned_policy_memory = memory
        node.shadow_generation = 1
        node.shadow_worker = SimpleNamespace(poll=lambda:self.result(2))
        node.planner_execution = SimpleNamespace(accept=Mock())
        node._shadow_poll()
        node.planner_execution.accept.assert_not_called()
        self.assertEqual(memory.revision, 1)
        self.assertEqual(memory.last_rejection['reason'], 'stale_policy_memory')


if __name__ == '__main__':
    unittest.main()
