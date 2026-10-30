"""RNN ranking and task-owned references must not cross planner handoffs."""
import json
from pathlib import Path
import runpy
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mvp21_policy.avoidance import LidarAvoidanceGuard, LidarAvoidanceSettings
from mvp21_policy.learned import PolicyExecutionMemory
from tests.test_lidar_maneuver import tick
from tests.test_learned_policy import job
from tools.package_lidar_avoidance import profile


class RNNManeuverChecks(unittest.TestCase):
    def worker(self, request):
        process = subprocess.run([sys.executable, '-B', '-m', 'mvp21_policy.worker'],
            input=json.dumps(request)+'\n', text=True, capture_output=True, cwd=ROOT, timeout=20)
        self.assertEqual(process.returncode, 0, process.stderr)
        return json.loads(process.stdout)

    def test_normal_worker_uses_rnn_with_task_guard_enabled(self):
        request = job()
        guard = LidarAvoidanceGuard(LidarAvoidanceSettings(enabled=True, maneuver_enabled=True))
        normal = tick(guard, points=[])
        request['lidar_avoidance'] = normal['lidar_avoidance']
        result = self.worker(request)
        self.assertEqual(result['status'], 'VALID', result)
        self.assertEqual(result['policy_selection']['status'], 'SELECTED')
        self.assertIsNone(result['avoidance']['maneuver_task_id'])
        self.assertTrue(all(score['candidate_id'] in {c['id'] for c in result['candidates']
            if c['collision'] is None} for score in result['policy_selection']['scores']))

    def test_task_worker_keeps_task_geometry_even_if_rnn_is_requested(self):
        guard = LidarAvoidanceGuard(LidarAvoidanceSettings(enabled=True, maneuver_enabled=True))
        request = tick(guard)
        self.assertTrue(guard.maneuver.active, guard.diagnostics)
        # Malformed RNN settings would fail if the worker tried to rank the task.
        request.update(learned_policy=dict(enabled=True, model_sha256='bad'),
                       policy_cruise_speed_mps=.5)
        result = self.worker(request)
        self.assertEqual(result['status'], 'VALID', result)
        self.assertEqual(result['selected']['family'], 'maneuver_shift')
        self.assertNotIn('policy_selection', result)

    def node_api(self):
        return runpy.run_path(str(ROOT/'tests/test_control.py'))

    def test_each_task_handoff_resets_memory_and_fences_old_worker_generation(self):
        api = self.node_api();node = api['make_node'](simulated=False, mvp=True)
        node.learned_policy_memory = PolicyExecutionMemory(16, 'a'*64)
        node.learned_policy_memory.hidden = [.2]*16
        node.learned_policy_memory.revision = 3
        node.learned_policy_memory.pending = dict(job_id=9, revision=3, hidden=[.4]*16)
        node.shadow_generation = 10
        node.shadow_previous_reference = {'points': [[0.,0.],[1.,0.]]}
        node.planner_execution = SimpleNamespace(clear=Mock(), accept=Mock())
        task = SimpleNamespace(active=True)
        node.lidar_avoidance_guard = SimpleNamespace(maneuver=task)
        node._sync_learned_maneuver()
        self.assertEqual(node.shadow_generation, 11)
        self.assertEqual(node.learned_policy_memory.snapshot(), dict(revision=0, hidden=[0.]*16))
        self.assertIsNone(node.learned_policy_memory.pending)
        self.assertIsNone(node.shadow_previous_reference)
        node._sync_learned_maneuver()
        self.assertEqual(node.shadow_generation, 11)  # SHIFT/PASSING is the same owner.
        task.active = False
        node._sync_learned_maneuver()
        self.assertEqual(node.shadow_generation, 12)
        node.fsm_state = api['ns']['FSM_STATE_PUBLISHING_POLICY_ACTION']
        node.shadow_worker = SimpleNamespace(poll=lambda:dict(status='VALID', generation=10))
        node._shadow_poll()
        node.planner_execution.accept.assert_not_called()

    def test_maneuver_action_never_commits_rnn_memory(self):
        api = self.node_api();node = api['make_node'](simulated=False, mvp=True)
        api['feed'](node);api['start'](node)
        memory = PolicyExecutionMemory(16, 'a'*64)
        memory.pending = dict(job_id=99, revision=0, hidden=[.2]*16)
        node.learned_policy_memory = memory
        def calculate(*args):
            node._trajectory_reference = dict(planner_version='lattice_execution',
                trajectory_id=99, maneuver_task_id=7)
            return .3, 0., None, None, None
        node.calculate_policy_actions = calculate
        node.run_policy_step()
        self.assertEqual(memory.commits, 0)
        self.assertGreater(node.action_publisher.messages[-1].drive, 0.)

    def test_profile_preserves_b_control_and_body_with_original_rnn_condition(self):
        import yaml
        original = subprocess.check_output(['git', 'show', 'f7ec808:config/ai4r_policy.yaml'], cwd=ROOT)
        before = yaml.safe_load(original)['/**/ai4r_policy']['ros__parameters']
        for proximity in (True, False):
            actual = profile(proximity_stop=proximity)['/**/ai4r_policy']['ros__parameters']
            self.assertEqual(actual['control'], before['control'])
            self.assertEqual(actual['planning']['cruise_speed_mps'], .5)
            self.assertTrue(actual['learned_policy']['enabled'])
            self.assertEqual(actual['estimation']['colour_assignment'], 'auto')
            self.assertTrue(actual['obstacle_stop']['enabled'])
            self.assertEqual(actual['obstacle_stop']['proximity_stop_enabled'], proximity)
            self.assertEqual(actual['lidar_avoidance']['width_m'], .3)
            self.assertTrue(actual['lidar_avoidance']['maneuver_enabled'])


if __name__ == '__main__':
    unittest.main()
