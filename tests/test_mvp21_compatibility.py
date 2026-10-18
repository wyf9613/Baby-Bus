"""Paired behavior against the immutable real-car MVP2.1 source, without ROS."""
import ast
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from numbers import Real
from pathlib import Path
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
try:
    from ament_index_python.packages import get_package_prefix
except ImportError:
    PACKAGE_ROOT = ROOT
else:
    PACKAGE_ROOT = Path(get_package_prefix('ai4r_policy'))/'lib/ai4r_policy'
sys.path.insert(0, str(PACKAGE_ROOT))
from mvp21_policy import api
from mvp21_policy.execution import PlannerExecutionManager
from mvp21_policy.lattice import ShadowLatticePlanner, ShadowSettings


def upstream():
    # Independent source is loaded only by this comparator, never by runtime.
    source = ROOT/'offline/mvp21_sim/upstream/3652e9b/scripts/policy_node.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    names = {'ShadowSettings', 'ShadowFrame', 'ShadowLatticePlanner', 'PlannerExecutionManager',
        'CenterlinePlanner', 'PlanningSettings', 'ControlSettings', 'ControlPID', 'PolicyController',
        'PolicyStopRequest', 'finite_number', 'wrap_angle', 'control_path_geometry'}
    definitions = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
        and (n.name in names or n.name.startswith(('_shadow_', '_path_', '_control_')))
        and n.name != '_shadow_worker_entry']
    ns = {**vars(api), 'dataclass': dataclass, 'math': math, 'time': time, 'Real': Real, 'deepcopy': deepcopy}
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(source), 'exec'), ns)
    return ns


BASE = upstream()


def packet(road=None):
    return dict(stamp_s=10., generation=1, job_id=1, preview_m=.35,
        state=dict(speed_mps=.2, speed_valid=True),
        road=road or dict(centerline_xy=[(0., 0.), (.5, 0.), (1., 0.)],
            left_boundary_xy=[(0., .5), (1., .5)], right_boundary_xy=[(0., -.5), (1., -.5)],
            boundary_source=dict(left='observed', right='observed'), lane_width_m=1.))


class MVP21CompatibilityChecks(unittest.TestCase):
    def compare_plan(self, job):
        values = job.get('settings') or dict(sample_count=12)
        baseline = BASE['ShadowLatticePlanner'](BASE['ShadowSettings'](**values)).plan(deepcopy(job))
        merged = ShadowLatticePlanner(ShadowSettings(**values)).plan(deepcopy(job))
        self.assertEqual(merged['status'], baseline['status'])
        self.assertEqual(merged.get('known_boundary_sides'), baseline.get('known_boundary_sides'))
        if baseline['status'] == 'VALID':
            equivalent = merged
            if merged.get('unknown_width_reference_scaled'):
                # User-authorized 0.45 -> 0.60 m guide adjustment changes paths
                # and costs here. Keep availability/provenance comparison at the
                # new default, and compare exact legacy geometry at scale 1.
                config=ShadowSettings(**{**values,'unknown_width_reference_scale':1.})
                equivalent=ShadowLatticePlanner(config).plan(deepcopy(job))
                current=ShadowSettings(**values)
                self.assertAlmostEqual(merged['reference_boundary_offset_m'],
                    current.search_half_width_m*current.unknown_width_reference_scale)
                self.assertEqual(merged['planning_input']['road'],baseline['planning_input']['road'])
            self.assertEqual(equivalent['selected']['id'], baseline['selected']['id'])
            self.assertEqual(equivalent['selected']['path'], baseline['selected']['path'])
            self.assertEqual(equivalent['selected']['cost'], baseline['selected']['cost'])
        return baseline, merged

    def test_20_recorded_inputs_keep_native_availability_selection_and_boundaries(self):
        fixture = json.loads((ROOT/'tests/fixtures/shadow_temporal_recorded_run.json').read_text())
        previous, valid = None, 0
        for entry in fixture['frames']:
            job = deepcopy(entry['job'])
            job['previous_reference'] = previous
            with self.subTest(job=job['job_id']):
                baseline, merged = self.compare_plan(job)
                valid += baseline['status'] == 'VALID'
                for side in ('left', 'right'):
                    self.assertEqual(merged['planning_input']['road'].get(side+'_boundary_xy'),
                                     baseline['planning_input']['road'].get(side+'_boundary_xy'))
                previous = (baseline.get('fusion') or {}).get('guide')
        self.assertEqual((len(fixture['frames']), valid), (20, 20))

    def test_partial_short_boundary_only_inferred_and_returning_inputs(self):
        original = packet()['road']
        cases = []
        for side in ('left', 'right'):
            for width in (None, 1.):
                road = deepcopy(original)
                road.update(centerline_xy=[], lane_width_m=width)
                road[('right' if side == 'left' else 'left')+'_boundary_xy'] = None
                cases.append(road)
        for points in ([(.6, .03)], [(.6, .03), (.62, .03)],
                       [(0., 0.), (0., .5), (-.3, .5), (-.3, -.3)],
                       [(0., 0.), (.4, 0.), (.7, .3), (.4, .7), (-.3, .7)]):
            road = deepcopy(original)
            road.update(centerline_xy=points, boundary_source={'left': 'inferred_width', 'right': 'predicted_history'})
            cases.append(road)
        cases.append(dict(centerline_xy=[], left_boundary_xy=[], right_boundary_xy=[]))
        for i, road in enumerate(cases):
            with self.subTest(case=i):
                self.compare_plan(packet(road))

    def test_execution_matches_upstream_with_current_observed_inferred_and_missing_boundaries(self):
        path = [(i*.05, 0., 0.) for i in range(41)]
        road = dict(frame_id='base_link', time_aligned=True,
            left_boundary_xy=[(-.4, .5), (2., .5)], right_boundary_xy=[(-.4, -.5), (2., -.5)],
            boundary_source={'left': 'observed', 'right': 'observed'})
        for source in ('observed', 'observed_with_history', 'inferred_width', 'predicted_history', 'absent'):
            for age in (.1, 3.2):
                baseline, merged = BASE['PlannerExecutionManager'](), PlannerExecutionManager(generation=1)
                value = dict(status='VALID', generation=1, job_id=1, input_stamp_s=10.,
                    measurement_stamp_s=9.8, execution_anchor=dict(pose=[0., 0., 0.], odometry_epoch=2),
                    selected=dict(path=path))
                baseline.accept(deepcopy(value)); merged.accept(deepcopy(value))
                now = 10.+age
                current = deepcopy(road)
                current.update(timestamp_s=now, left_boundary_xy=[(.6, -.1), (.6, .1)],
                    boundary_source={'left': source, 'right': 'absent'})
                state = dict(timestamp_s=now, frame_id='base_link', speed_valid=True,
                    yaw_rate_valid=True, speed_mps=.2, yaw_rate_rps=0.)
                args = (current, state, (.02, 0., 0.), 2, .35, .2, .05, .25)
                a = baseline.select(*args[:4], BASE['ShadowSettings'](), *args[4:])
                b = merged.select(*args[:4], ShadowSettings(), *args[4:])
                with self.subTest(source=source, age=age):
                    self.assertEqual(a, b)
                    self.assertEqual(baseline.diagnostics['reason'], merged.diagnostics['reason'])

    def test_collision_and_score_experiments_do_not_override_each_other(self):
        job = packet(); job['state']['speed_mps'] = 0.
        def plan(**values):
            return ShadowLatticePlanner(ShadowSettings(length_m=1.3, **values)).plan(job)
        default, check, score = plan(), plan(collision_coverage='generated'), plan(score_coverage='generated')
        self.assertEqual(default['check_length_m'], 0.)
        self.assertEqual(check['check_length_m'], 1.3)
        self.assertEqual(check['score_length_m'], default['score_length_m'])
        self.assertEqual(score['check_length_m'], default['check_length_m'])
        self.assertEqual(score['score_length_m'], 1.3)

    def test_obsolete_results_do_not_clear_or_replace_current_success(self):
        manager = PlannerExecutionManager(generation=4)
        value = dict(status='VALID', generation=4, job_id=5, input_stamp_s=10.,
            execution_anchor=dict(pose=[0., 0., 0.], odometry_epoch=1),
            selected=dict(path=[(0., 0., 0.), (1., 0., 0.)]))
        self.assertTrue(manager.accept(value))
        active = deepcopy(manager.active)
        for changes in (dict(generation=3, job_id=6), dict(job_id=4),
                        dict(job_id=6, input_stamp_s=9.9), dict(job_id=True)):
            self.assertFalse(manager.accept({**value, **changes}))
            self.assertEqual(manager.active, active)
        self.assertTrue(manager.accept(dict(status='ALL_COLLIDE', generation=4, job_id=6)))
        self.assertIsNone(manager.active)
        self.assertFalse(manager.accept(value))
        manager.clear(generation=5)
        self.assertFalse(manager.accept(value))

    def test_native_reference_does_not_require_a_valid_centerline_baseline(self):
        from mvp21_policy.transport import ShadowJobSource
        from mvp21_policy.planning import PlanningSettings
        from mvp21_policy.control import ControlSettings
        manager = PlannerExecutionManager(generation=1)
        manager.accept(dict(status='VALID', generation=1, job_id=1, input_stamp_s=10.,
            execution_anchor=dict(pose=[0., 0., 0.], odometry_epoch=0),
            selected=dict(path=[(0., 0., 0.), (1., 0., 0.)])))
        estimates = dict(road={**packet()['road'], 'timestamp_s':10., 'frame_id':'base_link',
            'time_aligned':True, 'valid':False, 'status':'near_field_unobserved'},
            state=dict(timestamp_s=10., frame_id='base_link', speed_valid=True, yaw_rate_valid=True,
                speed_mps=.2, yaw_rate_rps=0.))
        source = ShadowJobSource(); source.shadow_settings = ShadowSettings()
        source.estimation_output = estimates
        ref = manager.prepare(estimates, dict(valid=False, reason='invalid_centerline'), source,
            source.shadow_settings, PlanningSettings(), ControlSettings(), 20.)
        self.assertTrue(ref['valid'])
        self.assertEqual(ref['planner_version'], 'lattice_execution')


if __name__ == '__main__':
    unittest.main()
