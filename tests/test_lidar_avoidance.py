"""Hard geometry, causal snapshots, rear clearance and fallback safety contracts."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

try:
    from ament_index_python.packages import get_package_prefix
except ImportError:
    pass
else:
    sys.path.insert(0, str(Path(get_package_prefix('ai4r_policy'))/'lib/ai4r_policy'))
# The experimental causal service is source-only; core imports above keep the
# installed module first during the ROS gate.
sys.path.append(str(Path(__file__).resolve().parents[1]))

from mvp21_policy.avoidance import (LidarAvoidanceSettings, LidarAvoidanceGuard,
    collision, filter_candidates, obstacle_fault)
from mvp21_policy.execution import calculate_selected_control
from mvp21_policy.lattice import ShadowSettings
from mvp21_policy.obstacle_stop import ObstacleStopMonitor, ObstacleStopSettings
from offline.lidar_avoidance.service import AvoidancePlannerWorker


CFG = LidarAvoidanceSettings(enabled=True)


def observation(points=(), now=10., age=0.):
    return dict(available=True, frame_id='base_link', time_aligned=True,
        timestamp_s=now, measurement_timestamp_s=now-age, source_age_s=age,
        points_xyz_m=[[x, y, 0.] for x, y in points])


def reference(points, name='native'):
    return dict(valid=True, name=name, path=dict(samples_xy=points),
                timestamp_s=10., frame_id='base_link')


def job(points=()):
    return dict(settings=vars(ShadowSettings(enabled=True, length_m=2., fusion_enabled=False)),
        stamp_s=10., generation=1, job_id=1, submitted_monotonic_s=10.,
        shadow_pose=[0., 0., 0.], shadow_odometry_epoch=0, planner_control_enabled=True,
        preview_m=.35, state=dict(speed_mps=.2, speed_valid=True, yaw_rate_rps=0., yaw_rate_valid=True),
        road=dict(centerline_xy=[[0., 0.], [2., 0.], [4., 0.]],
            frame_id='base_link', time_aligned=True, timestamp_s=10.,
            left_boundary_xy=[[0., .65], [4., .65]], right_boundary_xy=[[0., -.65], [4., -.65]],
            boundary_source=dict(left='observed', right='observed'),
            lane_width_m=1.3, measurement_timestamp_s=10.),
        lidar_avoidance=dict(settings=vars(CFG), obstacles=observation(points),
            points_xy_m=list(points), side=0))


class AvoidanceGeometryChecks(unittest.TestCase):
    def test_point_returns_are_not_connected_across_a_free_gap(self):
        self.assertIsNone(collision([(0., 0., 0.), (1., 0., 0.)],
                                    [[.5, -.5], [.5, .5]], CFG))

    def test_side_of_rotated_body_and_between_samples_are_checked(self):
        self.assertIsNotNone(collision([(0., 0., 0.), (.7, .7, .785)], [[.4, .26]], CFG))
        tiny = replace(CFG, front_extent_m=.005, rear_extent_m=.005, width_m=.01, obstacle_margin_m=.01)
        self.assertIsNotNone(collision([(0., 0., 0.), (.3, 0., 0.)], [[.15, 0.]], tiny))

    def test_native_follow_is_kept_when_clear_and_smooth_bypass_exists(self):
        clear = filter_candidates(job())
        self.assertEqual(clear['selected']['family'], 'follow')
        blocked = filter_candidates(job([[1.35, y] for y in (-.1, -.05, 0., .05, .1)]))
        self.assertEqual(blocked['status'], 'VALID')
        self.assertTrue(any(c['avoidance']['rejected']=='lidar_point' for c in blocked['candidates']))
        self.assertEqual(blocked['selected']['family'], 'avoidance_lateral_hold')
        self.assertLessEqual(blocked['selected']['avoidance']['peak_curvature_1pm'], CFG.max_curvature_1pm)

    def test_blocked_wall_is_failure_without_a_diagnostic_motion_candidate(self):
        result = filter_candidates(job([[1.35, i*.05] for i in range(-13, 14)]))
        self.assertEqual(result['status'], 'NO_AVOIDANCE_PATH')
        self.assertIsNone(result['selected'])
        self.assertEqual(result['feasible_count'], 0)
        guard = LidarAvoidanceGuard(CFG);guard.accept(result)
        self.assertEqual(guard.stop_reason, 'no_feasible_avoidance_path')

    def test_commitment_and_missing_observed_boundary_reject_candidates(self):
        record = job([[1.35, y] for y in (-.1, 0., .1)])
        record['lidar_avoidance']['side'] = 1
        result = filter_candidates(record)
        self.assertEqual(result['selected']['avoidance']['side'], 1)
        self.assertTrue(any(c['avoidance']['rejected']=='committed_side' for c in result['candidates']))
        record['road']['boundary_source']['left'] = 'width_inferred'
        self.assertIsNone(filter_candidates(record)['selected'])


class AvoidanceStateChecks(unittest.TestCase):
    def test_empty_scan_does_not_erase_static_obstacle_before_rear_passes(self):
        guard = LidarAvoidanceGuard(CFG)
        guard.update(observation([[1., 0.]]), [0., 0., 0.], 0, 10.);guard.side = 1
        guard.update(observation(now=11.), [.9, 0., 0.], 0, 11.)
        self.assertEqual(len(guard.points), 1);self.assertEqual(guard.side, 1)
        self.assertEqual(next(iter(guard.memory.values()))[2], 10.)
        guard.update(observation(now=12.), [1.5, 0., 0.], 0, 12.)
        self.assertFalse(guard.memory);self.assertEqual(guard.side, 0)

    def test_source_fault_latches_and_fresh_clear_scan_cannot_resume(self):
        guard = LidarAvoidanceGuard(CFG)
        guard.update(observation(now=10., age=.31), [0., 0., 0.], 0, 10.)
        guard.update(observation(now=11.), [0., 0., 0.], 0, 11.)
        self.assertTrue(guard.latched)
        self.assertEqual(guard.stop_reason, 'lidar_stale_or_bad_timestamp')
        self.assertEqual(obstacle_fault(observation([[float('nan'), 0.]]), 10., CFG), 'lidar_points_invalid')

    def test_motion_epoch_loss_and_memory_overflow_fail_closed(self):
        guard = LidarAvoidanceGuard(CFG)
        guard.update(observation([[1., 0.]]), [0., 0., 0.], 0, 10.)
        guard.update(observation(now=11.), [0., 0., 0.], 1, 11.)
        self.assertEqual(guard.stop_reason, 'avoidance_motion_epoch_changed')
        small = LidarAvoidanceGuard(replace(CFG, memory_capacity=1))
        small.update(observation([[1., 0.], [1., .1]]), [0., 0., 0.], 0, 10.)
        self.assertEqual(small.stop_reason, 'avoidance_memory_capacity')

    def test_cached_reference_is_rechecked_against_new_obstacle(self):
        guard = LidarAvoidanceGuard(CFG)
        guard.update(observation(), [0., 0., 0.], 0, 10.)
        path = reference([[0., 0.], [2., 0.]])
        self.assertFalse(guard.evaluate(path, 10.)['stop_required'])
        guard.update(observation([[.8, 0.]], now=10.05), [0., 0., 0.], 0, 10.05)
        self.assertTrue(guard.evaluate(path, 10.05)['stop_required'])

    def test_worker_snapshots_obstacles_and_keeps_delivery_causal(self):
        guard = LidarAvoidanceGuard(CFG);guard.update(observation(), [0., 0., 0.], 0, 10.)
        worker = AvoidancePlannerWorker(guard, delay_s=.1);worker.now_s = 10.
        worker.submit(job())
        guard.points.append([.8, 0.])
        self.assertFalse(worker.pending['job']['lidar_avoidance']['points_xy_m'])
        self.assertIsNone(worker.poll(10.05));self.assertIsNotNone(worker.poll(10.1))

    def test_controller_rejection_cannot_fallback_through_blocked_centerline(self):
        class Controller:
            calls = []
            def clear(self): pass
            def calculate(self, ref, *_):
                self.calls.append(ref['name'])
                return .1, 0., dict(valid=False, stop_requested=True, reason='native_rejected')
        guard = LidarAvoidanceGuard(CFG)
        guard.update(observation([[.8, 0.]]), [0., 0., 0.], 0, 10.)
        safe = reference([[0., 0.], [.3, .4], [1., .4]])
        blocked = reference([[0., 0.], [1., 0.]], 'baseline')
        output = calculate_selected_control(Controller(), blocked, safe, {}, {}, 10., .05,
            execution=SimpleNamespace(diagnostics={}), lidar_avoidance=guard)
        self.assertEqual(output[1:3], (0., 0.))
        self.assertEqual(output[3]['reason'], 'avoidance_execution_collision')
        self.assertEqual(Controller.calls, ['native'])

    def test_existing_emergency_latch_has_priority_over_avoidance(self):
        class Controller:
            def clear(self): pass
        monitor = ObstacleStopMonitor(ObstacleStopSettings(enabled=True))
        guard = LidarAvoidanceGuard(CFG)
        output = calculate_selected_control(Controller(), None, None, {}, {}, 10., .05,
            obstacle_stop=monitor, obstacles=observation([[.35, 0.]]), lidar_avoidance=guard)
        self.assertEqual(output[3]['reason'], 'lidar_obstacle')
        self.assertFalse(guard.latched)


if __name__ == '__main__':
    unittest.main()
