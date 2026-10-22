"""Stop geometry, observation faults, latching and baseline-fallback priority."""
import unittest
from types import SimpleNamespace
from pathlib import Path
import sys

try:
    from ament_index_python.packages import get_package_prefix
except ImportError:
    package_root = Path(__file__).resolve().parents[1]
else:
    package_root = Path(get_package_prefix('ai4r_policy'))/'lib/ai4r_policy'
sys.path.insert(0, str(package_root))

from mvp21_policy.obstacle_stop import ObstacleStopSettings, ObstacleStopMonitor
from mvp21_policy.execution import calculate_selected_control


def observation(points=(), age=0.):
    return dict(available=True,time_aligned=True,frame_id='base_link',timestamp_s=10.,
                measurement_timestamp_s=10.-age,source_age_s=age,points_xyz_m=list(points))


class ObstacleStopChecks(unittest.TestCase):
    def monitor(self):
        return ObstacleStopMonitor(ObstacleStopSettings(enabled=True))

    def test_threshold_is_front_bumper_gap_and_inclusive(self):
        monitor=self.monitor()
        self.assertFalse(monitor.evaluate(observation([(.448,0.,0.)]),10.)['stop_required'])
        event=monitor.evaluate(observation([(.447,0.,0.)]),10.)
        self.assertTrue(event['stop_required'])
        self.assertAlmostEqual(event['trigger']['front_gap_m'],.15)

    def test_outside_body_width_and_behind_vehicle_do_not_stop(self):
        result=self.monitor().evaluate(observation([(.35,.126,0.),(-.5,0.,0.)]),10.)
        self.assertFalse(result['stop_required'])
        self.assertIsNone(result['nearest_front_gap_m'])

    def test_return_already_inside_body_stops(self):
        self.assertTrue(self.monitor().evaluate(observation([(.2,.12,0.)]),10.)['stop_required'])

    def test_fresh_empty_scan_does_not_mean_sensor_failure(self):
        self.assertFalse(self.monitor().evaluate(observation(),10.)['stop_required'])

    def test_missing_stale_misaligned_or_bad_points_fail_closed(self):
        cases=[None,observation(age=.15)]
        for key,value in [('available',False),('frame_id','laser'),('time_aligned',False),
                          ('timestamp_s',9.9),('measurement_timestamp_s',9.),
                          ('points_xyz_m',[(float('nan'),0.,0.)])]:
            item=observation();item[key]=value;cases.append(item)
        for item in cases:
            with self.subTest(item=item):
                self.assertTrue(self.monitor().evaluate(item,10.)['stop_required'])

    def test_obstacle_disappearance_does_not_release_stop(self):
        monitor=self.monitor();first=monitor.evaluate(observation([(.4,0.,0.)]),10.)
        second=monitor.evaluate(observation(),10.)
        self.assertTrue(second['stop_required']);self.assertEqual(first['trigger'],second['trigger'])
        monitor.reset()
        self.assertFalse(monitor.evaluate(observation(),10.)['stop_required'])

    def test_disabled_monitor_preserves_legacy_behavior(self):
        self.assertFalse(ObstacleStopMonitor().evaluate(None,float('nan'))['stop_required'])

    def test_receipt_age_limit_requires_positive_finite_value(self):
        for value in (0., -.1, float('nan'), float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                ObstacleStopSettings(max_receipt_age_s=value)

    def test_stop_bypasses_both_planner_and_baseline_control(self):
        class Controller:
            def clear(self): self.cleared=True
            def calculate(self,*args): raise AssertionError('Stop must not enter fallback/control')
        for native in [None,{'valid':False,'reason':'planner_failure'}]:
            controller=Controller();execution=SimpleNamespace(diagnostics={})
            result=calculate_selected_control(controller,{'valid':False},native,{}, {},10.,.05,
                execution,baseline_provider=lambda: self.fail('Stop cannot fall back'),
                obstacle_stop=self.monitor(),obstacles=observation([(.4,0.,0.)]))
            self.assertEqual(result[1:3],(0.,0.));self.assertTrue(result[3]['stop_requested'])
            self.assertTrue(controller.cleared)


if __name__=='__main__':
    unittest.main()
