"""Behavioral checks for the offline candidate, not the ROS/hardware gate."""
from dataclasses import replace
from copy import deepcopy
import json
import math
from pathlib import Path
import runpy
import unittest

from controller import Controller, Gains, PID, path_errors, closest_path_geometry
from tune import Scenario, simulate
from notebook_model import notebook_config

import gymnasium as gym
import numpy as np
from dreamgym.envs import BicycleModelDynamic


def reference(coeffs=None, speed=0.5):
    return {'timestamp_s': 0.0, 'valid': True,
            'path_coeffs': coeffs or [0.0,0.0,0.0,0.0],
            'x_range_m': [-0.5,2.0], 'target_speed_mps': speed}


class ControllerBehavior(unittest.TestCase):
    def test_path_on_left_and_right_produces_correct_turn(self):
        for y in [-0.15,0.15]:
            _,steer,info=Controller(Gains()).calculate(reference([y,0,0,0]),0.5,0,0)
            self.assertTrue(info['valid'])
            self.assertGreater(steer*y,0)

    def test_heading_error_corrects_even_when_position_error_is_zero(self):
        _,steer,_=Controller(Gains()).calculate(reference([0,0.1,0,0]),0.5,0,0)
        self.assertGreater(steer,0)

    def test_cg_curvature_feedforward_matches_no_slip_geometry(self):
        g=Gains()
        kappa=1/3
        beta=math.asin(g.rear_axle_from_cg_m*kappa)
        slope=math.tan(beta)
        second=kappa*(1+slope*slope)**1.5
        _,steer,_=Controller(g).calculate(reference([0,slope,second/2,0]),0.5,0,0)
        expected=math.atan(g.wheelbase_m*kappa/math.cos(beta))/g.steering_limit_rad
        self.assertAlmostEqual(steer,expected,places=6)

    def test_invalid_stale_future_or_nonfinite_reference_clears_state(self):
        for updates,now in [({'valid':False},0),({},0.2),({'timestamp_s':1.0},0),
                            ({'path_coeffs':[float('nan'),0,0,0]},0),
                            ({'x_range_m':[0,0]},0),({'target_speed_mps':-1},0)]:
            controller=Controller(Gains())
            controller.calculate(reference(),0,0,0.05)
            ref=reference();ref.update(updates)
            drive,steer,info=controller.calculate(ref,0.5,now,0.05)
            self.assertEqual((drive,steer),(0,0))
            self.assertFalse(info['valid'])
            self.assertEqual(controller.speed.integral,0)

    def test_saturation_does_not_accumulate_windup(self):
        pid=PID(1,1,0,0.15)
        for _ in range(200):
            self.assertEqual(pid.update(10,0,0.05,-0.3,0.35),0.35)
        self.assertEqual(pid.integral,0)
        self.assertEqual(pid.update(0,0,0.05,-0.3,0.35),0)

    def test_target_step_does_not_cause_derivative_kick(self):
        pid=PID(1,0,1,0.15)
        pid.update(0,0.5,0.05,-10,10)
        self.assertAlmostEqual(pid.update(0.5,0.5,0.05,-10,10),0.5)

    def test_first_step_and_stop_hold_are_finite(self):
        controller=Controller(Gains())
        drive,steer,_=controller.calculate(reference(),0,0,0)
        self.assertTrue(math.isfinite(drive) and math.isfinite(steer))
        drive,_,_=controller.calculate(reference(speed=0),0.4,0,0.05)
        self.assertLess(drive,0)
        drive,_,_=controller.calculate(reference(speed=0),0,0,0.05)
        self.assertLess(drive,0)

    def test_speed_pi_reduces_steady_error_and_stop_holds_without_reverse(self):
        gains=Gains(speed_kp=1.4,speed_ki=0.6,speed_kd=0.0)
        scenario=Scenario('test_speed','straight',initial_y=0,initial_heading=0)
        pi,trace=simulate(gains,scenario,longitudinal=True)
        p,_=simulate(replace(gains,speed_ki=0,speed_kd=0),scenario,longitudinal=True)
        self.assertTrue(pi['complete'])
        self.assertLess(pi['steady_speed_rmse_mps'],p['steady_speed_rmse_mps'])
        self.assertGreater(pi['progress_m'],10)
        self.assertIsNotNone(pi['stop_time_s'])
        self.assertTrue(all(r['speed_mps']>=-1e-9 for r in trace))
        self.assertLess(max(r['speed_mps'] for r in trace if 20<=r['time_s']<22),0.03)

    def test_direct_model_matches_notebook_gym_environment(self):
        # Compare full seven-state trajectories, including braking and steering.
        config=notebook_config()
        env=gym.make('dreamgym/autonomous_driving_env',
                     road_spec=config['CONE_ROAD_SPEC'],
                     bicycle_model_config=config['bicycle_model_config'],
                     initial_state_config={
                         'pose': {'mode':'road_relative','anchor':'reference_line',
                                  'progress':{'unit':'metres','lower':0.0,'upper':0.0},
                                  'lateral_offset_m':{'lower':0.0,'upper':0.0},
                                  'heading_error_rad':{'lower':0.0,'upper':0.0}},
                         'body_motion':{'longitudinal_velocity':{
                             'source':'absolute','value_mps':{'lower':0.0,'upper':0.0}}}},
                     integration_config=config['integration_config'],
                     observation_format='dict',render_mode=None)
        model=BicycleModelDynamic(config['bicycle_model_config'])
        try:
            env.reset(seed=0)
            model.reset(env.unwrapped.car.state)
            for action in [(0.2,0.1)]*10+[(-0.3,-0.1)]*10+[(0.0,0.0)]*4:
                env.step(np.array(action,dtype=np.float64))
                model.set_action_request(action[0],action[1]*math.pi/4)
                model.integrate(0.05,'rk4',update_stored_state=True)
                actual=model.state
                expected=env.unwrapped.car.state
                for group in ['world_pose','body_motion']:
                    np.testing.assert_allclose(list(actual[group].values()),
                                               list(expected[group].values()),atol=1e-7)
                self.assertAlmostEqual(actual['front_wheel_steering_angle_rad'],
                                       expected['front_wheel_steering_angle_rad'],places=7)
        finally:
            env.close()


class PlanningHandoff(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).resolve().parents[2]
        cls.planning = runpy.run_path(str(cls.root / 'tests' / 'test_planning.py'))

    def planned(self, offset=0.1, slope=0.0):
        api = self.planning
        ref, _ = api['Planner'](api['Settings']()).plan(*api['fixture'](offset, slope))
        self.assertTrue(ref['valid'], ref['reason'])
        return ref

    def test_actual_planner_front_only_path_is_consumed_without_mutation(self):
        for offset in (-0.1, 0.1):
            ref = self.planned(offset, 0.0)
            saved = deepcopy(ref)
            self.assertGreater(ref['path']['range'][0], 0)
            drive, steer, info = Controller(Gains()).calculate(ref, 0.1, 10.0, 0.1)
            self.assertTrue(info['valid'], info)
            self.assertGreater(drive, 0)
            self.assertGreater(steer*offset, 0)
            self.assertAlmostEqual(info['path_error_m'], offset)
            self.assertAlmostEqual(info['closest_x_m'], ref['path']['range'][0])
            self.assertTrue(info['closest_at_range_end'])
            self.assertEqual(ref, saved)

    def test_quintic_normalized_path_has_physical_heading_and_curvature(self):
        ref = self.planned()
        # y(x) = .1*x + .2*x^2 + .005*x^3 + .001*x^5;
        # u=x/2, so coefficients retain the producer's scaled convention.
        ref['path'].update(coeffs_low_to_high=[0, 0.2, 0.8, 0.04, 0, 0.032],
                           origin=0.0, scale=2.0, range=[0.0, 2.0])
        _, _, info = Controller(Gains()).calculate(ref, 0.1, 10.0, 0.1)
        self.assertTrue(info['valid'], info)
        self.assertEqual(info['path_degree'], 5)
        self.assertAlmostEqual(info['closest_x_m'], 0.0)
        self.assertAlmostEqual(info['path_heading_rad'], math.atan(0.1))
        self.assertAlmostEqual(info['curvature_1pm'], 0.4/(1+0.1**2)**1.5)

    def test_shift_and_scale_do_not_change_world_geometry(self):
        p = np.polynomial.Polynomial([0.1, 0.05, 0.02, -0.01, 0.002, 0.001])
        origin, scale = 1.2, 0.7
        normalized = p(np.polynomial.Polynomial([origin, scale]))
        expected = path_errors(p.coef.tolist(), [0.4, 2.0])
        actual = path_errors(normalized.coef.tolist(), [0.4, 2.0], origin, scale)
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_front_only_quintic_endpoint_uses_all_six_coefficients(self):
        ref = self.planned()
        coeffs = [0.1, 0.03, 0.02, -0.01, 0.002, 0.005]
        ref['path'].update(coeffs_low_to_high=coeffs, range=[0.5, 2.0])
        _, _, info = Controller(Gains()).calculate(ref, 0.1, 10.0, 0.1)
        self.assertTrue(info['valid'], info)
        x = 0.5
        y = 0.1 + 0.03*x + 0.02*x**2 - 0.01*x**3 + 0.002*x**4 + 0.005*x**5
        slope = 0.03 + 0.04*x - 0.03*x**2 + 0.008*x**3 + 0.025*x**4
        second = 0.04 - 0.06*x + 0.024*x**2 + 0.1*x**3
        heading = math.atan(slope)
        self.assertAlmostEqual(info['closest_x_m'], x)
        self.assertAlmostEqual(info['closest_y_m'], y)
        self.assertAlmostEqual(info['path_error_m'], -x*math.sin(heading)+y*math.cos(heading))
        self.assertAlmostEqual(info['curvature_1pm'], second/(1+slope*slope)**1.5)

    def test_quintic_multiple_minima_selects_global_bounded_nearest_point(self):
        p = 6*np.polynomial.Polynomial.fromroots([0.2, 0.7, 1.3, 1.8, 2.4])
        xs = np.linspace(0.0, 2.7, 100001)
        distances = xs**2+p(xs)**2
        local = (distances[1:-1] < distances[:-2]) & (distances[1:-1] < distances[2:])
        self.assertGreaterEqual(int(local.sum()), 3)
        nearest = closest_path_geometry(tuple(p.coef), (0.0, 2.7))
        self.assertAlmostEqual(nearest['closest_x_m'], float(xs[np.argmin(distances)]), delta=3e-5)
        self.assertLessEqual(nearest['closest_x_m']**2 + nearest['closest_y_m']**2,
                             float(distances.min())+1e-10)

    def test_legacy_cubic_and_quintic_padding_give_same_actions(self):
        original = reference([0.1, 0.05, 0.02, -0.005])
        padded = deepcopy(original)
        padded['path_coeffs'] += [0.0, 0.0]
        a = Controller(Gains()).calculate(original, 0.2, 0, 0.05)
        b = Controller(Gains()).calculate(padded, 0.2, 0, 0.05)
        np.testing.assert_allclose(a[:2], b[:2], atol=1e-12)

    def test_deadline_frame_encoding_and_bad_quintic_reset_integrators(self):
        good = self.planned()
        cases = []
        for update in ({'valid_for_s': 0.0}, {'frame_id': 'map'},
                       {'vehicle_reference_point': 'rear_axle'}, {'valid': False}):
            ref = deepcopy(good)
            ref.update(update)
            cases.append((ref, 10.0))
        cases.extend([(good, 10.1), (good, 10.01), (good, 9.99)])
        for update in ({'type': 'FRENET_D_OF_S'}, {'independent_variable': 'time_s'},
                       {'scale': 0}, {'origin': float('nan')}, {'range': [1.0, 1.0]},
                       {'coeffs_low_to_high': [0]*7},
                       {'coeffs_low_to_high': [0, 0, 0, 0, 0, float('inf')]}):
            ref = deepcopy(good)
            ref['path'].update(update)
            cases.append((ref, 10.0))
        for ref, now in cases:
            controller = Controller(Gains())
            controller.calculate(good, 0, 10, 0.1)
            drive, steer, info = controller.calculate(ref, 0.1, now, 0.1)
            self.assertEqual((drive, steer), (0, 0))
            self.assertFalse(info['valid'])
            self.assertTrue(info['stop_requested'])
            self.assertEqual(controller.speed.integral, 0)
            self.assertEqual(controller.lateral.integral, 0)

    def test_stop_request_overrides_positive_target(self):
        ref = self.planned()
        controller = Controller(Gains())
        controller.calculate(ref, 0, 10, 0.1)
        ref['stop_requested'] = True
        drive, _, info = controller.calculate(ref, 0.2, 10, 0.1)
        self.assertTrue(info['valid'], info)
        self.assertTrue(info['stop_requested'])
        self.assertLess(drive, 0)
        self.assertEqual(controller.speed.integral, 0)

    def test_estimator_planner_controller_model_closed_loop(self):
        api = self.planning['api']
        gains = Gains(**json.loads((self.root/'offline/control_pid/results/selected_gains.json').read_text()))
        for offset, heading in ((0.1, 0.04), (-0.1, -0.04)):
            model = BicycleModelDynamic(notebook_config()['bicycle_model_config'])
            model.reset({'world_pose': {'y_m': offset, 'heading_rad': heading}})
            estimator = api['Pipeline'](api['Settings']())
            planner = self.planning['Planner'](self.planning['Settings'](vehicle_limits_source='course_simulation'))
            controller = Controller(gains)
            errors = []
            for step in range(100):
                now, dt = 10.0+step*0.1, 0.1
                pose, motion = model.state['world_pose'], model.state['body_motion']
                slope = -math.tan(pose['heading_rad'])
                center = -pose['y_m']/math.cos(pose['heading_rad'])
                left, right = api['straight'](slope=slope, center=center)
                # Observe closer than the replay fixture; no invented rear points.
                left = [(x-0.4, y-0.4*slope) for x, y in left]
                right = [(x-0.4, y-0.4*slope) for x, y in right]
                args = list(api['input_record'](speed=motion['longitudinal_velocity_mps'], yaw=motion['yaw_rate_rad_per_s']))
                args[0]['cone_detections'] = api['cones'](left, right)
                args[1] = {key: 0.0 for key in args[0]}
                args[2] = {key: None if key == 'wheel_speed' else round(now*1e9) for key in args[0]}
                args[3]['wheel_speed'] = now
                estimates = estimator.update(*args, now)
                ref, _ = planner.plan(estimates['road'], estimates['state'], estimates['obstacles'],
                                      estimates['vehicle_limits'], now)
                self.assertTrue(ref['valid'], ref['reason'])
                drive, steer, info = controller.calculate(ref, estimates['state']['speed_mps'], now, dt)
                self.assertTrue(info['valid'], info)
                self.assertGreater(ref['path']['range'][0], 0)
                self.assertGreaterEqual(info['closest_x_m'], ref['path']['range'][0]-1e-9)
                self.assertLessEqual(info['closest_x_m'], ref['path']['range'][1]+1e-9)
                self.assertLess(abs(pose['y_m']), 0.2)
                errors.append(abs(pose['y_m']))
                model.set_action_request(drive, steer*gains.steering_limit_rad)
                model.integrate(dt, 'rk4', update_stored_state=True)
            self.assertLess(max(errors[-20:]), 0.03)
            self.assertAlmostEqual(model.state['body_motion']['longitudinal_velocity_mps'], 0.2, delta=0.05)

    def test_actual_policy_timer_with_delayed_ten_hz_cones_model_closed_loop(self):
        control_api = runpy.run_path(str(self.root/'tests/test_control.py'))
        for offset, heading, mvp in ((0.1, 0.04, False), (-0.1, -0.04, False),
                                     (0.1, 0.04, True), (-0.1, -0.04, True)):
            # MVP uses the course model only as the plant. It receives no
            # simulation vehicle parameters or limits in its planning/control.
            node = control_api['make_node'](simulated=not mvp, mvp=mvp)
            model = BicycleModelDynamic(notebook_config()['bicycle_model_config'])
            model.reset({'world_pose': {'y_m': offset, 'heading_rad': heading}})
            previous_pose = deepcopy(model.state['world_pose'])
            errors = []
            for step in range(420 if mvp else 200):
                pose, motion = model.state['world_pose'], model.state['body_motion']
                mono, stamp = node.test_clock.monotonic, node.test_clock.ros_ns
                node._store('wheel_speed', motion['longitudinal_velocity_mps'], None, mono, stamp)
                node._store('imu_angular_velocity', (0, 0, motion['yaw_rate_rad_per_s']), stamp, mono, stamp)
                if step % 2 == 0:
                    # Actual V1 source: a 10 Hz measurement, 50 ms old. The
                    # controller runs at 20 Hz, with every reference regenerated
                    # by the deployed method's motion-history alignment.
                    c = math.cos(previous_pose['heading_rad'])
                    slope = -math.tan(previous_pose['heading_rad'])
                    center = -previous_pose['y_m']/c
                    xs = [0.1+0.3*i for i in range(9)]
                    batch = self.planning['api']['cones'](
                        [(x, center+slope*x+0.5/c) for x in xs],
                        [(x, center+slope*x-0.5/c) for x in xs])
                    node._store('cone_detections', batch, stamp-50_000_000, mono, stamp)
                    node.last_nonempty_cones = node.observations['cone_detections']
                if step == 0:
                    control_api['start'](node)
                node.run_policy_step()
                node.supervision_callback()
                if node.fsm_state == 2:
                    self.assertTrue(mvp)
                    self.assertIn('Distance limit reached', node.state_reason)
                    self.assertTrue(3.0-1e-9 <= node.distance_limiter.distance_m < 3.02)
                    self.assertEqual(node.action_publisher.messages[-1].drive, 0)
                    self.assertEqual(node.action_publisher.messages[-1].steer, 0)
                    break
                self.assertEqual(node.fsm_state, 3, node.state_reason)
                self.assertEqual(node.logged_errors, [])
                action = node.action_publisher.messages[-1]
                self.assertTrue(0 <= action.drive <= 0.15)
                self.assertLessEqual(abs(action.steer), 0.5)
                self.assertEqual(node.pan_publisher.messages, [])
                errors.append(abs(pose['y_m']))
                self.assertLess(errors[-1], 0.2)
                previous_pose = deepcopy(pose)
                model.set_action_request(action.drive, action.steer*math.pi/4)
                model.integrate(0.05, 'rk4', update_stored_state=True)
                node.test_clock.advance(0.05)
            self.assertEqual(node.fsm_state, 2 if mvp else 3, node.state_reason)
            self.assertLess(max(errors[-40:]), 0.03)
            self.assertAlmostEqual(model.state['body_motion']['longitudinal_velocity_mps'], 0.2, delta=0.05)

    def test_single_file_root_isolation_matches_independent_numpy_solver(self):
        runtime = self.planning['api']['namespace']['control_path_geometry']
        rng = np.random.default_rng(307)
        for degree in range(6):
            for _ in range(12):
                coeffs = rng.uniform(-0.5, 0.5, degree+1).tolist()
                origin, scale = float(rng.uniform(-0.5, 0.5)), float(rng.uniform(0.5, 2.0))
                bounds = [-0.5, 2.0] if degree % 2 else [0.3, 2.0]
                expected = closest_path_geometry(tuple(coeffs), tuple(bounds), origin, scale)
                actual = runtime({'type': 'CARTESIAN_Y_OF_X', 'independent_variable': 'x_m',
                    'coeffs_low_to_high': coeffs, 'range': bounds, 'origin': origin, 'scale': scale})
                for field in ('closest_x_m', 'closest_y_m', 'path_error_m', 'path_heading_rad', 'curvature_1pm'):
                    self.assertAlmostEqual(actual[field], expected[field], delta=1e-8)


if __name__=='__main__':
    unittest.main(verbosity=2)
