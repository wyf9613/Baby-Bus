"""Behavioral checks for the offline candidate, not the ROS/hardware gate."""
from dataclasses import replace
import math
import unittest

from controller import Controller, Gains, PID
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


if __name__=='__main__':
    unittest.main(verbosity=2)
