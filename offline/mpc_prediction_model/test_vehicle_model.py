import unittest
import numpy as np
from vehicle_model import CourseModelParams, step


class VehicleModelTests(unittest.TestCase):
    def test_straight_equilibrium(self):
        state = np.array([0., 0., 0., 1., 0.])
        original = state.copy()
        np.testing.assert_allclose(step(state, [0.1, 0], .05), [.05, 0, 0, 1, 0], atol=1e-12)
        np.testing.assert_array_equal(state, original)

    def test_constant_turn_matches_analytic_com_arc(self):
        p = CourseModelParams()
        delta, duration = .2, .5
        state = np.array([0., 0., 0., 1., delta])
        for _ in range(10):
            state = step(state, [.1, delta / p.steering_gain_rad], .05, p)
        omega = np.tan(delta) / (p.front_axle_m + p.rear_axle_m)
        theta = omega * duration
        expected = [np.sin(theta)/omega + p.rear_axle_m*(np.cos(theta)-1),
                    (1-np.cos(theta))/omega + p.rear_axle_m*np.sin(theta), theta, 1., delta]
        np.testing.assert_allclose(state, expected, atol=1e-8)

    def test_steering_rate_and_drag(self):
        result = step([0, 0, 0, 1, 0], [0, 1], .05)
        self.assertAlmostEqual(result[4], np.pi/2*.05)
        self.assertLess(result[3], 1.)
        self.assertGreater(result[1], 0.)
        self.assertGreater(result[2], 0.)

    def test_offset_and_asymmetric_range(self):
        p = CourseModelParams(steering_gain_rad=-.3, steering_offset_rad=.02,
                              steering_min_rad=-.25, steering_max_rad=.2)
        hold = lambda u: step([0, 0, 0, 0, .02], [0, u], 10., p)[4]
        self.assertAlmostEqual(hold(0), .02)
        self.assertAlmostEqual(hold(-1), .2)   # negative gain: -1 steers left, clipped
        self.assertAlmostEqual(hold(1), -.25)
        with self.assertRaises(ValueError):
            CourseModelParams(steering_offset_rad=1.)

    def test_default_mapping_matches_notebook_request(self):
        for u in (-1, -.3, 0, .55, 1):
            result = step([0, 0, 0, 0, 0], [0, u], 10.)
            self.assertAlmostEqual(result[4], u*np.pi/4)

    def test_invalid_inputs(self):
        for control, dt in [([0, 2], .05), ([0, 0], 0), ([np.nan, 0], .05)]:
            with self.assertRaises(ValueError):
                step([0, 0, 0, 1, 0], control, dt)


if __name__ == '__main__':
    unittest.main()
