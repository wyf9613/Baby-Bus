import unittest
from dataclasses import replace
from unittest.mock import patch
import test_mpc as m
import test_speed_feedback as f
class FeedbackMotionPrediction(unittest.TestCase):
 def controller(self,hold=True):
  c=f.controller();c.cfg=replace(c.cfg,speed_feedback_measured_motion=hold);c.solver.cfg=c.cfg
  c.clock_s=2.;c.history=[(0.,.30,0.)];c.applied_drive=.30
  return c
 def test_delay_prediction_uses_measured_speed_not_gym_drive_gain(self):
  old=self.controller(False);new=self.controller(True)
  oz,_=old.initial_state(.25);nz,_=new.initial_state(.25)
  self.assertGreater(oz[3],.45);self.assertAlmostEqual(nz[3],.25)
  self.assertAlmostEqual(nz[0],.25*.26)
  self.assertEqual(old.vehicle,new.vehicle)
 def test_internal_balancing_effort_is_not_published_as_throttle(self):
  c=self.controller();tr=m.straight_trajectory(length=1.,start_x=0.,speed=.25);tr['reference_path_type']='ORDERED_CENTERLINE'
  out=c.step(tr,.25,.05,10.,False)
  self.assertAlmostEqual(out['drive'],out['debug']['speed_feedback_drive'])
  self.assertGreater(out['drive'],.1)
  self.assertAlmostEqual(out['debug']['pred_v_end_mps'],.25)
  self.assertEqual(out['debug']['longitudinal_prediction_mode'],'measured_speed_hold')
 def test_force_model_mode_probes_shorter_horizon_before_linearisation(self):
  c=self.controller(False);tr=m.straight_trajectory(length=.7,start_x=0.,speed=.25);tr['reference_path_type']='ORDERED_CENTERLINE'
  with patch.object(c.solver,'_linearise',wraps=c.solver._linearise) as linearise:
   out=c.step(tr,.25,.05,10.,False)
  d=out['debug'];self.assertEqual(d['branch'],'solved')
  self.assertEqual(len(d['horizon_attempts']),1)
  self.assertLess(d['effective_horizon_n'],d['horizon_attempts'][0])
  self.assertEqual(linearise.call_count,d['effective_horizon_n']*c.cfg.sqp_iterations)
  self.assertGreater(d['solver_phases_s']['support_probe'],0.)
 def test_measured_mode_fits_log_like_path_without_invented_acceleration(self):
  c=self.controller();tr=m.straight_trajectory(length=.7,start_x=0.,speed=.25);tr['reference_path_type']='ORDERED_CENTERLINE'
  out=c.step(tr,.25,.05,10.,False);d=out['debug']
  self.assertEqual(d['branch'],'solved');self.assertEqual(d['effective_horizon_n'],10)
  self.assertEqual(d['horizon_attempts'],[10])
  self.assertLessEqual(max(x for x,y in d['pred_xy']),.7-c.cfg.reference_end_margin_m)
 def test_true_short_support_still_stops(self):
  c=self.controller();tr=m.straight_trajectory(length=.1,start_x=0.,speed=.25);tr['reference_path_type']='ORDERED_CENTERLINE'
  with self.assertRaises(m.Stop):c.step(tr,.25,.05,10.,False)
  self.assertEqual(c.last_debug['applied_drive'],0.)
  self.assertEqual(c.last_debug['reject_reason'],'reference_too_short')
 def test_measured_mode_requires_feedback_and_boolean(self):
  with self.assertRaises(ValueError):m.Settings(speed_feedback_measured_motion=True)
  with self.assertRaises(ValueError):m.Settings(speed_feedback_measured_motion=1)

 def test_rolling_reference_with_delayed_wheel_feedback_keeps_solving(self):
  from collections import deque
  c=f.controller();q=deque();speed=0.;distance=0.;minimum=1.
  for i in range(160):
   t=i*.05;tr=m.straight_trajectory(length=.7,start_x=0.,speed=.25,stamp=10.+t)
   tr['reference_path_type']='ORDERED_CENTERLINE'
   out=c.step(tr,speed,0. if i==0 else .05,10.+t,i==0)
   self.assertEqual(out['debug']['branch'],'solved')
   q.append((t,out['drive']));applied=0.
   for at,u in q:
    if at<=t-.26:applied=u
    else:break
   while len(q)>1 and q[1][0]<=t-.26:q.popleft()
   # Assumed delayed FOPDT from the logged local response; not car validation.
   speed=max(0.,speed+.05*(3.725*applied-speed)/1.33)
   distance+=speed*.05
   if i>=80:minimum=min(minimum,speed)
  self.assertGreater(distance,1.);self.assertGreater(minimum,.02)
