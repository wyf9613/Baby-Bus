import ast
import math
import unittest
from pathlib import Path
from unittest.mock import patch
import test_mpc as m
import test_speed_feedback as f

class RuntimeOptimization(unittest.TestCase):
 def solver(self):
  return f.controller().solver
 def test_fixed_drive_elimination_keeps_bounds_with_half_variables(self):
  import numpy as np
  s=self.solver();original=s._solve_qp;shapes=[]
  def capture(P,q,C,lo,hi,deadline_s=None):
   shapes.append(P.shape[0]);return original(P,q,C,lo,hi,deadline_s)
  s._solve_qp=capture
  points=m.straight_trajectory(length=3.,speed=.25)['points']
  for steer in (-.1,0.,.1):
   z=[0.,.03,0.05,.2,steer]
   reduced,why=s.solve(points,z,.25,.3,fixed_drive_request=.3)
   self.assertIsNone(why);self.assertEqual(shapes[-1],10)
   for u in reduced['inputs']:self.assertAlmostEqual(u[0],.3)
   self.assertLessEqual(reduced['violation'],s.cfg.constraint_tolerance)
 def test_expired_deadline_does_no_model_or_qp_work(self):
  s=self.solver()
  with patch.object(s,'_linearise',side_effect=AssertionError('late model work')):
   out,why=s.solve(m.straight_trajectory()['points'],[0,0,0,.2,0],.25,.3,deadline_s=0.)
  self.assertIsNone(out);self.assertEqual(why,'deadline_exceeded')
 def test_deadline_during_linearisation_aborts_before_qp(self):
  s=self.solver();clock=[0.];real=s._linearise;calls=[]
  def linearise(*a,**kw):
   calls.append(1);out=real(*a,**kw);clock[0]=1.;return out
  with patch.object(m.time,'perf_counter',side_effect=lambda:clock[0]),patch.object(s,'_linearise',side_effect=linearise),patch.object(s,'_solve_qp',side_effect=AssertionError('late QP')):
   out,why=s.solve(m.straight_trajectory()['points'],[0,0,0,.2,0],.25,.3,deadline_s=.1)
  self.assertEqual(len(calls),1);self.assertIsNone(out);self.assertEqual(why,'deadline_exceeded')
 def test_dense_qp_cannot_return_solved_after_deadline(self):
  s=self.solver();np=s._np
  status,_,_=s._solve_dense(np.eye(1),np.zeros(1),np.eye(1),np.zeros(1),np.ones(1),deadline_s=0.)
  self.assertEqual(status,'run time limit reached')

 def test_original_qp_outputs_remain_equivalent(self):
  import json
  data=json.loads((Path(__file__).parent/'fixtures/runtime_qp_before_20261019.json').read_text())
  from dataclasses import replace
  s=self.solver();s.cfg=replace(s.cfg,speed_feedback_measured_motion=False)
  for item in data:
   sol,why=s.solve(item['points'],item['z'],.25,.3,fixed_drive_request=item['drive'])
   self.assertIsNone(why)
   for actual,expected in zip(sol['inputs'],item['inputs']):
    for a,b in zip(actual,expected):self.assertAlmostEqual(a,b,places=6)

 def test_translation_jacobian_matches_full_finite_differences(self):
  import numpy as np
  s=self.solver();z=[1.3,-.7,.23,.25,.05];u=[.32,-.2];p=s.vehicle
  base,A,B=s._linearise(z,u,.1)
  for j in (0,1):
   shifted=list(z);eps=1e-6*max(1.,abs(z[j]));shifted[j]+=eps
   full=(np.array(s._motion_step(shifted,u,.1,p,rate_limited=False))-base)/eps
   np.testing.assert_allclose(A[:,j],full,atol=1e-8,rtol=1e-8)

 def test_controller_retries_share_one_deadline_and_one_feedback_update(self):
  c=f.controller();real=c.solver.solve;calls=[];clock=[0.]
  tr=m.straight_trajectory(length=3.,start_x=0.,speed=.25)
  tr['reference_path_type']='ORDERED_CENTERLINE'
  def solve(*a,**kw):
   calls.append((kw['horizon_steps'],kw['deadline_s']))
   clock[0]+=.003
   if kw['horizon_steps']>2:return None,'reference_too_short'
   return real(*a,**kw)
  with patch.object(m.time,'perf_counter',side_effect=lambda:clock[0]),patch.object(c.solver,'solve',side_effect=solve),patch.object(c,'speed_feedback_drive',wraps=c.speed_feedback_drive) as feedback:
   out=c.step(tr,.2,0.,10.,True)
  self.assertEqual([a[0] for a in calls],[10,5,2])
  self.assertEqual(len(set(a[1] for a in calls)),1)
  self.assertLess(calls[0][1],c.cfg.max_step_time_s)
  self.assertEqual(feedback.call_count,1);self.assertEqual(len(c.history),1)
  self.assertEqual(out['debug']['branch'],'solved')
