import unittest,math,json
from pathlib import Path
import test_mpc as m
class ReferenceAdmissionRepair(unittest.TestCase):
 def test_minimum_model_horizon_fits_despite_conservative_rounding(self):
  tr=m.straight_trajectory(length=.1,start_x=0.,speed=.3)
  tr['points'][-1].update(x_m=.085,s_m=.085)
  c=m.verified(adaptive_horizon_enabled=True)
  out=c.step(tr,.3,0.,10.,True)
  self.assertEqual(out['debug']['branch'],'solved')
  self.assertTrue(out['debug']['minimum_horizon_support_check'])
  self.assertEqual(out['debug']['effective_horizon_n'],2)
  self.assertLessEqual(max(x for x,y in out['debug']['pred_xy']),.085-c.cfg.reference_end_margin_m+1e-6)
 def test_minimum_model_horizon_still_rejects_actual_end_overrun(self):
  c=m.verified(adaptive_horizon_enabled=True)
  tr=m.straight_trajectory(length=.05,start_x=0.,speed=.3)
  with self.assertRaises(m.Stop):c.step(tr,.3,0.,10.,True)
  self.assertEqual(c.last_debug['reject_reason'],'reference_too_short')
  self.assertEqual(c.last_debug['applied_drive'],0.)
 def test_saved_lateral_crop_is_not_behind_the_fitted_start(self):
  tr=json.loads((Path(__file__).parent/'fixtures/lattice_cropped_start_projection_20261019.json').read_text())['trajectory']
  for mirror in (1,-1):
   points=[dict(p,y_m=p['y_m']*mirror) for p in tr['points']]
   h,_,err=m.Controller._start_bridge_geometry(points);self.assertIsNone(err)
   along=-points[0]['x_m']*math.cos(h)-points[0]['y_m']*math.sin(h)
   self.assertGreaterEqual(along,0.)
   detail=m.namespace['mpc_path_projector'](points,1.2,h).details(0.,0.)
   self.assertLess(detail['raw_fraction'],0.)
   self.assertFalse(detail['before_start'])
   checked,why=m.errors(dict(tr,points=points),10,.1,.25,1.2,h)
   self.assertIsNone(why);self.assertFalse(checked['extended_start'])
 def test_genuine_start_extension_limit_and_path_end_are_unchanged(self):
  for start,reason in ((1.3,'origin_before_path_start'),(-3.,'origin_past_path_end')):
   tr=m.straight_trajectory(start_x=start,length=2.)
   info,why=m.errors(tr,10,.1,0.,1.2,0.)
   self.assertIsNone(info);self.assertEqual(why,reason)
 def test_permitted_straight_extension_still_reports_extension(self):
  info,why=m.errors(m.straight_trajectory(start_x=.2,length=2.),10,.1,.25,1.2,0.)
  self.assertIsNone(why);self.assertTrue(info['extended_start'])
  self.assertAlmostEqual(info['s_proj_m'],-.2)
