import unittest,json,math,os
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from test_user_mpc_bridge import PARAMS
from mvp21_policy.lattice import ShadowSettings,ShadowLatticePlanner,_shadow_collision
from mvp21_policy.learned import FrozenPathPolicy,LearnedPolicySettings
from mvp21_policy.geometry import _path_lengths,_path_tangent

class NarrowChecks(unittest.TestCase):
    def request(self,width=.4):
        cfg=replace(ShadowSettings(**PARAMS['shadow']),fusion_enabled=False)
        st=dict(speed_mps=0.,yaw_rate_rps=0.,speed_valid=True,yaw_rate_valid=True,timestamp_s=10.,frame_id='base_link')
        road=dict(centerline_xy=[(0.,0.),(1.,0.),(3.,0.)],lane_width_m=width,
            left_boundary_xy=[(0.,width/2),(3.,width/2)],right_boundary_xy=[(0.,-width/2),(3.,-width/2)],
            boundary_source=dict(left='observed',right='observed'))
        j=dict(settings=vars(cfg),road=road,state=st,generation=1,job_id=1,stamp_s=10.,preview_m=.35,
            shadow_pose=[0.,0.,0.],shadow_odometry_epoch=0,submitted_monotonic_s=1.,planner_control_enabled=True)
        return cfg,j

    def test_narrow_straight_still_has_full_body_feasible_candidate(self):
        cfg,j=self.request(.35);out=ShadowLatticePlanner(cfg).plan(j)
        self.assertEqual(out['status'],'VALID')
        self.assertGreater(out['feasible_count'],0)
        self.assertAlmostEqual(out['sampling_offset_budget_m'],.05)
        self.assertEqual(out['body_dimensions_m'],[.297,.198,.25])
        self.assertGreaterEqual(out['check_length_m'],.5)
        self.assertGreaterEqual(out['generated_length_m'],1.4)
        self.assertIsNone(out['selected']['collision'])
        self.assertIsNone(out['selected']['controller_rejection'])

    def test_gentle_narrow_bend_has_model_compatible_arc(self):
        cfg,j=self.request(.65)
        angles=[k*.025 for k in range(31)];radius=3.;width=.65
        j['road']['centerline_xy']=[(radius*math.sin(a),radius*(1-math.cos(a))) for a in angles]
        j['road']['left_boundary_xy']=[((radius-width/2)*math.sin(a),radius-(radius-width/2)*math.cos(a)) for a in angles]
        j['road']['right_boundary_xy']=[((radius+width/2)*math.sin(a),radius-(radius+width/2)*math.cos(a)) for a in angles]
        out=ShadowLatticePlanner(cfg).plan(j)
        self.assertEqual(out['status'],'VALID')
        compatible=[c for c in out['candidates'] if c['family']=='mpc_arc' and c['collision'] is None and c['controller_rejection'] is None]
        self.assertTrue(compatible)
        self.assertEqual(out['candidate_count'],48)

    def test_road_narrower_than_body_cannot_be_marked_clear(self):
        cfg,j=self.request(.2);out=ShadowLatticePlanner(cfg).plan(j)
        self.assertEqual(out['status'],'ALL_COLLIDE')
        self.assertEqual(out['feasible_count'],0)
        self.assertIsNotNone(out['selected']['collision'])

    def test_stationary_obstacle_ahead_is_not_missed(self):
        cfg,j=self.request(.8)
        j['road']['left_boundary_xy']=[(.4,-.5),(.4,.5)]
        out=ShadowLatticePlanner(cfg).plan(j)
        for c in out['candidates']:
            self.assertIsNotNone(c['collision'])

    def test_rnn_never_scores_controller_rejected_candidate(self):
        cfg,j=self.request(.9);out=ShadowLatticePlanner(cfg).plan(j)
        model=FrozenPathPolicy(LearnedPolicySettings(**PARAMS['learned_policy']),.5)
        self.assertEqual(out['status'],'VALID')
        rejected={c['id'] for c in out['candidates'] if c.get('controller_rejection')}
        self.assertTrue(rejected)
        obs=dict(frame_id='base_link',timestamp_s=10.,estimation=dict(state=j['state'],road=j['road'],obstacles=dict(available=True,points_xyz_m=[])))
        out=model.select(out,obs,dict(revision=0,hidden=[0.]*16))
        scored={row['candidate_id'] for row in out['policy_selection']['scores']}
        self.assertFalse(scored & rejected)
        self.assertEqual(out['policy_selection']['status'],'SELECTED')

    def test_logged_collision_remains_a_collision(self):
        source=os.environ.get('AI4R_REPLAY_LOG')
        if source:
            records=[json.loads(line.split('MPC rejection details: ',1)[1])
                     for line in Path(source).read_text(encoding='utf-8').splitlines()
                     if 'MPC rejection details: ' in line]
        else:
            fixture=Path(__file__).parent/'fixtures/rnn_mpc_boundary_replay_20261026.json'
            records=json.loads(fixture.read_text(encoding='utf-8'))
        count=0
        cfg=ShadowSettings(**PARAMS['shadow'])
        for d in records:
            info=d['planner']
            if info.get('reason')!='known_boundary_collision':continue
            xy=info['remaining_path_xy'];lengths=_path_lengths(xy)
            poses=[(*pt,math.atan2(*reversed(_path_tangent(xy,lengths,lengths[k],window=cfg.path_spacing_m/2)))) for k,pt in enumerate(xy)]
            if info['connector_length_m']>1e-9:poses.insert(0,(0.,0.,0.))
            hit=_shadow_collision(poses,info['check_length_m']+info['connector_length_m'],info['known_boundaries_xy'],cfg)
            self.assertIsNotNone(hit)
            self.assertEqual(hit['side'],info['collision']['side'])
            count+=1
        self.assertGreaterEqual(count,2)

if __name__=='__main__':unittest.main()
