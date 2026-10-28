"""Target identity, physical progress, job fencing and final-reference safety."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import unittest
import math
from types import SimpleNamespace
try:
    from ament_index_python.packages import get_package_prefix
except ImportError:
    pass
else:
    sys.path.insert(0,str(Path(get_package_prefix('ai4r_policy'))/'lib/ai4r_policy'))
from mvp21_policy.avoidance import LidarAvoidanceSettings,LidarAvoidanceGuard,filter_candidates,accept_avoidance_result
from mvp21_policy.maneuver import body,task_offset
from mvp21_policy.lattice import ShadowSettings,ShadowLatticePlanner
from mvp21_policy.execution import PlannerExecutionManager
from mvp21_policy.obstacle_stop import ObstacleStopMonitor,ObstacleStopSettings
from mvp21_policy.transport import ShadowJobSource

CFG=LidarAvoidanceSettings(enabled=True,maneuver_enabled=True,max_curvature_1pm=1.0)
TARGET=[[2.8,y] for y in (-.1,-.05,0.,.05,.1)]
ROADSIDE=[[x,y] for x in (1.,2.,3.,4.,5.,6.,7.) for y in (-.9,.9)]

def tick(guard,pose=(0.,0.,0.),now=10.,points=None,missing=None,measurement=None,width=1.8,
         left_boundary=None):
    road=dict(frame_id='base_link',time_aligned=True,timestamp_s=now,
        measurement_timestamp_s=now if measurement is None else measurement,lane_width_m=width,
        centerline_xy=body([[-1.,0.],[0.,0.],[4.,0.],[8.,0.]],pose),
        left_boundary_xy=body([[-1.,width/2],[8.,width/2]],pose),right_boundary_xy=body([[-1.,-width/2],[8.,-width/2]],pose),
        boundary_source=dict(left='observed',right='observed'))
    if left_boundary is not None:road['left_boundary_xy']=body(left_boundary,pose)
    if missing:road[missing+'_boundary_xy']=[];road['boundary_source'][missing]='absent'
    xy=body(TARGET+ROADSIDE if points is None else points,pose)
    obs=dict(available=True,frame_id='base_link',time_aligned=True,timestamp_s=now,
        measurement_timestamp_s=now,source_age_s=0.,points_xyz_m=[[*p,0.] for p in xy])
    guard.update(obs,list(pose),0,now,road=road)
    return dict(settings=vars(ShadowSettings(enabled=True,length_m=2.5,rate_hz=2.,fusion_use_prior=False)),
        road=road,stamp_s=now,state=dict(speed_mps=.2,speed_valid=True,yaw_rate_rps=0.,yaw_rate_valid=True),
        preview_m=.35,shadow_pose=list(pose),shadow_odometry_epoch=0,generation=1,job_id=1,
        lidar_avoidance=guard.context())


class ManeuverChecks(unittest.TestCase):
    def test_logged_two_point_two_eight_metre_start_can_generate_checked_path(self):
        # Logged r3 admission: 16 cm shift and 2.283 m forward distance was
        # rejected solely by the fixed 2 m minimum. Surrounding road here is
        # an explicit straight fixture, not a replay of unrecorded field data.
        cfg=replace(self.narrow_cfg(),adaptive_shift_enabled=True,maneuver_clearance_m=.02)
        g=LidarAvoidanceGuard(cfg)
        points=[[2.282685665830912,y] for y in (-.18226083517493913,-.16391498215839076)]
        j=tick(g,pose=(0.,-.0892126728750285,0.),width=.8,points=points)
        self.assertFalse(g.latched,g.diagnostics)
        details=g.maneuver.admission['shift_geometry'];task=g.maneuver.task
        self.assertLess(details['selected_shift_length_m'],2.)
        self.assertGreater(details['selected_shift_length_m'],details['minimum_shift_length_m'])
        distance=task['target_s_min']-task['shift_end_s']-cfg.front_extent_m-cfg.obstacle_margin_m
        self.assertAlmostEqual(distance,.10)
        r=filter_candidates(j)
        self.assertEqual(r['status'],'VALID',r.get('avoidance'))
        self.assertLessEqual(r['selected']['peak_curvature_1pm'],cfg.max_curvature_1pm*.8)

    def test_adaptive_shift_still_rejects_too_little_longitudinal_room(self):
        cfg=replace(self.narrow_cfg(),adaptive_shift_enabled=True,maneuver_clearance_m=.02)
        g=LidarAvoidanceGuard(cfg);tick(g,width=.8,points=[[.9,-.16]])
        self.assertEqual(g.stop_reason,'maneuver_too_close_to_start')
        d=g.first_failure['maneuver']['admission']['shift_geometry']
        self.assertGreater(d['minimum_shift_length_m'],d['available_shift_length_m'])
        self.assertIsNone(d['selected_shift_length_m'])
        for changes in (dict(adaptive_shift_enabled=1),dict(shift_curvature_utilization=1.1)):
            with self.assertRaises(ValueError):replace(cfg,**changes)

    def test_larger_clearance_keeps_the_rotating_front_corner_off_roadside_returns(self):
        cfg=replace(self.narrow_cfg(),adaptive_shift_enabled=True,maneuver_clearance_m=.02)
        g=LidarAvoidanceGuard(cfg)
        j=tick(g,width=.8,points=[[2.48055,-.13],[2.0204004846960135,.32],[2.86,.32]])
        self.assertFalse(g.latched,g.diagnostics)
        r=filter_candidates(j)
        self.assertEqual(r['status'],'VALID',r.get('collision'))
        self.assertGreater(g.maneuver.task['goal_d'],.11)
        self.assertLess(g.maneuver.task['goal_d'],.115)

    def narrow_cfg(self):
        return replace(CFG,front_extent_m=.275,rear_extent_m=.198,width_m=.30,
            obstacle_margin_m=.03,tracking_margin_m=.02,target_lateral_allowance_m=.02,
            maneuver_clearance_m=.01,road_edge_exclusion_m=.10)

    def narrow_points(self):
        return [[2.8,y] for y in (-.38,-.33,-.28,-.23,-.18,-.13)]+[
            [x,y] for x in (1.,2.,3.,4.,5.,6.,7.) for y in (-.32,.32)]

    def test_eighty_cm_road_right_obstacle_and_cone_surfaces_produce_left_path(self):
        g=LidarAvoidanceGuard(self.narrow_cfg());j=tick(g,width=.8,points=self.narrow_points())
        self.assertFalse(g.latched,g.diagnostics)
        self.assertEqual(g.maneuver.phase,'SHIFT')
        self.assertGreater(g.maneuver.task['goal_d'],0.)
        self.assertLess(g.maneuver.task['goal_d'],.15)
        self.assertLessEqual(g.maneuver.admission['measured_passing_edge_d_m'],.32)
        r=filter_candidates(j);self.assertEqual(r['status'],'VALID',r.get('avoidance'))
        # The path and target use the same body/uncertainty margins.
        self.assertEqual(r['selected']['family'],'maneuver_shift')

    def test_real_narrow_gap_stops_and_keeps_rejection_measurements(self):
        g=LidarAvoidanceGuard(self.narrow_cfg())
        tick(g,width=.8,points=[[2.8,y] for y in (-.125,-.05,0.,.05,.125)])
        self.assertEqual(g.stop_reason,'maneuver_no_side_clearance')
        a=g.first_failure['maneuver']['admission']
        self.assertLess(a['available_channel_m'],a['required_channel_m'])
        self.assertGreaterEqual(a['center_interval_d_m'][0],a['center_interval_d_m'][1])
        self.assertEqual(a['observed_target_sd_m']['d_max'],.125)

    def test_remote_bend_does_not_shrink_the_obstacle_passage(self):
        g=LidarAvoidanceGuard(self.narrow_cfg())
        j=tick(g,width=.8,points=self.narrow_points(),
            left_boundary=[[-1.,.4],[4.,.4],[7.,.2],[8.,.2]])
        self.assertFalse(g.latched,g.diagnostics)
        self.assertAlmostEqual(g.maneuver.admission['road_bounds_sd_m']['high'],.4)
        self.assertEqual(filter_candidates(j)['status'],'VALID')

    def test_local_constriction_and_new_wall_are_not_ignored(self):
        g=LidarAvoidanceGuard(self.narrow_cfg())
        tick(g,width=.8,points=self.narrow_points(),
            left_boundary=[[-1.,.4],[2.3,.4],[2.8,.18],[3.5,.18],[8.,.4]])
        self.assertEqual(g.stop_reason,'maneuver_no_side_clearance')
        g=LidarAvoidanceGuard(self.narrow_cfg());tick(g,width=.8,points=self.narrow_points())
        j=tick(g,now=10.1,width=.8,points=self.narrow_points(),
            left_boundary=[[-1.,.4],[.5,.05],[2.,.05],[4.,.4],[8.,.4]])
        r=filter_candidates(j)
        self.assertEqual(r['status'],'NO_AVOIDANCE_PATH')
        self.assertIn(r['avoidance']['reason'],('maneuver_road_containment','maneuver_road_boundary'))

    def test_normal_reference_survives_slow_replanning_and_rear_left_return(self):
        # The actual field point stopped r1 immediately; the original forward
        # monitor admits it. Keep fresh scans/state while holding a usable path
        # past the old 1.5 s input-age deadline.
        cfg=replace(CFG,front_extent_m=.275,rear_extent_m=.198,width_m=.30)
        g=LidarAvoidanceGuard(cfg)
        for now in (10.,10.9,11.5,12.,13.):
            j=tick(g,now=now,points=[[-.22286008138405408,.1946268196016643]])
            stop=ObstacleStopMonitor(ObstacleStopSettings(enabled=True,
                front_extent_m=.275,rear_extent_m=.198,width_m=.30))
            self.assertFalse(stop.evaluate(j['lidar_avoidance']['obstacles'],now)['stop_required'])
            ref=dict(valid=True,frame_id='base_link',timestamp_s=now,
                planner_version='lattice_execution',original_input_stamp_s=10.,
                path=dict(samples_xy=[[0.,0.],[1.,0.]]))
            decision=g.evaluate(ref,now)
            self.assertFalse(decision['stop_required'],decision)
            self.assertEqual(decision['protection_mode'],'baseline_forward_stop')
            self.assertFalse(g.maneuver.active)

    def test_active_maneuver_keeps_source_deadline_and_collision_checks(self):
        g=LidarAvoidanceGuard(CFG);tick(g)
        ref=dict(valid=True,frame_id='base_link',timestamp_s=10.,
            planner_version='lattice_execution',original_input_stamp_s=8.5,
            path=dict(samples_xy=[[0.,0.],[1.,0.]]))
        self.assertEqual(g.evaluate(ref,10.)['reason'],'avoidance_planner_stale')
        g=LidarAvoidanceGuard(CFG);tick(g)
        tick(g,now=10.1,points=TARGET+[[.4,.05]])
        ref.update(timestamp_s=10.1,original_input_stamp_s=10.)
        self.assertEqual(g.evaluate(ref,10.1)['reason'],'avoidance_execution_collision')

    def test_transport_changes_workload_only_after_a_task_starts(self):
        g=LidarAvoidanceGuard(CFG);j=tick(g,points=ROADSIDE);jobs=[];clock=[10.]
        source=SimpleNamespace(estimation_output=dict(road=j['road'],state=j['state']),
            shadow_settings=ShadowSettings(enabled=True,fusion_enabled=False,
                length_m=1.,rate_hz=1.,collision_coverage='prefix',score_coverage='prefix'),
            planner_control_settings=SimpleNamespace(enabled=False),
            planning_settings=SimpleNamespace(preview_m=.35),lidar_avoidance_guard=g,
            shadow_last_submit_s=-math.inf,shadow_job_id=0,shadow_generation=1,
            shadow_handoff_estimate_s=.1,shadow_worker=SimpleNamespace(submit=jobs.append),
            _monotonic=lambda:clock[0])
        ShadowJobSource._shadow_offer(source)
        self.assertEqual((jobs[-1]['settings']['length_m'],jobs[-1]['settings']['rate_hz']),(1.,1.))
        clock[0]=10.5;ShadowJobSource._shadow_offer(source);self.assertEqual(len(jobs),1)
        j=tick(g,now=10.5);source.estimation_output=dict(road=j['road'],state=j['state'])
        ShadowJobSource._shadow_offer(source);self.assertEqual(len(jobs),2)
        self.assertEqual((jobs[-1]['settings']['length_m'],jobs[-1]['settings']['rate_hz']),(2.5,2.))
        self.assertEqual(jobs[-1]['settings']['collision_coverage'],'generated')
        self.assertEqual(source.shadow_settings.length_m,1.)
        clock[0]=10.75;ShadowJobSource._shadow_offer(source);self.assertEqual(len(jobs),2)
        clock[0]=11.;ShadowJobSource._shadow_offer(source);self.assertEqual(len(jobs),3)

    def test_normal_uses_exact_native_selection(self):
        g=LidarAvoidanceGuard(CFG);j=tick(g,points=ROADSIDE)
        expected=ShadowLatticePlanner(ShadowSettings(**j['settings'])).plan(j)
        actual=filter_candidates(j)
        self.assertEqual(actual['selected'],expected['selected']);self.assertEqual(g.maneuver.phase,'NORMAL')

    def test_active_mode_has_one_dedicated_candidate_and_stable_road_goal(self):
        g=LidarAvoidanceGuard(CFG);j=tick(g);t=deepcopy(g.maneuver.task)
        r=filter_candidates(j)
        self.assertEqual(r['status'],'VALID',r['avoidance']);self.assertEqual(r['candidate_count'],1)
        self.assertEqual(r['selected']['family'],'maneuver_shift')
        tick(g,pose=(.5,.1,.08),now=10.1)
        self.assertEqual(g.maneuver.task['id'],t['id']);self.assertEqual(g.maneuver.task['goal_d'],t['goal_d'])
        self.assertEqual(g.maneuver.task['entry_s'],t['entry_s'])

    def test_all_phases_complete_with_roadside_returns_still_present(self):
        g=LidarAvoidanceGuard(CFG);tick(g);t=g.maneuver.task;goal=t['goal_d'];task_id=t['id']
        tick(g,pose=(2.1,goal,0.),now=10.1);self.assertEqual(g.maneuver.phase,'PASSING')
        tick(g,pose=(3.2,goal,0.),now=10.2,points=ROADSIDE)
        self.assertEqual(g.maneuver.phase,'PASSING')  # Unseen target depth allowance not passed.
        tick(g,pose=(3.6,goal,0.),now=10.3,points=ROADSIDE)
        self.assertEqual(g.maneuver.phase,'RETURN');self.assertFalse(g.latched)
        for k in range(1,11):
            x=3.6+.2*k;s=x+g.maneuver.task['entry_s']
            y=task_offset(g.maneuver.task,s)
            tick(g,pose=(x,y,0.),now=10.3+.01*k,points=ROADSIDE)
        self.assertEqual(g.maneuver.phase,'NORMAL');self.assertEqual(g.side,0)
        self.assertEqual(g.maneuver.last_completed_id,task_id);self.assertTrue(g.points)

    def test_empty_returns_do_not_mean_passed_and_rotated_rear_is_used(self):
        g=LidarAvoidanceGuard(CFG);tick(g);goal=g.maneuver.task['goal_d']
        tick(g,pose=(2.,goal,0.),now=10.1,points=[])
        self.assertEqual(g.maneuver.phase,'PASSING');self.assertTrue(g.points)
        # At yaw .5 the body's low-s corner is farther back than CG-rear.
        tick(g,pose=(3.46,goal,.5),now=10.2,points=[])
        self.assertNotEqual(g.maneuver.phase,'RETURN')

    def test_history_side_has_bounded_source_age_not_renewed_decision_stamp(self):
        g=LidarAvoidanceGuard(CFG);tick(g)
        tick(g,now=10.3,missing='right');self.assertFalse(g.latched)
        tick(g,now=10.7,missing='right')
        self.assertEqual(g.stop_reason,'maneuver_road_support_expired')

    def test_new_environment_obstacle_is_checked_but_not_target_identity(self):
        g=LidarAvoidanceGuard(CFG);tick(g);task_id=g.maneuver.task['id']
        j=tick(g,now=10.1,points=TARGET+[[1.,y] for y in (.25,.35,.45,.55)])
        r=filter_candidates(j)
        self.assertEqual(r['status'],'NO_AVOIDANCE_PATH');self.assertEqual(g.maneuver.task['id'],task_id)
        self.assertIn('lidar_point',r['avoidance']['reason'])

    def test_too_close_target_stops_without_forcing_short_curvature(self):
        g=LidarAvoidanceGuard(CFG);tick(g,points=[[1.,-.05],[1.,.05]])
        self.assertEqual(g.stop_reason,'maneuver_too_close_to_start')

    def test_late_normal_result_cannot_replace_current_task(self):
        g=LidarAvoidanceGuard(CFG);j=tick(g,points=ROADSIDE);old=filter_candidates(j)
        tick(g,now=10.1);e=PlannerExecutionManager(generation=1)
        old.update(generation=1,job_id=1,execution_anchor=dict(pose=[0.,0.,0.],odometry_epoch=0))
        self.assertFalse(accept_avoidance_result(e,g,old));self.assertIsNone(e.active);self.assertFalse(g.latched)

    def test_valid_task_is_admitted_and_unmarked_fallback_is_blocked(self):
        g=LidarAvoidanceGuard(CFG);j=tick(g);r=filter_candidates(j)
        r.update(generation=1,job_id=1,execution_anchor=dict(pose=[0.,0.,0.],odometry_epoch=0))
        e=PlannerExecutionManager(generation=1)
        self.assertTrue(accept_avoidance_result(e,g,r));self.assertTrue(g.maneuver.admitted)
        fallback=dict(valid=True,frame_id='base_link',timestamp_s=10.,path=dict(samples_xy=[[0.,0.],[1.,0.]]))
        self.assertTrue(g.evaluate(fallback,10.)['stop_required'])
        self.assertEqual(g.stop_reason,'maneuver_blocked_fallback')
        first=deepcopy(g.first_failure);g.stop('another_reason');self.assertEqual(first,g.first_failure)

    def test_actual_worker_process_returns_task_protocol(self):
        g=LidarAvoidanceGuard(CFG);j=tick(g)
        result=subprocess.run([sys.executable,'-m','mvp21_policy.worker'],input=json.dumps(j)+'\n',
            capture_output=True,text=True,timeout=10,cwd=Path(__file__).resolve().parents[1])
        self.assertEqual(result.returncode,0,result.stderr)
        r=json.loads(result.stdout);self.assertEqual(r['status'],'VALID',r)
        self.assertEqual(r['avoidance']['maneuver_task_id'],1)
        self.assertEqual(r['selected']['family'],'maneuver_shift')

    def test_single_beam_starts_conservative_target_and_goal_is_not_recomputed(self):
        g=LidarAvoidanceGuard(CFG);tick(g,points=[[2.8,0.]])
        goal=g.maneuver.task['goal_d']
        self.assertGreater(g.maneuver.task['target_d_max'],0.)
        tick(g,now=10.1,points=TARGET)
        self.assertEqual(g.maneuver.task['goal_d'],goal)

    def test_narrow_road_and_lidar_dropout_stop_the_new_mode(self):
        g=LidarAvoidanceGuard(CFG);tick(g,width=.6)
        self.assertEqual(g.stop_reason,'maneuver_no_side_clearance')
        g=LidarAvoidanceGuard(CFG);j=tick(g)
        stale=deepcopy(j['lidar_avoidance']['obstacles'])
        g.update(stale,[0.,0.,0.],0,10.5,road=j['road'])
        self.assertTrue(g.latched);self.assertIn('lidar',g.stop_reason)

    def test_configuration_rejects_invalid_side_and_non_boolean_mode(self):
        for changes in (dict(maneuver_side=0),dict(maneuver_side=True),dict(maneuver_enabled=1)):
            with self.assertRaises(ValueError):replace(CFG,**changes)


if __name__=='__main__':unittest.main()
