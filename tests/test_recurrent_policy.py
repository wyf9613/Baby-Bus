from copy import deepcopy
from dataclasses import replace
import unittest
from unittest.mock import patch
from pathlib import Path
import tempfile
import numpy as np

from offline.lattice_training.features import encode_candidate
from offline.lattice_training.spatial import input_matrices
from offline.lattice_training.model import SpatialPathPolicy
from offline.lattice_training.sequence import sequence_layout,forward_sequence,backward_sequence
from offline.lattice_training.objective import CStarTrackingSettings,tracking_loss
from offline.lattice_training.scenarios import expanded_road_spec,vary_road
from offline.lattice_training.roads import road_length
from offline.lattice_training.pipeline import make_environment,make_planner,candidates_at,sample_successor
from offline.lattice_training.cstar import CStarSolver,SolverSettings,execute_trial,performance_profile
from tests.test_lattice_training import candidate,decision_records
from dreamgym.envs import Road


class RecurrentPolicyChecks(unittest.TestCase):
    def examples(self):
        obs=decision_records()[0]['observation']
        matrices=[input_matrices(encode_candidate(obs,candidate(d))) for d in (0.,.1,-.1,.05)]
        groups=np.asarray(['a','a','b','b'])
        context=dict(episode_ids=np.asarray(['road']*4),sequence_steps=np.asarray([0,0,1,1]),
                     behavior_selected=np.asarray([False,True,True,False]))
        return matrices,sequence_layout(groups,context)

    def test_only_executed_branch_supplies_next_decision_state(self):
        matrices,layout=self.examples();model=SpatialPathPolicy(hidden=3,seed=4,recurrent=True)
        outputs=forward_sequence(model.network,matrices,layout)
        np.testing.assert_array_equal(outputs[0][1][-1],outputs[1][1][-1])
        np.testing.assert_allclose(outputs[2][1][-1],outputs[1][1][7])
        np.testing.assert_array_equal(outputs[2][1][-1],outputs[3][1][-1])
        other=forward_sequence(model.network,matrices,[[(np.asarray([0,1]),0),(np.asarray([2,3]),2)]])
        self.assertNotEqual(outputs[2][0],other[2][0])

    def test_saved_recurrent_selector_defers_and_commits_only_executed_job_once(self):
        from offline.mvp21_sim.policy_selection import FrozenPolicySelector,DEFAULT_MODEL
        from tests.test_runtime_policy_selection import job,observation
        from mvp21_policy.lattice import ShadowLatticePlanner,ShadowSettings
        previous=SpatialPathPolicy.load(DEFAULT_MODEL)
        model=SpatialPathPolicy(hidden=previous.network.hidden,recurrent=True)
        model.provenance=deepcopy(previous.provenance)
        for name,value in previous.parameters.items():model.parameters[name]=value.copy()
        model.normalization=deepcopy(previous.normalization)
        encoded=encode_candidate(decision_records()[0]['observation'],candidate(.1))
        self.assertAlmostEqual(model.step(encoded,model.initial_state())[0],previous.score(encoded),places=15)
        with tempfile.TemporaryDirectory() as temporary:
            target=Path(temporary)/'model.json';model.save(target)
            selector=FrozenPolicySelector(target)
            result=ShadowLatticePlanner(ShadowSettings(**job()['settings'])).plan(job())
            result['job_id']=job()['job_id']
            before=selector.hidden_state.copy()
            selected=selector.select(deepcopy(result),observation())
            self.assertEqual(selected['policy_selection']['status'],'SELECTED',selected['policy_selection'])
            np.testing.assert_array_equal(before,selector.hidden_state)
            reversed_result=deepcopy(result);reversed_result['candidates'].reverse()
            other=selector.select(reversed_result,observation())
            self.assertEqual(selected['selected']['id'],other['selected']['id'])
            selector.commit(999);np.testing.assert_array_equal(before,selector.hidden_state)
            selector.commit(result['job_id'])
            np.testing.assert_allclose(selector.hidden_state,selected['policy_selection']['hidden_selected'])
            self.assertEqual(selector.state_commits,1)
            selector.commit(result['job_id']);self.assertEqual(selector.state_commits,1)

    def test_bptt_gradients_include_scores_of_all_candidates_and_executed_history(self):
        matrices,layout=self.examples();model=SpatialPathPolicy(hidden=3,seed=4,recurrent=True)
        derivatives=np.asarray([.2,-.1,.8,-.3])
        outputs=forward_sequence(model.network,matrices,layout)
        gradients=backward_sequence(model.network,outputs,derivatives,layout)
        for name,values in model.parameters.items():
            index=np.unravel_index(np.argmax(np.abs(gradients[name])),values.shape)
            before=values[index];epsilon=1e-5
            values[index]=before+epsilon;plus=np.dot([v for v,_ in forward_sequence(model.network,matrices,layout)],derivatives)
            values[index]=before-epsilon;minus=np.dot([v for v,_ in forward_sequence(model.network,matrices,layout)],derivatives)
            values[index]=before
            self.assertAlmostEqual((plus-minus)/(2*epsilon),gradients[name][index],delta=2e-7,msg=name)

    def test_sequences_reject_skipped_decisions_or_unexecuted_history(self):
        for steps,selected in (([0,0,2,2],[True,False,True,False]),([0,0,1,1],[False]*4)):
            with self.assertRaises(ValueError):sequence_layout(np.asarray(['a','a','b','b']),
                dict(episode_ids=np.asarray(['road']*4),sequence_steps=np.asarray(steps),behavior_selected=np.asarray(selected)))

    def test_checkpoint_prioritizes_avoidable_failure_then_actual_return(self):
        from offline.lattice_training.model import decision_checkpoint_metrics,checkpoint_key
        groups=np.asarray(['a','a','b','b']);returns=np.asarray([1.,-1e6,-1e6,-1e6]);feasible=np.asarray([True,False,False,False])
        bad=decision_checkpoint_metrics(groups,returns,[.4,.5,.2,.3],feasible)
        good=decision_checkpoint_metrics(groups,returns,[.5,.4,.2,.3],feasible)
        self.assertEqual(bad['validation_avoidable_failures'],1)
        self.assertEqual(good['validation_avoidable_failures'],0)
        self.assertEqual(good['validation_decisions_with_feasible'],1)
        self.assertLess(checkpoint_key(dict(good,validation_rmse=.4),'validation_feasible_then_return'),
            checkpoint_key(dict(bad,validation_rmse=.1),'validation_feasible_then_return'))
        first=decision_checkpoint_metrics(np.asarray(['a']*2),[1.,2.],[.8,.3],np.asarray([True,True]))
        second=decision_checkpoint_metrics(np.asarray(['a']*2),[1.,2.],[.3,.8],np.asarray([True,True]))
        self.assertLess(checkpoint_key(dict(second,validation_rmse=.4),'validation_feasible_then_return'),
            checkpoint_key(dict(first,validation_rmse=.1),'validation_feasible_then_return'))

    def test_rank_stagnation_stops_training_even_with_rmse_tiebreak_changes(self):
        observation=decision_records()[0]['observation']
        inputs=np.asarray([encode_candidate(observation,candidate(0.))]*2,dtype=object)
        model=SpatialPathPolicy(hidden=3,seed=4)
        history=model.fit(inputs,np.asarray([1.,1.]),np.asarray(['train']*2),inputs,np.asarray([1.,1.]),
            epochs=10,validation_groups=np.asarray(['validation']*2),validation_feasible=np.asarray([True,True]),
            checkpoint='validation_feasible_then_return',early_stopping_patience=2,
            learning_rate=1e-12)
        self.assertEqual(history[0]['epoch'],0);self.assertEqual(history[-1]['epoch'],2)
        self.assertEqual(model.training_stop_reason,'validation_checkpoint_stagnation')
        self.assertTrue(all(row['validation_avoidable_failures']==0 for row in history))
        self.assertEqual(model.best_checkpoint_key[:2],(0,0.))

    def test_progress_bonus_is_signed_road_station_increment(self):
        states=np.asarray([[0.,0.,0.,.2],[.1,0.,0.,.2]])
        actions=np.zeros((1,2));cfg=CStarTrackingSettings(road_progress_weight=2.)
        before,_=tracking_loss(states,states,actions,[0,0],cfg,road_progress_m=[5.,5.])
        after,terms=tracking_loss(states,states,actions,[0,0],cfg,road_progress_m=[5.,5.3])
        self.assertAlmostEqual(before-after,.6);self.assertAlmostEqual(terms['road_progress'],-.6)
        with self.assertRaises(ValueError):tracking_loss(states,states,actions,[0,0],cfg)
        self.assertEqual(CStarTrackingSettings.from_specification(cfg.specification()),cfg)

    def test_behavior_is_per_decision_half_random_half_native_and_does_not_hide_native_failure(self):
        choices=[dict(candidate_id=str(i)) for i in range(3)]
        solved={str(i):dict(status='approximate') for i in range(3)}
        rng=np.random.default_rng(43);counts=dict(random=0,planner=0)
        with patch('offline.lattice_training.pipeline.simulate_segment',return_value=({'state':1},{})):
            for _ in range(500):
                chosen,_,info=sample_successor(None,choices,solved,[0,0],None,None,rng,3,random_probability=.5)
                counts[info['behavior']]+=1
                if info['behavior']=='planner':self.assertEqual(chosen['candidate_id'],'0')
        self.assertGreater(counts['random'],200);self.assertGreater(counts['planner'],200)
        solved['0']['status']='failed'
        chosen,_,info=sample_successor(None,choices,solved,[0,0],None,None,rng,3,random_probability=0.)
        self.assertIsNone(chosen);self.assertEqual(info['behavior'],'planner')

    def test_width_and_physical_single_side_gaps_are_seeded_and_keep_road_geometry(self):
        original=expanded_road_spec('suite_left',71)
        road,info=vary_road(original,71,width_range_m=(.8,1.4),dropout_probability=1.)
        self.assertTrue(info['physical_cone_gaps']);self.assertGreater(info['width_m'],.8)
        self.assertEqual((road,info),vary_road(original,71,width_range_m=(.8,1.4),dropout_probability=1.))
        self.assertAlmostEqual(road_length(road),road_length(original))
        actual=Road(road_spec=road).resolve_cones(0)
        complete,_=vary_road(original,71,width_range_m=(.8,1.4),dropout_probability=0.)
        self.assertLess(len(actual.records),len(Road(road_spec=complete).resolve_cones(0).records))

    def test_speed_half_meter_and_progress_labels_use_actual_solver_and_replay(self):
        cfg=CStarTrackingSettings(road_progress_weight=1.)
        safety=SolverSettings(max_evaluations=6,iterations=1,population=4,max_wall_s=30)
        with make_environment('straight',0,max_steps=1000) as env:
            env.reset(seed=0);obs,*_=env.step((0,0))
            _,diag=candidates_at(make_planner(cruise_speed=.5,sample_count=8,rate_hz=20),obs)
            proposal=min(diag['candidate_references'],key=lambda c:abs(c['descriptor']['offset_m']))
            self.assertEqual(proposal['reference']['target_speed_mps'],.5)
            solution=CStarSolver(cfg,safety).solve(env,proposal)
            self.assertEqual(solution['status'],'approximate',solution)
            trial=execute_trial(env,proposal,solution,[0,0],cfg,safety,trial_id='nominal',profile=performance_profile(0,nominal=True))
            self.assertTrue(trial['constraints_satisfied'],trial.get('reason'))
            self.assertGreater(trial['road_progress_m'][-1],trial['road_progress_m'][0])
            self.assertIn('road_progress',trial['actual_loss_terms'])

    def test_evaluation_uses_collection_candidate_budget_and_terminal_rule(self):
        from offline.lattice_training.evaluation import RoadEvaluation
        from offline.lattice_training.pipeline import road_goal_progress
        cfg=CStarTrackingSettings(road_progress_weight=1.)
        safety=SolverSettings(max_evaluations=6,iterations=1,population=4,max_wall_s=30)
        with make_environment('straight',0,max_steps=1000) as env:
            env.reset(seed=0);env.step((0,0))
            evaluator=RoadEvaluation(env,objective=cfg,solver_settings=safety,cruise_speed=.5,
                planner_sample_count=8,planner_rate_hz=2.,balance_terminal=True)
            report=evaluator.run(max_control_steps=1)
            self.assertEqual(report['planner_profile']['sample_count'],8)
            self.assertEqual(report['planner_profile']['rate_hz'],2.)
            self.assertTrue(report['planner_profile']['balance_terminal_horizon'])
            self.assertEqual(report['planner_profile']['goal_tolerance_m'],cfg.completion_tolerance_m)
            self.assertLessEqual(len(report['candidate_panels'][0]['candidates']),8)
            self.assertLess(report['planner_profile']['length_m'],1.5)
            goal=float(env._info['evaluation']['ground_truth']['reference_line_progress_m'])+.05
            near_goal=RoadEvaluation(env,objective=cfg,solver_settings=safety,balance_terminal=True).run(goal_progress=goal)
            self.assertTrue(near_goal['road_completed']);self.assertFalse(near_goal['candidate_panels'])

    def test_stratified_starts_reuse_one_physical_uturn_and_draw_once_per_stratum(self):
        from offline.lattice_training.pipeline import PipelineSettings,initialize_episode,road_goal_progress
        from offline.lattice_training.roads import road_identity
        settings=PipelineSettings(kinds=('uturn_left',),starts_per_road=4,
            initial_start_mode='stratified_random',width_range_m=(.8,1.4),single_side_dropout_probability=.65,
            terminal_support_m=2.)
        identities=[];starts=[]
        for index in range(4):
            env,_,initialization=initialize_episode('uturn_left',1800,index,settings,CStarTrackingSettings(),None)
            with env:
                draw=initialization['initial_sampling'];start=draw['drawn_progress_m'];low,high=draw['stratum_bounds_m']
                self.assertTrue(low<=start<=high)
                self.assertAlmostEqual(draw['maximum_progress_m'],road_goal_progress(env)-1.5)
                self.assertEqual(draw['draws'],1);self.assertEqual(initialization['initial_state_draws'],1)
                self.assertFalse(initialization['filtered_by_planner'])
                self.assertEqual(initialization['known_lane_width_prior_m'],0.)
                identities.append(road_identity(env.config.road_spec));starts.append(start)
        self.assertEqual(len(set(identities)),1);self.assertEqual(starts,sorted(set(starts)))
        with make_environment('uturn_left',1800,max_steps=100,initial_index=3,initial_count=4,
                initial_start_mode='stratified_random',width_range_m=(.8,1.4),single_side_dropout_probability=.65,
                terminal_support_m=2.) as repeated:
            self.assertEqual(repeated.collection_initial_sampling['drawn_progress_m'],starts[-1])


if __name__=='__main__':unittest.main()
