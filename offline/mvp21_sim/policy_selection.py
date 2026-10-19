"""Frozen single-path policy ranking at the offline native worker boundary."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import time

from mvp21_policy.candidates import from_lattice
from offline.lattice_training.features import encode_candidate, scene_features
from offline.lattice_training.model import SpatialPathPolicy


DEFAULT_MODEL=Path(__file__).resolve().parents[1]/'lattice_training/artifacts/2026-10-17-expanded-collection/model.json'


class FrozenPolicySelector:
    def __init__(self, path, *, cruise_speed_mps=None):
        self.path=Path(path).resolve()
        self.sha256=hashlib.sha256(self.path.read_bytes()).hexdigest()
        self.model=SpatialPathPolicy.load(self.path)
        condition=self.model.provenance['execution_condition']
        if cruise_speed_mps is not None and not math.isclose(cruise_speed_mps,condition['cruise_speed_mps'],abs_tol=1e-9,rel_tol=0.):
            raise ValueError('Untimed policy requires the frozen training cruise-speed condition')
        self.ranking_calls=self.candidate_calls=self.errors=0
        self.hidden_state=self.model.initial_state();self.pending_states={};self.committed_job=None;self.state_commits=0

    def commit(self,job_id):
        """Advance memory once when the shared execution port uses this job."""
        if not self.model.network.recurrent or job_id==self.committed_job or job_id not in self.pending_states:return
        self.hidden_state=self.pending_states[job_id].copy();self.committed_job=job_id;self.state_commits+=1
        self.pending_states={key:value for key,value in self.pending_states.items() if key>job_id}

    def provenance(self):
        return dict(model_path=str(self.path),model_sha256=self.sha256,
            input_groups=['current_state','current_road_and_obstacles','candidate_path'],
            input_scene='submission_snapshot_of_actual_estimator_output',
            candidate_input='original_frenet_and_authored_cartesian_geometry',
            planner_cost_input=False,history_feature_input=False,speed_curve_input=False,
            target=self.model.provenance['target'],
            training_execution_condition=self.model.provenance['execution_condition'],
            training_objective=self.model.provenance['objective'],
            score_semantics=self.model.prediction_space,tie_break='lowest_numeric_candidate_id',
            recurrent_state_enabled=self.model.network.recurrent,
            recurrent_state_commit='first_actual_execution_of_selected_job',
            retrained=False,simulation_only=True)

    def select(self, result, observation):
        started=time.perf_counter()
        native_id=(result.get('selected')or{}).get('id')
        info=dict(backend='policy',model_sha256=self.sha256,native_selected_id=native_id,
            selected_id=None,scores=[],status='SKIPPED',reason='native_result_not_valid')
        result['policy_selection']=info
        if result.get('status')!='VALID':return result
        feasible=[c for c in result.get('candidates',[]) if c.get('collision') is None]
        self.ranking_calls+=1
        try:
            if not feasible:raise ValueError('Native VALID result contains no admitted candidate')
            context=scene_features(observation)
            next_states={}
            for candidate in feasible:
                # The existing encoder reads only this spatial contract. Never
                # synthesize a timed C* reference for runtime model inference.
                proposal=dict(reference=dict(frenet_path=dict(type='SAMPLED_FRENET',
                    candidate=from_lattice(candidate,result))))
                encoded=encode_candidate(observation,proposal,context=context)
                self.candidate_calls+=1
                score,state=(self.model.step(encoded,self.hidden_state) if self.model.network.recurrent else
                             (float(self.model.score(encoded)),None))  # exactly one path per call
                next_states[candidate['id']]=state
                if not math.isfinite(score):raise ValueError('Nonfinite policy score')
                digest=hashlib.sha256(json.dumps(encoded,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
                info['scores'].append(dict(candidate_id=candidate['id'],score=score,input_sha256=digest))
            scores={row['candidate_id']:row['score'] for row in info['scores']}
            selected=max(feasible,key=lambda c:(scores[c['id']],-int(c['id'])))
            result['selected']=selected
            if self.model.network.recurrent:
                self.pending_states[result['job_id']]=next_states[selected['id']].copy()
                info['hidden_before']=self.hidden_state.tolist();info['hidden_selected']=next_states[selected['id']].tolist()
            ordered=sorted(scores.values(),reverse=True)
            info.update(status='SELECTED',reason='highest_frozen_policy_score',selected_id=selected['id'],
                changed_from_native=selected['id']!=native_id,feasible_count=len(feasible),
                best_score=ordered[0],score_gap=ordered[0]-ordered[1] if len(ordered)>1 else None,
                tied_best_count=sum(v==ordered[0] for v in ordered))
        except Exception as error:
            # Keep the native result/candidates as diagnostics, but never silently
            # execute the native pick after a policy inference/contract failure.
            result.update(status='POLICY_ERROR',selected=None)
            self.errors+=1
            info.update(status='ERROR',reason=type(error).__name__+': '+str(error))
        info['wall_s']=time.perf_counter()-started
        return result


def scoring_observation(observation):
    estimates=observation['estimation']
    return dict(frame_id=observation.get('frame_id','base_link'),timestamp_s=observation['timestamp_s'],
        estimation=deepcopy({key:estimates[key] for key in ('state','road','obstacles')}))
