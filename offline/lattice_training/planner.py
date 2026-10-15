"""Latest lattice -> policy Frenet contract -> existing full-path C* consumer."""
from copy import deepcopy
import math
from types import SimpleNamespace
from mvp21_policy.candidates import from_lattice
from mvp21_policy.lattice import ShadowLatticePlanner
from mvp21_policy.planning import RoadCenterlinePlanner
from mvp21_policy.transport import ShadowJobSource


class PolicyCandidatePlanner:
    def __init__(self,settings,config):
        self.settings,self.cfg=settings,config
        self.planner=ShadowLatticePlanner(config);self.admission=RoadCenterlinePlanner(settings);self.sequence=0

    def plan(self,road,state,obstacles,vehicle_limits,now_s,source_timeout_s=None,
             collect_candidates=False,candidate_scorer=None):
        self.sequence+=1
        envelope,admission=self.admission.plan(road,state,obstacles,vehicle_limits,now_s,source_timeout_s,mvp=True)
        if not envelope['valid']:return envelope,dict(admission=admission,candidate_references=[],reason=envelope['reason'])
        jobs=[];offer=ShadowJobSource()
        offer.shadow_settings,offer.planning_settings=self.cfg,self.settings
        offer.shadow_last_submit_s=-math.inf;offer.shadow_handoff_estimate_s=self.cfg.handoff_allowance_s
        offer.shadow_generation,offer.shadow_job_id=1,self.sequence-1
        offer._monotonic=lambda:now_s;offer.shadow_worker=SimpleNamespace(submit=jobs.append)
        offer.estimation_output=dict(road=road,state=state);offer._shadow_offer()
        result=self.planner.plan(jobs[0]);choices=[]
        for proposal in result['candidates']:
            if proposal['collision'] is not None:continue
            contract=from_lattice(proposal,result);length=contract['samples'][-1][0]
            speed=self.settings.cruise_speed_mps
            if speed<=0:raise ValueError('Candidate training requires positive authored cruise speed')
            initial_speed=max(0.,float(state['speed_mps']))
            # The native planner authors space only. This explicit offline time
            # law starts at measured speed and changes it uniformly over the
            # whole path; it must not introduce an instantaneous speed step.
            duration=2*length/(initial_speed+speed);ref=deepcopy(envelope)
            ref.update(planner_version='mvp21_lattice_policy_frenet',planned_stop=False,
                path=dict(type='ORDERED_CENTERLINE',independent_variable='s_m',samples_xy=[list(p[:2]) for p in proposal['path']],
                    range=[0.,length],preview_m=self.settings.preview_m),
                frenet_path=dict(type='SAMPLED_FRENET',candidate=contract),
                longitudinal_profile=dict(type='P_PATH_OF_TIME',duration_s=duration,
                    coeffs_low_to_high=[0.,initial_speed*duration,(speed-initial_speed)*duration/2],
                    stop=False,source='explicit_policy_uniform_acceleration_profile'),
                speed_profile=[[0.,initial_speed],[duration,speed]])
            entry=dict(candidate_id=str(proposal['id']),reference=ref,trajectory_xy_m=[list(p[:2]) for p in proposal['path']],
                descriptor=dict(length_m=length,offset_m=contract['samples'][-1][2],duration_s=duration,
                    terminal_speed_mps=speed,score=-proposal['cost'],family=proposal['family']),
                collision_check=dict(method=result['collision_method'],known_boundary_sides=result['known_boundary_sides'],
                    check_length_m=result['check_length_m'],continuous_sweep_certified=False))
            if candidate_scorer is not None:
                score=float(candidate_scorer(entry))
                if not math.isfinite(score):raise ValueError('Nonfinite learned candidate score')
                entry['descriptor']['score']=score
            choices.append(entry)
        choices.sort(key=lambda c:(-c['descriptor']['score'],int(c['candidate_id'])))
        if not choices:
            envelope.update(valid=False,path=None,stop_requested=True,target_speed_mps=0.,reason='lattice_'+result['status'])
            return envelope,dict(reason=envelope['reason'],admission=admission,result=result,candidate_references=[])
        return choices[0]['reference'],dict(reason='lattice_candidates_available',admission=admission,result=result,
            selected_candidate_id=choices[0]['candidate_id'],candidate_references=choices if collect_candidates else [])
