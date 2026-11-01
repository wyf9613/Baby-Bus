"""Frozen noisy boundary fixture port; truth is confined to sensing/auditing."""
from copy import deepcopy
from pathlib import Path
import json,math
import numpy as np
from offline.lattice_training.boundary_collection import BoundaryCollectionEnvironment
from offline.lattice_training.screen_roads import paired_cone_noise
from offline.lattice_training.cstar import SolverSettings
from offline.lattice_training.dataset import json_safe,write_json
from .diagnose_perception import query_cones


class BoundaryRuntimeEnvironment(BoundaryCollectionEnvironment):
    def __init__(self,*args,**kwargs):
        self.sensor_audit={}
        super().__init__(*args,**kwargs)

    def _perceive(self,native,*,initial=False):
        audit=None
        if getattr(self,'screen_noise',False):
            if self._backend._camera_pan_config['enabled']:raise ValueError('Boundary noise audit requires fixed camera')
            count=int(native['cone_detections_count'][0]);capacity=len(native['cone_detections_color_ids'])
            pose=list(self._backend.car._state_values()[:3]);co,si=math.cos(pose[2]),math.sin(pose[2])
            indices=[];noise=[]
            for x,y in zip(native['cone_detections_forward_positions_in_body_frame_m'][:count],
                           native['cone_detections_left_positions_in_body_frame_m'][:count]):
                world=np.array([pose[0]+co*x-si*y,pose[1]+si*x+co*y])
                i=int(np.argmin(np.sum((self.screen_cones-world)**2,axis=1)));indices.append(i)
                bias,duplicate,_=paired_cone_noise(self.screen_seed,self._step_count,self.screen_source_ids[i],
                    math.hypot(x,y),self.screen_sigma,0.)
                noise.append(dict(cone_id=i,source_id=self.screen_source_ids[i],radial_error_m=float(bias),duplicate=duplicate))
            identities=list(indices)
            for i,draw in zip(indices,noise):
                if len(identities)<capacity and draw['duplicate']:identities.append(i)
            unlimited=query_cones(self,pose,None)
            audit=dict(tick=self._step_count,acquisition_pose=pose,matched_cone_ids=identities,noise=noise,
                visible_without_cap=int(unlimited['num_cones']),
                visible_without_cap_counts={side:int(np.sum(np.asarray(unlimited['color'])+1==colour))
                    for side,colour in (('left',2),('right',1))})
        result=super()._perceive(native,initial=initial)
        if audit is not None:
            generated=None
            for _,acquisition,payload in self._pending['cone_detections']:
                if acquisition==self._step_count:generated=payload['cone_detections']['detections']
            if generated is None:
                latest=self._latest.get('cone_detections')
                if latest and latest[0]==self._step_count:generated=latest[2]['cone_detections']['detections']
            if generated is None or len(generated)!=len(audit['matched_cone_ids']):
                raise ValueError('Sensor producer audit did not match generated packet')
            audit['detected_raw']=deepcopy(generated)
            audit['stamp_ns']=round((self.clock_start_s+self._step_count*self.dt_s)*1e9)
            self.sensor_audit[audit['stamp_ns']]=audit
        return result

    def export_sensor_audit(self,output):
        with (Path(output)/'sensor-producer.jsonl').open('w',encoding='utf-8') as stream:
            for _,row in sorted(self.sensor_audit.items()):stream.write(json.dumps(json_safe(row),allow_nan=False)+'\n')


def boundary_runtime_environment(case,*,seed=0,progress=None,speed=0.,latency_steps=1,max_steps=2400,
                                 terminal_support_m=2.,physics_hz=20.,perception_hz=20.):
    if (physics_hz,perception_hz,latency_steps)!=(20.,20.,1):
        raise ValueError('This frozen noisy comparison requires 20 Hz sensing/plant and one-frame latency')
    if progress is not None or speed!=0.:raise ValueError('Boundary comparison starts at the entrance at rest')
    if terminal_support_m!=case.get('terminal_sensor_support_m',2.):raise ValueError('Frozen terminal support changed')
    noise=case['measurement_noise']
    env=BoundaryRuntimeEnvironment(case,case['frozen_physical_cones'],sigma=noise['sigma_m'],episode_seed=seed,
        start_index=0,start_count=1,max_steps=max_steps,safety=SolverSettings(safety_margin_m=0.),start_mode='road_start')
    env.reset(seed=seed)
    return env
