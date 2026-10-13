"""Offline road-only evaluation of shipped MVP2, independent of planner costs.

Run with normal Python + numpy/matplotlib. No source/config modifications.
The scalar YAML reader below only reads this repository's nested scalar config.
"""
from collections import Counter
from dataclasses import replace
from pathlib import Path
import argparse, hashlib, json, math, platform, random, runpy, statistics, time
import numpy as np
import mvp2_quality_oracle as q

ROOT = Path(__file__).resolve().parents[1]
NS = q.NS

def scalar_config(text):
    result = {}; stack = [(-1, result)]
    for line in text.splitlines():
        raw = line.split('#', 1)[0].rstrip()
        if not raw.strip(): continue
        indent = len(raw)-len(raw.lstrip()); key, value = raw.strip().split(':', 1)
        while stack[-1][0] >= indent: stack.pop()
        parent = stack[-1][1]; value = value.strip()
        if not value:
            parent[key] = {}; stack.append((indent, parent[key])); continue
        try: value = json.loads(value)
        except json.JSONDecodeError: pass
        parent[key] = value
    return result

PARAMS = scalar_config((ROOT/'config/ai4r_policy.yaml').read_text())['/**/ai4r_policy']['ros__parameters']
def settings(cls, name):
    values = PARAMS['planning']['lattice'] if name == 'lattice' else PARAMS[name]
    return NS[cls](**{k:v for k,v in values.items() if k in vars(NS[cls]())})

CONTROL = settings('ControlSettings','control')
BASE = settings('LatticeSettings','lattice')
PLANNING = settings('PlanningSettings','planning')

# Extend the historical independent oracle to straight and mirrored curves.
OLD_TRUTH = q.truth
def truth(name, x):
    x = np.asarray(x)
    if name == 'straight': return np.zeros_like(x), np.zeros_like(x)
    return OLD_TRUTH(name, x)
q.truth = truth
SCENES = ['straight','curve_0.04','curve_-0.04','curve_0.12','curve_-0.12','curve_0.25','s_bend','curvature_change']
q.SCENES = SCENES

def planner(budget):
    return NS['FrenetLatticePlanner'](PLANNING, replace(BASE,budget_s=budget,obstacle_check_enabled=False))

def args_for(name, pose, now, width=1., support=3.5, noise=0., frame=0, speed=.1, delta=0.):
    args = list(q.records(name, pose, speed, delta, now, width=width))
    road = args[0]; road['measurement_timestamp_s'] = now-.01
    for key, side in [('left_boundary_xy',1),('right_boundary_xy',-1),('centerline_xy',0)]:
        road[key] = [(x,y+noise*math.sin(2*x+frame*.73)) for x,y in road[key] if x<=support]
    args[1]['valid'] = True
    args.append({'road':.4,'wheel_speed':.15,'imu_angular_velocity':.15,'obstacles':.4})
    return args

def call(p,args,secondary=False):
    before = time.perf_counter()
    if secondary: ref,diag = p.plan(*args,mvp=True)
    else: ref,diag = NS['_mvp_plan'](p,CONTROL,args)
    elapsed = time.perf_counter()-before
    return ref, {'wall_ms':elapsed*1000,'internal_ms':diag.get('elapsed_s',0)*1000,
        'valid':ref['valid'],'on_time':elapsed<=.035,'reason':ref['reason'],
        'planner':diag.get('planner','lattice'),'centerline_rejection':diag.get('centerline_rejection'),
        'candidate_rows':len(diag.get('candidates',[])), 'truncated':diag.get('search_truncated',False)}

def stats(rows):
    values = [r['wall_ms'] for r in rows]
    return {'n':len(rows),'median_ms':float(np.median(values)),'p95_ms':float(np.quantile(values,.95)),
        'p99_ms':float(np.quantile(values,.99)),'max_ms':max(values),
        'valid_on_time':sum(r['valid'] and r['on_time'] for r in rows),'valid':sum(r['valid'] for r in rows),
        'over35':sum(not r['on_time'] for r in rows),'reasons':dict(Counter(r['reason'] for r in rows)),
        'routes':dict(Counter(r['planner'] for r in rows))}

def path_difference(a,b):
    # Both paths already in fixed WORLD coordinates; x-monotonic only.
    if np.any(np.diff(a[:,0])<=0) or np.any(np.diff(b[:,0])<=0): return None
    lo=max(a[0,0],b[0,0]); hi=min(a[-1,0],b[-1,0])
    if hi-lo<.2: return None
    xs=np.linspace(lo,hi,math.ceil((hi-lo)/.005)+1)
    dy=np.interp(xs,b[:,0],b[:,1])-np.interp(xs,a[:,0],a[:,1])
    dh=np.unwrap(np.interp(xs,b[:,0],np.unwrap(b[:,2]))-np.interp(xs,a[:,0],np.unwrap(a[:,2])))
    return {'rms_gap_m':float(np.sqrt(np.mean(dy**2))), 'max_gap_m':float(np.max(np.abs(dy))),
            'max_heading_gap_deg':float(np.degrees(np.max(np.abs(dh)))), 'overlap_m':hi-lo}

def audit(ref,pose,oracle):
    local, world=q.dense_path(ref,pose)
    return {'min_clearance_m':oracle.clearance(world),
            'length_m':float(np.linalg.norm(np.diff(local,axis=0),axis=1).sum())},world

def statics(repeats,rng,oracles):
    cases=[{'name':n,'scene':n,'width':1.,'support':3.5} for n in SCENES]
    cases += [{'name':'narrow_0.6','scene':'curve_0.12','width':.6,'support':3.5},
              {'name':'wide_1.4','scene':'curve_0.12','width':1.4,'support':3.5},
              {'name':'short_1.0','scene':'straight','width':1.,'support':1.},
              {'name':'short_0.5','scene':'straight','width':1.,'support':.5}]
    initials=[(0.,0.),(-.15,0.),(.15,0.),(0.,-.52),(0.,.52)]
    rows=[]; cache={}
    jobs=[(case,init,mode) for case in cases for init in initials for mode in ('primary','secondary')]
    for rep in range(repeats):
        rng.shuffle(jobs)
        for case,init,mode in jobs:
            pose=(0.,*init); args=args_for(case['scene'],pose,10.,case['width'],case['support'])
            pair={}; worlds={}
            order=[.035,5.]; rng.shuffle(order)
            for budget in order:
                ref,meta=call(planner(budget),args,mode=='secondary')
                if ref['valid']:
                    token=(case['scene'],case['width'],pose,json.dumps(ref['path'],sort_keys=True))
                    # Truth auditing excluded from timed section; deduplicate identical output.
                    if token not in cache: cache[token]=audit(ref,pose,q.Oracle(case['scene'],case['width']))
                    meta['quality'],worlds[budget]=cache[token]
                    meta['quality']=dict(meta['quality'])
                pair[str(budget)]=meta
            gap=path_difference(worlds[.035],worlds[5.]) if len(worlds)==2 else None
            rows.append({'case':case['name'],'scene':case['scene'],'offset':init[0],'heading':init[1],
                         'mode':mode,'pair':pair,'difference':gap})
        if (rep+1)%5==0: print('static repeat',rep+1,flush=True)
    return rows

def sequence(name,budget,noise,oracles,steps=120):
    p=planner(budget); rows=[]; transitions=[]; previous=None; lastspeed=None
    for i in range(steps):
        x=i*.02; y,slope=truth(name,x); pose=(x,float(y)+.05,math.atan(float(slope)))
        ref,meta=call(p,args_for(name,pose,10+i*.1,noise=noise,frame=i))
        row={'step':i,**meta}; rows.append(row)
        if not ref['valid'] or (budget==.035 and not meta['on_time']):
            previous=None; continue
        _,world=q.dense_path(ref,pose)
        if previous is not None:
            gap=path_difference(previous,world)
            if gap: transitions.append({'step':i,**gap,'target_speed_jump_mps':abs(ref['target_speed_mps']-lastspeed)})
        previous=world; lastspeed=ref['target_speed_mps']
    return {'scene':name,'budget':budget,'noise_m':noise,'calls':rows,'transitions':transitions,'summary':stats(rows)}

def loop(name,offset,budget,oracle,steps):
    p=planner(budget); controller=NS['PolicyController'](CONTROL)
    _,slope=truth(name,0.); pose=(0.,offset,math.atan(float(slope))); speed=.1;delta=0.
    trace=[];calls=[]; transitions=[]; previous=None;fail=None; audits=[]
    for i in range(steps):
        args=args_for(name,pose,10+i*.05,speed=speed,delta=delta)
        ref,meta=call(p,args); calls.append(meta)
        if not ref['valid'] or (budget==.035 and not meta['on_time']):
            fail={'step':i,'reason':ref['reason'] if not ref['valid'] else 'valid_but_late'};break
        _,world=q.dense_path(ref,pose)
        if previous is not None:
            gap=path_difference(previous,world)
            if gap: transitions.append(gap)
        previous=world
        if i%20==0: audits.append(oracle.clearance(world))
        drive,steer,diag=controller.calculate(ref,args[1],{},args[4]+meta['wall_ms']/1000,.05)
        if not diag['valid'] or diag['stop_requested']:
            fail={'step':i,'reason':diag['reason']};break
        requested=steer*CONTROL.mvp_steering_direction*math.pi/4
        delta+=float(np.clip(requested-delta,-math.pi/2*.05,math.pi/2*.05))
        # Geometry test: ideal target-speed lag, no ESC calibration claim.
        target=ref['target_speed_mps']
        for _ in range(10):
            speed+=(target-speed)*(1-math.exp(-.005/.2))
            beta=math.atan(.4*math.tan(delta)); yaw=speed/.33*math.cos(beta)*math.tan(delta)
            pose=(pose[0]+speed*math.cos(pose[2]+beta)*.005,pose[1]+speed*math.sin(pose[2]+beta)*.005,pose[2]+yaw*.005)
        error,progress=oracle.tracking(pose[0],pose[1])
        trace.append({'x':pose[0],'y':pose[1],'heading':pose[2],'error_m':error,'progress_m':progress,
                      'requested_rad':requested,'speed_mps':speed})
    errors=np.array([r['error_m'] for r in trace]);progress=np.array([r['progress_m'] for r in trace])
    req=np.array([r['requested_rad'] for r in trace]); changes=np.diff(req)
    signs=np.sign(changes[np.abs(changes)>math.radians(.5)])
    spatial=None
    if len(progress) and max(progress)>=1:
        e0,p0=oracle.tracking(0,offset)
        unique,idx=np.unique(np.r_[p0,progress],return_index=True)
        if unique[0]<=0: spatial=float(np.sqrt(np.mean(np.interp(np.linspace(0,1,101),unique,np.r_[e0,errors][idx])**2)))
    poses=[[r['x'],r['y'],r['heading']] for r in trace]
    metrics={'steps':len(trace),'failure':fail,'progress_m':float(progress[-1]) if len(progress) else 0,
             'spatial_rmse_first_1m_m':spatial,'final_error_m':float(errors[-1]) if len(errors) else None,
             'p95_abs_error_m':float(np.quantile(np.abs(errors),.95)) if len(errors) else None,
             'min_actual_clearance_m':oracle.clearance(poses) if poses else None,
             'min_output_clearance_m':min(audits) if audits else None,
             'max_command_jump_deg':float(np.degrees(max(abs(changes)))) if len(changes) else None,
             'command_reversals':int(sum(signs[1:]!=signs[:-1])) if len(signs)>1 else 0}
    print('loop',name,offset,budget,metrics,flush=True)
    return {'scene':name,'offset':offset,'budget':budget,'metrics':metrics,'calls':calls,'trace':trace,'transitions':transitions}

def worker_check(cases):
    worker=NS['AsyncLatticePlanner'](); results=[]
    try:
        start=time.perf_counter()
        while not worker.ready and time.perf_counter()-start<10:
            ready=worker.poll()
            if ready and ready.get('ready'): worker.ready=True
            time.sleep(.001)
        startup=time.perf_counter()-start
        if not worker.ready: return {'ready':False,'startup_s':startup}
        for name in cases:
            for i in range(20):
                args=args_for(name,(0.,0.,0.),10+i*.1)
                job={'generation':1,'frame_id':'base_link','planning':vars(PLANNING),'lattice':vars(BASE),'control':vars(CONTROL),'args':args}
                start=time.perf_counter();worker.submit(job,start); result=None
                while result is None and time.perf_counter()-start<1:
                    result=worker.poll()
                    if result is None: time.sleep(.001)
                results.append({'scene':name,'roundtrip_ms':(time.perf_counter()-start)*1000,
                    'internal_ms':(result or {}).get('diagnostics',{}).get('elapsed_s',0)*1000,
                    'valid':(result or {}).get('reference',{}).get('valid',False),
                    'reason':(result or {}).get('reference',{}).get('reason',(result or {}).get('error'))})
        return {'ready':True,'startup_s':startup,'rows':results,'note':'Actual subprocess + pipes; polling at 1 ms; no ROS scheduling or motion transport included'}
    finally: worker.close()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--repeats',type=int,default=20);parser.add_argument('--steps',type=int,default=240)
    parser.add_argument('--output',type=Path,default=ROOT/'.verification/mvp2-evaluation/results');opt=parser.parse_args();opt.output.mkdir(parents=True,exist_ok=True)
    protected=[ROOT/'scripts/policy_node.py',ROOT/'config/ai4r_policy.yaml'];hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in protected}
    rng=random.Random(20261009); oracles={name:q.Oracle(name) for name in SCENES}
    result={'metadata':{'platform':platform.platform(),'python':platform.python_version(),'source_hashes':hashes,'seed':20261009,
        'settings':{'planning':vars(PLANNING),'lattice':vars(BASE),'control':vars(CONTROL)},
        'limitations':'Synthetic geometry, perfect motion feedback, approximate bicycle, ideal speed lag; wall time not propagated to plant; direct geometry loops have no cache. No ROS/hardware.'}}
    # QA historical dense/continuous projection against exhaustive grid.
    result['metadata']['oracle_qa']=q.oracle_qa(); print('Oracle QA passed',flush=True)
    result['static']=statics(opt.repeats,rng,oracles)
    def save(): (opt.output/'results.json').write_text(json.dumps(result),encoding='utf-8')
    save()
    result['sequences']=[]
    jobs=[(name,budget,noise) for name in SCENES for budget in (.035,5.) for noise in (0.,.005)]
    rng.shuffle(jobs)
    for name,budget,noise in jobs:
        run=sequence(name,budget,noise,oracles);result['sequences'].append(run);print('sequence',name,budget,noise,run['summary'],flush=True);save()
    result['loops']=[]
    jobs=[(name,offset,budget) for name in SCENES for offset in (-.1,0.,.1) for budget in (.035,5.)];rng.shuffle(jobs)
    for name,offset,budget in jobs:
        result['loops'].append(loop(name,offset,budget,oracles[name],opt.steps));save()
    result['worker']=worker_check(SCENES);save()
    assert all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p)] for p in protected)
    result['metadata']['production_unchanged']=True;save();print('Saved',opt.output,flush=True)

if __name__=='__main__': main()
