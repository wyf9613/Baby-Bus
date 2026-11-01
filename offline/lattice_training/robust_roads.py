"""Seeded short road fragments and cone layouts for sensor/planner screening.

Width is clear distance between cone inner edges. Native boundary-line widths
add both cone radii. No road is rejected based on perception/planner success.
"""
from copy import deepcopy
import hashlib
import json
import math
import numpy as np
from dreamgym.envs import Road
from offline.planner_sim.notebook import load_notebook
from .roads import road_length

FAMILIES=('s_left','s_right','u_left','u_right','compound_left','compound_right')
GENERATION_VERSION='clear05_dense_boundary_arc_reference_v3'
CONE_PLACEMENT=dict(regular_spacing_m=.18,irregular_spacing_range_m=[.12,.25],
    maximum_gap_m=.5,outward_displacement_range_m=[0.,.08],drop_probability=.12,
    spacing_coordinate='boundary_arc_length',minimum_physical_center_separation_m=.08)
_REFERENCE_CACHE={}


def road_geometry(case,support=0.):
    if case.get('boundary_geometry'):
        from .boundary_roads import geometry
        return geometry(case,support)
    from .pipeline import with_terminal_support
    return Road(road_spec=with_terminal_support(case['road_spec'],support) if support else case['road_spec'])


def vehicle_model():
    cfg=load_notebook().bicycle_model_config
    axle=cfg['axle_geometry'];body=cfg['body_geometry'];steering=cfg['steering']
    lf=axle['front_axle_distance_from_center_of_mass_m'];lr=axle['rear_axle_distance_from_center_of_mass_m']
    limit=steering['requested_front_wheel_angle_magnitude_limit_rad']
    rear_radius=(lf+lr)/math.tan(limit)
    return dict(wheelbase_m=lf+lr,rear_axle_from_cg_m=lr,steering_limit_rad=limit,
        minimum_steady_cg_radius_m=math.hypot(rear_radius,lr),
        steering_rate_limit_rad_per_s=steering['physical_front_wheel_angle_rate_bounds_rad_per_s']['upper'],
        front_m=body['front_extent_from_center_of_mass_m'],rear_m=body['rear_extent_from_center_of_mass_m'],
        width_m=body['width_m'])


def _pieces(turn_deg,radius,ramp,spacing=.15):
    sign=math.copysign(1,turn_deg);total=abs(math.radians(turn_deg))*radius
    ramp=min(ramp,total/2);plateau=max(0,total-ramp)
    lengths=[ramp,plateau,ramp];items=[]
    for phase,length in enumerate(lengths):
        if length<1e-8:continue
        n=max(1,math.ceil(length/spacing))
        for index in range(n):
            t=(index+.5)/n;smooth=t*t*(3-2*t)
            k=sign/radius*(smooth if phase==0 else 1-smooth if phase==2 else 1)
            items.append((length/n,k))
    scale=abs(math.radians(turn_deg))/abs(sum(d*k for d,k in items))
    return [(d,k*scale) for d,k in items]


def generate_case(family,seed,split):
    if family not in FAMILIES:raise ValueError('Unknown short-road family')
    rng=np.random.default_rng(seed);sign=-1 if family.endswith('right') else 1
    model=vehicle_model();widths=rng.uniform(.5,1.5,4)
    regime=(seed+FAMILIES.index(family)//2)%3
    if regime==0:widths=np.array([1.0,.5,1.5,.8])
    if regime==2:widths=rng.uniform(.5,.9,4)
    radius=max(float(rng.uniform(model['minimum_steady_cg_radius_m']+.08,1.45)),float(widths.max()+.08)/2+.06)
    if family.startswith('s_'):angles=[float(rng.uniform(45,95)),-float(rng.uniform(45,95))]
    elif family.startswith('u_'):angles=[180.]
    else:angles=[float(rng.uniform(45,80)),-float(rng.uniform(85,130)),float(rng.uniform(40,70))]
    pieces=[(float(rng.uniform(.8,1.3)),0.)]
    for angle in angles:
        pieces.extend(_pieces(sign*angle,radius,float(rng.uniform(.4,.65))))
        pieces.append((float(rng.uniform(.25,.6)),0.))
    pieces.append((float(rng.uniform(.8,1.3)),0.))
    length=sum(d for d,k in pieces)
    def width(s):
        position=min(3.,max(0.,s/length*3));index=min(2,int(position));u=position-index
        return .08+float(widths[index]+(widths[index+1]-widths[index])*u*u*(3-2*u))
    elements=[];station=0.
    for distance,curvature in pieces:
        geometry=(dict(type='straight',length_m=distance) if abs(curvature)<1e-10 else
            dict(type='circular_arc',curvature_per_m=curvature,sweep_angle_deg=math.degrees(abs(distance*curvature))))
        elements.append(dict(geometry=geometry,speed_limit_kph=1.8,
            lanes=dict(reference_line_role='lane_center',reference_lane=dict(surface='tarmac',
                width_m=dict(start=width(station),end=width(station+distance)))),
            active_cone_paths=dict(left_boundary='regular',right_boundary='regular')))
        station+=distance
    cone_paths={side:dict(placement=dict(mode='lane_line',lane_line_index=index),color=color,radius_m=.04,
        profiles=dict(regular=dict(inter_cone_spacing=dict(distribution='fixed',value_m=CONE_PLACEMENT['regular_spacing_m']))))
        for side,index,color in [('left_boundary',1,'blue'),('right_boundary',-1,'yellow')]}
    spec=dict(active_lane_minimum_width_m=.05,elements=elements,cone_paths=cone_paths,obstacles=[])
    digest=hashlib.sha256(json.dumps(spec,sort_keys=True).encode()).hexdigest()
    return dict(format='runtime_road_fixture_v1',kind=family,variant='short_fragments',direction='right' if sign<0 else 'left',
        fixture_id=f'robust_{family}_{seed}_{digest[:8]}',title=f'{family} / seed {seed}',collection='robust_screen',
        split=split,split_group=f'robust_seed_{seed}',road_seed=seed,runtime_seed=seed,road_spec=spec,
        mission_road_length_m=road_length(spec),road_sha256=digest,width_convention='cone_inner_edge_clear_width',
        terminal_sensor_support_m=2.,
        generation=dict(version=GENERATION_VERSION,width_range_m=[.5,1.5],width_controls_m=widths.tolist(),boundary_width_addition_m=.08,
            width_regime=regime,turn_angles_deg=[sign*a for a in angles],
            base_radius_m=radius,short_fragment_count=len(elements),vehicle=model,
            prescribed_reference='footprint_balanced',cone_placement=deepcopy(CONE_PLACEMENT),
            frame_noise=dict(distribution='gaussian_radial',sigma_range_m=[.08,.12],correlation=0.)))


def cone_layout(case,*,irregular=False):
    from .pipeline import with_terminal_support
    road=road_geometry(case,case.get('terminal_sensor_support_m',0.))
    rng=np.random.default_rng(np.random.SeedSequence([case['road_seed'],3051]))
    length=case['mission_road_length_m'];support=case.get('terminal_sensor_support_m',0.);records=[];removed=0;restored=0;pruned=0
    settings=case['generation']['cone_placement'];limit=settings['maximum_gap_m'];nominal_gaps=[]
    separation=settings['minimum_physical_center_separation_m']
    for side,colour in [('left','blue'),('right','yellow')]:
        # Distances belong to each boundary, not the road centre's station.
        # The latter compresses cones into the inside of tight, wide curves.
        grid=np.linspace(0,length+support,math.ceil((length+support)/.005)+1)
        if hasattr(road,'boundary_coordinates'):boundary,_=road.boundary_coordinates(side,grid)
        else:
            xy,head=road.convert_progression_to_coordinates(grid)
            offsets=np.array([road.get_reference_lane_bounds_at_progress(s)['left_bound_m' if side=='left' else 'right_bound_m'] for s in grid])
            boundary=xy+np.column_stack((-np.sin(head),np.cos(head)))*offsets[:,None]
        boundary_arc=np.r_[0.,np.linalg.norm(np.diff(boundary,axis=0),axis=1).cumsum()]
        def record(station,outward=None):
            if hasattr(road,'boundary_coordinates'):
                point,heading=road.boundary_coordinates(side,np.array([station]));offset=0.
            else:
                bounds=road.get_reference_lane_bounds_at_progress(station)
                point,heading=road.convert_progression_to_coordinates(np.array([station]))
                offset=bounds['left_bound_m' if side=='left' else 'right_bound_m']
            if outward is None:outward=float(rng.uniform(*settings['outward_displacement_range_m'])) if irregular else 0.
            tangent=float(heading[0]);normal=np.array([-math.sin(tangent),math.cos(tangent)])
            nominal=point[0]+normal*offset;xy=nominal+normal*outward*(1 if side=='left' else -1)
            return dict(x_m=float(xy[0]),y_m=float(xy[1]),colour=colour,radius_m=.04,
                nominal_x_m=float(nominal[0]),nominal_y_m=float(nominal[1]),outward_displacement_m=outward,
                role='boundary' if station<=length else 'terminal_support',side=side,station_m=station)
        distances=[0.]
        while distances[-1]<boundary_arc[-1]:
            step=float(rng.uniform(*settings['irregular_spacing_range_m'])) if irregular else settings['regular_spacing_m']
            distances.append(min(boundary_arc[-1],distances[-1]+step))
        stations=np.interp(distances,boundary_arc,grid)
        side_records=[];end=record(float(stations[-1]))
        def separated(candidate,others):
            return all(math.hypot(candidate['x_m']-c['x_m'],candidate['y_m']-c['y_m'])>=separation-1e-9 for c in others)
        def place(station,others):
            for attempt in range(17 if irregular else 1):
                candidate=record(station,0. if attempt==16 else None)
                if separated(candidate,others):return candidate
            return None
        for i,station in enumerate(stations):
            if irregular and 0<i<len(stations)-1 and rng.random()<settings['drop_probability']:
                removed+=1;continue
            if i==len(stations)-1:
                assert separated(end,records+side_records),'Terminal cone conflicts with another boundary'
                side_records.append(end);continue
            candidate=place(float(station),records+side_records+[end])
            if candidate is None:pruned+=1
            else:side_records.append(candidate)
        # Enforce the cap after omissions, curvature, varying widths and fixed offsets.
        # Bound BOTH along-side nominal arc distance and actual centre-to-centre distance.
        def boundary_distance(a,b):
            return float(np.interp(b['station_m'],grid,boundary_arc)-np.interp(a['station_m'],grid,boundary_arc))
        pending=list(reversed(side_records));dense=[]
        while pending:
            item=pending.pop()
            if dense:
                previous=dense[-1]
                distance=math.hypot(item['x_m']-previous['x_m'],item['y_m']-previous['y_m'])
                if max(distance,boundary_distance(previous,item))>limit+1e-9:
                    arc=(np.interp(previous['station_m'],grid,boundary_arc)+np.interp(item['station_m'],grid,boundary_arc))/2
                    middle=place(float(np.interp(arc,boundary_arc,grid)),records+dense+[item]+pending)
                    if middle is None:raise ValueError('Cannot satisfy boundary gap and physical cone separation')
                    pending.extend([item,middle])
                    restored+=1;continue
                nominal_gaps.append(boundary_distance(previous,item))
            dense.append(item)
        records.extend(dense)
    if irregular:
        # Another road's locally coherent cone chain outside the current corridor.
        station=float(rng.uniform(.3,.7)*length);side=int(rng.choice([-1,1]))
        for j in range(7):
            s=min(length,station+(j-3)*.22);point,heading=road.convert_progression_to_coordinates(np.array([s]))
            bounds=road.get_reference_lane_bounds_at_progress(s);normal=np.array([-math.sin(heading[0]),math.cos(heading[0])])
            offset=(bounds['left_bound_m']+.25+j*.04) if side>0 else (bounds['right_bound_m']-.25-j*.04)
            xy=point[0]+normal*offset
            records.append(dict(x_m=float(xy[0]),y_m=float(xy[1]),colour='blue' if j%2==0 else 'yellow',
                radius_m=.04,role='adjacent_road',side='clutter',station_m=s))
    gaps=[]
    for side in ('left','right'):
        items=[c for c in records if c['side']==side]
        gaps.extend(math.hypot(b['x_m']-a['x_m'],b['y_m']-a['y_m']) for a,b in zip(items,items[1:]))
    return records,dict(removed_boundary_cones=removed,overlap_pruned_cones=pruned,gap_fill_cones=restored,irregular=irregular,
        placement=deepcopy(settings),maximum_actual_cone_gap_m=max(gaps),
        maximum_nominal_boundary_gap_m=max(nominal_gaps),
        outward_displacement_is_frozen=True,
        overlap='angular_overlap_and_duplicate_detections_not_interpenetrating_physical_cones')


def centerline_samples(case,spacing=.025,speed=.5):
    """Prescribed CG reference, optionally balanced for the full body footprint."""
    cache_key=(case.get('boundary_sha256'),spacing,speed)
    if cache_key[0] and cache_key in _REFERENCE_CACHE:return _REFERENCE_CACHE[cache_key]
    model=vehicle_model();road=road_geometry(case);length=case['mission_road_length_m']
    station=np.arange(model['rear_m']+.03,length-model['front_m']-.03,spacing)
    xy,tangent=road.convert_progression_to_coordinates(station);tangent=np.unwrap(tangent)
    curvature=np.asarray(road.convert_progression_to_curvature(station))
    baseline=_kinematic_samples(station,xy,tangent,curvature,spacing,speed,model)
    if case['generation'].get('prescribed_reference')!='footprint_balanced':return baseline
    # Balance the actual corner margins, not just the CG's distance to a boundary.
    # This is an offline motion reference; no oracle path enters perception/planning.
    displacement=[]
    for center,angle,s in zip(xy,baseline['heading'],station):
        left,right=body_road_margins(road,model,center,angle,s)
        displacement.append((left-right)/2)
    radius=max(1,math.ceil(.3/spacing));grid=np.arange(-radius,radius+1)*spacing
    kernel=np.exp(-.5*(grid/.1)**2);kernel/=kernel.sum()
    displacement=np.convolve(np.pad(displacement,(radius,radius),mode='edge'),kernel,mode='valid')
    displacement=np.clip(displacement,-.12,.12)
    fade=np.minimum(np.clip((station-station[0])/.4,0,1),np.clip((station[-1]-station)/.4,0,1))
    displacement*=fade*fade*(3-2*fade)
    candidates=[baseline]
    for factor in (1.,.5):
        shifted=xy+np.column_stack((-np.sin(tangent),np.cos(tangent)))*displacement[:,None]*factor
        arc=np.r_[0.,np.linalg.norm(np.diff(shifted,axis=0),axis=1).cumsum()]
        travel=np.arange(0,arc[-1]+1e-9,spacing)
        path=np.column_stack([np.interp(travel,arc,shifted[:,j]) for j in (0,1)])
        road_station=np.interp(travel,arc,station)
        derivative=np.gradient(path,spacing,axis=0,edge_order=2)
        direction=np.unwrap(np.arctan2(derivative[:,1],derivative[:,0]))
        k=np.gradient(direction,spacing,edge_order=2)
        sample=_kinematic_samples(road_station,path,direction,k,spacing,speed,model)
        sample.update(reference_kind='footprint_balanced',maximum_reference_shift_m=float(np.max(abs(displacement)))*factor)
        if np.max(abs(sample['steering']))<=model['steering_limit_rad'] and np.max(abs(np.diff(sample['steering'])))/sample['dt_s']<=model['steering_rate_limit_rad_per_s']:
            candidates.append(sample)
    # Choose purely on physical body/road clearance, independently of sensor success.
    def clearance(candidate):
        return min(min(body_road_margins(road,model,c,a,s)) for c,a,s in zip(candidate['xy'],candidate['heading'],candidate['station']))
    chosen=max(candidates,key=clearance)
    chosen['baseline_minimum_body_road_clearance_m']=clearance(baseline)
    if cache_key[0]:_REFERENCE_CACHE[cache_key]=chosen
    return chosen


def _kinematic_samples(station,xy,tangent,curvature,spacing,speed,model):
    """Inverse no-slip CG kinematics: beta' = kappa - sin(beta)/lr."""
    beta=np.zeros(len(station));lr=model['rear_axle_from_cg_m'];wheelbase=model['wheelbase_m']
    for i in range(1,len(station)):
        ds=spacing;k=(curvature[i-1]+curvature[i])/2
        def f(v):return k-math.sin(v)/lr
        a=f(beta[i-1]);b=f(beta[i-1]+ds*a/2);c=f(beta[i-1]+ds*b/2);d=f(beta[i-1]+ds*c)
        beta[i]=beta[i-1]+ds*(a+2*b+2*c+d)/6
    steering=np.arctan(wheelbase/lr*np.tan(beta));yaw_rate=speed/lr*np.sin(beta)
    heading=tangent-beta
    return dict(station=station,xy=xy,heading=heading,beta=beta,steering=steering,yaw_rate=yaw_rate,
        speed_mps=speed,dt_s=spacing/speed,curvature=curvature,model=model,
        reference_kind='road_center',maximum_reference_shift_m=0.)


def body_road_margins(road,model,center,angle,station):
    if hasattr(road,'footprint_margins'):return road.footprint_margins(model,center,angle)
    co,si=math.cos(angle),math.sin(angle);left=right=float('inf')
    for bx,by in [(-model['rear_m'],-model['width_m']/2),(model['front_m'],-model['width_m']/2),
                  (model['front_m'],model['width_m']/2),(-model['rear_m'],model['width_m']/2)]:
        wx,wy=center+np.array([co*bx-si*by,si*bx+co*by])
        cx,cy,_,_,s,t,_=road.find_closest_point_to(float(wx),float(wy),reference_line_progress_hint=float(station),
            reference_line_progress_search_radius_m=.7,allow_global_closest_point_fallback=False)
        bounds=road.get_reference_lane_bounds_at_progress(s);lateral=-(wx-cx)*math.sin(t)+(wy-cy)*math.cos(t)
        left=min(left,bounds['left_bound_m']-lateral);right=min(right,lateral-bounds['right_bound_m'])
    return float(left),float(right)


def pose_clearances(road,points,radii,model,center,angle,station):
    """One prescribed pose, shared by screening and passive failure annotations."""
    co,si=math.cos(angle),math.sin(angle);contact=float('inf')
    minimum=min(body_road_margins(road,model,center,angle,station))
    if len(points):
        rel=points-center;bx=rel[:,0]*co+rel[:,1]*si;by=-rel[:,0]*si+rel[:,1]*co
        dx=np.maximum(np.maximum(-model['rear_m']-bx,bx-model['front_m']),0)
        dy=np.maximum(abs(by)-model['width_m']/2,0)
        contact=float(np.min(np.hypot(dx,dy)-radii))
    return float(minimum),float(contact)


def physical_screen(case,cones,samples):
    road=road_geometry(case);model=samples['model'];minimum=float('inf');contact=float('inf')
    points=np.asarray([[c['x_m'],c['y_m']] for c in cones]);radii=np.asarray([c['radius_m'] for c in cones])
    for center,angle,station in zip(samples['xy'],samples['heading'],samples['station']):
        road_margin,cone_margin=pose_clearances(road,points,radii,model,center,angle,station)
        minimum=min(minimum,road_margin);contact=min(contact,cone_margin)
    maximum=float(np.max(abs(samples['steering'])));rate=float(np.max(abs(np.diff(samples['steering'])))/samples['dt_s'])
    reasons=[]
    if maximum>model['steering_limit_rad']+1e-8:reasons.append('steering_limit')
    if rate>model['steering_rate_limit_rad_per_s']+1e-8:reasons.append('steering_rate')
    if minimum<0:reasons.append('body_outside_road_on_centerline')
    if contact<0:reasons.append('body_cone_contact_on_centerline')
    return dict(centerline_passable=not reasons,reasons=reasons,minimum_body_road_clearance_m=minimum,
        minimum_body_cone_clearance_m=contact,max_required_steering_deg=math.degrees(maximum),
        max_required_steering_rate_rad_per_s=rate,spacing_m=.025,continuous_sweep_certified=False,
        reference_kind=samples['reference_kind'],maximum_reference_shift_m=samples['maximum_reference_shift_m'],
        baseline_minimum_body_road_clearance_m=samples.get('baseline_minimum_body_road_clearance_m',minimum))
