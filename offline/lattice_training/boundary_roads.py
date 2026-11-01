"""Independently drawn physical boundaries, then an environment-only reference.

The native simulator's Road is a derived observation host, never physical truth.
No common continuous centre curve is used to construct the two boundaries.
"""
from copy import deepcopy
import hashlib,json,math
import numpy as np

VERSION='boundary_first_v4'
WIDTH_PROFILES=('wide_entry','wide_exit','similar_ends','apex_narrow')
_GEOMETRY_CACHE={}


def cross(a,b):return a[...,0]*b[...,1]-a[...,1]*b[...,0]


def cubic_curve(points,count=1201):
    """Clamped C2 cubic interpolation, using NumPy only; vector-valued knots."""
    p=np.asarray(points,dtype=float);n=len(p);t=np.arange(n,dtype=float)
    matrix=np.zeros((n,n));rhs=np.zeros_like(p)
    matrix[0,:2]=[2.,1.];matrix[-1,-2:]=[1.,2.]
    # Endpoint derivatives are the first/last control-segment vectors.
    for i in range(1,n-1):matrix[i,i-1:i+2]=[1.,4.,1.];rhs[i]=6*(p[i+1]-2*p[i]+p[i-1])
    second=np.linalg.solve(matrix,rhs)
    u=np.linspace(0,n-1,count);index=np.minimum(u.astype(int),n-2);v=u-index
    return ((1-v)[:,None]*p[index]+v[:,None]*p[index+1]+
        (((1-v)**3-(1-v))[:,None]*second[index]+(v**3-v)[:,None]*second[index+1])/6)


def _width_request(profile,u):
    values={'wide_entry':[1.30,.90,.65],'wide_exit':[.65,.90,1.30],
        'similar_ends':[.90,.90,.90],'apex_narrow':[1.25,.65,1.25]}[profile]
    return np.interp(u,[0.,.5,1.],values)


def generate_boundary_case(family,seed,split):
    from .robust_roads import FAMILIES,CONE_PLACEMENT,vehicle_model
    if family not in FAMILIES:raise ValueError('Unknown boundary topology')
    rng=np.random.default_rng(np.random.SeedSequence([seed,FAMILIES.index(family)//2,4177]))
    profile=WIDTH_PROFILES[(seed+FAMILIES.index(family)//2)%4]
    sign=-1 if family.endswith('right') else 1
    if family.startswith('u_'):
        entry=float(rng.uniform(.8,1.9));requested=_width_request(profile,np.array([0.,.5,1.]))
        wen,wap,wex=requested+rng.uniform(-.07,.07,3)+.08
        ry=max(float(rng.uniform(1.05,1.65)),(wen+wex)/2+.20)
        rx=ry*float(rng.uniform(.90,1.18));tip=entry+rx-wap
        lo,hi=wen,2*ry-wex;entry_in=entry+float(rng.uniform(-.22,.18));exit_in=entry+float(rng.uniform(-.22,.18))
        # Outer and inner XY controls are independently authored, not offsets.
        right=np.array([[-1,0],[0,0],[entry,0],[entry+.72*rx,.25*ry],
            [entry+rx,ry],[entry+.72*rx,1.75*ry],[entry,2*ry],[0,2*ry],[-1,2*ry]],float)
        left=np.array([[-1,lo],[0,lo],[entry_in,lo],[entry_in+.70*(tip-entry_in),lo+.22*(hi-lo)],
            [tip,(lo+hi)/2],[exit_in+.65*(tip-exit_in),lo+.78*(hi-lo)],
            [exit_in,hi],[0,hi],[-1,hi]],float)
        left[2:7]+=rng.uniform([-.09,-.05],[.09,.05],(5,2))
        right[2:7]+=rng.uniform([-.09,-.06],[.09,.06],(5,2))
        layout='asymmetric_inner_outer'
    else:
        if family.startswith('s_'):
            x=np.array([-1.,0.,.85,1.75,2.85,3.75,4.7,5.6])
            amplitude=float(rng.uniform(.8,1.7))
            levels=np.array([0.,0.,0.,.28*amplitude,.85*amplitude,amplitude,amplitude,amplitude])
            layout='offset_s'
        else:
            x=np.array([-1.,0.,.85,1.7,2.55,3.45,4.35,5.25,6.15,7.05,8.])*1.15
            a,b,c=rng.uniform(.65,1.15,3)
            levels=np.array([0.,0.,0.,.6*a,a,.15*a,-b,-.35*b,c,.55*c,.55*c])
            layout=('short_knees','unequal_s_bends','offset_apices')[seed%3]
            if layout=='short_knees':x[3:9]+=np.array([-.12,.12,-.12,.12,-.12,.12])
        x[2:-2]+=rng.uniform(-.16,.16,len(x)-4)
        # A coarse topology supplies regions only. Each side has separate XY
        # controls, bend timing, local bulges and endpoint widths within regions.
        u=np.linspace(0,1,len(x));request=_width_request(profile,u)+.08
        # These independent world-coordinate bands must span the local slanted
        # region, rather than treating vertical separation as clear road width.
        request*=np.sqrt(1+np.gradient(levels,x)**2)
        left=np.column_stack((x,levels+request/2+rng.uniform(-.055,.055,len(x))))
        right=np.column_stack((x,levels-request/2+rng.uniform(-.055,.055,len(x))))
        left[2:-2,0]+=rng.uniform(-.12,.12,len(x)-4)
        right[2:-2,0]+=rng.uniform(-.12,.12,len(x)-4)
        left[2:-2,1]+=rng.uniform(-.065,.065,len(x)-4)
        right[2:-2,1]+=rng.uniform(-.065,.065,len(x)-4)
        # Straight sensor support at either end, not an extra path cue.
        for points in (left,right):points[0,1]=points[1,1];points[-1,1]=points[-2,1]
    lxy,rxy=cubic_curve(left),cubic_curve(right)
    if sign<0:
        # Reflection swaps physical side ownership as well as coordinates.
        left,right=right*np.array([1.,-1.]),left*np.array([1.,-1.])
        lxy,rxy=rxy*np.array([1.,-1.]),lxy*np.array([1.,-1.])
    midpoint=(lxy+rxy)/2
    # Derive an initial frame only AFTER physical boundaries are fixed.
    origin=midpoint[0].copy();direction=midpoint[1]-midpoint[0];angle=math.atan2(direction[1],direction[0])
    co,si=math.cos(angle),math.sin(angle);rotation=np.array([[co,-si],[si,co]])
    left,right=(left-origin)@rotation,(right-origin)@rotation
    lxy,rxy=(lxy-origin)@rotation,(rxy-origin)@rotation
    midpoint=(lxy+rxy)/2
    arc=np.r_[0.,np.linalg.norm(np.diff(midpoint,axis=0),axis=1).cumsum()]
    length=float(arc[-1]);station=np.linspace(0,length,max(3,math.ceil(length/.015)+1))
    ref=np.column_stack([np.interp(station,arc,midpoint[:,j]) for j in (0,1)])
    # Save original boundary curves against the fitted path's progression. This
    # is correspondence metadata, not reconstruction from the fitted reference.
    boundary=dict(left_xy=lxy.tolist(),right_xy=rxy.tolist(),pair_progress_m=arc.tolist(),
        reference_xy=ref.tolist(),reference_station_m=station.tolist(),
        left_controls_xy=left.tolist(),right_controls_xy=right.tolist())
    from .robust_roads import vehicle_model
    model=vehicle_model();spec=_host_spec(ref,station)
    road_hash=hashlib.sha256(json.dumps(spec,sort_keys=True).encode()).hexdigest()
    boundary_hash=hashlib.sha256(json.dumps(boundary,sort_keys=True).encode()).hexdigest()
    return dict(format='runtime_road_fixture_v1',kind=family,variant='boundary_first',direction='right' if sign<0 else 'left',
        fixture_id=f'boundary_{family}_{seed}_{boundary_hash[:8]}',title=f'{family} / seed {seed} / {profile}',
        collection='boundary_first_screen',split=split,split_group=f'boundary_seed_{seed}',road_seed=seed,runtime_seed=seed,
        road_spec=spec,road_sha256=road_hash,boundary_sha256=boundary_hash,boundary_geometry=boundary,
        mission_road_length_m=length,width_convention='cone_inner_edge_clear_width',terminal_sensor_support_m=2.,
        generation=dict(version=VERSION,width_range_m=[.5,1.5],width_controls_m=_width_request(profile,np.linspace(0,1,4)).tolist(),
            boundary_width_addition_m=.08,width_regime=profile,width_profile=profile,layout_variant=layout,
            construction_order=['independent_left_and_right_controls','physical_boundary_curves','fitted_middle_reference','native_observation_host'],
            road_spec_role='observation_host_only',prescribed_reference='footprint_balanced',vehicle=model,
            cone_placement=deepcopy(CONE_PLACEMENT),frame_noise=dict(distribution='gaussian_radial',sigma_range_m=[.08,.12],correlation=0.)))


def _host_spec(reference,station):
    """Only a native observation carrier; its symmetric lane is not road truth."""
    heading=np.unwrap(np.arctan2(*np.gradient(reference,axis=0)[:,::-1].T))
    elements=[]
    for a in range(0,len(station)-1,4):
        b=min(len(station)-1,a+4);distance=float(station[b]-station[a]);k=float((heading[b]-heading[a])/distance)
        # Native Road and AutonomousDrivingEnv reject nonzero |k| < 1e-4.
        # This encoding threshold affects only the observation carrier.
        geometry=(dict(type='straight',length_m=distance) if abs(k)<1e-4 else
            dict(type='circular_arc',curvature_per_m=k,sweep_angle_deg=math.degrees(abs(k*distance))))
        elements.append(dict(geometry=geometry,speed_limit_kph=1.8,lanes=dict(reference_line_role='lane_center',
            reference_lane=dict(surface='tarmac',width_m=dict(start=1.,end=1.))),active_cone_paths={}))
    # Never compile offset cones on this derived carrier. In sharp curves they
    # may fold even though the independent physical boundaries are valid.
    return dict(active_lane_minimum_width_m=.05,elements=elements,cone_paths={},obstacles=[])


def _point_segments(points,a,b):
    v=b-a;denom=np.maximum((v*v).sum(axis=1),1e-18)
    fraction=np.clip(((points[:,None]-a[None])*v[None]).sum(axis=2)/denom[None],0,1)
    projection=a[None]+fraction[:,:,None]*v[None]
    delta=points[:,None]-projection
    return np.linalg.norm(delta,axis=2),projection,delta


def _intersects(a,b,c,d):
    v=b-a;w=d-c;denom=cross(v[:,None],w[None]);delta=c[None]-a[:,None]
    safe=np.where(abs(denom)>1e-12,denom,1.)
    t=cross(delta,w[None])/safe;u=cross(delta,v[:,None])/safe
    return (abs(denom)>1e-12)&(t>=0)&(t<=1)&(u>=0)&(u<=1)


class BoundaryCorridor:
    def __init__(self,case,support=0.):
        data=case['boundary_geometry'];self.length=case['mission_road_length_m'];self.support=support
        self.station=np.asarray(data['reference_station_m']);self.reference=np.asarray(data['reference_xy'])
        self.pair=np.asarray(data['pair_progress_m']);self.boundaries={k:np.asarray(data[k+'_xy']) for k in ('left','right')}
        if support:
            for side,points in self.boundaries.items():
                direction=points[-1]-points[-2];direction/=np.linalg.norm(direction)
                self.boundaries[side]=np.vstack((points,points[-1]+direction*support))
            self.pair=np.r_[self.pair,self.length+support]
            direction=self.reference[-1]-self.reference[-2];direction/=np.linalg.norm(direction)
            self.reference=np.vstack((self.reference,self.reference[-1]+direction*support));self.station=np.r_[self.station,self.length+support]
        self.heading=np.unwrap(np.arctan2(*np.gradient(self.reference,axis=0)[:,::-1].T))
        self.curvature=np.gradient(self.heading,self.station,edge_order=2)
        # Decimate only redundant dense curve samples; physical polylines remain
        # at <= 2 cm boundary arc spacing for containment/contact checks.
        self.edges={}
        for side,points in self.boundaries.items():
            arc=np.r_[0.,np.linalg.norm(np.diff(points,axis=0),axis=1).cumsum()]
            ss=np.linspace(0,arc[-1],math.ceil(arc[-1]/.02)+1)
            self.edges[side]=np.column_stack([np.interp(ss,arc,points[:,j]) for j in (0,1)])
        self.polygon=np.vstack((self.edges['left'],self.edges['right'][::-1]))

    def convert_progression_to_coordinates(self,station):
        ss=np.asarray(station);return np.column_stack([np.interp(ss,self.station,self.reference[:,j]) for j in (0,1)]),np.interp(ss,self.station,self.heading)

    def convert_progression_to_curvature(self,station):return np.interp(station,self.station,self.curvature)

    def boundary_coordinates(self,side,station):
        points=self.boundaries[side];ss=np.asarray(station)
        xy=np.column_stack([np.interp(ss,self.pair,points[:,j]) for j in (0,1)])
        # Local physical-boundary tangent, independent of fitted-route tangent.
        tangent=np.unwrap(np.arctan2(*np.gradient(points,axis=0)[:,::-1].T))
        return xy,np.interp(ss,self.pair,tangent)

    def cross_section(self,center,heading):
        n=np.array([-math.sin(heading),math.cos(heading)]);result={}
        for side,points in self.edges.items():
            a,b=points[:-1],points[1:];v=b-a;delta=a-center;denom=cross(n,v)
            safe=np.where(abs(denom)>1e-12,denom,1.)
            distance=cross(delta,v)/safe;fraction=cross(delta,n)/safe
            valid=(abs(denom)>1e-12)&(fraction>=-1e-8)&(fraction<=1+1e-8)&(distance>0 if side=='left' else distance<0)
            values=distance[valid];result[side]=float(values.min() if side=='left' and len(values) else values.max() if len(values) else np.nan)
        return result['left'],result['right']

    def get_reference_lane_bounds_at_progress(self,station):
        xy,angle=self.convert_progression_to_coordinates(np.array([station]));left,right=self.cross_section(xy[0],angle[0])
        return dict(left_bound_m=left,right_bound_m=right)

    def inside(self,points):
        a=self.polygon;b=np.roll(a,-1,axis=0);y=points[:,1,None];x=points[:,0,None]
        dy=b[:,1]-a[:,1];safe=np.where(abs(dy)>1e-12,dy,1.)
        hit=((a[:,1]>y)!=(b[:,1]>y))&(x<a[:,0]+(y-a[:,1])*(b[:,0]-a[:,0])/safe)
        return hit.sum(axis=1)%2==1

    def footprint_margins(self,model,center,angle):
        c,s=math.cos(angle),math.sin(angle)
        body=np.array([[-model['rear_m'],-model['width_m']/2],[model['front_m'],-model['width_m']/2],
            [model['front_m'],model['width_m']/2],[-model['rear_m'],model['width_m']/2]])@np.array([[c,s],[-s,c]])+center
        margins=[];end=np.roll(body,-1,axis=0)
        for side,points in self.edges.items():
            a,b=points[:-1],points[1:];dist,proj,delta=_point_segments(body,a,b);index=dist.argmin(axis=1)
            signed=cross((b-a)[index],delta[np.arange(4),index])/np.linalg.norm((b-a)[index],axis=1)
            margin=float(np.min(-signed if side=='left' else signed))
            other=_point_segments(points,body,end)[0].min()
            margin=min(margin,float(dist.min()),float(other))
            if _intersects(body,end,a,b).any():margin=min(margin,-1e-7)
            margins.append(margin)
        if not self.inside(body).all() and min(margins)>=0:margins=[-1e-7,-1e-7]
        return tuple(margins)

    def has_self_intersection(self):
        points=self.polygon;a=points;b=np.roll(points,-1,axis=0);n=len(a)
        hits=_intersects(a,b,a,b);i,j=np.indices((n,n))
        return bool(np.any(hits&(j>i+1)&~((i==0)&(j==n-1))))


def geometry(case,support=0.):
    key=(case['boundary_sha256'],support)
    if key not in _GEOMETRY_CACHE:_GEOMETRY_CACHE[key]=BoundaryCorridor(case,support)
    return _GEOMETRY_CACHE[key]


def boundary_geometry_audit(case):
    road=geometry(case);ss=np.arange(.23,road.length-.33,.04)
    xy,angle=road.convert_progression_to_coordinates(ss)
    sections=np.array([road.cross_section(p,a) for p,a in zip(xy,angle)])
    widths=sections[:,0]-sections[:,1]-.08
    reasons=[]
    if road.has_self_intersection():reasons.append('physical_boundaries_self_intersect')
    if not np.isfinite(widths).all():reasons.append('missing_local_cross_section')
    if np.nanmin(widths)<.5-1e-4:reasons.append('clear_width_below_0_5')
    if np.nanmax(widths)>1.5+1e-4:reasons.append('clear_width_above_1_5')
    return dict(passed=not reasons,reasons=reasons,minimum_clear_width_m=float(np.nanmin(widths)),
        maximum_clear_width_m=float(np.nanmax(widths)),
        entry_clear_width_m=float(np.nanmedian(widths[:10])),exit_clear_width_m=float(np.nanmedian(widths[-10:])),
        width_profile=case['generation']['width_profile'],construction='boundaries_before_reference')
