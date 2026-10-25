"""Bridge the upstream learned reference stream to Zhenyu's unchanged MPC.

Admission/fallback is read-only. Only one engine.step is allowed per control tick.
RNN state is committed by the ROS node only after publication of an executed path.
"""
from copy import deepcopy
import math,time
from .local_mpc import (MPCController,MPCStop,reference_trajectory_from_planning,
    mpc_reference_errors,finite_number,_path_lengths,_path_tangent,wrap_angle)
from .lattice import _shadow_collision


class MPCPolicyController:
    def __init__(self, settings, mpc_settings, vehicle, shadow_settings, frame_id="base_link"):
        self.settings,self.mpc_settings=settings,mpc_settings
        self.shadow_settings,self.frame_id=shadow_settings,frame_id
        self.policy_frame_id=frame_id
        self.mpc_controller=MPCController(mpc_settings,vehicle)
        self.estimates={}
        self.first=True

    def clear(self):
        self.mpc_controller.reset()
        self.first=True

    def prepare(self, reference, state, now_s):
        """Pure admission: never advances MPC time, PI integral or command history."""
        cfg=self.mpc_settings
        self.admission_debug={"branch":"admission","reference_timestamp_s":reference.get("timestamp_s") if isinstance(reference,dict) else None}
        if not isinstance(reference,dict):
            return None,"reference_missing"
        if (reference.get("frame_id") != self.frame_id or state.get("frame_id") != self.frame_id
                or reference.get("vehicle_reference_point") != "cg_ground_projection"
                or state.get("valid") is not True or state.get("speed_valid") is not True
                or state.get("yaw_rate_valid") is not True
                or not finite_number(state.get("timestamp_s"))
                or not finite_number(reference.get("timestamp_s"))
                or abs(state["timestamp_s"]-reference["timestamp_s"])>1e-6):
            return None,"reference_frame_or_state_alignment"
        traj=reference_trajectory_from_planning(reference,cfg.reference_spacing_m)
        if (not traj.get("valid") or traj.get("stop_required")
                or not finite_number(now_s) or not finite_number(traj.get("valid_for_s"))
                or not traj["timestamp_s"]-1e-6 <= now_s < traj["timestamp_s"]+traj["valid_for_s"]):
            return None,traj.get("reason") or "reference_expired_or_stop"
        traj=self._repair_mpc_reference(traj,self.estimates)
        points=traj.get("points") or []
        self.admission_debug.update(reference_point_count=len(points),
            first_xy_m=[points[0]["x_m"],points[0]["y_m"]] if points else None,
            reference_valid_for_s=traj.get("valid_for_s"),join_repair=traj.get("join_repair"),
            max_backward_extension_m=cfg.max_backward_extension_m)
        heading,_,geometry=MPCController._start_bridge_geometry(traj.get("points"))
        info,reason=mpc_reference_errors(traj,cfg.horizon_n,cfg.dt_pred_s,0.,
                                        cfg.max_backward_extension_m,heading)
        if info is None:
            self.admission_debug["reject_reason"]=reason
            return None,reason
        self.admission_debug.update(e_y_m=info["e_y_m"],e_psi_rad=info["e_psi_rad"],
                                    path_remaining_m=info["path_remaining_m"])
        if cfg.start_bridge_enabled and info["extended_start"]:
            if geometry: return None,geometry
            if abs(info["e_y_m"])>.20 or abs(info["e_psi_rad"])>.35:
                return None,"start_bridge_alignment"
        return traj,None

    def execute(self, reference, traj, state, now_s, dt):
        diagnostics={"valid":False,"stop_requested":True,"reason":"not_executed",
            "path_error_m":0.,"heading_error_rad":0.,"curvature_1pm":0.,
            "controller":"user_mpc_speed_feedback","mpc_reference_executed":False}
        if traj is None:
            return 0.,0.,diagnostics
        try:
            result=self.mpc_controller.step(traj,state["speed_mps"],dt,now_s,self.first)
            self.first=False
            debug=result["debug"]
            diagnostics.update(valid=True,stop_requested=False,reason=debug.get("branch"),
                path_error_m=debug.get("e_y_m") or 0.,heading_error_rad=debug.get("e_psi_rad") or 0.,
                curvature_1pm=(traj.get("points") or [{}])[0].get("curvature_1pm",0.),
                drive_action=result["drive"],steering_action=result["steer"],
                mpc_reference_executed=(debug.get("branch")=="solved" and not self.mpc_settings.shadow),
                path_encoding=traj.get("reference_path_type"),mpc=deepcopy(debug),
                join_repair=traj.get("join_repair"))
            if debug.get("branch") not in ("solved","shadow","zero_target","ref_hold","plan_hold"):
                diagnostics.update(valid=False,stop_requested=True,reason=debug.get("reject_reason") or debug.get("branch"))
            return result["drive"],result["steer"],diagnostics
        except (MPCStop,ValueError,TypeError,ArithmeticError) as error:
            diagnostics.update(reason=str(error),mpc=deepcopy(self.mpc_controller.last_debug),
                               join_repair=traj.get("join_repair"))
            return 0.,0.,diagnostics


def calculate_selected_mpc(controller, baseline, planner_reference, state, vehicle_params,
                           now_s,dt,execution=None,baseline_provider=None):
    reference=planner_reference if planner_reference is not None else baseline
    traj,reason=controller.prepare(reference,state,now_s)
    if planner_reference is not None and reason is not None:
        if execution is not None:
            execution.diagnostics.update(mode="baseline",fallback=True,reason="mpc_admission_rejected: "+str(reason))
        reference=baseline_provider() if baseline_provider is not None else baseline
        traj,reason=controller.prepare(reference,state,now_s)
    if reason is not None:
        diag=dict(valid=False,stop_requested=True,reason="mpc_admission: "+str(reason),
                  path_error_m=0.,heading_error_rad=0.,curvature_1pm=0.,mpc_reference_executed=False,
                  mpc=dict(getattr(controller,"admission_debug",{}),branch="admission_rejected",reject_reason=reason))
        drive=steer=0.
    else:
        drive,steer,diag=controller.execute(reference,traj,state,now_s,dt)
        if (traj.get("join_repair") or {}).get("active"):
            reference=deepcopy(reference)
            reference["path"]={"type":"ORDERED_CENTERLINE","independent_variable":"s_m",
                "samples_xy":[[p["x_m"],p["y_m"]] for p in traj["points"]],
                "range":[0.,traj["points"][-1]["s_m"]],
                "preview_m":reference.get("path",{}).get("preview_m",.35)}
            reference["mpc_join_repair"]=deepcopy(traj["join_repair"])
    if execution is not None:
        diag.update(reference_source=execution.diagnostics.get("mode"),
                    reference_selection_reason=execution.diagnostics.get("reason"))
        execution.diagnostics.update(controller="user_mpc_speed_feedback",drive_action=drive,
            steering_action=steer,control_dt_s=dt,mpc_reason=diag.get("reason"))
    return controller,drive,steer,diag,reference


def _repair_mpc_reference(self, traj, estimates):
    """Keep admitted checkpoint paths exact; repair only rejected short joins."""
    cfg = self.mpc_settings
    if (not cfg.reference_join_repair_enabled or not traj.get("valid")
            or traj.get("stop_required") or traj.get("planned_stop")
            or traj.get("reference_path_type") != "ORDERED_CENTERLINE"):
        return traj
    state = estimates.get("state") or {}
    now = state.get("timestamp_s")
    if (not state.get("valid") or not finite_number(now)
            or not finite_number(traj.get("timestamp_s"))
            or not finite_number(traj.get("valid_for_s"))
            or not traj["timestamp_s"]-1e-6 <= now < traj["timestamp_s"]+traj["valid_for_s"]):
        return traj
    heading, _, geometry_reason = MPCController._start_bridge_geometry(traj.get("points"))
    info, reason = mpc_reference_errors(traj,cfg.horizon_n,cfg.dt_pred_s,0.0,
                                        cfg.max_backward_extension_m,heading)
    if info is not None:
        if not info["extended_start"] or (not geometry_reason and
                abs(info["e_y_m"]) <= .20 and abs(info["e_psi_rad"]) <= .35):
            return traj
        reason = geometry_reason or "start_bridge_alignment"
    if reason not in ("origin_before_path_start","start_bridge_alignment","start_bridge_geometry"):
        return traj
    road = estimates.get("road") or {}
    road = road if road.get("time_aligned") else road.get("forward_geometry") or road
    if (road.get("frame_id") != self.policy_frame_id or not road.get("time_aligned")
            or not finite_number(road.get("timestamp_s"))
            or abs(road["timestamp_s"]-now)>1e-6):
        return traj
    boundaries = {name:road[name+"_boundary_xy"] for name in ("left","right")
        if road.get("boundary_source",{}).get(name) in ("observed","observed_with_history")
        and len(road.get(name+"_boundary_xy") or [])>=2}
    if not boundaries:
        return traj
    points = traj.get("points") or []
    xy = [(p["x_m"],p["y_m"]) for p in points]
    if len(xy)<2 or math.hypot(*xy[0])>cfg.max_backward_extension_m:
        return dict(traj,join_repair={"active":False,"original_reason":reason,
                    "reason":"join_gap_exceeds_limit","first_gap_m":math.hypot(*xy[0]) if xy else None})
    started=time.perf_counter()
    attempts={}
    vehicle=self.mpc_controller.vehicle
    _, bounds=self.mpc_controller.solver.input_bounds()
    delta_max=max(abs(vehicle.steering_gain_rad*b+vehicle.steering_offset_rad) for b in bounds)
    curvature_limit=math.tan(delta_max)/vehicle.wheelbase_m
    # First try an explicit tangent prefix. Unlike a zero-error Hermite join,
    # this preserves a small REAL tracking error and does not demand an abrupt
    # lateral manoeuvre merely to make a forward-only reference start at CG.
    # Same extension/alignment bounds as the checkpoint; no unknown far tail.
    lengths_xy=_path_lengths(xy)
    tangent=_path_tangent(xy,lengths_xy,0.)
    heading=math.atan2(tangent[1],tangent[0])
    along=-(xy[0][0]*tangent[0]+xy[0][1]*tangent[1])
    lateral=xy[0][0]*tangent[1]-xy[0][1]*tangent[0]
    if (-cfg.max_backward_extension_m <= along < 0. and abs(lateral)<=.20
            and abs(heading)<=.35):
        extension=min(cfg.max_backward_extension_m,-along+.02)
        count=max(1,math.ceil(extension/cfg.reference_spacing_m))
        prefix=[(xy[0][0]-extension*(1-k/count)*tangent[0],
                 xy[0][1]-extension*(1-k/count)*tangent[1]) for k in range(count)]
        curve=prefix+xy
        lengths=_path_lengths(curve)
        poses=[(p[0],p[1],heading if k<len(prefix) else points[k-len(prefix)]["yaw_rad"])
               for k,p in enumerate(curve)]
        if not _shadow_collision(poses,lengths[-1],boundaries,self.shadow_settings):
            candidate=deepcopy(traj)
            candidate["points"]=[dict(points[0],s_m=lengths[k],x_m=p[0],y_m=p[1],
                yaw_rad=heading,curvature_1pm=0.) if k<len(prefix)
                else dict(points[k-len(prefix)],s_m=lengths[k]) for k,p in enumerate(curve)]
            check,_=mpc_reference_errors(candidate,cfg.horizon_n,cfg.dt_pred_s,0.,cfg.max_backward_extension_m,0.)
            if check is not None and not check["extended_start"]:
                candidate["join_repair"]={"active":True,"method":"bounded_tangent_prefix",
                    "original_reason":reason,"extension_m":extension,"e_y_m":lateral,
                    "e_psi_rad":-heading,"elapsed_s":time.perf_counter()-started}
                return candidate
        attempts["tangent_boundary_or_projection"]=1
    # Search joins into the existing measured path; never invent a farther tail.
    for index in sorted(set((0,min(2,len(xy)-2),min(4,len(xy)-2),min(8,len(xy)-2)))):
        end=xy[index]
        if end[0]<=.05 or math.hypot(*end)>cfg.max_backward_extension_m+.5:
            continue
        tangent=_path_tangent(xy,lengths_xy,lengths_xy[index])
        distance=math.hypot(*end)
        for scale in (1.0,1.5):
            if time.perf_counter()-started>.008:
                break
            length=distance*scale
            # Cubic Hermite: vehicle forward heading to the measured path tangent.
            curve=[]
            for k in range(49):
                u=k/48; u2=u*u; u3=u2*u
                h10=u3-2*u2+u; h01=-2*u3+3*u2; h11=u3-u2
                curve.append((h10*length+h01*end[0]+h11*length*tangent[0],
                              h01*end[1]+h11*length*tangent[1]))
            curve += xy[index+1:]
            lengths=_path_lengths(curve)
            if any(b-a<1e-7 for a,b in zip(lengths,lengths[1:])):
                continue
            yaws=[math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in zip(curve,curve[1:])]
            if any(abs(angle)>math.pi/2 for angle in yaws[:48]):
                attempts["reverse"]=attempts.get("reverse",0)+1; continue
            curv=[wrap_angle(b-a)/max(1e-6,(lengths[k+2]-lengths[k])/2)
                  for k,(a,b) in enumerate(zip(yaws,yaws[1:]))]
            if any(abs(v)>curvature_limit for v in curv[:49]):
                attempts["curvature"]=attempts.get("curvature",0)+1; continue
            if any(abs(wrap_angle(b-a))>.8 for a,b in zip(yaws,yaws[1:])):
                continue
            poses=[(p[0],p[1],yaws[min(k,len(yaws)-1)]) for k,p in enumerate(curve)]
            collision=_shadow_collision(poses,lengths[-1],boundaries,self.shadow_settings)
            if collision:
                attempts["boundary"]=attempts.get("boundary",0)+1; continue
            candidate=deepcopy(traj)
            speed=min(cfg.v_exec_max_mps,points[0].get("target_speed_mps",cfg.v_exec_max_mps))
            candidate["points"]=[{"s_m":lengths[k],"x_m":p[0],"y_m":p[1],
                "yaw_rad":poses[k][2],"curvature_1pm":curv[min(k,len(curv)-1)] if curv else 0.,
                "target_speed_mps":speed} for k,p in enumerate(curve)]
            check,_=mpc_reference_errors(candidate,cfg.horizon_n,cfg.dt_pred_s,0.,
                                          cfg.max_backward_extension_m,0.)
            if check is None or check["extended_start"]:
                continue
            candidate["join_repair"]={"active":True,"original_reason":reason,
                "join_index":index,"join_length_m":lengths[min(48,len(lengths)-1)],
                "curvature_limit_1pm":curvature_limit,"elapsed_s":time.perf_counter()-started}
            return candidate
    unchanged=deepcopy(traj)
    unchanged["join_repair"]={"active":False,"original_reason":reason,
        "rejections":attempts,"elapsed_s":time.perf_counter()-started}
    return unchanged

MPCPolicyController._repair_mpc_reference = _repair_mpc_reference
