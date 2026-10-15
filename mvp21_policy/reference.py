"""Bounded lattice geometry reuse between planner deliveries; no actuator cache.

This consumer is opt-in. ROS shadow publication and the legacy straight-path
recovery remain unchanged. The caller still admits current sensors/state.
"""
from copy import deepcopy
import math

from .control import control_path_geometry
from .geometry import _path_lengths, _path_points, finite_number


class LatticeReferenceManager:
    def __init__(self, generation=1, frame_id='base_link', max_source_age_s=None):
        if max_source_age_s is not None and (not finite_number(max_source_age_s) or max_source_age_s <= 0):
            raise ValueError('lattice_invalid_source_age_limit')
        self.frame_id = frame_id
        self.max_source_age_s = max_source_age_s
        self.reset(generation)

    def reset(self, generation):
        self.generation, self.job_id = generation, 0
        self.result = self.points = self.frame_time_s = self.motion_reset_id = None
        self.deadline_s = None
        self.travelled_m = 0.
        self.diagnostics = {}

    def accept(self, result, now_s, motion):
        """Fence old runs/jobs; hard failures never silently reuse a path."""
        if result.get('generation') != self.generation:
            return False
        identifier = result.get('job_id')
        if not isinstance(identifier, int) or isinstance(identifier, bool):
            raise ValueError('lattice_invalid_job_id')
        if identifier <= self.job_id:
            return False
        if result.get('status') != 'VALID':
            self.points = None
            raise ValueError('lattice_'+str(result.get('status')))
        stamp, hold = result.get('input_stamp_s'), result.get('hold_time_s')
        if (not all(finite_number(v) for v in (stamp, hold, now_s)) or hold <= 0
                or not stamp <= now_s < stamp+hold):
            raise ValueError('lattice_result_expired_or_future')
        if self.result is not None and stamp <= self.result['input_stamp_s']:
            return False
        selected = result.get('selected') or {}
        if selected.get('collision') is not None:
            raise ValueError('lattice_selected_collision')
        try:
            points = _path_points([p[:2] for p in selected['path']])
        except (KeyError, TypeError, IndexError):
            points = None
        if points is None:
            raise ValueError('lattice_selected_path_invalid')
        pose, problem = motion.pose_between_segments(stamp, now_s)
        if problem:
            raise ValueError('lattice_motion:'+problem)
        source_stamp = result.get('measurement_stamp_s')
        if not finite_number(source_stamp):
            source_age = ((result.get('planning_input') or {}).get('road') or {}).get('source_age_s',0.)
            source_stamp = stamp-source_age if finite_number(source_age) and source_age >= 0 else stamp
        if source_stamp > stamp:
            raise ValueError('lattice_measurement_time_invalid')
        deadline = stamp+hold
        if self.max_source_age_s is not None:
            deadline = min(deadline,source_stamp+self.max_source_age_s)
        if not now_s < deadline:
            raise ValueError('lattice_source_expired')
        self.points = [motion.transform_xy(point, pose) for point in points]
        self.result, self.job_id = deepcopy(result), identifier
        self.frame_time_s, self.motion_reset_id = now_s, motion.reset_id
        self.deadline_s = deadline  # Never restarted at receipt or on control ticks.
        self.travelled_m = pose['travelled_distance_m']
        return True

    def select(self, envelope, state, motion, now_s):
        """Transform the cached polyline into this cycle's vehicle frame."""
        if self.points is None:
            raise ValueError('lattice_no_reference')
        if (not envelope.get('valid') or not state.get('valid') or not state.get('speed_valid')
                or not state.get('yaw_rate_valid')):
            raise ValueError('lattice_current_input_invalid')
        if (envelope.get('frame_id') != self.frame_id or state.get('frame_id') != self.frame_id
                or envelope.get('vehicle_reference_point') != 'cg_ground_projection'
                or abs(state.get('timestamp_s', math.inf)-now_s) > 1e-6
                or abs(envelope.get('timestamp_s', math.inf)-now_s) > 1e-6):
            raise ValueError('lattice_current_frame_or_time_mismatch')
        if motion.reset_id != self.motion_reset_id:
            raise ValueError('lattice_motion_history_reset')
        if not self.frame_time_s <= now_s < self.deadline_s-1e-9:
            raise ValueError('lattice_reference_expired_or_clock_reversed')
        pose, problem = motion.pose_between_segments(self.frame_time_s, now_s)
        if problem:
            raise ValueError('lattice_motion:'+problem)
        points = [motion.transform_xy(point, pose) for point in self.points]
        path = dict(type='ORDERED_CENTERLINE', independent_variable='s_m', samples_xy=points,
                    range=[0., _path_lengths(points)[-1]], preview_m=envelope['path']['preview_m'])
        geometry = control_path_geometry(path)
        remaining = path['range'][1]-geometry['closest_s_m']
        if remaining+1e-9 < path['preview_m']:
            raise ValueError('lattice_path_support_exhausted')
        lifetime = min(envelope['valid_for_s'], self.deadline_s-now_s)
        if not finite_number(lifetime) or lifetime <= 0:
            raise ValueError('lattice_execution_reference_expired')
        self.points, self.frame_time_s = points, now_s
        self.travelled_m += pose['travelled_distance_m']
        self.diagnostics = dict(job_id=self.job_id,generation=self.generation,
            input_timestamp_s=self.result['input_stamp_s'],frame_timestamp_s=now_s,
            path_age_s=now_s-self.result['input_stamp_s'],deadline_s=self.deadline_s,
            remaining_path_m=remaining,travelled_since_input_m=self.travelled_m,
            original_check_length_m=self.result['check_length_m'],
            original_score_length_m=self.result['score_length_m'],
            selected_id=self.result['selected']['id'],selected_family=self.result['selected']['family'])
        reference = deepcopy(envelope)
        reference.update(planner_version='mvp21_lattice_cached',path=path,valid_for_s=lifetime,
                         reason='lattice_current_frame_available')
        reference['lattice_reference'] = dict(self.diagnostics)
        return reference
