"""Explicit offline path handoff; never imported by the deployed policy.

The centerline planner still supplies the admission/freshness envelope. Only its
geometry is replaced, without fitting, by a VALID shadow candidate. This tests
path selection with the existing controller, not a complete planner protocol.
"""
from copy import deepcopy
import math


def selected_reference(envelope, result):
    """Keep current-cycle metadata and replace geometry, or fail closed."""
    reference = deepcopy(envelope)
    status = result.get('status')
    reason = None
    if not envelope.get('valid'):
        reason = 'lattice_envelope_invalid'
    elif status != 'VALID':
        reason = 'lattice_'+str(status)
    elif result.get('input_stamp_s') != envelope.get('timestamp_s'):
        reason = 'lattice_input_time_mismatch'
    else:
        path = (result.get('selected') or {}).get('path')
        try:
            points = [(float(p[0]), float(p[1])) for p in path]
            lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
            if (len(points) < 2 or not all(math.isfinite(v) for p in points for v in p)
                    or not lengths or min(lengths) <= 0):
                raise ValueError('degenerate selected path')
        except (TypeError, ValueError, IndexError, OverflowError):
            reason = 'lattice_selected_path_invalid'
    if reason:
        reference.update(valid=False, reason=reason, path=None, target_speed_mps=0.,
                         stop_requested=True)
        return reference
    reference.update(planner_version='offline_mvp21_shadow_lattice_handoff',
        path=dict(type='ORDERED_CENTERLINE', independent_variable='s_m',
                  samples_xy=points, range=[0., sum(lengths)],
                  preview_m=envelope['path']['preview_m']))
    reference['offline_path_experiment'] = dict(
        path_source='lattice', upstream_shadow_only=True,
        generation_timestamp_s=result['input_stamp_s'],
        selected_id=result['selected']['id'], selected_family=result['selected']['family'],
        check_length_m=result['check_length_m'], score_length_m=result['score_length_m'],
        generated_length_m=result['generated_length_m'],
        admission_source='unchanged RoadCenterlinePlanner envelope')
    return reference
