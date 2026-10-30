"""Trainable native-segment encoders and per-path scene attention, using NumPy."""
import math

import numpy as np

from .features import ROAD_ROLES, BOUNDARY_SOURCES, OBSTACLE_SECTORS, unpack_input

PATH_CHANNELS = 14
SCENE_CHANNELS = 4+4+len(BOUNDARY_SOURCES)+OBSTACLE_SECTORS
STATE_CHANNELS = 8


def input_matrices(record):
    """Encode individual native segments, never a flattened spatial point list."""
    record = unpack_input(record)
    path = record['candidate_path']
    samples = np.asarray(path['samples'], dtype=float)
    geometry = np.asarray(path['geometry_xy_heading'], dtype=float)
    directions = np.column_stack((np.cos(geometry[:, 2]), np.sin(geometry[:, 2])))
    path_rows = np.column_stack((samples[:-1], samples[1:], geometry[:-1, :2],
                                geometry[1:, :2], directions[:-1], directions[1:]))
    scene = record['current_road_and_obstacles']
    rows, present = [], []
    for index, role in enumerate(ROAD_ROLES):
        curve = scene['road'][role]
        points = np.asarray(curve['points_xy'], dtype=float).reshape(-1, 2)
        present.append(float(len(points) > 0))
        if not len(points):
            continue
        starts, ends = (points[:-1], points[1:]) if len(points) > 1 else (points, points)
        for a, b in zip(starts, ends):
            row = np.zeros(SCENE_CHANNELS)
            row[:4] = [*a, *b]
            row[4+index] = 1.
            row[8+BOUNDARY_SOURCES.index(curve['source'])] = 1.
            rows.append(row)
    for index, (x, y, exists) in enumerate(scene['obstacles']['sectors_xy_present']):
        if exists:
            row = np.zeros(SCENE_CHANNELS)
            row[:4] = [x, y, x, y]
            row[7] = 1.  # obstacle role
            row[8] = 1.  # observed source
            row[8+len(BOUNDARY_SOURCES)+index] = 1.
            rows.append(row)
    state = np.asarray([*record['current_state'], *present,
                        float(scene['obstacles']['available'])], dtype=float)
    return dict(state=state, path=path_rows,
                scene=np.asarray(rows, dtype=float).reshape(-1, SCENE_CHANNELS))


class SpatialNetwork:
    """Each forward call scores one path; candidate batches never interact."""
    def __init__(self, hidden=32, seed=0, *, recurrent=False):
        self.hidden = hidden
        self.recurrent=recurrent
        rng = np.random.default_rng(seed)
        def weight(rows, columns):
            return rng.normal(0., 1/math.sqrt(rows), (rows, columns))
        self.parameters = dict(path_w=weight(PATH_CHANNELS, hidden), path_b=np.zeros(hidden),
            scene_w=weight(SCENE_CHANNELS, hidden), scene_b=np.zeros(hidden),
            empty_scene=np.zeros(hidden), mix_w=weight(2*hidden, hidden), mix_b=np.zeros(hidden),
            head_w=weight(hidden+STATE_CHANNELS, hidden), head_b=np.zeros(hidden),
            output_w=rng.normal(0., 1/math.sqrt(hidden), hidden), output_b=np.zeros(1))
        if recurrent:self.parameters['recurrent_w']=np.eye(hidden)*.1

    def forward(self, matrices, previous_hidden=None):
        p, h = self.parameters, self.hidden
        path = np.tanh(matrices['path']@p['path_w']+p['path_b'])
        empty = not len(matrices['scene'])
        scene = (p['empty_scene'][None, :] if empty else
                 np.tanh(matrices['scene']@p['scene_w']+p['scene_b']))
        logits = path@scene.T/math.sqrt(h)
        weights = np.exp(logits-logits.max(axis=1, keepdims=True))
        weights /= weights.sum(axis=1, keepdims=True)
        paired = np.column_stack((path, weights@scene))
        mixed = np.tanh(paired@p['mix_w']+p['mix_b'])
        head_input = np.r_[mixed.mean(axis=0), matrices['state']]
        previous_hidden=np.zeros(h) if previous_hidden is None else np.asarray(previous_hidden,dtype=float)
        if previous_hidden.shape!=(h,) or not np.isfinite(previous_hidden).all():raise ValueError('Invalid recurrent hidden state')
        head = np.tanh(head_input@p['head_w']+p['head_b']+
                       (previous_hidden@p['recurrent_w'] if self.recurrent else 0.))
        logit = float(head@p['output_w']+p['output_b'][0])
        score = 1/(1+math.exp(-logit)) if logit >= 0 else math.exp(logit)/(1+math.exp(logit))
        return score, (matrices, path, scene, weights, paired, mixed, head_input, head, score, empty,previous_hidden)

    def backward(self, cache, score_derivative, hidden_derivative=None, *, return_hidden_gradient=False):
        matrices, path, scene, attention, paired, mixed, head_input, head, score, empty,previous_hidden = cache
        p, h = self.parameters, self.hidden
        gradients = {name: np.zeros_like(value) for name, value in p.items()}
        dz = score_derivative*score*(1-score)
        gradients['output_w'] = head*dz
        gradients['output_b'][0] = dz
        dhead = (p['output_w']*dz+(0. if hidden_derivative is None else hidden_derivative))*(1-head**2)
        if self.recurrent:gradients['recurrent_w']=np.outer(previous_hidden,dhead)
        previous_gradient=dhead@p['recurrent_w'].T if self.recurrent else np.zeros(h)
        gradients['head_w'] = np.outer(head_input, dhead)
        gradients['head_b'] = dhead
        dpool = (dhead@p['head_w'].T)[:h]
        dmixed = np.broadcast_to(dpool/len(mixed), mixed.shape)*(1-mixed**2)
        gradients['mix_w'] = paired.T@dmixed
        gradients['mix_b'] = dmixed.sum(axis=0)
        dpaired = dmixed@p['mix_w'].T
        dpath, dcontext = dpaired[:, :h], dpaired[:, h:]
        dattention = dcontext@scene.T
        dscene = attention.T@dcontext
        dlogits = attention*(dattention-(attention*dattention).sum(axis=1, keepdims=True))
        dpath += dlogits@scene/math.sqrt(h)
        dscene += dlogits.T@path/math.sqrt(h)
        dpath *= 1-path**2
        gradients['path_w'] = matrices['path'].T@dpath
        gradients['path_b'] = dpath.sum(axis=0)
        if empty:
            gradients['empty_scene'] = dscene.sum(axis=0)
        else:
            dscene *= 1-scene**2
            gradients['scene_w'] = matrices['scene'].T@dscene
            gradients['scene_b'] = dscene.sum(axis=0)
        return (gradients,previous_gradient) if return_hidden_gradient else gradients
