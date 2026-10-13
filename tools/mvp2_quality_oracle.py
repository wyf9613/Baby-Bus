"""Independent dense-geometry and closed-loop evaluation of 35 ms V2 output.

No planner score is used. Original methods are uninstrumented. Production
source and YAML remain untouched. Requires numpy/matplotlib locally.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import runpy
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NS = runpy.run_path(str(ROOT / "tests/test_estimation.py"))["namespace"]
SCENES = ["curve_0.04", "curve_0.12", "curve_0.25", "s_bend", "curvature_change"]
DIRECT_SAMPLE_OUTPUT = False


def truth(name, x):
    x = np.asarray(x)
    if name.startswith("curve_"):
        b = float(name.split("_")[1])
        return b*x*x, 2*b*x
    if name == "s_bend":
        return 0.12*np.sin(2*x), 0.24*np.cos(2*x)
    return 0.3*np.maximum(0, x-0.8)**2, 0.6*np.maximum(0, x-0.8)


class GridNearest:
    """Independent analytic projection, then neighboring 5 mm grid points.
    Newton projection is checked against exhaustive grid search in smoke QA.
    """
    def __init__(self, name, offset, xs):
        self.name, self.offset, self.xs = name, offset, xs

    def project(self, points):
        points = np.asarray(points)
        single = points.ndim == 1
        points = np.atleast_2d(points)
        px, py = points[:, 0], points[:, 1]
        x = np.clip(px.copy(), self.xs[0], self.xs[-1])
        for _ in range(18):
            y, dy = truth(self.name, x)
            if self.name.startswith("curve_"):
                ddy = np.full_like(x, 2*float(self.name.split("_")[1]))
            elif self.name == "s_bend":
                ddy = -0.48*np.sin(2*x)
            else:
                ddy = np.where(x > 0.8, 0.6, 0.0)
            f = x-px+(y+self.offset-py)*dy
            df = 1+dy*dy+(y+self.offset-py)*ddy
            x = np.clip(x-np.clip(f/np.maximum(df, 0.1), -0.2, 0.2), self.xs[0], self.xs[-1])
        return x

    def query(self, points):
        points = np.asarray(points)
        single = points.ndim == 1
        points = np.atleast_2d(points)
        px, py = points[:, 0], points[:, 1]
        x = self.project(points)
        idx = np.rint((x-self.xs[0])/0.005).astype(int)
        candidates = np.clip(idx[:, None]+np.arange(-2, 3), 0, len(self.xs)-1)
        cx = self.xs[candidates]
        cy, _ = truth(self.name, cx)
        d2 = (cx-px[:, None])**2+(cy+self.offset-py[:, None])**2
        best = np.argmin(d2, axis=1)
        dist = np.sqrt(d2[np.arange(len(points)), best])
        index = candidates[np.arange(len(points)), best]
        return (float(dist[0]), int(index[0])) if single else (dist, index)


class Oracle:
    """Ground-truth boundaries, sampled every 5 mm in world x, independent
    of LatticeCurve, planner clearance, and all planner cost functions.
    Euclidean distance to nearest sampled point has <= half-segment-spacing
    discretization error (larger than 2.5 mm on sloped boundaries).
    """
    def __init__(self, name, width=1.0):
        self.name = name
        self.half_width = width/2
        self.x = np.arange(-2, 8.0001, 0.005)
        y, slope = truth(name, self.x)
        self.center = np.column_stack((self.x, y))
        self.center_tree = GridNearest(name, 0, self.x)
        self.left = GridNearest(name, self.half_width, self.x)
        self.right = GridNearest(name, -self.half_width, self.x)
        self.slope = slope
        self.arc = np.r_[0, np.cumsum(np.hypot(np.diff(self.x), np.diff(y)))]
        self.origin_arc = np.interp(0, self.x, self.arc)
        # Sample each of the four body perimeter edges at <= 2 cm spacing.
        corners = np.array([[-0.198, -0.125], [0.297, -0.125], [0.297, 0.125], [-0.198, 0.125]])
        perimeter = []
        for a, b in zip(corners, np.roll(corners, -1, axis=0)):
            n = math.ceil(np.linalg.norm(b-a)/0.02)
            perimeter.extend(a+(b-a)*i/n for i in range(n))
        self.body = np.asarray(perimeter)

    def clearance(self, poses):
        poses = np.asarray(poses)
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        bx, by = self.body[:, 0], self.body[:, 1]
        px = poses[:, 0, None]+c[:, None]*bx-s[:, None]*by
        py = poses[:, 1, None]+s[:, None]*bx+c[:, None]*by
        points = np.column_stack((px.ravel(), py.ravel()))
        yc, _ = truth(self.name, points[:, 0])
        l = self.left.query(points)[0]*np.where(points[:, 1] <= yc+self.half_width, 1, -1)
        r = self.right.query(points)[0]*np.where(points[:, 1] >= yc-self.half_width, 1, -1)
        return float(np.minimum(l, r).min())

    def tracking(self, x, y):
        # Continuous projection avoids nearest-grid quantization being mistaken
        # for millimetre tracking error or temporal oscillation.
        projected_x = float(self.center_tree.project([x, y])[0])
        projected_y, _ = truth(self.name, projected_x)
        distance = math.hypot(x-projected_x, y-float(projected_y))
        yc, _ = truth(self.name, x)
        signed = float(distance)*(1 if y >= yc else -1)
        progress = float(np.interp(projected_x, self.x, self.arc)-self.origin_arc)
        return signed, progress


def world_transform(points, pose):
    x, y, heading = pose
    c, s = math.cos(heading), math.sin(heading)
    return np.column_stack((x+c*points[:, 0]-s*points[:, 1], y+s*points[:, 0]+c*points[:, 1]))


def dense_path(ref, pose, step=0.005):
    """Evaluate the serialized control representation independently.
    Audit the executed polynomial or interpolated samples, never planner score.
    """
    path = ref["path"]
    lo, hi = path["range"]
    xs = np.linspace(lo, hi, math.ceil((hi-lo)/step)+1)
    if path["type"] == "CARTESIAN_SAMPLES":
        samples = np.asarray(path["samples_xy_heading_curvature"])
        y = np.interp(xs, samples[:, 0], samples[:, 1])
        headings = np.interp(xs, samples[:, 0], np.unwrap(samples[:, 2]))
        local = np.column_stack((xs, y))
        return local, np.column_stack((world_transform(local, pose), headings+pose[2]))
    u = (xs-path["origin"])/path["scale"]
    c = np.asarray(path["coeffs_low_to_high"])
    y = np.polynomial.polynomial.polyval(u, c)
    dy = np.polynomial.polynomial.polyval(u, np.polynomial.polynomial.polyder(c))/path["scale"]
    local = np.column_stack((xs, y))
    world = world_transform(local, pose)
    poses = np.column_stack((world, np.arctan(dy)+pose[2]))
    return local, poses


def quality(ref, pose, oracle):
    local, poses = dense_path(ref, pose)
    return {"min_clearance_m": oracle.clearance(poses),
            "length_m": float(np.linalg.norm(np.diff(local, axis=0), axis=1).sum()),
            "points": local.tolist()}


def records(name, pose, speed, steering, now, width=1.0):
    # Retain the earlier study's y=f(x)+/-0.5 m world boundaries, with observed
    # rear support. Rotate actual samples into the current body frame.
    wx = np.arange(pose[0]-1.0, pose[0]+4.01, 0.025)
    cy, _ = truth(name, wx)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    road = {"valid": True, "visibility": "both", "frame_id": "base_link", "timestamp_s": now, "source_age_s": 0.01}
    curves = {}
    for key, offset in (("centerline_xy", 0), ("left_boundary_xy", width/2), ("right_boundary_xy", -width/2)):
        dx, dy = wx-pose[0], cy+offset-pose[1]
        px, py = c*dx+s*dy, -s*dx+c*dy
        # Retain the forward single-valued branch before a body-x fold.
        folds = np.flatnonzero(np.diff(px) <= 0)
        if len(folds):
            px, py = px[:folds[0]+1], py[:folds[0]+1]
        keep = (px >= -0.5) & (px <= 3.5)
        curves[key] = (px[keep], py[keep])
    # Common local-x grid avoids almost coincident endpoint knots introduced
    # solely by rotating separately sampled world boundaries then clipping.
    lo = math.ceil(max(v[0][0] for v in curves.values())/0.1+1e-7)*0.1
    hi = math.floor(min(v[0][-1] for v in curves.values())/0.1-1e-7)*0.1
    grid = np.linspace(lo, hi, round((hi-lo)/0.1)+1)
    for key, (px, py) in curves.items():
        road[key] = list(map(tuple, np.column_stack((grid, np.interp(grid, px, py))).tolist()))
    beta = math.atan(0.4*math.tan(steering))
    yaw = speed/0.33*math.cos(beta)*math.tan(steering)
    state = {"speed_valid": True, "yaw_rate_valid": True, "frame_id": "base_link", "timestamp_s": now,
             "speed_mps": speed, "yaw_rate_rps": yaw, "source_age_s": {"wheel_speed": 0.01, "imu_angular_velocity": 0.01}}
    return road, state, {"available": True, "frame_id": "base_link", "timestamp_s": now, "source_age_s": 0.01, "points_xyz_m": []}, {}, now


def oracle_qa():
    rng = np.random.default_rng(91)
    worst, count, max_stationarity = 0.0, 0, 0.0
    for name in SCENES:
        oracle = Oracle(name)
        px = rng.uniform(-0.5, 3, 200)
        yc, _ = truth(name, px)
        points = np.column_stack((px, yc+rng.uniform(-0.8, 0.8, 200)))
        for shift in (-0.5, 0, 0.5):
            distance, _ = GridNearest(name, shift, oracle.x).query(points)
            gy, _ = truth(name, oracle.x)
            exhaustive = np.sqrt(((points[:, None, 0]-oracle.x)**2+
                                  (points[:, None, 1]-gy-shift)**2).min(axis=1))
            error = float(np.max(np.abs(distance-exhaustive)))
            worst = max(worst, error)
            count += len(points)
            assert error < 1e-8, (name, shift, error)
            projected = GridNearest(name, shift, oracle.x).project(points)
            cy, dy = truth(name, projected)
            residual = np.abs(projected-points[:, 0]+(cy+shift-points[:, 1])*dy)
            max_stationarity = max(max_stationarity, float(residual.max()))
            assert residual.max() < 1e-8
            continuous = np.hypot(projected-points[:, 0], cy+shift-points[:, 1])
            assert np.all(continuous <= exhaustive+1e-10)
    return {"queries": count, "max_distance_discrepancy_m": worst,
            "max_projection_stationarity_residual": max_stationarity,
            "method": "independent projection versus exhaustive boundary-grid search"}
