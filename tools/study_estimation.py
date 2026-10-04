"""Bounded synthetic comparison; no ROS, hardware or ground-truth policy input."""
from pathlib import Path
import hashlib
import json
import math
import random
import runpy
import statistics
import time

root = Path(__file__).resolve().parents[1]
api = runpy.run_path(str(root / 'tests/test_estimation.py'))
pipeline = api['Pipeline'](api['Settings']())
solve = api['namespace']['_estimation_solve']
xs = [0.5+0.3*i for i in range(8)]
def truth(x):
    return 0.1*x+0.04*x*x
def ordinary(points):
    basis = [[1, x, x*x] for x, _ in points]
    matrix = [[sum(row[i]*row[j] for row in basis) for j in range(3)] for i in range(3)]
    rhs = [sum(row[i]*p[1] for row, p in zip(basis, points)) for i in range(3)]
    return solve(matrix, rhs)

errors, ordinary_errors, runtimes = [], [], []
valid = 0
for seed in range(5):
    rng = random.Random(seed)
    for frame in range(20):
        left = [(x, truth(x)+0.5+rng.gauss(0, 0.03)) for x in xs]
        right = [(x, truth(x)-0.5+rng.gauss(0, 0.03)) for x in xs]
        # One grossly misplaced cone per frame, random side and position.
        side = left if rng.randrange(2) else right
        k = rng.randrange(len(xs))
        side[k] = (side[k][0], side[k][1]+rng.choice([-0.8, 0.8]))
        base_l, base_r = ordinary(left), ordinary(right)
        t0 = time.perf_counter()
        road = pipeline.road(api['cones'](left, right), 10_000_000_000, 0.05)
        runtimes.append((time.perf_counter()-t0)*1000)
        if not road['valid']:
            continue
        valid += 1
        samples = road['centerline_xy']
        errors.append(math.sqrt(sum((y-truth(x))**2 for x, y in samples)/len(samples)))
        ordinary_errors.append(math.sqrt(sum(((sum((a+b)/2*x**i for i, (a,b) in enumerate(zip(base_l,base_r))))-truth(x))**2 for x,_ in samples)/len(samples)))

runtime_sorted = sorted(runtimes)
report = {
    'scenario': 'synthetic local curved road; 0.03 m y-noise; one +/-0.8 m outlier/frame',
    'seeds': list(range(5)), 'frames_per_seed': 20, 'frames_total': 100,
    'valid_frames': valid, 'rejected_frames': 100-valid,
    'comparison_scope': 'same samples in valid robust frames; rejected frames reported separately',
    'robust_mean_frame_rmse_m': statistics.mean(errors) if errors else None,
    'ordinary_mean_frame_rmse_m': statistics.mean(ordinary_errors) if ordinary_errors else None,
    'runtime_ms': {'median': statistics.median(runtimes), 'p95': runtime_sorted[94], 'max': max(runtimes)},
    'configuration': vars(api['Settings']()),
    'limits': 'synthetic single-frame geometry; no x-noise, motion fusion, ROS latency or physical driving',
    'source_sha256': hashlib.sha256(api['SOURCE'].read_bytes()).hexdigest(),
}
print(json.dumps(report, indent=2, allow_nan=False))
