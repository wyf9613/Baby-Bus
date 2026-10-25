"""ROS-free JSON worker, usable as a module, a file or inside a source bundle."""
import hashlib
import json
import os
from pathlib import Path
import sys
import time

# ShadowWorker launches this actual file. The same layout is restored by the
# bundler, so a child needs neither the caller's cwd nor inherited PYTHONPATH.
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mvp21_policy.lattice import ShadowLatticePlanner, ShadowSettings


def worker_source_sha256():
    digest = hashlib.sha256()
    for name in ('geometry.py', 'frenet.py', 'lattice.py', 'candidates.py', 'learned.py', 'worker.py'):
        digest.update(name.encode()+b'\0')
        digest.update(Path(__file__).with_name(name).read_bytes().replace(b'\r\n', b'\n'))
    return digest.hexdigest()


def run_worker():
    """Keep the upstream one-job/one-result protocol and generation fence."""
    if hasattr(os, 'nice'):
        try:
            os.nice(10)
        except OSError:
            pass
    source_sha256 = worker_source_sha256()
    policy, policy_key = None, None
    for line in sys.stdin:
        job = None
        decode_started = time.perf_counter()
        try:
            job = json.loads(line)
            decode_s = time.perf_counter()-decode_started
            result = ShadowLatticePlanner(ShadowSettings(**job['settings'])).plan(job)
            if (job.get('learned_policy') or {}).get('enabled'):
                from mvp21_policy.learned import FrozenPathPolicy, LearnedPolicySettings
                settings = LearnedPolicySettings(**job['learned_policy'])
                key = (settings.model_path, settings.model_sha256, job.get('policy_cruise_speed_mps'))
                if key != policy_key:
                    policy = FrozenPathPolicy(settings, key[2])
                    policy_key = key
                result = policy.select(result, job.get('policy_observation'), job.get('policy_context') or {})
                elapsed = result.get('elapsed_s', 0)+result['policy_selection'].get('wall_s', 0)
                result.update(elapsed_s=elapsed, late_for_requested_period=elapsed > 1/job['settings']['rate_hz'])
                result.setdefault('timing_s', {}).update(learned_policy=result['policy_selection'].get('wall_s', 0), total=elapsed)
        except Exception as error:
            decode_s = time.perf_counter()-decode_started
            result = dict(status='WORKER_ERROR', error=str(error), selected=None)
        result['generation'] = None if job is None else job.get('generation')
        result['job_id'] = None if job is None else job.get('job_id')
        result['source_sha256'] = source_sha256
        result['source_sha256_format'] = 'mvp21_worker_policy_lattice_v3'
        result['execution_anchor'] = None if job is None else dict(
            pose=job.get('shadow_pose'), odometry_epoch=job.get('shadow_odometry_epoch'))
        result['shadow_only'] = not bool(job and job.get('planner_control_enabled'))
        if job and job.get("compact_result"):
            # Rank ALL original candidates first, then omit only unused wire
            # geometry. Selected path, recurrent state, collision/fusion and
            # execution anchors remain exact. Default offline payload unchanged.
            candidates=result.pop("candidates", [])
            result["candidate_summaries"]=[{k:c.get(k) for k in ("id","family","collision","cost","controller_rejection")}
                                            for c in candidates]
            result["compact_result"]=True
        encode_started = time.perf_counter()
        encoded = json.dumps(result, allow_nan=False, separators=(',', ':'))
        wire_timing = dict(json_decode_s=decode_s, json_encode_s=time.perf_counter()-encode_started)
        print(encoded[:-1]+',"wire_timing_s":'+json.dumps(wire_timing)+'}', flush=True)


if __name__ == '__main__':
    run_worker()
