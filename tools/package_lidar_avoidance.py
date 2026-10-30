#!/usr/bin/env python3
"""Build the complete opt-in MVP2.1 LiDAR avoidance candidate, never deploy it."""
from dataclasses import asdict
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mvp21_policy.avoidance import LidarAvoidanceSettings


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def profile(*, proximity_stop=True):
    data = yaml.safe_load((ROOT/'config/ai4r_policy.yaml').read_text())
    p = data['/**/ai4r_policy']['ros__parameters']
    if not p['obstacle_stop']['enabled'] or p['control']['mode'] != 'mvp':
        raise ValueError('Package requires the integrated MVP stop baseline')
    p['policy_update_mode'] = 'timer'
    p['policy_update_rate_hz'] = 20.
    p['control'].update(enabled=True, robustness_enabled=False, end_straight_distance_m=0.)
    p['planner_control']['enabled'] = True
    p['planning']['cruise_speed_mps'] = .50
    p['learned_policy'] = dict(enabled=True, model_path='models/path_policy.json',
        model_sha256='a3116bb7bc4bb46ccb6e208fa3de628eca940820810943409c3157e5b12aa66b')
    p['estimation'].update(colour_assignment='auto', left_colour=2, right_colour=1)
    stop = p['obstacle_stop']
    stop['proximity_stop_enabled'] = proximity_stop
    p['lidar_avoidance'] = asdict(LidarAvoidanceSettings(enabled=True,
        maneuver_enabled=True, max_curvature_1pm=1.0,
        obstacle_margin_m=.02, tracking_margin_m=.01, boundary_margin_m=.01,
        target_lateral_allowance_m=.02, maneuver_clearance_m=.02,
        adaptive_shift_enabled=True,
        road_edge_exclusion_m=.10,
        front_extent_m=stop['front_extent_m'], rear_extent_m=stop['rear_extent_m'],
        width_m=stop['width_m'], max_source_age_s=stop['max_source_age_s']))
    # Ordinary following uses the successful stop baseline workload. Only an
    # active task asks transport for its dedicated longer/faster reference.
    p['shadow'].update(enabled=True, length_m=1.0, rate_hz=1.,
        collision_coverage='prefix', score_coverage='prefix', fusion_use_prior=False,
        body_front_m=stop['front_extent_m'], body_rear_m=stop['rear_extent_m'], body_width_m=stop['width_m'])
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--no-proximity-stop', action='store_true',
        help='Diagnostic profile: disable the forward proximity zone; retain source/body/path protection')
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    archive = output.with_name(output.name+'.tar.gz')
    if output.exists() or archive.exists():
        parser.error('Use a new output path; previous candidates are immutable')
    data = profile(proximity_stop=not args.no_proximity_stop)
    output.mkdir(parents=True)
    subprocess.run([sys.executable, str(ROOT/'tools/bundle_python.py'), 'scripts/policy_node.py',
        '--root', str(ROOT), '--resource', 'mvp21_policy/models/path_policy.json',
        '--output', str(output/'policy_node.py')], cwd=ROOT, check=True)
    (output/'ai4r_policy.yaml').write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
    shutil.copy2(ROOT/'docs/LIDAR_AVOIDANCE_DEPLOYMENT_CN.md', output/'说明.md')
    shutil.copy2(ROOT/'docs/RNN_MANEUVER_ALIGNMENT_20261030.json', output/'算法来源.json')
    shutil.copy2(ROOT/'docs/RNN_MANEUVER_ALIGNMENT_CN.md', output/'RNN_MANEUVER_ALIGNMENT_CN.md')
    sources = [*sorted((ROOT/'mvp21_policy').glob('*.py')),
        ROOT/'mvp21_policy/models/path_policy.json',
        *[ROOT/'offline/lattice_training'/name for name in ('features.py','spatial.py','source.py')],
        ROOT/'scripts/policy_node.py',
        ROOT/'config/ai4r_policy.yaml', ROOT/'tools/package_lidar_avoidance.py',
        ROOT/'tools/bundle_python.py', ROOT/'docs/LIDAR_AVOIDANCE_DEPLOYMENT_CN.md',
        ROOT/'docs/RNN_MANEUVER_ALIGNMENT_20261030.json', ROOT/'docs/RNN_MANEUVER_ALIGNMENT_CN.md',
        ROOT/'package.xml', ROOT/'ci/dependencies.repos']
    manifest = dict(git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        working_tree=subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines(),
        source_sha256={str(p.relative_to(ROOT)):sha256(p) for p in sources},
        files_sha256={p.name:sha256(p) for p in output.iterdir()},
        profile=data['/**/ai4r_policy']['ros__parameters'],hardware_used=False,
        vehicle_bound=False,physical_validation='not performed by packager',
        dependencies=['ROS 2 Jazzy','pinned dream_interfaces','python3-numpy'])
    (output/'MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    with tarfile.open(archive,'w:gz') as tar:
        tar.add(output,arcname=output.name)
    archive.with_name(archive.name+'.sha256').write_text(sha256(archive)+'  '+archive.name+'\n')
    print(json.dumps(dict(directory=str(output),archive=str(archive),sha256=sha256(archive)),ensure_ascii=False))


if __name__=='__main__':
    main()
