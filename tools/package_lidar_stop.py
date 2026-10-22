#!/usr/bin/env python3
"""Package the current MVP2.1 source and common car profile without deploying."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

import yaml


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='New deployment directory; existing paths are refused')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.expanduser().resolve()
    archive = output.with_name(output.name + '.tar.gz')
    if output.exists() or archive.exists():
        parser.error('Use a new output path to preserve earlier deployment evidence')
    parameters = yaml.safe_load((root / 'config/ai4r_policy.yaml').read_text())[
        '/**/ai4r_policy']['ros__parameters']
    if not parameters['obstacle_stop']['enabled']:
        parser.error('The common configuration must explicitly enable obstacle_stop')
    output.mkdir(parents=True)
    subprocess.run([sys.executable, str(root / 'tools/bundle_python.py'),
                    'scripts/policy_node.py', '--root', str(root),
                    '--output', str(output / 'policy_node.py')], cwd=root, check=True)
    shutil.copyfile(root / 'config/ai4r_policy.yaml', output / 'ai4r_policy.yaml')
    shutil.copyfile(root / 'docs/LIDAR_STOP_DEPLOYMENT_CN.md', output / '说明.md')
    sources = sorted((root / 'mvp21_policy').glob('*.py')) + [
        root / 'scripts/policy_node.py', root / 'config/ai4r_policy.yaml',
        root / 'tools/bundle_python.py', Path(__file__).resolve(),
        root / 'docs/LIDAR_STOP_DEPLOYMENT_CN.md']
    manifest = {
        'git_head': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
        'git_branch': subprocess.check_output(
            ['git', 'branch', '--show-current'], cwd=root, text=True).strip(),
        'working_tree_changes': subprocess.check_output(
            ['git', 'status', '--porcelain'], cwd=root, text=True).splitlines(),
        'source_sha256': {str(path.relative_to(root)): sha256(path) for path in sources},
        'files_sha256': {path.name: sha256(path) for path in sorted(output.iterdir())},
        'obstacle_stop': parameters['obstacle_stop'],
        'target_speed_mps': parameters['planning']['cruise_speed_mps'],
        'hardware_used_by_packager': False,
    }
    (output / 'MANIFEST.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with tarfile.open(archive, 'w:gz') as packaged:
        packaged.add(output, arcname=output.name)
    checksum = archive.with_name(archive.name + '.sha256')
    checksum.write_text(f'{sha256(archive)}  {archive.name}\n', encoding='utf-8')
    print(json.dumps({'directory': str(output), 'archive': str(archive),
                      'sha256': sha256(archive)}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
