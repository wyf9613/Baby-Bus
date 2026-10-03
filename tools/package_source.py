#!/usr/bin/env python3
"""Package the exact clean Git source; never fetch, install or publish."""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess
import tarfile
import xml.etree.ElementTree as ET

from check_history import check, git, require, ROOT


REQUIRED = ('package.xml', 'CMakeLists.txt', 'README.md', 'LICENSE',
            'CHANGELOG.md', 'CONTRIBUTING.md', 'docs/acceptance.md',
            'docs/release-v0.1.0.md', 'scripts/policy_node.py',
            'launch/ai4r_policy.launch.py', 'config/ai4r_policy.yaml',
            'ci/dependencies.repos')


def dependency_provenance(root):
    text = (Path(root) / 'ci/dependencies.repos').read_text(encoding='utf-8')
    url = re.search(r'^\s+url:\s*(\S+)\s*$', text, re.MULTILINE)
    commit = re.search(r'^\s+version:\s*([0-9a-f]{40})\s*$', text,
                       re.MULTILINE)
    require(url is not None and commit is not None,
            'invalid dream_interfaces source pin')
    return {'dream_interfaces': {'url': url.group(1),
                                 'commit': commit.group(1)}}


def package(root, output, environment=None):
    root, output = Path(root), Path(output)
    identity = check(root, environment)
    tag = identity.get('tag')
    label = tag or 'preview-' + identity['source_commit'][:12]
    prefix = identity['package'] + '-' + label + '/'
    data = subprocess.check_output([
        'git', '-C', str(root), 'archive', '--format=tar',
        '--prefix=' + prefix, 'HEAD'])
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:') as archive:
        for name in REQUIRED:
            member = archive.getmember(prefix + name)
            require(member.isfile(), 'missing source payload: ' + name)
        xml = archive.extractfile(prefix + 'package.xml').read()
        require(ET.fromstring(xml).findtext('version') == identity['version'],
                'archive version differs from source')
    epoch = int(git(root, 'show', '-s', '--format=%ct', 'HEAD'))
    compressed_file = io.BytesIO()
    with gzip.GzipFile(fileobj=compressed_file, mode='wb', filename='',
                       mtime=epoch) as archive:
        archive.write(data)
    compressed = compressed_file.getvalue()
    filename = prefix[:-1] + '.tar.gz'
    identity.setdefault('kind', 'preview')
    identity.update(dependencies=dependency_provenance(root),
                    source_archive=filename,
                    archive_sha256=hashlib.sha256(compressed).hexdigest())
    metadata = (json.dumps(identity, sort_keys=True, indent=2) + '\n').encode()
    files = {filename: compressed, 'release.json': metadata}
    files['SHA256SUMS'] = ''.join(
        hashlib.sha256(value).hexdigest() + '  ' + name + '\n'
        for name, value in sorted(files.items())).encode()
    output.mkdir(parents=True, exist_ok=False)
    for name, value in files.items():
        (output / name).write_bytes(value)
    return identity


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'build/release')
    args = parser.parse_args()
    print(json.dumps(package(ROOT, args.output), indent=2))
