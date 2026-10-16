"""Use normal module imports; retain the pinned source/config as provenance."""
from functools import lru_cache
import hashlib
import json
from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parent
ROOT = PACKAGE.parents[1]
SOURCE = PACKAGE / 'upstream/3652e9b/scripts/policy_node.py'
UPSTREAM_CONFIG = PACKAGE / 'upstream/3652e9b/config/ai4r_policy.yaml'
CONFIG = ROOT / 'config/ai4r_policy.yaml'
UPSTREAM_COMMIT = '3652e9ba6f42180cb28f27c02299d5b7a0ec77b0'
SOURCE_SHA256 = '9f18ff37612b29a9e1d6f0baaf9d4cfb5c3e99604b26a7bab616953215a5f5dc'
UPSTREAM_CONFIG_SHA256 = '2e51dfcf5d51e78203043188904e866ccaecc5ec28b7a9fd0e5339bae60e809c'
CONFIG_SHA256 = hashlib.sha256(CONFIG.read_bytes().replace(b'\r\n', b'\n')).hexdigest()


def checked_bytes(path, expected):
    data = path.read_bytes()
    # Git's text checkout can use CRLF; the authoritative Git blob uses LF.
    data = data.replace(b'\r\n', b'\n')
    if hashlib.sha256(data).hexdigest() != expected:
        raise ValueError('Pinned MVP2.1 source/config changed: ' + str(path))
    return data


@lru_cache(maxsize=1)
def api():
    checked_bytes(SOURCE, SOURCE_SHA256)
    from mvp21_policy import api as modules
    return modules


def runtime_provenance():
    from mvp21_policy.provenance import source_fingerprint
    return source_fingerprint()


def parameters():
    checked_bytes(UPSTREAM_CONFIG, UPSTREAM_CONFIG_SHA256)
    return yaml.safe_load(checked_bytes(CONFIG, CONFIG_SHA256))['/**/ai4r_policy']['ros__parameters']


def verify_preserved_files():
    """Check protected artifacts; report explicitly authorized source migrations."""
    manifest = json.loads((PACKAGE/'preserved-files.json').read_text(encoding='utf-8'))
    changed = []
    for name, expected in manifest['sha256_lf'].items():
        path = ROOT/name
        data = path.read_bytes() if path.is_file() else b''
        if path.suffix in {'.py','.json','.md','.txt','.yaml','.yml','.html'} or path.name=='.gitattributes':
            data = data.replace(b'\r\n', b'\n')
        if hashlib.sha256(data).hexdigest() != expected:
            changed.append(name)
    if changed:
        raise ValueError('Preserved files changed: ' + ', '.join(changed))
    return dict(checkpoint_commit=manifest['checkpoint_commit'],checked_files=len(manifest['sha256_lf']),
                unchanged=True,authorized_migrations=len(manifest.get('authorized_migrations',{})),
                migration_reason=manifest.get('migration_reason'))
