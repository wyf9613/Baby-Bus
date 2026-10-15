"""Use normal module imports; retain the pinned source/config as provenance."""
from functools import lru_cache
import hashlib
import json
from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parent
ROOT = PACKAGE.parents[1]
SOURCE = PACKAGE / 'upstream/scripts/policy_node.py'
CONFIG = PACKAGE / 'upstream/config/ai4r_policy.yaml'
UPSTREAM_COMMIT = 'f11428b5444ac4c714a95e86e71662086b3187bd'
SOURCE_SHA256 = '524ffcb244a85e98d5993c273e0ff684629f44be6128a102e5ff31096ef26edd'
CONFIG_SHA256 = 'f9690e39ca898fb878c9ffa6780c1824b805c3d1f5cab912220d9ff5e48dacf0'


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
