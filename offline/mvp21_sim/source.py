"""Load pinned, byte-identical upstream pure modules without the policy or ROS node."""
import ast
from copy import deepcopy
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import json
import math
from numbers import Real
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace

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
    tree = ast.parse(checked_bytes(SOURCE, SOURCE_SHA256))
    nodes = []
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == 'PolicyNode':
            break
        if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name not in {
                '_shadow_worker_entry', 'ShadowWorker'}:
            nodes.append(node)
    policy = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name=='PolicyNode')
    offer = next(n for n in policy.body if isinstance(n, ast.FunctionDef) and n.name=='_shadow_offer')
    # Reuse the actual upstream selection/forward-geometry fallback, without
    # extracting or executing calculate_policy_actions or the policy lifecycle.
    nodes.append(ast.fix_missing_locations(ast.ClassDef(name='ShadowJobSource',bases=[],keywords=[],
        body=[offer],decorator_list=[])))
    module = ModuleType('_mvp21_sim_' + SOURCE_SHA256[:16])
    module.__dict__.update(dataclass=dataclass, deepcopy=deepcopy, math=math, Real=Real,
        time=time, ConeDetection=SimpleNamespace(COLOR_YELLOW=1, COLOR_BLUE=2))
    sys.modules[module.__name__] = module
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), 'exec'), module.__dict__)
    return module


def parameters():
    return yaml.safe_load(checked_bytes(CONFIG, CONFIG_SHA256))['/**/ai4r_policy']['ros__parameters']


def verify_preserved_files():
    """Check the original policy, model, config and simulator against the checkpoint."""
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
                unchanged=True)
