"""Read the native planner geometry needed by policy training, without ROS."""
import ast
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import math
from numbers import Real
from pathlib import Path
import sys
import time
from types import ModuleType

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "scripts/policy_node.py"
_CANDIDATE_SOURCE_CACHE = None
TRAINING_EXPORT_VERSION = "native_frenet_export_v1"


def _candidate_nodes(data):
    tree = ast.parse(data)
    boundary = next(node.lineno for node in tree.body
                    if isinstance(node, ast.ClassDef) and node.name == "ControlSettings")
    # This shared polynomial primitive lives below ControlSettings in the source,
    # but is also used by Frenet geometry. Include it without control algorithms.
    return [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))
            and (node.lineno < boundary or node.name in {"_control_poly_eval", "_control_poly_roots"})]


def _candidate_source():
    global _CANDIDATE_SOURCE_CACHE
    data = SOURCE.read_bytes()
    full_digest = hashlib.sha256(data).hexdigest()
    if _CANDIDATE_SOURCE_CACHE is None or _CANDIDATE_SOURCE_CACHE[0] != full_digest:
        nodes = _candidate_nodes(data)
        canonical = ast.dump(ast.Module(body=nodes, type_ignores=[]), include_attributes=False)
        digest = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
        _CANDIDATE_SOURCE_CACHE = (full_digest, digest, nodes)
    return _CANDIDATE_SOURCE_CACHE[1:]


def planner_source_sha256():
    """Pin native geometry and training admission semantics, excluding control edits."""
    return hashlib.sha256((_candidate_source()[0]+":"+TRAINING_EXPORT_VERSION).encode()).hexdigest()


def candidate_api():
    """Only geometry/estimation/planner definitions needed to encode candidates."""
    digest, nodes = _candidate_source()
    name = "_baby_bus_candidate_api_" + digest[:16]
    if name not in sys.modules:
        api = ModuleType(name)
        api.__dict__.update(dataclass=dataclass, deepcopy=deepcopy, math=math, Real=Real, time=time)
        sys.modules[name] = api
        try:
            exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"), api.__dict__)
        except Exception:
            del sys.modules[name]
            raise
        api.source_sha256 = digest
    return sys.modules[name]
