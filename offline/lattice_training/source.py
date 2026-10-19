"""Fingerprint latest planner and public policy candidate adapters; no AST loading."""
import hashlib
import json
from pathlib import Path
from .features import FEATURE_VERSION,FEATURE_SCHEMA

ROOT=Path(__file__).resolve().parents[2]
POLICY_INPUT_SCHEMA=FEATURE_VERSION


def planner_source_sha256():
    files=('mvp21_policy/geometry.py','mvp21_policy/frenet.py','mvp21_policy/lattice.py','mvp21_policy/candidates.py',
           'offline/lattice_training/planner.py','offline/lattice_training/reference.py')
    digest=hashlib.sha256(POLICY_INPUT_SCHEMA.encode())
    for name in files:
        digest.update(name.encode()+b'\0')
        digest.update((ROOT/name).read_bytes().replace(b'\r\n',b'\n'))
    return digest.hexdigest()


def policy_encoder_sha256():
    """Model compatibility follows feature semantics, independently of planner."""
    return hashlib.sha256(json.dumps(dict(schema=FEATURE_SCHEMA,
        encoding='adjacent_native_segments_path14_scene23_state8_v1'),
        sort_keys=True,separators=(',',':')).encode()).hexdigest()
