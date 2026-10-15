"""Fingerprint latest planner and public policy candidate adapters; no AST loading."""
import hashlib
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
POLICY_INPUT_SCHEMA='state_road_trajectory_v1'


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
    digest=hashlib.sha256(POLICY_INPUT_SCHEMA.encode())
    for name in ('offline/lattice_training/features.py','offline/lattice_training/reference.py'):
        digest.update(name.encode()+b'\0')
        digest.update((ROOT/name).read_bytes().replace(b'\r\n',b'\n'))
    return digest.hexdigest()
