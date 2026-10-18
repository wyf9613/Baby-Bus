"""Load frozen random-road inputs independently of local collection archives."""
import hashlib
import json
from pathlib import Path

from offline.lattice_training.roads import road_length
CATALOG = Path(__file__).with_name('random_road_catalog.json')


def fingerprint(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def saved_random_roads():
    """Return road definitions and provenance; no recorded trajectories are needed."""
    catalog=json.loads(CATALOG.read_text(encoding='utf-8'))
    cases=catalog['cases']
    if len(cases)!=catalog['physical_variants']:
        raise ValueError('Random-road catalog count differs')
    for case in cases:
        if fingerprint(case['road_spec'])!=case['road_sha256']:
            raise ValueError('Random-road definition fingerprint changed: '+case['fixture_id'])
        if abs(road_length(case['road_spec'])-case['mission_road_length_m'])>1e-8:
            raise ValueError('Random-road mission length differs: '+case['fixture_id'])
    return catalog
