"""Named, hash-checked validation fixtures; never regenerate a road from its seed."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

DIRECTORY = Path(__file__).with_name('validation_roads')
VARIANTS = ('original', 'half_width', 'curved', 'half_width_double')
VALIDATION_KINDS = tuple('validation_uturn_'+variant+'_'+side
                         for variant in VARIANTS for side in ('left', 'right'))


def validation_case(kind):
    if kind not in VALIDATION_KINDS:
        raise ValueError('Unknown fixed validation road: '+str(kind))
    manifest = json.loads((DIRECTORY/'manifest.json').read_text(encoding='utf-8'))
    entry = next(row for row in manifest['cases'] if row['kind']==kind)
    data = (DIRECTORY/entry['file']).read_bytes()
    if hashlib.sha256(data).hexdigest() != entry['sha256']:
        raise ValueError('Fixed validation road fingerprint changed: '+kind)
    case = json.loads(data)
    if case.get('format') != 'fixed_uturn_validation_road_v1' or case.get('kind') != kind:
        raise ValueError('Invalid fixed validation road contract: '+kind)
    return deepcopy(case)
