"""Combine current native records and freeze a fresh road-group split."""
from collections import defaultdict
import json
from pathlib import Path

import numpy as np

from .dataset import read_decisions, import_dataset, write_json, file_sha256
from .scenarios import SUITE_KINDS


def split_unit(record):
    episode = record.get('episode') or {}
    # Reflections share construction parameters; all families/views for a seed
    # stay together. Old roads keep their existing physical geometry identity.
    if episode.get('kind') in SUITE_KINDS:
        return 'expanded', 'expanded-seed:'+str(episode['road_seed'])
    return 'previous', record['scene_id']


def repartition_records(paths, output, *, seed=17):
    paths = [Path(path).resolve() for path in paths]
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    units, scenes, groups = defaultdict(set), {}, set()
    inputs = {str(path): file_sha256(path) for path in paths}
    for path in paths:
        for record in read_decisions(path):
            if record['group_id'] in groups:
                raise ValueError('Repeated decision: '+record['group_id'])
            groups.add(record['group_id'])
            stratum, unit = split_unit(record)
            units[stratum].add(unit)
            if scenes.setdefault(record['scene_id'], unit) != unit:
                raise ValueError('Physical road has inconsistent split group')
    assignments = {}
    rng = np.random.default_rng(seed)
    for stratum, items in sorted(units.items()):
        items = sorted(items)
        if len(items) < 3:
            raise ValueError('Each source stratum needs at least three independent road groups')
        items = [items[i] for i in rng.permutation(len(items))]
        training = max(1, min(len(items)-2, int(.6*len(items))))
        validation = max(1, (len(items)-training)//2)
        for index, item in enumerate(items):
            assignments[item] = 'train' if index < training else 'validation' if index < training+validation else 'test'
    output.mkdir(parents=True)
    manifest = dict(seed=seed, method='seeded_source_stratified_independent_road_groups_60_20_20',
        grouping='same physical road, both centerline views, mirrored suite families and same construction seed stay together',
        assignments=assignments, scene_groups=scenes, input_sha256=inputs,
        prior_holdouts_retired=True, labels_recomputed=False,
        source_observations_and_candidates_preserved=True)
    write_json(output/'partition.json', manifest)
    with (output/'decisions.jsonl').open('w', encoding='utf-8') as stream:
        for path in paths:
            for record in read_decisions(path):
                _, unit = split_unit(record)
                record['partition_origin'] = dict(split=record['split'], input_file=str(path),
                                                  split_group_id=unit)
                record['split'] = assignments[unit]
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
    metadata = import_dataset(output/'decisions.jsonl', output/'dataset', allow_collection_variation=True)
    metadata['repartition'] = manifest
    write_json(output/'dataset/metadata.json', metadata)
    for path in paths:
        if file_sha256(path) != inputs[str(path)]:
            raise ValueError('Source records changed during repartition')
    return metadata
