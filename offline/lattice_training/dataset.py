"""C* decision import, trace-derived labels and dataset persistence."""
import hashlib
import json
import math
from pathlib import Path
import time
from copy import deepcopy

import numpy as np

from .features import FEATURE_NAMES, FEATURE_VERSION, INPUT_GROUPS, encode_candidate, scene_features
from .source import planner_source_sha256,policy_encoder_sha256,POLICY_INPUT_SCHEMA
from .objective import CSTAR_TARGET, CStarTrackingSettings, cstar_trial_returns


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None  # raw LiDAR no-return encoding in the optional observation log
    return value


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(json_safe(value), ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


DECISION_FORMAT = "state_road_trajectory_decision_v1"
DATASET_FORMAT = "state_road_trajectory_dataset_v1"
SPLITS = {"train", "validation", "test"}
INTERNAL_LABEL_VERSION = "6"


def file_sha256(path):
    """Hash large trace files without loading the whole file into memory."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def label_source_contract(source):
    """Road seeds/split scope vary by batch; solver, plant and sampling must match."""
    result = deepcopy(json_safe(source))
    if isinstance(result, dict) and isinstance(result.get('collection_settings'), dict):
        for key in ('train_seeds', 'validation_seeds', 'test_seeds', 'train_only'):
            result['collection_settings'].pop(key, None)
    return result


def training_readiness(data):
    """Called after label/schema/hash validation; road coverage is diagnostic."""
    counts = {split: int(np.sum(data['splits'] == split)) for split in sorted(SPLITS)}
    missing = [split for split in ('train', 'validation') if counts[split] < (2 if split == 'train' else 1)]
    return {'eligible': not missing, 'basis': 'validated_candidate_labels_and_nonempty_fit_splits',
            'split_rows': counts, 'insufficient_splits': missing,
            'minimum_training_rows': 2, 'requires_complete_road_rollout': False}


def road_coverage_complete(coverage):
    return (isinstance(coverage, dict) and bool(coverage.get('roads'))
            and not any(coverage.get(key) for key in
                        ('missing_label_roads', 'missing_goal_roads', 'illegal_initial_episodes')))


def validate_label_source(source):
    if (isinstance(source,dict) and source.get('name')=='internal_cstar_native_env'
            and source.get('version')!=INTERNAL_LABEL_VERSION):
        raise ValueError("Legacy internal C* labels omit finite-road body constraints; recollect with label source v6")


def objective_hash(objective):
    return hashlib.sha256(json.dumps(objective, sort_keys=True, ensure_ascii=False,
                                    allow_nan=False, separators=(",", ":")).encode("utf-8")).hexdigest()


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def validate_objective(objective):
    CStarTrackingSettings.from_specification(objective)
    return objective_hash(objective)


def _reject_constant(value):
    raise ValueError(f"Nonfinite JSON value: {value}; encode unavailable sensor values as null")


def read_decisions(path):
    """Validate candidate records; producer identity is recorded, not a gate."""
    with Path(path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line, parse_constant=_reject_constant)
                if not isinstance(record, dict) or record.get("format") != DECISION_FORMAT:
                    raise ValueError(f"Expected {DECISION_FORMAT}")
                producer=record.get('planner_source_sha256')
                if not isinstance(producer,str) or len(producer)!=64 or any(c not in '0123456789abcdef' for c in producer):
                    raise ValueError('Decision requires a producer planner source fingerprint')
                if not isinstance(record.get("group_id"), str) or not record["group_id"]:
                    raise ValueError("A nonempty decision group_id is required")
                if not isinstance(record.get("observation"), dict):
                    raise ValueError("An observable perception/estimation record is required")
                choices = record.get("candidates")
                if not isinstance(choices, list) or not choices:
                    raise ValueError("At least one already-filtered native candidate is required")
                identities = []
                for entry in choices:
                    proposal = entry["candidate"]
                    identity = proposal.get("candidate_id")
                    if not isinstance(identity, str) or not identity:
                        raise ValueError("Candidate IDs must be nonempty strings")
                    if proposal["reference"].get("valid") is not True:
                        raise ValueError("Only candidates admitted by native hard filters may be scored")
                    identities.append(identity)
                if len(identities) != len(set(identities)):
                    raise ValueError("Candidate IDs repeat within a decision")
            except (ValueError, TypeError, KeyError) as error:
                raise ValueError(f"Decision line {line_number}: {error}") from error
            yield record


def _label_mean(label):
    """Check an optional return assertion against the recomputed C* trial labels."""
    if not isinstance(label, dict) or ("return" in label) == ("returns" in label):
        raise ValueError("Provide exactly one of label.return or label.returns")
    values = [label["return"]] if "return" in label else label["returns"]
    if not isinstance(values, list) or not values:
        raise ValueError("label.returns must contain at least one trial")
    return float(np.mean([_finite_number(value, "label return") for value in values])), len(values)


def import_dataset(records_path, output, *, required_splits=None):
    """Encode actual C* traces, enforce splits and preserve producer provenance."""
    started = time.perf_counter()
    required_splits = SPLITS if required_splits is None else set(required_splits)
    if not required_splits or not required_splits <= SPLITS:
        raise ValueError('Invalid required dataset splits')
    records_sha256 = file_sha256(records_path)
    rows, scene_splits, groups, contract, sources = [], {}, set(), None, {}
    producers=set()
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    partial_records = output / 'decisions.jsonl.part'
    def encoded_records():
        with partial_records.open('w', encoding='utf-8') as stream:
            for record in read_decisions(records_path):
                yield record
                # Encode one validated decision at a time; traces can exceed 1GB.
                visible = {key: record['observation'][key] for key in
                           ('timestamp_s', 'dt_s', 'frame_id', 'estimation', 'sensor_available', 'sensor_age_s')
                           if key in record['observation']}
                stream.write(json.dumps(json_safe({**record, 'observation': visible}),
                                        ensure_ascii=False, allow_nan=False) + '\n')
    current_source = planner_source_sha256()
    for record in encoded_records():
        producers.add(record['planner_source_sha256'])
        objective = record.get("objective")
        fingerprint = validate_objective(objective)
        source = record.get("label_source")
        if not isinstance(source, dict) or not source.get("name") or not source.get("version"):
            raise ValueError("Label source needs a name and version")
        validate_label_source(source)
        if source.get("objective_sha256") != fingerprint:
            raise ValueError("Label-source objective fingerprint differs from the training objective")
        if source.get("execution_assumption") != "cstar":
            raise ValueError("C* labels require execution_assumption=cstar")
        identity = (fingerprint, objective_hash(label_source_contract(source)))
        if contract is not None and identity != contract:
            raise ValueError("A dataset cannot mix objective definitions or label-source versions")
        contract = identity
        sources.setdefault(objective_hash(source), deepcopy(source))
        scene, split, group = record.get("scene_id"), record.get("split"), record["group_id"]
        if not isinstance(scene, str) or not scene or split not in SPLITS:
            raise ValueError("Each decision needs a scene_id and train/validation/test split")
        if group in groups:
            raise ValueError("Decision group_id repeats; combine random trials in cstar_trials")
        groups.add(group)
        if scene_splits.setdefault(scene, split) != split:
            raise ValueError("The same base scene appears in different splits")
        selected = record.get("rule_selected_candidate_id")
        if selected is not None and selected not in [entry["candidate"]["candidate_id"]
                                                    for entry in record["candidates"]]:
            raise ValueError("Rule-selected candidate is absent from the admitted candidate set")
        context = scene_features(record["observation"])
        shared_initial = None
        for entry in record["candidates"]:
            candidate = entry["candidate"]
            computed, initial = cstar_trial_returns(candidate, entry.get("cstar_trials"), objective)
            if shared_initial is None:
                shared_initial = initial
            elif initial.keys() != shared_initial.keys() or any(
                    not np.allclose(initial[key][index], shared_initial[key][index], atol=1e-8, rtol=0)
                    for key in initial for index in (0, 1)):
                raise ValueError("All candidates must share C* trial IDs and initial vehicle/action states")
            if entry.get("label") is not None:
                supplied_mean, _ = _label_mean(entry["label"])
                expected_mean, _ = _label_mean(computed)
                if not math.isclose(supplied_mean, expected_mean, abs_tol=1e-8, rel_tol=1e-8):
                    raise ValueError("Supplied C* return differs from negative tracking loss")
            entry["label"] = computed
            value, trials = _label_mean(entry.get("label"))
            rows.append({"X": encode_candidate(record["observation"], candidate, context=context),
                         "y": value, "group": group, "scenario": scene, "split": split,
                         "candidate": candidate["candidate_id"], "trials": trials,
                         "rule_score": _finite_number(candidate["descriptor"]["score"], "rule score"),
                         "rule_selected": candidate["candidate_id"] == selected})
    if not rows or set(scene_splits.values()) != required_splits:
        raise ValueError('Data must provide exactly these nonempty splits: ' + ', '.join(sorted(required_splits)))
    np.savez_compressed(output / "data.npz", X=np.stack([row["X"] for row in rows]),
        y=np.asarray([row["y"] for row in rows]), groups=np.asarray([row["group"] for row in rows]),
        scenarios=np.asarray([row["scenario"] for row in rows]), splits=np.asarray([row["split"] for row in rows]),
        candidate_ids=np.asarray([row["candidate"] for row in rows]),
        rule_scores=np.asarray([row["rule_score"] for row in rows]),
        rule_selected=np.asarray([row["rule_selected"] for row in rows], dtype=bool),
        label_trials=np.asarray([row["trials"] for row in rows], dtype=int))
    partial_records.replace(output / 'decisions.jsonl')
    readiness = training_readiness({'splits': np.asarray([row['split'] for row in rows])})
    metadata = {"format": DATASET_FORMAT, "feature_version": FEATURE_VERSION,
                "feature_names": FEATURE_NAMES, "rows": len(rows), "decisions": len(groups),
                "split_rows": {split: sum(row["split"] == split for row in rows) for split in sorted(SPLITS)},
                "objective": objective, "objective_sha256": contract[0],
                "label_source": source if len(sources) == 1 else label_source_contract(source),
                "label_sources": list(sources.values()),
                "training_readiness": readiness, "training_eligible": readiness['eligible'],
                "label_aggregation": "negative_arithmetic_mean_of_cstar_tracking_losses",
                "label_definition": CSTAR_TARGET, "planner_source_sha256": current_source,
                "producer_planner_sha256": sorted(producers),
                "policy_input_schema": POLICY_INPUT_SCHEMA,"policy_encoder_sha256":policy_encoder_sha256(),
                "input_groups": INPUT_GROUPS,
                "records_sha256": records_sha256,
                "encoded_records_sha256": file_sha256(output / 'decisions.jsonl'),
                "pipeline_code_sha256": {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                                        for path in sorted(Path(__file__).parent.glob("*.py"))},
                "data_sha256": hashlib.sha256((output / "data.npz").read_bytes()).hexdigest(),
                "scene_splits": scene_splits, "elapsed_wall_s": time.perf_counter()-started}
    write_json(output / "metadata.json", metadata)
    return metadata


def append_training_dataset(base, addition, output):
    """Append already-validated train rows and keep both holdouts exactly intact."""
    started = time.perf_counter()
    base, addition, output = Path(base).resolve(), Path(addition).resolve(), Path(output).resolve()
    if (output == base or output == addition or output in base.parents or output in addition.parents
            or base in output.parents or addition in output.parents):
        raise ValueError('Merged output must be separate from both input datasets')
    if output.exists():
        raise FileExistsError(f'Merged dataset already exists: {output}')
    old, old_metadata = load_dataset(base)
    new, new_metadata = load_dataset(addition)
    if set(old['splits']) != SPLITS or set(new['splits']) != {'train'}:
        raise ValueError('Append requires an existing three-split dataset and train-only additions')
    if old.keys() != new.keys():
        raise ValueError('Dataset array schemas differ')
    if (old_metadata['objective'] != new_metadata['objective'] or
            label_source_contract(old_metadata['label_source']) != label_source_contract(new_metadata['label_source'])):
        raise ValueError('Append would mix different C* objectives, solvers, plants or sampling contracts')
    if set(old['groups']) & set(new['groups']) or set(old_metadata['scene_splits']) & set(new_metadata['scene_splits']):
        raise ValueError('Added decision groups or roads must be new, including relative to holdouts')
    old_coverage, new_coverage = old_metadata.get('collection_coverage'), new_metadata.get('collection_coverage')
    if not isinstance(old_coverage, dict) or not isinstance(new_coverage, dict):
        raise ValueError('Append requires collection coverage from both batches')
    if set(r['scene_id'] for r in old_coverage['roads']) & set(r['scene_id'] for r in new_coverage['roads']):
        raise ValueError('Added generated roads overlap an earlier collection')
    merged = {key: np.concatenate((old[key], new[key]), axis=0) for key in old}
    holdout_checks = {}
    for split in ('validation', 'test'):
        old_mask, merged_mask = old['splits'] == split, merged['splits'] == split
        holdout_checks[split] = all(np.array_equal(old[key][old_mask], merged[key][merged_mask]) for key in old)
    if not all(holdout_checks.values()):
        raise ValueError('Append changed a holdout array')
    coverage = deepcopy(old_coverage)
    coverage['roads'] += deepcopy(new_coverage['roads'])
    for key in ('missing_label_roads', 'missing_goal_roads', 'illegal_initial_episodes'):
        coverage[key] = list(dict.fromkeys([*old_coverage.get(key, []), *new_coverage.get(key, [])]))
    coverage['require_road_end'] = old_coverage.get('require_road_end', False) or new_coverage.get('require_road_end', False)
    coverage.pop('training_eligible', None)  # Legacy field conflated coverage with sample validity.
    coverage['coverage_complete'] = road_coverage_complete(coverage)
    coverage['empty_splits'] = []
    readiness = training_readiness(merged)
    output.mkdir(parents=True)
    records_digest = hashlib.sha256()
    encoded_hashes = []
    with (output / 'decisions.jsonl').open('wb') as destination:
        for directory, metadata in ((base, old_metadata), (addition, new_metadata)):
            digest = hashlib.sha256()
            with (directory / 'decisions.jsonl').open('rb') as source:
                for block in iter(lambda: source.read(8 * 1024 * 1024), b''):
                    digest.update(block)
                    records_digest.update(block)
                    destination.write(block)
            fingerprint = digest.hexdigest()
            if metadata.get('encoded_records_sha256', fingerprint) != fingerprint:
                raise ValueError('Encoded input records hash mismatch')
            encoded_hashes.append(fingerprint)
    np.savez_compressed(output / 'data.npz', **merged)
    sources = {}
    for metadata in (old_metadata, new_metadata):
        for source in metadata.get('label_sources', [metadata['label_source']]):
            sources.setdefault(objective_hash(source), deepcopy(source))
    metadata = deepcopy(old_metadata)
    metadata.update(rows=len(merged['y']), decisions=len(np.unique(merged['groups'])),
                    producer_planner_sha256=sorted(set(old_metadata['producer_planner_sha256']) |
                                                  set(new_metadata['producer_planner_sha256'])),
                    split_rows={split: int(np.sum(merged['splits'] == split)) for split in sorted(SPLITS)},
                    scene_splits={**old_metadata['scene_splits'], **new_metadata['scene_splits']},
                    label_source=label_source_contract(old_metadata['label_source']),
                    label_sources=list(sources.values()), collection_coverage=coverage,
                    training_eligible=readiness['eligible'], training_readiness=readiness,
                    road_coverage_complete=coverage['coverage_complete'],
                    records_sha256=records_digest.hexdigest(), encoded_records_sha256=records_digest.hexdigest(),
                    data_sha256=file_sha256(output / 'data.npz'), elapsed_wall_s=time.perf_counter()-started,
                    pipeline_code_sha256={path.name: file_sha256(path) for path in sorted(Path(__file__).parent.glob('*.py'))})
    metadata['augmentation'] = {'operation': 'append_new_roads_to_train_only',
                                'base_dataset': str(base), 'addition_dataset': str(addition),
                                'base_data_sha256': old_metadata['data_sha256'],
                                'addition_data_sha256': new_metadata['data_sha256'],
                                'input_encoded_records_sha256': encoded_hashes,
                                'base_rows': len(old['y']), 'added_train_rows': len(new['y']),
                                'added_train_decisions': len(np.unique(new['groups'])),
                                'holdout_arrays_unchanged': holdout_checks,
                                'input_pipeline_code_sha256': [old_metadata['pipeline_code_sha256'],
                                                              new_metadata['pipeline_code_sha256']]}
    write_json(output / 'metadata.json', metadata)
    load_dataset(output)
    return metadata


def score_decisions(model, records_path, output):
    """Offline inference from admitted candidates; no plant/controller is loaded."""
    results = []
    seen = set()
    for record in read_decisions(records_path):
        if record["group_id"] in seen:
            raise ValueError("Decision group_id repeats")
        seen.add(record["group_id"])
        if record.get("objective") is not None and validate_objective(record["objective"]) != objective_hash(
                model.provenance.get("objective")):
            raise ValueError("Scoring record and policy use different objectives")
        context = scene_features(record["observation"])
        candidates = [entry["candidate"] for entry in record["candidates"]]
        scores = model.predict(np.stack([encode_candidate(record["observation"], candidate, context=context)
                                        for candidate in candidates]))
        index = int(np.argmax(scores))
        results.append({"group_id": record["group_id"],
                        "selected_candidate_id": candidates[index]["candidate_id"],
                        "prediction_space": model.prediction_space,
                        "scores": [{"candidate_id": candidate["candidate_id"],
                                    "predicted_score": float(score)}
                                   for candidate, score in zip(candidates, scores)]})
    if not results:
        raise ValueError("No decisions to score")
    write_json(output, {"format": "lattice_policy_scores_v2", "model_provenance": model.provenance,
                        "prediction_space": model.prediction_space,
                        "decisions": results})
    return results

def load_dataset(directory):
    directory = Path(directory)
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    validate_label_source(metadata.get('label_source'))
    if (metadata.get('policy_input_schema')!=POLICY_INPUT_SCHEMA or metadata.get('policy_encoder_sha256')!=policy_encoder_sha256()
            or metadata.get('input_groups')!={k:list(v) for k,v in INPUT_GROUPS.items()}):
        raise ValueError('Dataset state/road/trajectory encoder/schema differs; collect current-format data')
    fingerprint = validate_objective(metadata.get("objective"))
    if metadata.get("objective_sha256", fingerprint) != fingerprint:
        raise ValueError("Dataset objective fingerprint mismatch")
    if (metadata.get("format") != DATASET_FORMAT or metadata.get("label_definition") != CSTAR_TARGET
            or metadata.get("label_aggregation") != "negative_arithmetic_mean_of_cstar_tracking_losses"
            or not isinstance(metadata.get("label_source"), dict)
            or metadata["label_source"].get("execution_assumption") != "cstar"
            or metadata["label_source"].get("objective_sha256") != fingerprint):
        raise ValueError("Dataset must use the single C* execution/target contract")
    if metadata.get("feature_version") != FEATURE_VERSION or metadata.get("feature_names") != list(FEATURE_NAMES):
        raise ValueError("Dataset feature schema differs from the current scorer")
    if hashlib.sha256((directory / "data.npz").read_bytes()).hexdigest() != metadata["data_sha256"]:
        raise ValueError("Dataset content hash mismatch")
    with np.load(directory / "data.npz", allow_pickle=False) as archive:
        data = {name: archive[name].copy() for name in archive.files}
    required = {"X", "y", "groups", "scenarios", "splits", "candidate_ids", "rule_scores", "rule_selected"}
    if not required <= data.keys() or any(data[name].ndim != 1 or len(data[name]) != len(data["y"])
                                         for name in required - {"X"}):
        raise ValueError("Dataset row arrays are missing or inconsistent")
    if data["X"].shape != (len(data["y"]), len(FEATURE_NAMES)) or not (
            np.isfinite(data["X"]).all() and np.isfinite(data["y"]).all()):
        raise ValueError("Dataset shape/nonfinite values are invalid")
    if not np.isfinite(data["rule_scores"]).all() or not set(data["splits"]) <= {"train", "validation", "test"}:
        raise ValueError("Dataset rule scores or split labels are invalid")
    for field in ("groups", "scenarios"):
        for identity in np.unique(data[field]):
            if len(set(data["splits"][data[field] == identity])) != 1:
                raise ValueError("Dataset leaks decisions/scenes across splits")
    return data, metadata
