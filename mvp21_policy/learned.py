"""Frozen path inference and execution-owned RNN state; no training or ROS."""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time
import sys

import numpy as np

from .candidates import from_lattice
from offline.lattice_training.features import (
    FEATURE_VERSION, FEATURE_SCHEMA, INPUT_GROUPS, encode_candidate, scene_features,
)
from offline.lattice_training.source import policy_encoder_sha256
from offline.lattice_training.spatial import (
    PATH_CHANNELS, SCENE_CHANNELS, STATE_CHANNELS, SpatialNetwork, input_matrices,
)


@dataclass
class LearnedPolicySettings:
    enabled: bool = False
    model_path: str = "models/path_policy.json"
    model_sha256: str = "a3116bb7bc4bb46ccb6e208fa3de628eca940820810943409c3157e5b12aa66b"

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("learned_policy.enabled must be bool")
        if (not isinstance(self.model_path, str) or not self.model_path
                or Path(self.model_path).is_absolute() or ".." in Path(self.model_path).parts):
            raise ValueError("learned_policy.model_path must be package relative")
        if (not isinstance(self.model_sha256, str) or len(self.model_sha256) != 64
                or any(c not in "0123456789abcdef" for c in self.model_sha256)):
            raise ValueError("learned_policy.model_sha256 must be a SHA-256 digest")


class FrozenPathPolicy:
    """Inference-only loader using the training encoder/network without copies."""
    def __init__(self, settings, cruise_speed_mps):
        path = (Path(__file__).resolve().parent/settings.model_path).resolve()
        if not path.is_relative_to(Path(__file__).resolve().parent):
            raise ValueError("Policy model must be inside the packaged module tree")
        raw = path.read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        if self.sha256 != settings.model_sha256:
            raise ValueError("Learned policy model checksum mismatch")
        fingerprint = hashlib.sha256()
        for module in (__name__, scene_features.__module__, SpatialNetwork.__module__,
                       policy_encoder_sha256.__module__):
            fingerprint.update(module.encode()+b"\0")
            fingerprint.update(Path(sys.modules[module].__file__).read_bytes().replace(b"\r\n", b"\n"))
        self.inference_source_sha256 = fingerprint.hexdigest()
        record = json.loads(raw)
        if (record.get("format") != "spatial_path_recurrent_policy_v1"
                or record.get("feature_version") != FEATURE_VERSION
                or record.get("feature_schema") != FEATURE_SCHEMA
                or record.get("input_groups") != list(INPUT_GROUPS)
                or record.get("policy_input_schema") != FEATURE_VERSION
                or record.get("policy_encoder_sha256") != policy_encoder_sha256()):
            raise ValueError("Learned policy schema/encoder mismatch")
        self.provenance = record.get("provenance") or {}
        condition = self.provenance.get("execution_condition") or {}
        if (condition.get("reference_policy") != "uniform_acceleration_over_full_path"
                or condition.get("planned_stop") is not False
                or not isinstance(cruise_speed_mps, (float, int))
                or not math.isfinite(cruise_speed_mps)
                or not isinstance(condition.get("cruise_speed_mps"), (float, int))
                or not math.isclose(cruise_speed_mps, condition["cruise_speed_mps"],
                                    abs_tol=1e-9, rel_tol=0)):
            raise ValueError("Learned policy requires its training cruise-speed condition")
        transform = dict(name="sigmoid", temperature=1.0, offset=0.0,
            input="aggregated_candidate_raw_cstar_return", order="after_trial_arithmetic_mean",
            output_space="sigmoid_of_aggregated_cstar_return")
        if (record.get("training_target_transform") != transform
                or record.get("output_activation") != "sigmoid"
                or self.provenance.get("target") != "negative_expected_full_path_cost"
                or self.provenance.get("training_target_transform") != transform):
            raise ValueError("Learned policy score semantics mismatch")
        hidden = record.get("hidden")
        if type(hidden) is not int or not 1 <= hidden <= 256:
            raise ValueError("Invalid learned policy hidden width")
        self.network = SpatialNetwork(hidden, recurrent=True)
        parameters = record.get("parameters") or {}
        if set(parameters) != set(self.network.parameters):
            raise ValueError("Invalid learned policy tensor names")
        for name, expected in self.network.parameters.items():
            value = np.asarray(parameters[name], dtype=float)
            if value.shape != expected.shape or not np.isfinite(value).all():
                raise ValueError("Invalid learned policy tensor")
            value.flags.writeable = False
            self.network.parameters[name] = value
        normalization = record.get("normalization") or {}
        sizes = dict(state=STATE_CHANNELS, path=PATH_CHANNELS, scene=SCENE_CHANNELS)
        if set(normalization) != set(sizes):
            raise ValueError("Invalid learned policy normalization groups")
        self.normalization = {}
        for name, size in sizes.items():
            self.normalization[name] = {}
            for field in ("mean", "scale"):
                value = np.asarray(normalization[name][field], dtype=float)
                if (value.shape != (size,) or not np.isfinite(value).all()
                        or field == "scale" and (value <= 0).any()):
                    raise ValueError("Invalid learned policy normalization")
                value.flags.writeable = False
                self.normalization[name][field] = value

    def step(self, record, hidden):
        matrices = input_matrices(record)
        normalized = {name: (value-self.normalization[name]["mean"])/self.normalization[name]["scale"]
                      for name, value in matrices.items()}
        score, cache = self.network.forward(normalized, hidden)
        if not math.isfinite(score) or not np.isfinite(cache[7]).all():
            raise ValueError("Nonfinite learned policy output")
        return float(score), cache[7].tolist()

    def select(self, result, observation, context):
        started = time.perf_counter()
        native_id = (result.get("selected") or {}).get("id")
        info = dict(backend="learned_policy", model_sha256=self.sha256,
            inference_source_sha256=self.inference_source_sha256,
            native_selected_id=native_id, selected_id=None, scores=[], status="SKIPPED",
            reason="native_result_not_valid", context_revision=context.get("revision"),
            hidden_before=deepcopy(context.get("hidden")))
        result["policy_selection"] = info
        if result.get("status") != "VALID":
            return result
        try:
            if type(context.get("revision")) is not int or context["revision"] < 0:
                raise ValueError("Invalid policy memory revision")
            hidden = np.asarray(context.get("hidden"), dtype=float)
            if hidden.shape != (self.network.hidden,) or not np.isfinite(hidden).all():
                raise ValueError("Invalid policy memory snapshot")
            feasible = [c for c in result.get("candidates", []) if c.get("collision") is None]
            if not feasible:
                raise ValueError("Native VALID result has no admitted candidates")
            scene = scene_features(observation)
            next_states = {}
            for candidate in feasible:
                proposal = dict(reference=dict(frenet_path=dict(type="SAMPLED_FRENET",
                    candidate=from_lattice(candidate, result))))
                record = encode_candidate(observation, proposal, context=scene)
                score, state = self.step(record, hidden)
                next_states[candidate["id"]] = state
                digest = hashlib.sha256(json.dumps(record, sort_keys=True,
                    separators=(",", ":"), allow_nan=False).encode()).hexdigest()
                info["scores"].append(dict(candidate_id=candidate["id"], score=score, input_sha256=digest))
            scores = {row["candidate_id"]: row["score"] for row in info["scores"]}
            selected = max(feasible, key=lambda c: (scores[c["id"]], -int(c["id"])))
            result["selected"] = selected
            info.update(status="SELECTED", reason="highest_frozen_policy_score",
                selected_id=selected["id"], hidden_selected=next_states[selected["id"]],
                changed_from_native=selected["id"] != native_id,
                feasible_count=len(feasible), best_score=scores[selected["id"]])
        except Exception as error:
            result.update(status="POLICY_ERROR", selected=None)
            info.update(status="ERROR", reason=type(error).__name__+": "+str(error))
        info["wall_s"] = time.perf_counter()-started
        return result


class PolicyExecutionMemory:
    """Only a successfully published control action can commit selected memory."""
    def __init__(self, width, model_sha256):
        self.width, self.model_sha256 = width, model_sha256
        self.reset()

    def reset(self):
        self.hidden = [0.0]*self.width
        self.revision = self.commits = 0
        self.committed_job = self.pending = self.last_selection = self.last_rejection = None

    def snapshot(self):
        return dict(revision=self.revision, hidden=list(self.hidden))

    def validate_result(self, result):
        if result.get("status") != "VALID":
            return True
        info = result.get("policy_selection") or {}
        if (info.get("model_sha256") == self.model_sha256
                and type(info.get("context_revision")) is int
                and info["context_revision"] != self.revision):
            self.last_rejection = dict(reason="stale_policy_memory", job_id=result.get("job_id"),
                context_revision=info["context_revision"], current_revision=self.revision)
            return False  # Keep the current admitted path; this job never executed.
        hidden = info.get("hidden_selected")
        if (info.get("status") != "SELECTED" or info.get("model_sha256") != self.model_sha256
                or type(info.get("context_revision")) is not int
                or info.get("context_revision") != self.revision
                or info.get("hidden_before") != self.hidden
                or info.get("selected_id") != (result.get("selected") or {}).get("id")
                or not isinstance(hidden, list) or len(hidden) != self.width
                or not all(type(v) in (int, float) and math.isfinite(v) for v in hidden)):
            result.update(status="POLICY_ERROR", selected=None)
            result.pop("wire_json", None)
            result["policy_selection"] = dict(info, status="ERROR", reason="invalid_policy_execution_payload")
        return True

    def stage(self, result, admitted):
        self.pending = None
        if result.get("status") == "VALID" and admitted:
            self.last_selection = deepcopy(result["policy_selection"])
            self.pending = dict(job_id=result["job_id"], revision=self.revision,
                                hidden=list(self.last_selection["hidden_selected"]))

    def commit(self, job_id):
        pending = self.pending
        if (job_id == self.committed_job or pending is None or pending["job_id"] != job_id
                or pending["revision"] != self.revision):
            return False
        self.hidden = pending["hidden"]
        self.committed_job = job_id
        self.revision += 1
        self.commits += 1
        self.pending = None
        return True

    def diagnostics(self):
        selection = self.last_selection or {}
        return dict(model_sha256=self.model_sha256, memory_revision=self.revision,
            state_commits=self.commits, committed_job_id=self.committed_job,
            selected_candidate_id=selection.get("selected_id"),
            native_selected_id=selection.get("native_selected_id"),
            best_score=selection.get("best_score"), last_rejection=self.last_rejection)
