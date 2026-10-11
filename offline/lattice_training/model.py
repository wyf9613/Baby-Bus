"""NumPy C* candidate scorer with sigmoid targets after trial aggregation."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from .dataset import (load_dataset, objective_hash, write_json, validate_label_source,
                      training_readiness, road_coverage_complete)
from .features import FEATURE_NAMES, FEATURE_VERSION
from .source import planner_source_sha256
from .objective import CSTAR_TARGET, CStarTrackingSettings


RAW_RETURN_SPACE = "raw_cstar_return"
SIGMOID_SCORE_SPACE = "sigmoid_of_aggregated_cstar_return"


def stable_sigmoid(values):
    """Avoid overflow for the deliberately large C* failure penalties."""
    values = np.asarray(values, dtype=np.float64)
    result = np.empty_like(values)
    positive = values >= 0
    result[positive] = 1 / (1 + np.exp(-values[positive]))
    exponential = np.exp(values[~positive])
    result[~positive] = exponential / (1 + exponential)
    return result


class ReturnMLP:
    def __init__(self, hidden=32, seed=0, *, target_transform="sigmoid", target_temperature=1.0):
        if not isinstance(hidden, int) or hidden < 1:
            raise ValueError("hidden must be a positive integer")
        if target_transform not in ("sigmoid", "identity") or (
                not np.isfinite(target_temperature) or target_temperature <= 0):
            raise ValueError("Invalid training target transform")
        # Identity is retained to read the preceding v2 raw-return checkpoints.
        self.target_transform, self.target_temperature = target_transform, float(target_temperature)
        rng = np.random.default_rng(seed)
        count = len(FEATURE_NAMES)
        self.parameters = {"w1": rng.normal(0, 1/np.sqrt(count), (count, hidden)),
                           "b1": np.zeros(hidden), "w2": rng.normal(0, 1/np.sqrt(hidden), hidden),
                           "b2": np.zeros(1)}
        self.x_mean, self.x_scale = np.zeros(count), np.ones(count)
        self.y_mean, self.y_scale = 0.0, 1.0
        self.provenance = {}
        self.planner_source_sha256 = planner_source_sha256()

    @property
    def prediction_space(self):
        return SIGMOID_SCORE_SPACE if self.target_transform == "sigmoid" else RAW_RETURN_SPACE

    @property
    def target_specification(self):
        return {"name": self.target_transform, "temperature": self.target_temperature, "offset": 0.0,
                "input": "aggregated_candidate_raw_cstar_return",
                "order": "after_trial_arithmetic_mean", "output_space": self.prediction_space}

    def training_targets(self, raw_returns):
        """Dataset y already contains the mean of the original trial returns."""
        values = np.asarray(raw_returns, dtype=np.float64)
        if not np.isfinite(values).all():
            raise ValueError("Nonfinite C* training targets")
        return stable_sigmoid(values / self.target_temperature) if self.target_transform == "sigmoid" else values

    def _forward(self, X):
        p = self.parameters
        hidden = np.tanh(X @ p["w1"]+p["b1"])
        output = hidden @ p["w2"]+p["b2"]
        return (stable_sigmoid(output) if self.target_transform == "sigmoid" else output), hidden

    def predict(self, X):
        X = np.asarray(X, dtype=np.float64)
        if X.ndim != 2 or X.shape[1] != len(FEATURE_NAMES) or not np.isfinite(X).all():
            raise ValueError("Invalid scorer feature input")
        scores, _ = self._forward((X-self.x_mean)/self.x_scale)
        result = scores*self.y_scale+self.y_mean
        if not np.isfinite(result).all():
            raise ValueError("Nonfinite learned candidate scores")
        return result

    def save(self, path):
        write_json(path, {"format": "lattice_return_mlp_v3", "feature_version": FEATURE_VERSION,
                         "feature_names": FEATURE_NAMES, "parameters": self.parameters,
                         "x_mean": self.x_mean, "x_scale": self.x_scale,
                         "y_mean": self.y_mean, "y_scale": self.y_scale,
                         "training_target_transform": self.target_specification,
                         "output_activation": "sigmoid" if self.target_transform == "sigmoid" else "linear",
                         "planner_source_sha256": self.planner_source_sha256,
                         "provenance": self.provenance})

    @classmethod
    def load(cls, path, *, check_source=True):
        record = json.loads(Path(path).read_text(encoding="utf-8"))
        if (record.get("format") not in ("lattice_return_mlp_v2", "lattice_return_mlp_v3")
                or record.get("feature_version") != FEATURE_VERSION
                or record.get("feature_names") != list(FEATURE_NAMES)):
            raise ValueError("Unsupported scorer format/schema")
        if check_source and record["planner_source_sha256"] != planner_source_sha256():
            raise ValueError("Scorer was trained against a different planner source revision")
        legacy = record["format"] == "lattice_return_mlp_v2"
        transform = record.get("training_target_transform")
        if not legacy and not isinstance(transform, dict):
            raise ValueError("Checkpoint does not declare its training target transform")
        model = cls(hidden=len(record["parameters"]["b1"]),
                    target_transform="identity" if legacy else transform.get("name"),
                    target_temperature=1.0 if legacy else transform.get("temperature", float("nan")))
        if not legacy and (transform != model.target_specification or record.get("output_activation") != (
                "sigmoid" if model.target_transform == "sigmoid" else "linear")):
            raise ValueError("Checkpoint score semantics are inconsistent")
        model.parameters = {name: np.asarray(record["parameters"][name], dtype=float)
                            for name in ("w1", "b1", "w2", "b2")}
        model.x_mean, model.x_scale = np.asarray(record["x_mean"]), np.asarray(record["x_scale"])
        model.y_mean, model.y_scale = float(record["y_mean"]), float(record["y_scale"])
        model.planner_source_sha256, model.provenance = record["planner_source_sha256"], record.get("provenance", {})
        validate_label_source(model.provenance.get('label_source'))
        CStarTrackingSettings.from_specification(model.provenance.get("objective"))
        if model.provenance.get("target") != CSTAR_TARGET:
            raise ValueError("Checkpoint does not declare the C* policy target")
        if not legacy and model.provenance.get("training_target_transform") != model.target_specification:
            raise ValueError("Checkpoint target transform differs from its training provenance")
        hidden = len(model.parameters["b1"])
        shapes = {"w1": (len(FEATURE_NAMES), hidden), "b1": (hidden,), "w2": (hidden,), "b2": (1,)}
        if any(model.parameters[name].shape != shape or not np.isfinite(model.parameters[name]).all()
               for name, shape in shapes.items()):
            raise ValueError("Invalid model tensor shapes or values")
        if (model.x_mean.shape != (len(FEATURE_NAMES),) or model.x_scale.shape != model.x_mean.shape
                or not np.isfinite(model.x_mean).all() or not np.isfinite(model.x_scale).all()
                or (model.x_scale <= 0).any() or not np.isfinite([model.y_mean, model.y_scale]).all()
                or model.y_scale <= 0):
            raise ValueError("Invalid model normalization")
        if model.target_transform == "sigmoid" and (model.y_mean != 0.0 or model.y_scale != 1.0):
            raise ValueError("Sigmoid scores must not use raw-return output normalization")
        return model

    def fit(self, X, y, groups, validation_X, validation_y, *,
            epochs=150, learning_rate=0.005, pair_weight=0.25, l2=1e-4):
        if epochs < 1 or not np.isfinite([learning_rate, pair_weight, l2]).all() or (
                learning_rate <= 0 or pair_weight < 0 or l2 < 0):
            raise ValueError("Invalid training hyperparameters")
        if len(X) < 2 or not len(validation_X):
            raise ValueError("Nonempty training and validation scenarios are required")
        # Inputs use training rows only. Sigmoid targets need no fitted y scaler.
        self.x_mean, self.x_scale = X.mean(axis=0), X.std(axis=0)
        self.x_scale[self.x_scale < 1e-8] = 1.0
        targets, validation_targets = self.training_targets(y), self.training_targets(validation_y)
        self.y_mean, self.y_scale = (0.0, 1.0) if self.target_transform == "sigmoid" else (
            float(y.mean()), max(float(y.std()), 1e-6))
        normalized_X = (X-self.x_mean)/self.x_scale
        normalized_y = (targets-self.y_mean)/self.y_scale
        weights = np.empty(len(y))
        pairs = []
        for group in np.unique(groups):
            indices = np.flatnonzero(groups == group)
            weights[indices] = 1/len(indices)
            pairs.extend((int(a), int(b)) for offset, a in enumerate(indices) for b in indices[offset+1:])
        weights /= weights.sum()
        pair_indices = np.asarray(pairs, dtype=int).reshape(-1, 2)
        moments = {name: np.zeros_like(value) for name, value in self.parameters.items()}
        variances = deepcopy(moments)
        best, best_loss, history = None, float("inf"), []
        for epoch in range(1, epochs+1):
            predictions, hidden = self._forward(normalized_X)
            error = predictions-normalized_y
            derivative = 2*weights*error
            pair_loss = 0.0
            if len(pair_indices):
                a, b = pair_indices.T
                differences = predictions[a]-predictions[b]-(normalized_y[a]-normalized_y[b])
                pair_loss = float(np.mean(differences**2))
                np.add.at(derivative, a, 2*pair_weight*differences/len(a))
                np.add.at(derivative, b, -2*pair_weight*differences/len(a))
            p = self.parameters
            if self.target_transform == "sigmoid":
                derivative *= predictions * (1 - predictions)
            back = derivative[:, None]*p["w2"][None, :]*(1-hidden**2)
            gradients = {"w1": normalized_X.T @ back+2*l2*p["w1"], "b1": back.sum(axis=0),
                         "w2": hidden.T @ derivative+2*l2*p["w2"],
                         "b2": np.array([derivative.sum()])}
            norm = np.sqrt(sum(float(np.sum(value*value)) for value in gradients.values()))
            for name, gradient in gradients.items():
                gradient *= min(1.0, 10/max(norm, 1e-12))
                moments[name] = 0.9*moments[name]+0.1*gradient
                variances[name] = 0.999*variances[name]+0.001*gradient*gradient
                p[name] -= learning_rate*(moments[name]/(1-0.9**epoch))/(
                    np.sqrt(variances[name]/(1-0.999**epoch))+1e-8)
            train_rmse = float(np.sqrt(np.mean((self.predict(X)-targets)**2)))
            validation_rmse = float(np.sqrt(np.mean((self.predict(validation_X)-validation_targets)**2)))
            history.append({"epoch": epoch, "training_rmse": train_rmse,
                            "validation_rmse": validation_rmse, "pair_loss": pair_loss,
                            "rmse_space": self.prediction_space})
            if validation_rmse < best_loss:
                best_loss, best = validation_rmse, deepcopy(p)
        self.parameters = best
        return history


def selection_metrics(data, mask, prediction, *, metric_targets=None, prediction_space=RAW_RETURN_SPACE):
    """Calibration uses the declared score space; selection always uses raw y."""
    indices = np.flatnonzero(mask)
    if not len(indices):
        return {"rows": 0, "decisions": 0}
    learned, rule, oracle, pair_correct = [], [], [], []
    for group in np.unique(data["groups"][mask]):
        rows = np.flatnonzero(mask & (data["groups"] == group))
        values, predicted = data["y"][rows], prediction[rows]
        learned.append(float(values[int(np.argmax(predicted))]))
        selected = np.flatnonzero(data["rule_selected"][rows])
        rule_index = int(selected[0]) if len(selected) else int(np.argmax(data["rule_scores"][rows]))
        rule.append(float(values[rule_index]))
        oracle.append(float(values.max()))
        for a in range(len(rows)):
            for b in range(a+1, len(rows)):
                if abs(values[a]-values[b]) > 1e-8:
                    pair_correct.append(bool((predicted[a]-predicted[b])*(values[a]-values[b]) > 0))
    targets = data["y"] if metric_targets is None else metric_targets
    rmse = float(np.sqrt(np.mean((prediction[indices]-targets[indices])**2)))
    return {"rows": len(indices), "decisions": len(learned),
            "prediction_rmse": rmse, "prediction_space": prediction_space,
            **({"score_prediction_rmse": rmse} if prediction_space == SIGMOID_SCORE_SPACE else {}),
            "learned_selected_mean_return": float(np.mean(learned)),
            "rule_selected_mean_return": float(np.mean(rule)), "oracle_mean_return": float(np.mean(oracle)),
            "learned_mean_regret": float(np.mean(np.asarray(oracle)-learned)),
            "rule_mean_regret": float(np.mean(np.asarray(oracle)-rule)),
            "pairwise_accuracy": float(np.mean(pair_correct)) if pair_correct else None}


def train_model(dataset, output, *, epochs=150, hidden=32, seed=0, learning_rate=0.005,
                allow_incomplete_coverage=False, target_temperature=1.0):
    data, metadata = load_dataset(dataset)
    if not isinstance(allow_incomplete_coverage, bool):
        raise ValueError("allow_incomplete_coverage must be boolean")
    # Legacy metadata used training_eligible for whole-road coverage. Recompute
    # fit eligibility from validated candidate labels without rewriting the corpus.
    readiness = training_readiness(data)
    if not readiness['eligible']:
        raise ValueError('Training requires at least two Train rows and a nonempty Validation split')
    coverage = metadata.get('collection_coverage')
    train, validation = data["splits"] == "train", data["splits"] == "validation"
    model = ReturnMLP(hidden=hidden, seed=seed, target_temperature=target_temperature)
    history = model.fit(data["X"][train], data["y"][train], data["groups"][train],
                        data["X"][validation], data["y"][validation],
                        epochs=epochs, learning_rate=learning_rate)
    model.planner_source_sha256 = metadata["planner_source_sha256"]
    model.provenance = {"dataset_sha256": metadata["data_sha256"], "seed": seed,
                        "hidden": hidden, "epochs": epochs, "learning_rate": learning_rate,
                        "normalization": "X_training_rows_only_no_y_scaler",
                        "checkpoint": "lowest_validation_score_rmse",
                        "optimizer": "Adam", "activation": "tanh", "pair_weight": 0.25, "l2": 1e-4,
                        "output_activation": "sigmoid", "training_target_transform": model.target_specification,
                        "loss": "decision_weighted_MSE_plus_within_decision_pair_difference_MSE"}
    model.provenance["pipeline_code_sha256"] = metadata["pipeline_code_sha256"]
    model.provenance["objective"] = metadata["objective"]
    model.provenance["label_source"] = metadata["label_source"]
    model.provenance["label_aggregation"] = metadata["label_aggregation"]
    model.provenance["target"] = metadata["label_definition"]
    model.provenance["training_code_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    model.provenance["dataset_training_eligible"] = readiness['eligible']
    model.provenance["dataset_reported_training_eligible"] = metadata.get('training_eligible')
    model.provenance["training_readiness"] = readiness
    model.provenance["road_coverage_complete"] = road_coverage_complete(coverage)
    model.provenance["coverage_override_applied"] = False
    model.provenance["legacy_coverage_override_requested"] = allow_incomplete_coverage
    model.provenance["training_scope"] = "candidate_supervision"
    model.provenance["collection_coverage"] = deepcopy(coverage)
    model.provenance["best_epoch"] = min(history,key=lambda row:row['validation_rmse'])['epoch']
    model.save(output)
    predictions = model.predict(data["X"])
    targets = model.training_targets(data["y"])
    report = {"model": model.provenance, "history": history, "rmse_space": model.prediction_space,
              "splits": {split: selection_metrics(data, data["splits"] == split, predictions,
                         metric_targets=targets, prediction_space=model.prediction_space)
                         for split in ("train", "validation", "test")}}
    write_json(Path(output).with_suffix(".training.json"), report)
    return model, report


def evaluate_policy(model, dataset, output):
    """Return/ranking evaluation on supplied labels, without closed-loop control."""
    data, metadata = load_dataset(dataset)
    if objective_hash(model.provenance.get("objective")) != objective_hash(metadata["objective"]):
        raise ValueError("Model and evaluation dataset use different reward objectives")
    predictions = model.predict(data["X"])
    targets = model.training_targets(data["y"])
    report = {"format": "lattice_policy_evaluation_v1", "model_provenance": model.provenance,
              "dataset_sha256": metadata["data_sha256"], "objective": metadata["objective"],
              "label_source": metadata["label_source"], "prediction_space": model.prediction_space,
              "training_target_transform": model.target_specification,
              "splits": {split: selection_metrics(data, data["splits"] == split, predictions,
                         metric_targets=targets, prediction_space=model.prediction_space)
                         for split in ("train", "validation", "test")}}
    write_json(output, report)
    return report
