"""NumPy C* candidate scorer with sigmoid targets after trial aggregation."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from .dataset import (load_dataset, objective_hash, write_json, validate_label_source,
                       training_readiness, road_coverage_complete, validate_execution_condition,read_decisions)
from .features import FEATURE_SCHEMA, FEATURE_VERSION, INPUT_GROUPS
from .spatial import SpatialNetwork, input_matrices, STATE_CHANNELS, PATH_CHANNELS, SCENE_CHANNELS
from .source import planner_source_sha256,policy_encoder_sha256,POLICY_INPUT_SCHEMA
from .objective import CSTAR_TARGET, CStarTrackingSettings
from .sequence import SEQUENCE_FIELDS,sequence_layout,forward_sequence,backward_sequence


RAW_RETURN_SPACE = "raw_cstar_return"
SIGMOID_SCORE_SPACE = "sigmoid_of_aggregated_cstar_return"
CHECKPOINT_CRITERIA=('validation_rmse','validation_feasible_then_return')


def decision_checkpoint_metrics(groups,returns,predictions,feasible):
    """Validation labels affect checkpoint choice only, never the policy input."""
    groups,returns,predictions,feasible=map(np.asarray,(groups,returns,predictions,feasible))
    if any(v.shape!=returns.shape for v in (groups,predictions,feasible)) or returns.ndim!=1:
        raise ValueError('Invalid checkpoint decision arrays')
    if feasible.dtype!=bool or not len(returns) or not np.isfinite(returns).all() or not np.isfinite(predictions).all():
        raise ValueError('Invalid checkpoint feasibility or scores')
    failures=available=0;regrets=[]
    for group in np.unique(groups):
        rows=np.flatnonzero(groups==group);selected=rows[int(np.argmax(predictions[rows]))]
        if feasible[rows].any():
            available+=1;failures+=not feasible[selected]
        regrets.append(float(np.max(returns[rows])-returns[selected]))
    return dict(validation_avoidable_failures=int(failures),validation_decisions_with_feasible=available,
                validation_mean_return_regret=float(np.mean(regrets)))


def checkpoint_key(row,criterion):
    if criterion=='validation_rmse':return (row['validation_rmse'],)
    if criterion=='validation_feasible_then_return':
        return (row['validation_avoidable_failures'],row['validation_mean_return_regret'],row['validation_rmse'])
    raise ValueError('Unknown checkpoint criterion')


def stable_sigmoid(values):
    """Avoid overflow for the deliberately large C* failure penalties."""
    values = np.asarray(values, dtype=np.float64)
    result = np.empty_like(values)
    positive = values >= 0
    result[positive] = 1 / (1 + np.exp(-values[positive]))
    exponential = np.exp(values[~positive])
    result[~positive] = exponential / (1 + exponential)
    return result


class SpatialPathPolicy:
    """Learned segment/scene encoders; one untimed candidate in each prediction."""
    def __init__(self, hidden=32, seed=0, *, target_transform="sigmoid", target_temperature=1.0,recurrent=False):
        if not isinstance(hidden, int) or isinstance(hidden, bool) or hidden < 1:
            raise ValueError("hidden must be a positive integer")
        if target_transform != "sigmoid" or (
                not np.isfinite(target_temperature) or target_temperature <= 0):
            raise ValueError("Invalid training target transform")
        self.target_transform, self.target_temperature = target_transform, float(target_temperature)
        self.network = SpatialNetwork(hidden, seed,recurrent=recurrent)
        self.normalization = {name: dict(mean=np.zeros(count), scale=np.ones(count))
            for name, count in (('state', STATE_CHANNELS), ('path', PATH_CHANNELS), ('scene', SCENE_CHANNELS))}
        self.provenance = {}
        self.planner_source_sha256 = planner_source_sha256()

    @property
    def parameters(self):
        return self.network.parameters

    @parameters.setter
    def parameters(self, values):
        self.network.parameters = values

    @property
    def prediction_space(self):
        return SIGMOID_SCORE_SPACE

    @property
    def target_specification(self):
        return {"name": self.target_transform, "temperature": self.target_temperature, "offset": 0.0,
                "input": "aggregated_candidate_raw_cstar_return",
                "order": "after_trial_arithmetic_mean", "output_space": self.prediction_space}

    def training_targets(self, raw_returns):
        values = np.asarray(raw_returns, dtype=np.float64)
        if not np.isfinite(values).all():
            raise ValueError("Nonfinite C* training targets")
        return stable_sigmoid(values / self.target_temperature)

    def _normalize(self, matrices):
        return {name: (values-self.normalization[name]['mean'])/self.normalization[name]['scale']
                for name, values in matrices.items()}

    def score(self, record):
        """The production scoring entry accepts exactly one structured path."""
        value, _ = self.network.forward(self._normalize(input_matrices(record)))
        if not np.isfinite(value):
            raise ValueError("Nonfinite learned path score")
        return float(value)

    def initial_state(self):return np.zeros(self.network.hidden)

    def step(self,record,previous_hidden=None):
        value,cache=self.network.forward(self._normalize(input_matrices(record)),previous_hidden)
        return float(value),cache[7].copy()

    def predict_dataset(self,data):
        if not self.network.recurrent:return self.predict(data['X'])
        layout=sequence_layout(data['groups'],data)
        outputs=forward_sequence(self.network,[self._normalize(input_matrices(x)) for x in data['X']],layout)
        return np.asarray([value for value,_ in outputs])

    def predict(self, records):
        """Independent scoring loop for training/evaluation, with no cross-row attention."""
        if isinstance(records, dict):
            raise ValueError("Use score() for one path or predict() for independent records")
        return np.asarray([self.score(record) for record in records], dtype=float)

    def save(self, path):
        write_json(path, dict(format="spatial_path_recurrent_policy_v1" if self.network.recurrent else "spatial_path_attention_policy_v1", feature_version=FEATURE_VERSION,
            input_groups=INPUT_GROUPS, feature_schema=FEATURE_SCHEMA, hidden=self.network.hidden,
            parameters=self.parameters, normalization=self.normalization,
            training_target_transform=self.target_specification, output_activation="sigmoid",
            planner_source_sha256=self.planner_source_sha256, policy_input_schema=POLICY_INPUT_SCHEMA,
            policy_encoder_sha256=policy_encoder_sha256(), provenance=self.provenance))

    @classmethod
    def load(cls, path):
        record = json.loads(Path(path).read_text(encoding="utf-8"))
        if (record.get("format") not in ("spatial_path_attention_policy_v1","spatial_path_recurrent_policy_v1")
                or record.get("feature_version") != FEATURE_VERSION
                or record.get("feature_schema") != FEATURE_SCHEMA):
            raise ValueError("Unsupported scorer format/schema")
        # The frozen v1 initialization has identical segment input semantics.
        # Its former fingerprint included the entire training implementation.
        old_hash='7c5bc9c001b4eb5b310d7618d0be66a3b01d7f87b66b69299c97dedd3f08ebd3'
        accepted={policy_encoder_sha256()}
        if record['format']=='spatial_path_attention_policy_v1':accepted.add(old_hash)
        if (record.get('policy_input_schema') != POLICY_INPUT_SCHEMA
                or record.get('policy_encoder_sha256') not in accepted
                or record.get('input_groups') != list(INPUT_GROUPS)):
            raise ValueError('Scorer spatial-path input or encoder semantics differ')
        transform = record.get("training_target_transform") or {}
        model = cls(hidden=record["hidden"], target_transform=transform.get("name"),
                    target_temperature=transform.get("temperature", float("nan")),
                    recurrent=record['format']=='spatial_path_recurrent_policy_v1')
        if transform != model.target_specification or record.get("output_activation") != "sigmoid":
            raise ValueError("Checkpoint score semantics are inconsistent")
        parameters = record.get('parameters') or {}
        if set(parameters) != set(model.parameters):
            raise ValueError("Invalid model parameter names")
        for name, expected in model.parameters.items():
            values = np.asarray(parameters[name], dtype=float)
            if values.shape != expected.shape or not np.isfinite(values).all():
                raise ValueError("Invalid model tensor shapes or values")
            model.parameters[name] = values
        normalization = record.get('normalization') or {}
        if set(normalization) != set(model.normalization):
            raise ValueError("Invalid model normalization groups")
        for name, expected in model.normalization.items():
            for field in ('mean', 'scale'):
                values = np.asarray(normalization[name][field], dtype=float)
                if values.shape != expected[field].shape or not np.isfinite(values).all():
                    raise ValueError("Invalid model normalization")
                if field == 'scale' and (values <= 0).any():
                    raise ValueError("Invalid model normalization")
                model.normalization[name][field] = values
        model.planner_source_sha256 = record['planner_source_sha256']
        model.provenance = record.get("provenance", {})
        validate_execution_condition(model.provenance.get('execution_condition'))
        validate_label_source(model.provenance.get('label_source'))
        CStarTrackingSettings.from_specification(model.provenance.get("objective"))
        if model.provenance.get("target") != CSTAR_TARGET:
            raise ValueError("Checkpoint does not declare the C* policy target")
        if model.provenance.get("training_target_transform") != model.target_specification:
            raise ValueError("Checkpoint target transform differs from its training provenance")
        return model

    def fit(self, X, y, groups, validation_X, validation_y, *,
            epochs=150, learning_rate=0.005, pair_weight=0.25, l2=1e-4,
            sequence_context=None,validation_context=None,validation_groups=None,refit_normalization=True,progress=None,
            checkpoint='validation_rmse',validation_feasible=None,early_stopping_patience=None):
        if epochs < 1 or not np.isfinite([learning_rate, pair_weight, l2]).all() or (
                learning_rate <= 0 or pair_weight < 0 or l2 < 0):
            raise ValueError("Invalid training hyperparameters")
        if len(X) < 2 or not len(validation_X):
            raise ValueError("Nonempty training and validation scenarios are required")
        if checkpoint not in CHECKPOINT_CRITERIA:raise ValueError('Unknown checkpoint criterion')
        if early_stopping_patience is not None and (type(early_stopping_patience) is not int or early_stopping_patience<1):
            raise ValueError('Early stopping patience must be a positive integer')
        if checkpoint=='validation_feasible_then_return' and (validation_groups is None or validation_feasible is None):
            raise ValueError('Feasibility checkpoint selection requires Validation decision labels')
        training = [input_matrices(record) for record in X]
        validation = [input_matrices(record) for record in validation_X]
        # Per-channel normalization is fitted on Train only; no curve is resampled.
        for name, values in self.normalization.items():
            if not refit_normalization:continue
            rows = [item[name].reshape(-1, len(values['mean'])) for item in training if item[name].size]
            if rows:
                joined = np.concatenate(rows)
                values['mean'], values['scale'] = joined.mean(axis=0), joined.std(axis=0)
                values['scale'][values['scale'] < 1e-8] = 1.
        training = [self._normalize(item) for item in training]
        validation = [self._normalize(item) for item in validation]
        layout=sequence_layout(groups,sequence_context) if self.network.recurrent else None
        validation_layout=sequence_layout(validation_groups,validation_context) if self.network.recurrent else None
        def forward(items,order):
            return forward_sequence(self.network,items,order) if self.network.recurrent else [self.network.forward(item) for item in items]
        targets, validation_targets = self.training_targets(y), self.training_targets(validation_y)
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
        best,best_key,history=None,None,[]
        best_rank=None;last_rank_improvement=0
        self.best_epoch=None;self.training_stop_reason='epoch_budget'
        def measure(epoch,pair_loss):
            train_prediction=np.asarray([value for value,_ in forward(training,layout)])
            validation_prediction=np.asarray([value for value,_ in forward(validation,validation_layout)])
            row=dict(epoch=epoch,training_rmse=float(np.sqrt(np.mean((train_prediction-targets)**2))),
                validation_rmse=float(np.sqrt(np.mean((validation_prediction-validation_targets)**2))),
                pair_loss=pair_loss,rmse_space=self.prediction_space)
            if validation_feasible is not None:
                row.update(decision_checkpoint_metrics(validation_groups,validation_y,validation_prediction,validation_feasible))
            return row
        if checkpoint=='validation_feasible_then_return':
            baseline=measure(0,None);history.append(baseline)
            best_key=checkpoint_key(baseline,checkpoint);best=deepcopy(self.parameters);self.best_epoch=0
            best_rank=best_key[:2]
            if progress:progress(baseline)
        for epoch in range(1, epochs+1):
            outputs = forward(training,layout)
            predictions = np.asarray([value for value, _ in outputs])
            derivative = 2*weights*(predictions-targets)
            pair_loss = 0.
            if len(pair_indices):
                a, b = pair_indices.T
                differences = predictions[a]-predictions[b]-(targets[a]-targets[b])
                pair_loss = float(np.mean(differences**2))
                np.add.at(derivative, a, 2*pair_weight*differences/len(a))
                np.add.at(derivative, b, -2*pair_weight*differences/len(a))
            if self.network.recurrent:gradients=backward_sequence(self.network,outputs,derivative,layout)
            else:
                gradients = {name: np.zeros_like(value) for name, value in self.parameters.items()}
                for (_, cache), scale in zip(outputs, derivative):
                    for name, value in self.network.backward(cache, scale).items():gradients[name] += value
            for name, gradient in gradients.items():
                if name.endswith('_w'):
                    gradient += 2*l2*self.parameters[name]
            norm = np.sqrt(sum(float(np.sum(value*value)) for value in gradients.values()))
            for name, gradient in gradients.items():
                gradient *= min(1., 10/max(norm, 1e-12))
                moments[name] = .9*moments[name]+.1*gradient
                variances[name] = .999*variances[name]+.001*gradient*gradient
                self.parameters[name] -= learning_rate*(moments[name]/(1-.9**epoch))/(
                    np.sqrt(variances[name]/(1-.999**epoch))+1e-8)
            row=measure(epoch,pair_loss);history.append(row);key=checkpoint_key(row,checkpoint)
            if best_key is None or key<best_key:
                best_key,best=key,deepcopy(self.parameters);self.best_epoch=epoch
            rank=key[:2] if checkpoint=='validation_feasible_then_return' else key
            if best_rank is None or rank<best_rank:
                best_rank=rank;last_rank_improvement=epoch
            if progress:progress(history[-1])
            if early_stopping_patience is not None and epoch-last_rank_improvement>=early_stopping_patience:
                self.training_stop_reason='validation_checkpoint_stagnation';break
        self.parameters = best
        self.best_checkpoint_key=best_key
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
                target_temperature=1.0,recurrent=False,warm_start=None,progress=None,
                checkpoint='validation_rmse',early_stopping_patience=None):
    data, metadata = load_dataset(dataset)
    # Fit readiness depends on valid labels and splits; road completion is
    # reported separately from per-candidate supervision.
    readiness = training_readiness(data)
    if not readiness['eligible']:
        raise ValueError('Training requires at least two Train rows and a nonempty Validation split')
    coverage = metadata.get('collection_coverage')
    train, validation = data["splits"] == "train", data["splits"] == "validation"
    model = SpatialPathPolicy(hidden=hidden, seed=seed, target_temperature=target_temperature,recurrent=recurrent)
    if warm_start is not None:
        previous=SpatialPathPolicy.load(warm_start)
        if previous.network.hidden!=hidden:raise ValueError('Fine tuning requires matching encoder width')
        for name,value in previous.parameters.items():
            if name in model.parameters:model.parameters[name]=value.copy()
        model.normalization=deepcopy(previous.normalization)
    contexts={key:data[key] for key in SEQUENCE_FIELDS} if recurrent else {}
    validation_feasible=None
    if checkpoint=='validation_feasible_then_return':
        labels={}
        for record in read_decisions(Path(dataset)/'decisions.jsonl'):
            if record['split']!='validation':continue
            for entry in record['candidates']:
                labels[(record['group_id'],entry['candidate']['candidate_id'])]=bool(entry['cstar_trials'][0]['constraints_satisfied'])
        validation_feasible=np.asarray([labels[(g,c)] for g,c in zip(data['groups'][validation],data['candidate_ids'][validation])],dtype=bool)
    learning=[]
    def learning_progress(row):
        learning.append(row)
        if progress:
            write_json(Path(output).with_suffix('.learning.json'),dict(history=learning))
            progress(dict(stage='training_epoch',**row))
    history = model.fit(data["X"][train], data["y"][train], data["groups"][train],
                        data["X"][validation], data["y"][validation],
                        epochs=epochs, learning_rate=learning_rate,
                        sequence_context={k:v[train] for k,v in contexts.items()} if recurrent else None,
                        validation_context={k:v[validation] for k,v in contexts.items()} if recurrent else None,
                        validation_groups=data['groups'][validation],refit_normalization=warm_start is None,
                        progress=learning_progress if progress else None,checkpoint=checkpoint,
                        validation_feasible=validation_feasible,early_stopping_patience=early_stopping_patience)
    model.planner_source_sha256 = metadata["planner_source_sha256"]
    model.provenance = {"dataset_sha256": metadata["data_sha256"], "seed": seed,
                        "hidden": hidden, "epochs": epochs, "learning_rate": learning_rate,
                        "normalization": "X_training_rows_only_no_y_scaler",
                        "checkpoint": "lowest_validation_score_rmse" if checkpoint=='validation_rmse' else checkpoint,
                        "optimizer": "Adam", "activation": "tanh", "pair_weight": 0.25, "l2": 1e-4,
                        "output_activation": "sigmoid", "training_target_transform": model.target_specification,
                        "loss": "decision_weighted_MSE_plus_within_decision_pair_difference_MSE"}
    model.provenance["pipeline_code_sha256"] = metadata["pipeline_code_sha256"]
    model.provenance["objective"] = metadata["objective"]
    model.provenance["label_source"] = metadata["label_source"]
    model.provenance['execution_condition'] = metadata['execution_condition']
    model.provenance["label_aggregation"] = metadata["label_aggregation"]
    model.provenance["target"] = metadata["label_definition"]
    model.provenance["training_code_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    model.provenance["dataset_training_eligible"] = readiness['eligible']
    model.provenance["dataset_reported_training_eligible"] = metadata.get('training_eligible')
    model.provenance["training_readiness"] = readiness
    model.provenance["road_coverage_complete"] = road_coverage_complete(coverage)
    model.provenance["coverage_override_applied"] = False
    model.provenance["training_scope"] = "candidate_supervision"
    model.provenance["collection_coverage"] = deepcopy(coverage)
    model.provenance["best_epoch"] = model.best_epoch
    model.provenance['best_checkpoint_key']=model.best_checkpoint_key
    model.provenance['epochs_completed']=history[-1]['epoch']
    model.provenance['early_stopping_patience']=early_stopping_patience
    model.provenance['training_stop_reason']=model.training_stop_reason
    model.provenance['recurrent_state']='executed_branch_replay_BPTT' if recurrent else 'none'
    model.provenance['warm_start_sha256']=hashlib.sha256(Path(warm_start).read_bytes()).hexdigest() if warm_start else None
    if warm_start is not None:model.provenance['normalization']='preserved_training_only_statistics_from_warm_start'
    model.save(output)
    predictions = model.predict_dataset(data)
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
    if model.provenance.get('execution_condition') != metadata['execution_condition']:
        raise ValueError('Model and evaluation dataset use different fixed execution conditions')
    predictions = model.predict_dataset(data)
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
