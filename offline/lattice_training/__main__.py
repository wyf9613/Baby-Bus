"""One native-env/C* policy pipeline, plus contract/configuration and inference tools."""
import argparse
import json
from pathlib import Path

from .dataset import DECISION_FORMAT, json_safe, score_decisions, validate_objective, write_json
from .features import FEATURE_SCHEMA, FEATURE_VERSION, INPUT_GROUPS
from .model import SpatialPathPolicy
from .objective import CStarTrackingSettings
from .source import planner_source_sha256, POLICY_INPUT_SCHEMA


def progress(record):
    print(json.dumps(json_safe(record), ensure_ascii=False, allow_nan=False), flush=True)


def seeds(value):
    try:
        return tuple(int(part) for part in value.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError("Use comma-separated integer seeds") from error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    common = argparse.ArgumentParser(add_help=False)
    run = common
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--objective", type=Path, default=Path(__file__).with_name("cstar_objective.json"))
    run.add_argument("--kinds", default="randomized")
    run.add_argument("--road-settings", type=Path, default=Path(__file__).with_name("road_randomization.json"))
    run.add_argument("--train-seeds", type=seeds, default=tuple(range(8)))
    run.add_argument("--decisions", type=int, default=3)
    run.add_argument("--until-road-end", action="store_true",
                     help="Continue random candidate chains to the body-safe road goal or an explicit failure")
    run.add_argument("--max-segments", type=int, default=100,
                     help="Safety ceiling for until-road-end; exhausting it is reported as a failure")
    run.add_argument("--starts-per-road", type=int, default=2)
    run.add_argument("--planner-profiles", default="standard,conservative")
    run.add_argument('--planner-sample-count', type=int,
                     help='Explicit offline native candidate count; minimum 8')
    run.add_argument('--planner-rate-hz', type=float, default=1.,
                     help='Native job horizon parameter; collection itself advances once per full segment')
    run.add_argument('--terminal-support-m', type=float, default=0.,
                     help='Explicit sensor-support road extension beyond the unchanged collection goal')
    run.add_argument("--segment-resampling-attempts", type=int, default=6,
                     help="Try random C*-feasible candidates until a complete legal execution is found")
    run.add_argument("--trials", type=int, default=2)
    run.add_argument("--evaluation-steps", type=int, default=60)
    run.add_argument("--population", type=int, default=12)
    run.add_argument("--iterations", type=int, default=4)
    run.add_argument("--max-evaluations", type=int, default=64)
    run.add_argument("--solver-wall-s", type=float, default=10.0)
    run.add_argument("--safety-margin-m", type=float, default=0.02)
    run.add_argument("--speed-max-mps", type=float, default=0.5)
    run.add_argument("--acceleration-max-mps2", type=float, default=1.0)
    run.add_argument("--lateral-acceleration-max-mps2", type=float, default=1.0)
    run = commands.add_parser("run", parents=[common],
                              help="env -> lattice -> C* -> actual labels -> training -> closed-loop evaluation")
    collect = commands.add_parser("collect", parents=[common], help="Collect/freeze the same dataset without training or policy evaluation")
    for command in (run, collect):
        command.add_argument("--validation-seeds", type=seeds, default=(100, 101))
        command.add_argument("--test-seeds", type=seeds, default=(200, 201))
    augment = commands.add_parser('augment', parents=[common], help='Collect new Train roads and append to an existing dataset; preserve holdouts')
    augment.add_argument('--dataset', type=Path, required=True)
    augment.set_defaults(train_seeds=tuple(range(4, 12)))
    run.add_argument("--epochs", type=int, default=150)
    run.add_argument("--hidden", type=int, default=32)
    run.add_argument("--seed", type=int, default=0)
    run.add_argument("--learning-rate", type=float, default=0.005)
    train = commands.add_parser("train", help="Train the same policy on an existing frozen C* dataset")
    train.add_argument("--dataset", type=Path, required=True)
    train.add_argument("--output", type=Path, required=True)
    train.add_argument("--epochs", type=int, default=150)
    train.add_argument("--hidden", type=int, default=32)
    train.add_argument("--seed", type=int, default=0)
    train.add_argument("--learning-rate", type=float, default=.005)
    score = commands.add_parser("score", help="Score admitted candidates using a C*-trained policy")
    score.add_argument("--records", type=Path, required=True)
    score.add_argument("--model", type=Path, required=True)
    score.add_argument("--output", type=Path, required=True)
    schema = commands.add_parser("schema", help="Show the internal C* decision/trial contract")
    schema.add_argument("--objective", type=Path)
    objective = commands.add_parser("cstar-objective", help="Write the shared tracking loss configuration")
    objective.add_argument("--output", type=Path, required=True)
    for name, default in vars(CStarTrackingSettings()).items():
        objective.add_argument("--"+name.replace("_", "-"), type=float, default=default)
    args = parser.parse_args()
    if args.command == "cstar-objective":
        settings = CStarTrackingSettings(**{name: getattr(args, name) for name in vars(CStarTrackingSettings())})
        specification = settings.specification()
        write_json(args.output, specification)
        progress({"stage": "cstar_objective_written", "output": str(args.output),
                  "objective_sha256": validate_objective(specification), "objective": specification})
    elif args.command == "schema":
        specification = (json.loads(args.objective.read_text(encoding="utf-8")) if args.objective
                         else CStarTrackingSettings().specification())
        progress({"format": DECISION_FORMAT, "planner_source_sha256": planner_source_sha256(),
                  "policy_input_schema": POLICY_INPUT_SCHEMA,"input_groups":INPUT_GROUPS,
                  "default_candidate_representation": "policy_frenet_candidate_v2",
                  "feature_version": FEATURE_VERSION, "feature_schema": FEATURE_SCHEMA,
                  "required_fields": ["group_id", "observation", "candidates"],
                  "training_fields": ["scene_id", "split", "objective", "label_source"],
                  "candidate_fields": ["candidate", "cstar_trials"],
                  "label_source_fields": ["name", "version", "objective_sha256", "execution_assumption=cstar"],
                  "objective": specification, "objective_sha256": validate_objective(specification)})
    elif args.command == "score":
        model = SpatialPathPolicy.load(args.model)
        results = score_decisions(model, args.records, args.output)
        progress({"stage": "policy_scoring_complete", "decisions": len(results), "output": str(args.output)})
    elif args.command == "train":
        from .model import train_model, evaluate_policy
        from .plots import plot_training
        if args.epochs < 1 or args.hidden < 1 or args.learning_rate <= 0:
            parser.error("epochs, hidden and learning-rate must be positive")
        output = args.output/'model.json'
        if output.exists() or output.with_suffix('.training.json').exists():
            raise FileExistsError(f"Training output already exists: {args.output}")
        progress({'stage':'training_start','dataset':str(args.dataset),'epochs':args.epochs})
        model,report=train_model(args.dataset,output,epochs=args.epochs,hidden=args.hidden,seed=args.seed,
                                 learning_rate=args.learning_rate)
        plot_training(report,args.output)
        evaluate_policy(model,args.dataset,args.output/'evaluation.json')
        progress({'stage':'training_complete','model':str(output),'best_epoch':model.provenance['best_epoch'],
                  'prediction_space':model.prediction_space,'target_transform':model.target_specification,
                  'training_scope':model.provenance['training_scope'],'splits':report['splits']})
    else:
        if args.command == "run" and (args.epochs < 1 or args.hidden < 1 or args.learning_rate <= 0):
            parser.error("epochs, hidden and learning-rate must be positive")
        from .cstar import SolverSettings
        from .pipeline import PipelineSettings, run_pipeline, collect_dataset, CollectionCoverageError, augment_training_dataset
        from .roads import RoadRandomizationSettings
        configuration = CStarTrackingSettings.from_specification(json.loads(args.objective.read_text(encoding="utf-8")))
        settings = PipelineSettings(kinds=tuple(args.kinds.split(',')), train_seeds=args.train_seeds, validation_seeds=() if args.command == 'augment' else args.validation_seeds, test_seeds=() if args.command == 'augment' else args.test_seeds, train_only=args.command == 'augment', decisions=args.decisions, until_road_end=args.until_road_end, max_segments=args.max_segments, trials=args.trials, evaluation_steps=args.evaluation_steps, starts_per_road=args.starts_per_road, planner_profiles=tuple(args.planner_profiles.split(',')), segment_resampling_attempts=args.segment_resampling_attempts, planner_sample_count=args.planner_sample_count, planner_rate_hz=args.planner_rate_hz, terminal_support_m=args.terminal_support_m)
        road_settings = RoadRandomizationSettings(**json.loads(args.road_settings.read_text(encoding="utf-8")))
        solver = SolverSettings(population=args.population, iterations=args.iterations,
            max_evaluations=args.max_evaluations, max_wall_s=args.solver_wall_s,
            safety_margin_m=args.safety_margin_m, speed_max_mps=args.speed_max_mps,
            acceleration_max_mps2=args.acceleration_max_mps2,
            lateral_acceleration_max_mps2=args.lateral_acceleration_max_mps2)
        if args.command == 'augment':
            summary = augment_training_dataset(args.dataset, args.output, settings=settings, objective=configuration,
                solver_settings=solver, progress=progress, road_settings=road_settings)
            progress({'stage': 'augmentation_complete', **summary})
            return
        if args.command == "collect":
            try:
                summary = collect_dataset(args.output, settings=settings, objective=configuration,
                    solver_settings=solver, progress=progress, road_settings=road_settings)
            except CollectionCoverageError as error:
                progress({'stage':'collection_incomplete',**error.summary})
                raise SystemExit(2) from error
            progress({"stage": "collection_complete", **summary})
            return
        try:
            summary = run_pipeline(args.output, settings=settings, objective=configuration, solver_settings=solver,
                                   epochs=args.epochs, hidden=args.hidden, seed=args.seed,
                                   learning_rate=args.learning_rate, progress=progress,
                                   road_settings=road_settings)
        except CollectionCoverageError as error:
            progress({'stage':'collection_incomplete',**error.summary})
            raise SystemExit(2) from error
        progress({"stage": "pipeline_complete", "dataset_rows": summary["dataset_rows"],
                  "closed_loop": summary["closed_loop"], "model": summary["model"]})


if __name__ == "__main__":
    main()
