"""Replay the recorded road through the shared modular MVP2.1 execution."""
import argparse
import json
from pathlib import Path
from .runtime import run_runtime as run
from .source import ROOT

DEFAULT_ROAD = ROOT/'offline/lattice_training/artifacts/2026-10-15-native-diagnosis/recorded-road.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--road', type=Path, default=DEFAULT_ROAD)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--road-input', choices=('perception', 'map'), default='perception')
    parser.add_argument('--path-source', choices=('perception', 'lattice'),
        help='Default reads planner_control.enabled from the common YAML')
    parser.add_argument('--delivery-delay-s', type=float, default=.1)
    parser.add_argument('--measured-delay', action='store_true')
    parser.add_argument('--handoff-budget-s', type=float)
    parser.add_argument('--collision-coverage', choices=('prefix', 'generated'),
        help='Explicit experiment override; default reads the common configuration')
    parser.add_argument('--score-coverage', choices=('prefix', 'generated'))
    parser.add_argument('--lattice-length-m', type=float,
        help='Explicit experiment override; default keeps MVP2.1 length_m=1.0')
    parser.add_argument('--planner-rate-hz', type=float)
    parser.add_argument('--require-native-only', action='store_true',
        help='Fail acceptance on any baseline cycle; does not change runtime fallback')
    parser.add_argument('--max-steps', type=int, default=1200)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--progress', type=float)
    parser.add_argument('--initial-speed', type=float, default=0.)
    parser.add_argument('--latency-steps', type=int, default=4)
    parser.add_argument('--steering-sign', type=float, choices=(-1., 1.), default=-1.)
    parser.add_argument('--no-shadow', action='store_true')
    args = parser.parse_args()
    if args.max_steps < 1 or args.latency_steps < 0 or args.initial_speed < 0:
        parser.error('Positive max_steps and nonnegative latency/speed required')
    result = run(json.loads(args.road.read_text(encoding='utf-8')), args.output,
        road_input=args.road_input, max_steps=args.max_steps, seed=args.seed, progress=args.progress,
        initial_speed=args.initial_speed, latency_steps=args.latency_steps, steering_sign=args.steering_sign,
        shadow_enabled=not args.no_shadow, path_source=args.path_source,
        delivery_delay_s=args.delivery_delay_s, measured_delay=args.measured_delay,
        handoff_budget_s=args.handoff_budget_s, collision_coverage=args.collision_coverage,
        score_coverage=args.score_coverage, lattice_length_m=args.lattice_length_m,
        planner_rate_hz=args.planner_rate_hz)
    accepted = result['native_only_passed'] if args.require_native_only else result['completed']
    return 0 if accepted else 1


if __name__ == '__main__':
    raise SystemExit(main())
