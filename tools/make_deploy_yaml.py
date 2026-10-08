#!/usr/bin/env python3
"""Build the single policy YAML that DREAM reads on car .27 (babysitter guide 5.2).

    python tools/make_deploy_yaml.py --mode mpc-shadow [--distance 3.0] [--out DIR]

Layers, merged recursively in order (the repository files stay unchanged):
  mvp:        ai4r_policy.yaml + ai4r_policy_newcar27.yaml
  mpc-shadow: ... + ai4r_policy_mpc_newcar27.yaml
  mpc-exec:   ... + the same, with shadow off and v_exec_max_mps <= 0.2 (needs vehicle.valid: true)
Writes DIR/ai4r_policy.yaml (copy it to the car as config/ai4r_policy.yaml), DIR/mode.txt
and DIR/policy_node.py, and prints both SHA-256 values to compare on the car.
Needs PyYAML only.
"""
import argparse
from copy import deepcopy
import hashlib
from pathlib import Path
import shutil
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
KEY = "/**/ai4r_policy"


def merge(target, changes):
    for name, value in changes.items():
        if isinstance(value, dict) and isinstance(target.get(name), dict):
            merge(target[name], value)
        else:
            target[name] = deepcopy(value)


def build(mode, distance):
    layers = ["ai4r_policy.yaml", "ai4r_policy_newcar27.yaml"]
    if mode != "mvp":
        layers.append("ai4r_policy_mpc_newcar27.yaml")
    base = yaml.safe_load((ROOT / "config" / layers[0]).read_text(encoding="utf-8"))
    for layer in layers[1:]:
        merge(base, yaml.safe_load((ROOT / "config" / layer).read_text(encoding="utf-8")))
    p = base[KEY]["ros__parameters"]
    p["control"]["max_distance_m"] = distance
    if mode == "mpc-exec":
        p["mpc"].update(shadow=False, v_exec_max_mps=min(p["mpc"]["v_exec_max_mps"], 0.2))

    assert p["policy_update_mode"] == "timer" and p["policy_update_rate_hz"] == 20.0
    assert p["required_sensors"] == ["cone_detections", "wheel_speed", "imu_angular_velocity"]
    assert p["id_test"]["mode"] == "off"
    if mode == "mvp":
        assert p["control"]["enabled"] is True and p["mpc"]["enabled"] is False
        assert p["control"]["mode"] == "mvp" and p["control"]["mvp_steering_direction"] == -1.0
    else:
        assert p["control"]["enabled"] is False and p["mpc"]["enabled"] is True
        assert p["mpc"]["vehicle_params_source"] == "vehicle"
        assert p["mpc"]["reference_source"] == "planning"
        assert p["mpc"]["qp_solver"] == "dense", "the car has no osqp (G1, 2026-10-08)"
        assert p["planning"]["vehicle_limits_source"] == "upstream"
        if mode == "mpc-shadow":
            assert p["mpc"]["shadow"] is True
        else:
            assert p["vehicle"]["valid"] is True, \
                "mpc-exec: put the measured values in ai4r_policy_mpc_newcar27.yaml and set vehicle.valid: true first"
    return base, layers


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mode", required=True, choices=["mvp", "mpc-shadow", "mpc-exec"])
    parser.add_argument("--distance", type=float, default=3.0, help="control.max_distance_m, in (0, 3]")
    parser.add_argument("--out", type=Path, default=ROOT / ".verification" / "car-deploy")
    args = parser.parse_args()
    if not 0 < args.distance <= 3.0:
        parser.error("--distance must be in (0, 3]")
    params, layers = build(args.mode, args.distance)
    args.out.mkdir(parents=True, exist_ok=True)
    yaml_path, node_path = args.out / "ai4r_policy.yaml", args.out / "policy_node.py"
    yaml_path.write_bytes(yaml.safe_dump(params, sort_keys=False).encode("utf-8"))
    shutil.copyfile(ROOT / "scripts" / "policy_node.py", node_path)
    (args.out / "mode.txt").write_text(f"{args.mode} {args.distance} layers={'+'.join(layers)}\n", encoding="utf-8")
    p = params[KEY]["ros__parameters"]
    print(f"mode {args.mode}, layers {' + '.join(layers)}, max_distance_m {args.distance}")
    print("control.enabled", p["control"]["enabled"], "| mpc",
          {k: p["mpc"][k] for k in ("enabled", "shadow", "v_exec_max_mps", "qp_solver")},
          "| vehicle", p["vehicle"]["source"], "valid", p["vehicle"]["valid"])
    print(f"{yaml_path}\n  sha256 {sha256(yaml_path)}")
    print(f"{node_path}\n  sha256 {sha256(node_path)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
