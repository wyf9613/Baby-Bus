"""Load scripts/policy_node.py without ROS, for offline replay and simulation.

The whole module is executed the way the async planner worker loads it
(sys.argv == [..., "--planner-worker"]): no rclpy/DDS/TF imports, Node = object,
and the __main__ entry point does not run. Every pure definition is therefore
available without a per-tool whitelist; PolicyNode methods are taken from the
class and bound to a stand-in node.

Only ROS message types are missing. Callers pass stand-ins via `extra`
(for example String, ConeDetection).
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "scripts" / "policy_node.py"


def load_policy_namespace(extra=None):
    namespace = {"__name__": "policy_node_offline", "__file__": str(SOURCE)}
    argv = sys.argv
    sys.argv = [str(SOURCE), "--planner-worker"]
    try:
        exec(compile(SOURCE.read_text(encoding="utf-8"), str(SOURCE), "exec"), namespace)
    finally:
        sys.argv = argv
    namespace.update(extra or {})
    return namespace


def bind_methods(node, namespace, names):
    """Bind PolicyNode methods (by name) to a stand-in node."""
    cls = namespace["PolicyNode"]
    for name in names:
        setattr(node, name, getattr(cls, name).__get__(node))


def policy_settings(namespace, params, synchronous_planning=True):
    """Settings objects from a parameter tree, as PolicyNode declares them.

    planning.lattice.* becomes LatticeSettings. With synchronous_planning the
    async worker is disabled: offline loops call Planning V2 in-line, so their
    results exclude the worker delay and the MPC hold of pending cycles.
    """
    planning = dict(params.get("planning", {}))
    lattice = dict(planning.pop("lattice", None) or {})
    if synchronous_planning:
        lattice["async_enabled"] = False
    sections = {"estimation": "EstimationSettings", "mpc": "MPCSettings",
                "vehicle": "VehicleParamsSettings", "control": "ControlSettings"}
    settings = {name: namespace[cls](**params.get(name, {})) for name, cls in sections.items()}
    settings["planning"] = namespace["PlanningSettings"](**planning)
    settings["lattice"] = namespace["LatticeSettings"](**lattice)
    return settings
