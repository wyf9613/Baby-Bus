"""Extract the course configuration without executing notebook cells."""
import ast
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
import operator
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NOTEBOOK = ROOT / "docs/ad_gym_system_project_v2026_09_18.ipynb"
DREAMGYM_VERSION = "0.4.0"
DREAMGYM_COMMIT = "9e9d031fabf5c97b32428925e56bb6a13de713f3"
_NAMES = {
    "COURSE_SOURCE_REF", "CONE_LANE_WIDTH_M", "CONE_REGULAR_PROFILE",
    "CONE_IRREGULAR_PROFILE", "CONE_PATHS", "CONE_ROAD_SPEC",
    "bicycle_model_config", "integration_config", "initial_state_config",
    "termination_config", "CONE_DETECTION_NOISE_STD_M", "CONE_NOISE_SEED",
    "CONE_OBSERVATION_CONFIG", "ROAD_SPEC_WITH_OBSTACLES",
    "OBSERVATION_CONFIG_WITH_LIDAR",
}
_OPERATORS = {ast.Add: operator.add, ast.Sub: operator.sub,
              ast.Mult: operator.mul, ast.Div: operator.truediv,
              ast.Pow: operator.pow}


def _value(node, context):
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name) and node.id in context:
        return deepcopy(context[node.id])
    if isinstance(node, (ast.List, ast.Tuple)):
        values = [_value(v, context) for v in node.elts]
        return tuple(values) if isinstance(node, ast.Tuple) else values
    if isinstance(node, ast.Dict) and all(k is not None for k in node.keys):
        return {_value(k, context): _value(v, context)
                for k, v in zip(node.keys, node.values)}
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
        value = _value(node.operand, context)
        return value if isinstance(node.op, ast.UAdd) else -value
    if isinstance(node, ast.BinOp) and type(node.op) in _OPERATORS:
        return _OPERATORS[type(node.op)](_value(node.left, context), _value(node.right, context))
    if isinstance(node, ast.Call) and len(node.args) == 1 and not node.keywords:
        if isinstance(node.func, ast.Name) and node.func.id == "deepcopy":
            return deepcopy(_value(node.args[0], context))
        if (isinstance(node.func, ast.Attribute) and node.func.attr == "deg2rad"
                and isinstance(node.func.value, ast.Name) and node.func.value.id == "np"):
            return math.radians(_value(node.args[0], context))
    raise ValueError(f"Unsupported notebook configuration expression: {ast.dump(node)}")


@dataclass
class NotebookConfig:
    road_spec: dict
    bicycle_model_config: dict
    observation_config: dict
    initial_state_config: dict
    integration_config: dict
    termination_config: dict
    provenance: dict


def load_notebook(path=NOTEBOOK, *, with_obstacles=True):
    """Read only named assignments and their explicit dictionary additions.

    Imports, pip/git setup, environment construction and notebook policies are
    never executed. Unknown expressions in selected assignments fail closed.
    Returned dictionaries are independent and may be changed for experiments.
    """
    path = Path(path).resolve()
    data = path.read_bytes()
    context, source_cells = {}, {}
    for index, cell in enumerate(json.loads(data)["cells"]):
        if cell["cell_type"] != "code":
            continue
        for statement in ast.parse("".join(cell["source"])).body:
            if not isinstance(statement, ast.Assign) or len(statement.targets) != 1:
                continue
            target = statement.targets[0]
            if isinstance(target, ast.Name) and target.id in _NAMES:
                context[target.id] = _value(statement.value, context)
                source_cells[target.id] = index
            elif (isinstance(target, ast.Subscript) and isinstance(target.value, ast.Name)
                  and target.value.id in {"ROAD_SPEC_WITH_OBSTACLES", "OBSERVATION_CONFIG_WITH_LIDAR"}):
                context[target.value.id][_value(target.slice, context)] = _value(statement.value, context)
    missing = _NAMES - context.keys()
    if missing:
        raise ValueError(f"Notebook is missing configuration assignments: {sorted(missing)}")
    if context["COURSE_SOURCE_REF"] != "v" + DREAMGYM_VERSION:
        raise ValueError("Notebook dependency differs from the supported Dream Gym v0.4.0")
    road = "ROAD_SPEC_WITH_OBSTACLES" if with_obstacles else "CONE_ROAD_SPEC"
    observation = "OBSERVATION_CONFIG_WITH_LIDAR" if with_obstacles else "CONE_OBSERVATION_CONFIG"
    return NotebookConfig(
        road_spec=context[road], bicycle_model_config=context["bicycle_model_config"],
        observation_config=context[observation], initial_state_config=context["initial_state_config"],
        integration_config=context["integration_config"], termination_config=context["termination_config"],
        provenance={"notebook": str(path), "notebook_sha256": hashlib.sha256(data).hexdigest(),
                    "dreamgym_version": DREAMGYM_VERSION, "dreamgym_expected_commit": DREAMGYM_COMMIT,
                    "configuration_cells_zero_based": source_cells, "with_obstacles": with_obstacles})
