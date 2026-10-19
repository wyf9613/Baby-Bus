"""Read configuration assignments, never execute or modify the notebook."""
import ast
from pathlib import Path
import json

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
NOTEBOOK = ROOT / 'docs/ad_gym_system_project_v2026_09_18.ipynb'


def notebook_config() -> dict:
    cells = json.loads(NOTEBOOK.read_text(encoding='utf-8'))['cells']
    wanted = {
        'CONE_LANE_WIDTH_M', 'CONE_REGULAR_PROFILE', 'CONE_IRREGULAR_PROFILE',
        'CONE_PATHS', 'CONE_ROAD_SPEC', 'bicycle_model_config', 'integration_config',
    }
    context = {'np': np}
    for cell in cells:
        if cell['cell_type'] != 'code':
            continue
        for statement in ast.parse(''.join(cell['source'])).body:
            if (isinstance(statement, ast.Assign) and len(statement.targets) == 1
                    and isinstance(statement.targets[0], ast.Name)
                    and statement.targets[0].id in wanted):
                context[statement.targets[0].id] = eval(
                    compile(ast.Expression(statement.value), str(NOTEBOOK), 'eval'),
                    {'__builtins__': {}, **context})
    missing = wanted - context.keys()
    if missing:
        raise ValueError(f'Missing notebook configurations: {missing}')
    return {name: context[name] for name in wanted}
