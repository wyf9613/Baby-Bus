"""Copy pinned headless simulator state including RNGs and hidden latches."""
from collections.abc import Mapping
from copy import deepcopy
from dataclasses import fields, is_dataclass
import importlib
from types import FunctionType, MappingProxyType, ModuleType

import numpy as np


def clone_runtime(runtime):
    """Deep copy mapping-proxy contents without globally patching copy.

    Dream Gym v0.4.0 uses two object identity sentinels. Keep their module
    identity; copy all mutable data, preserving RNG aliases within the copy.
    Mapping proxies are rebuilt so even mutable values underneath are isolated.
    Render resources are excluded by the environment's headless-only contract.
    """
    module = importlib.import_module("dreamgym.envs.autonomous_driving_env")
    memo = {id(getattr(module, name)): getattr(module, name)
            for name in ("_UNRESOLVED_VISUALIZATION_THEME", "_NO_PENDING_REFERENCE_LINE_PROGRESS_SEARCH")}
    proxies, seen = [], set()

    def walk(value):
        identity = id(value)
        if identity in seen:
            return
        seen.add(identity)
        if isinstance(value, (str, bytes, int, float, bool, type(None), type, ModuleType, FunctionType)):
            return
        if isinstance(value, np.ndarray) and not value.dtype.hasobject:
            return
        if isinstance(value, MappingProxyType):
            backing = {}
            memo[identity] = MappingProxyType(backing)
            proxies.append((value, backing))
        if isinstance(value, Mapping):
            for key, item in value.items():
                walk(key)
                walk(item)
        elif isinstance(value, (list, tuple, set, frozenset)):
            for item in value:
                walk(item)
        elif is_dataclass(value):
            for field in fields(value):
                walk(getattr(value, field.name))
        elif hasattr(value, "__dict__"):
            walk(vars(value))
        # Objects with slots (compiled immutable road records) may also hold
        # mapping proxies; inspect data slots without invoking properties.
        for cls in type(value).__mro__:
            slots = cls.__dict__.get("__slots__", ())
            if isinstance(slots, str):
                slots = (slots,)
            for name in slots:
                if name not in {"__dict__", "__weakref__"} and hasattr(value, name):
                    walk(getattr(value, name))

    walk(runtime)
    for original, backing in proxies:
        backing.update(deepcopy(dict(original), memo))
    return deepcopy(runtime, memo)
