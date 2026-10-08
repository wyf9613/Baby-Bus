#!/usr/bin/env python3
"""Build an executable .py containing project modules and resource files.

No entry/imported user code is executed during the build. The Python runtime,
standard library and unselected third-party packages remain runtime dependencies.
"""
import argparse
import ast
import base64
import hashlib
import importlib.machinery
import importlib.util
import io
import json
import os
from pathlib import Path, PurePosixPath
import pprint
import sys
import tempfile
import tokenize

_IGNORED = {".git", ".hg", ".svn", ".venv", "venv", "__pycache__", ".verification",
            ".codex", ".aws", ".pytest_cache"}
_NATIVE_SUFFIXES = tuple(dict.fromkeys((*importlib.machinery.EXTENSION_SUFFIXES, ".pyd", ".so")))


class BundleError(ValueError):
    pass


def _under(path, root):
    return path == root or root in path.parents


def _safe_relative(value):
    path = PurePosixPath(str(value).replace("\\", "/"))
    if path.is_absolute() or ".." in path.parts or not path.parts or ":" in path.parts[0]:
        raise BundleError(f"Unsafe bundle path: {value}")
    return path.as_posix()


def _read_source(path):
    data = path.read_bytes()
    encoding, _ = tokenize.detect_encoding(io.BytesIO(data).readline)
    text = data.decode(encoding)
    return text, encoding, ast.parse(data, filename=str(path))


def _default_root(entry):
    for parent in entry.parents:
        if any((parent / name).exists() for name in (".git", "pyproject.toml", "setup.py")):
            return parent
    root = entry.parent
    while (root / "__init__.py").exists():
        root = root.parent
    return root


class Builder:
    def __init__(self, entry, root, output, *, includes=(), externals=(), resources=(),
                 profile="off", cwd="caller", auto_resources=True, strict=False,
                 search_paths=()):
        self.entry, self.root, self.output = map(lambda p: Path(p).resolve(), (entry, root, output))
        if not self.entry.is_file() or self.entry.suffix != ".py":
            raise BundleError("Entry must be an existing Python .py file")
        if not _under(self.entry, self.root):
            raise BundleError("Entry must be inside --root")
        if self.entry == self.output:
            raise BundleError("Output must not overwrite the entry")
        self.includes, self.externals = set(includes), set(externals)
        self.resources_requested = list(resources)
        self.profile, self.cwd, self.auto_resources, self.strict = profile, cwd, auto_resources, strict
        roots = [self.entry.parent, self.root, *(Path(p).resolve() for p in search_paths)]
        roots.extend(Path(p or os.getcwd()).resolve() for p in sys.path)
        self.search_paths = list(dict.fromkeys(str(p) for p in roots))
        self.modules, self.sources, self.resources = {}, {}, {}
        self.external_modules, self.dynamic_imports, self.first_level = {}, [], set()
        self.path_roots = {str(self.root): "."}
        self._pending, self._scanned, self._forced_packages = [], set(), set()
        self.entry_relative = self.entry.relative_to(self.root).as_posix()
        parts = self.entry.relative_to(self.root).with_suffix("").parts
        self.entry_name = ".".join(parts[:-1] if parts[-1] == "__init__" else parts)
        package_mode = self.entry.name == "__init__.py" or (self.entry.parent / "__init__.py").exists()
        self.entry_package = (self.entry_name if self.entry.name == "__init__.py"
                              else self.entry_name.rpartition(".")[0]) if package_mode else ""

    def is_project(self, path):
        return _under(path, self.root) and not any(
            part in _IGNORED for part in path.relative_to(self.root).parts)

    def is_selected(self, name):
        return any(name == prefix or name.startswith(prefix + ".") for prefix in self.includes)

    def is_external(self, name):
        return any(name == prefix or name.startswith(prefix + ".") for prefix in self.externals)

    def find(self, name):
        """Resolve filesystem specs without importlib.util.find_spec executing parents."""
        path, spec = self.search_paths, None
        for index, part in enumerate(name.split(".")):
            fullname = ".".join(name.split(".")[:index+1])
            spec = importlib.machinery.PathFinder.find_spec(fullname, path)
            if spec is None:
                if index == 0:
                    spec = (importlib.machinery.BuiltinImporter.find_spec(fullname)
                            or importlib.machinery.FrozenImporter.find_spec(fullname))
                if spec is None:
                    return None
            if index < len(name.split("."))-1:
                if spec.submodule_search_locations is None:
                    return None
                path = list(spec.submodule_search_locations)
        return spec

    def _add_source(self, path, destination, name, *, package=False):
        path, destination = path.resolve(), _safe_relative(destination)
        if path == self.output:
            raise BundleError(f"Output is also an input dependency: {path}")
        if destination in self.resources:
            if self.resources[destination] != path.read_bytes():
                raise BundleError(f"Resource/source collision: {destination}")
            del self.resources[destination]
        if destination in self.sources and self.sources[destination]["path"] != path:
            raise BundleError(f"Two different source files map to {destination}")
        if destination not in self.sources:
            text, encoding, tree = _read_source(path)
            self.sources[destination] = {"source": text, "encoding": encoding, "path": path, "tree": tree}
        self.modules[name] = {"file": destination, "package": package}
        self._pending.append((name, path, destination, package))

    def add_module(self, name, *, force=False):
        if not name or name in self.modules:
            return
        if name in self.external_modules:
            if not force and not self.is_selected(name):
                return
            del self.external_modules[name]
        if self.is_external(name):
            self.external_modules[name] = "explicitly external"
            return
        spec = self.find(name)
        if spec is None:
            if force:
                raise BundleError(f"Explicitly included module cannot be resolved: {name}")
            self.external_modules[name] = "not found; may be an optional import"
            return
        package = spec.submodule_search_locations is not None
        if spec.origin in (None, "built-in", "frozen"):
            if package:
                self.modules[name] = {"file": None, "package": True}
            else:
                self.external_modules[name] = spec.origin or "no Python source"
            return
        reported_path = Path(spec.origin).absolute()
        path = reported_path.resolve()
        if reported_path != path and self.is_project(reported_path):
            raise BundleError(f"Project source symlink changes module/file layout: {reported_path}. "
                              "Use the real source package instead.")
        local, selected = self.is_project(path), force or self.is_selected(name)
        if path.suffix not in {".py", ".pyw"}:
            if local or selected:
                raise BundleError(f"Native/non-source dependency {name} cannot be source-bundled. "
                                  f"Keep it external with --external {name.split('.')[0]}.")
            self.external_modules[name] = "native extension or non-source module"
            return
        if not local and not selected:
            self.external_modules[name] = "runtime Python package / standard library"
            return
        if local:
            destination = path.relative_to(self.root).as_posix()
        else:
            destination = "_vendor/" + name.replace(".", "/") + ("/__init__.py" if package else ".py")
            if package:
                self.path_roots[str(path.parent)] = "_vendor/" + name.replace(".", "/")
        parent = name.rpartition(".")[0]
        if parent:
            self.add_module(parent, force=selected)
        self._add_source(path, destination, name, package=package)
        if selected and package and name not in self._forced_packages:
            self._forced_packages.add(name)
            package_root = path.parent
            resource_prefix = (package_root.relative_to(self.root).as_posix() if local
                               else "_vendor/" + name.replace(".", "/"))
            for child in sorted(package_root.rglob("*")):
                relative = child.relative_to(package_root)
                if any(part in _IGNORED or part.startswith(".") for part in relative.parts):
                    continue
                if not child.is_file():
                    continue
                if child.name.endswith(_NATIVE_SUFFIXES):
                    raise BundleError(f"Selected package {name} contains native extensions; "
                                      "leave the whole package external.")
                if child.suffix in {".py", ".pyw"}:
                    suffix = relative.with_suffix("").parts
                    child_name = ".".join((name, *(suffix[:-1] if suffix[-1] == "__init__" else suffix)))
                    if child_name != name:
                        self.add_module(child_name, force=True)
                elif child.suffix != ".pyc":
                    self._add_resource(child, resource_prefix + "/" + relative.as_posix())

    def _add_resource(self, path, destination):
        path, destination = Path(path).resolve(), _safe_relative(destination)
        if path == self.output:
            raise BundleError("Output is also a requested resource")
        data = path.read_bytes()
        if destination in self.sources:
            if data != self.sources[destination]["path"].read_bytes():
                raise BundleError(f"Resource/source collision: {destination}")
            return
        if destination in self.resources and self.resources[destination] != data:
            raise BundleError(f"Two different resources map to {destination}")
        self.resources[destination] = data

    def add_resource_request(self, request):
        # An external resource requires an explicit portable destination.
        source = Path(request)
        if source.exists():
            destination = None
        elif "=" in request:
            source_text, destination = request.split("=", 1)
            source = Path(source_text)
        else:
            destination = None
        if not source.is_absolute() and not source.exists() and (self.root / source).exists():
            source = self.root / source
        logical_source = Path(os.path.abspath(source))
        source = logical_source.resolve()
        if not source.exists():
            raise BundleError(f"Resource does not exist: {source}")
        if destination is None:
            if not _under(logical_source, self.root):
                raise BundleError("External resource needs SOURCE=RELATIVE_DESTINATION")
            destination = logical_source.relative_to(self.root).as_posix()
        if source.is_file():
            self._add_resource(source, destination)
        else:
            for child in sorted(source.rglob("*")):
                relative = child.relative_to(source)
                if any(part in _IGNORED for part in relative.parts) or not child.is_file():
                    continue
                if not _under(child.resolve(), source):
                    raise BundleError(f"Resource symlink leaves its directory: {child}")
                if child.resolve() != self.output and child.suffix != ".pyc":
                    self._add_resource(child, PurePosixPath(destination, relative.as_posix()))

    def _path_expression(self, node, context, source):
        """Evaluate only path-building syntax, never calls in user code."""
        try:
            if isinstance(node, ast.Constant) and isinstance(node.value, (str, int)):
                return node.value
            if isinstance(node, ast.Name):
                return context.get(node.id)
            if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
                left = self._path_expression(node.left, context, source)
                right = self._path_expression(node.right, context, source)
                return left / right if isinstance(left, Path) and isinstance(right, str) else None
            if isinstance(node, ast.Attribute) and node.attr == "parent":
                value = self._path_expression(node.value, context, source)
                return value.parent if isinstance(value, Path) else None
            if (isinstance(node, ast.Subscript) and isinstance(node.value, ast.Attribute)
                    and node.value.attr == "parents"):
                value = self._path_expression(node.value.value, context, source)
                index = self._path_expression(node.slice, context, source)
                return value.parents[index] if isinstance(value, Path) and isinstance(index, int) else None
            if isinstance(node, ast.Call) and not node.keywords:
                if isinstance(node.func, ast.Name) and node.func.id in {"Path", "PurePath"} and len(node.args) == 1:
                    value = self._path_expression(node.args[0], context, source)
                    return Path(value) if isinstance(value, (str, Path)) else None
                if isinstance(node.func, ast.Attribute) and not node.args and node.func.attr in {"resolve", "absolute"}:
                    value = self._path_expression(node.func.value, context, source)
                    return value.resolve() if isinstance(value, Path) else None
                if isinstance(node.func, ast.Attribute):
                    function = ast.unparse(node.func)
                    args = [self._path_expression(arg, context, source) for arg in node.args]
                    if function in {"os.path.dirname", "os.path.abspath"} and len(args) == 1 and args[0] is not None:
                        value = Path(args[0])
                        return value.parent if function.endswith("dirname") else value.resolve()
                    if function == "os.path.join" and args and all(isinstance(a, (str, Path)) for a in args):
                        return Path(*args)
        except (OSError, ValueError, IndexError, TypeError):
            pass
        return None

    def _auto_resources(self, tree, source):
        context = {"__file__": source}
        candidates = []
        for statement in tree.body:
            if isinstance(statement, (ast.Assign, ast.AnnAssign)):
                targets = statement.targets if isinstance(statement, ast.Assign) else [statement.target]
                value = self._path_expression(statement.value, context, source)
                for target in targets:
                    if isinstance(target, ast.Name) and value is not None:
                        context[target.id] = value
                if isinstance(value, Path):
                    candidates.append(value)
        for node in ast.walk(tree):
            value = self._path_expression(node, context, source)
            if isinstance(value, Path):
                candidates.append(value)
            elif isinstance(node, ast.Constant) and isinstance(node.value, str):
                text = node.value
                if "\n" not in text and "\0" not in text and len(text) < 4096:
                    path = Path(text)
                    if path.suffix or "/" in text or "\\" in text:
                        candidates.extend((source.parent / path, self.root / path))
        for candidate in candidates:
            try:
                logical = Path(os.path.abspath(candidate))
                candidate = logical.resolve()
                relative = logical.relative_to(self.root)
                if (_under(candidate, self.root) and candidate.is_file() and candidate != self.output
                        and not any(part in _IGNORED or part.startswith(".") for part in relative.parts)):
                    self._add_resource(candidate, relative.as_posix())
            except (OSError, ValueError):
                continue

    def _imports(self, tree, package, source):
        dynamic_functions = {"__import__"}
        importlib_names, builtin_names = {"importlib"}, {"builtins"}
        for declaration in ast.walk(tree):
            if isinstance(declaration, ast.Import):
                for alias in declaration.names:
                    if alias.name == "importlib":
                        importlib_names.add(alias.asname or alias.name)
                    elif alias.name == "builtins":
                        builtin_names.add(alias.asname or alias.name)
            elif isinstance(declaration, ast.ImportFrom) and declaration.level == 0:
                for alias in declaration.names:
                    if declaration.module == "importlib" and alias.name == "import_module":
                        dynamic_functions.add(alias.asname or alias.name)
                    elif declaration.module == "builtins" and alias.name == "__import__":
                        dynamic_functions.add(alias.asname or alias.name)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    yield alias.name
            elif isinstance(node, ast.ImportFrom):
                if node.level:
                    try:
                        base = importlib.util.resolve_name("." * node.level + (node.module or ""), package)
                    except (ImportError, ValueError):
                        raise BundleError(f"Relative import has no valid package: {source}:{node.lineno}")
                else:
                    base = node.module or ""
                submodules, has_attributes = [], False
                for alias in node.names:
                    child = base + "." + alias.name
                    if alias.name != "*" and self.find(child) is not None:
                        submodules.append(child)
                    else:
                        has_attributes = True
                if has_attributes or not submodules:
                    yield base
                yield from submodules
            elif isinstance(node, ast.Call):
                recognized = isinstance(node.func, ast.Name) and node.func.id in dynamic_functions
                if isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name):
                    recognized = ((node.func.attr == "import_module" and node.func.value.id in importlib_names)
                                  or (node.func.attr == "__import__" and node.func.value.id in builtin_names))
                if not recognized:
                    continue
                argument = node.args[0] if node.args else next(
                    (k.value for k in node.keywords if k.arg == "name"), None)
                if isinstance(argument, ast.Constant) and isinstance(argument.value, str):
                    name = argument.value
                    if name.startswith("."):
                        explicit = node.args[1] if len(node.args) > 1 else next(
                            (k.value for k in node.keywords if k.arg == "package"), None)
                        parent = explicit.value if isinstance(explicit, ast.Constant) else package
                        try:
                            name = importlib.util.resolve_name(name, parent)
                        except (ImportError, ValueError):
                            raise BundleError(f"Unresolvable dynamic relative import: {source}:{node.lineno}")
                    yield name
                else:
                    self.dynamic_imports.append({"file": str(source), "line": node.lineno})

    def collect(self):
        entry_text, encoding, entry_tree = _read_source(self.entry)
        self.sources[self.entry_relative] = {"source": entry_text, "encoding": encoding,
                                            "path": self.entry, "tree": entry_tree}
        self.first_level.update(self._imports(entry_tree, self.entry_package, self.entry))
        if self.entry_package and self.entry.name != "__init__.py":
            self.add_module(self.entry_package)
        for name in sorted(self.first_level | self.includes):
            self.add_module(name, force=name in self.includes)
        self._pending.append((self.entry_name, self.entry, self.entry_relative,
                              self.entry.name == "__init__.py"))
        while self._pending:
            name, path, destination, package = self._pending.pop()
            key = (name, path)
            if key in self._scanned:
                continue
            self._scanned.add(key)
            data = self.sources[destination]
            parent = name if package else name.rpartition(".")[0]
            if path == self.entry:
                parent = self.entry_package
            for dependency in self._imports(data["tree"], parent, path):
                self.add_module(dependency)
            if self.auto_resources:
                self._auto_resources(data["tree"], path)
        for request in self.resources_requested:
            self.add_resource_request(request)
        self.dynamic_imports = sorted({(x["file"], x["line"]) for x in self.dynamic_imports})
        if self.strict and (self.dynamic_imports or any(
                reason.startswith("not found") for reason in self.external_modules.values())):
            raise BundleError("Strict build found unresolved imports; specify dynamic modules "
                              "with --include-module and provide missing runtime dependencies")
        return self

    def manifest(self):
        return {
            "format": "python_source_bundle_v1",
            "entry": self.entry_relative, "entry_module": self.entry_name,
            "entry_package": self.entry_package, "profile_first_level": self.profile,
            "working_directory": self.cwd, "first_level_dependencies": sorted(self.first_level),
            "modules": dict(sorted(self.modules.items())),
            "external_dependencies": dict(sorted(self.external_modules.items())),
            "unresolved_dynamic_imports": [{"file": f, "line": n} for f, n in self.dynamic_imports],
            "path_roots": dict(sorted(self.path_roots.items())),
            "sources": {key: {"sha256": hashlib.sha256(value["path"].read_bytes()).hexdigest(),
                              "encoding": value["encoding"]} for key, value in sorted(self.sources.items())},
            "resources": {key: {"sha256": hashlib.sha256(value).hexdigest(), "bytes": len(value)}
                          for key, value in sorted(self.resources.items())},
        }

    def write(self):
        for value in self.sources.values():
            if value["path"] == self.output:
                raise BundleError("Output would overwrite an input source")
        manifest = self.manifest()
        chunks = ["#!/usr/bin/env python3\n# -*- coding: utf-8 -*-\n",
                  "# Generated by tools/bundle_python.py; Python sources remain readable below.\n",
                  "BUNDLE_MANIFEST = " + pprint.pformat(manifest, width=100, sort_dicts=True) + "\n",
                  "_BUNDLE_SOURCES = {\n"]
        for name, source in sorted(self.sources.items()):
            lines = source["source"].splitlines(keepends=True)
            literal = "(\n" + "".join("        " + repr(line) + "\n" for line in lines) + "    )" if lines else "''"
            chunks.append(f"    {name!r}: ({source['encoding']!r}, {literal}),\n")
        chunks.append("}\n_BUNDLE_RESOURCES = " + pprint.pformat(
            {name: base64.b64encode(value).decode("ascii") for name, value in sorted(self.resources.items())},
            width=100, sort_dicts=True) + "\n")
        chunks.append(_RUNTIME)
        data = "".join(chunks)
        compile(data, str(self.output), "exec")  # Syntax check, never run the entry.
        self.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile("w", encoding="utf-8", newline="\n",
                                             dir=self.output.parent, delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(data)
            os.replace(temporary, self.output)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
        return manifest


_RUNTIME = r'''

# ---- Single-file execution support; uses only the Python standard library. ----
import ast as _sf_ast
import atexit as _sf_atexit
import base64 as _sf_base64
import importlib as _sf_importlib
import importlib.machinery as _sf_machinery
import importlib.util as _sf_util
import os as _sf_os
from pathlib import Path as _sf_Path
import sys as _sf_sys
import tempfile as _sf_tempfile
import threading as _sf_threading
import time as _sf_time
import types as _sf_types


class _SFProfile:
    def __init__(self, dependencies, mode):
        self.dependencies = sorted(dependencies, key=len, reverse=True)
        self.mode = mode
        self.stats = {name: dict(calls=0, total=0.0, self=0.0, imports=0.0) for name in dependencies}
        self.local = _sf_threading.local()
        self.lock = _sf_threading.RLock()
        self.c_cache = {}
        self.enabled = True
        self.previous = _sf_sys.getprofile()
        self.previous_thread = _sf_threading.getprofile()

    def group(self, module):
        if not isinstance(module, str):
            return None
        return next((name for name in self.dependencies
                     if module == name or module.startswith(name + ".")), None)

    def state(self):
        if not hasattr(self.local, "stack"):
            self.local.stack, self.local.active, self.local.loading = [], {}, {}
            self.local.overhead, self.local.last = 0.0, _sf_time.perf_counter()
        return self.local

    def clock(self):
        state = self.state()
        return _sf_time.perf_counter() - state.overhead

    def c_group(self, function):
        owner = getattr(function, "__self__", None)
        owner_name = (getattr(owner, "__name__", None) if isinstance(owner, _sf_types.ModuleType)
                      else type(owner).__module__ + "." + type(owner).__qualname__)
        key = (getattr(function, "__module__", None),
               getattr(function, "__qualname__", getattr(function, "__name__", type(function).__qualname__)),
               owner_name)
        if key in self.c_cache:
            return self.c_cache[key]
        group = self.group(getattr(function, "__module__", None))
        if group is None and owner is not None and not isinstance(owner, _sf_types.ModuleType):
            group = self.group(type(owner).__module__)
        if group is None:
            # os.getcwd is a native nt/posix function re-exported by os.
            for dependency in self.dependencies:
                module = _sf_sys.modules.get(dependency)
                if module is not None and any(value is function for value in getattr(module, "__dict__", {}).values()):
                    group = dependency
                    break
        # Structural keys avoid keeping every short-lived bound method alive.
        self.c_cache[key] = group
        return group

    def __call__(self, frame, event, argument):
        started = _sf_time.perf_counter()
        state = self.state()
        now = started - state.overhead
        try:
            with self.lock:
                if state.stack and state.stack[-1][3] is not None:
                    self.stats[state.stack[-1][3]]["self"] += max(0.0, now-state.last)
                state.last = now
                if event in ("call", "c_call"):
                    if event == "call":
                        group = None if frame.f_code.co_name == "<module>" else self.group(
                            frame.f_globals.get("__name__"))
                        kind = "py"
                    else:
                        group = self.c_group(argument)
                        kind = "c"
                    inherited = state.stack[-1][3] if state.stack else None
                    outer = None
                    if group is not None:
                        self.stats[group]["calls"] += 1
                        count = state.active.get(group, 0)
                        outer = now if count == 0 else None
                        state.active[group] = count+1
                    state.stack.append((kind, id(frame), group, group or inherited, outer))
                elif event in ("return", "c_return", "c_exception"):
                    kind = "py" if event == "return" else "c"
                    if state.stack and state.stack[-1][:2] == (kind, id(frame)):
                        _, _, group, _, outer = state.stack.pop()
                        if group is not None:
                            state.active[group] -= 1
                            if outer is not None:
                                self.stats[group]["total"] += max(0.0, now-outer)
        finally:
            state.overhead += _sf_time.perf_counter()-started

    def load_start(self, module):
        if self.mode == "calls" or not self.enabled:
            return None
        group = self.group(module)
        if group is None:
            return None
        state = self.state()
        count, now = state.loading.get(group, 0), self.clock()
        state.loading[group] = count+1
        return group, now if count == 0 else None

    def load_end(self, token):
        if token is not None:
            group, started = token
            state = self.state()
            state.loading[group] -= 1
            if started is not None:
                with self.lock:
                    self.stats[group]["imports"] += max(0.0, self.clock()-started)

    def start(self):
        if self.mode != "imports":
            _sf_sys.setprofile(self)
            _sf_threading.setprofile(self)
        _sf_atexit.register(self.report)

    def report(self):
        if not self.enabled:
            return
        self.enabled = False
        if self.mode != "imports":
            _sf_sys.setprofile(self.previous)
            _sf_threading.setprofile(self.previous_thread)
        stream = _sf_sys.stderr
        if stream is None:
            return
        print("\n[bundle-profile] First-level dependencies; wall time in milliseconds", file=stream)
        print(f"{'dependency':<44} {'calls':>8} {'total_ms':>12} {'self_ms':>12} {'import_ms':>12}", file=stream)
        for name, row in sorted(self.stats.items(), key=lambda item: (-item[1]["total"], item[0])):
            print(f"{name:<44} {row['calls']:8d} {row['total']*1000:12.3f} "
                  f"{row['self']*1000:12.3f} {row['imports']*1000:12.3f}", file=stream)
        print("total_ms includes subcalls; self_ms assigns intervals to the innermost dependency; "
              "import_ms measures module loading. Columns are not additive.", file=stream)


class _SFRelocate(_sf_ast.NodeTransformer):
    def __init__(self, roots):
        self.roots = sorted(roots.items(), key=lambda item: len(item[0]), reverse=True)

    def visit_Expr(self, node):
        # Do not turn a module/class/function docstring into an expression.
        if isinstance(node.value, _sf_ast.Constant) and isinstance(node.value.value, str):
            return node
        return self.generic_visit(node)

    def visit_JoinedStr(self, node):
        # Static f-string pieces are not complete path literals.
        for value in node.values:
            if isinstance(value, _sf_ast.FormattedValue):
                self.visit(value)
        return node

    def visit_Constant(self, node):
        if isinstance(node.value, str):
            value = node.value.replace("\\", "/")
            for original, destination in self.roots:
                normalized = original.replace("\\", "/").rstrip("/")
                windows = len(normalized) > 1 and normalized[1] == ":"
                left, right = (value.casefold(), normalized.casefold()) if windows else (value, normalized)
                if left == right or left.startswith(right + "/"):
                    suffix = value[len(normalized):].lstrip("/")
                    relative = str(_sf_Path(destination) / suffix)
                    return _sf_ast.copy_location(_sf_ast.Call(
                        func=_sf_ast.Name(id="__singlefile_resource_path__", ctx=_sf_ast.Load()),
                        args=[_sf_ast.Constant(value=relative)], keywords=[]), node)
        return node


class _SFSourceLoader(_sf_machinery.SourceFileLoader):
    def __init__(self, name, path, roots, resource_path):
        super().__init__(name, path)
        self.roots, self.resource_path = roots, resource_path

    def get_code(self, fullname):
        path = self.get_filename(fullname)
        tree = _sf_ast.parse(self.get_data(path), filename=path)
        tree = _SFRelocate(self.roots).visit(tree)
        _sf_ast.fix_missing_locations(tree)
        return compile(tree, path, "exec", dont_inherit=True)

    def exec_module(self, module):
        module.__dict__["__singlefile_resource_path__"] = self.resource_path
        super().exec_module(module)


class _SFTimedLoader:
    def __init__(self, original, name, profiler):
        self.original, self.name, self.profiler = original, name, profiler

    def __getattr__(self, name):
        return getattr(self.original, name)

    def create_module(self, spec):
        token = self.profiler.load_start(self.name)
        try:
            method = getattr(self.original, "create_module", None)
            return method(spec) if method is not None else None
        finally:
            self.profiler.load_end(token)

    def exec_module(self, module):
        token = self.profiler.load_start(self.name)
        try:
            self.original.exec_module(module)
        finally:
            self.profiler.load_end(token)


class _SFFinder:
    def __init__(self, root, source_paths, roots, resource_path, profiler):
        self.root, self.source_paths, self.roots = root, source_paths, roots
        self.resource_path, self.profiler = resource_path, profiler

    def find_spec(self, fullname, path=None, target=None):
        for finder in tuple(_sf_sys.meta_path):
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(fullname, path, target)
            if spec is None:
                continue
            if spec.origin and spec.origin not in ("built-in", "frozen"):
                origin = str(_sf_Path(spec.origin).resolve())
                if origin in self.source_paths:
                    spec.loader = _SFSourceLoader(fullname, origin, self.roots, self.resource_path)
            if (self.profiler is not None and self.profiler.group(fullname) is not None
                    and spec.loader is not None and hasattr(spec.loader, "exec_module")):
                spec.loader = _SFTimedLoader(spec.loader, fullname, self.profiler)
            return spec
        return None


def _singlefile_run():
    temporary = _sf_tempfile.TemporaryDirectory(prefix="python-source-bundle-")
    # Keep sources alive for worker threads and entry atexit handlers. Cleanup
    # is registered first, so profiling and user callbacks execute before it.
    _sf_atexit.register(temporary.cleanup)
    root = _sf_Path(temporary.name).resolve()

    def resource_path(relative):
        path = (root / relative).resolve()
        if not path.is_relative_to(root):
            raise RuntimeError("Relocated path leaves the bundled project")
        return str(path)

    source_paths = set()
    for relative, (encoding, source) in _BUNDLE_SOURCES.items():
        path = _sf_Path(resource_path(relative))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(source.encode(encoding))
        source_paths.add(str(path))
    for relative, encoded in _BUNDLE_RESOURCES.items():
        path = _sf_Path(resource_path(relative))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(_sf_base64.b64decode(encoded))
    entry = _sf_Path(resource_path(BUNDLE_MANIFEST["entry"]))
    paths = [str(entry.parent), str(root), str(root / "_vendor")]
    _sf_sys.path[:0] = list(dict.fromkeys(paths))
    _sf_sys.dont_write_bytecode = True
    mode = BUNDLE_MANIFEST["profile_first_level"]
    profiler = None if mode == "off" else _SFProfile(BUNDLE_MANIFEST["first_level_dependencies"], mode)
    finder = _SFFinder(root, source_paths, BUNDLE_MANIFEST["path_roots"], resource_path, profiler)
    _sf_sys.meta_path.insert(0, finder)
    if profiler is not None:
        profiler.start()
    cwd = BUNDLE_MANIFEST["working_directory"]
    if cwd != "caller":
        _sf_os.chdir(root if cwd == "project" else entry.parent)
    package = BUNDLE_MANIFEST["entry_package"]
    if package and entry.name != "__init__.py":
        _sf_importlib.import_module(package)
    module = _sf_types.ModuleType("__main__")
    module.__file__, module.__package__ = str(entry), package
    module.__spec__ = (_sf_util.spec_from_file_location(BUNDLE_MANIFEST["entry_module"], entry)
                       if package else None)
    loader = _SFSourceLoader(BUNDLE_MANIFEST["entry_module"], str(entry),
                             BUNDLE_MANIFEST["path_roots"], resource_path)
    module.__loader__ = loader
    if entry.name == "__init__.py":
        module.__path__ = [str(entry.parent)]
    module.__dict__["__singlefile_resource_path__"] = resource_path
    _sf_sys.modules["__main__"] = module
    _sf_sys.argv[0] = str(entry)
    exec(loader.get_code(BUNDLE_MANIFEST["entry_module"]), module.__dict__)


if __name__ == "__main__":
    _singlefile_run()
'''


def bundle(entry, output, *, root=None, **options):
    entry = Path(entry).resolve()
    builder = Builder(entry, root or _default_root(entry), output, **options).collect()
    return builder.write()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("entry", type=Path, help="Entry Python file")
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--root", type=Path, help="Project root; inferred from .git/setup/pyproject otherwise")
    parser.add_argument("--search-path", type=Path, action="append", default=[])
    parser.add_argument("--include-module", action="append", default=[],
                        help="Embed an extra/dynamically imported module, or an entire pure-Python package")
    parser.add_argument("--external", action="append", default=[], help="Keep a module/package external")
    parser.add_argument("--resource", action="append", default=[],
                        help="Embed file/directory; outside --root use SOURCE=RELATIVE_DESTINATION")
    parser.add_argument("--no-auto-resources", action="store_true")
    parser.add_argument("--cwd", choices=("caller", "project", "entry"), default="caller")
    parser.add_argument("--profile-first-level", "--profile", nargs="?", const="all", default="off",
                        choices=("off", "all", "calls", "imports"))
    parser.add_argument("--strict", action="store_true", help="Reject unresolved dynamic/missing imports")
    parser.add_argument("--print-manifest", action="store_true")
    args = parser.parse_args()
    try:
        manifest = bundle(args.entry, args.output, root=args.root, includes=args.include_module,
                          externals=args.external, resources=args.resource,
                          search_paths=args.search_path, cwd=args.cwd,
                          profile=args.profile_first_level, strict=args.strict,
                          auto_resources=not args.no_auto_resources)
    except (BundleError, OSError, SyntaxError) as error:
        parser.error(str(error))
    summary = manifest if args.print_manifest else {
        "output": str(args.output.resolve()), "source_files": len(manifest["sources"]),
        "resources": len(manifest["resources"]), "profile": manifest["profile_first_level"],
        "first_level_dependencies": manifest["first_level_dependencies"],
        "external_dependencies": sorted(manifest["external_dependencies"]),
        "unresolved_dynamic_imports": manifest["unresolved_dynamic_imports"]}
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
