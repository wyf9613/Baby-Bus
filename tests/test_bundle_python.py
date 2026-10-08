"""Exercise generated files in fresh processes after relocating their sources."""
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools/bundle_python.py"
spec = importlib.util.spec_from_file_location("_test_bundle_python_builder", TOOL)
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


class BundleBehavior(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="bundle-python-test-")
        self.base = Path(self.directory.name).resolve()
        self.project = self.base / "source project"
        self.project.mkdir()
        self.output = self.base / "relocated output" / "program.py"
        self.caller = self.base / "unrelated cwd"
        self.caller.mkdir()

    def tearDown(self):
        self.directory.cleanup()

    def write(self, name, text):
        path = self.project / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        return path

    def run_output(self, *arguments):
        environment = dict(os.environ)
        environment.pop("PYTHONPATH", None)
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        return subprocess.run([sys.executable, str(self.output), *arguments],
                              cwd=self.caller, env=environment,
                              capture_output=True, text=True, timeout=20)

    def hide_source(self):
        # Both targets are explicitly inside this test-owned temporary directory.
        assert self.project.parent == self.base
        self.project.rename(self.base / "original source no longer on path")

    def profile_row(self, result, module):
        match = re.search(r"^" + re.escape(module) + r"\s+(\d+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)$",
                          result.stderr, re.MULTILINE)
        self.assertIsNotNone(match, result.stderr)
        return int(match[1]), float(match[2]), float(match[3]), float(match[4])

    def test_package_relative_imports_circular_modules_and_resources_survive_relocation(self):
        self.write("pkg/__init__.py", "from .service import Worker\n")
        self.write("pkg/a.py", "from . import b\nVALUE = 2\ndef value(): return b.VALUE\n")
        self.write("pkg/b.py", "from . import a\nVALUE = 3\ndef value(): return a.VALUE\n")
        self.write("pkg/data.txt", "PACKAGED_DATA\n")
        self.write("assets/message.json", '{"value": 7}\n')
        original = str(self.project / "assets/message.json")
        self.write("pkg/service.py", f"""
from pathlib import Path
from importlib.resources import files
from . import a, b
import json
class Worker:
    def compute(self):
        project = Path(__file__).resolve().parents[1]
        automatic = json.loads((project / 'assets' / 'message.json').read_text())
        absolute = json.loads(Path({original!r}).read_text())
        return [a.value(), b.value(), automatic['value'], absolute['value'],
                files('pkg').joinpath('data.txt').read_text().strip()]
""")
        entry = self.write("pkg/main.py", "from .service import Worker\nimport json, sys\n"
                           "print(json.dumps(Worker().compute() + sys.argv[1:]))\n")
        manifest = builder.bundle(entry, self.output, root=self.project)
        self.assertIn("pkg.service", manifest["first_level_dependencies"])
        self.assertIn("assets/message.json", manifest["resources"])
        self.assertIn("pkg/data.txt", manifest["resources"])
        self.hide_source()
        result = self.run_output("forwarded argument")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), [3, 2, 7, 7, "PACKAGED_DATA", "forwarded argument"])
        self.assertNotIn("bundle-profile", result.stderr)

    def test_plain_entry_sibling_import_and_explicit_cwd_for_bare_relative_paths(self):
        self.write("scripts/helper.py", "VALUE = 11\n")
        self.write("scripts/payload.txt", "BARE_RELATIVE_DATA")
        entry = self.write("scripts/main.py", "import helper\n"
                           "print(helper.VALUE, open('payload.txt').read())\n")
        builder.bundle(entry, self.output, root=self.project, cwd="entry")
        self.hide_source()
        result = self.run_output()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "11 BARE_RELATIVE_DATA")

    def test_build_never_executes_entry_or_package_initializers(self):
        marker = self.base / "must-not-be-written"
        self.write("pkg/__init__.py", f"from pathlib import Path\nPath({str(marker)!r}).write_text('BAD')\n"
                   "class Worker: pass\n")
        entry = self.write("main.py", f"from pkg import Worker\n"
                           f"open({str(marker)!r}, 'w').write('BAD')\n")
        builder.bundle(entry, self.output, root=self.project)
        self.assertFalse(marker.exists())

    def test_profiling_includes_methods_subcalls_and_new_threads(self):
        self.write("worker.py", """
import time
class Worker:
    def run(self): return self.inner()
    def inner(self):
        time.sleep(0.01)
        return 5
""")
        entry = self.write("main.py", "from worker import Worker\nimport threading\n"
                           "w = Worker()\nprint(w.run())\n"
                           "t = threading.Thread(target=w.run); t.start(); t.join()\n")
        builder.bundle(entry, self.output, root=self.project, profile="all")
        self.hide_source()
        result = self.run_output()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "5")
        calls, total, own, importing = self.profile_row(result, "worker")
        self.assertGreaterEqual(calls, 4)
        self.assertGreater(total, 10)
        self.assertGreater(own, 10)
        self.assertGreater(importing, 0)

    def test_profile_survives_system_exit_and_unhandled_exception(self):
        self.write("worker.py", "import time\ndef execute(): time.sleep(0.005)\n")
        entry = self.write("main.py", "import worker, sys\nworker.execute()\nsys.exit(7)\n")
        builder.bundle(entry, self.output, root=self.project, profile="all")
        result = self.run_output()
        self.assertEqual(result.returncode, 7)
        self.assertGreater(self.profile_row(result, "worker")[1], 1)
        entry.write_text("import worker\nworker.execute()\nraise ValueError('EXPECTED_ERROR')\n", encoding="utf-8")
        builder.bundle(entry, self.output, root=self.project, profile="all")
        result = self.run_output()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("EXPECTED_ERROR", result.stderr)
        self.assertGreater(self.profile_row(result, "worker")[1], 1)

    def test_import_only_mode_and_native_function_reexport_attribution(self):
        self.write("worker.py", "import time\ntime.sleep(0.005)\ndef run(): return 2\n")
        entry = self.write("main.py", "import worker, os\nprint(worker.run(), bool(os.getcwd()))\n")
        builder.bundle(entry, self.output, root=self.project, profile="imports")
        result = self.run_output()
        self.assertEqual(result.returncode, 0, result.stderr)
        row = self.profile_row(result, "worker")
        self.assertEqual(row[:3], (0, 0.0, 0.0))
        self.assertGreater(row[3], 1)
        builder.bundle(entry, self.output, root=self.project, profile="calls")
        result = self.run_output()
        self.assertGreaterEqual(self.profile_row(result, "os")[0], 1)
        self.assertEqual(self.profile_row(result, "worker")[3], 0.0)

    def test_literal_and_explicit_computed_dynamic_imports(self):
        self.write("plugins/__init__.py", "")
        self.write("plugins/one.py", "VALUE = 'DYNAMIC_MODULE'\n")
        self.write("plugins/data.txt", "FULL_PACKAGE_RESOURCE")
        entry = self.write("main.py", "import importlib, sys\n"
                           "one = importlib.import_module('plugins.one')\n"
                           "two = importlib.import_module('plugins.' + sys.argv[1])\nprint(one.VALUE, two.VALUE)\n")
        manifest = builder.bundle(entry, self.output, root=self.project, includes=["plugins"])
        self.assertTrue(manifest["unresolved_dynamic_imports"])
        self.assertIn("plugins/data.txt", manifest["resources"])
        self.hide_source()
        result = self.run_output("one")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "DYNAMIC_MODULE DYNAMIC_MODULE")

    def test_selected_external_pure_python_package_and_package_data(self):
        vendor = self.base / "external library"
        package = vendor / "examplelib"
        package.mkdir(parents=True)
        (package / "__init__.py").write_text("from .api import read\n", encoding="utf-8")
        (package / "api.py").write_text("from pathlib import Path\n"
                                      "def read(): return (Path(__file__).parent / 'binary.dat').read_bytes().hex()\n",
                                      encoding="utf-8")
        (package / "binary.dat").write_bytes(b"\x00\xff\x01")
        entry = self.write("main.py", "import examplelib\nprint(examplelib.read())\n")
        manifest = builder.bundle(entry, self.output, root=self.project, includes=["examplelib"],
                                  search_paths=[vendor])
        self.assertIn("_vendor/examplelib/binary.dat", manifest["resources"])
        assert vendor.parent == self.base
        vendor.rename(self.base / "hidden external library")
        self.hide_source()
        result = self.run_output()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "00ff01")

    def test_explicit_binary_resource_and_external_mapping(self):
        external = self.base / "outside.bin"
        external.write_bytes(bytes(range(256)))
        entry = self.write("main.py", "from pathlib import Path\n"
                           "print(len((Path(__file__).parent / 'data/blob.bin').read_bytes()))\n")
        builder.bundle(entry, self.output, root=self.project,
                       resources=[str(external) + "=data/blob.bin"], auto_resources=False)
        external.unlink()
        self.hide_source()
        result = self.run_output()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "256")

    def test_source_encoding_and_line_endings_are_preserved(self):
        entry = self.project / "main.py"
        entry.write_bytes("# -*- coding: latin-1 -*-\r\nprint('caf\u00e9')\r\n".encode("latin-1"))
        builder.bundle(entry, self.output, root=self.project)
        self.hide_source()
        result = self.run_output()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "caf\u00e9")

    def test_rejects_destructive_outputs_unsafe_resources_and_unresolved_strict_builds(self):
        entry = self.write("main.py", "import importlib\nimportlib.import_module(input())\n")
        original = entry.read_bytes()
        with self.assertRaises(builder.BundleError):
            builder.bundle(entry, entry, root=self.project)
        self.assertEqual(entry.read_bytes(), original)
        with self.assertRaises(builder.BundleError):
            builder.bundle(entry, self.output, root=self.project, strict=True)
        resource = self.write("data.bin", "DATA")
        with self.assertRaises(builder.BundleError):
            builder.bundle(entry, self.output, root=self.project,
                           resources=[str(resource) + "=../escape"])

    def test_cli_profile_flag_and_deterministic_output(self):
        self.write("helper.py", "def value(): return 3\n")
        entry = self.write("main.py", "import helper\nprint(helper.value())\n")
        command = [sys.executable, str(TOOL), str(entry), "--root", str(self.project),
                   "--output", str(self.output), "--profile-first-level"]
        first = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertEqual(json.loads(first.stdout)["profile"], "all")
        data = self.output.read_bytes()
        second = subprocess.run(command, capture_output=True, text=True, timeout=20)
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertEqual(self.output.read_bytes(), data)
        self.hide_source()
        result = self.run_output()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "3")
        self.assertGreaterEqual(self.profile_row(result, "helper")[0], 1)

    def test_profiler_does_not_retain_short_lived_bound_methods(self):
        entry = self.write("main.py", "print('OK')\n")
        builder.bundle(entry, self.output, root=self.project, profile="all")
        specification = importlib.util.spec_from_file_location("_memory_test_bundle", self.output)
        generated = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(generated)
        profile = generated._SFProfile(["os"], "calls")
        for _ in range(10000):
            profile.c_group([].append)
        # A long-running policy creates many bound method objects; storage must
        # scale with callable kinds rather than the number of iterations.
        self.assertLess(len(profile.c_cache), 100)

    @unittest.skipUnless(importlib.util.find_spec("numpy"), "Optional native-library integration needs NumPy")
    def test_profile_can_load_an_external_native_package(self):
        entry = self.write("main.py", "import numpy\nprint(int(numpy.arange(4).sum()))\n")
        builder.bundle(entry, self.output, root=self.project, profile="all")
        self.hide_source()
        result = self.run_output()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "6")
        self.assertGreater(self.profile_row(result, "numpy")[0], 0)

    def test_resource_symlink_keeps_the_referenced_filename(self):
        asset = self.write("assets/real.txt", "LINK_DATA")
        alias = self.project / "alias.txt"
        try:
            alias.symlink_to(asset)
        except OSError as error:
            self.skipTest(f"This host cannot create test symlinks: {error}")
        entry = self.write("main.py", "from pathlib import Path\n"
                           "print((Path(__file__).parent / 'alias.txt').read_text())\n")
        manifest = builder.bundle(entry, self.output, root=self.project)
        self.assertIn("alias.txt", manifest["resources"])
        self.hide_source()
        result = self.run_output()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "LINK_DATA")

    def test_dynamic_import_aliases_are_collected_without_executing_them(self):
        self.write("one.py", "VALUE = 4\n")
        self.write("two.py", "VALUE = 5\n")
        self.write("three.py", "VALUE = 6\n")
        entry = self.write("main.py", "import importlib as il\n"
                           "from importlib import import_module as load\n"
                           "from builtins import __import__ as native_load\n"
                           "print(il.import_module('one').VALUE, load('two').VALUE, native_load('three').VALUE)\n")
        manifest = builder.bundle(entry, self.output, root=self.project)
        self.assertTrue({"one", "two", "three"}.issubset(manifest["first_level_dependencies"]))
        self.hide_source()
        result = self.run_output()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "4 5 6")

    def test_selected_external_subpackage_includes_its_parent_initializer(self):
        vendor = self.base / "external subpackage library"
        package = vendor / "examplelib"
        child = package / "feature"
        child.mkdir(parents=True)
        (package / "__init__.py").write_text("VALUE = 12\n", encoding="utf-8")
        (child / "__init__.py").write_text("from examplelib import VALUE\n", encoding="utf-8")
        entry = self.write("main.py", "import examplelib\nimport examplelib.feature\n"
                           "print(examplelib.VALUE, examplelib.feature.VALUE)\n")
        manifest = builder.bundle(entry, self.output, root=self.project,
                                  includes=["examplelib.feature"], search_paths=[vendor])
        self.assertIn("_vendor/examplelib/__init__.py", manifest["sources"])
        self.assertNotIn("examplelib", manifest["external_dependencies"])
        assert vendor.parent == self.base
        vendor.rename(self.base / "hidden subpackage library")
        self.hide_source()
        result = self.run_output()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "12 12")


if __name__ == "__main__":
    unittest.main()
