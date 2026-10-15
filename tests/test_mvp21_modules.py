"""Normal imports, real worker processes and relocated single-file delivery."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mvp21_policy.lattice import ShadowLatticePlanner, ShadowSettings
from mvp21_policy.transport import ShadowWorker
from mvp21_policy.worker import worker_source_sha256
from tools.bundle_python import bundle


def job():
    return dict(settings=vars(ShadowSettings(enabled=True, sample_count=12)),
        stamp_s=100., generation=4, job_id=12, preview_m=.35, submitted_monotonic_s=time.monotonic(),
        state=dict(speed_mps=.2, speed_valid=True),
        road=dict(centerline_xy=[(0.,0.),(.5,0.),(1.,0.)],
            left_boundary_xy=[(0.,.5),(1.,.5)], right_boundary_xy=[(0.,-.5),(1.,-.5)],
            boundary_source=dict(left='observed', right='observed'), lane_width_m=1.))


def child_environment():
    environment = dict(os.environ)
    environment.pop('PYTHONPATH', None)
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    return environment


class MVP21ModuleChecks(unittest.TestCase):
    def test_pure_imports_need_no_ros_and_start_no_threads_or_processes(self):
        code = """
import importlib.abc
import subprocess
import sys
import threading
sys.path.insert(0, sys.argv[1])
class RejectROS(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'rclpy','dream_interfaces','tf2_ros','sensor_msgs'}:
            raise AssertionError('pure import requested ROS: '+fullname)
def reject(*args, **kwargs):
    raise AssertionError('import started background execution')
sys.meta_path.insert(0, RejectROS())
subprocess.Popen = reject
threading.Thread.start = reject
from mvp21_policy import api
from offline.mvp21_sim.source import api as simulation_api
assert simulation_api() is api
assert api.EstimationPipeline.__module__ == 'mvp21_policy.estimation'
assert api.PolicyController.__module__ == 'mvp21_policy.control'
assert api.ShadowLatticePlanner.__module__ == 'mvp21_policy.lattice'
assert not hasattr(api, 'PolicyNode')
print('isolated')
"""
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable,'-B','-c',code,str(ROOT)],cwd=directory,
                env=child_environment(),capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(result.stdout.strip(),'isolated')

    def test_entry_worker_protocol_recovers_from_bad_json_without_ros(self):
        record = job()
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable,'-B',str(ROOT/'scripts/mvp21_policy_node.py'),'--shadow-worker'],
                input='invalid json\n'+json.dumps(record)+'\n',cwd=directory,env=child_environment(),
                capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)
        bad, good = map(json.loads,result.stdout.splitlines())
        self.assertEqual(bad['status'],'WORKER_ERROR')
        self.assertEqual(good['status'],'VALID')
        self.assertEqual((good['generation'],good['job_id']),(4,12))
        self.assertTrue(good['shadow_only'])
        self.assertEqual(good['source_sha256'],worker_source_sha256())
        direct = ShadowLatticePlanner(ShadowSettings(**record['settings'])).plan(record)
        self.assertEqual(good['selected']['id'],direct['selected']['id'])
        self.assertEqual(good['selected']['path'],json.loads(json.dumps(direct['selected']['path'])))
        # CMake installs the package next to the separate executable; this is
        # different from the source tree, where scripts/ is below the root.
        with tempfile.TemporaryDirectory() as directory:
            install = Path(directory)/'lib/ai4r_policy'
            install.mkdir(parents=True)
            shutil.copy2(ROOT/'scripts/mvp21_policy_node.py',install/'mvp21_policy_node.py')
            shutil.copytree(ROOT/'mvp21_policy',install/'mvp21_policy',ignore=shutil.ignore_patterns('__pycache__'))
            result = subprocess.run([sys.executable,'-B',str(install/'mvp21_policy_node.py'),'--shadow-worker'],
                input=json.dumps(record)+'\n',cwd=directory,env=child_environment(),
                capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(json.loads(result.stdout)['status'],'VALID')

    def test_real_transport_launches_module_worker_and_closes(self):
        worker = ShadowWorker()
        try:
            worker.submit(job())
            deadline = time.monotonic()+10
            result = None
            while result is None and time.monotonic()<deadline:
                result = worker.poll()
                if result is None:
                    time.sleep(.01)
            self.assertIsNotNone(result)
            self.assertEqual(result['status'],'VALID',result)
            self.assertEqual(result['generation'],4)
        finally:
            worker.close()
            worker.thread.join(timeout=3)
        self.assertFalse(worker.thread.is_alive())

    def test_bundled_entry_and_child_transport_work_outside_repository(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'.verification',prefix='mvp21-bundle-') as sources, \
                tempfile.TemporaryDirectory(prefix='mvp21-relocated-') as outputs:
            sources, outputs = Path(sources), Path(outputs)
            entry = outputs/'policy_single.py'
            manifest = bundle(ROOT/'scripts/mvp21_policy_node.py',entry,root=ROOT)
            self.assertIn('mvp21_policy/node.py',manifest['sources'])
            result = subprocess.run([sys.executable,'-B',str(entry),'--shadow-worker'],
                input=json.dumps(job())+'\n',cwd=outputs,env=child_environment(),
                capture_output=True,text=True,timeout=15)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(json.loads(result.stdout)['status'],'VALID')
            probe = sources/'transport_probe.py'
            probe.write_text('''import json, sys, time
from mvp21_policy.transport import ShadowWorker
worker = ShadowWorker()
try:
    worker.submit(json.loads(sys.stdin.readline()))
    deadline = time.monotonic()+10
    result = None
    while result is None and time.monotonic()<deadline:
        result = worker.poll()
        if result is None: time.sleep(.01)
    if result is None: raise RuntimeError('worker timeout')
    print(json.dumps(result))
finally:
    worker.close()
    worker.thread.join(timeout=3)
''',encoding='utf-8')
            bundled_probe = outputs/'transport_single.py'
            bundle(probe,bundled_probe,root=ROOT)
            result = subprocess.run([sys.executable,'-B',str(bundled_probe)],
                input=json.dumps(job())+'\n',cwd=outputs,env=child_environment(),
                capture_output=True,text=True,timeout=15)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(json.loads(result.stdout)['status'],'VALID')


if __name__ == '__main__':
    unittest.main()
