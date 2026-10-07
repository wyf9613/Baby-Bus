import cProfile
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

from dream_command import runtime_runner
baseline = Path(sys.argv[1])
profile_path = Path(sys.argv[2])
module_name = 'dream_command._profile_baseline'
spec = importlib.util.spec_from_file_location(module_name, baseline)
module = importlib.util.module_from_spec(spec)
sys.modules[module_name] = module
spec.loader.exec_module(module)
result = {}
for label, cls in [('before', module.LinuxProcessObserver), ('after', runtime_runner.LinuxProcessObserver)]:
    observer = cls()
    observer.group_members(os.getpgrp())
    start = time.monotonic()
    for _ in range(30):
        observer.group_members(os.getpgrp())
    result[label+'_scan_seconds'] = (time.monotonic()-start)/30
    profiler = cProfile.Profile()
    profiler.enable()
    for _ in range(30):
        observer.group_members(os.getpgrp())
    profiler.disable()
    profiler.dump_stats(str(profile_path / (label+'-scan.pstats')))
result['speedup'] = result['before_scan_seconds']/result['after_scan_seconds']
(profile_path / 'scan-benchmark.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
