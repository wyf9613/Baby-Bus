import importlib.util
import json
import time
from pathlib import Path
source=Path.home()/'ai4r_student_workspace/src/ai4r_policy/scripts/policy_node.py'
spec=importlib.util.spec_from_file_location('policy_probe_source',source)
module=importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name]=module
spec.loader.exec_module(module)
log=(Path.home()/'timing-fix-evidence/policy-timing.jsonl').open('a',buffering=1)
original=module.PolicyNode.calculate_policy_actions
last=None

def timed(self,*args,**kwargs):
    global last
    start=time.monotonic()
    previous=last
    last=start
    try:
        return original(self,*args,**kwargs)
    finally:
        estimates=getattr(self,'estimation_output',{})
        planning=getattr(self,'planning_output',{})
        state=estimates.get('state',{})
        road=estimates.get('road',{})
        log.write(json.dumps({'time':time.time(),'duration_ms':(time.monotonic()-start)*1000,'interval_ms':None if previous is None else (start-previous)*1000,'state':state,'road_curvature':road.get('local_curvature_1pm'),'planning_diagnostics':getattr(self,'planning_diagnostics',{}),'road_status':road.get('status'),'road_valid':road.get('valid'),'planning_reason':planning.get('reason'),'source_ages':planning.get('source_ages_s'),'valid_for_s':planning.get('valid_for_s')},default=str)+'\n')
module.PolicyNode.calculate_policy_actions=timed
module.main()
