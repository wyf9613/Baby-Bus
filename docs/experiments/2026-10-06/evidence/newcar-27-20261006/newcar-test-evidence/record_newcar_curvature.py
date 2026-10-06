import importlib.util,sys,time,json,yaml
from pathlib import Path
from dataclasses import fields
import rclpy
from rclpy.node import Node
from dream_interfaces.msg import ConeDetections
p=Path.home()/'ai4r_student_workspace/src/ai4r_policy'
spec=importlib.util.spec_from_file_location('policy_probe',p/'scripts/policy_node.py');mod=importlib.util.module_from_spec(spec);sys.modules[spec.name]=mod;spec.loader.exec_module(mod)
cfg=yaml.safe_load((p/'config/ai4r_policy.yaml').read_text());params=next(iter(cfg.values()))['ros__parameters'];settings=mod.EstimationSettings(**params['estimation']);est=mod.EstimationPipeline(settings)
rclpy.init();n=Node('road_curvature_record');out=(Path.home()/'newcar-test-evidence/extended-curvature.jsonl').open('w',buffering=1)
def cb(m):
 stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9;now=n.get_clock().now().nanoseconds/1e9
 batch={'detections':[(d.position.x,d.position.y,d.position.z,d.color,d.classification_confidence) for d in m.detections]}
 road=est.road(batch,int(stamp*1e9),now-stamp)
 out.write(json.dumps({'time_unix_s':time.time(),'valid':road['valid'],'status':road['status'],'visibility':road['visibility'],'curvature_1pm':road['local_curvature_1pm'],'x_range_m':road['x_range_m'],'fit_rms_m':road.get('fit_rms_m'),'raw_batch':batch})+'\n')
sub=n.create_subscription(ConeDetections,'/car/cone_detections',cb,10);end=time.monotonic()+45
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
out.close();n.destroy_node();rclpy.shutdown()
