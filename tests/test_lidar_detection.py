"""Portable detector/actual node-method checks; no ROS or physical vehicle."""
import ast
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from numbers import Real
from pathlib import Path
import runpy
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

api = runpy.run_path(str(Path(__file__).with_name('test_control.py')))
ns = api['ns']
tree = ast.parse(api['api']['SOURCE'].read_text(encoding='utf-8'))
names = {'LidarObstacleSettings', 'lidar_roi_bounds', '_lidar_ray_rectangle', 'detect_lidar_obstacles',
         'quaternion_xyzw', 'valid_frame'}
definitions = [n for n in tree.body if isinstance(n,(ast.ClassDef,ast.FunctionDef)) and n.name in names]
ns.update(Real=Real, dataclass=dataclass, json=json)
exec(compile(ast.Module(body=definitions,type_ignores=[]),str(api['api']['SOURCE']),'exec'),ns)
subject = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PolicyNode')
methods = [n for n in subject.body if isinstance(n,ast.FunctionDef)
           and n.name in ('lidar_callback','_convert_lidar_scan','_stamp_ns')]
port = ast.ClassDef(name='LidarPort',bases=[ast.Name(id='SourceNode',ctx=ast.Load())],
                   keywords=[],body=methods,decorator_list=[])
exec(compile(ast.fix_missing_locations(ast.Module(body=[port],type_ignores=[])),
             str(api['api']['SOURCE']),'exec'),ns)
Settings, detect = ns['LidarObstacleSettings'], ns['detect_lidar_obstacles']


class Cloud:
    def __init__(self):
        self.header = SimpleNamespace(frame_id='',stamp=None)
        self.points = []


class PathMessage:
    def __init__(self,header=None):
        self.header, self.poses = header, []


class Pose:
    def __init__(self,header=None):
        self.header = header
        self.pose = SimpleNamespace(position=SimpleNamespace(x=0.,y=0.,z=0.),
                                    orientation=SimpleNamespace(w=0.))


class Stamp:
    def __init__(self,nanoseconds):
        self.ns = nanoseconds
    def to_msg(self):
        return SimpleNamespace(sec=self.ns//10**9,nanosec=self.ns%10**9)
    @staticmethod
    def from_msg(stamp):
        return stamp


ns.update(PointCloud=Cloud,Point32=SimpleNamespace,TrajectoryPath=PathMessage,
          PoseStamped=Pose,Time=Stamp,TransformException=ValueError)


def scan_fixture(hits=None, yaw=0., origin=(.2,0.,.12)):
    count=720
    ranges=[4.]*count
    for i,r in (hits or {}).items(): ranges[i]=r
    increment=2*math.pi/count
    points=[];indices=[]
    for i,r in enumerate(ranges):
        if math.isfinite(r) and .05<=r<=16.:
            points.append((origin[0]+r*math.cos(yaw+i*increment),
                           origin[1]+r*math.sin(yaw+i*increment),origin[2]))
            indices.append(i)
    raw=dict(frame_id='laser',ranges=ranges,intensities=[],angle_min=0.,
        angle_max=(count-1)*increment,angle_increment=increment,range_min=.05,
        range_max=16.,time_increment=0.,scan_time=.1)
    cart=dict(frame_id='base_link',points_xyz=points,scan_indices=indices,
        ray_origin_xyz_m=origin,ray_axes_xyz=((math.cos(yaw),math.sin(yaw),0.),
                                           (-math.sin(yaw),math.cos(yaw),0.)))
    return raw,cart


def node_fixture(enabled=True,stop=False):
    node=api['make_node'](simulated=False,mvp=True)
    node.__class__=ns['LidarPort']
    node.policy_update_mode='timer'
    node.lidar_obstacle_settings=Settings(enabled=enabled,stop_enabled=stop,
        stop_distance_m=1.,roi_half_width_m=.6,allow_no_return_clear=True)
    node.lidar_obstacle_output=None
    node._lidar_obstacle_stamp_ns=node._lidar_obstacle_publication_key=None
    for name in ('lidar_obstacle','lidar_points','lidar_roi'):
        setattr(node,name+'_publisher',api['Recorder']())
    return node


def feed(node,hits=None):
    api['feed'](node)
    for name,value in zip(('lidar_scan','lidar_cartesian'),scan_fixture(hits)):
        node._store(name,value,node.test_clock.ros_ns,node.test_clock.monotonic,node.test_clock.ros_ns)


class LidarDetectionChecks(unittest.TestCase):
    def test_roi_keeps_both_lateral_sides_and_excludes_far_points(self):
        raw,cart=scan_fixture({45:.8,675:.8,90:1.2,0:1.4})
        result=detect(raw,cart,node_fixture().lidar_obstacle_settings)
        self.assertEqual(result['scan_indices'],[45,675])
        self.assertEqual(result['roi_xy_m'],((.297,1.297),(-.6,.6)))
        self.assertGreater(result['points_xyz_m'][0][1],0.)
        self.assertLess(result['points_xyz_m'][1][1],0.)

    def test_clear_sparse_cluster_and_invalid_returns_are_distinguished(self):
        cfg=node_fixture().lidar_obstacle_settings
        for hits,expected in ((None,'CLEAR'),({0:.6},'UNKNOWN'),({0:.6,1:.6},'OBSTACLE')):
            raw,cart=scan_fixture(hits)
            self.assertEqual(detect(raw,cart,cfg)['status'],expected)
        raw,cart=scan_fixture()
        raw['ranges']=[math.nan]*720
        cart.update(points_xyz=[],scan_indices=[])
        self.assertEqual(detect(raw,cart,cfg)['status'],'UNKNOWN')

    def test_rotated_sensor_mount_keeps_body_forward_roi(self):
        raw,cart=scan_fixture({360:.6,361:.6},yaw=math.pi)
        result=detect(raw,cart,node_fixture().lidar_obstacle_settings)
        self.assertEqual(result['status'],'OBSTACLE')
        self.assertEqual(result['scan_indices'],[360,361])

    def test_detect_only_is_control_equivalent_with_obstacles_and_without_lidar(self):
        outputs=[]
        for enabled,hits in ((False,{0:.6,1:.6}),(True,{0:.6,1:.6}),(True,None)):
            node=node_fixture(enabled)
            if hits is None: api['feed'](node)
            else: feed(node,hits)
            api['start'](node);node.run_policy_step()
            self.assertEqual(node.fsm_state,3,(node.state_reason,node.logged_errors))
            self.assertFalse(node.logged_errors,node.logged_errors)
            command=node.action_publisher.messages[-1]
            outputs.append((command.drive,command.steer))
            if enabled:
                self.assertFalse(node.lidar_obstacle_output['stop_requested'])
                self.assertEqual(node.lidar_obstacle_output['mode'],'detect_only')
        self.assertEqual(outputs[0],outputs[1])
        self.assertEqual(outputs[1],outputs[2])

    def test_standard_visualization_messages_publish_while_policy_is_stopped(self):
        node=node_fixture();feed(node,{0:.6,1:.6})
        node.supervision_callback()
        self.assertEqual(node.fsm_state,2)
        result=json.loads(node.lidar_obstacle_publisher.messages[-1].data)
        cloud=node.lidar_points_publisher.messages[-1]
        self.assertEqual(len(cloud.points),2)
        self.assertEqual(cloud.header.frame_id,'base_link')
        self.assertEqual(cloud.header.stamp.sec,10)
        rectangle=node.lidar_roi_publisher.messages[-1]
        self.assertEqual(len(rectangle.poses),5)
        self.assertAlmostEqual(rectangle.poses[0].pose.position.x,.297)
        self.assertAlmostEqual(rectangle.poses[2].pose.position.x,1.297)
        self.assertEqual(result['status'],'OBSTACLE')
        self.assertFalse(result['stop_requested'])

    def test_loss_clears_visual_points_and_never_stops_detect_only(self):
        node=node_fixture();feed(node,{0:.6,1:.6});api['start'](node);node.run_policy_step()
        for _ in range(24):
            node.test_clock.advance(.025);api['feed'](node)
            node.run_policy_step();node.supervision_callback()
            self.assertEqual(node.fsm_state,3,(node.state_reason,node.logged_errors))
        self.assertEqual(node.lidar_obstacle_output['status'],'UNKNOWN')
        self.assertFalse(node.lidar_obstacle_output['available'])
        self.assertEqual(node.lidar_points_publisher.messages[-1].points,[])

    def test_optional_stop_retains_explicit_resume(self):
        node=node_fixture(stop=True);feed(node);api['start'](node);node.run_policy_step()
        self.assertEqual(node.fsm_state,3)
        node.test_clock.advance(.05);feed(node,{0:.6,1:.6});node.supervision_callback()
        self.assertEqual(node.fsm_state,2)
        self.assertIn('Lidar stop',node.state_reason)
        self.assertEqual(node.action_publisher.messages[-1].drive,0.)
        node.test_clock.advance(.05);feed(node);node.supervision_callback()
        self.assertEqual(node.fsm_state,2)

    def test_telemetry_exception_does_not_change_detect_only_control(self):
        node=node_fixture();feed(node,{0:.6,1:.6})
        node.lidar_obstacle_publisher.publish=Mock(side_effect=RuntimeError('telemetry unavailable'))
        api['start'](node);node.run_policy_step()
        self.assertEqual(node.fsm_state,3,(node.state_reason,node.logged_errors))

    def test_scan_conversion_preserves_upstream_points_and_adds_ray_metadata(self):
        raw,_=scan_fixture({360:.6},yaw=math.pi)
        transform=SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(x=.2,y=0.,z=.12),
            rotation=SimpleNamespace(x=0.,y=0.,z=1.,w=0.)))
        outputs=[]
        for enabled in (False,True):
            node=node_fixture(enabled)
            node.tf_buffer=SimpleNamespace(lookup_transform=lambda *_:transform)
            outputs.append(node._convert_lidar_scan(raw,SimpleNamespace(sec=10,nanosec=0)))
        self.assertEqual(outputs[0]['points_xyz'],outputs[1]['points_xyz'])
        self.assertEqual(outputs[0]['scan_indices'],outputs[1]['scan_indices'])
        self.assertNotIn('ray_origin_xyz_m',outputs[0])
        self.assertEqual(outputs[1]['ray_origin_xyz_m'],(.2,0.,.12))
        self.assertEqual(outputs[1]['ray_axes_xyz'][0],(-1.,0.,0.))

    def test_bad_scan_or_missing_tf_invalidates_old_display_points(self):
        for malformed in (False,True):
            node=node_fixture();feed(node,{0:.6,1:.6})
            node._lidar_stop_problem(node.test_clock.monotonic,node.test_clock.ros_ns)
            node.test_clock.advance(.05)
            raw,_=scan_fixture()
            if malformed: raw['range_max']=-1.
            stamp=Stamp(node.test_clock.ros_ns).to_msg()
            msg=SimpleNamespace(header=SimpleNamespace(frame_id='laser',stamp=stamp),**raw)
            node.tf_buffer=SimpleNamespace(lookup_transform=Mock(side_effect=ValueError('missing TF')))
            node.lidar_callback(msg)
            self.assertIsNone(node.observations['lidar_cartesian'])
            self.assertIsNone(node._lidar_stop_problem(node.test_clock.monotonic,node.test_clock.ros_ns))
            self.assertEqual(node.lidar_obstacle_output['status'],'UNKNOWN')
            self.assertEqual(node.lidar_points_publisher.messages[-1].points,[])

    def test_deploy_layering_preserves_lidar_and_new_team_mpc_fixes(self):
        import sys
        root=Path(__file__).resolve().parents[1]
        sys.path.insert(0,str(root/'tools'))
        from make_deploy_yaml import build,KEY
        profile,_=build('mpc-exec-lattice21',5.,cones='yellow-left',car='05',speed_profile='v05')
        p=profile[KEY]['ros__parameters']
        self.assertTrue(p['lidar_obstacle']['enabled'])
        self.assertFalse(p['lidar_obstacle']['stop_enabled'])
        self.assertEqual(p['lidar_obstacle']['roi_half_width_m'],.4)
        self.assertEqual(p['lidar_obstacle']['stop_distance_m'],1.4)
        self.assertTrue(p['mpc']['geometry_glitch_tolerable'])
        self.assertTrue(p['estimation']['keep_width_on_geometry_glitch'])
        self.assertEqual(p['planning']['cruise_speed_mps'],.5)


if __name__=='__main__':unittest.main()
