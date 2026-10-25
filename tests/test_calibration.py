"""Calibration cloud demand, frame composition, and verified SDK writeback."""
import ast
import hashlib
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[4]
CAMERA = Path(__file__).resolve().parents[1]/'roboscan_nsl3130'
sys.path.insert(0, str(ROOT/'controller/src/robot_dt_bridge/config'))
sys.path.insert(0, str(CAMERA/'scripts'))


def methods():
    path = Path(__file__).parents[1]/'roboscan_nsl3130/scripts/multiview_calib_node.py'
    tree = ast.parse(path.read_text())
    selected = [n for node in tree.body if isinstance(node, ast.ClassDef)
                for n in node.body if isinstance(n, ast.FunctionDef)
                and n.name in ('_set_raw_cloud_subscription', '_cloud_cb')]
    namespace = {'PointCloud2': object, '_sensor_qos': lambda depth: depth,
                 '_cloud_to_xyz': lambda msg: ('full-raw-xyz',)}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


def test_raw_subscription_only_while_refining():
    ns = methods()
    created, destroyed = [], []
    def create(*args):
        created.append(args)
        return 'raw-subscription'
    node = SimpleNamespace(a=SimpleNamespace(depth_refine=True), _ext=True,
                           _raw_cloud_sub=None, _raw_cloud_topic='/cam_161/camera/point_cloud',
                           _cloud_cb=lambda msg: None, create_subscription=create,
                           destroy_subscription=destroyed.append)
    update = ns['_set_raw_cloud_subscription']
    update(node, False)
    assert not created
    update(node, True)
    update(node, True)
    assert len(created) == 1
    assert created[0][1] == '/cam_161/camera/point_cloud'
    update(node, False)
    assert destroyed == ['raw-subscription']
    assert node._raw_cloud_sub is None


def test_rgb_frame_metadata_does_not_replace_raw_depth_refinement():
    callback = methods()['_cloud_cb']
    node = SimpleNamespace(live_lidar_frame='cam_161_lidar_frame',
                           a=SimpleNamespace(depth_refine=True), _ext=True,
                           saved=False, _clouds=[])
    msg = SimpleNamespace(header=SimpleNamespace(frame_id='cam_161_lidar_frame'))
    callback(node, msg, frame_only=True)
    assert not node._clouds
    callback(node, msg)
    assert node._clouds == [('full-raw-xyz',)]


def test_frame_metadata_subscription_released_after_first_message():
    callback = methods()['_cloud_cb']
    destroyed = []
    node = SimpleNamespace(live_lidar_frame='cam_161_lidar_frame',
                           a=SimpleNamespace(depth_refine=True), _ext=True,
                           saved=False, _clouds=[], _frame_sub='metadata',
                           destroy_subscription=destroyed.append)
    msg = SimpleNamespace(header=SimpleNamespace(frame_id='cam_161_lidar_frame'))
    callback(node, msg, frame_only=True)
    assert destroyed == ['metadata']
    assert node._frame_sub is None
    assert not node._clouds


class GlobalFrameTests(unittest.TestCase):
    def test_saved_marker_relative_poses_compose_to_the_map_pose(self):
        frames = yaml.safe_load((ROOT/'controller/src/NSL-3130AA-ROS2/calib_output/global_frames.yml').read_text())
        identity = frames['global']['anchor']['id_0']
        if identity['verification'] == 'unverified_identity_seed':
            np.testing.assert_array_equal(np.array(identity['tf']).reshape(4, 4), np.eye(4))
        for name, camera in frames['global']['cameras'].items():
            for marker, anchor in frames['global']['anchor'].items():
                relation = frames['relative'][marker][name]
                self.assertEqual(relation['parent'], marker)
                self.assertEqual(relation['child'], f'{name}_lidar_frame')
                if camera['tf'] is None:
                    self.assertIsNone(relation['tf'])
                else:
                    map_anchor = np.array(anchor['tf']).reshape(4, 4)
                    anchor_camera = np.array(relation['tf']).reshape(4, 4)
                    np.testing.assert_allclose(map_anchor @ anchor_camera,
                                               np.array(camera['tf']).reshape(4, 4), atol=1e-12)


class CompletedWritebackTests(unittest.TestCase):
    def node(self,config,calibration):
        path=CAMERA/'scripts/multiview_put_server.py'
        tree=ast.parse(path.read_text())
        retained=[item for item in tree.body if isinstance(item,(ast.FunctionDef,ast.ClassDef))
                  and item.name in ('_sha256','sync_completed_writeback','MultiviewPutServer')]
        scope={'Node':object,'os':os,'hashlib':hashlib,'ALLOWED':'multiview.yml'}
        exec(compile(ast.Module(body=retained,type_ignores=[]),str(path),'exec'),scope)
        node=scope['MultiviewPutServer'].__new__(scope['MultiviewPutServer'])
        node.serial='TEST_SERIAL'
        node.namespace='cam_161'
        node.calib_dir=str(calibration)
        node.dev_dir=str(calibration/'TEST_SERIAL')
        Path(node.dev_dir).mkdir(parents=True)
        logs=[]
        node.get_logger=lambda:SimpleNamespace(info=logs.append,warning=logs.append)
        return node,logs

    def test_verified_install_updates_binding_and_profile_without_touching_robot_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            config=root/'config.yaml'
            config.write_text(yaml.safe_dump({'hi6':{'keep':'robot'},'cameras':{'cam_161':{'serial':None}}}))
            calibration=root/'calibration'
            node,logs=self.node(config,calibration)
            payload=b'%YAML:1.0\n---\nreference_frame: stag_marker\nnamespace: cam_161\n'
            request=SimpleNamespace(name='multiview.yml',offset=0,total_size=len(payload),
                                    sha256=hashlib.sha256(payload).hexdigest(),data=payload)
            with patch.dict(os.environ,{'SIRIUS_CONFIG':str(config)}):
                response=node._on_put(request,SimpleNamespace())
            self.assertTrue(response.success)
            self.assertTrue(response.complete)
            saved=yaml.safe_load(config.read_text())
            self.assertEqual(saved['hi6'],{'keep':'robot'})
            self.assertEqual(saved['cameras']['cam_161']['serial'],'TEST_SERIAL')
            self.assertEqual(saved['calibration_profiles']['TEST_SERIAL']['multiview']['sha256'],request.sha256)
            self.assertEqual((calibration/'TEST_SERIAL/multiview.yml').read_bytes(),payload)

    def test_digest_mismatch_cannot_update_registry(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            config=root/'config.yaml'
            config.write_text('cameras: {cam_161: {serial: null}}\n')
            before=config.read_bytes()
            calibration=root/'calibration'
            node,_=self.node(config,calibration)
            request=SimpleNamespace(name='multiview.yml',offset=0,total_size=3,sha256='invalid',data=b'bad')
            with patch.dict(os.environ,{'SIRIUS_CONFIG':str(config)}):
                response=node._on_put(request,SimpleNamespace())
            self.assertFalse(response.success)
            self.assertFalse(response.complete)
            self.assertEqual(config.read_bytes(),before)
            self.assertFalse((calibration/'TEST_SERIAL/multiview.yml').exists())

    def test_compact_writeback_updates_only_serial_and_profile_pointer(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root/'config.yaml'
            identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
            robots = {'loading': {'ip': None, 'port': 8888, 'tf': identity,
                                  'direction': [1, 1, -1, 1, -1, 1], 'pose': [1, 2, 3, 4, 5, 6]}}
            config.write_text(yaml.safe_dump({'schema_version': 2,
                'server': {'ip': '192.168.3.166', 'domain_id': 42}, 'robots': robots,
                'cameras': {'cam_161': {'ip': '192.168.3.161', 'serial': None,
                                      'parameters': 'calibration/cam_161', 'tf': identity}},
                'anchor': {'id': [0, 7], 'tf': identity}}))
            calibration = root/'calibration'
            node, _ = self.node(config, calibration)
            payload = b'%YAML:1.0\n---\nreference_frame: stag_marker\nnamespace: cam_161\n'
            request = SimpleNamespace(name='multiview.yml', offset=0, total_size=len(payload),
                                      sha256=hashlib.sha256(payload).hexdigest(), data=payload)
            with patch.dict(os.environ, {'SIRIUS_CONFIG': str(config)}), \
                 patch('detect_camera_id.detect_camera_serials', return_value=['TEST_SERIAL']):
                response = node._on_put(request, SimpleNamespace())
            self.assertTrue(response.success)
            self.assertTrue(response.complete)
            saved = yaml.safe_load(config.read_text())
            self.assertEqual(saved['robots'], robots)
            self.assertEqual(saved['cameras']['cam_161']['serial'], 'TEST_SERIAL')
            self.assertEqual(saved['cameras']['cam_161']['parameters'], 'calibration/TEST_SERIAL')
            self.assertEqual(saved['cameras']['cam_161']['tf'], identity)
            self.assertEqual(set(saved), {'schema_version', 'server', 'robots', 'cameras', 'anchor'})
            manifest = yaml.safe_load((calibration/'profiles.yaml').read_text())
            metadata = manifest['profiles']['TEST_SERIAL']['multiview']
            self.assertEqual(metadata['sha256'], request.sha256)
            self.assertNotIn('values', metadata)
            self.assertLess(len(config.read_text().splitlines()), 25)

    def test_native_writeback_updates_camera_entry_and_external_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            module = Path(directory)
            (module/'setup').mkdir()
            config = module/'setup/fleet.yaml'
            config.write_text(yaml.safe_dump({'schema_version': 1,
                'server': {'ip': '192.168.3.166', 'domain_id': 42},
                'cameras': {'cam_161': {'ip': '192.168.3.161', 'serial': None,
                                      'parameters': '../calib_output/runtime/cam_161',
                                      'sensor_host': '192.168.2.202'}}}))
            calibration = module/'calib_output'
            node, _ = self.node(config, calibration)
            payload = b'%YAML:1.0\n---\nnamespace: cam_161\n'
            request = SimpleNamespace(name='multiview.yml', offset=0, total_size=len(payload),
                                      sha256=hashlib.sha256(payload).hexdigest(), data=payload)
            with patch.dict(os.environ, {'SIRIUS_CONFIG': str(config)}), \
                 patch('detect_camera_id.detect_camera_serials', return_value=['TEST_SERIAL']):
                response = node._on_put(request, SimpleNamespace())
            self.assertTrue(response.success)
            saved = yaml.safe_load(config.read_text())
            self.assertEqual(saved['cameras']['cam_161']['serial'], 'TEST_SERIAL')
            self.assertTrue(saved['cameras']['cam_161']['serial_verified'])
            self.assertEqual(saved['cameras']['cam_161']['parameters'], '../calib_output/TEST_SERIAL')
            self.assertEqual(saved['cameras']['cam_161']['sensor_host'], '192.168.2.202')
            self.assertNotIn('calibration_profiles', saved)
            self.assertFalse((config.parent/'cameras').exists())
            manifest = yaml.safe_load((calibration/'runtime/profiles.yaml').read_text())
            self.assertEqual(manifest['profiles']['TEST_SERIAL']['multiview']['sha256'], request.sha256)
