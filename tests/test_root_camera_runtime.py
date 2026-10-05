"""Public camera edits affect bound runtime geometry without inventing identities."""
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

MODULE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE/'setup'))
sys.path.insert(0, str(MODULE/'roboscan_nsl3130/scripts'))
import camera_network
import sirius_camera_config as camera
import camera_device_info_node as device


class RootCameraRuntimeTests(unittest.TestCase):
    def fixture(self, directory, serial='ACTUAL'):
        root = Path(directory)
        module = root/'controller/src/NSL-3130AA-ROS2'
        fleet = module/'setup/fleet.yaml'
        fleet.parent.mkdir(parents=True)
        fleet.write_text(yaml.safe_dump({'schema_version': 1,
            'server': {'ip': '192.168.3.166', 'domain_id': 42},
            'cameras': {'cam_161': {'ip': '192.168.3.161', 'serial': None,
                'sensor_host': '192.168.2.202', 'sensor_source_address': '192.168.2.102'},
                        'cam_162': {'ip': '192.168.3.162', 'serial': None,
                'sensor_host': '192.168.2.203', 'sensor_source_address': '192.168.2.103'}}}))
        profile = module/'calib_output/ACTUAL'
        profile.mkdir(parents=True)
        storage = cv2.FileStorage(str(profile/'intrinsic.yml'), cv2.FILE_STORAGE_WRITE)
        storage.write('camera_id', 'ACTUAL')
        storage.write('image_width', 1920)
        storage.write('image_height', 1080)
        storage.write('distortion_model', 'equidistant')
        storage.write('camera_matrix', np.array([[1000., 0, 960], [0, 1000, 540], [0, 0, 1]]))
        storage.write('distortion_coefficients', np.zeros((1, 4)))
        storage.release()
        storage = cv2.FileStorage(str(profile/'extrinsic.yml'), cv2.FILE_STORAGE_WRITE)
        storage.write('R', np.eye(3))
        storage.write('t', np.zeros((3, 1)))
        storage.release()
        config = root/'config/config.yaml'
        config.parent.mkdir()
        transform = np.eye(4)
        transform[:3, 3] = [0.1, -0.2, 0.3]
        config.write_text(yaml.safe_dump({'schema_version': 2,
            'server': {'ip': '192.168.3.170', 'domain_id': 42},
            'cameras': {'cam_161': {'ip': '192.168.3.171', 'serial': serial,
                'parameters': '../controller/src/NSL-3130AA-ROS2/calib_output/ACTUAL',
                'intrinsic': [1010., 0, 950., 0, 1020., 530., 0, 0, 1],
                'distortion': [0.01, 0.02, 0.03, 0.04], 'extrinsic': transform.ravel().tolist()}},
            'anchor': {'id_0': {'tf': None}, 'id_7': {'tf': None}},
            'pose': {'config': '../controller/src/ros_humanpose/config/custom.yml'},
            'localization': {'session': '../controller/src/robot_dt_bridge/localization/calibration/custom.yaml', 'anchor_id': 0}}))
        return root, module, config, profile

    def test_public_inventory_drives_addresses_without_registering_planned_edges(self):
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory, serial=None)
            original = config.read_bytes()
            with patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(root), 'SIRIUS_CONFIG': ''}):
                data, selected = camera.inventory()
                self.assertEqual(selected, config)
                self.assertEqual(set(data['cameras']), {'cam_161'})
                self.assertEqual(camera_network.inventory_path(), config)
                self.assertEqual(camera_network.network_field('host'), '192.168.3.170')
                self.assertEqual(camera_network.camera_field(161, 'host'), '192.168.3.171')
                self.assertEqual(camera_network.camera_field(161, 'sensor_host'), '192.168.2.202')
                self.assertEqual(camera_network.camera_field(162, 'host'), '')
                self.assertEqual(camera_network.camera_field(162, 'sensor_host'), '192.168.2.203')
                self.assertEqual(camera.camera_runtime('cam_162', '', profile.parent), ('', ''))
                self.assertEqual(camera.anchor_ids(), [0, 7])
            self.assertEqual(config.read_bytes(), original)
            self.assertFalse((root/'config/cameras').exists())

    def test_matching_physical_serial_uses_overrides_and_preserves_raw_sdk(self):
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory)
            original = {name: (profile/name).read_bytes() for name in ('intrinsic.yml', 'extrinsic.yml')}
            with patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(root), 'SIRIUS_CONFIG': str(config)}):
                overrides = camera.runtime_override_files('cam_161', 'ACTUAL', profile, profile.parent,
                                                          detected_serial='ACTUAL')
                self.assertEqual(set(overrides), {'intrinsic.yml', 'camera_info.yaml', 'extrinsic.yml'})
                storage = cv2.FileStorage(str(overrides['intrinsic.yml']), cv2.FILE_STORAGE_READ)
                self.assertEqual(storage.getNode('camera_matrix').mat()[0, 0], 1010.)
                np.testing.assert_allclose(storage.getNode('distortion_coefficients').mat(), [[.01, .02, .03, .04]])
                self.assertEqual(storage.getNode('image_width').real(), 1920)
                storage.release()
                info = yaml.safe_load(overrides['camera_info.yaml'].read_text())
                self.assertEqual(info['fx'], 1010.)
                self.assertEqual(info['D'], [.01, .02, .03, .04])
                self.assertEqual(info['camera_model'], 'fisheye')
                storage = cv2.FileStorage(str(overrides['extrinsic.yml']), cv2.FILE_STORAGE_READ)
                np.testing.assert_allclose(storage.getNode('t').mat().ravel(), [.1, -.2, .3])
                storage.release()
                self.assertTrue(all(file.is_relative_to(profile.parent/'runtime/cam_161/overrides') for file in overrides.values()))
                self.assertEqual(camera.profile_file(profile, profile.parent, 'ACTUAL', 'intrinsic.yml', raw=True), profile/'intrinsic.yml')
            self.assertEqual(original, {name: (profile/name).read_bytes() for name in original})

    def test_unknown_or_replaced_serial_cannot_apply_unbound_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory, serial=None)
            with patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(root), 'SIRIUS_CONFIG': str(config)}):
                self.assertEqual(camera.runtime_override_files('cam_161', 'ACTUAL', profile, profile.parent, detected_serial='ACTUAL'), {})
                settings = yaml.safe_load(config.read_text())
                settings['cameras']['cam_161']['serial'] = 'OLD'
                config.write_text(yaml.safe_dump(settings))
                self.assertEqual(camera.runtime_override_files('cam_161', 'ACTUAL', profile, profile.parent, detected_serial='ACTUAL'), {})
                self.assertEqual(camera.camera_runtime('cam_161', 'ACTUAL', profile.parent)[1], str(profile))
            self.assertFalse((profile.parent/'runtime').exists())

    def test_native_verified_observation_enables_same_serial_and_raw_service_stays_raw(self):
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory)
            cache = profile.parent/'runtime/cam_161/device_info.yaml'
            cache.parent.mkdir(parents=True)
            cache.write_text('serial: ACTUAL\nverified: true\nsource: usb_sysfs\n')
            usb = root/'usb/1'
            usb.mkdir(parents=True)
            for key, value in {'idVendor': '1fc9', 'idProduct': '0099', 'serial': 'ACTUAL'}.items():
                (usb/key).write_text(value)
            with patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(root), 'SIRIUS_CONFIG': str(config)}):
                selected = camera.profile_file(profile, profile.parent, 'ACTUAL', 'intrinsic.yml')
                self.assertNotEqual(selected, profile/'intrinsic.yml')
                report = device.describe_device('cam_161', profile.parent, config, usb.parent)
                self.assertTrue(report['verified'])
                self.assertEqual(report['files']['intrinsic.yml']['file'], str(profile/'intrinsic.yml'))
                self.assertEqual(report['files']['extrinsic.yml']['file'], str(profile/'extrinsic.yml'))
            self.assertFalse((root/'config/cameras').exists())

    def test_explicit_profile_has_precedence_over_inline_edits(self):
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory)
            alternate = module/'calib_output/explicit'
            alternate.mkdir()
            with patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(root), 'SIRIUS_CONFIG': str(config)}):
                self.assertEqual(camera.runtime_override_files('cam_161', 'ACTUAL', alternate, profile.parent,
                                                               detected_serial='ACTUAL'), {})

    def test_unchanged_mirrored_values_retain_original_sdk_paths_and_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory)
            settings = yaml.safe_load(config.read_text())
            settings['cameras']['cam_161'].update(intrinsic=[1000., 0, 960, 0, 1000., 540, 0, 0, 1],
                                                  distortion=[0, 0, 0, 0], extrinsic=np.eye(4).reshape(-1).tolist())
            config.write_text(yaml.safe_dump(settings))
            with patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(root), 'SIRIUS_CONFIG': str(config)}):
                self.assertEqual(camera.runtime_override_files('cam_161', 'ACTUAL', profile, profile.parent,
                                                               detected_serial='ACTUAL'), {})
            self.assertFalse((profile.parent/'runtime').exists())

    def test_camera_fingerprint_ignores_global_tf_and_other_camera_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory)
            with patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(root)}):
                baseline = camera_network.camera_fingerprint(161, config)
                values = yaml.safe_load(config.read_text())
                values['cameras']['cam_161']['tf'] = np.eye(4).reshape(-1).tolist()
                values['cameras']['cam_162'] = {'ip': '192.168.3.162', 'serial': 'other'}
                config.write_text(yaml.safe_dump(values))
                self.assertEqual(camera_network.camera_fingerprint(161, config), baseline)
                values['cameras']['cam_161']['intrinsic'][0] += 1
                config.write_text(yaml.safe_dump(values))
                self.assertNotEqual(camera_network.camera_fingerprint(161, config), baseline)

    def test_nested_workflow_pose_and_session_paths_resolve_relative_to_public_config(self):
        spec = importlib.util.spec_from_file_location('root_camera_workflow', MODULE/'setup/workflow.py')
        workflow = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(workflow)
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory)
            receipt = '{"success":true,"config_path":"prepared.yml"}'
            with patch.object(workflow, 'ROOT', root), \
                 patch.object(workflow, 'camera_settings', return_value=(config, {})), \
                 patch.object(workflow.subprocess, 'run', return_value=type('Result', (), {'stdout': receipt, 'stderr': '', 'returncode': 0})()) as run:
                self.assertEqual(workflow.session_file().resolve(), root/'controller/src/robot_dt_bridge/localization/calibration/custom.yaml')
                workflow.prepare_pose(None, ['161'])
                argv = run.call_args.args[0]
                self.assertEqual(Path(argv[argv.index('--config')+1]).resolve(), root/'controller/src/ros_humanpose/config/custom.yml')
                self.assertEqual(argv[argv.index('--fleet')+1], str(config))


if __name__ == '__main__':
    unittest.main()
