"""Camera inventory, profile binding and launch assembly without device access."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[4]
MODULE = ROOT/'controller/src/NSL-3130AA-ROS2'
PACKAGE = MODULE/'roboscan_nsl3130'
SCRIPTS = PACKAGE/'scripts'
for path in (ROOT/'controller/src/robot_dt_bridge/config', MODULE/'setup', SCRIPTS):
    sys.path.insert(0, str(path))

import camera_network as network
import camera_calibration_pull as pull
import camera_device_info_node as device
import sirius_camera_config


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


compact = load_module('camera_inventory_runtime', SCRIPTS/'sirius_camera_config.py')
solver = load_module('camera_inventory_solver', SCRIPTS/'multiview_solver_node.py')
calib = load_module('camera_inventory_calib', SCRIPTS/'multiview_calib_node.py')
tf_module = load_module('camera_inventory_tf', SCRIPTS/'multiview_tf_node.py')


class CompactCameraTests(unittest.TestCase):
    def config(self, root):
        # This compact fixture has no native sensor defaults. Never inherit the
        # real checkout's planned Edges or the operator's sourced project root.
        native = {'schema_version': 1, 'server': {'ip': '192.168.3.166'}, 'cameras': {}}
        native_path = root/'native_fleet.yaml'
        native_path.write_text(yaml.safe_dump(native))
        native_patch = patch.object(sirius_camera_config, 'native_fleet',
                                    return_value=(native, native_path))
        native_patch.start()
        self.addCleanup(native_patch.stop)
        project = root.parent if root.name == 'config' else root
        environment = patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(project), 'SIRIUS_CONFIG': ''})
        environment.start()
        self.addCleanup(environment.stop)
        path = root/'config.yaml'
        path.write_text(yaml.safe_dump({'schema_version': 2,
            'server': {'ip': '192.168.3.166', 'domain_id': 87},
            'cameras': {'cam_161': {'ip': '192.168.3.161', 'serial': None,
                                  'parameters': 'calibration/cam_161', 'tf': None},
                        'cam_163': {'ip': '192.168.3.163', 'serial': None,
                                  'parameters': 'calibration/cam_163', 'tf': None}}}))
        return path

    def test_compact_config_uses_explicit_sensor_sidecar_and_server(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.config(root)
            (root/'cameras').mkdir()
            (root/'cameras/cam_161.yaml').write_text(
                'sensor_host: 192.168.2.202\nsensor_source_address: 192.168.2.102\n')
            self.assertEqual(network.camera_field('161', 'host', config), '192.168.3.161')
            self.assertEqual(network.camera_field('161', 'sensor_host', config), '192.168.2.202')
            self.assertEqual(network.camera_field('161', 'sensor_source_address', config), '192.168.2.102')
            self.assertEqual(network.camera_field('163', 'sensor_host', config), '')
            self.assertEqual(network.network_field('domain_id', config), '87')
            self.assertEqual(network.network_field('host', config), '192.168.3.166')

    def test_pull_uses_configured_edge_and_installs_only_verified_yaml(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.config(root)
            original = config.read_bytes()
            remote = root/'remote'
            remote.mkdir()
            (remote/'multiview.yml').write_text('%YAML:1.0\n---\nnamespace: cam_161\n')
            (remote/'large-image.png').write_bytes(b'ignored binary')
            calls = []
            def fake_scp(argv, **kwargs):
                calls.append(argv)
                shutil.copytree(remote, argv[-1], dirs_exist_ok=True)
            report = {'errors': []}
            with patch.object(pull.subprocess, 'run', side_effect=fake_scp), \
                 patch.dict(os.environ, {'SSHPASS': ''}), \
                 patch('sirius_config.sync.sync_profiles', return_value=report) as sync:
                result = pull.pull_profile(config, '161', user='operator')
            self.assertIn('operator@192.168.3.161:~/SiriusRobotDT/config/calibration/cam_161/*.y*ml', calls[0])
            self.assertNotIn('-r', calls[0])
            self.assertEqual(result['files'], ['multiview.yml'])
            self.assertIsNone(result['serial'])
            self.assertEqual(config.read_bytes(), original)
            self.assertTrue((root/'calibration/cam_161/multiview.yml').is_file())
            self.assertFalse((root/'calibration/cam_161/large-image.png').exists())
            sync.assert_called_once_with(config.resolve(), root/'calibration')

    def test_invalid_download_preserves_entire_existing_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.config(root)
            local = root/'calibration/cam_161'
            local.mkdir(parents=True)
            existing = local/'multiview.yml'
            existing.write_text('original: preserved\n')
            def fake_scp(argv, **kwargs):
                downloaded = Path(argv[-1])
                (downloaded/'multiview.yml').write_text('namespace: cam_161\n')
                (downloaded/'intrinsic.yml').write_text('invalid: [\n')
            with patch.object(pull.subprocess, 'run', side_effect=fake_scp), \
                 patch.dict(os.environ, {'SSHPASS': ''}):
                with self.assertRaises(yaml.YAMLError):
                    pull.pull_profile(config, '161', '/tmp/profile')
            self.assertEqual(existing.read_text(), 'original: preserved\n')
            self.assertFalse((local/'intrinsic.yml').exists())

    def test_unsafe_remote_path_does_not_start_scp(self):
        with tempfile.TemporaryDirectory() as directory:
            config = self.config(Path(directory))
            with patch.object(pull.subprocess, 'run') as run:
                with self.assertRaises(ValueError):
                    pull.pull_profile(config, '161', '/tmp/profile;touch /tmp/unexpected')
                run.assert_not_called()

    def test_module_profile_pointer_resolves_relative_to_inventory(self):
        with tempfile.TemporaryDirectory() as directory:
            project = Path(directory)
            (project/'config').mkdir()
            config = self.config(project/'config')
            document = yaml.safe_load(config.read_text())
            document['cameras']['cam_161']['parameters'] = '../controller/src/NSL-3130AA-ROS2/calib_output/NTEST'
            config.write_text(yaml.safe_dump(document))
            def fake_scp(argv, **kwargs):
                self.assertIn('192.168.3.161:~/SiriusRobotDT/controller/src/NSL-3130AA-ROS2/calib_output/NTEST/*.y*ml', argv)
                (Path(argv[-1])/'intrinsic.yml').write_text('camera: unchanged\n')
            with patch.object(pull.subprocess, 'run', side_effect=fake_scp), \
                 patch.dict(os.environ, {'SSHPASS': ''}), \
                 patch('sirius_config.sync.sync_profiles', return_value={'errors': []}):
                pull.pull_profile(config, '161')
            self.assertEqual((project/'controller/src/NSL-3130AA-ROS2/calib_output/NTEST/intrinsic.yml').read_text(), 'camera: unchanged\n')


class CommonCameraSettingsTests(unittest.TestCase):
    def fixture(self, root):
        module = root/'controller/src/NSL-3130AA-ROS2'
        (module/'setup').mkdir(parents=True)
        fleet = module/'setup/fleet.yaml'
        fleet.write_text(yaml.safe_dump({'schema_version': 1,
            'server': {'ip': '192.168.3.166', 'domain_id': 91},
            'cameras': {'cam_161': {'ip': '192.168.3.161', 'serial': 'SERIAL_NATIVE',
                'parameters': '../calib_output/SERIAL_NATIVE', 'sensor_host': '192.168.2.202'}}}))
        (root/'config').mkdir()
        manifest = root/'config/config.yaml'
        manifest.write_text(yaml.safe_dump({'schema_version': 3,
            'camera_settings': '../controller/src/NSL-3130AA-ROS2/setup/fleet.yaml', 'anchor_id': 13}))
        profile = module/'calib_output/SERIAL_NATIVE'
        profile.mkdir(parents=True)
        (profile/'params.yaml').write_text('original: tuning\n')
        return manifest, fleet, profile

    def test_default_manifest_routes_all_readers_to_native_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, fleet, profile = self.fixture(root)
            with patch.dict(os.environ, {'SIRIUS_CONFIG': '', 'SIRIUS_ROBOT_DT_ROOT': str(root)}):
                self.assertEqual(network.inventory_path(), fleet)
                self.assertEqual(network.network_field('domain_id'), '91')
                self.assertEqual(network.camera_field(161, 'sensor_host'), '192.168.2.202')
                self.assertEqual(sirius_camera_config.inventory()[1], fleet)
                self.assertEqual(sirius_camera_config.camera_runtime('cam_161', '', profile.parent), ('SERIAL_NATIVE', str(profile)))
                self.assertEqual(sirius_camera_config.anchor_ids(), [13])

    def test_explicit_native_override_keeps_priority(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, fleet, profile = self.fixture(root)
            override = root/'override.yaml'
            override.write_text(yaml.safe_dump({'schema_version': 1,
                'server': {'ip': '192.168.3.166', 'domain_id': 93},
                'cameras': {'cam_161': {'ip': '192.168.3.161', 'serial': 'EXPLICIT'}}}))
            with patch.dict(os.environ, {'SIRIUS_CONFIG': str(override), 'SIRIUS_ROBOT_DT_ROOT': str(root)}):
                self.assertEqual(network.network_field('domain_id'), '93')
                self.assertEqual(sirius_camera_config.camera_runtime('cam_161', '', profile.parent)[0], 'EXPLICIT')
                self.assertEqual(sirius_camera_config.inventory()[1], override)

    def test_pull_resolves_manifest_and_preserves_both_inventory_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, fleet, profile = self.fixture(root)
            before_manifest, before_fleet = manifest.read_bytes(), fleet.read_bytes()
            def fake_scp(argv, **kwargs):
                self.assertIn('192.168.3.161:~/SiriusRobotDT/controller/src/NSL-3130AA-ROS2/calib_output/SERIAL_NATIVE/*.y*ml', argv)
                (Path(argv[-1])/'multiview.yml').write_text('namespace: cam_161\n')
            with patch.object(pull.subprocess, 'run', side_effect=fake_scp),\
                 patch.dict(os.environ, {'SSHPASS': ''}):
                result = pull.pull_profile(manifest, 161)
            self.assertEqual(result['files'], ['multiview.yml'])
            self.assertEqual(manifest.read_bytes(), before_manifest)
            self.assertEqual(fleet.read_bytes(), before_fleet)
            self.assertEqual((profile/'params.yaml').read_text(), 'original: tuning\n')
            self.assertTrue((profile.parent/'runtime/profiles.yaml').is_file())

    def test_sensor_addresses_do_not_use_arithmetic_or_fill_unknown_devices(self):
        spec=importlib.util.spec_from_file_location('sirius_camera_network_test',PACKAGE.parent/'setup/camera_network.py')
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            config=Path(directory)/'config.yaml'
            config.write_text(yaml.safe_dump({'cameras':{
                'cam_161':{'sensor_host':'192.168.2.202','sensor_source_address':'192.168.2.102'},
                'cam_163':{'sensor_host':None,'sensor_source_address':None}}}))
            self.assertEqual(module.camera_field('161','sensor_host',config),'192.168.2.202')
            self.assertEqual(module.camera_field('163','sensor_host',config),'')
            self.assertEqual(module.camera_field('165','sensor_host',config),'')


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
                data, selected = sirius_camera_config.inventory()
                self.assertEqual(selected, config)
                self.assertEqual(set(data['cameras']), {'cam_161'})
                self.assertEqual(network.inventory_path(), config)
                self.assertEqual(network.network_field('host'), '192.168.3.170')
                self.assertEqual(network.camera_field(161, 'host'), '192.168.3.171')
                self.assertEqual(network.camera_field(161, 'sensor_host'), '192.168.2.202')
                self.assertEqual(network.camera_field(162, 'host'), '')
                self.assertEqual(network.camera_field(162, 'sensor_host'), '192.168.2.203')
                self.assertEqual(sirius_camera_config.camera_runtime('cam_162', '', profile.parent), ('', ''))
                self.assertEqual(sirius_camera_config.anchor_ids(), [0, 7])
            self.assertEqual(config.read_bytes(), original)
            self.assertFalse((root/'config/cameras').exists())

    def test_matching_physical_serial_uses_overrides_and_preserves_raw_sdk(self):
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory)
            original = {name: (profile/name).read_bytes() for name in ('intrinsic.yml', 'extrinsic.yml')}
            with patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(root), 'SIRIUS_CONFIG': str(config)}):
                overrides = sirius_camera_config.runtime_override_files('cam_161', 'ACTUAL', profile, profile.parent,
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
                self.assertEqual(sirius_camera_config.profile_file(profile, profile.parent, 'ACTUAL', 'intrinsic.yml', raw=True), profile/'intrinsic.yml')
            self.assertEqual(original, {name: (profile/name).read_bytes() for name in original})

    def test_unknown_or_replaced_serial_cannot_apply_unbound_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory, serial=None)
            with patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(root), 'SIRIUS_CONFIG': str(config)}):
                self.assertEqual(sirius_camera_config.runtime_override_files('cam_161', 'ACTUAL', profile, profile.parent, detected_serial='ACTUAL'), {})
                settings = yaml.safe_load(config.read_text())
                settings['cameras']['cam_161']['serial'] = 'OLD'
                config.write_text(yaml.safe_dump(settings))
                self.assertEqual(sirius_camera_config.runtime_override_files('cam_161', 'ACTUAL', profile, profile.parent, detected_serial='ACTUAL'), {})
                self.assertEqual(sirius_camera_config.camera_runtime('cam_161', 'ACTUAL', profile.parent)[1], str(profile))
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
                selected = sirius_camera_config.profile_file(profile, profile.parent, 'ACTUAL', 'intrinsic.yml')
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
                self.assertEqual(sirius_camera_config.runtime_override_files('cam_161', 'ACTUAL', alternate, profile.parent,
                                                               detected_serial='ACTUAL'), {})

    def test_unchanged_mirrored_values_retain_original_sdk_paths_and_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory)
            settings = yaml.safe_load(config.read_text())
            settings['cameras']['cam_161'].update(intrinsic=[1000., 0, 960, 0, 1000., 540, 0, 0, 1],
                                                  distortion=[0, 0, 0, 0], extrinsic=np.eye(4).reshape(-1).tolist())
            config.write_text(yaml.safe_dump(settings))
            with patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(root), 'SIRIUS_CONFIG': str(config)}):
                self.assertEqual(sirius_camera_config.runtime_override_files('cam_161', 'ACTUAL', profile, profile.parent,
                                                               detected_serial='ACTUAL'), {})
            self.assertFalse((profile.parent/'runtime').exists())

    def test_camera_fingerprint_ignores_global_tf_and_other_camera_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory)
            with patch.dict(os.environ, {'SIRIUS_ROBOT_DT_ROOT': str(root)}):
                baseline = network.camera_fingerprint(161, config)
                values = yaml.safe_load(config.read_text())
                values['cameras']['cam_161']['tf'] = np.eye(4).reshape(-1).tolist()
                values['cameras']['cam_162'] = {'ip': '192.168.3.162', 'serial': 'other'}
                config.write_text(yaml.safe_dump(values))
                self.assertEqual(network.camera_fingerprint(161, config), baseline)
                values['cameras']['cam_161']['intrinsic'][0] += 1
                config.write_text(yaml.safe_dump(values))
                self.assertNotEqual(network.camera_fingerprint(161, config), baseline)

    def test_nested_workflow_pose_and_session_paths_resolve_relative_to_public_config(self):
        spec = importlib.util.spec_from_file_location('root_camera_workflow', MODULE/'setup/workflow.py')
        workflow = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(workflow)
        with tempfile.TemporaryDirectory() as directory:
            root, module, config, profile = self.fixture(directory)
            receipt = '{"success":true,"config_path":"prepared.yml"}'
            with patch.object(workflow, 'ROOT', root),\
                 patch.object(workflow, 'camera_settings', return_value=(config, {})),\
                 patch.object(workflow.subprocess, 'run', return_value=type('Result', (), {'stdout': receipt, 'stderr': '', 'returncode': 0})()) as run:
                self.assertEqual(workflow.session_file().resolve(), root/'controller/src/robot_dt_bridge/localization/calibration/custom.yaml')
                workflow.prepare_pose(None, ['161'])
                argv = run.call_args.args[0]
                self.assertEqual(Path(argv[argv.index('--config')+1]).resolve(), root/'controller/src/ros_humanpose/config/custom.yml')
                self.assertEqual(argv[argv.index('--fleet')+1], str(config))


class CameraInventoryRuntimeTests(unittest.TestCase):
    def configuration(self, root, ids=(0, 7), serial='N010005C4'):
        config = root/'config.yaml'
        config.write_text(yaml.safe_dump({'schema_version': 2,
            'cameras': {'cam_161': {'ip': '192.168.3.161', 'serial': serial,
                                  'parameters': 'profiles/custom'}},
            'anchor': {'id': list(ids) if isinstance(ids, tuple) else ids}}))
        (root/'cameras').mkdir(exist_ok=True)
        (root/'cameras/cam_161.yaml').write_text('sensor_host: 192.168.2.202\n')
        return config

    def assemble(self, root, config, detected='N010005C4', overrides=None):
        module = load_module('camera_inventory_launch', PACKAGE/'launch/camera.launch.py')
        prefix = root/'install'
        scripts = prefix/'lib/roboscan_nsl3130'
        scripts.parent.mkdir(parents=True, exist_ok=True)
        scripts.symlink_to(SCRIPTS, target_is_directory=True)
        defaults = {}
        def declare(name, default_value='', **kwargs):
            defaults[name] = default_value
            return name
        class Configuration:
            def __init__(self, name):
                self.name = name
            def perform(self, context):
                return context.get(self.name, defaults[self.name])
        def fake_output(argv, **kwargs):
            if argv[0] == 'ip':
                return 'inet 192.168.3.161/24\n'
            return detected
        real_run = module.subprocess.run
        def fake_run(argv, **kwargs):
            if Path(argv[0]).name == 'camera_identity':
                record = {'serial': detected, 'chip_id': int(detected[1:5],16),
                          'wafer_id': int(detected[5:9],16), 'source': 'ethernet_sdk_ids',
                          'sensor_host': argv[1]}
                return SimpleNamespace(returncode=0,stdout=json.dumps(record)+'\n',stderr='')
            return real_run(argv,**kwargs)
        with patch.dict(os.environ, {'SIRIUS_CONFIG': str(config), 'SIRIUS_ROBOT_DT_ROOT': str(ROOT),
                                     'NSL_CALIB_DIR': str(root/'calibration'), 'HOME': str(root)}),\
             patch.object(module, 'get_package_share_directory', return_value=str(PACKAGE)),\
             patch.object(module, 'get_package_prefix', return_value=str(prefix)),\
             patch.object(module.subprocess, 'check_output', side_effect=fake_output),\
             patch.object(module.subprocess, 'run', side_effect=fake_run),\
             patch.object(module, 'DeclareLaunchArgument', side_effect=declare),\
             patch.object(module, 'LaunchDescription', side_effect=lambda entities: entities),\
             patch.object(module, 'LaunchConfiguration', Configuration),\
             patch.object(module, 'OpaqueFunction', side_effect=lambda function: function),\
             patch.object(module, 'Node', side_effect=lambda **kwargs: {'node': kwargs}),\
             patch.object(module, 'ExecuteProcess', side_effect=lambda **kwargs: {'process': kwargs}):
            description = module.generate_launch_description()
            context = {'use_rviz': 'false', 'use_rqt': 'false', 'use_rgb_compressor': 'false'}
            context.update(overrides or {})
            return description[-1](context)

    def test_custom_profile_reaches_driver_and_all_helper_processes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.configuration(root)
            profile = root/'profiles/custom'
            profile.mkdir(parents=True)
            (profile/'params.yaml').write_text('tuning: custom\n')
            actions = self.assemble(root, config)
            driver = actions[0]['node']['additional_env']
            self.assertEqual(driver['NSL_CAMERA_ID'], 'N010005C4')
            self.assertEqual(driver['NSL_PROFILE_DIR'], str(profile))
            self.assertEqual(driver['NSL_PARAMS_FILE'], str(profile/'params.yaml'))
            self.assertEqual(driver['NSL_CAMERA_IP'], '192.168.2.202')
            self.assertEqual(driver['NSL_FRAME_ID'], 'cam_161_lidar_frame')
            commands = [action['process']['cmd'] for action in actions if 'process' in action]
            self.assertEqual(len(commands), 5)
            for command in commands:
                if command[1].endswith('camera_device_info_node.py'):
                    continue
                self.assertEqual(command[command.index('--profile-dir') + 1], str(profile))
                self.assertEqual(command[command.index('--camera-id') + 1], 'N010005C4')

    def test_partial_download_retains_serial_tuning_intrinsic_and_extrinsic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.configuration(root)
            profile = root/'profiles/custom'
            profile.mkdir(parents=True)
            (profile/'multiview.yml').write_text('updated: multiview\n')
            legacy = root/'calibration/N010005C4'
            legacy.mkdir(parents=True)
            for name in ('params.yaml', 'intrinsic.yml', 'extrinsic.yml'):
                (legacy/name).write_text('existing: preserved\n')
            actions = self.assemble(root, config)
            env = actions[0]['node']['additional_env']
            self.assertEqual(env['NSL_PROFILE_DIR'], str(profile))
            self.assertEqual(env['NSL_PARAMS_FILE'], str(legacy/'params.yaml'))
            self.assertEqual(compact.profile_file(profile, legacy.parent, 'N010005C4', 'intrinsic.yml'), legacy/'intrinsic.yml')
            self.assertEqual(compact.profile_file(profile, legacy.parent, 'N010005C4', 'extrinsic.yml'), legacy/'extrinsic.yml')
            node = tf_module.MultiviewTfNode.__new__(tf_module.MultiviewTfNode)
            node.profile_dir, node.calib_dir, node.camera_filter = profile, legacy.parent, 'N010005C4'
            self.assertEqual(node._profile_file(profile, 'multiview.yml'), profile/'multiview.yml')
            self.assertEqual(node._profile_file(profile, 'extrinsic.yml'), legacy/'extrinsic.yml')
            self.assertIn(str(legacy/'extrinsic.yml'), node._watch_mtimes())

    def test_observation_only_initial_folder_uses_physical_serial_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.configuration(root, serial=None)
            profile = root/'profiles/custom'
            profile.mkdir(parents=True)
            (profile/'observed.yaml').write_text('intrinsic_rgb: null\n')
            serial, selected = compact.camera_runtime('cam_161', 'N010005C4', root/'calibration', config)
            self.assertEqual(serial, 'N010005C4')
            self.assertEqual(selected, str(root/'calibration/N010005C4'))

    def test_replaced_physical_serial_uses_new_profile_and_discovery_can_start(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.configuration(root)
            old = root/'profiles/custom'
            old.mkdir(parents=True)
            (old/'params.yaml').write_text('old_camera: true\n')
            replacement = root/'calibration/N00C705C3'
            replacement.mkdir(parents=True)
            (replacement/'params.yaml').write_text('new_camera: true\n')
            actions = self.assemble(root, config, detected='N00C705C3')
            env = actions[0]['node']['additional_env']
            self.assertEqual(env['NSL_CAMERA_ID'], 'N00C705C3')
            self.assertEqual(env['NSL_PARAMS_FILE'], str(replacement/'params.yaml'))
            self.assertTrue(any('camera_device_info_node.py' in str(action) for action in actions))

    def test_explicit_profile_launch_argument_wins(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.configuration(root)
            profile = root/'explicit'
            profile.mkdir()
            (profile/'params.yaml').write_text('explicit: true\n')
            actions = self.assemble(root, config, overrides={'profile_dir': str(profile)})
            self.assertEqual(actions[0]['node']['additional_env']['NSL_PARAMS_FILE'], str(profile/'params.yaml'))

    def test_configured_camera_serial_supports_ethernet_without_usb(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.configuration(root)
            serial, profile = compact.camera_runtime('cam_161', '', root/'calibration', config)
            self.assertEqual(serial, 'N010005C4')
            self.assertEqual(profile, str(root/'calibration/N010005C4'))

    def test_anchor_config_reaches_edge_and_host_origin_choices(self):
        observations = [{'tags': {0: {}, 7: {}, 3: {}, 9: {}}}]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.configuration(root)
            edge = calib.MultiviewCalibNode.__new__(calib.MultiviewCalibNode)
            edge.tags = {value: {'rv': [1] * 5} for value in (0, 7, 3, 9)}
            edge.a = SimpleNamespace(marker_id=-1, min_frames=5)
            for ids, edge_expected, host_expected in [([0, 7], 7, 0), ([9, 3], 9, 9), (3, 3, 3)]:
                contents = yaml.safe_load(config.read_text())
                contents['anchor']['id'] = ids
                config.write_text(yaml.safe_dump(contents))
                allowed = compact.anchor_ids(config)
                edge.anchor_ids = allowed
                self.assertEqual(edge._ref_id(), edge_expected)
                host = solver.BundleSolver(allowed_anchor_ids=allowed)
                self.assertEqual(host._anchor_id(observations, -1), host_expected)
                self.assertEqual(host._anchor_id(observations, 7), 7)
            edge.a.marker_id = 0
            self.assertEqual(edge._ref_id(), 0)

    def test_calibration_accepts_ids_zero_through_nine_independently_of_saved_map(self):
        allowed = compact.calibration_marker_ids()
        self.assertEqual(allowed, list(range(10)))
        edge = calib.MultiviewCalibNode.__new__(calib.MultiviewCalibNode)
        edge.anchor_ids = allowed
        edge.a = SimpleNamespace(marker_id=-1, min_frames=5)
        for marker in range(10):
            edge.tags = {marker: {'rv': [1] * 10}}
            self.assertEqual(edge._ref_id(), marker)
            host = solver.BundleSolver(allowed_anchor_ids=allowed)
            self.assertEqual(host._anchor_id([{'tags': {marker: {}}}], -1), marker)
            self.assertEqual(calib._size_of(marker), 0.32 if marker == 7 else 0.19)
        edge.tags = {13: {'rv': [1] * 10}}
        self.assertIsNone(edge._ref_id())

    def test_standalone_helper_does_not_import_sirius_config(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory)/'legacy.yaml'
            config.write_text('schema_version: 1\n')
            with patch.dict(sys.modules, {'sirius_config': None}):
                serial, profile = compact.camera_runtime('cam_161', 'N010005C4', config.parent, config)
            self.assertEqual(serial, 'N010005C4')
            self.assertEqual(profile, str(config.parent/'N010005C4'))


if __name__ == '__main__':
    unittest.main()
