"""Exercise compact settings through real launch assembly without starting devices."""
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[4]
PACKAGE = ROOT/'controller/src/NSL-3130AA-ROS2/roboscan_nsl3130'
SCRIPTS = PACKAGE/'scripts'
sys.path.insert(0, str(SCRIPTS))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


compact = load('camera_inventory_runtime', SCRIPTS/'sirius_camera_config.py')
solver = load('camera_inventory_solver', SCRIPTS/'multiview_solver_node.py')
calib = load('camera_inventory_calib', SCRIPTS/'multiview_calib_node.py')
tf_module = load('camera_inventory_tf', SCRIPTS/'multiview_tf_node.py')


class CameraInventoryRuntimeTests(unittest.TestCase):
    def configuration(self, root, ids=(0, 7), serial='SERIAL_A'):
        config = root/'config.yaml'
        config.write_text(yaml.safe_dump({'schema_version': 2,
            'cameras': {'cam_161': {'ip': '192.168.3.161', 'serial': serial,
                                  'parameters': 'profiles/custom'}},
            'anchor': {'id': list(ids) if isinstance(ids, tuple) else ids}}))
        (root/'cameras').mkdir(exist_ok=True)
        (root/'cameras/cam_161.yaml').write_text('sensor_host: 192.168.2.202\n')
        return config

    def assemble(self, root, config, detected='SERIAL_A', overrides=None):
        module = load('camera_inventory_launch', PACKAGE/'launch/camera.launch.py')
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
        with patch.dict(os.environ, {'SIRIUS_CONFIG': str(config), 'SIRIUS_ROBOT_DT_ROOT': str(ROOT),
                                     'NSL_CALIB_DIR': str(root/'calibration'), 'HOME': str(root)}), \
             patch.object(module, 'get_package_share_directory', return_value=str(PACKAGE)), \
             patch.object(module, 'get_package_prefix', return_value=str(prefix)), \
             patch.object(module.subprocess, 'check_output', side_effect=fake_output), \
             patch.object(module, 'DeclareLaunchArgument', side_effect=declare), \
             patch.object(module, 'LaunchDescription', side_effect=lambda entities: entities), \
             patch.object(module, 'LaunchConfiguration', Configuration), \
             patch.object(module, 'OpaqueFunction', side_effect=lambda function: function), \
             patch.object(module, 'Node', side_effect=lambda **kwargs: {'node': kwargs}), \
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
            self.assertEqual(driver['NSL_CAMERA_ID'], 'SERIAL_A')
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
                self.assertEqual(command[command.index('--camera-id') + 1], 'SERIAL_A')

    def test_partial_download_retains_serial_tuning_intrinsic_and_extrinsic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.configuration(root)
            profile = root/'profiles/custom'
            profile.mkdir(parents=True)
            (profile/'multiview.yml').write_text('updated: multiview\n')
            legacy = root/'calibration/SERIAL_A'
            legacy.mkdir(parents=True)
            for name in ('params.yaml', 'intrinsic.yml', 'extrinsic.yml'):
                (legacy/name).write_text('existing: preserved\n')
            actions = self.assemble(root, config)
            env = actions[0]['node']['additional_env']
            self.assertEqual(env['NSL_PROFILE_DIR'], str(profile))
            self.assertEqual(env['NSL_PARAMS_FILE'], str(legacy/'params.yaml'))
            self.assertEqual(compact.profile_file(profile, legacy.parent, 'SERIAL_A', 'intrinsic.yml'), legacy/'intrinsic.yml')
            self.assertEqual(compact.profile_file(profile, legacy.parent, 'SERIAL_A', 'extrinsic.yml'), legacy/'extrinsic.yml')
            node = tf_module.MultiviewTfNode.__new__(tf_module.MultiviewTfNode)
            node.profile_dir, node.calib_dir, node.camera_filter = profile, legacy.parent, 'SERIAL_A'
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
            serial, selected = compact.camera_runtime('cam_161', 'SERIAL_A', root/'calibration', config)
            self.assertEqual(serial, 'SERIAL_A')
            self.assertEqual(selected, str(root/'calibration/SERIAL_A'))

    def test_replaced_physical_serial_uses_new_profile_and_discovery_can_start(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.configuration(root)
            old = root/'profiles/custom'
            old.mkdir(parents=True)
            (old/'params.yaml').write_text('old_camera: true\n')
            replacement = root/'calibration/OTHER_SERIAL'
            replacement.mkdir(parents=True)
            (replacement/'params.yaml').write_text('new_camera: true\n')
            actions = self.assemble(root, config, detected='OTHER_SERIAL')
            env = actions[0]['node']['additional_env']
            self.assertEqual(env['NSL_CAMERA_ID'], 'OTHER_SERIAL')
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
            self.assertEqual(serial, 'SERIAL_A')
            self.assertEqual(profile, str(root/'calibration/SERIAL_A'))

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

    def test_standalone_helper_does_not_import_sirius_config(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory)/'legacy.yaml'
            config.write_text('schema_version: 1\n')
            with patch.dict(sys.modules, {'sirius_config': None}):
                serial, profile = compact.camera_runtime('cam_161', 'SERIAL_A', config.parent, config)
            self.assertEqual(serial, 'SERIAL_A')
            self.assertEqual(profile, str(config.parent/'SERIAL_A'))


if __name__ == '__main__':
    unittest.main()
