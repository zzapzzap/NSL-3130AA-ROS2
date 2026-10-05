"""Compact camera settings, explicit SCP, and preservation of verified profile files."""
import importlib.util
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'controller/src/robot_dt_bridge/config'))
sys.path.insert(0, str(ROOT/'controller/src/NSL-3130AA-ROS2/setup'))


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


network = load_module('compact_camera_network', ROOT/'controller/src/NSL-3130AA-ROS2/setup/camera_network.py')
pull = load_module('compact_camera_pull', ROOT/'controller/src/NSL-3130AA-ROS2/setup/camera_calibration_pull.py')


class CompactCameraTests(unittest.TestCase):
    def config(self, root):
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


if __name__ == '__main__':
    unittest.main()
