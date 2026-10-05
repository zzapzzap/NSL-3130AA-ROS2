"""The shared manifest points at native camera settings; writes stay in that module."""
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'controller/src/NSL-3130AA-ROS2/setup'))
sys.path.insert(0, str(ROOT/'controller/src/NSL-3130AA-ROS2/roboscan_nsl3130/scripts'))
sys.path.insert(0, str(ROOT/'controller/src/robot_dt_bridge/config'))
import camera_network as network
import sirius_camera_config as camera
import camera_calibration_pull as pull


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
                self.assertEqual(camera.inventory()[1], fleet)
                self.assertEqual(camera.camera_runtime('cam_161', '', profile.parent), ('SERIAL_NATIVE', str(profile)))
                self.assertEqual(camera.anchor_ids(), [13])

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
                self.assertEqual(camera.camera_runtime('cam_161', '', profile.parent)[0], 'EXPLICIT')
                self.assertEqual(camera.inventory()[1], override)

    def test_pull_resolves_manifest_and_preserves_both_inventory_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, fleet, profile = self.fixture(root)
            before_manifest, before_fleet = manifest.read_bytes(), fleet.read_bytes()
            def fake_scp(argv, **kwargs):
                self.assertIn('192.168.3.161:~/SiriusRobotDT/controller/src/NSL-3130AA-ROS2/calib_output/SERIAL_NATIVE/*.y*ml', argv)
                (Path(argv[-1])/'multiview.yml').write_text('namespace: cam_161\n')
            with patch.object(pull.subprocess, 'run', side_effect=fake_scp), \
                 patch.dict(os.environ, {'SSHPASS': ''}):
                result = pull.pull_profile(manifest, 161)
            self.assertEqual(result['files'], ['multiview.yml'])
            self.assertEqual(manifest.read_bytes(), before_manifest)
            self.assertEqual(fleet.read_bytes(), before_fleet)
            self.assertEqual((profile/'params.yaml').read_text(), 'original: tuning\n')
            self.assertTrue((profile.parent/'runtime/profiles.yaml').is_file())


if __name__ == '__main__':
    unittest.main()
