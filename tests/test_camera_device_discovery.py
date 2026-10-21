"""Use fake USB sysfs and raw profiles; no camera/network access is needed."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from ros_humanpose.srv import GetWeight

import yaml

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT/'controller/src/NSL-3130AA-ROS2/roboscan_nsl3130/scripts'
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location('device_discovery_test', SCRIPTS/'camera_device_info_node.py')
device = importlib.util.module_from_spec(spec)
spec.loader.exec_module(device)


class CameraDiscoveryTests(unittest.TestCase):
    def test_native_inventory_uses_inline_cache_and_canonical_serial_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            module = root/'controller/src/NSL-3130AA-ROS2'
            (module/'setup').mkdir(parents=True)
            config = module/'setup/fleet.yaml'
            config.write_text(yaml.safe_dump({'schema_version': 1,
                'server': {'ip': '192.168.3.166', 'domain_id': 42},
                'cameras': {'cam_161': {'ip': '192.168.3.161', 'serial': 'OLD_SERIAL',
                                      'serial_verified': True, 'parameters': '../calib_output/OLD_SERIAL'}}}))
            usb = root/'usb'
            entry = usb/'0'
            entry.mkdir(parents=True)
            for key, value in {'idVendor': '1fc9', 'idProduct': '0099', 'serial': 'NEW_SERIAL'}.items():
                (entry/key).write_text(value+'\n')
            profile = module/'calib_output/NEW_SERIAL'
            profile.mkdir(parents=True)
            (profile/'intrinsic.yml').write_text('actual_camera: true\n')
            with patch.dict(os.environ, {'SIRIUS_CONFIG': '', 'SIRIUS_ROBOT_DT_ROOT': str(root)}):
                report = device.describe_device('cam_161', profile.parent, usb_sys=usb)
            self.assertTrue(report['verified'])
            self.assertEqual(report['serial'], 'NEW_SERIAL')
            self.assertEqual(report['cached_serial'], 'OLD_SERIAL')
            self.assertEqual(report['profile_dir'], str(profile))
            self.assertFalse((config.parent/'cameras').exists())

    def fixture(self, root, serials=('NEW_SERIAL',)):
        config = root/'config.yaml'
        config.write_text(yaml.safe_dump({'schema_version': 2,
            'cameras': {'cam_161': {'serial': 'OLD_SERIAL', 'parameters': 'profiles/old'}},
            'anchor': {'id_7': {'tf': None}, 'id_0': {'tf': None}}}))
        cache = root/'cameras/cam_161.yaml'
        cache.parent.mkdir()
        cache.write_text('serial: OLD_SERIAL\nserial_verified: true\nserial_source: usb_sysfs\n')
        usb = root/'usb'
        usb.mkdir()
        for index, serial in enumerate(serials):
            entry = usb/str(index)
            entry.mkdir()
            (entry/'idVendor').write_text('1fc9\n')
            (entry/'idProduct').write_text('0099\n')
            (entry/'serial').write_text(serial+'\n')
        profile = root/'calibration/NEW_SERIAL'
        profile.mkdir(parents=True)
        (profile/'intrinsic.yml').write_bytes(b'%YAML:1.0\n---\nactual_profile: true\n')
        old = root/'profiles/old'
        old.mkdir(parents=True)
        (old/'intrinsic.yml').write_text('stale_profile: true\n')
        return config, usb, profile

    def test_physical_replacement_discovery_returns_new_serial_and_its_raw_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, usb, profile = self.fixture(root)
            original = config.read_bytes()
            report = device.describe_device('cam_161', profile.parent, config, usb)
            self.assertTrue(report['verified'])
            self.assertEqual(report['serial'], 'NEW_SERIAL')
            self.assertEqual(report['source'], 'usb_sysfs')
            self.assertEqual(report['cached_serial'], 'OLD_SERIAL')
            self.assertEqual(report['profile_dir'], str(profile))
            metadata = report['files']['intrinsic.yml']
            self.assertEqual(metadata['sha256'], hashlib.sha256((profile/'intrinsic.yml').read_bytes()).hexdigest())
            self.assertEqual(metadata['file'], str(profile/'intrinsic.yml'))
            self.assertEqual(config.read_bytes(), original)

    def test_ethernet_cache_is_not_reported_as_newly_verified_physical_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, usb, profile = self.fixture(root, serials=())
            report = device.describe_device('cam_161', profile.parent, config, usb)
            self.assertFalse(report['verified'])
            self.assertIsNone(report['serial'])
            self.assertEqual(report['cached_serial'], 'OLD_SERIAL')
            self.assertEqual(report['files'], {})
            self.assertIn('No live USB or Ethernet SDK identity', report['reason'])

    def test_live_ethernet_ids_select_actual_profile_without_usb(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, usb, profile = self.fixture(root, serials=())
            actual = profile.parent/'N010005C4'
            actual.mkdir()
            (actual/'intrinsic.yml').write_bytes(b'%YAML:1.0\n---\nactual_camera: true\n')
            observed = {'serial': 'N010005C4', 'chip_id': 0x100, 'wafer_id': 0x5c4,
                        'source': 'ethernet_sdk_ids', 'sensor_host': '192.168.2.201'}
            before = config.read_bytes()
            report = device.describe_device('cam_161', profile.parent, config, usb, observed)
            self.assertTrue(report['verified'])
            self.assertEqual(report['source'], 'ethernet_sdk_ids')
            self.assertEqual(report['serial'], 'N010005C4')
            self.assertEqual(report['profile_dir'], str(actual))
            self.assertEqual(report['files']['intrinsic.yml']['file'], str(actual/'intrinsic.yml'))
            self.assertEqual(config.read_bytes(), before)

    def test_ethernet_serial_must_match_live_ids_and_sensor_address(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, usb, profile = self.fixture(root, serials=())
            observed = {'serial': 'WRONG_SERIAL', 'chip_id': 0x100, 'wafer_id': 0x5c4,
                        'source': 'ethernet_sdk_ids', 'sensor_host': '192.168.2.201'}
            report = device.describe_device('cam_161', profile.parent, config, usb, observed)
            self.assertFalse(report['verified'])
            observed['serial'] = 'N010005C4'
            value = yaml.safe_load(config.read_text())
            value['cameras']['cam_161']['sensor_host'] = '192.168.2.202'
            config.write_text(yaml.safe_dump(value))
            report = device.describe_device('cam_161', profile.parent, config, usb, observed)
            self.assertFalse(report['verified'])
            self.assertIn('different sensor IP', report['reason'])

    def test_multiple_physical_devices_require_unambiguous_binding(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, usb, profile = self.fixture(root, serials=('FIRST', 'SECOND'))
            report = device.describe_device('cam_161', profile.parent, config, usb)
            self.assertFalse(report['verified'])
            self.assertIsNone(report['serial'])
            self.assertIn('ambiguous', report['reason'])

    def test_raw_profile_service_chunks_preserve_hash_and_reject_other_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, usb, profile = self.fixture(root)
            report = device.describe_device('cam_161', profile.parent, config, usb)
            node = device.CameraDeviceInfoNode.__new__(device.CameraDeviceInfoNode)
            node._report = lambda: report
            assembled = b''
            while len(assembled) < report['files']['intrinsic.yml']['size']:
                request = SimpleNamespace(name='intrinsic.yml', offset=len(assembled), max_bytes=7)
                response = node._on_get(request, SimpleNamespace())
                self.assertTrue(response.success)
                self.assertEqual(response.sha256, report['files']['intrinsic.yml']['sha256'])
                assembled += response.data
            self.assertEqual(assembled, (profile/'intrinsic.yml').read_bytes())
            response = node._on_info(None, SimpleNamespace())
            self.assertTrue(response.success)
            self.assertEqual(json.loads(response.message)['profile_get_service'], '/cam_161/camera/profile/get')
            refused = node._on_get(SimpleNamespace(name='../config.yaml', offset=0, max_bytes=7), SimpleNamespace())
            self.assertFalse(refused.success)

    def test_anchor_ids_derive_new_marker_instance_keys(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, _, _ = self.fixture(root)
            from sirius_camera_config import anchor_ids
            self.assertEqual(anchor_ids(config), [0, 7])  # Fixture SafeDumper sorts its marker keys.

    def test_real_ros_services_serialize_identity_and_raw_profile_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, usb, profile = self.fixture(root)
            report = device.describe_device('cam_161', profile.parent, config, usb)
            environment = {key: value for key, value in os.environ.items()
                           if key not in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT',
                                          'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE')}
            environment.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='217')
            with patch.dict(os.environ, environment, clear=True), \
                 patch.object(device.CameraDeviceInfoNode, '_report', return_value=report):
                rclpy.init(domain_id=217)
                server = device.CameraDeviceInfoNode('cam_161', str(profile.parent))
                client = rclpy.create_node('camera_discovery_service_test')
                executor = SingleThreadedExecutor()
                executor.add_node(server)
                executor.add_node(client)
                try:
                    info = client.create_client(Trigger, '/cam_161/camera/device_info')
                    raw = client.create_client(GetWeight, '/cam_161/camera/profile/get')
                    self.assertTrue(info.wait_for_service(timeout_sec=3))
                    self.assertTrue(raw.wait_for_service(timeout_sec=3))
                    future = info.call_async(Trigger.Request())
                    executor.spin_until_future_complete(future, timeout_sec=3)
                    self.assertTrue(future.result().success)
                    self.assertEqual(json.loads(future.result().message)['serial'], 'NEW_SERIAL')
                    request = GetWeight.Request(name='intrinsic.yml', offset=0, max_bytes=65536)
                    future = raw.call_async(request)
                    executor.spin_until_future_complete(future, timeout_sec=3)
                    self.assertTrue(future.result().success)
                    self.assertEqual(bytes(future.result().data), (profile/'intrinsic.yml').read_bytes())
                    self.assertEqual(future.result().sha256, report['files']['intrinsic.yml']['sha256'])
                finally:
                    executor.shutdown()
                    client.destroy_node()
                    server.destroy_node()
                    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
