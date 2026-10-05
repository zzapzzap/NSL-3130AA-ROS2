"""DDS environment selection without starting DDS or touching real interfaces."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve().parents[1] / 'setup/setup_dds_interface.bash'
DDS_KEYS = ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT',
            'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE')


class DdsInterfaceTests(unittest.TestCase):
    def run_setup(self, addresses, mode='server'):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            ip = home / 'ip'
            ip.write_text('#!/bin/bash\nprintf "%s\\n" "$TEST_IP_OUTPUT"\n')
            ip.chmod(0o755)
            env = {key: value for key, value in os.environ.items()
                   if not key.startswith(('NSL_', 'ROS_DISCOVERY_', 'ROS_SUPER_',
                                          'FASTRTPS_', 'FASTDDS_'))}
            env.update(HOME=directory, PATH=directory + os.pathsep + env['PATH'],
                       TEST_IP_OUTPUT='\n'.join(
                           f'{index}: eth{index} inet {address}/24 scope global eth{index}'
                           for index, address in enumerate(addresses, 1)),
                       NSL_DDS_MODE=mode, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='42',
                       ROS_DISCOVERY_SERVER='192.168.3.52:11811', ROS_SUPER_CLIENT='TRUE',
                       FASTRTPS_DEFAULT_PROFILES_FILE='/old/fastdds.xml',
                       FASTDDS_DEFAULT_PROFILES_FILE='/old/alternate.xml',
                       CYCLONEDDS_URI='untouched-external-config')
            code = '\n'.join((
                'source "$1"',
                'python3 - <<\'PY\'',
                'import json, os',
                'from pathlib import Path',
                f'keys = {DDS_KEYS + ("ROS_LOCALHOST_ONLY", "ROS_DOMAIN_ID", "CYCLONEDDS_URI")!r}',
                'profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")',
                'xml = Path(profile).read_text() if profile and Path(profile).is_file() else None',
                'print(json.dumps({"env": {key: os.environ.get(key) for key in keys}, "xml": xml}))',
                'PY',
            ))
            result = subprocess.run(['bash', '-eu', '-c', code, 'test-dds', str(SCRIPT)],
                                    env=env, capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            return json.loads(result.stdout), result.stderr

    def test_camera_edges_are_regular_clients(self):
        for octet in (161, 165):
            with self.subTest(octet=octet):
                result, warning = self.run_setup([f'192.168.3.{octet}'])
                self.assertEqual(result['env']['ROS_DISCOVERY_SERVER'], '192.168.3.166:11811')
                self.assertIsNone(result['env']['ROS_SUPER_CLIENT'])
                self.assertEqual(result['env']['ROS_LOCALHOST_ONLY'], '0')
                self.assertIsNone(result['env']['FASTRTPS_DEFAULT_PROFILES_FILE'])
                self.assertIsNone(result['env']['FASTDDS_DEFAULT_PROFILES_FILE'])
                self.assertEqual(warning, '')

    def test_main_and_reserved_hosts_receive_complete_graph(self):
        for octet in (166, 169):
            with self.subTest(octet=octet):
                result, _ = self.run_setup([f'192.168.3.{octet}'])
                self.assertEqual(result['env']['ROS_SUPER_CLIENT'], 'TRUE')
                self.assertEqual(result['env']['ROS_DISCOVERY_SERVER'], '192.168.3.166:11811')

    def test_outside_range_clears_stale_fleet_settings_preserving_offline_scope(self):
        for address in ('192.168.3.160', '192.168.3.170', '192.168.2.161',
                        '192.168.3.1610'):
            with self.subTest(address=address):
                result, warning = self.run_setup([address])
                self.assertTrue(all(result['env'][key] is None for key in DDS_KEYS))
                self.assertEqual(result['env']['ROS_LOCALHOST_ONLY'], '1')
                self.assertEqual(result['env']['ROS_DOMAIN_ID'], '42')
                self.assertEqual(result['env']['CYCLONEDDS_URI'], 'untouched-external-config')
                self.assertIsNone(result['xml'])
                self.assertIn('192.168.3.161-169', warning)

    def test_later_valid_interface_is_selected_after_excluded_addresses(self):
        result, warning = self.run_setup(
            ['192.168.2.103', '192.168.3.170', '192.168.3.160', '192.168.3.169'])
        self.assertEqual(result['env']['ROS_SUPER_CLIENT'], 'TRUE')
        self.assertEqual(result['env']['ROS_LOCALHOST_ONLY'], '0')
        self.assertEqual(warning, '')

    def test_multicast_profile_pins_the_selected_valid_local_address(self):
        for address in ('192.168.3.161', '192.168.3.169'):
            with self.subTest(address=address):
                result, warning = self.run_setup(
                    ['192.168.2.161', '192.168.3.170', address], mode='multicast')
                profile = ET.fromstring(result['xml'])
                namespace = {'dds': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
                whitelist = profile.findall('.//dds:interfaceWhiteList/dds:address', namespace)
                self.assertEqual([entry.text for entry in whitelist], [address])
                self.assertEqual(profile.find('.//dds:useBuiltinTransports', namespace).text, 'false')
                self.assertEqual(result['env']['ROS_LOCALHOST_ONLY'], '0')
                self.assertIsNone(result['env']['ROS_DISCOVERY_SERVER'])
                self.assertIsNone(result['env']['ROS_SUPER_CLIENT'])
                self.assertIsNone(result['env']['FASTDDS_DEFAULT_PROFILES_FILE'])
                self.assertEqual(warning, '')

    def test_unknown_mode_drops_stale_profile_without_enabling_lan(self):
        result, warning = self.run_setup(['192.168.3.166'], mode='invalid')
        self.assertTrue(all(result['env'][key] is None for key in DDS_KEYS))
        self.assertEqual(result['env']['ROS_LOCALHOST_ONLY'], '1')
        self.assertIn('unknown NSL_DDS_MODE', warning)


if __name__ == '__main__':
    unittest.main()
