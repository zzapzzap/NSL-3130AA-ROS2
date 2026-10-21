"""Validate fleet interface selection and unicast-only profiles without networking."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

SETUP = Path(__file__).resolve().parents[1] / 'setup'
SCRIPT = SETUP/'setup_dds_interface.bash'
NS = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
KEYS = ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTRTPS_DEFAULT_PROFILES_FILE',
        'FASTDDS_DEFAULT_PROFILES_FILE', 'ROS_LOCALHOST_ONLY')


class DdsInterfaceTests(unittest.TestCase):
    def run_setup(self, addresses, mode='server', interactive=False):
        with tempfile.TemporaryDirectory() as directory:
            ip = Path(directory)/'ip'
            ip.write_text('#!/bin/bash\nprintf "%s\\n" "$TEST_IP_OUTPUT"\n')
            ip.chmod(0o755)
            env = dict(os.environ, HOME=directory, PATH=directory+os.pathsep+os.environ['PATH'],
                       TEST_IP_OUTPUT='\n'.join(f'{i}: eth{i} inet {a}/24' for i,a in enumerate(addresses,1)),
                       NSL_DDS_MODE=mode, ROS_LOCALHOST_ONLY='1', ROS_DISCOVERY_SERVER='old:11811',
                       ROS_SUPER_CLIENT='TRUE', FASTRTPS_DEFAULT_PROFILES_FILE='/old',
                       FASTDDS_DEFAULT_PROFILES_FILE='/old2')
            code = '''source "$1"
python3 - <<'CHECK'
import os,json
from pathlib import Path
keys=KEYS_PLACEHOLDER
p=os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
print(json.dumps({'env':{k:os.environ.get(k) for k in keys}, 'xml':Path(p).read_text() if p else None}))
CHECK
'''.replace('KEYS_PLACEHOLDER',repr(KEYS))
            result=subprocess.run(['bash','--noprofile','--norc','-eui' if interactive else '-eu','-c',code,'test',str(SCRIPT)],env=env,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            return json.loads(result.stdout),result.stderr

    def test_only_edges_and_main_are_dds_participants(self):
        for octet in (161,162,163,164,166):
            with self.subTest(octet=octet):
                data,warning=self.run_setup([f'192.168.3.{octet}'])
                self.assertEqual(warning,'')
                self.assertEqual(data['env']['ROS_DISCOVERY_SERVER'],'192.168.3.166:11811')
                self.assertEqual(data['env']['ROS_SUPER_CLIENT'],'TRUE' if octet==166 else None)
                self.assertIsNone(data['env']['FASTDDS_DEFAULT_PROFILES_FILE'])
                xml=ET.fromstring(data['xml'])
                self.assertEqual([x.text for x in xml.findall('.//d:interfaceWhiteList/d:address',NS)],[f'192.168.3.{octet}'])
                self.assertEqual(xml.find('.//d:useBuiltinTransports',NS).text,'false')
                self.assertIsNotNone(xml.find('.//d:metatrafficUnicastLocatorList/d:locator',NS))
                self.assertFalse(xml.findall('.//d:metatrafficMulticastLocatorList/d:locator',NS))
                self.assertEqual([x.text for x in xml.findall('.//d:initialPeersList//d:address',NS)],['192.168.3.166'])
                self.assertEqual(xml.find('.//d:maxMessageSize',NS).text,'1400')
                self.assertEqual(xml.find('.//d:sendBufferSize',NS).text,'16777216')

    def test_interactive_edge_terminal_gets_full_graph_for_cli_daemon(self):
        data,_=self.run_setup(['192.168.3.161'],interactive=True)
        self.assertEqual(data['env']['ROS_SUPER_CLIENT'],'TRUE')
        self.assertEqual(data['env']['ROS_DISCOVERY_SERVER'],'192.168.3.166:11811')

    def test_plcs_unused_hosts_and_other_subnets_are_excluded(self):
        for a in ('192.168.3.155','192.168.3.160','192.168.3.165','192.168.3.167',
                  '192.168.3.168','192.168.3.169','192.168.3.170','192.168.3.1610','10.8.0.67'):
            with self.subTest(address=a):
                data,warning=self.run_setup([a])
                self.assertIsNone(data['xml'])
                self.assertIn('skipped',warning)
                for key in KEYS[:-1]:self.assertIsNone(data['env'][key])
                self.assertEqual(data['env']['ROS_LOCALHOST_ONLY'],'1')

    def test_selects_lan_after_vpn_and_plc_addresses(self):
        data,_=self.run_setup(['10.8.0.67','192.168.3.169','192.168.3.166'])
        self.assertEqual(data['env']['ROS_SUPER_CLIENT'],'TRUE')

    def test_multicast_and_unknown_modes_cannot_restore_stale_settings(self):
        for mode in ('multicast','unknown'):
            data,warning=self.run_setup(['192.168.3.166'],mode)
            self.assertIsNone(data['xml'])
            self.assertIn('multicast DDS is disabled',warning)
            for key in KEYS[:-1]:self.assertIsNone(data['env'][key])

    def test_server_profile_has_fixed_identity_and_unicast_port(self):
        spec=importlib.util.spec_from_file_location('dds_profile',SETUP/'dds_profile.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        xml=ET.fromstring(module.profile_xml('192.168.3.166',True))
        self.assertEqual(xml.find('.//d:discoveryProtocol',NS).text,'SERVER')
        self.assertEqual(xml.find('.//d:prefix',NS).text,'44.53.00.5f.45.50.52.4f.53.49.4d.41')
        self.assertEqual(xml.find('.//d:metatrafficUnicastLocatorList//d:port',NS).text,'11811')
        with self.assertRaises(ValueError):module.profile_xml('192.168.3.170')


if __name__=='__main__':unittest.main()
