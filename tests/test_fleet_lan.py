import ast
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from unittest.mock import patch


SETUP = Path(__file__).resolve().parents[1]/'setup'
EXEC = SETUP
LAUNCH = SETUP.parent / 'roboscan_nsl3130/launch'


class FleetLanTests(unittest.TestCase):
    def run_shell(self, script, address='192.168.3.166', config=None):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            ip = home / 'ip'
            ip.write_text('#!/bin/bash\nprintf "%s\\n" "$TEST_IP_OUTPUT"\n')
            ip.chmod(0o755)
            (home / '.edges.conf').write_text(config if config is not None else
                'Edge 192.168.3.161\n    id sb\n    project ~/SiriusRobotDT\n'
                'Edge 192.168.3.162\n    id sb\n'
            )
            env = {k: v for k, v in os.environ.items()
                   if not k.startswith(('NSL_', 'ROS_DISCOVERY_', 'ROS_SUPER_', 'FASTRTPS_', 'FASTDDS_'))}
            env.update(HOME=directory, PATH=directory + os.pathsep + env['PATH'],
                       TEST_IP_OUTPUT=f'2: usb inet 192.168.0.96/24\n3: eno1 inet {address}/24',
                       TEST_SETUP=str(SETUP), TEST_EXEC=str(EXEC), TEST_PING_LOG=str(home/'ping-calls'))
            return subprocess.run(['bash', '-e', '-c', script], env=env,
                                  capture_output=True, text=True, timeout=5)

    def test_host_dds_uses_new_lan_despite_old_usb_address(self):
        result = self.run_shell('''
            source "$TEST_SETUP/setup_dds_interface.bash"
            test "$ROS_DISCOVERY_SERVER" = 192.168.3.166:11811
            test "$ROS_SUPER_CLIENT" = TRUE
        ''')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_edge_dds_is_regular_client(self):
        result = self.run_shell('''
            source "$TEST_SETUP/setup_dds_interface.bash"
            test "$ROS_DISCOVERY_SERVER" = 192.168.3.166:11811
            test -z "${ROS_SUPER_CLIENT:-}"
        ''', address='192.168.3.162')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_fleet_config_and_ssh_targets(self):
        result = self.run_shell('''
            source "$TEST_EXEC/fleet_tools.sh"
            test "$(_nsl_self_ip)" = 192.168.3.166
            test "$(_nsl_config_edges | tr '\\n' ' ')" = '161 162 '
            test "$(_nsl_target 161)" = sb@192.168.3.161
            test "$(_nsl_target 162)" = sb@192.168.3.162
            test "$(_nsl_edge_project 161)" = '~/SiriusRobotDT'
        ''')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_legacy_config_falls_back_to_current_range_without_network_calls(self):
        result = self.run_shell('''
            source "$TEST_EXEC/fleet_tools.sh"
            ping() { printf "%s\\n" "${@: -1}" >> "$TEST_PING_LOG"; }
            test -z "$(_nsl_config_edges)"
            test "$(_nsl_edges | tr '\\n' ' ')" = '161 162 163 164 '
            test "$(sort -u "$TEST_PING_LOG" | tr '\\n' ' ')" = '192.168.3.161 192.168.3.162 192.168.3.163 192.168.3.164 '
        ''', config='Edge 192.168.3.52\n    id legacy_one\nEdge 53\n    id legacy_two\n')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_mixed_credentials_do_not_limit_discovery_without_network_calls(self):
        result = self.run_shell('''
            source "$TEST_EXEC/fleet_tools.sh"
            ping() { printf "%s\\n" "${@: -1}" >> "$TEST_PING_LOG"; }
            test "$(_nsl_config_edges | tr '\\n' ' ')" = '161 163 '
            test "$(_nsl_edges | tr '\\n' ' ')" = '161 162 163 164 '
            test "$(sort -u "$TEST_PING_LOG" | tr '\\n' ' ')" = '192.168.3.161 192.168.3.162 192.168.3.163 192.168.3.164 '
        ''', config='Edge 52\nEdge 192.168.3.161\nEdge 192.168.3.163\nEdge 192.168.3.166\nEdge 192.168.3.1610\n')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_launch_detection_ignores_other_subnets(self):
        for path in LAUNCH.glob('*.launch.py'):
            tree = ast.parse(path.read_text())
            functions = [n for n in ast.walk(tree)
                         if isinstance(n, ast.FunctionDef) and n.name == '_detect_ip_octet']
            for function in functions:
                with self.subTest(launch=path.name):
                    scope = {'subprocess': subprocess, 're': re}
                    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), scope)
                    with patch.object(subprocess, 'check_output', return_value=(
                        'inet 192.168.0.96/24\ninet 192.168.2.103/24\ninet 192.168.3.162/24'
                    )):
                        self.assertEqual(scope['_detect_ip_octet'](), '162')
                    with patch.object(subprocess, 'check_output', return_value='inet 192.168.0.53/24'):
                        self.assertEqual(scope['_detect_ip_octet'](), '')


if __name__ == '__main__':
    unittest.main()
