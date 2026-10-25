"""Setup stays opt-in; replace OS/network/build commands with argument-recording stubs."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[4]
SETUP = ROOT/'setup/setup.bash'


class OneTouchSetupTests(unittest.TestCase):
    def run_setup(self, directory, *args, extra_env=None):
        temporary = Path(directory)
        commands = temporary/'bin'
        commands.mkdir(exist_ok=True)
        calls = temporary/'calls'
        for name in ('sudo', 'ssh', 'sshpass', 'scp', 'rosdep', 'colcon', 'systemctl'):
            file = commands/name
            file.write_text('#!/usr/bin/env bash\nprintf -v arguments " <%s>" "$@"\nprintf "%s%s\\n" "${0##*/}" "$arguments" >> "$TEST_SETUP_CALLS"\n')
            if name == 'ssh':
                file.write_text(file.read_text()+'if [[ "${@: -1}" == "id -un" ]]; then echo keyedge; fi\nexit "${TEST_SSH_STATUS:-0}"\n')
            file.chmod(0o755)
        (commands/'ip').write_text('#!/bin/sh\necho "2: eth0 inet 192.168.3.166/24"\n')
        (commands/'ping').write_text('#!/bin/sh\nif [ -n "$TEST_ONLINE_IPS" ]; then\n for value in "$@"; do address="$value"; done\n case " $TEST_ONLINE_IPS " in *" $address "*) exit 0 ;; *) exit 1 ;; esac\nfi\nexit "${TEST_PING_STATUS:-0}"\n')
        (commands/'ip').chmod(0o755)
        (commands/'ping').chmod(0o755)
        (temporary/'.edges.conf').write_text('Edge 192.168.3.161\n    id testedge\n')
        env = os.environ.copy()
        env.update(HOME=directory, PATH=str(commands)+os.pathsep+env['PATH'], TEST_SETUP_CALLS=str(calls),
                   SIRIUS_ROBOT_DT_ROOT=str(ROOT), SIRIUS_UNIT_OUTPUT=str(temporary/'sensor_streaming.service'),
                   NSL_FLEET_CONFIG=str(temporary/'.edges.conf'), NSL_FLEET_USER='', USER='testuser')
        env.update(extra_env or {})
        result = subprocess.run(['bash', str(SETUP), *args], env=env, capture_output=True, text=True, timeout=10)
        return result, calls.read_text().splitlines() if calls.is_file() else []

    def test_check_detects_host_without_installing_or_starting(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--check', '--ip', '192.168.3.166')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(calls, [])
            self.assertIn('Role: host', result.stdout)
            self.assertFalse((Path(directory)/'.bashrc').exists())

    def test_check_detects_edge_without_invoking_vendor_setup(self):
        for octet in (162, 165):
            with self.subTest(octet=octet), tempfile.TemporaryDirectory() as directory:
                result, calls = self.run_setup(directory, '--check', '--ip', f'192.168.3.{octet}')
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(calls, [])
                self.assertIn('Role: edge', result.stdout)

    def test_automatic_host_flow_installs_shell_and_only_main_unit_with_stubs(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--ip', '192.168.3.166')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((Path(directory)/'.bashrc').is_file())
            enable = 'sudo <systemctl> <enable> <--now> <sensor_streaming.service>'
            self.assertIn(enable, calls)

    def test_unknown_lan_address_requires_a_role(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--check', '--ip', '192.168.3.52')
            self.assertEqual(result.returncode, 2)
            self.assertEqual(calls, [])

    def test_automatic_edge_flow_delegates_vendor_setup_with_stubs(self):
        with tempfile.TemporaryDirectory() as directory:
            vendor = Path(directory)/'vendor-edge.bash'
            vendor.write_text('printf "vendor <%s> <%s>\\n" "$NSL_FLEET_ROLE" "$NSL_SETUP_IP" >> "$TEST_SETUP_CALLS"\n')
            result, calls = self.run_setup(directory, '--ip', '192.168.3.161',
                extra_env={'SIRIUS_EDGE_SETUP_SCRIPT': str(vendor)})
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((Path(directory)/'.bashrc').is_file())
            self.assertEqual(calls[0], 'vendor <edge> <192.168.3.161>')
            self.assertIn('NSL_ENABLE_POSE=true', '\n'.join(calls))
            self.assertIn('<systemctl> <start> <nsl-train-listener@testuser.service>', calls[-1])
            self.assertFalse((Path(directory)/'sensor_streaming.service').exists())

    def test_service_preview_generates_only_main_unit(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--service')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(calls, [])
            unit = (Path(directory)/'sensor_streaming.service').read_text()
            self.assertIn('ros2 launch sirius_robot_dt main.launch.py', unit)
            self.assertIn(str(ROOT/'setup/fleet_env.bash'), unit)
            self.assertIn('ExecStopPost=', unit)
            self.assertIn('--stop-edges', unit)
            self.assertIn('ExecStartPre=', unit)
            self.assertIn('roboscan_publish_node', unit)
            self.assertNotIn('nsl-edge-agent', unit)

    def test_explicit_service_install_start_calls_only_main_unit(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--service-install-start')
            self.assertEqual(result.returncode, 0, result.stderr)
            enable = 'sudo <systemctl> <enable> <--now> <sensor_streaming.service>'
            self.assertIn(enable, calls)

    def test_module_selects_build_graph_without_starting_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--module', 'camera')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(len(calls), 2)
            self.assertTrue(calls[0].startswith('rosdep '))
            self.assertIn('<build> <--symlink-install> <--packages-up-to> <roboscan_nsl3130> <rqt_reconfigure_combo>', calls[1])

    def test_start_edge_only_starts_already_installed_agent(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--start-edge', '161')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(any(call.startswith('scp ') for call in calls))
            self.assertTrue(calls[-1].startswith('ssh '))
            self.assertIn('nsl-edge-agent@testedge.service', calls[-1])
            self.assertNotIn('apt-get', '\n'.join(calls))

    def test_start_edge_can_select_camera_only_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--start-edge', '161', '--pose', 'false')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(calls[0].startswith('ssh '))
            self.assertIn('NSL_ENABLE_POSE=false', '\n'.join(calls))

    def test_pose_config_uses_the_edge_project_path_without_host_username(self):
        with tempfile.TemporaryDirectory() as directory:
            config=ROOT/'controller/src/ros_humanpose/config/config.yml'
            result,calls=self.run_setup(directory,'--start-edge','161','--pose-config',str(config))
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('NSL_POSE_CONFIG=controller/src/ros_humanpose/config/config.yml','\n'.join(calls))
            self.assertNotIn(str(ROOT), calls[-1])  # remote service paths stay module-relative

    def test_external_pose_config_is_rejected_before_remote_calls(self):
        with tempfile.TemporaryDirectory() as directory:
            result,calls=self.run_setup(directory,'--start-edge','161','--pose-config','/tmp/unbound-pose.yaml')
            self.assertNotEqual(result.returncode,0)
            self.assertEqual(calls,[])
            self.assertIn('inside SiriusRobotDT',result.stderr)

    def test_stop_edges_calls_only_configured_online_edge_unit(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--stop-edges')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(calls[0].startswith('ssh '))
            rendered = '\n'.join(calls)
            self.assertIn('nsl-edge-agent@testedge.service', rendered)
            self.assertIn('systemctl', rendered)
            self.assertIn('ActiveState', rendered)
            self.assertNotIn('apt-get', rendered)

    def test_failed_ssh_stop_is_not_acknowledged(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--stop-edges', extra_env={'TEST_SSH_STATUS': '255'})
            self.assertEqual(result.returncode, 1)
            self.assertIn('state is not acknowledged', result.stderr)
            self.assertTrue(any(call.startswith('ssh ') for call in calls))
            self.assertNotIn('Stopped nsl-edge-agent', result.stdout)

    def test_stop_reaches_key_authenticated_edge_omitted_from_credentials(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--stop-edges',
                extra_env={'TEST_ONLINE_IPS': '192.168.3.161 192.168.3.163'})
            self.assertEqual(result.returncode, 0, result.stderr)
            rendered = '\n'.join(calls)
            self.assertIn('testedge@192.168.3.161', rendered)
            self.assertIn('192.168.3.163', rendered)
            self.assertIn('nsl-edge-agent@keyedge.service', rendered)
            self.assertNotIn('192.168.3.162', rendered)
            self.assertNotIn('192.168.3.164', rendered)
            self.assertNotIn('192.168.3.166', rendered)

    def test_offline_edges_are_not_claimed_stopped(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--stop-edges', extra_env={'TEST_PING_STATUS': '1'})
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(calls, [])
            self.assertIn('No online Edge acknowledged stop', result.stdout)
            self.assertNotIn('Stopped nsl-edge-agent', result.stdout)

    def test_invalid_module_never_invokes_system_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            result, calls = self.run_setup(directory, '--module', 'unknown')
            self.assertEqual(result.returncode, 2)
            self.assertEqual(calls, [])


if __name__ == '__main__':
    unittest.main()
