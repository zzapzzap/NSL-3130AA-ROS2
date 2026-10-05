"""Test mode-file idempotence and camera-only startup using local command stubs."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[4]
SETUP = ROOT/'controller/src/NSL-3130AA-ROS2/setup'
sys.path.insert(0, str(SETUP))
from edge_pose_mode import edge_pose_command, edge_stop_command


class EdgePoseModeTests(unittest.TestCase):
    def test_stop_only_acknowledges_an_inactive_unit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root/'bin'
            binary.mkdir()
            systemctl = binary/'systemctl'
            systemctl.write_text('#!/bin/sh\nif [ "$1" = show ]; then echo "$TEST_ACTIVE_STATE"; fi\n')
            systemctl.chmod(0o755)
            env = dict(os.environ, PATH=str(binary)+os.pathsep+os.environ['PATH'], TEST_ACTIVE_STATE='inactive')
            command = edge_stop_command('testedge', 2)
            stopped = subprocess.run(['bash', '-c', command], env=env, capture_output=True, text=True)
            self.assertEqual(stopped.returncode, 0, stopped.stderr)
            self.assertIn('Stopped', stopped.stdout)
            env['TEST_ACTIVE_STATE'] = 'active'
            failed = subprocess.run(['bash', '-c', command], env=env, capture_output=True, text=True)
            self.assertNotEqual(failed.returncode, 0)
            self.assertIn('Stop not acknowledged', failed.stderr)

    def test_unchanged_mode_starts_without_restart_or_reload(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root/'bin'
            binary.mkdir()
            log = root/'calls'
            systemctl = binary/'systemctl'
            systemctl.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$TEST_MODE_CALLS"\n')
            systemctl.chmod(0o755)
            env = dict(os.environ, PATH=str(binary)+os.pathsep+os.environ['PATH'], TEST_MODE_CALLS=str(log))
            command = edge_pose_command('testedge', 'false', root/'systemd')
            subprocess.run(['bash', '-c', command], env=env, check=True)
            self.assertEqual(log.read_text().splitlines(), ['daemon-reload', 'restart nsl-edge-agent@testedge.service'])
            mode = root/'systemd/nsl-edge-agent@testedge.service.d/pose.conf'
            inode = mode.stat().st_ino
            log.write_text('')
            subprocess.run(['bash', '-c', command], env=env, check=True)
            self.assertEqual(log.read_text().splitlines(), ['start nsl-edge-agent@testedge.service'])
            self.assertEqual(mode.stat().st_ino, inode)
            log.write_text('')
            subprocess.run(['bash', '-c', edge_pose_command('testedge', 'true', root/'systemd')], env=env, check=True)
            self.assertEqual(log.read_text().splitlines(), ['daemon-reload', 'restart nsl-edge-agent@testedge.service'])
            self.assertIn('NSL_ENABLE_POSE=true', mode.read_text())

    def test_camera_only_agent_skips_weight_and_inference_startup(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            binary = home/'bin'
            binary.mkdir()
            calls = home/'calls'
            (binary/'ip').write_text('#!/bin/sh\necho "2: eth0 inet 192.168.3.161/24"\necho "3: eth1 inet 192.168.2.102/24"\n')
            (binary/'ping').write_text('#!/bin/sh\nexit 0\n')
            (binary/'ros2').write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$TEST_EDGE_CALLS"\n')
            for path in binary.iterdir():
                path.chmod(0o755)
            env = {key: value for key, value in os.environ.items()
                   if not key.startswith(('SIRIUS_', 'ROS_', 'NSL_', 'FASTRTPS_', 'FASTDDS_'))}
            env.update(PATH=str(binary)+os.pathsep+env['PATH'], HOME=str(home),
                       SIRIUS_ROBOT_DT_ROOT=str(ROOT), NSL_ENABLE_POSE='false', TEST_EDGE_CALLS=str(calls))
            result = subprocess.run(['bash', str(SETUP/'run_edge_agent.bash')], env=env,
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            invocations = calls.read_text().splitlines()
            self.assertEqual(len(invocations), 1)
            self.assertIn('launch roboscan_nsl3130 camera.launch.py', invocations[0])
            self.assertIn('use_multiview_tf:=true', invocations[0])
            self.assertIn('camera-only mode', result.stdout)

    def test_changed_camera_fingerprint_restarts_once_without_touching_raw_profiles(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root/'bin'
            binary.mkdir()
            log = root/'calls'
            systemctl = binary/'systemctl'
            systemctl.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$TEST_MODE_CALLS"\n')
            systemctl.chmod(0o755)
            env = dict(os.environ, PATH=str(binary)+os.pathsep+os.environ['PATH'], TEST_MODE_CALLS=str(log))
            first = edge_pose_command('testedge', 'false', root/'systemd', camera_sha='a'*64)
            subprocess.run(['bash', '-c', first], env=env, check=True)
            log.write_text('')
            subprocess.run(['bash', '-c', first], env=env, check=True)
            self.assertEqual(log.read_text().splitlines(), ['start nsl-edge-agent@testedge.service'])
            log.write_text('')
            changed = edge_pose_command('testedge', 'false', root/'systemd', camera_sha='b'*64)
            subprocess.run(['bash', '-c', changed], env=env, check=True)
            self.assertEqual(log.read_text().splitlines(), ['daemon-reload', 'restart nsl-edge-agent@testedge.service'])


if __name__ == '__main__':
    unittest.main()
