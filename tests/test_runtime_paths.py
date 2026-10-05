"""Verify standalone workspace sourcing and scoped fleet deployment without ROS/SSH."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[4]
EXE = Path(__file__).resolve().parents[1]/'setup'


class RuntimePathTests(unittest.TestCase):
    def test_env_works_from_other_directory_without_starting_processes(self):
        with tempfile.TemporaryDirectory() as directory:
            temporary = Path(directory)
            calls = temporary / 'calls'
            for name in ('ros2', 'ssh', 'scp', 'sshpass', 'rsync', 'sudo', 'systemctl', 'ip'):
                stub = temporary / name
                stub.write_text('#!/bin/sh\necho "$0" >> "$SIRIUS_TEST_CALLS"\nexit 99\n')
                stub.chmod(0o755)
            env = {k: v for k, v in os.environ.items()
                   if not k.startswith(('SIRIUS_', 'ROS_HUMANPOSE_', 'NSL_', 'COLCON_WS'))}
            env.update(PATH=directory + os.pathsep + env['PATH'], SIRIUS_TEST_CALLS=str(calls),
                       ROS_DOMAIN_ID='85', ROS_LOCALHOST_ONLY='1')
            script = '''set -eu
source "$1"
[[ "$SIRIUS_ROBOT_DT_ROOT" == "$2" ]]
[[ "$COLCON_WS" == "$2/controller" ]]
[[ "$ROS2_VERSION" == humble ]]
[[ "$ROOT_DIRECTORY" == "$2" && "$CONTROLLER_DIRECTORY" == "$2/controller" ]]
[[ "$CONFIG_DIRECTORY" == "$2" && "$DATA_DIRECTORY" == "$2" ]]
[[ ! -v SIRIUS_CONFIG ]]
[[ "$NSL_CALIB_DIR" == "$2/controller/src/NSL-3130AA-ROS2/calib_output" ]]
[[ "$ROS_HUMANPOSE_WEIGHT_ROOT" == "$2/controller/src/ros_humanpose/weight" ]]
[[ "$ROS_HUMANPOSE_DATASET_DIR" == "$2/controller/src/ros_humanpose/dataset" ]]
[[ "$ROS_DOMAIN_ID" == 42 && "$ROS_LOCALHOST_ONLY" == 1 ]]
[[ $- == *u* ]]
for name in mvw mvp mdt mtf mcalpull mtr mrb mba mloc sensors srbuild; do declare -F "$name" >/dev/null; done
'''
            result = subprocess.run(['bash', '-c', script, 'test-env', str(ROOT/'setup/env.bash'), str(ROOT)],
                                    env=env, cwd=directory, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(calls.exists())
            self.assertTrue(all(Path(name).name=='ip' for name in calls.read_text().splitlines()),calls.read_text())

    def test_env_defaults_to_domain_42_when_no_domain_is_set(self):
        env = {key: value for key, value in os.environ.items()
               if not key.startswith(('SIRIUS_', 'ROS_', 'NSL_', 'COLCON_WS'))}
        result = subprocess.run(
            ['bash', '-c', 'source "$1" && [[ "$ROS_DOMAIN_ID" == 42 ]]',
             'test-domain-default', str(ROOT/'setup/env.bash')],
            env=env, capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_mgp_scopes_destination_to_project_and_preserves_state(self):
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            calls = temp/'rsync-args'
            stub = temp/'rsync'
            stub.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@" > "$SIRIUS_TEST_CALLS"\n')
            stub.chmod(0o755)
            env = os.environ.copy()
            env.update(PATH=directory+os.pathsep+env['PATH'], SIRIUS_TEST_CALLS=str(calls),
                       SIRIUS_ROBOT_DT_ROOT=str(ROOT), NSL_FLEET_USER='test')
            script = '''source "$1"
_nsl_edges() { echo 161; }
_nsl_edge_password() { return 0; }
_nsl_ssh() { return 0; }
mgp
'''
            result = subprocess.run(['bash','-c',script,'test-mgp',str(EXE/'fleet_tools.sh')], env=env,
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            args = calls.read_text().splitlines()
            self.assertIn('--exclude=state/',args)
            self.assertIn('--exclude=config/calibration/',args)
            self.assertEqual(args[-2],str(ROOT)+'/')
            self.assertEqual(args[-1],'test@192.168.3.161:~/SiriusRobotDT/')

    def test_mba_forwards_paths_as_independent_arguments(self):
        with tempfile.TemporaryDirectory() as directory:
            temp=Path(directory)
            calls=temp/'python-args'
            stub=temp/'python3'
            stub.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@" > "$SIRIUS_TEST_CALLS"\n')
            stub.chmod(0o755)
            env=os.environ.copy()
            env.update(PATH=directory+os.pathsep+env['PATH'], SIRIUS_TEST_CALLS=str(calls),
                       SIRIUS_ROBOT_DT_ROOT=str(ROOT))
            script = 'source '+shlex.quote(str(EXE/'fleet_tools.sh'))+'\nmba '+shlex.quote('/tmp/a bag')+' --help\n'
            result=subprocess.run(['bash','-c',script],env=env,capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(calls.read_text().splitlines(),[str(ROOT/'controller/src/robot_dt_bridge/localization/tools/bag_autopsy.py'),'/tmp/a bag','--help'])

    def test_mdt_bag_uses_domain42_and_isolates_offline_transport_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            calls = temp/'ros-args'
            stub = temp/'ros2'
            stub.write_text('#!/bin/bash\nprintf "%s\\n" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "${ROS_DISCOVERY_SERVER-unset}" "${FASTRTPS_DEFAULT_PROFILES_FILE-unset}" "${RMW_FASTRTPS_USE_QOS_FROM_XML-unset}" "${CYCLONEDDS_URI-unset}" "$@" > "$SIRIUS_TEST_CALLS"\n')
            stub.chmod(0o755)
            env = dict(os.environ, PATH=directory+os.pathsep+os.environ['PATH'],
                SIRIUS_ROBOT_DT_ROOT=str(ROOT), SIRIUS_TEST_CALLS=str(calls), ROS_DOMAIN_ID='42',
                ROS_LOCALHOST_ONLY='0', ROS_DISCOVERY_SERVER='192.168.3.166:11811',
                FASTRTPS_DEFAULT_PROFILES_FILE='/tmp/existing-profile.xml',
                RMW_FASTRTPS_USE_QOS_FROM_XML='1', CYCLONEDDS_URI='existing.xml')
            script = 'source "$1"\nmdt bag "/tmp/a bag" with_rviz:=false\n[[ "$ROS_DISCOVERY_SERVER" == 192.168.3.166:11811 && "$ROS_LOCALHOST_ONLY" == 0 ]]\n'
            result = subprocess.run(['bash', '-c', script, 'test-mdt', str(EXE/'fleet_tools.sh')],
                env=env, capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(calls.read_text().splitlines(), ['42', '1', 'unset', 'unset', 'unset', 'unset',
                'launch', 'sirius_localization', 'mdt.launch.py', 'source:=bag', 'use_mvp:=false',
                'session_file:='+str(ROOT/'config/config.yaml'),
                'bag:=/tmp/a bag', 'with_rviz:=false'])

    def test_frontends_can_be_resourced_and_reference_existing_root_launches(self):
        script='''source "$1"
source "$1"
for command_name in msb mcb mvw mtf mtr mvp; do alias "$command_name"; done
'''
        result=subprocess.run(['bash','-O','expand_aliases','-c',script,'test-frontends',str(ROOT/'setup/env.bash')],
                              capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)
        for name in ('msb','mcb','mvw','mtf','mtr','mvp','mtd','mdt'):
            self.assertTrue((ROOT/'exec/launch'/f'{name}.launch.py').is_file())
        self.assertIn(str(ROOT/'exec/launch/mvw.launch.py'),result.stdout)

    def test_mloc_offline_uses_domain42_and_keeps_caller_transport(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            location=root/'controller/src/robot_dt_bridge/localization'
            location.mkdir(parents=True)
            original=ROOT/'controller/src/robot_dt_bridge/localization/start_two_robots.sh'
            (location/'start_two_robots.sh').write_bytes(original.read_bytes())
            install=root/'controller/install'
            install.mkdir()
            (install/'setup.bash').write_text('')
            (install/'local_setup.bash').write_text('export PATH="$TEST_STUB_BIN:$PATH"\n')
            binary=root/'bin'
            binary.mkdir()
            calls=root/'calls'
            (binary/'systemctl').write_text('#!/bin/sh\nexit 3\n')
            (binary/'ros2').write_text('#!/bin/bash\nprintf "%s\\n" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "${ROS_DISCOVERY_SERVER-unset}" "${ROS_SUPER_CLIENT-unset}" "${CYCLONEDDS_URI-unset}" "$@" > "$SIRIUS_TEST_CALLS"\n')
            for file in binary.iterdir(): file.chmod(0o755)
            env=dict(os.environ,SIRIUS_ROBOT_DT_ROOT=str(root),TEST_STUB_BIN=str(binary),
                     SIRIUS_TEST_CALLS=str(calls),ROS_DOMAIN_ID='85',ROS_LOCALHOST_ONLY='0',
                     ROS_DISCOVERY_SERVER='192.168.3.166:11811',ROS_SUPER_CLIENT='TRUE',CYCLONEDDS_URI='old.xml')
            script='source "$1"\nmloc\n[[ "$ROS_DOMAIN_ID" == 85 && "$ROS_DISCOVERY_SERVER" == 192.168.3.166:11811 ]]\n'
            result=subprocess.run(['bash','-c',script,'test-mloc',str(EXE/'fleet_tools.sh')],env=env,
                                  capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(calls.read_text().splitlines()[:5],['42','1','unset','unset','unset'])


if __name__ == '__main__':
    unittest.main()
