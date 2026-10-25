"""Fleet DDS, SSH lifecycle, deployment preservation, and Edge inference modes."""
import ast
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[4]
MODULE = Path(__file__).resolve().parents[1]
SETUP = MODULE/'setup'
EXEC = SETUP
LAUNCH = MODULE/'roboscan_nsl3130/launch'
SCRIPT = SETUP/'setup_dds_interface.bash'
TOOLS = SETUP/'fleet_tools.sh'
NS = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
KEYS = ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTRTPS_DEFAULT_PROFILES_FILE',
        'FASTDDS_DEFAULT_PROFILES_FILE', 'ROS_LOCALHOST_ONLY')
sys.path.insert(0, str(SETUP))
from edge_pose_mode import edge_pose_command, edge_stop_command


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
            test "$(_nsl_edges | tr '\\n' ' ')" = '161 162 163 164 165 '
            test "$(sort -u "$TEST_PING_LOG" | tr '\\n' ' ')" = '192.168.3.161 192.168.3.162 192.168.3.163 192.168.3.164 192.168.3.165 '
        ''', config='Edge 192.168.3.52\n    id legacy_one\nEdge 53\n    id legacy_two\n')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_mixed_credentials_do_not_limit_discovery_without_network_calls(self):
        result = self.run_shell('''
            source "$TEST_EXEC/fleet_tools.sh"
            ping() { printf "%s\\n" "${@: -1}" >> "$TEST_PING_LOG"; }
            test "$(_nsl_config_edges | tr '\\n' ' ')" = '161 163 '
            test "$(_nsl_edges | tr '\\n' ' ')" = '161 162 163 164 165 '
            test "$(sort -u "$TEST_PING_LOG" | tr '\\n' ' ')" = '192.168.3.161 192.168.3.162 192.168.3.163 192.168.3.164 192.168.3.165 '
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
        for octet in (161,162,163,164,165,166):
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
                self.assertIsNone(xml.find('.//d:defaultUnicastLocatorList', NS))
                descriptors = xml.findall('.//d:transport_descriptor', NS)
                shm = next(d for d in descriptors if d.find('d:type', NS).text == 'SHM')
                self.assertEqual(shm.find('d:segment_size', NS).text, '134217728')
                self.assertGreaterEqual(int(shm.find('d:maxMessageSize', NS).text), 1920*1080*3)
                self.assertIn('sirius_local_shm', [d.text for d in xml.findall('.//d:userTransports/d:transport_id', NS)])

    def test_interactive_edge_terminal_gets_full_graph_for_cli_daemon(self):
        data,_=self.run_setup(['192.168.3.161'],interactive=True)
        self.assertEqual(data['env']['ROS_SUPER_CLIENT'],'TRUE')
        self.assertEqual(data['env']['ROS_DISCOVERY_SERVER'],'192.168.3.166:11811')

    def test_plcs_unused_hosts_and_other_subnets_are_excluded(self):
        for a in ('192.168.3.155','192.168.3.160','192.168.3.167',
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


class FleetWorkflowTests(unittest.TestCase):
    def shell(self, directory, body, extra=None):
        temporary=Path(directory)
        env=dict(os.environ, SIRIUS_ROBOT_DT_ROOT=str(ROOT), NSL_WORKFLOW_STATE=str(temporary/'workflow.yaml'),
                 TEST_COMMANDS=str(temporary/'calls'), NSL_FLEET_CONFIG=str(temporary/'empty-credentials'),
                 NSL_FLEET_CONFIG_LEGACY=str(temporary/'empty-legacy'), NSL_FLEET_USER='testedge')
        env.update(extra or {})
        result=subprocess.run(['bash','-c','source "$1"\n'+body,'test-fleet',str(TOOLS)],
                              env=env,capture_output=True,text=True,timeout=10)
        calls=(temporary/'calls').read_text().splitlines() if (temporary/'calls').is_file() else []
        return result,calls

    def test_mtf_waits_for_writeback_then_imports_before_marking_success(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            ros2=root/'ros2'
            ros2.write_text('#!/bin/sh\nexit 0\n')
            ros2.chmod(0o755)
            result,calls=self.shell(directory, r'''_nsl_edges() { echo 161; }
_nsl_login_user() { echo testedge; }
_nsl_workflow() { printf '%s\n' "$1" >> "$TEST_COMMANDS"; }
_nsl_sudo() {
 local count=0
 [[ ! -f "$TEST_COMMANDS.count" ]] || count="$(cat "$TEST_COMMANDS.count")"
 count=$((count+1)); echo "$count" > "$TEST_COMMANDS.count"
 echo '[observe] published 1 tag obs on /cam_161/tag_observations (ref id 3).'
 (( count < 2 )) || echo '[multiview_put] complete, verified, installed -> result'
}
mcalpull() { echo import >> "$TEST_COMMANDS"; }
python3() { return 0; }
sleep() { :; }
mtf
''',{'PATH':directory+os.pathsep+os.environ['PATH'], 'NSL_CALIB_WAIT_SEC':'0',
     'NSL_CALIB_WRITEBACK_WAIT_SEC':'3'})
            self.assertEqual(result.returncode,0,result.stderr+result.stdout)
            self.assertEqual((root/'calls.count').read_text().strip(),'2')
            self.assertEqual(calls,['require','import','mark-mtf'])

    def test_mtf_does_not_accept_observations_without_verified_writeback(self):
        with tempfile.TemporaryDirectory() as directory:
            ros2=Path(directory)/'ros2'
            ros2.write_text('#!/bin/sh\nexit 0\n')
            ros2.chmod(0o755)
            result,calls=self.shell(directory,r'''_nsl_edges() { echo 161; }
_nsl_login_user() { echo testedge; }
_nsl_workflow() { printf '%s\n' "$1" >> "$TEST_COMMANDS"; }
_nsl_sudo() { echo '[observe] published 1 tag obs on /cam_161/tag_observations (ref id 3).'; }
mcalpull() { echo unexpected-import >> "$TEST_COMMANDS"; }
python3() { return 0; }
sleep() { :; }
mtf
''',{'PATH':directory+os.pathsep+os.environ['PATH'], 'NSL_CALIB_WAIT_SEC':'0',
     'NSL_CALIB_WRITEBACK_WAIT_SEC':'0'})
            self.assertEqual(result.returncode,1,result.stderr+result.stdout)
            self.assertEqual(calls,['require'])
            self.assertIn('completed solver writeback was not confirmed',result.stdout)

    def test_msb_restarts_only_camera_agent_without_clock_or_training_listener(self):
        with tempfile.TemporaryDirectory() as directory:
            result,calls=self.shell(directory,'''_nsl_edges() { echo 161; }
_nsl_login_user() { echo testedge; }
_nsl_workflow() { return 0; }
_nsl_sudo() { printf '%s\n' "$2" >> "$TEST_COMMANDS"; }
msb
''')
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(len(calls),1)
            self.assertIn('nsl-edge-agent@testedge.service',calls[0])
            self.assertNotIn('train-listener',calls[0])
            self.assertNotIn('timedatectl',calls[0])

    def test_credentials_do_not_hide_key_authenticated_edges_from_lifecycle_and_build(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            credentials = root/'edges'
            credentials.write_text('Edge 192.168.3.161\n id configurededge\n')
            ping = root/'ping'
            ping.write_text('#!/bin/sh\nfor value in "$@"; do address="$value"; done\ncase "$address" in 192.168.3.161|192.168.3.163) exit 0 ;; *) exit 1 ;; esac\n')
            ip = root/'ip'
            ip.write_text('#!/bin/sh\necho "2: eth0 inet 192.168.3.166/24"\n')
            ping.chmod(0o755)
            ip.chmod(0o755)
            result, calls = self.shell(directory, '''
[[ "$(_nsl_edges | tr '\n' ' ')" == '161 163 ' ]] || exit 91
_nsl_workflow() { return 0; }
_nsl_ssh() {
    if [[ "$2" == 'id -un' ]]; then echo keyedge;
    else printf 'ssh edge%s %s\n' "$1" "$2" >> "$TEST_COMMANDS"; fi
}
_nsl_sudo() { printf 'sudo edge%s %s\n' "$1" "$2" >> "$TEST_COMMANDS"; }
_nsl_restart
mgp() { for octet in $(_nsl_edges); do printf 'share edge%s\n' "$octet" >> "$TEST_COMMANDS"; done; }
mcb
''', {'PATH': directory+os.pathsep+os.environ['PATH'], 'NSL_FLEET_CONFIG': str(credentials),
      'NSL_FLEET_USER': '', 'NSL_EDGES': ''})
            self.assertEqual(result.returncode, 0, result.stderr)
            rendered = '\n'.join(calls)
            self.assertIn('sudo edge161', rendered)
            self.assertIn('nsl-edge-agent@configurededge.service', rendered)
            self.assertIn('sudo edge163', rendered)
            self.assertIn('nsl-edge-agent@keyedge.service', rendered)
            self.assertIn('share edge163', rendered)
            self.assertIn('ssh edge163', rendered)
            self.assertIn('colcon build --symlink-install', rendered)
            self.assertNotIn('edge166', rendered)

    def test_mcb_shares_whole_project_before_build_and_does_not_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            result,calls=self.shell(directory,'''_nsl_edges() { echo 161; }
mgp() { echo share-project >> "$TEST_COMMANDS"; }
_nsl_workflow() { return 0; }
_nsl_ssh() { printf '%s\n' "$2" >> "$TEST_COMMANDS"; }
_nsl_restart() { echo unexpected-restart >> "$TEST_COMMANDS"; return 99; }
mcb
''')
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(calls[0],'share-project')
            self.assertIn('SiriusRobotDT/controller', '\n'.join(calls))
            self.assertIn('colcon build --symlink-install','\n'.join(calls))
            self.assertNotIn('unexpected-restart',calls)

    def test_mcb_does_not_build_when_project_sync_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            result,calls=self.shell(directory,'''_nsl_edges() { echo 161; }
mgp() { echo failed-sync >> "$TEST_COMMANDS"; return 7; }
_nsl_ssh() { echo unexpected-build >> "$TEST_COMMANDS"; }
mcb
''')
            self.assertEqual(result.returncode,7,result.stderr)
            self.assertEqual(calls,['failed-sync'])

    def test_rsync_failure_survives_output_prefix_pipeline(self):
        with tempfile.TemporaryDirectory() as directory:
            temporary=Path(directory)
            stub=temporary/'rsync'
            stub.write_text('#!/bin/sh\nexit 9\n')
            stub.chmod(0o755)
            result,_=self.shell(directory,'''_nsl_edges() { echo 161; }
_nsl_ssh() { return 0; }
mgp
''',{'PATH':directory+os.pathsep+os.environ['PATH']})
            self.assertEqual(result.returncode,1,result.stderr)
            self.assertIn('rsync FAILED',result.stdout)

    @unittest.skipUnless(shutil.which('rsync'),'local rsync is required')
    def test_mirroring_preserves_git_file_and_edge_calibration_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            source=root/'source'
            target=root/'target'
            source.mkdir(); target.mkdir()
            for folder in (source,target):
                (folder/'controller/src/module').mkdir(parents=True)
                (folder/'controller/src/module/calib_output').mkdir()
            (source/'controller/src/module/.git').write_text('host git pointer')
            (target/'controller/src/module/.git').write_text('edge git pointer')
            (source/'controller/src/module/calib_output/intrinsic.yml').write_text('host calibration')
            (target/'controller/src/module/calib_output/intrinsic.yml').write_text('actual edge calibration')
            (source/'controller/src/module/node.py').write_text('new code')
            body='rsync -a --delete "${NSL_MIRROR_EXCLUDES[@]}" "$2/" "$3/"\n'
            result=subprocess.run(['bash','-c','source "$1"\n'+body,'test-rsync',str(TOOLS),str(source),str(target)],
                                  capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual((target/'controller/src/module/.git').read_text(),'edge git pointer')
            self.assertEqual((target/'controller/src/module/calib_output/intrinsic.yml').read_text(),'actual edge calibration')
            self.assertEqual((target/'controller/src/module/node.py').read_text(),'new code')

    def test_prepared_config_is_copied_and_hash_verified(self):
        with tempfile.TemporaryDirectory() as directory:
            temporary=Path(directory)
            project=temporary/'remote'
            credentials=temporary/'edges'
            credentials.write_text('Edge 192.168.3.161\n id testedge\n project '+str(project)+'\n')
            scp=temporary/'scp'
            scp.write_text('#!/usr/bin/env bash\nsource_file="${@: -2:1}"\ndestination="${@: -1}"\ncp -- "$source_file" "${destination#*:}"\n')
            scp.chmod(0o755)
            config=ROOT/'controller/src/ros_humanpose/config/config.yml'
            env=dict(os.environ, PATH=directory+os.pathsep+os.environ['PATH'], SIRIUS_ROBOT_DT_ROOT=str(ROOT),
                     NSL_FLEET_CONFIG=str(credentials),NSL_FLEET_USER='testedge')
            script='source "$1"\n_nsl_ssh() { bash -c "$2"; }\n_nsl_share_pose_config "$2" 161\n'
            result=subprocess.run(['bash','-c',script,'test-share',str(TOOLS),str(config)],env=env,
                                  capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
            copied=project/'controller/src/ros_humanpose/config/config.yml'
            self.assertEqual(copied.read_bytes(),config.read_bytes())
            self.assertIn('SHA verified',result.stdout)

    def test_public_config_copy_is_atomic_and_digest_failure_preserves_edge_config(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            host = root/'host'
            remote = root/'remote'
            (host/'config').mkdir(parents=True)
            (remote/'config').mkdir(parents=True)
            config = host/'config/config.yaml'
            config.write_text('schema_version: 2\ncameras: {}\n')
            destination = remote/'config/config.yaml'
            destination.write_text('original edge settings')
            raw = remote/'controller/src/NSL-3130AA-ROS2/calib_output/ACTUAL/intrinsic.yml'
            raw.parent.mkdir(parents=True)
            raw.write_bytes(b'physical SDK calibration unchanged')
            credentials = root/'edges'
            credentials.write_text('Edge 192.168.3.161\n id testedge\n project '+str(remote)+'\n')
            scp = root/'scp'
            scp.write_text('#!/usr/bin/env bash\nsource_file="${@: -2:1}"\ndestination="${@: -1}"\ncp -- "$source_file" "${destination#*:}"\nif [[ "${TEST_CORRUPT:-}" == true ]]; then printf corrupt > "${destination#*:}"; fi\n')
            scp.chmod(0o755)
            environment = dict(os.environ, PATH=directory+os.pathsep+os.environ['PATH'], SIRIUS_ROBOT_DT_ROOT=str(host),
                               NSL_FLEET_CONFIG=str(credentials), NSL_FLEET_USER='testedge', TEST_CORRUPT='true')
            script = 'source "$1"\n_nsl_ssh() { bash -c "$2"; }\n_nsl_share_project_file "$2" 161\n'
            failed = subprocess.run(['bash', '-c', script, 'test-share', str(TOOLS), str(config)], env=environment,
                                    capture_output=True, text=True, timeout=10)
            self.assertNotEqual(failed.returncode, 0)
            self.assertEqual(destination.read_text(), 'original edge settings')
            environment['TEST_CORRUPT'] = 'false'
            copied = subprocess.run(['bash', '-c', script, 'test-share', str(TOOLS), str(config)], env=environment,
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(copied.returncode, 0, copied.stderr)
            self.assertEqual(destination.read_bytes(), config.read_bytes())
            self.assertEqual(raw.read_bytes(), b'physical SDK calibration unchanged')
