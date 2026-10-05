"""Stub fleet I/O to prove command scope and failure propagation."""
import os
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[4]
TOOLS = Path(__file__).resolve().parents[1]/'setup/fleet_tools.sh'


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


if __name__ == '__main__':
    unittest.main()
