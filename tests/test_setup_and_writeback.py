"""Check opt-in shell installation and verified calibration registry updates."""
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import yaml

ROOT=Path(__file__).resolve().parents[4]
SETUP=ROOT/'setup'
CAMERA=ROOT/'controller/src/NSL-3130AA-ROS2/roboscan_nsl3130'
sys.path.insert(0,str(ROOT/'controller/src/robot_dt_bridge/config'))
sys.path.insert(0,str(CAMERA/'scripts'))


class ShellInstallTests(unittest.TestCase):
    def test_template_replaces_controller_aliases_and_sets_requested_directories(self):
        script = '\n'.join([
            'alias cb="echo legacy"',
            'export ROOT_DIRECTORY="$2"',
            'source "$1"',
            '[[ "$ROOT_DIRECTORY" == "$2" ]]',
            '[[ "$CONTROLLER_DIRECTORY" == "$2/controller" ]]',
            '[[ "$CONFIG_DIRECTORY" == "$2" && "$DATA_DIRECTORY" == "$2" ]]',
            '[[ "$ROS_DOMAIN_ID" == 42 ]]',
            '[[ "$(type -t cb)" == function ]]',
        ])
        result = subprocess.run(['bash','-c',script,'test-template',str(SETUP/'bashrc.template.bash'),str(ROOT)],
                                capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_preview_cannot_edit_requested_rc(self):
        with tempfile.TemporaryDirectory() as directory:
            rc=Path(directory)/'test.bashrc'
            rc.write_text('# user settings\nexport USER_TEST=kept\n')
            before=rc.read_bytes()
            result=subprocess.run(['bash',str(SETUP/'install_shell.bash'),'--rc',str(rc)],
                                  capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(rc.read_bytes(),before)
            self.assertIn('Preview only',result.stdout)

    def test_explicit_install_is_idempotent_and_keeps_personal_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            rc=Path(directory)/'test.bashrc'
            original='# user settings\nalias user_cmd="echo kept"\n'
            rc.write_text(original)
            command=['bash',str(SETUP/'install_shell.bash'),'--install-shell','--rc',str(rc)]
            first=subprocess.run(command,capture_output=True,text=True,timeout=5)
            self.assertEqual(first.returncode,0,first.stderr)
            content=rc.read_bytes()
            second=subprocess.run(command,capture_output=True,text=True,timeout=5)
            self.assertEqual(second.returncode,0,second.stderr)
            self.assertEqual(rc.read_bytes(),content)
            self.assertIn('alias user_cmd=',rc.read_text())
            self.assertEqual(rc.read_text().count('# >>> SiriusRobotDT managed shell >>>'),1)
            self.assertEqual(rc.with_name(rc.name+'.siriusrobotdt.bak').read_text(),original)

    def test_sensor_addresses_do_not_use_arithmetic_or_fill_unknown_devices(self):
        spec=importlib.util.spec_from_file_location('sirius_camera_network_test',CAMERA.parent/'setup/camera_network.py')
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            config=Path(directory)/'config.yaml'
            config.write_text(yaml.safe_dump({'cameras':{
                'cam_161':{'sensor_host':'192.168.2.202','sensor_source_address':'192.168.2.102'},
                'cam_163':{'sensor_host':None,'sensor_source_address':None}}}))
            self.assertEqual(module.camera_field('161','sensor_host',config),'192.168.2.202')
            self.assertEqual(module.camera_field('163','sensor_host',config),'')
            self.assertEqual(module.camera_field('165','sensor_host',config),'')


class CompletedWritebackTests(unittest.TestCase):
    def node(self,config,calibration):
        path=CAMERA/'scripts/multiview_put_server.py'
        tree=ast.parse(path.read_text())
        retained=[item for item in tree.body if isinstance(item,(ast.FunctionDef,ast.ClassDef))
                  and item.name in ('_sha256','sync_completed_writeback','MultiviewPutServer')]
        scope={'Node':object,'os':os,'hashlib':hashlib,'ALLOWED':'multiview.yml'}
        exec(compile(ast.Module(body=retained,type_ignores=[]),str(path),'exec'),scope)
        node=scope['MultiviewPutServer'].__new__(scope['MultiviewPutServer'])
        node.serial='TEST_SERIAL'
        node.namespace='cam_161'
        node.calib_dir=str(calibration)
        node.dev_dir=str(calibration/'TEST_SERIAL')
        Path(node.dev_dir).mkdir(parents=True)
        logs=[]
        node.get_logger=lambda:SimpleNamespace(info=logs.append,warning=logs.append)
        return node,logs

    def test_verified_install_updates_binding_and_profile_without_touching_robot_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            config=root/'config.yaml'
            config.write_text(yaml.safe_dump({'hi6':{'keep':'robot'},'cameras':{'cam_161':{'serial':None}}}))
            calibration=root/'calibration'
            node,logs=self.node(config,calibration)
            payload=b'%YAML:1.0\n---\nreference_frame: stag_marker\nnamespace: cam_161\n'
            request=SimpleNamespace(name='multiview.yml',offset=0,total_size=len(payload),
                                    sha256=hashlib.sha256(payload).hexdigest(),data=payload)
            with patch.dict(os.environ,{'SIRIUS_CONFIG':str(config)}):
                response=node._on_put(request,SimpleNamespace())
            self.assertTrue(response.success)
            self.assertTrue(response.complete)
            saved=yaml.safe_load(config.read_text())
            self.assertEqual(saved['hi6'],{'keep':'robot'})
            self.assertEqual(saved['cameras']['cam_161']['serial'],'TEST_SERIAL')
            self.assertEqual(saved['calibration_profiles']['TEST_SERIAL']['multiview']['sha256'],request.sha256)
            self.assertEqual((calibration/'TEST_SERIAL/multiview.yml').read_bytes(),payload)

    def test_digest_mismatch_cannot_update_registry(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            config=root/'config.yaml'
            config.write_text('cameras: {cam_161: {serial: null}}\n')
            before=config.read_bytes()
            calibration=root/'calibration'
            node,_=self.node(config,calibration)
            request=SimpleNamespace(name='multiview.yml',offset=0,total_size=3,sha256='invalid',data=b'bad')
            with patch.dict(os.environ,{'SIRIUS_CONFIG':str(config)}):
                response=node._on_put(request,SimpleNamespace())
            self.assertFalse(response.success)
            self.assertFalse(response.complete)
            self.assertEqual(config.read_bytes(),before)
            self.assertFalse((calibration/'TEST_SERIAL/multiview.yml').exists())

    def test_compact_writeback_updates_only_serial_and_profile_pointer(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root/'config.yaml'
            identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
            robots = {'loading': {'ip': None, 'port': 8888, 'tf': identity,
                                  'direction': [1, 1, -1, 1, -1, 1], 'pose': [1, 2, 3, 4, 5, 6]}}
            config.write_text(yaml.safe_dump({'schema_version': 2,
                'server': {'ip': '192.168.3.166', 'domain_id': 42}, 'robots': robots,
                'cameras': {'cam_161': {'ip': '192.168.3.161', 'serial': None,
                                      'parameters': 'calibration/cam_161', 'tf': identity}},
                'anchor': {'id': [0, 7], 'tf': identity}}))
            calibration = root/'calibration'
            node, _ = self.node(config, calibration)
            payload = b'%YAML:1.0\n---\nreference_frame: stag_marker\nnamespace: cam_161\n'
            request = SimpleNamespace(name='multiview.yml', offset=0, total_size=len(payload),
                                      sha256=hashlib.sha256(payload).hexdigest(), data=payload)
            with patch.dict(os.environ, {'SIRIUS_CONFIG': str(config)}), \
                 patch('detect_camera_id.detect_camera_serials', return_value=['TEST_SERIAL']):
                response = node._on_put(request, SimpleNamespace())
            self.assertTrue(response.success)
            self.assertTrue(response.complete)
            saved = yaml.safe_load(config.read_text())
            self.assertEqual(saved['robots'], robots)
            self.assertEqual(saved['cameras']['cam_161']['serial'], 'TEST_SERIAL')
            self.assertEqual(saved['cameras']['cam_161']['parameters'], 'calibration/TEST_SERIAL')
            self.assertEqual(saved['cameras']['cam_161']['tf'], identity)
            self.assertEqual(set(saved), {'schema_version', 'server', 'robots', 'cameras', 'anchor'})
            manifest = yaml.safe_load((calibration/'profiles.yaml').read_text())
            metadata = manifest['profiles']['TEST_SERIAL']['multiview']
            self.assertEqual(metadata['sha256'], request.sha256)
            self.assertNotIn('values', metadata)
            self.assertLess(len(config.read_text().splitlines()), 25)

    def test_native_writeback_updates_camera_entry_and_external_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            module = Path(directory)
            (module/'setup').mkdir()
            config = module/'setup/fleet.yaml'
            config.write_text(yaml.safe_dump({'schema_version': 1,
                'server': {'ip': '192.168.3.166', 'domain_id': 42},
                'cameras': {'cam_161': {'ip': '192.168.3.161', 'serial': None,
                                      'parameters': '../calib_output/runtime/cam_161',
                                      'sensor_host': '192.168.2.202'}}}))
            calibration = module/'calib_output'
            node, _ = self.node(config, calibration)
            payload = b'%YAML:1.0\n---\nnamespace: cam_161\n'
            request = SimpleNamespace(name='multiview.yml', offset=0, total_size=len(payload),
                                      sha256=hashlib.sha256(payload).hexdigest(), data=payload)
            with patch.dict(os.environ, {'SIRIUS_CONFIG': str(config)}), \
                 patch('detect_camera_id.detect_camera_serials', return_value=['TEST_SERIAL']):
                response = node._on_put(request, SimpleNamespace())
            self.assertTrue(response.success)
            saved = yaml.safe_load(config.read_text())
            self.assertEqual(saved['cameras']['cam_161']['serial'], 'TEST_SERIAL')
            self.assertTrue(saved['cameras']['cam_161']['serial_verified'])
            self.assertEqual(saved['cameras']['cam_161']['parameters'], '../calib_output/TEST_SERIAL')
            self.assertEqual(saved['cameras']['cam_161']['sensor_host'], '192.168.2.202')
            self.assertNotIn('calibration_profiles', saved)
            self.assertFalse((config.parent/'cameras').exists())
            manifest = yaml.safe_load((calibration/'runtime/profiles.yaml').read_text())
            self.assertEqual(manifest['profiles']['TEST_SERIAL']['multiview']['sha256'], request.sha256)


if __name__=='__main__':
    unittest.main()
