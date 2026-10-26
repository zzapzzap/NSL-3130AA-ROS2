"""Exercise operator prerequisites, saved bytes, and fresh model readiness without devices."""
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import textwrap
import time
import unittest
from unittest.mock import patch, Mock
import subprocess

import yaml

MODULE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('nsl_workflow', MODULE/'setup/workflow.py')
workflow = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(workflow)


class WorkflowTests(unittest.TestCase):
    def test_mvp_reuses_existing_models_after_mdt_without_fabricating_mtf_or_mtr(self):
        with tempfile.TemporaryDirectory() as directory:
            manual, _ = self.fixture_mtd(directory)
            config = Path(directory)/'inference.yaml'
            config.write_text('existing_models: true')
            state_file = Path(directory)/'workflow.yaml'
            # An existing camera viewer may stay open while map pose is selected.
            state_file.write_text(yaml.safe_dump({'mvw': {'pid': os.getpid(),
                'start_ticks': workflow.process_start(os.getpid()), 'token': 'camera-view'}}))
            prepared = {'marker_id': 0, 'weight_reference_verified': True,
                        'camera_octets': ['161'], 'config_path': str(config)}
            child = Mock(pid=os.getpid(), returncode=0)
            child.wait.return_value = 0
            with patch.object(workflow, 'STATE', state_file), \
                 patch.object(workflow, 'saved_mtd', return_value=manual), \
                 patch.object(workflow, 'calibration_digest', return_value='physical-hash'), \
                 patch.object(workflow, 'prepare_pose', return_value=prepared) as prepare, \
                 patch.object(workflow, 'share_prepared_pose'), \
                 patch.object(workflow, 'wait_pose_ready', return_value={'fusion': 'ready'}), \
                 patch.object(workflow.subprocess, 'run') as backend, \
                 patch.object(workflow.subprocess, 'Popen', return_value=child) as viewer:
                self.assertEqual(workflow.launch_viewer('map_pose'), 0)
            prepare.assert_called_once_with(None, ['161'])
            self.assertIn('camera_pose_robot', backend.call_args[0][0])
            self.assertIn('multiview_camera_pose_robot.launch.py', str(viewer.call_args[0][0]))
            state = workflow.read_yaml(state_file)
            self.assertNotIn('mtf', state)
            self.assertNotIn('mtr', state)
            self.assertFalse(state['mdt_pose']['fresh_mtf_claimed'])
            self.assertEqual(state['mdt_pose']['source'], 'existing_models_and_saved_mdt')

    def test_inference_preparation_keeps_the_selected_trained_model_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'config').mkdir()
            native = root/'controller/src/ros_humanpose/config/previous.yaml'
            native.parent.mkdir(parents=True)
            model_dir = '${SIRIUS_ROBOT_DT_ROOT}/controller/src/ros_humanpose/config/runtime/training/old/models'
            native.write_text(yaml.safe_dump({'pose_workflow': {'model_dir': model_dir}}))
            (root/'config/config.yaml').write_text(yaml.safe_dump({'pose': {
                'config': '../controller/src/ros_humanpose/config/previous.yaml'}}))
            result = subprocess.CompletedProcess([], 0, stdout='{"success": true}', stderr='')
            with patch.object(workflow, 'ROOT', root), \
                 patch.object(workflow, 'camera_settings', return_value=(root/'config/config.yaml', {})), \
                 patch.object(workflow.subprocess, 'run', return_value=result) as run:
                self.assertTrue(workflow.prepare_pose(None, ['161'])['success'])
            command = run.call_args[0][0]
            self.assertEqual(command[command.index('--model-dir')+1], model_dir)
            self.assertNotIn('--for-training', command)

    def test_readiness_receives_status_without_an_initialized_default_context(self):
        if importlib.util.find_spec('rclpy') is None:
            self.skipTest('ROS Python environment is unavailable')
        program = textwrap.dedent('''
            import importlib.util
            import json
            import sys
            import time
            from unittest.mock import patch

            import rclpy
            from rclpy.context import Context
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
            from std_msgs.msg import String

            spec = importlib.util.spec_from_file_location('readiness_test', sys.argv[1])
            workflow = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(workflow)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            prepared = {'marker_id': 3, 'reference_frame': 'id_3',
                        'config_sha256': 'prepared-hash', 'config_path': sys.argv[2],
                        'camera_octets': ['161'],
                        'ready_topics': {'cam_161': '/test/single_status', 'fusion': '/test/fusion_status'}}
            selected_at = time.time_ns()
            common = {'state': 'ready', 'marker_id': 3, 'reference_frame': 'id_3',
                      'initialized_at_ns': time.time_ns(), 'config_sha256': 'prepared-hash'}
            publishers, nodes = [], []
            class PublishingContext(Context):
                def init(self, *args, **kwargs):
                    super().init(*args, **kwargs)
                    single = Node('singleview_pose', namespace='/cam_161', context=self)
                    fusion = Node('multiview_pose_fusion', context=self)
                    nodes.extend([single, fusion])
                    for node, key, extra in [(single, 'cam_161', {'kind': 'single', 'models_loaded': True,
                            'serial': 'TEST_SERIAL', 'octet': '161'}), (fusion, 'fusion', {'kind': 'fusion'})]:
                        publisher = node.create_publisher(String, prepared['ready_topics'][key], qos)
                        publishers.append(publisher)
                        publisher.publish(String(data=json.dumps(dict(common, **extra))))
            try:
                assert not rclpy.get_default_context().ok()
                with patch.object(workflow, 'service_active', return_value=True), \\
                     patch('rclpy.context.Context', PublishingContext):
                    statuses = workflow.wait_pose_ready(prepared, selected_at, timeout=10)
                assert set(statuses) == {'cam_161', 'fusion'}, statuses
                assert statuses['cam_161']['serial'] == 'TEST_SERIAL'
                assert not rclpy.get_default_context().ok()
                print('Custom-context readiness received both fresh statuses')
            finally:
                for node in nodes:
                    node.destroy_node()
        ''')
        environment = dict(os.environ, ROS_DOMAIN_ID='223', ROS_LOCALHOST_ONLY='1')
        for name in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                     'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI'):
            environment.pop(name, None)
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory)/'inference.yaml'
            config.write_text(yaml.safe_dump({'pose_workflow': {'cameras': {'cam_161': {'serial': 'TEST_SERIAL'}}}}))
            result = subprocess.run([sys.executable, '-c', program,
                                     str(MODULE/'setup/workflow.py'), str(config)],
                                    env=environment, capture_output=True, text=True, timeout=25)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            self.assertIn('received both fresh statuses', result.stdout)

    def test_mtr_trains_and_opens_measurement_view_without_reading_mdt(self):
        with tempfile.TemporaryDirectory() as directory:
            state_file = Path(directory)/'workflow.yaml'
            state_file.write_text(yaml.safe_dump({'mtf': {'cameras': ['161']}}))
            config = Path(directory)/'inference.yaml'
            config.write_text('projection: measured')
            prepared = {'config_path': str(config), 'camera_octets': ['161']}
            inventory = {'cameras': {'cam_161': {'serial': 'ACTUAL_SERIAL', 'serial_verified': True}}}
            with patch.object(workflow, 'STATE', state_file), \
                 patch.object(workflow, 'saved_mtd', side_effect=AssertionError('mtr must not read mdt')), \
                 patch.object(workflow, 'camera_settings', return_value=(Path('/tmp/fleet.yaml'), inventory)), \
                 patch.object(workflow, 'measured_receipt', return_value=True), \
                 patch.object(workflow, 'calibration_digest', return_value='physical-calibration'), \
                 patch.object(workflow, 'prepare_pose', return_value=prepared) as prepare, \
                 patch.object(workflow, 'share_prepared_pose'), \
                 patch.object(workflow, 'wait_pose_ready', return_value={}), \
                 patch.object(workflow, 'launch_viewer', return_value=0) as view, \
                 patch.object(workflow.subprocess, 'run') as backend:
                self.assertEqual(workflow.run_pose(), 0)
                prepare.assert_called_once_with(None, ['161'], train=True)
                view.assert_called_once_with('measurement')
                command = backend.call_args[0][0]
                self.assertEqual(command[command.index('--select')+1], 'camera_pose')
                self.assertIsNone(workflow.read_yaml(state_file)['mtr']['display_session_sha256'])

    def test_new_training_can_reopen_mvw_without_mtf_and_keeps_its_models_selected(self):
        with tempfile.TemporaryDirectory() as directory:
            weight = Path(directory)/'new-model.pkl'
            weight.write_bytes(b'completed training')
            manual = Path(directory)/'manual.yaml'
            manual.write_text(yaml.safe_dump({'anchor_states': {'id_0': {'id': 0}}}))
            config = Path(directory)/'inference.yaml'
            config.write_text('prepared: true')
            state_file = Path(directory)/'workflow.yaml'
            record = {'ready': True, 'result': 'trained_saved_alignment', 'calibration_sha256': 'hash',
                      'display_session_sha256': workflow.digest(manual),
                      'preparation': {'marker_id': 0, 'config_path': str(config)},
                      'files': {str(weight): workflow.digest(weight)}}
            state_file.write_text(yaml.safe_dump({'mtr': record}))
            child = Mock(pid=os.getpid(), returncode=0)
            child.wait.return_value = 0
            with patch.object(workflow, 'STATE', state_file), \
                 patch.object(workflow, 'calibration_digest', return_value='hash'), \
                 patch.object(workflow, 'saved_mtd', return_value=manual), \
                 patch.object(workflow, 'prepare_mdt_mode', side_effect=AssertionError('no legacy prerequisites')), \
                 patch.object(workflow.subprocess, 'Popen', return_value=child) as start:
                workflow.require('mvp')
                self.assertEqual(workflow.launch_viewer('camera'), 0)
                self.assertIn('multiview_camera_pose_robot.launch.py', str(start.call_args[0][0]))
                self.assertEqual(workflow.read_yaml(state_file)['mtr'], record)
                manual.write_text(yaml.safe_dump({'anchor_states': {'id_0': {'id': 0}}, 'new_map_position': 1.25}))
                with patch.object(workflow.subprocess, 'run') as reload_display, \
                     patch.object(workflow, 'wait_pose_ready', return_value={'fusion': 'ready'}):
                    workflow.require('mvp')
                    self.assertEqual(workflow.launch_viewer('camera'), 0)
                    self.assertIn('--restart', reload_display.call_args[0][0])
                    updated = workflow.read_yaml(state_file)['mtr']
                    self.assertEqual(updated['files'], record['files'])
                    self.assertEqual(updated['display_session_sha256'], workflow.digest(manual))
                weight.write_bytes(b'changed model')
                with self.assertRaisesRegex(ValueError, 'weights changed'):
                    workflow.require('mvp')

    def test_map_placement_does_not_change_the_measured_training_fingerprint(self):
        inventory = {'cameras': {'cam_161': {'serial': 'TEST', 'ip': '192.168.3.161',
                        'intrinsic': [100., 100.], 'tf': [1., 2., 3.]}}}
        with patch.object(workflow, 'camera_settings', return_value=(Path('/tmp/test_inventory.yaml'), inventory)):
            measured = workflow.calibration_digest(include_map=False)
            display = workflow.calibration_digest()
            inventory['cameras']['cam_161']['tf'] = [5., 6., 7.]
            self.assertEqual(workflow.calibration_digest(include_map=False), measured)
            self.assertNotEqual(workflow.calibration_digest(), display)
            inventory['cameras']['cam_161']['intrinsic'] = [200., 200.]
            self.assertNotEqual(workflow.calibration_digest(include_map=False), measured)

    def test_reopening_viewer_preserves_only_an_unchanged_calibration_receipt(self):
        for fingerprint, preserved in [('same-calibration', True), ('changed-calibration', False)]:
            with self.subTest(fingerprint=fingerprint), tempfile.TemporaryDirectory() as directory:
                state_file=Path(directory)/'workflow.yaml'
                state_file.write_text(yaml.safe_dump({
                    'mvw': {'token': 'old-viewer'},
                    'mtf': {'token': 'verified-calibration', 'mvw_token': 'old-viewer',
                            'calibration_sha256': 'same-calibration', 'sdk_writeback': {'cam_161': {'sha256': 'saved'}}},
                    'mtr': {'ready': True}}))
                child=Mock(pid=os.getpid(),returncode=0)
                child.wait.return_value=0
                with patch.object(workflow,'STATE',state_file), \
                     patch.object(workflow,'calibration_digest',return_value=fingerprint), \
                     patch.object(workflow.subprocess,'Popen',return_value=child):
                    self.assertEqual(workflow.launch_viewer('camera'),0)
                current=workflow.read_yaml(state_file)
                self.assertEqual('mtf' in current,preserved)
                self.assertNotIn('mtr',current)
                if preserved:
                    self.assertEqual(current['mtf']['token'],'verified-calibration')
                    self.assertEqual(current['mtf']['mvw_token'],current['mvw']['token'])
                    self.assertEqual(current['mtf']['sdk_writeback'],{'cam_161':{'sha256':'saved'}})

    def test_closed_or_reused_pid_is_not_an_open_camera_viewer(self):
        state = {'mvw': {'pid': os.getpid(), 'start_ticks': workflow.process_start(os.getpid()), 'token': 'viewer'}}
        with patch.object(workflow, 'service_active', return_value=True):
            workflow.require('mvw', state)
            state['mvw']['start_ticks'] = 'an-old-process'
            with self.assertRaisesRegex(ValueError, 'Open mvw'):
                workflow.require('mvw', state)

    def test_reopened_viewer_and_changed_calibration_require_new_anchor_detection(self):
        state = {'mvw': {'pid': os.getpid(), 'start_ticks': workflow.process_start(os.getpid()), 'token': 'new'},
                 'mtf': {'token': 'calibration', 'mvw_token': 'old', 'calibration_sha256': 'old-hash'}}
        with self.assertRaisesRegex(ValueError, 'current camera calibration'):
            workflow.require('mtf', state, calibration='new-hash')
        with patch.object(workflow, 'service_active', return_value=True):
            with self.assertRaisesRegex(ValueError, 'current mvw session'):
                workflow.require('mtf', state, calibration='old-hash')

    def test_anchor_observations_do_not_claim_success_when_sdk_writeback_is_stale(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            profile=root/'profile'
            profile.mkdir()
            (profile/'multiview.yml').write_text('SDK raw output')
            inventory={'cameras': {'cam_161': {'serial': 'physical', 'serial_verified': True, 'parameters': 'profile'}}}
            state={'mvw': {'pid': os.getpid(), 'start_ticks': workflow.process_start(os.getpid()), 'token': 'viewer'}}
            state_file=root/'workflow.yaml'
            state_file.write_text(yaml.safe_dump(state))
            with patch.object(workflow, 'STATE', state_file), \
                 patch.object(workflow, 'service_active', return_value=True), \
                 patch.object(workflow, 'camera_settings', return_value=(root/'fleet.yaml',inventory)), \
                 patch.object(workflow, 'calibration_digest', return_value='hash'):
                with self.assertRaisesRegex(ValueError, 'writeback was not confirmed'):
                    workflow.mark_mtf(['161'], time.time()+100)
                self.assertNotIn('mtf',workflow.read_yaml(state_file))
                workflow.mark_mtf(['161'], 0)
                self.assertEqual(workflow.read_yaml(state_file)['mtf']['sdk_writeback']['cam_161']['serial'],'physical')

    def test_mvp_rejects_a_changed_weight_even_when_the_viewer_is_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            weight = Path(directory)/'weight.pkl'
            weight.write_bytes(b'original')
            state = {'mtf': {'token': 'calibration', 'calibration_sha256': 'hash'},
                     'mtr': {'ready': True, 'mtf_token': 'calibration', 'files': {str(weight): workflow.digest(weight)}}}
            workflow.require('mvp', state, calibration='hash')
            weight.write_bytes(b'replaced')
            with self.assertRaisesRegex(ValueError, 'weights changed'):
                workflow.require('mvp', state, calibration='hash')

    def fixture_mtd(self, directory, source='legacy_manual_two_robot_session'):
        root = Path(directory)
        capture = root/'captures/cam_161.npz'
        capture.parent.mkdir()
        capture.write_bytes(b'frozen-original-point-cloud')
        sha256 = workflow.digest(capture)
        inventory = {'cameras': {'cam_161': {'file': 'captures/cam_161.npz', 'sha256': sha256}}}
        (root/'snapshot.yaml').write_text(yaml.safe_dump(inventory))
        session = {'mtd_snapshot': {'source': source, 'saved': True, 'frozen': True,
                   'saved_at': None, 'timestamp_source': 'original_save_time_unavailable',
                   'snapshot_file': 'snapshot.yaml', 'camera_ids': [161]},
                   'camera_states': {'cam_161': {'capture_sha256': sha256}},
                   'reference_anchor_id': 0,
                   'robot_states': {role: {'controller_deg': [0., 1., 2., 3., 4., 5.]}
                                    for role in ('loading', 'unloading')}}
        path = root/'session.yaml'
        path.write_text(yaml.safe_dump(session))
        return path, capture

    def test_original_saved_mtd_is_accepted_without_fabricating_a_timestamp(self):
        with tempfile.TemporaryDirectory() as directory:
            path, capture = self.fixture_mtd(directory)
            with patch.object(workflow, 'session_file', return_value=path):
                self.assertEqual(workflow.saved_mtd(), path)
                capture.write_bytes(b'different-point-cloud')
                with self.assertRaisesRegex(ValueError, 'capture hash'):
                    workflow.saved_mtd()

    def test_live_capture_without_explicit_save_is_not_a_saved_mtd(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.fixture_mtd(directory, source='live_point_cloud_one_frame')
            with patch.object(workflow, 'session_file', return_value=path):
                self.assertIsNone(workflow.saved_mtd())

    def test_missing_saved_snapshot_reports_the_missing_path_and_recovery(self):
        for missing in ('snapshot.yaml', 'captures/cam_161.npz'):
            with self.subTest(missing=missing), tempfile.TemporaryDirectory() as directory:
                path, _ = self.fixture_mtd(directory)
                absent = Path(directory)/missing
                absent.unlink()
                with patch.object(workflow, 'session_file', return_value=path):
                    with self.assertRaisesRegex(ValueError, 'is missing') as error:
                        workflow.saved_mtd()
                self.assertIn(str(absent), str(error.exception))
                self.assertIn('run mtd', str(error.exception))

    def test_closing_new_mtd_without_saving_does_not_replace_the_prior_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            path,_=self.fixture_mtd(directory)
            session=workflow.read_yaml(path)
            session['mtd_snapshot']['id']='previous-archive'
            path.write_text(yaml.safe_dump(session))
            state_file=Path(directory)/'workflow.yaml'
            original={'mtd': {'session': str(path), 'saved_at': 'previous-save'}}
            state_file.write_text(yaml.safe_dump(original))
            with patch.object(workflow,'session_file',return_value=path), \
                 patch.object(workflow,'STATE',state_file), \
                 patch.object(workflow,'service_active',return_value=True), \
                 patch.object(workflow.subprocess,'run',return_value=subprocess.CompletedProcess([],0)):
                with self.assertRaisesRegex(ValueError,'No new mtd snapshot was saved'):
                    workflow.run_mtd()
                self.assertEqual(workflow.read_yaml(state_file),original)
                self.assertEqual(workflow.saved_mtd(),path)

    def test_new_saved_live_snapshot_records_its_own_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            path,_=self.fixture_mtd(directory)
            session=workflow.read_yaml(path)
            session['mtd_snapshot']['id']='previous-archive'
            path.write_text(yaml.safe_dump(session))
            state_file=Path(directory)/'workflow.yaml'
            def editor_save(command,**kwargs):
                updated=workflow.read_yaml(path)
                updated['mtd_snapshot'].update(id='new-live-capture',source='live_point_cloud_one_frame',saved_at='2026-10-05T12:00:00+00:00')
                path.write_text(yaml.safe_dump(updated))
                return subprocess.CompletedProcess(command,0)
            with patch.object(workflow,'session_file',return_value=path), \
                 patch.object(workflow,'STATE',state_file), \
                 patch.object(workflow,'service_active',return_value=True), \
                 patch.object(workflow.subprocess,'run',side_effect=editor_save):
                self.assertEqual(workflow.run_mtd(),0)
                self.assertEqual(workflow.read_yaml(state_file)['mtd']['sha256'],workflow.digest(path))

    def test_readiness_rejects_old_status_and_wrong_physical_binding(self):
        prepared = {'marker_id': 0, 'reference_frame': 'stag_marker', 'config_sha256': 'prepared-hash'}
        config = {'pose_workflow': {'cameras': {'cam_161': {'serial': 'physical-serial'}}}}
        payload = {'state': 'ready', 'kind': 'single', 'models_loaded': True, 'serial': 'physical-serial',
                   'marker_id': 0, 'reference_frame': 'stag_marker', 'initialized_at_ns': 101, 'octet': '161',
                   'config_sha256': 'prepared-hash'}
        self.assertTrue(workflow.ready_status(prepared, 'cam_161', payload, 100, config))
        self.assertFalse(workflow.ready_status(prepared, 'cam_161', payload, 102, config))
        self.assertFalse(workflow.ready_status(prepared, 'cam_161', dict(payload, serial='other-device'), 100, config))
        self.assertFalse(workflow.ready_status(prepared, 'cam_161', dict(payload, marker_id=2), 100, config))
        self.assertFalse(workflow.ready_status(prepared, 'cam_161', dict(payload, config_sha256='old-config'), 100, config))

    def test_fusion_initialized_status_does_not_require_a_person_or_claim_model_accuracy(self):
        prepared = {'marker_id': 0, 'reference_frame': 'stag_marker', 'config_sha256': 'prepared-hash'}
        payload = {'state': 'ready', 'kind': 'fusion', 'models_loaded': False, 'marker_id': 0,
                   'reference_frame': 'stag_marker', 'initialized_at_ns': 101, 'config_sha256': 'prepared-hash'}
        self.assertTrue(workflow.ready_status(prepared, 'fusion', payload, 100, {}))

    def test_mdt_preserves_saved_camera_and_robot_view_when_pose_anchor_is_unsafe(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.fixture_mtd(directory)
            with patch.object(workflow, 'saved_mtd', return_value=path), \
                 patch.object(workflow, 'read_yaml', side_effect=lambda file: yaml.safe_load(path.read_text()) if file == path else {}), \
                 patch.object(workflow, 'calibration_digest', return_value='hash'), \
                 patch.object(workflow, 'prepare_pose', return_value={'marker_id': 2}):
                self.assertEqual(workflow.prepare_mdt_mode(), 'camera_robot')

    def test_matching_marker_alone_does_not_attach_unknown_model_coordinates_globally(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.fixture_mtd(directory)
            state = {'mtr': {'preparation': {'marker_id': 0, 'weight_reference_verified': False}}}
            with patch.object(workflow, 'saved_mtd', return_value=path), \
                 patch.object(workflow, 'read_yaml', side_effect=lambda file: yaml.safe_load(path.read_text()) if file == path else state), \
                 patch.object(workflow, 'require', return_value=state), \
                 patch.object(workflow, 'prepare_pose', return_value={'marker_id': 0, 'weight_reference_verified': False}), \
                 patch.object(workflow.subprocess, 'run', side_effect=AssertionError('Unverified global pose must not start')):
                self.assertEqual(workflow.prepare_mdt_mode(), 'camera_robot')

    def test_verified_model_coordinates_can_reuse_matching_saved_mtd(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.fixture_mtd(directory)
            state = {'mtr': {'preparation': {'marker_id': 0, 'weight_reference_verified': True}}}
            with patch.object(workflow, 'saved_mtd', return_value=path), \
                 patch.object(workflow, 'read_yaml', side_effect=lambda file: yaml.safe_load(path.read_text()) if file == path else state), \
                 patch.object(workflow, 'require', return_value=state), \
                 patch.object(workflow, 'prepare_pose', side_effect=AssertionError('Reuse actual verified preparation')):
                self.assertEqual(workflow.prepare_mdt_mode(), 'camera_pose_robot')


if __name__ == '__main__':
    unittest.main()
