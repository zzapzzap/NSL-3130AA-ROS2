"""Exercise operator prerequisites, saved bytes, and fresh model readiness without devices."""
import importlib.util
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
import subprocess

import yaml

MODULE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('nsl_workflow', MODULE/'setup/workflow.py')
workflow = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(workflow)


class WorkflowTests(unittest.TestCase):
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
                 patch.object(workflow,'require',return_value={}), \
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
                 patch.object(workflow,'require',return_value={}), \
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
