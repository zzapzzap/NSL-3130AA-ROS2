#!/usr/bin/env python3
"""Guard the operator workflow using native calibration files and owned viewer PIDs."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid

import yaml

MODULE = Path(__file__).resolve().parents[1]
ROOT = Path(os.environ.get('SIRIUS_ROBOT_DT_ROOT', MODULE.parents[2]))
RUNTIME = Path(os.environ.get('NSL_CALIB_DIR', MODULE/'calib_output'))/'runtime'
STATE = Path(os.environ.get('NSL_WORKFLOW_STATE', RUNTIME/'workflow.yaml'))
SERVICE = 'sensor_streaming.service'


def now():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_yaml(path):
    try:
        value = yaml.safe_load(Path(path).read_text())
    except FileNotFoundError:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f'Expected YAML mapping: {path}')
    return value


@contextmanager
def locked_state(path=None):
    path = Path(STATE if path is None else path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = read_yaml(path)
        data.setdefault('schema_version', 1)
        yield data
        descriptor, staged = tempfile.mkstemp(prefix='.workflow.', dir=path.parent)
        try:
            with os.fdopen(descriptor, 'w') as file:
                yaml.safe_dump(data, file, sort_keys=False)
                file.flush()
                os.fsync(file.fileno())
            os.replace(staged, path)
        finally:
            Path(staged).unlink(missing_ok=True)


def process_start(pid):
    if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
        return None
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] in ('Z', 'X'):
            return None
        return fields[19]  # Linux field 22, following pid and parenthesized comm.
    except (OSError, IndexError):
        return None


def viewer_active(state):
    record = state.get('mvw', {})
    start = process_start(record.get('pid'))
    return start is not None and str(record.get('start_ticks')) == start


def service_active():
    try:
        return subprocess.run(['systemctl', 'is-active', '--quiet', SERVICE],
                              timeout=2, check=False).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def camera_settings():
    sys.path.insert(0, str(MODULE/'roboscan_nsl3130/scripts'))
    from sirius_camera_config import inventory
    data, path = inventory()
    return path, data


def calibration_digest(include_map=True):
    path, data = camera_settings()
    records = {}
    for camera, entry in sorted(data.get('cameras', {}).items()):
        selected = {key: entry.get(key) for key in ('ip', 'serial', 'parameters', 'sensor_host', 'sensor_source_address',
                                                    'intrinsic', 'distortion', 'extrinsic', 'tf')}
        if not include_map:
            selected.pop('tf', None)
        records[camera] = selected
        pointer = entry.get('parameters')
        if pointer:
            folder = Path(pointer).expanduser()
            if not folder.is_absolute():
                folder = path.parent/folder
            for name in ('multiview.yml', 'intrinsic.yml', 'extrinsic.yml', 'params.yaml', 'camera_info.yaml'):
                file = folder/name
                if file.is_file():
                    records[f'{camera}/{name}'] = digest(file)
        serial = entry.get('serial')
        if serial:
            folder = MODULE/'calib_output'/str(serial)
            for name in ('multiview.yml', 'intrinsic.yml', 'extrinsic.yml', 'params.yaml', 'camera_info.yaml'):
                file = folder/name
                if file.is_file():
                    records[f'{camera}/sdk/{name}'] = digest(file)
    return hashlib.sha256(json.dumps(records, sort_keys=True).encode()).hexdigest()


def measured_receipt(mtf, cameras=None, inventory=None):
    records = mtf.get('sdk_writeback', {})
    selected = ['cam_'+str(o).removeprefix('cam_') for o in cameras] if cameras else list(records)
    if not mtf.get('token') or not selected:
        return False
    if mtf.get('measurement_sha256') and mtf['measurement_sha256'] != calibration_digest(include_map=False):
        return False
    for name in selected:
        record = records.get(name, {})
        file = Path(record.get('file') or '/nonexistent')
        if not file.is_file() or digest(file) != record.get('sha256'):
            return False
        if inventory and record.get('serial') != inventory.get('cameras', {}).get(name, {}).get('serial'):
            return False
    return True


def require(stage, state=None, calibration=None):
    state = read_yaml(STATE) if state is None else state
    if stage == 'mvw':
        if not viewer_active(state) or not service_active():
            raise ValueError('Open mvw first; its camera viewer and sensor_streaming service must be active.')
        return state
    trained = state.get('mtr', {})
    if stage == 'mvp' and trained.get('result') == 'trained_saved_alignment':
        if not trained.get('ready'):
            raise ValueError('Run mtr to train using the saved mdt alignment first.')
        for file, sha256 in trained.get('files', {}).items():
            if not Path(file).is_file() or digest(file) != sha256:
                raise ValueError('Pose config or weights changed; run mtr again.')
        fingerprint = calibration if calibration is not None else calibration_digest(include_map=False)
        if trained.get('calibration_sha256') != fingerprint:
            raise ValueError('Measured camera calibration changed; run mtf and mtr again.')
        return state
    fingerprint = calibration if calibration is not None else calibration_digest()
    mtf = state.get('mtf', {})
    if not mtf.get('token') or mtf.get('calibration_sha256') != fingerprint:
        raise ValueError('Run mtf successfully for the current camera calibration first.')
    if stage == 'mtf':
        require('mvw', state)
        if mtf.get('mvw_token') != state.get('mvw', {}).get('token'):
            raise ValueError('Run mtf in the current mvw session first.')
        return state
    mtr = state.get('mtr', {})
    if not mtr.get('ready') or mtr.get('mtf_token') != mtf['token']:
        raise ValueError('Run mtr and wait for the single/multiview pose nodes first.')
    for file, sha256 in mtr.get('files', {}).items():
        if not Path(file).is_file() or digest(file) != sha256:
            raise ValueError('Pose config or weights changed; run mtr again.')
    if stage == 'mvp' and viewer_active(state):
        raise ValueError('Close the mvw window and wait for its command to finish before mvp.')
    return state


def launch_viewer(mode):
    if mode == 'map_pose':
        print('[mvp] Checking trained models and saved MDT marker placement...', flush=True)
    current = read_yaml(STATE)
    trained = current.get('mtr', {}).get('result') == 'trained_saved_alignment'
    if mode == 'camera' and trained:
        try:
            require('mvp', current)
            prepared = current['mtr']['preparation']
            manual = saved_mtd()
            mode = ('map_pose' if manual is not None and f"id_{prepared['marker_id']}" in
                    read_yaml(manual).get('anchor_states', {}) else 'measurement')
        except ValueError as error:
            print(f'[mvw] {error} Showing camera streams.', file=sys.stderr)
    if mode == 'camera_pose_robot' and trained:
        mode = 'map_pose'
    if mode == 'map_pose' and not trained:
        print('[mvp] Reusing existing models with saved MDT placement; no new mtf or training is claimed.', flush=True)
        mode = prepare_mdt_mode(allow_open_viewer=True)
    elif mode in ('measurement', 'camera_pose', 'map_pose'):
        require('mvp', current)
        prepared = current['mtr']['preparation']
        manual = saved_mtd() if mode == 'map_pose' else None
        if mode == 'map_pose':
            if manual is None or f"id_{prepared['marker_id']}" not in read_yaml(manual).get('anchor_states', {}):
                raise ValueError('Save the measured marker placement in mdt before mvp. Training remains available without mdt.')
            if current['mtr'].get('display_session_sha256') != digest(manual):
                print('[mvp] Applying saved MDT placement; camera and pose services are restarting...', flush=True)
                selected_at = time.time_ns()
                subprocess.run(['python3', '-m', 'sirius_robot_dt.pipeline', '--select', 'camera_pose_robot',
                                '--pose-settings', prepared['config_path'], '--restart'], check=True)
                readiness = wait_pose_ready(prepared, selected_at)
                with locked_state() as updated:
                    updated['mtr']['display_session_sha256'] = digest(manual)
                    updated['mtr']['readiness'] = readiness
            mode = 'camera_pose_robot'
        else:
            subprocess.run(['python3', '-m', 'sirius_robot_dt.pipeline', '--select', 'camera_pose',
                            '--pose-settings', prepared['config_path']], check=True)
            mode = 'camera_pose'
    elif mode == 'camera_pose_robot':
        mode = prepare_mdt_mode()
    launch = {'camera': 'multiview_camera.launch.py', 'camera_pose': 'multiview_camera_pose.launch.py',
              'camera_pose_robot': 'multiview_camera_pose_robot.launch.py',
              'camera_robot': 'multiview_camera_robot.launch.py'}[mode]
    fingerprint = calibration_digest()
    print('[view] Opening RViz...', flush=True)
    child = subprocess.Popen(['ros2', 'launch', str(ROOT/'exec/launch'/launch)], start_new_session=True)
    token = uuid.uuid4().hex
    if mode in ('camera', 'camera_pose', 'camera_pose_robot', 'camera_robot'):
        with locked_state() as state:
            state['mvw'] = {'pid': child.pid, 'start_ticks': process_start(child.pid), 'token': token, 'started_at': now()}
            # Reopening the viewer does not change a persisted calibration.
            # Keep its receipt only while the exact calibration bytes still match.
            mtf = state.get('mtf', {})
            if mtf.get('token') and (mtf.get('calibration_sha256') == fingerprint or measured_receipt(mtf)):
                mtf['mvw_token'] = token
            else:
                state.pop('mtf', None)
            # Camera-only mode stops pose nodes, so pose readiness must be renewed.
            if mode == 'camera':
                state.pop('mtr', None)
    old_handlers = {}
    def forward(number, frame):
        if child.poll() is None:
            os.killpg(child.pid, number)
    for number in (signal.SIGINT, signal.SIGTERM):
        old_handlers[number] = signal.signal(number, forward)
    try:
        return child.wait()
    finally:
        for number, handler in old_handlers.items():
            signal.signal(number, handler)
        if mode in ('camera', 'camera_pose', 'camera_pose_robot', 'camera_robot'):
            with locked_state() as state:
                if state.get('mvw', {}).get('token') == token:
                    state['mvw']['closed_at'] = now()
                    state['mvw']['exit_code'] = child.returncode


def mark_mtf(cameras, since):
    if not math.isfinite(since) or since < 0:
        raise ValueError('mtf request time must be a finite nonnegative timestamp.')
    state = require('mvw')
    if not cameras or any(not str(camera).isdigit() or not 161 <= int(camera) <= 165 for camera in cameras):
        raise ValueError('Current mtf requires observed cameras 161..165.')
    inventory_file, data = camera_settings()
    completed = {}
    for camera in cameras:
        name = f'cam_{camera}'
        binding = data.get('cameras', {}).get(name, {})
        serial = binding.get('serial')
        if not serial or binding.get('serial_verified') is not True:
            raise ValueError(f'{name}: physical serial is not verified; complete automatic device discovery before mtf.')
        pointer = binding.get('parameters')
        folder = Path(pointer).expanduser() if pointer else MODULE/'calib_output'/str(serial)
        if not folder.is_absolute():
            folder = inventory_file.parent/folder
        file = folder/'multiview.yml'
        if not file.is_file():
            file = MODULE/'calib_output'/str(serial)/'multiview.yml'
        if not file.is_file() or file.stat().st_mtime < since:
            raise ValueError(f'{name}: completed SDK multiview writeback was not confirmed for this mtf request.')
        completed[name] = {'serial': serial, 'file': str(file.resolve()), 'sha256': digest(file)}
    fingerprint = calibration_digest()
    with locked_state() as current:
        if current.get('mvw', {}).get('token') != state.get('mvw', {}).get('token') or not viewer_active(current):
            raise ValueError('mvw closed or changed before mtf finished.')
        current['mtf'] = {'token': uuid.uuid4().hex, 'mvw_token': current['mvw']['token'],
                          'completed_at': now(), 'cameras': sorted(set(map(str, cameras))),
                          'calibration_sha256': fingerprint, 'sdk_writeback': completed,
                          'measurement_sha256': calibration_digest(include_map=False),
                          'result': 'fresh_stag_observations_and_sdk_writeback'}
        current.pop('mtr', None)
    print('mtf: current anchor observations and completed SDK writeback recorded. Global map accuracy remains unverified.')


def session_file():
    manifest = read_yaml(ROOT/'config/config.yaml')
    pointer = manifest.get('localization')
    if isinstance(pointer, dict):
        pointer = pointer.get('session')
    if pointer:
        path = Path(pointer).expanduser()
        return path if path.is_absolute() else ROOT/'config'/path
    return ROOT/'controller/src/robot_dt_bridge/localization/calibration/session.yaml'


def saved_mtd():
    path = session_file().resolve()
    session = read_yaml(path)
    snapshot = session.get('mtd_snapshot')
    if not isinstance(snapshot, dict) or snapshot.get('saved') is not True or snapshot.get('frozen') is not True:
        return None
    archive = snapshot.get('source') == 'legacy_manual_two_robot_session'
    if not snapshot.get('saved_at') and not (archive and snapshot.get('timestamp_source') == 'original_save_time_unavailable'):
        return None
    inventory_file = snapshot.get('snapshot_file')
    if not inventory_file:
        raise ValueError('Saved mtd has no capture hash inventory.')
    inventory_path = (path.parent/str(inventory_file)).resolve()
    if not inventory_path.is_file():
        raise ValueError(f'Saved mtd capture inventory is missing: {inventory_path}. '
                         'Restore the original snapshot directory, or run mtd and explicitly save a new placement before mvp.')
    inventory = read_yaml(inventory_path)
    cameras = snapshot.get('camera_ids')
    if not isinstance(cameras, list) or not cameras:
        raise ValueError('Saved mtd has no camera captures.')
    for camera in cameras:
        name = str(camera) if str(camera).startswith('cam_') else f'cam_{camera}'
        capture = inventory.get('cameras', {}).get(name, {})
        file = inventory_path.parent/str(capture.get('file', ''))
        sha256 = capture.get('sha256')
        if not file.is_file():
            raise ValueError(f'Saved mtd capture file is missing for {name}: {file}. '
                             'Restore the original snapshot directory, or run mtd and explicitly save a new placement before mvp.')
        if (not isinstance(sha256, str) or not file.is_file() or digest(file) != sha256
                or session.get('camera_states', {}).get(name, {}).get('capture_sha256') != sha256):
            raise ValueError(f'Saved mtd capture hash does not match: {name}')
    for role in ('loading', 'unloading'):
        joints = session.get('robot_states', {}).get(role, {}).get('controller_deg')
        if (not isinstance(joints, list) or len(joints) != 6 or any(isinstance(value, bool)
                or not isinstance(value, (int, float)) or not math.isfinite(value) for value in joints)):
            raise ValueError(f'Saved mtd has no complete six-axis pose for {role}.')
    return path


def run_mtd(auto=False):
    if not service_active():
        subprocess.run(['python3', '-m', 'sirius_robot_dt.pipeline', '--select', 'camera'], check=True)
    previous = read_yaml(session_file()).get('mtd_snapshot', {}).get('id')
    backend = ROOT/'controller/src/robot_dt_bridge/localization/tools/mtd.py'
    command = ['python3', str(backend), '--config', str(ROOT/'config/config.yaml')]
    if auto:
        command.append('--auto-mode')
    result = subprocess.run(command, check=False)
    if result.returncode:
        return result.returncode
    path = saved_mtd()
    snapshot = read_yaml(path).get('mtd_snapshot', {}) if path is not None else {}
    if (path is None or not snapshot.get('id') or snapshot['id'] == previous
            or snapshot.get('source') != 'live_point_cloud_one_frame'):
        raise ValueError('No new mtd snapshot was saved. The previous saved TF remains available to MDT.')
    with locked_state() as state:
        state['mtd'] = {'session': str(path), 'sha256': digest(path), 'saved_at': now()}
    return 0


def save_mtd():
    command = ROOT/'controller/src/robot_dt_bridge/localization/start_two_robots.sh'
    result = subprocess.run(['bash', str(command), 'save'], check=False)
    if result.returncode:
        return result.returncode
    path = saved_mtd()
    if path is None:
        raise ValueError('The current editor session is not an mtd capture; open mtd first.')
    with locked_state() as state:
        state['mtd'] = {'session': str(path), 'sha256': digest(path), 'saved_at': now()}
    return 0


def ready_status(prepared, key, payload, selected_at_ns, config):
    if not isinstance(payload, dict) or payload.get('state') != 'ready':
        return False
    if (payload.get('marker_id') != prepared['marker_id']
            or payload.get('reference_frame') != prepared['reference_frame']
            or not isinstance(payload.get('initialized_at_ns'), int)
            or payload['initialized_at_ns'] < selected_at_ns
            or payload['initialized_at_ns'] > time.time_ns()+5_000_000_000):
        return False
    if key == 'fusion':
        return payload.get('kind') == 'fusion' and payload.get('config_sha256') == prepared['config_sha256']
    binding = config.get('pose_workflow', {}).get('cameras', {}).get(key, {})
    return (payload.get('kind') == 'single' and payload.get('models_loaded') is True
            and payload.get('serial') == binding.get('serial')
            and payload.get('config_sha256') == prepared['config_sha256']
            and payload.get('octet') == key.removeprefix('cam_'))


def wait_pose_ready(prepared, selected_at_ns, timeout=120):
    """Require fresh latched model status and live nodes, without requiring a person."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    context = Context()
    rclpy.init(context=context)
    node = Node('sirius_pose_readiness_'+uuid.uuid4().hex[:8], context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    statuses = {}
    rejected = {}
    config = read_yaml(prepared['config_path'])
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    def received(key, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if ready_status(prepared, key, payload, selected_at_ns, config):
            statuses[key] = payload
            rejected.pop(key, None)
        elif isinstance(payload, dict) and isinstance(payload.get('initialized_at_ns'), int):
            stamp = payload['initialized_at_ns']
            if stamp < selected_at_ns or stamp > time.time_ns()+5_000_000_000:
                rejected[key] = 'stale status or unsynchronized Edge clock; check timedatectl status on Host/Edge'
            else:
                rejected[key] = 'prepared config/serial/marker binding did not match ready status'
    subscriptions = []
    for key, topic in prepared['ready_topics'].items():
        subscriptions.append(node.create_subscription(String, topic, lambda message, key=key: received(key, message), qos))
    expected_nodes = {'/multiview_pose_fusion'} | {f'/cam_{octet}/singleview_pose' for octet in prepared['camera_octets']}
    started = time.monotonic()
    deadline = started+timeout
    next_update = started
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(.5, max(0, deadline-time.monotonic())))
            live_nodes = {namespace.rstrip('/')+'/'+name for name, namespace in node.get_node_names_and_namespaces()}
            if set(statuses) == set(prepared['ready_topics']) and expected_nodes <= live_nodes and service_active():
                print('[pose] Edge models and fusion are ready.', flush=True)
                return statuses
            if time.monotonic() >= next_update:
                pending = sorted(set(prepared['ready_topics'])-set(statuses))
                pending += sorted(expected_nodes-live_nodes)
                waiting = ', '.join(pending) or 'sensor streaming service'
                print(f'[pose] Waiting for {waiting} '
                      f'({int(time.monotonic()-started)}/{timeout:g}s)...', flush=True)
                next_update = time.monotonic()+10
        missing = set(prepared['ready_topics'])-set(statuses)
        reason = '; '.join(f'{key}: {value}' for key, value in sorted(rejected.items()))
        raise ValueError('Fresh pose readiness was not confirmed for '+', '.join(sorted(missing or expected_nodes-live_nodes))
                         + ('. '+reason if reason else ''))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def prepare_pose(session, cameras, *, train=False):
    helper = ROOT/'controller/src/ros_humanpose/setup/prepare_pose_workflow.py'
    fleet, _ = camera_settings()
    native = ROOT/'controller/src/ros_humanpose/config/config.yml'
    manifest = read_yaml(ROOT/'config/config.yaml')
    pointer = (manifest.get('pose') or {}).get('config') if isinstance(manifest.get('pose'), dict) else None
    pointer = pointer or manifest.get('pose_settings')
    if pointer:
        native = Path(pointer).expanduser()
        if not native.is_absolute():
            native = ROOT/'config'/native
    output = ROOT/'controller/src/ros_humanpose/config/runtime/pose_workflow.yaml'
    command = ['python3', str(helper), '--config', str(native), '--fleet', str(fleet), '--out', str(output),
               '--octets', ','.join(cameras)]
    if session is not None:
        command += ['--reuse-mtd', str(session)]
    if not train:
        models = read_yaml(native).get('pose_workflow', {}).get('model_dir')
        if models:
            command += ['--model-dir', str(models)]
    if train:
        command = ['python3', str(helper.with_name('train_pose_workflow.py')),
                   '--config', str(native), '--fleet', str(fleet),
                   '--octets', ','.join(cameras)]
        # Training prints progress to stderr; stdout contains only its receipt.
        result = subprocess.run(command, stdout=subprocess.PIPE, text=True)
    else:
        result = subprocess.run(command, capture_output=True, text=True)
    try:
        prepared = json.loads(result.stdout)
    except ValueError as error:
        raise ValueError((result.stderr or '').strip() or 'Pose preflight returned no valid receipt.') from error
    if result.returncode or prepared.get('success') is not True:
        raise ValueError(prepared.get('error') or (result.stderr or '').strip() or 'Pose preflight failed.')
    return prepared


def share_prepared_pose(prepared):
    tools = str(MODULE/'setup/fleet_tools.sh')
    subprocess.run(['bash', tools, '--internal-command', 'share-pose-config',
                    str(Path(prepared['config_path']).resolve()), *prepared['camera_octets']], check=True)
    for camera, files in prepared.get('runtime_override_files', {}).items():
        octet = camera.removeprefix('cam_')
        if octet not in prepared['camera_octets']:
            raise ValueError('Prepared calibration override is not bound to a selected camera.')
        for file in files:
            subprocess.run(['bash', tools, '--internal-command', 'share-runtime-file', str(file), octet], check=True)
    for file in prepared.get('weight_files', []):
        path = Path(file).resolve()
        if (ROOT/'controller/src/ros_humanpose/config/runtime') in path.parents:
            subprocess.run(['bash', tools, '--internal-command', 'share-runtime-file', str(path),
                            *prepared['camera_octets']], check=True)


def run_pose():
    mtf = read_yaml(STATE).get('mtf', {})
    cameras = [str(value).removeprefix('cam_') for value in mtf.get('cameras', [])]
    _, inventory = camera_settings()
    if not cameras or not measured_receipt(mtf, cameras, inventory):
        raise ValueError('Current marker-to-RGB-D measurements are missing or changed. Complete mtf, then mtr.')
    for octet in cameras:
        name = 'cam_'+octet
        observed = inventory.get('cameras', {}).get(name, {})
        if observed.get('serial_verified') is not True:
            raise ValueError(name+': current camera serial is not verified; complete device discovery before mtr.')
    calibration = calibration_digest(include_map=False)
    print('[mtr] Training marker-relative models from mtf measurements; mdt controls map placement.', flush=True)
    prepared = prepare_pose(None, cameras, train=True)
    if calibration_digest(include_map=False) != calibration:
        raise ValueError('Alignment changed during training. Earlier models remain selected; rerun mtr.')
    for warning in prepared.get('warnings', []):
        print('mtr: '+warning, file=sys.stderr)
    path = Path(prepared['config_path']).resolve()
    share_prepared_pose(prepared)
    selected_at_ns = time.time_ns()
    subprocess.run(['python3', '-m', 'sirius_robot_dt.pipeline', '--select', 'camera_pose', '--pose-settings', str(path), '--restart'], check=True)
    ready = wait_pose_ready(prepared, selected_at_ns)
    files = {str(path): digest(path)}
    for camera_files in prepared.get('runtime_override_files', {}).values():
        for file in camera_files:
            files[str(Path(file).resolve())] = digest(file)
    for file in prepared.get('weight_files', []):
        files[str(Path(file).resolve())] = digest(file)
    for key in ('training_config',):
        if prepared.get(key):
            file = str(Path(prepared[key]).resolve())
            files[file] = digest(file)
    with locked_state() as state:
        if calibration_digest(include_map=False) != calibration:
            raise ValueError('Alignment changed during mtr; save mdt and rerun mtr.')
        state['mtr'] = {'ready': True, 'calibration_sha256': calibration, 'completed_at': now(),
                        'display_session_sha256': None,
                        'readiness': ready, 'files': files, 'preparation': prepared,
                        'result': 'trained_saved_alignment',
                        'accuracy_verified': False}
    print('mtr: measured camera/marker view ready. mvp publishes poses using the saved mdt marker placement.')
    return launch_viewer('measurement')


def prepare_mdt_mode(*, allow_open_viewer=False):
    manual = saved_mtd()
    if manual is None:
        raise ValueError('Save the manual global TF with mtd first.')
    state = read_yaml(STATE)
    if viewer_active(state) and not allow_open_viewer:
        raise ValueError('Close mvw before opening MDT.')
    session = read_yaml(manual)
    marker = session.get('reference_anchor_id')
    try:
        require('mvp', state)
        prepared = state['mtr']['preparation']
        if prepared.get('weight_reference_verified') is not True:
            raise ValueError('Pose model coordinate references are unverified; anchor-local mvp remains available.')
        if prepared['marker_id'] != marker:
            raise ValueError('Pose physical marker differs from the saved global TF reference.')
        return 'camera_pose_robot'
    except ValueError:
        pass
    try:
        cameras = [str(value).removeprefix('cam_') for value in session['mtd_snapshot']['camera_ids']]
        prepared = prepare_pose(None, cameras)
        if prepared.get('weight_reference_verified') is not True:
            raise ValueError('SDK pose models have unverified coordinate references; global human placement is not established.')
        if prepared['marker_id'] != marker:
            raise ValueError(f"SDK pose marker {prepared['marker_id']} differs from saved map reference {marker}; global human placement is not established.")
        path = Path(prepared['config_path']).resolve()
        share_prepared_pose(prepared)
        selected_at_ns = time.time_ns()
        subprocess.run(['python3', '-m', 'sirius_robot_dt.pipeline', '--select', 'camera_pose_robot',
                        '--pose-settings', str(path), '--restart'], check=True)
        ready = wait_pose_ready(prepared, selected_at_ns)
        with locked_state() as current:
            current['mdt_pose'] = {'result': 'single_models_loaded_fusion_initialized', 'completed_at': now(),
                                   'session_sha256': digest(manual), 'preparation': prepared,
                                   'readiness': ready, 'accuracy_verified': False,
                                   'source': 'existing_models_and_saved_mdt', 'fresh_mtf_claimed': False}
        return 'camera_pose_robot'
    except (ValueError, OSError, subprocess.SubprocessError, KeyError) as error:
        print(f'[mdt] Saved camera/robot TF remains available. Pose is unprepared: {error}', file=sys.stderr)
        return 'camera_robot'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    view = sub.add_parser('view')
    view.add_argument('mode', choices=('camera', 'camera_pose', 'camera_pose_robot', 'measurement', 'map_pose'))
    check = sub.add_parser('require')
    check.add_argument('stage', choices=('mvw', 'mtf', 'mvp'))
    observed = sub.add_parser('mark-mtf')
    observed.add_argument('cameras', nargs='+')
    observed.add_argument('--since', type=float, required=True)
    invalidate = sub.add_parser('invalidate')
    invalidate.add_argument('reason')
    mtd = sub.add_parser('mtd')
    mtd.add_argument('--auto', action='store_true')
    sub.add_parser('save-mtd')
    sub.add_parser('session-file')
    sub.add_parser('pose')
    args = parser.parse_args()
    try:
        if args.command == 'view':
            return launch_viewer(args.mode)
        if args.command == 'require':
            require(args.stage)
        elif args.command == 'mark-mtf':
            mark_mtf(args.cameras, args.since)
        elif args.command == 'invalidate':
            with locked_state() as state:
                state.pop('mtf', None)
                state.pop('mtr', None)
                state['invalidated'] = {'reason': args.reason, 'at': now()}
        elif args.command == 'mtd':
            return run_mtd(args.auto)
        elif args.command == 'save-mtd':
            return save_mtd()
        elif args.command == 'session-file':
            print(session_file().resolve())
        elif args.command == 'pose':
            return run_pose()
        return 0
    except (ValueError, OSError, subprocess.SubprocessError, KeyError) as error:
        print(f'[{args.command}] {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
