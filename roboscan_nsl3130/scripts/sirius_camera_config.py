#!/usr/bin/env python3
"""Resolve compact camera profiles and eligible anchor IDs without touching hardware."""
import os
import hashlib
import json
from pathlib import Path
import re
import tempfile
import yaml

SDK_FILES = ('params.yaml', 'camera_info.yaml', 'intrinsic.yml', 'extrinsic.yml', 'multiview.yml')
DRIVER_FIELDS = ('sensor_host', 'sensor_source_address', 'lidar_frame', 'rgb_frame', 'camera_info_topic')


def project_root(path=None):
    if path and Path(path).parent.name == 'config':
        candidate = Path(path).parent.parent
        if (candidate/'controller/src/NSL-3130AA-ROS2/setup/fleet.yaml').is_file():
            return candidate
    if os.environ.get('SIRIUS_ROBOT_DT_ROOT'):
        return Path(os.environ['SIRIUS_ROBOT_DT_ROOT']).expanduser().resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent/'controller/src/NSL-3130AA-ROS2/setup/fleet.yaml').is_file():
            return parent
    return None


def native_fleet(path=None):
    root = project_root(path)
    fleet = root/'controller/src/NSL-3130AA-ROS2/setup/fleet.yaml' if root else Path(__file__).resolve().parents[2]/'setup/fleet.yaml'
    value = yaml.safe_load(fleet.read_text()) or {} if fleet.is_file() else {}
    return value, fleet


def native_inventory():
    root = os.environ.get('SIRIUS_ROBOT_DT_ROOT')
    if root:
        root = Path(root).expanduser()
        common = root / 'config/config.yaml'
        if common.is_file() and (yaml.safe_load(common.read_text()) or {}).get('schema_version') in (2, 3):
            return common
        return root / 'controller/src/NSL-3130AA-ROS2/setup/fleet.yaml'
    root = project_root()
    if root:
        common = root/'config/config.yaml'
        if common.is_file() and (yaml.safe_load(common.read_text()) or {}).get('schema_version') in (2, 3):
            return common
    for parent in Path(__file__).resolve().parents:
        common = parent / 'config/config.yaml'
        if common.is_file() and (yaml.safe_load(common.read_text()) or {}).get('schema_version') in (2, 3):
            return common
        for path in (parent / 'setup/fleet.yaml',
                     parent / 'controller/src/NSL-3130AA-ROS2/setup/fleet.yaml'):
            if path.is_file():
                return path
    return None


def is_native(data):
    return data.get('schema_version') == 1 and isinstance(data.get('server'), dict) and isinstance(data.get('cameras'), dict)


def inventory(config_path=None):
    if config_path is None:
        config_path = os.environ.get('SIRIUS_CONFIG')
        if not config_path:
            config_path = native_inventory()
        if config_path is None:
            return {}, None
    path = Path(config_path).expanduser().resolve()
    if not path.is_file():
        if os.environ.get('SIRIUS_CONFIG'):
            raise FileNotFoundError(f'Configured SiriusRobotDT inventory does not exist: {path}')
        return {}, path
    value = yaml.safe_load(path.read_text()) or {}
    if not isinstance(value, dict):
        raise ValueError('SiriusRobotDT inventory must be a YAML mapping')
    if value.get('schema_version') == 3:
        reference = value.get('camera_settings')
        if not isinstance(reference, str) or not reference.strip():
            raise ValueError('Common config camera_settings must name the camera fleet file')
        target = Path(reference).expanduser()
        target = (target if target.is_absolute() else path.parent / target).resolve()
        if target == path:
            raise ValueError('Common config camera_settings cannot refer to itself')
        camera_data = yaml.safe_load(target.read_text()) or {}
        if not isinstance(camera_data, dict) or camera_data.get('schema_version') == 3:
            raise ValueError('camera_settings must refer directly to a camera fleet mapping')
        if 'anchor_id' in value:
            camera_data['anchor'] = {'id': value['anchor_id']}
        value, path = camera_data, target
    if value.get('schema_version') == 2:
        # Only the public discovered entries belong to the inventory. Native
        # addresses of not-yet-discovered devices are bootstrap data, not new entries.
        value = dict(value)
        value['cameras'] = {name: camera_entry(value, path, name)
                            for name in value.get('cameras', {})}
    return (value if value.get('schema_version') == 2 or is_native(value) else {}), path


def camera_entry(data, path, namespace, bootstrap=False):
    public = data.get('cameras', {}).get(namespace)
    if is_native(data):
        return dict(public or {})
    native, fleet = native_fleet(path)
    original = native.get('cameras', {}).get(namespace, {})
    if public is None:
        if not bootstrap:
            return {}
        entry = dict(original)
        if entry.get('parameters'):
            pointer = Path(entry['parameters']).expanduser()
            entry['parameters'] = str((pointer if pointer.is_absolute() else fleet.parent/pointer).resolve())
        return entry
    entry = {key: original[key] for key in DRIVER_FIELDS if key in original}
    sidecar = path.parent/'cameras'/f'{namespace}.yaml' if path else None
    details = yaml.safe_load(sidecar.read_text()) or {} if sidecar and sidecar.is_file() else {}
    entry.update({key: details[key] for key in DRIVER_FIELDS if key in details})
    entry.update(public)
    if isinstance(public.get('sensor'), dict):
        entry['sensor_host'] = public['sensor'].get('ip', entry.get('sensor_host'))
        entry['sensor_source_address'] = public['sensor'].get('source', entry.get('sensor_source_address'))
    # Verification is observation metadata, never inferred from an address/name.
    runtime = fleet.parent.parent/'calib_output/runtime'/namespace/'device_info.yaml'
    observed = yaml.safe_load(runtime.read_text()) or {} if runtime.is_file() else {}
    for cache in (original, details, observed):
        if cache.get('serial') and cache.get('serial') == public.get('serial'):
            verified = cache.get('serial_verified', cache.get('verified'))
            if verified is True:
                entry['serial_verified'] = True
                entry['serial_source'] = cache.get('serial_source', cache.get('source'))
    return entry


def camera_cache(data, path, namespace):
    if is_native(data):
        return data.get('cameras', {}).get(namespace, {})
    return camera_entry(data, path, namespace, bootstrap=True)


def camera_runtime(namespace, detected_serial, calibration_root, config_path=None):
    """Physical discovery overrides the internal cache; keep fallback for Ethernet."""
    data, path = inventory(config_path)
    namespace = namespace.strip('/')
    camera = camera_entry(data, path, namespace, bootstrap=True)
    cache = camera_cache(data, path, namespace)
    assigned = cache.get('serial') or camera.get('serial') or ''
    detected = detected_serial or ''
    serial = detected or assigned
    profile = None
    replacement = bool(assigned and detected and assigned != detected)
    if camera.get('parameters') and path and not replacement:
        configured = Path(camera['parameters']).expanduser()
        configured = (configured if configured.is_absolute() else path.parent / configured).resolve()
        if any((configured / name).is_file() for name in SDK_FILES):
            profile = configured
    if profile is None and serial:
        profile = Path(calibration_root).expanduser().resolve() / serial
    return serial, str(profile) if profile else ''


def raw_profile_file(profile, calibration_root, serial, name):
    """Resolve untouched SDK bytes, retaining per-file partial-profile fallback."""
    preferred = Path(profile).expanduser() / name if profile else None
    if preferred is not None and preferred.is_file():
        return preferred
    return Path(calibration_root).expanduser() / serial / name


def runtime_override_files(namespace, serial, profile, calibration_root, config_path=None, detected_serial=''):
    """Materialize bound public calibration edits without changing SDK originals."""
    data, path = inventory(config_path)
    camera = data.get('cameras', {}).get(namespace, {})
    if (data.get('schema_version') != 2 or not serial or camera.get('serial') != serial
            or not (detected_serial == serial or camera.get('serial_verified') is True)):
        return {}
    fields = {key: camera.get(key) for key in ('intrinsic', 'distortion', 'extrinsic')}
    if all(value is None for value in fields.values()):
        return {}
    if camera.get('parameters') and path and profile:
        configured = Path(camera['parameters']).expanduser()
        configured = (configured if configured.is_absolute() else path.parent/configured).resolve()
        # Explicit profile CLI overrides retain precedence over public edits.
        if Path(profile).expanduser().resolve() != configured and any((configured/name).is_file() for name in SDK_FILES):
            return {}
    import cv2
    import numpy as np
    template = raw_profile_file(profile, calibration_root, serial, 'intrinsic.yml')
    metadata = {}
    matrix = coefficients = None
    if template.is_file():
        storage = cv2.FileStorage(str(template), cv2.FILE_STORAGE_READ)
        try:
            identity_node = storage.getNode('camera_id')
            if not identity_node.empty() and identity_node.string() != serial:
                raise ValueError(f'{namespace}: SDK intrinsic belongs to another physical serial')
            matrix = storage.getNode('camera_matrix').mat()
            coefficients = storage.getNode('distortion_coefficients').mat()
            for key in ('image_width', 'image_height'):
                node = storage.getNode(key)
                if not node.empty():
                    metadata[key] = int(node.real())
            node = storage.getNode('distortion_model')
            if not node.empty():
                metadata['distortion_model'] = node.string()
        finally:
            storage.release()
    original_matrix = None if matrix is None else matrix.copy()
    original_coefficients = None if coefficients is None else coefficients.copy()
    identity = {'serial': serial, **fields, 'intrinsic_template_sha256':
                hashlib.sha256(template.read_bytes()).hexdigest() if template.is_file() else None}
    fingerprint = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    folder = Path(calibration_root).expanduser().resolve()/'runtime'/namespace/'overrides'/fingerprint
    files = {}

    def write_file(name, values):
        destination = folder/name
        if not destination.is_file():
            folder.mkdir(parents=True, exist_ok=True)
            fd, temporary = tempfile.mkstemp(suffix='.yml', dir=folder)
            os.close(fd)
            storage = cv2.FileStorage(temporary, cv2.FILE_STORAGE_WRITE)
            try:
                storage.write('camera_id', serial)
                for key, value in values.items():
                    storage.write(key, value)
            finally:
                storage.release()
            try:
                os.replace(temporary, destination)
            finally:
                Path(temporary).unlink(missing_ok=True)
        files[name] = destination

    if fields['intrinsic'] is not None or fields['distortion'] is not None:
        if fields['intrinsic'] is not None:
            matrix = np.asarray(fields['intrinsic'], dtype=float).reshape(3, 3)
        if fields['distortion'] is not None:
            coefficients = np.asarray(fields['distortion'], dtype=float).reshape(1, -1)
        if (matrix is None or coefficients is None or not metadata.get('distortion_model')
                or not metadata.get('image_width') or not metadata.get('image_height')):
            raise ValueError(f'{namespace}: inline intrinsic needs the verified SDK image size/distortion model and coefficients')
        if (not np.isfinite(matrix).all() or not np.isfinite(coefficients).all()
                or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                or not np.allclose(matrix[2], [0, 0, 1])):
            raise ValueError(f'{namespace}: inline intrinsic is invalid')
        unchanged = (original_matrix is not None and original_coefficients is not None
                     and matrix.shape == original_matrix.shape and coefficients.shape == original_coefficients.shape
                     and np.allclose(matrix, original_matrix, rtol=0, atol=1e-12)
                     and np.allclose(coefficients, original_coefficients, rtol=0, atol=1e-12))
        if not unchanged:
            write_file('intrinsic.yml', {**metadata, 'camera_matrix': matrix, 'distortion_coefficients': coefficients})
            info_source = raw_profile_file(profile, calibration_root, serial, 'camera_info.yaml')
            info = yaml.safe_load(info_source.read_text()) or {} if info_source.is_file() else {}
            # Preserve the physical image size/model, update every projection field
            # consumed by native pose and keep the SDK source untouched.
            info.update(metadata, K=matrix.reshape(-1).tolist(), D=coefficients.reshape(-1).tolist(),
                        fx=float(matrix[0, 0]), fy=float(matrix[1, 1]),
                        cx=float(matrix[0, 2]), cy=float(matrix[1, 2]))
            info.pop('rmse_px', None)  # SDK fit quality does not verify an operator edit.
            info['source'] = 'SiriusRobotDT public runtime calibration override'
            if metadata['distortion_model'] in ('equidistant', 'fisheye'):
                info['camera_model'] = 'fisheye'
            else:
                info.setdefault('camera_model', 'pinhole')
            for key in ('fx', 'fy', 'cx', 'cy'):
                info[key+'_rectified'] = info[key]
            coefficient_keys = ('k1', 'k2', 'k3', 'k4') if info['camera_model'] == 'fisheye' else ('k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6')
            for key in ('k1', 'k2', 'k3', 'k4', 'k5', 'k6', 'p1', 'p2'):
                info.pop(key, None)
            for key, value in zip(coefficient_keys, coefficients.reshape(-1)):
                info[key] = float(value)
            destination = folder/'camera_info.yaml'
            if not destination.is_file():
                fd, temporary = tempfile.mkstemp(suffix='.yaml', dir=folder)
                try:
                    with os.fdopen(fd, 'w') as file:
                        yaml.safe_dump(info, file, sort_keys=False)
                    os.replace(temporary, destination)
                finally:
                    Path(temporary).unlink(missing_ok=True)
            files['camera_info.yaml'] = destination
    if fields['extrinsic'] is not None:
        transform = np.asarray(fields['extrinsic'], dtype=float).reshape(4, 4)
        rotation = transform[:3, :3]
        if (not np.isfinite(transform).all() or not np.allclose(transform[3], [0, 0, 0, 1])
                or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
            raise ValueError(f'{namespace}: extrinsic must be rigid T_rgb_lidar; scale is not supported')
        source = raw_profile_file(profile, calibration_root, serial, 'extrinsic.yml')
        original_rotation = original_translation = None
        if source.is_file():
            storage = cv2.FileStorage(str(source), cv2.FILE_STORAGE_READ)
            try:
                original_rotation = storage.getNode('R').mat()
                original_translation = storage.getNode('t').mat()
            finally:
                storage.release()
        unchanged = (original_rotation is not None and original_translation is not None
                     and original_rotation.shape == (3, 3) and original_translation.size == 3
                     and np.allclose(rotation, original_rotation, rtol=0, atol=1e-12)
                     and np.allclose(transform[:3, 3], original_translation.reshape(3), rtol=0, atol=1e-12))
        if not unchanged:
            write_file('extrinsic.yml', {'R': rotation, 't': transform[:3, 3].reshape(3, 1)})
    return files


def profile_file(profile, calibration_root, serial, name, raw=False):
    """Use operator edits for runtime geometry; raw discovery always bypasses them."""
    if not raw and name in ('intrinsic.yml', 'camera_info.yaml', 'extrinsic.yml'):
        data, path = inventory()
        matches = [namespace for namespace, camera in data.get('cameras', {}).items()
                   if serial and camera.get('serial') == serial]
        if len(matches) == 1:
            files = runtime_override_files(matches[0], serial, profile, calibration_root, path)
            if name in files:
                return files[name]
    return raw_profile_file(profile, calibration_root, serial, name)


def calibration_marker_ids():
    """Calibration boards use IDs 0..9, independently of saved map anchor poses."""
    return list(range(10))


def anchor_ids(config_path=None):
    data, path = inventory(config_path)
    anchor = data.get('anchor', {})
    if not anchor and is_native(data):
        global_file = path.parent.parent / 'calib_output/global_frames.yml'
        if global_file.is_file():
            global_data = yaml.safe_load(global_file.read_text()) or {}
            anchor = global_data.get('global', {}).get('anchor', {})
    if anchor and 'id' not in anchor:
        ids = []
        for key in anchor:
            match = re.fullmatch(r'id_(\d+)', key)
            if not match:
                raise ValueError('anchor keys must use id_<marker ID>')
            ids.append(int(match.group(1)))
    else:
        ids = anchor.get('id')
    if ids is None:
        return None
    if isinstance(ids, int) and not isinstance(ids, bool):
        ids = [ids]
    if (not isinstance(ids, list) or not ids
            or any(isinstance(value, bool) or not isinstance(value, int) or value < 0 for value in ids)
            or len(set(ids)) != len(ids)):
        raise ValueError('anchor.id must contain distinct nonnegative integer marker IDs')
    return ids


def choose_anchor(visible, allowed, legacy_preferred):
    """Retain each existing preferred origin when eligible; use configured order otherwise."""
    visible = set(visible)
    eligible = visible if allowed is None else visible.intersection(allowed)
    if not eligible:
        return None
    if legacy_preferred in eligible:
        return legacy_preferred
    return min(eligible) if allowed is None else next(value for value in allowed if value in eligible)
