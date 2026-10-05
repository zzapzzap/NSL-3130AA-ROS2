#!/usr/bin/env python3
"""Resolve compact camera profiles and eligible anchor IDs without touching hardware."""
import os
from pathlib import Path
import yaml

SDK_FILES = ('params.yaml', 'camera_info.yaml', 'intrinsic.yml', 'extrinsic.yml', 'multiview.yml')


def inventory(config_path=None):
    if config_path is None:
        config_path = os.environ.get('SIRIUS_CONFIG')
        if not config_path and os.environ.get('SIRIUS_ROBOT_DT_ROOT'):
            config_path = Path(os.environ['SIRIUS_ROBOT_DT_ROOT']) / 'config/config.yaml'
        if not config_path:
            config_path = next((parent / 'config/config.yaml' for parent in Path(__file__).resolve().parents
                                if (parent / 'config/config.yaml').is_file() and (parent / 'setup').is_dir()), None)
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
    return (value if value.get('schema_version') == 2 else {}), path


def camera_runtime(namespace, detected_serial, calibration_root, config_path=None):
    """Use the assigned profile, falling back for observation-only initial folders."""
    data, path = inventory(config_path)
    camera = data.get('cameras', {}).get(namespace.strip('/'), {})
    assigned = camera.get('serial') or ''
    detected = detected_serial or ''
    if assigned and detected and assigned != detected:
        raise ValueError(f'{namespace}: configured serial {assigned} differs from detected camera {detected}')
    serial = detected or assigned
    profile = None
    if camera.get('parameters') and path:
        configured = Path(camera['parameters']).expanduser()
        configured = (configured if configured.is_absolute() else path.parent / configured).resolve()
        if any((configured / name).is_file() for name in SDK_FILES):
            profile = configured
    if profile is None and serial:
        profile = Path(calibration_root).expanduser().resolve() / serial
    return serial, str(profile) if profile else ''


def profile_file(profile, calibration_root, serial, name):
    """Partial downloaded profiles retain SDK files from the verified serial folder."""
    preferred = Path(profile).expanduser() / name if profile else None
    if preferred is not None and preferred.is_file():
        return preferred
    return Path(calibration_root).expanduser() / serial / name


def anchor_ids(config_path=None):
    data, _ = inventory(config_path)
    ids = data.get('anchor', {}).get('id')
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
