#!/usr/bin/env python3
"""Explicitly fetch one Edge's SDK profile; sourcing the environment never runs this."""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[4]
PROFILE_FILES = ('params.yaml', 'camera_info.yaml', 'intrinsic.yml', 'extrinsic.yml',
                 'multiview.yml', 'observed.yaml')


def pull_profile(config_path, camera_id, remote_dir=None, remote_project='~/SiriusRobotDT', user=''):
    from sirius_config.sync import read_profile, sync_profiles

    from camera_network import inventory_path
    config_path = inventory_path(config_path)
    config = yaml.safe_load(config_path.read_text()) or {}
    name = str(camera_id) if str(camera_id).startswith('cam_') else f'cam_{camera_id}'
    camera = (config.get('cameras') or {}).get(name)
    if not isinstance(camera, dict):
        raise ValueError(f'unknown camera {name}')
    address = camera.get('ip', camera.get('host'))
    ipaddress.IPv4Address(address)
    if user and not re.fullmatch(r'[A-Za-z0-9_.-]+', user):
        raise ValueError('SSH user contains unsupported characters')
    parameters = camera.get('parameters')
    if not isinstance(parameters, str) or not parameters.strip():
        raise ValueError(f'{name}.parameters must name a local profile directory')
    local = Path(parameters).expanduser()
    local = (local if local.is_absolute() else config_path.parent / local).resolve()
    if local == config_path.parent or local == config_path.parent.parent or local == Path('/'):
        raise ValueError('parameters must name a camera profile directory')
    if remote_dir is None:
        if Path(parameters).is_absolute():
            raise ValueError('absolute local parameters path needs an explicit remote profile directory')
        # Parameters are relative to config.yaml, including ../controller/... .
        # Normalize that local address before producing a shell-safe remote path.
        configured_project = os.environ.get('SIRIUS_ROBOT_DT_ROOT')
        if 'controller' in config_path.parts:
            project_root = Path(*config_path.parts[:config_path.parts.index('controller')])
        else:
            project_root = Path(configured_project).expanduser().resolve() if configured_project else config_path.parent.parent
        try:
            project_relative = local.relative_to(project_root)
        except ValueError as error:
            raise ValueError('parameters outside the project need an explicit remote profile directory') from error
        if 'controller' not in config_path.parts and config_path.parent.name != 'config':
            project_relative = Path('config') / Path(parameters)
        remote_dir = f'{remote_project.rstrip("/")}/{project_relative.as_posix()}'
    # SCP can use a remote shell. Restrict the path rather than interpolating shell commands.
    if not re.fullmatch(r'[/~A-Za-z0-9_.-]+', remote_dir) or '..' in Path(remote_dir).parts:
        raise ValueError('remote profile path must use simple path characters without ..')
    target = f'{user}@{address}' if user else str(address)
    with tempfile.TemporaryDirectory(prefix='sirius-calibration-') as temporary:
        downloaded = Path(temporary) / 'downloaded'
        downloaded.mkdir()
        argv = ['scp', '-o', 'ConnectTimeout=6', '-o',
                'BatchMode=no' if os.environ.get('SSHPASS') else 'BatchMode=yes',
                f'{target}:{remote_dir.rstrip("/")}/*.y*ml', str(downloaded)]
        if os.environ.get('SSHPASS'):
            argv = ['sshpass', '-e', *argv]
        subprocess.run(argv, check=True)
        files = [downloaded / name for name in PROFILE_FILES if (downloaded / name).is_file()]
        if not files:
            raise ValueError('remote profile contains no supported SDK YAML files')
        for path in files:
            if path.is_symlink():
                raise ValueError(f'refusing a symlink profile file: {path.name}')
            if not isinstance(read_profile(path), dict):
                raise ValueError(f'{path.name} must contain a YAML mapping')
        # Validate every downloaded file first. Existing files survive an invalid profile.
        local.mkdir(parents=True, exist_ok=True)
        for path in files:
            descriptor, temporary_name = tempfile.mkstemp(prefix=f'.{path.name}.', dir=local)
            try:
                with os.fdopen(descriptor, 'wb') as stream:
                    stream.write(path.read_bytes())
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary_name, local / path.name)
            finally:
                if os.path.exists(temporary_name):
                    os.unlink(temporary_name)
    # Fetching a directory does not prove a camera serial. Keep the existing binding.
    report = sync_profiles(config_path, local.parent)
    return {'camera': name, 'parameters': str(local), 'files': [path.name for path in files],
            'serial': camera.get('serial'), 'sync': report}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('camera', help='161 or cam_161')
    parser.add_argument('remote_profile', nargs='?', help='Override the remote profile directory')
    parser.add_argument('--config', default=os.environ.get('SIRIUS_CONFIG'), help='Common config or native fleet file; defaults to the project manifest')
    parser.add_argument('--remote-project', default=os.environ.get('NSL_REMOTE_PROJECT', '~/SiriusRobotDT'))
    parser.add_argument('--user', default='')
    args = parser.parse_args(argv)
    sys.path.insert(0, str(ROOT/'controller/src/robot_dt_bridge/config'))
    try:
        result = pull_profile(args.config, args.camera, args.remote_profile, args.remote_project, args.user)
        print(json.dumps(result, ensure_ascii=False))
        return 1 if result['sync']['errors'] else 0
    except (OSError, ValueError, yaml.YAMLError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'mcalpull: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
