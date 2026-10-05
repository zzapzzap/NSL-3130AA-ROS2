#!/usr/bin/env python3
"""Render an idempotent service mode command; execution stays with the operator."""
import argparse
from pathlib import PurePosixPath
import re
import shlex


def edge_pose_command(user, enabled, unit_directory='/etc/systemd/system', *, config=None, camera_sha=None, config_sha=None):
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.-]*', user):
        raise ValueError('Invalid Edge service account')
    if enabled not in ('true', 'false'):
        raise ValueError('Pose mode must be true or false')
    unit = f'nsl-edge-agent@{user}.service'
    directory = PurePosixPath(unit_directory) / (unit + '.d')
    content = f'[Service]\nEnvironment=NSL_ENABLE_POSE={enabled}\n'
    for variable, value in (('NSL_ROOT_CAMERA_SHA', camera_sha), ('NSL_POSE_CONFIG_SHA', config_sha)):
        if value is not None:
            if not re.fullmatch(r'[a-f0-9]{64}', value):
                raise ValueError('Runtime config fingerprint must be SHA256')
            content += 'Environment='+variable+'='+value+'\n'
    if config is not None:
        if (not config.startswith('controller/src/ros_humanpose/config/')
                or '..' in PurePosixPath(config).parts or not re.fullmatch(r'[A-Za-z0-9_./-]+', config)):
            raise ValueError('Pose config must be a native module-relative path')
        content += 'Environment=NSL_POSE_CONFIG='+config+'\n'
    return '\n'.join([
        'set -e', 'unit=' + shlex.quote(unit),
        'mode_directory=' + shlex.quote(str(directory)),
        'mode_file="$mode_directory/pose.conf"', 'desired=' + shlex.quote(content),
        'if [[ -f "$mode_file" ]] && cmp -s "$mode_file" <(printf "%s" "$desired"); then',
        '    systemctl start "$unit"',
        'else',
        '    install -d -m 0755 "$mode_directory"',
        '    staged="$(mktemp "$mode_directory/.pose.XXXXXX")"',
        '    trap \'rm -f "$staged"\' EXIT',
        '    printf "%s" "$desired" > "$staged"',
        '    chmod 0644 "$staged"', '    mv "$staged" "$mode_file"',
        '    systemctl daemon-reload', '    systemctl restart "$unit"', 'fi', ''])


def edge_stop_command(user, wait_seconds=25):
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.-]*', user):
        raise ValueError('Invalid Edge service account')
    if isinstance(wait_seconds, bool) or not isinstance(wait_seconds, int) or not 1 <= wait_seconds <= 60:
        raise ValueError('Stop wait must be an integer from 1 to 60 seconds')
    unit = f'nsl-edge-agent@{user}.service'
    return '\n'.join(['set -e', 'unit=' + shlex.quote(unit),
        f'timeout --signal=TERM --kill-after=2s {wait_seconds}s systemctl stop "$unit"',
        'active="$(systemctl show --value -p ActiveState "$unit")"',
        'case "$active" in',
        '    inactive|failed) printf "Stopped %s (%s)\\n" "$unit" "$active" ;;',
        '    *) printf "Stop not acknowledged: %s is %s\\n" "$unit" "$active" >&2; exit 1 ;;',
        'esac', ''])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('user')
    parser.add_argument('enabled', nargs='?', choices=('true', 'false'), default='true')
    parser.add_argument('--stop', action='store_true')
    parser.add_argument('--wait-seconds', type=int, default=25)
    parser.add_argument('--config')
    parser.add_argument('--camera-sha')
    parser.add_argument('--config-sha')
    args = parser.parse_args()
    print(edge_stop_command(args.user, args.wait_seconds) if args.stop else edge_pose_command(
        args.user, args.enabled, config=args.config, camera_sha=args.camera_sha, config_sha=args.config_sha), end='')


if __name__ == '__main__':
    main()
