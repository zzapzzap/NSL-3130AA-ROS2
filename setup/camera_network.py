#!/usr/bin/env python3
"""Read an Edge's explicit camera-link addresses; never infer sensor IP offsets."""
import argparse
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import sys
import yaml


def inventory_path(config_path=None):
    root = Path(os.environ.get('SIRIUS_ROBOT_DT_ROOT', Path(__file__).resolve().parents[4]))
    selected = config_path or os.environ.get('SIRIUS_CONFIG')
    common = root/'config/config.yaml'
    if not selected and common.is_file() and (yaml.safe_load(common.read_text()) or {}).get('schema_version') in (2, 3):
        selected = common
    path = Path(selected or root/'controller/src/NSL-3130AA-ROS2/setup/fleet.yaml').expanduser().resolve()
    if path.is_file():
        config = yaml.safe_load(path.read_text()) or {}
        if config.get('schema_version') == 3:
            reference = config.get('camera_settings')
            if not isinstance(reference, str) or not reference.strip():
                raise ValueError('Common config camera_settings must name a camera fleet file')
            target = Path(reference).expanduser()
            target = (target if target.is_absolute() else path.parent / target).resolve()
            if target == path:
                raise ValueError('Common config camera_settings cannot refer to itself')
            camera_data = yaml.safe_load(target.read_text()) or {}
            if not isinstance(camera_data, dict) or camera_data.get('schema_version') == 3:
                raise ValueError('camera_settings must refer directly to a camera fleet mapping')
            path = target
    return path


def inventory_data(config_path=None):
    path = inventory_path(config_path)
    scripts = Path(__file__).resolve().parents[1]/'roboscan_nsl3130/scripts'
    if str(scripts) not in sys.path:
        sys.path.insert(0, str(scripts))
    from sirius_camera_config import inventory
    data, path = inventory(path)
    if not data:
        # Standalone historical inventories did not require server/schema fields.
        data = yaml.safe_load(path.read_text()) or {}
    return data, path


def camera_field(octet, field, config_path=None):
    config, config_path = inventory_data(config_path)
    name = f'cam_{octet}'
    camera = (config.get('cameras') or {}).get(name, {})
    if config.get('schema_version') == 2 and field != 'host':
        from sirius_camera_config import camera_entry
        camera = camera_entry(config, config_path, name, bootstrap=True)
    value = camera.get('ip', camera.get('host')) if field == 'host' else camera.get(field)
    if value is None:
        return ''
    value = str(value).strip()
    if value:
        ipaddress.IPv4Address(value)
    return value


def network_field(field, config_path=None):
    path = inventory_path(config_path)
    config = yaml.safe_load(path.read_text()) or {}
    network = config.get('network') or {}
    server = config.get('server') or {}
    if field == 'domain_id':
        value = server.get('domain_id', (network.get('ros2') or {}).get('domain_id', 42))
        if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 232:
            raise ValueError('ROS domain_id must be an integer in 0..232')
        return str(value)
    value = server.get('ip', ((network.get('address_plan') or {}).get('ros2') or {}).get('hosts', {}).get('server', '192.168.3.166'))
    ipaddress.IPv4Address(value)
    return str(value)


def camera_fingerprint(octet, config_path=None):
    data, _ = inventory_data(config_path)
    camera = data.get('cameras', {}).get(f'cam_{octet}', {})
    keys = ('ip', 'serial', 'parameters', 'intrinsic', 'distortion', 'extrinsic')
    server = data.get('server') or {}
    effective = {'server': {key: server.get(key) for key in ('ip', 'domain_id')},
                 'camera': {key: camera.get(key) for key in keys}}
    return hashlib.sha256(json.dumps(effective, sort_keys=True).encode()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--octet')
    parser.add_argument('--field',choices=('sensor_host','sensor_source_address','host'))
    parser.add_argument('--network-field',choices=('domain_id','host'))
    parser.add_argument('--config')
    parser.add_argument('--inventory-file', action='store_true', help='Print the active camera inventory path')
    parser.add_argument('--camera-fingerprint', help='Hash only one Edge runtime calibration/network settings')
    args=parser.parse_args()
    try:
        if args.camera_fingerprint:
            print(camera_fingerprint(args.camera_fingerprint, args.config))
        elif args.inventory_file:
            print(inventory_path(args.config))
        elif args.network_field:
            print(network_field(args.network_field,args.config))
        elif args.octet and args.field:
            print(camera_field(args.octet,args.field,args.config))
        else:
            parser.error('use --network-field or both --octet and --field')
    except (OSError,ValueError,yaml.YAMLError) as exc:
        parser.exit(2,f'camera config: {exc}\n')


if __name__ == '__main__':
    main()
