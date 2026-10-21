#!/usr/bin/env python3
"""Report actual NSL USB or Ethernet SDK identity and serve its existing raw calibration files."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ros_humanpose.srv import GetWeight

from detect_camera_id import detect_camera_serials, validate_ethernet_identity
from sirius_camera_config import camera_runtime, inventory, camera_cache, profile_file, SDK_FILES

PROFILE_FILES = SDK_FILES + ('observed.yaml',)


def describe_device(namespace, calibration_root, config_path=None, usb_sys=Path('/sys/bus/usb/devices'), ethernet_identity=None):
    serials = detect_camera_serials(usb_sys)
    data, path = inventory(config_path)
    cache = camera_cache(data, path, namespace)
    cached = cache.get('serial') or data.get('cameras', {}).get(namespace, {}).get('serial')
    report = {'camera': namespace, 'serial': None, 'source': 'unavailable', 'verified': False,
              'profile_dir': None, 'files': {}, 'cached_serial': cached,
              'profile_get_service': f'/{namespace}/camera/profile/get'}
    if ethernet_identity:
        try:
            serial = validate_ethernet_identity(ethernet_identity, cache.get('sensor_host'))
        except ValueError as error:
            report['reason'] = str(error)
            return report
        source = 'ethernet_sdk_ids'
        report.update(chip_id=ethernet_identity['chip_id'], wafer_id=ethernet_identity['wafer_id'],
                      sensor_host=ethernet_identity['sensor_host'])
    elif len(serials) == 1:
        serial, source = serials[0], 'usb_sysfs'
    else:
        report['reason'] = ('Multiple NSL USB serials are present; physical binding is ambiguous'
                            if serials else 'No live USB or Ethernet SDK identity observation is available')
        return report
    serial, profile = camera_runtime(namespace, serial, calibration_root, config_path)
    report.update(serial=serial, source=source, verified=True, profile_dir=profile)
    for name in PROFILE_FILES:
        file = profile_file(profile, calibration_root, serial, name, raw=True)
        if file.is_file():
            payload = file.read_bytes()
            report['files'][name] = {'sha256': hashlib.sha256(payload).hexdigest(),
                                     'size': len(payload), 'file': str(file.resolve())}
    return report


class CameraDeviceInfoNode(Node):
    def __init__(self, namespace, calibration_root):
        if not re.fullmatch(r'cam_[0-9]+', namespace):
            raise ValueError('namespace must use cam_<Edge octet>')
        super().__init__(f'{namespace}_camera_device_info')
        self.namespace = namespace
        self.calibration_root = calibration_root
        # This snapshot is passed only to this driver launch, never loaded from an old cache.
        self.ethernet_identity = json.loads(os.environ.get('NSL_CAMERA_ETHERNET_IDENTITY', '{}'))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.identity = self.create_publisher(String, f'/{namespace}/camera_id', qos)
        self.device_service = self.create_service(Trigger, f'/{namespace}/camera/device_info', self._on_info)
        self.profile_service = self.create_service(GetWeight, f'/{namespace}/camera/profile/get', self._on_get)
        self.last_serial = None
        self.create_timer(2.0, self._publish_identity)
        self._publish_identity()

    def _report(self):
        return describe_device(self.namespace, self.calibration_root, ethernet_identity=self.ethernet_identity)

    def _publish_identity(self):
        report = self._report()
        if report['verified'] and report['serial'] != self.last_serial:
            self.identity.publish(String(data=report['serial']))
            self.last_serial = report['serial']
        elif not report['verified']:
            self.last_serial = None

    def _on_info(self, request, response):
        report = self._report()
        response.success = report['verified']
        response.message = json.dumps(report, ensure_ascii=False)
        return response

    def _on_get(self, request, response):
        response.success = False
        report = self._report()
        if not report['verified']:
            response.message = report.get('reason', 'Physical camera identity is unavailable')
            return response
        entry = report['files'].get(request.name)
        if request.name not in PROFILE_FILES or entry is None:
            response.message = 'Requested calibration file is not available for this physical camera'
            return response
        payload = Path(entry['file']).read_bytes()
        if request.offset > len(payload):
            response.message = 'Offset exceeds the calibration file size'
            return response
        count = min(request.max_bytes or 65536, 1048576)
        response.data = payload[request.offset:request.offset + count]
        response.total_size = len(payload)
        response.sha256 = hashlib.sha256(payload).hexdigest()
        response.success = True
        response.message = ''
        return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--calib-dir', required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = CameraDeviceInfoNode(args.namespace, args.calib_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
