#!/usr/bin/env python3
"""
Extrinsic calibration launcher — wraps extrinsic_calib.sh.
camera_id (USB serial) is auto-detected via detect_camera_id.py for the intrinsic.yml
path / output. Subscribed topics are namespaced by this machine's IP last octet
(cam_{octet}), matching camera.launch.py — so `namespace:=auto` agrees with the
running driver: `/cam_{octet}/camera/rgb/image_raw`, `/cam_{octet}/camera/point_cloud`, ...

Usage:
  ros2 launch roboscan_nsl3130 extrinsic_calib.launch.py
  ros2 launch roboscan_nsl3130 extrinsic_calib.launch.py camera_id:=N00A5060D
  ros2 launch roboscan_nsl3130 extrinsic_calib.launch.py namespace:=''        # /camera/... (no namespace)
  ros2 launch roboscan_nsl3130 extrinsic_calib.launch.py lidar_topic:=/point_cloud
  ros2 launch roboscan_nsl3130 extrinsic_calib.launch.py points_per_frame:=5

Requires: camera.launch.py running + intrinsic_calib done first.
"""

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _detect_ip_octet():
    """Last octet of this machine's 192.168.0.x address ('' if none).
    Mirrors camera.launch.py so the calib namespace matches the running driver."""
    try:
        out = subprocess.check_output(
            ['ip', '-4', 'addr'], text=True, stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            m = re.search(r'inet 192\.168\.0\.(\d+)/', line)
            if m:
                return m.group(1)
    except Exception:
        pass
    return ''


def _ns_prefix(context):
    """Topic namespace, matching camera.launch.py:
      'auto' → cam_{ip_octet} (this machine's 192.168.0.x), or '' if none found;
      '' → no namespace; else used verbatim."""
    raw = LaunchConfiguration('namespace').perform(context).strip().strip('/')
    if raw.lower() != 'auto':
        return raw
    octet = _detect_ip_octet()
    return f'cam_{octet}' if octet else ''


def _topic(value, ns, rel):
    """'auto' → /<ns>/<rel> (or /<rel> when ns empty); else use value verbatim
    (so an explicit topic, or '' to disable, is preserved)."""
    if value != 'auto':
        return value
    return f'/{ns}/{rel}' if ns else f'/{rel}'


def _launch_setup(context):
    camera_id   = LaunchConfiguration('camera_id').perform(context)
    points_per_frame = LaunchConfiguration('points_per_frame').perform(context)

    pkg_share     = get_package_share_directory('roboscan_nsl3130')
    ws_root       = os.path.normpath(os.path.join(pkg_share, '..', '..', '..', '..'))
    detect_script = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2',
                                 'NSL3130_driver', 'src', 'roboscan_nsl3130',
                                 'scripts', 'detect_camera_id.py')
    sh_path       = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'extrinsic_calib.sh')

    if not camera_id:
        try:
            camera_id = subprocess.check_output(
                ['python3', detect_script], text=True, stderr=subprocess.DEVNULL).strip()
            print(f'[extrinsic_calib] Auto-detected camera_id: {camera_id}')
        except subprocess.CalledProcessError:
            print('[extrinsic_calib] WARNING: Camera not detected.'
                  ' Pass camera_id:=<serial> to override.'
                  ' extrinsic_calib.sh will fall back to intrinsic.yml.')

    ns = _ns_prefix(context)
    image_topic     = _topic(LaunchConfiguration('image_topic').perform(context),     ns, 'camera/rgb/image_raw')
    lidar_topic     = _topic(LaunchConfiguration('lidar_topic').perform(context),     ns, 'camera/point_cloud')
    amplitude_topic = _topic(LaunchConfiguration('amplitude_topic').perform(context), ns, 'camera/ampl')
    print(f'[extrinsic_calib] image={image_topic} lidar={lidar_topic} amplitude={amplitude_topic}')

    return [ExecuteProcess(
        cmd=['bash', sh_path, camera_id, image_topic, lidar_topic,
             amplitude_topic, points_per_frame],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_id',    default_value='',
            description='Camera serial; auto-detected via detect_camera_id.py if empty'),
        DeclareLaunchArgument('namespace',    default_value='auto',
            description="Topic namespace: 'auto'=cam_{ip_octet} matching camera.launch.py (DEFAULT), ''=none (/camera/...), or explicit"),
        DeclareLaunchArgument('image_topic',  default_value='auto',
            description="'auto' → /<namespace>/camera/rgb/image_raw; or explicit topic"),
        DeclareLaunchArgument('lidar_topic',  default_value='auto',
            description="'auto' → /<namespace>/camera/point_cloud; or explicit topic"),
        DeclareLaunchArgument('amplitude_topic', default_value='auto',
            description="'auto' → /<namespace>/camera/ampl; '' disables assisted picker; or explicit topic"),
        DeclareLaunchArgument('points_per_frame', default_value='5',
            description='Number of RGB/LiDAR marker correspondences stored per frame'),
        OpaqueFunction(function=_launch_setup),
    ])
