#!/usr/bin/env python3
"""
Intrinsic calibration launcher — runs scripts/intrinsic_calibration_node.py.
camera_id (USB serial) is auto-detected for the intrinsic.yml path / output. The
subscribed image topic is namespaced by this machine's IP last octet (cam_{octet}),
matching camera.launch.py → `/cam_{octet}/camera/rgb/image_raw`.
Override with namespace:='' or an explicit image_topic.

Calibration mode (default):
  ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py
  ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py board_size:=8x13 square_size:=0.04

Debug / verify mode (requires existing {camera_id}_intrinsic.yml):
  ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py debug:=true
  ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py debug:=true balance:=0.0
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
    """'auto' → /<ns>/<rel> (or /<rel> when ns empty); else use value verbatim."""
    if value != 'auto':
        return value
    return f'/{ns}/{rel}' if ns else f'/{rel}'


def _launch_setup(context):
    board_size  = LaunchConfiguration('board_size').perform(context)
    square_size = LaunchConfiguration('square_size').perform(context)
    camera_id   = LaunchConfiguration('camera_id').perform(context)
    debug       = LaunchConfiguration('debug').perform(context).lower() == 'true'
    balance     = LaunchConfiguration('balance').perform(context)

    pkg_share     = get_package_share_directory('roboscan_nsl3130')
    ws_root       = os.path.normpath(os.path.join(pkg_share, '..', '..', '..', '..'))
    scripts_dir   = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2',
                                 'NSL3130_driver', 'src', 'roboscan_nsl3130', 'scripts')
    detect_script = os.path.join(scripts_dir, 'detect_camera_id.py')
    calib_dir     = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'calib_output')

    if not camera_id:
        try:
            camera_id = subprocess.check_output(
                ['python3', detect_script], text=True, stderr=subprocess.DEVNULL).strip()
            print(f'[intrinsic_calib] Auto-detected camera_id: {camera_id}')
        except subprocess.CalledProcessError:
            print('[intrinsic_calib] WARNING: Camera not detected. Pass camera_id:=<serial>.')
            camera_id = 'nsl'

    ns = _ns_prefix(context)
    image_topic = _topic(LaunchConfiguration('image_topic').perform(context),
                         ns, 'camera/rgb/image_raw')
    print(f'[intrinsic_calib] image_topic: {image_topic}')

    if debug:
        verify_script = os.path.join(scripts_dir, 'verify_intrinsic.py')
        return [ExecuteProcess(
            cmd=['python3', verify_script,
                 '--camera-id',   camera_id,
                 '--calib-dir',   calib_dir,
                 '--image-topic', image_topic,
                 '--balance',     balance],
            output='screen',
        )]

    node_script = os.path.join(scripts_dir, 'intrinsic_calibration_node.py')
    return [ExecuteProcess(
        cmd=['python3', node_script,
             '--camera-id', camera_id,
             '--size', board_size,
             '--square', square_size,
             '--image-topic', image_topic,
             '--output-dir', calib_dir],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('board_size',   default_value='8x13',
            description='Checkerboard inner corners WxH'),
        DeclareLaunchArgument('square_size',  default_value='0.04',
            description='Square side length in metres'),
        DeclareLaunchArgument('namespace',    default_value='auto',
            description="Topic namespace: 'auto'=cam_{ip_octet} matching camera.launch.py (DEFAULT), ''=none (/camera/...), or explicit"),
        DeclareLaunchArgument('image_topic',  default_value='auto',
            description="'auto' → /<namespace>/camera/rgb/image_raw; or an explicit topic"),
        DeclareLaunchArgument('camera_id',    default_value='',
            description='Camera serial; auto-detected via USB if empty'),
        DeclareLaunchArgument('debug',        default_value='false',
            description='Show live rectified image using existing intrinsic.yml'),
        DeclareLaunchArgument('balance',      default_value='0.5',
            description='Undistort balance: 0.0=crop black, 1.0=keep all pixels'),
        OpaqueFunction(function=_launch_setup),
    ])
