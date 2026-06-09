#!/usr/bin/env python3
"""
Unified multiview entry point (no separate multiview_calib.launch.py).

  ros2 launch roboscan_nsl3130 multiview.launch.py
      → VIEWER: anchor every camera + tag under the shared `stag_marker` frame.

  ros2 launch roboscan_nsl3130 multiview.launch.py calibration:=True
      → viewer + the STag multiview CALIBRATION node. Multi-tag: id 7 = 0.32 m
        REFERENCE/origin, all other ids = 0.19 m auxiliaries; each tag gets a
        per-tag LiDAR-RANSAC depth refine. Saves calib_output/{serial}/multiview.yml.
      e.g.  calibration:=True display:=false num_frames:=40

The RGB topic follows camera.launch.py's `/cam_{ip_octet}/camera/...` convention and
the output folder is keyed by the camera's USB serial (auto-detected).
"""

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _detect_ip_octet():
    """Last octet of this machine's 192.168.0.x address ('' if not found)."""
    try:
        out = subprocess.check_output(['ip', '-4', 'addr'], text=True, stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            m = re.search(r'inet 192\.168\.0\.(\d+)/', line)
            if m:
                return m.group(1)
    except Exception:
        pass
    return ''


def _calib_setup(context):
    """Run the multiview calibration node only when calibration:=true."""
    if LaunchConfiguration('calibration').perform(context).strip().lower() not in ('true', '1', 'yes'):
        return []

    pkg_share   = get_package_share_directory('roboscan_nsl3130')
    ws_root     = os.path.normpath(os.path.join(pkg_share, '..', '..', '..', '..'))
    scripts_dir = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'NSL3130_driver',
                               'src', 'roboscan_nsl3130', 'scripts')
    detect      = os.path.join(scripts_dir, 'detect_camera_id.py')
    node_script = os.path.join(scripts_dir, 'multiview_calib_node.py')
    calib_dir   = os.environ.get(
        'NSL_CALIB_DIR', os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'calib_output'))

    camera_id = LaunchConfiguration('camera_id').perform(context)
    if not camera_id:
        try:
            camera_id = subprocess.check_output(
                ['python3', detect], text=True, stderr=subprocess.DEVNULL).strip()
            print(f'[multiview] auto-detected camera_id: {camera_id}')
        except subprocess.CalledProcessError:
            print('[multiview] ERROR: camera not detected; pass camera_id:=<serial>')
            camera_id = ''

    image_topic = LaunchConfiguration('image_topic').perform(context)
    if image_topic == 'auto':
        octet = _detect_ip_octet()
        image_topic = f'/cam_{octet}/camera/rgb/image_raw' if octet else '/camera/rgb/image_raw'
    print(f'[multiview] calibration: camera_id={camera_id} image_topic={image_topic}')

    return [ExecuteProcess(
        cmd=['python3', node_script,
             '--camera-id',    camera_id,
             '--calib-dir',    calib_dir,
             '--image-topic',  image_topic,
             '--library-hd',   LaunchConfiguration('library_hd').perform(context),
             '--num-frames',   LaunchConfiguration('num_frames').perform(context),
             '--reproj-thresh', LaunchConfiguration('reproj_thresh').perform(context),
             '--display',      LaunchConfiguration('display').perform(context),
             '--depth-refine', LaunchConfiguration('depth_refine').perform(context),
             '--depth-band',   LaunchConfiguration('depth_band').perform(context)],
        output='screen')]


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('roboscan_nsl3130'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('calibration', default_value='false',
            description='true → also run the STag multiview calibration node alongside the viewer'),
        # ── calibration-node options (only used when calibration:=true) ──
        DeclareLaunchArgument('camera_id',   default_value='',
            description='Camera serial; auto-detected via USB if empty'),
        DeclareLaunchArgument('image_topic', default_value='auto',
            description="'auto' → /cam_{ip_octet}/camera/rgb/image_raw; or an explicit topic"),
        DeclareLaunchArgument('library_hd',  default_value='21',
            description='STag HD library [11,13,15,17,19,21,23]'),
        DeclareLaunchArgument('num_frames',  default_value='30',
            description='Good views per tag before averaging+saving'),
        DeclareLaunchArgument('reproj_thresh', default_value='3.0',
            description='Max per-view reprojection RMSE (px) to accept a view'),
        DeclareLaunchArgument('display',     default_value='true',
            description='Show live detection window — [s]save [r]reset [q]quit; false = headless'),
        DeclareLaunchArgument('depth_refine', default_value='true',
            description='Per-tag LiDAR-plane RANSAC depth refine (needs extrinsic.yml)'),
        DeclareLaunchArgument('depth_band',  default_value='0.20',
            description='± depth band (m) around the 1st-pass marker plane for the LiDAR RANSAC'),

        # Viewer: RViz fixed frame = stag_marker, receives every edge/tag TF (always).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'multi_viewer.launch.py'))),

        # Calibration node — only when calibration:=true.
        OpaqueFunction(function=_calib_setup),
    ])
