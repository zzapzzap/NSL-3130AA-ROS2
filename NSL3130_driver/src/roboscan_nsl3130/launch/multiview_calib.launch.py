#!/usr/bin/env python3
"""
Multiview (STag) calibration launcher.

Detects a STag marker in one camera's RGB stream, extracts its 6-DoF pose, and
saves the marker<->rgb relationship to calib_output/{serial}/multiview.yml so
several cameras can share a single reference frame (the marker) — see
multi_viewer.launch.py.

The RGB topic follows camera.launch.py's convention `/cam_{ip_octet}/camera/...`
(e.g. this machine on 192.168.0.59 → /cam_59/camera/rgb/image_raw). The output
folder is keyed by the camera's USB serial, exactly like intrinsic/extrinsic.

Usage (run on the camera's own machine, after intrinsic_calib):
  ros2 launch roboscan_nsl3130 multiview_calib.launch.py
  ros2 launch roboscan_nsl3130 multiview_calib.launch.py marker_size:=0.32 library_hd:=21
  ros2 launch roboscan_nsl3130 multiview_calib.launch.py marker_id:=7
  ros2 launch roboscan_nsl3130 multiview_calib.launch.py display:=false      # headless/auto
  ros2 launch roboscan_nsl3130 multiview_calib.launch.py \
      image_topic:=/cam_59/camera/rgb/image_raw camera_id:=N00A5060D        # explicit

Requires: camera.launch.py running + intrinsic_calib done first
          (reads calib_output/{serial}/intrinsic.yml).
"""

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _detect_ip_octet():
    """Last octet of this machine's 192.168.0.x address ('' if not found)."""
    try:
        out = subprocess.check_output(['ip', '-4', 'addr'], text=True,
                                      stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            m = re.search(r'inet 192\.168\.0\.(\d+)/', line)
            if m:
                return m.group(1)
    except Exception:
        pass
    return ''


def _resolve_image_topic(context):
    """'auto' → /cam_{ip_octet}/camera/rgb/image_raw (camera.launch.py convention);
    falls back to /camera/rgb/image_raw when no 192.168.0.x address is found."""
    value = LaunchConfiguration('image_topic').perform(context)
    if value != 'auto':
        return value
    octet = _detect_ip_octet()
    if octet:
        return f'/cam_{octet}/camera/rgb/image_raw'
    print('[multiview_calib] no 192.168.0.x address → /camera/rgb/image_raw')
    return '/camera/rgb/image_raw'


def _launch_setup(context):
    camera_id = LaunchConfiguration('camera_id').perform(context)

    pkg_share   = get_package_share_directory('roboscan_nsl3130')
    ws_root     = os.path.normpath(os.path.join(pkg_share, '..', '..', '..', '..'))
    scripts_dir = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2',
                               'NSL3130_driver', 'src', 'roboscan_nsl3130', 'scripts')
    detect_script = os.path.join(scripts_dir, 'detect_camera_id.py')
    node_script   = os.path.join(scripts_dir, 'multiview_calib_node.py')
    calib_dir     = os.environ.get(
        'NSL_CALIB_DIR',
        os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'calib_output'))

    if not camera_id:
        try:
            camera_id = subprocess.check_output(
                ['python3', detect_script], text=True, stderr=subprocess.DEVNULL).strip()
            print(f'[multiview_calib] Auto-detected camera_id: {camera_id}')
        except subprocess.CalledProcessError:
            print('[multiview_calib] ERROR: camera not detected. Pass camera_id:=<serial>.')
            camera_id = ''

    image_topic = _resolve_image_topic(context)
    print(f'[multiview_calib] camera_id={camera_id} image_topic={image_topic}')

    return [ExecuteProcess(
        cmd=['python3', node_script,
             '--camera-id',    camera_id,
             '--calib-dir',    calib_dir,
             '--image-topic',  image_topic,
             '--library-hd',   LaunchConfiguration('library_hd').perform(context),
             '--marker-size',  LaunchConfiguration('marker_size').perform(context),
             '--marker-id',    LaunchConfiguration('marker_id').perform(context),
             '--num-frames',   LaunchConfiguration('num_frames').perform(context),
             '--reproj-thresh', LaunchConfiguration('reproj_thresh').perform(context),
             '--display',      LaunchConfiguration('display').perform(context),
             '--depth-refine', LaunchConfiguration('depth_refine').perform(context),
             '--depth-band',   LaunchConfiguration('depth_band').perform(context)],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_id',   default_value='',
            description='Camera serial; auto-detected via USB if empty'),
        DeclareLaunchArgument('image_topic', default_value='auto',
            description="'auto' → /cam_{ip_octet}/camera/rgb/image_raw; or an explicit topic"),
        DeclareLaunchArgument('library_hd',  default_value='21',
            description='STag HD library [11,13,15,17,19,21,23]'),
        DeclareLaunchArgument('marker_size', default_value='0.32',
            description='Marker side length in metres (outer black border)'),
        DeclareLaunchArgument('marker_id',   default_value='-1',
            description='Reference marker id; -1 = lowest visible id'),
        DeclareLaunchArgument('num_frames',  default_value='30',
            description='Good views to average before saving multiview.yml'),
        DeclareLaunchArgument('reproj_thresh', default_value='3.0',
            description='Max per-view reprojection RMSE (px) to accept a view'),
        DeclareLaunchArgument('display',     default_value='true',
            description='Show live window — [s]save [r]reset [q]quit (you decide); '
                        'false = headless auto-save at num_frames'),
        DeclareLaunchArgument('depth_refine', default_value='true',
            description='Snap marker depth onto the LiDAR plane via RANSAC (needs extrinsic.yml)'),
        DeclareLaunchArgument('depth_band',  default_value='0.20',
            description='± depth band (m) around the 1st-pass marker plane for the LiDAR RANSAC'),
        OpaqueFunction(function=_launch_setup),
    ])
