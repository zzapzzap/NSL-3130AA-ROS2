#!/usr/bin/env python3
"""Multi-sensor RViz viewer for all NSL-3130 units on the same ROS_DOMAIN_ID,
tied together under one shared STag reference frame.

Each sensor machine launches with a namespace so topics don't collide:

  # On 192.168.0.56:
  ros2 launch roboscan_nsl3130 camera.launch.py \
      use_rviz:=false use_rqt:=false \
      point_cloud_topic:=/cam_56/camera/point_cloud \
      point_cloud_rgb_topic:=/cam_56/camera/point_cloud_rgb \
      rgb_topic:=/cam_56/camera/rgb/image_raw \
      depth_topic:=/cam_56/camera/depth/image_raw

  # On 192.168.0.57 → replace cam_56 with cam_57, etc.

After running multiview_calib.launch.py on each camera (which writes
calib_output/{serial}/multiview.yml), this viewer publishes the shared
`stag_marker` reference frame and anchors every calibrated camera under it, so
all point clouds line up in one frame. A camera whose multiview.yml is missing
or has an unreadable R|t is REJECTED (skipped) — it just won't appear.

Then on the viewer machine (any machine with ROS_DOMAIN_ID=42):
  ros2 launch roboscan_nsl3130 multi_viewer.launch.py
  ros2 launch roboscan_nsl3130 multi_viewer.launch.py use_multiview_tf:=false   # raw, no shared frame
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REFERENCE_FRAME = 'stag_marker'


def _is_true(context, arg):
    return LaunchConfiguration(arg).perform(context).strip().lower() in ('true', '1', 'yes')


def _rviz_config_with_reference(rviz_config, fixed_frame):
    """Rewrite the bundled rviz config so its Fixed Frame is the STag reference
    frame; everything then renders in the shared frame. Returns a /tmp path."""
    try:
        with open(rviz_config) as f:
            content = f.read()
        content = content.replace('Fixed Frame: world', f'Fixed Frame: {fixed_frame}')
        out = os.path.join('/tmp', f'multi_viewer_{fixed_frame}.rviz')
        with open(out, 'w') as f:
            f.write(content)
        return out
    except OSError:
        return rviz_config


def _setup(context):
    pkg_share   = get_package_share_directory('roboscan_nsl3130')
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    ws_root      = os.path.normpath(os.path.join(pkg_share, '..', '..', '..', '..'))
    calib_dir    = os.environ.get(
        'NSL_CALIB_DIR',
        os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'calib_output'))
    repo_scripts = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'NSL3130_driver',
                                'src', 'roboscan_nsl3130', 'scripts')
    tf_script    = os.path.join(repo_scripts, 'multiview_tf_node.py')

    actions = []

    use_tf = _is_true(context, 'use_multiview_tf')
    rviz_delay = 0.0
    if use_tf:
        actions.append(ExecuteProcess(
            cmd=['python3', tf_script, '--calib-dir', calib_dir],
            output='screen'))
        rviz_config = _rviz_config_with_reference(rviz_config, REFERENCE_FRAME)
        # Start RViz a few seconds after multiview_tf_node so the shared static
        # transforms are already on /tf_static before RViz subscribes to clouds.
        # Otherwise RViz holds early cloud messages waiting for the transform and
        # logs "Message Filter dropping message ... queue is full" at startup.
        rviz_delay = 3.0

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='multi_sensor_viewer',
        arguments=['-d', rviz_config],
        output='screen',
    )
    actions.append(TimerAction(period=rviz_delay, actions=[rviz_node])
                   if rviz_delay else rviz_node)
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('roboscan_nsl3130')
    rviz_config = os.path.join(pkg_share, 'rviz', 'multi_sensor.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config', default_value=rviz_config,
            description='Path to rviz2 config file'),
        DeclareLaunchArgument(
            'use_multiview_tf', default_value='true',
            description=f'Publish the shared "{REFERENCE_FRAME}" frame from each '
                        f'calib_output/{{serial}}/multiview.yml and set it as the '
                        f'RViz Fixed Frame; cameras with no/unreadable R|t are skipped'),
        OpaqueFunction(function=_setup),
    ])
