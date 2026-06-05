#!/usr/bin/env python3
"""Multi-sensor RViz viewer for all NSL-3130 units on the same ROS_DOMAIN_ID.

Each sensor machine launches with a namespace so topics don't collide:

  # On 192.168.0.56:
  ros2 launch roboscan_nsl3130 camera.launch.py \
      use_rviz:=false use_rqt:=false \
      point_cloud_topic:=/cam_56/camera/point_cloud \
      point_cloud_rgb_topic:=/cam_56/camera/point_cloud_rgb \
      rgb_topic:=/cam_56/camera/rgb/image_raw \
      depth_topic:=/cam_56/camera/depth/image_raw

  # On 192.168.0.57 → replace cam_56 with cam_57, etc.

Then on the viewer machine (any machine with ROS_DOMAIN_ID=42):
  ros2 launch roboscan_nsl3130 multi_viewer.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('roboscan_nsl3130')
    rviz_config = os.path.join(pkg_share, 'rviz', 'multi_sensor.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config', default_value=rviz_config,
            description='Path to rviz2 config file'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='multi_sensor_viewer',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
        ),
    ])
