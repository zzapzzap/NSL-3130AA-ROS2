#!/usr/bin/env python3
"""
Publish the camera↔LiDAR extrinsic as a static TF so it is visible in RViz / tf2.

The extrinsic YAML stores  x_cam = R · x_lidar + t  (LiDAR frame → Camera frame).
This node broadcasts the camera optical frame as a child of the *live* LiDAR frame:

    {lidar_frame}  →  {camera_id}_camera_frame

The LiDAR frame name is read from the point-cloud header so it always matches the
driver (e.g. `N00A5060D_lidar_frame`). The camera's pose in the LiDAR frame is the
inverse of the extrinsic:

    R_lc = Rᵀ          (camera axes expressed in the LiDAR frame)
    t_lc = −Rᵀ · t     (camera origin expressed in the LiDAR frame)

If the extrinsic file does not exist yet (calibration not done), the node simply
waits and warns — it never crashes the launch.

Usage (via camera.launch.py, automatic) or standalone:
    python3 extrinsic_tf_node.py --calib-dir /path/to/calib_output
    python3 extrinsic_tf_node.py --calib-dir ... --camera-id N00A5060D
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


# Latched (transient-local) so a consumer that starts later — e.g. multiview_train
# on another host — still receives the camera serial without the camera node restarting.
_LATCHED_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _sensor_qos(depth=1):
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _rotmat_to_quat(R):
    """Rotation matrix → quaternion (x, y, z, w), numerically robust."""
    m = np.asarray(R, dtype=np.float64).reshape(3, 3)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / np.linalg.norm(q)


class ExtrinsicTfNode(Node):

    def __init__(self, calib_dir: str, camera_id: str, lidar_topic: str, frame_prefix: str = ''):
        super().__init__('extrinsic_tf_node')
        self._calib_dir = Path(os.path.expanduser(calib_dir))
        self._camera_id = camera_id
        self._frame_prefix = frame_prefix  # overrides TF child-frame name; calib lookup still uses camera_id
        self._bc = StaticTransformBroadcaster(self)
        self._done = False
        self._warned = False
        # Share the camera serial on /{prefix}/camera_id so any host can map
        # octet→serial straight from the camera bringup — no singleview node required.
        self._id_topic = f'/{frame_prefix}/camera_id' if frame_prefix else 'camera_id'
        self._id_pub = self.create_publisher(String, self._id_topic, _LATCHED_QOS)
        self._id_published = False
        if camera_id:
            self._publish_camera_id(camera_id)
        self._sub = self.create_subscription(PointCloud2, lidar_topic, self._cb, _sensor_qos(1))
        self.get_logger().info(
            f'Waiting for {lidar_topic} to read the live LiDAR frame '
            f'(calib_dir={self._calib_dir}) ...')

    def _publish_camera_id(self, serial: str):
        if self._id_published or not serial:
            return
        self._id_pub.publish(String(data=serial))
        self._id_published = True
        self.get_logger().info(f'[id] camera serial {serial} → {self._id_topic} (latched)')

    def _cb(self, msg: PointCloud2):
        if self._done:
            return
        lidar_frame = msg.header.frame_id or 'lidar_frame'

        camera_id = self._camera_id
        if not camera_id:
            camera_id = (lidar_frame[:-len('_lidar_frame')]
                         if lidar_frame.endswith('_lidar_frame') else lidar_frame)
        # camera_id may only be known here (derived from the live LiDAR frame).
        self._publish_camera_id(camera_id)

        yml = self._calib_dir / camera_id / 'extrinsic.yml'
        if not yml.exists():
            if not self._warned:
                self.get_logger().warn(
                    f'{yml} not found — extrinsic TF not published.\n'
                    f'  Run extrinsic_calib.launch.py, then restart camera.launch.py.')
                self._warned = True
            return

        fs = cv2.FileStorage(str(yml), cv2.FILE_STORAGE_READ)
        R = fs.getNode('R').mat()
        t = fs.getNode('t').mat()
        fs.release()
        if R is None or t is None or R.size != 9 or t.size != 3:
            self.get_logger().error(f'R/t missing or malformed in {yml}')
            self._done = True
            return

        R = np.asarray(R, np.float64).reshape(3, 3)
        t = np.asarray(t, np.float64).reshape(3)
        R_lc = R.T                       # camera axes in LiDAR frame
        t_lc = -R.T @ t                  # camera origin in LiDAR frame
        q = _rotmat_to_quat(R_lc)
        cam_frame = f'{self._frame_prefix}_camera_frame' if self._frame_prefix else f'{camera_id}_camera_frame'

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = lidar_frame
        ts.child_frame_id = cam_frame
        ts.transform.translation.x = float(t_lc[0])
        ts.transform.translation.y = float(t_lc[1])
        ts.transform.translation.z = float(t_lc[2])
        ts.transform.rotation.x = float(q[0])
        ts.transform.rotation.y = float(q[1])
        ts.transform.rotation.z = float(q[2])
        ts.transform.rotation.w = float(q[3])
        self._bc.sendTransform(ts)

        self.get_logger().info(
            f'[TF] {lidar_frame} → {cam_frame}  '
            f'cam@lidar=({t_lc[0]:.3f}, {t_lc[1]:.3f}, {t_lc[2]:.3f}) m  from {yml.name}')
        self._done = True   # static transform latches; keep node alive to serve it


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib-dir', required=True,
                    help='Directory holding {camera_id}_extrinsic.yml')
    ap.add_argument('--camera-id', default='',
                    help='Camera serial for calibration file lookup; if empty, derived from point-cloud frame_id')
    ap.add_argument('--frame-prefix', default='',
                    help='Prefix for TF frame names (e.g. cam_59 → cam_59_camera_frame). '
                         'Defaults to camera-id when not set.')
    ap.add_argument('--lidar-topic', default='/camera/point_cloud')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args or None)
    node = ExtrinsicTfNode(args.calib_dir, args.camera_id, args.lidar_topic, args.frame_prefix)
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
