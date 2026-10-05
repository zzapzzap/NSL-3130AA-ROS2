#!/usr/bin/env python3
"""
Live rectification preview using {camera_id}_intrinsic.yml.

Subscribes to the RGB image topic, undistorts each frame with the
saved fisheye intrinsics, and shows Original | Rectified side-by-side.
A trackbar lets you tune the balance (0=crop black, 100=keep all pixels)
in real time without restarting.

Usage (standalone):
  python3 verify_intrinsic.py --camera-id N00A5060D --calib-dir /path/to/calib_output
  python3 verify_intrinsic.py --camera-id N00A5060D --calib-dir ... --balance 0.0

Via launch:
  ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py debug:=true
  ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py debug:=true balance:=0.0

balance: 0.0 = crop all black borders, 1.0 = keep all pixels (with black corners)
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

WIN = 'Original | Rectified'


class VerifyIntrinsicNode(Node):

    def __init__(self, calib_path: str, image_topic: str, balance: float):
        super().__init__('verify_intrinsic')

        fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            self.get_logger().error(f'Cannot open {calib_path}')
            raise RuntimeError(calib_path)

        self._K = fs.getNode('camera_matrix').mat()
        self._D = fs.getNode('distortion_coefficients').mat().reshape(-1, 1)
        self._W = int(fs.getNode('image_width').real())
        self._H = int(fs.getNode('image_height').real())
        dist_model = fs.getNode('distortion_model').string()
        fs.release()

        self._fisheye = dist_model in ('equidistant', 'fisheye')
        self._D4 = self._D[:4].reshape(4, 1) if self._fisheye else self._D

        self._map1 = None
        self._map2 = None
        self._balance = balance
        self._recompute_maps(balance)

        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, 1920, 540)

        # Trackbar 0–100 → balance 0.0–1.0
        init_pos = int(round(balance * 100))
        cv2.createTrackbar('balance x100', WIN, init_pos, 100, self._on_trackbar)

        self._bridge = CvBridge()
        self._sub = self.create_subscription(Image, image_topic, self._cb, 1)

        self.get_logger().info(
            f'[verify] {dist_model}  balance={balance}\n'
            f'  K  fx={self._K[0,0]:.1f} fy={self._K[1,1]:.1f}'
            f' cx={self._K[0,2]:.1f} cy={self._K[1,2]:.1f}\n'
            f'  D  {self._D.flatten().round(5).tolist()}\n'
            f'  subscribing to {image_topic} ...\n'
            f'  trackbar: drag to adjust undistort balance\n'
            f'  [q] quit')

    def _recompute_maps(self, balance: float):
        if self._fisheye:
            K_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                self._K, self._D4, (self._W, self._H), np.eye(3), balance=balance)
            self._map1, self._map2 = cv2.fisheye.initUndistortRectifyMap(
                self._K, self._D4, np.eye(3), K_new,
                (self._W, self._H), cv2.CV_16SC2)
        else:
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                self._K, self._D, None, self._K,
                (self._W, self._H), cv2.CV_16SC2)

    def _on_trackbar(self, pos: int):
        self._balance = pos / 100.0
        self._recompute_maps(self._balance)

    def _cb(self, msg: Image):
        img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        rect = cv2.remap(img, self._map1, self._map2, cv2.INTER_LINEAR)

        for frame, label in ((img, 'Original'), (rect, 'Rectified')):
            cv2.putText(frame, label, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)

        bal_txt = f'balance={self._balance:.2f}'
        cv2.putText(rect, bal_txt, (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)

        vis = np.hstack([img, rect])
        cv2.imshow(WIN, vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera-id',   required=True)
    ap.add_argument('--calib-dir',   required=True)
    ap.add_argument('--image-topic', default='/camera/rgb/image_raw')
    ap.add_argument('--balance',     type=float, default=0.5)
    args, ros_args = ap.parse_known_args()

    calib_path = str(Path(args.calib_dir) / args.camera_id / 'intrinsic.yml')
    if not Path(calib_path).exists():
        print(f'[verify] ERROR: {calib_path} not found.')
        print(f'         Run intrinsic_calib.launch.py first.')
        sys.exit(1)

    rclpy.init(args=ros_args or None)
    try:
        node = VerifyIntrinsicNode(calib_path, args.image_topic, args.balance)
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
