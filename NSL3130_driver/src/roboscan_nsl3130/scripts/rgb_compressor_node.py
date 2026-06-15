#!/usr/bin/env python3
"""Lightweight RGB JPEG compressor for the NSL fleet runtime.

The camera driver publishes raw RGB for local edge inference. Remote hosts should
prefer this compressed mirror when they only need visual inspection or archival
RGB, while pose/fusion continues to use DDS result topics.
"""

import argparse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


def _sensor_qos(depth=5):
    # RELIABLE so the full (optionally 1080p) JPEG is reassembled losslessly at
    # the host even when switch incast drops fragments; KeepLast(5) gives the
    # writer room to retransmit before the slot is overwritten. The LAN is
    # <5% utilised so the retransmits are cheap. RViz/host subscribers must also
    # request RELIABLE (see multiview.launch.py Image display).
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


class RgbCompressor(Node):
    def __init__(self, args):
        super().__init__("rgb_compressor")
        self.bridge = CvBridge()
        self.quality = max(1, min(100, int(args.quality)))
        self.resize_width = max(0, int(args.resize_width))
        self.frame_skip = max(0, int(args.frame_skip))
        self.frame_index = 0

        self.pub = self.create_publisher(CompressedImage, args.output_topic, _sensor_qos(1))
        self.sub = self.create_subscription(Image, args.input_topic, self._cb, _sensor_qos(1))
        self.get_logger().info(
            f"RGB compressor: {args.input_topic} -> {args.output_topic}, "
            f"jpeg_quality={self.quality}, resize_width={self.resize_width or 'native'}, "
            f"frame_skip={self.frame_skip}"
        )

    def _cb(self, msg):
        if self.frame_skip and (self.frame_index % (self.frame_skip + 1)) != 0:
            self.frame_index += 1
            return
        self.frame_index += 1

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"image decode failed: {exc}")
            return

        if self.resize_width > 0 and image.shape[1] > self.resize_width:
            scale = float(self.resize_width) / float(image.shape[1])
            image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            self.get_logger().warn("JPEG encode failed")
            return

        out = CompressedImage()
        out.header = msg.header
        # compressed_image_transport's parser expects "<raw_encoding>; <codec>
        # compressed <encoding>"; anything else (e.g. "jpeg; quality=80")
        # crashes rviz2 with "Unknown encoding jpeg".
        out.format = "bgr8; jpeg compressed bgr8"
        out.data = encoded.tobytes()
        self.pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", default="/camera/rgb/image_raw")
    parser.add_argument("--output-topic", default="/camera/rgb/image_raw/compressed")
    parser.add_argument("--quality", type=int, default=80)
    parser.add_argument("--resize-width", type=int, default=0)
    parser.add_argument("--frame-skip", type=int, default=0, help="0=publish every frame, 1=every other frame")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args or None)
    node = RgbCompressor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
