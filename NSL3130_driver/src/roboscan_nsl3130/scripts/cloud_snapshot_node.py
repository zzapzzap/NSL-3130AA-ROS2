#!/usr/bin/env python3
"""Publish a one-shot PointCloud2 snapshot when a trigger topic is received."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty


def _reliable_qos(depth=1):
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class CloudSnapshot(Node):
    def __init__(self, args):
        super().__init__(args.node_name)
        self.source_topic = args.source_topic
        self.output_topic = args.output_topic
        self.trigger_topic = args.trigger_topic
        self.max_age_sec = max(0.0, float(args.max_age_sec))
        self.debounce_sec = max(0.0, float(args.debounce_sec))
        self.pending = False
        self.last_cloud = None
        self.last_cloud_rx = 0.0
        self.last_trigger = 0.0
        self.last_publish = 0.0

        self.pub = self.create_publisher(PointCloud2, self.output_topic, _reliable_qos(1))
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.source_topic, self._cloud_cb, _reliable_qos(5))
        self.trigger_sub = self.create_subscription(
            Empty, self.trigger_topic, self._trigger_cb, _reliable_qos(5))
        self.get_logger().info(
            f'cloud snapshot: {self.source_topic} -> {self.output_topic} '
            f'on {self.trigger_topic}, max_age={self.max_age_sec:.2f}s, '
            f'debounce={self.debounce_sec:.2f}s')

    def _cloud_cb(self, msg):
        self.last_cloud = msg
        self.last_cloud_rx = time.monotonic()
        if self.pending:
            self._publish_snapshot(reason='next cloud')

    def _trigger_cb(self, _msg):
        now = time.monotonic()
        if self.debounce_sec > 0.0 and now - self.last_trigger < self.debounce_sec:
            return
        self.last_trigger = now

        if self.last_cloud is not None and (
                self.max_age_sec <= 0.0 or now - self.last_cloud_rx <= self.max_age_sec):
            self._publish_snapshot(reason='cached cloud')
        else:
            self.pending = True
            self.get_logger().info('snapshot armed; waiting for the next cloud')

    def _publish_snapshot(self, reason):
        if self.last_cloud is None:
            self.pending = True
            return
        self.pending = False
        self.last_publish = time.monotonic()
        self.pub.publish(self.last_cloud)
        self.get_logger().info(
            f'published snapshot ({reason}): {self.last_cloud.width} pts, '
            f'frame={self.last_cloud.header.frame_id}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-topic', required=True)
    parser.add_argument('--output-topic', required=True)
    parser.add_argument('--trigger-topic', default='/fleet/calibrate')
    parser.add_argument('--max-age-sec', type=float, default=1.0)
    parser.add_argument('--debounce-sec', type=float, default=3.0)
    parser.add_argument('--node-name', default='cloud_snapshot')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args or None)
    node = CloudSnapshot(args)
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
