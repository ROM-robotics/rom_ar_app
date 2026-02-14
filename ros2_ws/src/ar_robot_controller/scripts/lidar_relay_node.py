#!/usr/bin/env python3
"""
LiDAR Relay Node

Subscribes to /scan topic and reformats LiDAR data
for efficient WebSocket transmission to the AR app.
Downsamples the scan data to reduce bandwidth.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json
import math


class LidarRelayNode(Node):
    def __init__(self):
        super().__init__('lidar_relay_node')

        self.declare_parameter('downsample_factor', 4)
        self.declare_parameter('max_range_viz', 5.0)
        self.declare_parameter('relay_rate', 10.0)

        self.downsample = self.get_parameter('downsample_factor').value
        self.max_range = self.get_parameter('max_range_viz').value
        self.relay_rate = self.get_parameter('relay_rate').value

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.scan_pub = self.create_publisher(
            String, '/ar_controller/lidar_viz', 10)

        self.latest_scan = None
        self.timer = self.create_timer(1.0 / self.relay_rate, self.publish_scan)

        self.get_logger().info('📡 LiDAR Relay Node started')
        self.get_logger().info(f'   Downsample factor: {self.downsample}')
        self.get_logger().info(f'   Max range: {self.max_range}m')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def publish_scan(self):
        if self.latest_scan is None:
            return

        scan = self.latest_scan
        points = []

        for i in range(0, len(scan.ranges), self.downsample):
            r = scan.ranges[i]
            if math.isnan(r) or math.isinf(r) or r > self.max_range:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([round(x, 3), round(y, 3)])

        msg = String()
        msg.data = json.dumps({
            'type': 'lidar',
            'points': points,
            'max_range': self.max_range,
            'angle_min': scan.angle_min,
            'angle_max': scan.angle_max
        })
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
