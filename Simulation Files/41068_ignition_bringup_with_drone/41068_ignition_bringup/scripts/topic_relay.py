#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class TopicRelay(Node):
    def __init__(self):
        super().__init__('topic_relay')

        # Relay /husky/scan to /scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/husky/scan',
            self.scan_callback,
            10)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Relay /husky/odometry to /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/husky/odometry',
            self.odom_callback,
            10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.get_logger().info('Topic relay started: /husky/scan -> /scan, /husky/odometry -> /odom')

    def scan_callback(self, msg):
        self.scan_pub.publish(msg)

    def odom_callback(self, msg):
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TopicRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
