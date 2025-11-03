#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class ScanToXYOnce(Node):
    def __init__(self):
        super().__init__('scan_to_xy_once')
        self.subscription = self.create_subscription(
            LaserScan,
            '/husky/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.received = False

    def scan_callback(self, msg: LaserScan):
        if self.received:
            return  # Ignore extra messages
        self.received = True

        angle = msg.angle_min
        coords = []

        for r in msg.ranges:
            if math.isfinite(r):  # filter out inf and NaN
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                coords.append((x, y))
            angle += msg.angle_increment

        # Print the coordinates
        print("Valid scan points (x, y):")
        for x, y in coords:
            print(f"{x:.3f}, {y:.3f}")

        # Shut down after printing
        self.get_logger().info("Printed one scan, shutting down...")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ScanToXYOnce()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
