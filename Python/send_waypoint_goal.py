#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import time

class WaypointGoalSender(Node):
    def __init__(self):
        super().__init__('waypoint_goal_sender')

        self.current_x = 0.0
        self.current_y = 0.0
        self.odom_received = False

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Action client
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.get_logger().info('Waypoint Goal Sender initialized')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.odom_received = True

    def generate_waypoints(self, goal_x, goal_y, spacing=5.0):
        """Generate waypoints from current position to goal"""
        waypoints = []

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        num_waypoints = max(1, int(distance / spacing))

        self.get_logger().info(f'Current position: ({self.current_x:.2f}, {self.current_y:.2f})')
        self.get_logger().info(f'Goal: ({goal_x:.2f}, {goal_y:.2f})')
        self.get_logger().info(f'Distance: {distance:.2f}m')
        self.get_logger().info(f'Generating {num_waypoints} intermediate waypoints')

        for i in range(1, num_waypoints + 2):
            t = i / (num_waypoints + 1)

            wp = PoseStamped()
            wp.header.frame_id = 'map'
            wp.header.stamp = self.get_clock().now().to_msg()

            wp.pose.position.x = self.current_x + t * dx
            wp.pose.position.y = self.current_y + t * dy
            wp.pose.position.z = 0.0

            # Orientation toward goal
            yaw = math.atan2(dy, dx)
            wp.pose.orientation.z = math.sin(yaw / 2.0)
            wp.pose.orientation.w = math.cos(yaw / 2.0)

            waypoints.append(wp)
            self.get_logger().info(f'  Waypoint {i}: ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})')

        return waypoints

    def send_goal(self, goal_x, goal_y):
        """Send navigation goal with waypoints"""
        # Wait for odometry
        self.get_logger().info('Waiting for odometry...')
        while not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Wait for action server
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return False

        # Generate waypoints
        waypoints = self.generate_waypoints(goal_x, goal_y)

        # Create goal
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = waypoints

        # Send goal
        self.get_logger().info('Sending waypoints to Nav2...')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted! Robot navigating...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        self.get_logger().info(f'Navigation result: {result.status}')

        return True

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 send_waypoint_goal.py <goal_x> <goal_y> [waypoint_spacing]")
        print("Example: python3 send_waypoint_goal.py 8.0 8.0")
        print("Example: python3 send_waypoint_goal.py 8.0 8.0 3.0")
        return

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    spacing = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

    rclpy.init()

    sender = WaypointGoalSender()

    try:
        sender.send_goal(goal_x, goal_y)
    except KeyboardInterrupt:
        print("\nCancelled by user")
    finally:
        sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
