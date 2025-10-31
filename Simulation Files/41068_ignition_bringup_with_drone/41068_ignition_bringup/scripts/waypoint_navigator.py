#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Action client for Nav2 waypoint following
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        # Subscribe to odometry to get current position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.current_x = 0.0
        self.current_y = 0.0

        # Parameters
        self.declare_parameter('waypoint_spacing', 10.0)  # meters between waypoints

        self.get_logger().info('Waypoint Navigator ready!')
        self.get_logger().info('Usage: Call navigate_to_goal(x, y) to navigate to a far goal')

    def odom_callback(self, msg):
        """Store current robot position"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def generate_waypoints(self, goal_x, goal_y):
        """
        Generate evenly spaced waypoints from current position to goal.

        Args:
            goal_x: Goal X coordinate
            goal_y: Goal Y coordinate

        Returns:
            List of PoseStamped waypoints
        """
        waypoints = []

        # Calculate distance and direction
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        spacing = self.get_parameter('waypoint_spacing').value

        # Calculate number of intermediate waypoints
        num_waypoints = max(1, int(distance / spacing))

        self.get_logger().info(f'Distance to goal: {distance:.2f}m')
        self.get_logger().info(f'Generating {num_waypoints} intermediate waypoints')

        # Generate intermediate waypoints
        for i in range(1, num_waypoints + 1):
            t = i / (num_waypoints + 1)  # Interpolation factor (0 to 1)

            wp = PoseStamped()
            wp.header.frame_id = 'map'
            wp.header.stamp = self.get_clock().now().to_msg()

            wp.pose.position.x = self.current_x + t * dx
            wp.pose.position.y = self.current_y + t * dy
            wp.pose.position.z = 0.0

            # Orientation pointing toward goal
            yaw = math.atan2(dy, dx)
            wp.pose.orientation.z = math.sin(yaw / 2.0)
            wp.pose.orientation.w = math.cos(yaw / 2.0)

            waypoints.append(wp)
            self.get_logger().info(f'  Waypoint {i}: ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})')

        # Add final goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        yaw = math.atan2(dy, dx)
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        waypoints.append(goal_pose)
        self.get_logger().info(f'  Final goal: ({goal_x:.2f}, {goal_y:.2f})')

        return waypoints

    def navigate_to_goal(self, goal_x, goal_y):
        """
        Navigate to a far goal using intermediate waypoints.

        Args:
            goal_x: Goal X coordinate in map frame
            goal_y: Goal Y coordinate in map frame
        """
        self.get_logger().info(f'Starting navigation from ({self.current_x:.2f}, {self.current_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})')

        # Wait for action server
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()

        # Generate waypoints
        waypoints = self.generate_waypoints(goal_x, goal_y)

        # Create goal message
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = waypoints

        # Send goal
        self.get_logger().info('Sending waypoints to Nav2...')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback when goal is accepted/rejected"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2!')
            return

        self.get_logger().info('Goal accepted! Robot navigating through waypoints...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Callback when navigation completes"""
        result = future.result().result

        if result:
            self.get_logger().info('Navigation completed successfully!')
        else:
            self.get_logger().warn('Navigation failed or was cancelled')


def main(args=None):
    rclpy.init(args=args)

    navigator = WaypointNavigator()

    # Example: Navigate to a far goal
    # Uncomment and modify these coordinates to test:
    # navigator.navigate_to_goal(50.0, 0.0)  # 50 meters ahead

    # Or use it interactively in Python:
    # from waypoint_navigator import WaypointNavigator
    # nav = WaypointNavigator()
    # nav.navigate_to_goal(x, y)

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
