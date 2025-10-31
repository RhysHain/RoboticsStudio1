#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/simple_nav/goal', self.goal_callback, 10)

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_x = None
        self.goal_y = None

        self.min_front_distance = float('inf')
        self.min_left_distance = float('inf')
        self.min_right_distance = float('inf')

        # Stuck detection
        self.last_position_x = 0.0
        self.last_position_y = 0.0
        self.stuck_counter = 0
        self.preferred_turn_direction = 1.0  # 1.0 = left, -1.0 = right

        # Parameters
        self.declare_parameter('max_speed', 1.5)  # Reduced from 2.0 for better control
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('obstacle_distance', 2.5)  # Stop if obstacle closer than this (was 1.5)
        self.declare_parameter('avoidance_distance', 4.0)  # Start avoiding if closer than this (was 2.5)

        # Control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info('Simple Navigator started!')
        self.get_logger().info('Send goals to /simple_nav/goal topic')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        """Process laser scan to detect obstacles"""
        if len(msg.ranges) == 0:
            return

        # Laser scan: 360 readings from 0 to 2π
        # Index 0 = FORWARD (0 rad), 90 = LEFT (π/2), 180 = BACK (π), 270 = RIGHT (3π/2)
        num_ranges = len(msg.ranges)

        # Front: wrap around 0 (last 30 degrees + first 30 degrees)
        # Take last 30 indices (330-360) and first 30 indices (0-30)
        front_ranges = list(msg.ranges[-30:]) + list(msg.ranges[:30])
        self.min_front_distance = min([r for r in front_ranges if r > 0.1 and r != float('inf')], default=float('inf'))

        # Left: 60-120 degrees (90 degree arc on left side, around index 90)
        left_start = 60
        left_end = 120
        left_ranges = msg.ranges[left_start:left_end]
        self.min_left_distance = min([r for r in left_ranges if r > 0.1 and r != float('inf')], default=float('inf'))

        # Right: 240-300 degrees (90 degree arc on right side, around index 270)
        right_start = 240
        right_end = 300
        right_ranges = msg.ranges[right_start:right_end]
        self.min_right_distance = min([r for r in right_ranges if r > 0.1 and r != float('inf')], default=float('inf'))

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f'New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def control_loop(self):
        if self.goal_x is None or self.goal_y is None:
            return

        # Calculate distance and angle to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        # Check if goal reached
        goal_tolerance = self.get_parameter('goal_tolerance').value
        if distance_to_goal < goal_tolerance:
            self.get_logger().info('Goal reached!')
            self.stop()
            self.goal_x = None
            self.goal_y = None
            self.stuck_counter = 0
            return

        # Detect if stuck (not moving)
        position_delta = math.sqrt((self.current_x - self.last_position_x)**2 +
                                   (self.current_y - self.last_position_y)**2)

        if position_delta < 0.05:  # Moved less than 5cm in 0.05s
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        self.last_position_x = self.current_x
        self.last_position_y = self.current_y

        # Calculate angle error
        angle_error = self.normalize_angle(angle_to_goal - self.current_yaw)

        # Get parameters
        max_speed = self.get_parameter('max_speed').value
        obstacle_dist = self.get_parameter('obstacle_distance').value
        avoid_dist = self.get_parameter('avoidance_distance').value

        cmd = Twist()

        # STUCK RECOVERY - if stuck for more than 2 seconds (40 iterations at 20Hz)
        if self.stuck_counter > 40:
            self.get_logger().warn(f'STUCK! Executing recovery maneuver')
            # Back up slowly while turning
            cmd.linear.x = -0.3
            cmd.angular.z = self.preferred_turn_direction * 1.0

        # OBSTACLE AVOIDANCE LOGIC
        elif self.min_front_distance < obstacle_dist:
            # STOP - obstacle too close in front
            # Pick a turn direction and stick with it to avoid oscillation
            turn_diff = self.min_left_distance - self.min_right_distance

            if abs(turn_diff) < 0.5:  # Too close to call - use preferred direction
                self.get_logger().warn(f'Obstacle both sides, using preferred direction')
                cmd.angular.z = self.preferred_turn_direction * 0.8
            else:
                # Clear winner - pick the more open side
                if turn_diff > 0:
                    self.preferred_turn_direction = 1.0  # Left
                else:
                    self.preferred_turn_direction = -1.0  # Right
                cmd.angular.z = self.preferred_turn_direction * 0.8

            cmd.linear.x = 0.0
            self.get_logger().warn(f'Obstacle at {self.min_front_distance:.2f}m - Turning {("left" if self.preferred_turn_direction > 0 else "right")}')

        elif self.min_front_distance < avoid_dist:
            # AVOID - slow down and turn away from obstacle
            # Use preferred direction to maintain consistency
            turn_diff = self.min_left_distance - self.min_right_distance

            if abs(turn_diff) > 0.5:
                # Update preferred direction if there's a clear choice
                if turn_diff > 0:
                    self.preferred_turn_direction = 1.0
                else:
                    self.preferred_turn_direction = -1.0

            cmd.linear.x = max_speed * 0.2
            cmd.angular.z = self.preferred_turn_direction * 1.5
            self.get_logger().info(f'Avoiding obstacle at {self.min_front_distance:.2f}m')

        else:
            # NORMAL NAVIGATION - head toward goal
            # Reset stuck counter when moving freely
            if self.stuck_counter > 0:
                self.stuck_counter = max(0, self.stuck_counter - 1)

            # Proportional control for angular velocity
            angular_gain = 2.0
            cmd.angular.z = max(min(angular_gain * angle_error, 2.0), -2.0)

            # Speed based on angle error (slow down when turning)
            if abs(angle_error) > 0.5:  # ~30 degrees
                cmd.linear.x = max_speed * 0.5
            elif abs(angle_error) > 0.2:  # ~11 degrees
                cmd.linear.x = max_speed * 0.7
            else:
                cmd.linear.x = max_speed

        self.cmd_vel_pub.publish(cmd)

        # Log status periodically
        if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # ~1 second
            self.get_logger().info(
                f'Distance: {distance_to_goal:.2f}m, '
                f'Angle error: {math.degrees(angle_error):.1f}°, '
                f'Front obstacle: {self.min_front_distance:.2f}m'
            )

    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.stop()
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
