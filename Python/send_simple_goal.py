#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 send_simple_goal.py <x> <y>")
        print("Example: python3 send_simple_goal.py 7.0 7.0")
        return

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])

    rclpy.init()
    node = Node('simple_goal_sender')

    pub = node.create_publisher(PoseStamped, '/simple_nav/goal', 10)

    # Wait for publisher to connect
    import time
    time.sleep(0.5)

    # Send goal
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = 'map'
    goal_msg.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.position.x = goal_x
    goal_msg.pose.position.y = goal_y
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation.w = 1.0

    pub.publish(goal_msg)
    print(f'Goal sent: ({goal_x}, {goal_y})')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
