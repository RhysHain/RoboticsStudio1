#!/usr/bin/env python3
# simple_navigator.py
#
# Usage:
#  - publish a goal as geometry_msgs/PoseStamped to /goal_point (frame: odom)
#  - ensure /odom and /husky/scan are available
#  - node publishes to /cmd_vel

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
import time

def angle_normalize(x):
    return math.atan2(math.sin(x), math.cos(x))

class SimpleReactiveNavigator(Node):
    def __init__(self):
        super().__init__('simple_reactive_navigator')

        # Parameters (tune these)
        self.declare_parameter('scan_topic', '/husky/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('goal_topic', '/goal_point')
        self.declare_parameter('max_speed', 0.8)
        self.declare_parameter('max_ang', 1.5)
        self.declare_parameter('goal_tolerance', 0.6)         # accept waypoint
        self.declare_parameter('yaw_tolerance', 0.35)         # rad
        self.declare_parameter('max_step', 8.0)               # meters between waypoints
        self.declare_parameter('attractive_gain', 0.9)
        self.declare_parameter('repulsive_gain', 0.9)
        self.declare_parameter('obstacle_influence_dist', 6.0) # radius for repulsion
        self.declare_parameter('min_obstacle_dist', 0.45)     # stop distance close obstacle
        self.declare_parameter('backup_time', 1.0)
        self.declare_parameter('recovery_turn_time', 0.8)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value

        # Gains etc
        self.max_speed = self.get_parameter('max_speed').value
        self.max_ang = self.get_parameter('max_ang').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.yaw_tol = self.get_parameter('yaw_tolerance').value
        self.max_step = self.get_parameter('max_step').value
        self.K_att = self.get_parameter('attractive_gain').value
        self.K_rep = self.get_parameter('repulsive_gain').value
        self.obs_infl = self.get_parameter('obstacle_influence_dist').value
        self.min_obs_dist = self.get_parameter('min_obstacle_dist').value
        self.backup_time = self.get_parameter('backup_time').value
        self.recovery_turn_time = self.get_parameter('recovery_turn_time').value

        # State
        self.odom = None
        self.scan = None
        self.current_goal = None  # final goal pose (x,y)
        self.waypoints = []       # list of (x,y)
        self.current_waypoint_idx = 0
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # Subscribers
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)
        self.create_subscription(Pose, '/husky/pose', self.pose_cb, 10)

        # timer
        self.timer = self.create_timer(0.1, self.timer_cb)

        # Recovery state
        self.in_recovery = False
        self.recovery_end_time = 0.0

        self.get_logger().info('Simple reactive navigator started. Listening for goals on %s' % self.goal_topic)

    def odom_cb(self, msg: Odometry):
        self.odom = Odometry()
        self.odom.twist = msg.twist

    def scan_cb(self, msg: LaserScan):
        self.scan = msg
    
    def pose_cb(self, msg: Pose):
        self.odom.pose = msg

    def goal_cb(self, msg: PoseStamped):
        # Only accept goal if in odom frame (or same as odom)
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(f'New goal received: x={gx:.2f}, y={gy:.2f}')
        self.current_goal = (gx, gy)
        self.waypoints = self._split_waypoints(gx, gy)
        self.current_waypoint_idx = 0
        self.in_recovery = False

    def _split_waypoints(self, gx, gy):
        if self.odom is None:
            # can't split without current position; just one waypoint
            return [(gx, gy)]
        sx = self.odom.pose.pose.position.x
        sy = self.odom.pose.pose.position.y
        dx = gx - sx
        dy = gy - sy
        dist = math.hypot(dx, dy)
        if dist <= self.max_step:
            return [(gx, gy)]
        n = max(1, int(math.ceil(dist / self.max_step)))
        waypoints = []
        for i in range(1, n+1):
            wx = sx + dx * (i / float(n))
            wy = sy + dy * (i / float(n))
            waypoints.append((wx, wy))
        self.get_logger().info(f'Goal split into {len(waypoints)} waypoints (max_step={self.max_step} m)')
        return waypoints

    def timer_cb(self):
        if self.odom is None or self.scan is None:
            return

        # If no goal -> stop
        if not self.waypoints:
            self._publish_stop()
            return

        # recovery: if in recovery, perform backup / turn until end time
        now = self.get_clock().now().nanoseconds / 1e9
        if self.in_recovery:
            if now < self.recovery_end_time:
                self._do_recovery()
                return
            else:
                self.in_recovery = False

        # current waypoint
        wx, wy = self.waypoints[self.current_waypoint_idx]
        sx = self.odom.pose.pose.position.x
        sy = self.odom.pose.pose.position.y
        dx = wx - sx
        dy = wy - sy
        dist = math.hypot(dx, dy)

        # orientation of robot
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

        # heading to waypoint
        target_yaw = math.atan2(dy, dx)
        yaw_err = angle_normalize(target_yaw - yaw)

        # If close to waypoint -> advance
        if dist < self.goal_tol:
            self.get_logger().info(f'Waypoint {self.current_waypoint_idx+1}/{len(self.waypoints)} reached (dist {dist:.2f})')
            if self.current_waypoint_idx + 1 < len(self.waypoints):
                self.current_waypoint_idx += 1
                return
            else:
                self.get_logger().info('Final goal reached.')
                self.waypoints = []
                self.current_goal = None
                self._publish_stop()
                return

        # if obstacle too close directly ahead, trigger recovery
        if self._closest_obstacle_distance() < self.min_obs_dist:
            self.get_logger().warn('Obstacle too close! Starting recovery (backup+turn).')
            self._start_recovery()
            return

        # Reactive control: attractive vector + repulsive vectors from scan
        ax, ay = self._attractive_vector(sx, sy, wx, wy)
        rx, ry = self._repulsive_vector()
        # combine
        vx = self.K_att * ax + self.K_rep * rx
        vy = self.K_att * ay + self.K_rep * ry

        # convert desired velocity vector to linear speed and heading
        desired_angle = math.atan2(vy, vx)
        desired_speed = math.hypot(vx, vy)

        # rotate speed into robot frame (we have yaw)
        # we command forward speed = desired_speed * cos(angle error)
        ang_err = angle_normalize(desired_angle - yaw)
        # simple angular control
        ang_cmd = max(-self.max_ang, min(self.max_ang, 2.0 * ang_err))
        # linear reduced when turning
        lin_cmd = max(0.0, min(self.max_speed, desired_speed * max(0.0, math.cos(ang_err))))

        # small bias to prefer heading to waypoint (to avoid circling)
        # if yaw error large, rotate first
        if abs(yaw_err) > 0.6:
            lin_cmd *= 0.2

        # publish
        t = Twist()
        t.linear.x = lin_cmd
        t.angular.z = ang_cmd
        self.cmd_pub.publish(t)

    def _publish_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def _closest_obstacle_distance(self):
        # find min valid range
        if self.scan is None:
            return float('inf')
        rngs = [r for r in self.scan.ranges if r is not None and not math.isinf(r)]
        if not rngs:
            return float('inf')
        return min(rngs)

    def _start_recovery(self):
        self.in_recovery = True
        now = self.get_clock().now().nanoseconds / 1e9
        # backup for backup_time then turn
        self.recovery_end_time = now + self.backup_time + self.recovery_turn_time
        # store a flag for two-step recovery: we'll back up first then turn
        self._recovery_phase = 'backup'
        self._recovery_phase_end = now + self.backup_time

    def _do_recovery(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = Twist()
        if now < self._recovery_phase_end:
            # back up
            t.linear.x = -0.2
            t.angular.z = 0.0
        else:
            # turn in place
            t.linear.x = 0.0
            t.angular.z = 0.8
        self.cmd_pub.publish(t)

    def _attractive_vector(self, sx, sy, wx, wy):
        # unit vector pointing to waypoint
        dx = wx - sx
        dy = wy - sy
        d = math.hypot(dx, dy)
        if d == 0:
            return 0.0, 0.0
        return dx / d * min(self.max_speed, d), dy / d * min(self.max_speed, d)

    def _repulsive_vector(self):
        # compute repulsive vector from scan points in robot frame,
        # then rotate to world frame based on odom yaw. For simplicity we do everything in robot frame
        if self.scan is None or self.odom is None:
            return 0.0, 0.0
        ranges = self.scan.ranges
        angle = self.scan.angle_min
        rx = 0.0
        ry = 0.0
        cnt = 0
        for r in ranges:
            if r is None or math.isinf(r):
                angle += self.scan.angle_increment
                continue
            if r > self.obs_infl:
                angle += self.scan.angle_increment
                continue
            # vector from robot toward obstacle (robot-frame)
            obs_x = r * math.cos(angle)
            obs_y = r * math.sin(angle)
            # repulsive magnitude (inverse-square)
            if r <= 0.0:
                angle += self.scan.angle_increment
                continue
            mag = max(0.0, (self.obs_infl - r) / (r*r + 1e-6))
            # direction away from obstacle
            rx += -mag * (obs_x)
            ry += -mag * (obs_y)
            cnt += 1
            angle += self.scan.angle_increment

        if cnt == 0:
            return 0.0, 0.0

        # normalize repulsive vector
        norm = math.hypot(rx, ry)
        if norm == 0:
            return 0.0, 0.0
        rx /= norm
        ry /= norm

        # rotate from robot frame to world frame by adding current yaw
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        cosy = math.cos(yaw)
        siny = math.sin(yaw)
        wx = rx * cosy - ry * siny
        wy = rx * siny + ry * cosy
        # scale
        return wx, wy

def main(args=None):
    rclpy.init(args=args)
    node = SimpleReactiveNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
