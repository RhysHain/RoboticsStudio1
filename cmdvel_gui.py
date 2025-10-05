#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QLabel, QPushButton, QSlider, QGroupBox,
                             QLineEdit, QGridLayout, QMessageBox)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, Qt
from PyQt5.QtGui import QFont
import threading
import math

class CmdVelSignals(QObject):
    """Signals for thread-safe GUI updates"""
    velocity_updated = pyqtSignal(float, float, float)
    goal_status_updated = pyqtSignal(str, str)  # (status, message)
    drone_goal_sent = pyqtSignal(str)  # status message
    codes_bool_sent = pyqtSignal(str)  # status message for bool topic

class CmdVelNode(Node):
    def __init__(self, signals):
        super().__init__('cmdvel_gui_node')
        self.signals = signals
        
        # Subscriber to monitor cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10)
        
        # Publisher to send cmd_vel commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for drone goals
        self.drone_goal_publisher = self.create_publisher(
            Pose, 
            'CODES/parrot/goals', 
            10
        )
        
        # Publisher for CODES bool topic
        self.codes_bool_publisher = self.create_publisher(
            Bool,
            '/CODES/parrot/active',
            10
        )
        
        # Nav2 Action Client
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        
        self.get_logger().info('CMD_VEL GUI node started - Husky + Drone control enabled')

    def velocity_callback(self, msg):
        # Extract velocities and emit signal for GUI update
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        self.signals.velocity_updated.emit(linear_x, linear_y, angular_z)

    def publish_velocity(self, linear_x, linear_y, angular_z):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def publish_drone_goal(self, x, y, z, yaw=0.0):
        """Publish 3D goal for drone to CODES/parrot/goals"""
        goal_msg = Pose()
        
        # Set position (3D)
        goal_msg.position.x = x
        goal_msg.position.y = y
        goal_msg.position.z = z
        
        # Convert yaw to quaternion (z-axis rotation)
        goal_msg.orientation.x = 0.0
        goal_msg.orientation.y = 0.0
        goal_msg.orientation.z = math.sin(yaw / 2.0)
        goal_msg.orientation.w = math.cos(yaw / 2.0)
        
        self.drone_goal_publisher.publish(goal_msg)
        self.get_logger().info(f'Drone goal published: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}')
        
        self.signals.drone_goal_sent.emit(f'Goal sent: ({x:.2f}, {y:.2f}, {z:.2f})')

    def publish_codes_bool(self):
        """Publish True to /CODES/parrot/active topic"""
        msg = Bool()
        msg.data = True
        self.codes_bool_publisher.publish(msg)
        self.get_logger().info('Published True to /CODES/parrot/active')
        self.signals.codes_bool_sent.emit('Active signal sent')

    def send_nav_goal(self, x, y, yaw):
        """Send navigation goal to Nav2"""
        # Wait for action server
        if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
            self.signals.goal_status_updated.emit('error', 'Nav2 action server not available')
            self.get_logger().error('Navigate to pose action server not available')
            return False
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion (simplified for z-axis rotation)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        self.signals.goal_status_updated.emit('sending', f'Sending goal to ({x:.2f}, {y:.2f})')
        
        # Send goal
        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
        
        return True

    def nav_goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.signals.goal_status_updated.emit('rejected', 'Goal was rejected by Nav2')
            self.get_logger().error('Goal rejected')
            return
        
        self.current_goal_handle = goal_handle
        self.signals.goal_status_updated.emit('accepted', 'Goal accepted, navigating...')
        self.get_logger().info('Goal accepted')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        pass

    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.signals.goal_status_updated.emit('succeeded', 'Goal reached successfully!')
            self.get_logger().info('Goal succeeded')
        elif status == GoalStatus.STATUS_ABORTED:
            self.signals.goal_status_updated.emit('aborted', 'Goal was aborted')
            self.get_logger().error('Goal aborted')
        elif status == GoalStatus.STATUS_CANCELED:
            self.signals.goal_status_updated.emit('canceled', 'Goal was canceled')
            self.get_logger().warn('Goal canceled')
        else:
            self.signals.goal_status_updated.emit('unknown', f'Unknown status: {status}')
            self.get_logger().error(f'Unknown result status: {status}')
        
        self.current_goal_handle = None

    def cancel_nav_goal(self):
        """Cancel current navigation goal"""
        if self.current_goal_handle:
            self.get_logger().info('Canceling current navigation goal')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            return True
        return False

    def cancel_done_callback(self, future):
        """Handle goal cancellation result"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.signals.goal_status_updated.emit('canceling', 'Canceling navigation...')
            self.get_logger().info('Goal cancellation accepted')
        else:
            self.signals.goal_status_updated.emit('error', 'Failed to cancel goal')
            self.get_logger().error('Goal cancellation rejected')

class CmdVelGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Husky + Drone Control Center')
        self.setGeometry(100, 100, 700, 1050)
        
        # Create signals object
        self.signals = CmdVelSignals()
        self.signals.velocity_updated.connect(self.update_velocity_display)
        self.signals.goal_status_updated.connect(self.update_goal_status)
        self.signals.drone_goal_sent.connect(self.update_drone_status)
        self.signals.codes_bool_sent.connect(self.update_codes_bool_status)
        
        # Initialize ROS2 in separate thread
        self.ros_thread = None
        self.node = None
        self.init_ros()
        
        self.init_ui()
        
        # Current velocity values
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # Real-time control - always enabled
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.send_continuous_commands)
        self.control_timer.start(50)  # Send commands every 50ms (20Hz)
        
        # Keyboard control state
        self.key_forward = False
        self.key_backward = False
        self.key_left = False
        self.key_right = False

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Title
        title = QLabel('Husky + Drone Control Center')
        title.setFont(QFont('Arial', 18, QFont.Bold))
        title.setStyleSheet("QLabel { color: #2E86C1; margin: 15px; }")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # ========== DRONE GOAL SECTION ==========
        drone_group = QGroupBox("🚁 Drone 3D Goal Control (Parrot)")
        drone_layout = QVBoxLayout(drone_group)
        
        # Drone goal input fields
        drone_input_layout = QGridLayout()
        
        drone_input_layout.addWidget(QLabel('X (m):'), 0, 0)
        self.drone_x_input = QLineEdit('0.0')
        self.drone_x_input.setMaximumWidth(100)
        drone_input_layout.addWidget(self.drone_x_input, 0, 1)
        
        drone_input_layout.addWidget(QLabel('Y (m):'), 0, 2)
        self.drone_y_input = QLineEdit('0.0')
        self.drone_y_input.setMaximumWidth(100)
        drone_input_layout.addWidget(self.drone_y_input, 0, 3)
        
        drone_input_layout.addWidget(QLabel('Z (m):'), 1, 0)
        self.drone_z_input = QLineEdit('1.0')  # Default 1m altitude
        self.drone_z_input.setMaximumWidth(100)
        drone_input_layout.addWidget(self.drone_z_input, 1, 1)
        
        drone_input_layout.addWidget(QLabel('Yaw (rad):'), 1, 2)
        self.drone_yaw_input = QLineEdit('0.0')
        self.drone_yaw_input.setMaximumWidth(100)
        drone_input_layout.addWidget(self.drone_yaw_input, 1, 3)
        
        drone_layout.addLayout(drone_input_layout)
        
        # Send drone goal button
        self.send_drone_goal_btn = QPushButton("Send Drone Goal")
        self.send_drone_goal_btn.setFont(QFont('Arial', 11, QFont.Bold))
        self.send_drone_goal_btn.setStyleSheet("""
            QPushButton { 
                background-color: #8E44AD; color: white; 
                border-radius: 5px; padding: 10px;
            }
            QPushButton:pressed { 
                background-color: #6C3483; 
            }
        """)
        self.send_drone_goal_btn.clicked.connect(self.send_drone_goal)
        drone_layout.addWidget(self.send_drone_goal_btn)
        
        # Send True to /CODES/parrot/active button
        self.send_codes_bool_btn = QPushButton("Activate Drone")
        self.send_codes_bool_btn.setFont(QFont('Arial', 11, QFont.Bold))
        self.send_codes_bool_btn.setStyleSheet("""
            QPushButton { 
                background-color: #16A085; color: white; 
                border-radius: 5px; padding: 10px;
            }
            QPushButton:pressed { 
                background-color: #138D75; 
            }
        """)
        self.send_codes_bool_btn.clicked.connect(self.send_codes_bool)
        drone_layout.addWidget(self.send_codes_bool_btn)
        
        # Drone status display
        self.drone_status_label = QLabel('Drone Status: Ready')
        self.drone_status_label.setFont(QFont('Arial', 10))
        self.drone_status_label.setStyleSheet("""
            QLabel { 
                background-color: #F4ECF7; 
                padding: 8px; 
                border-radius: 5px;
                color: #6C3483;
            }
        """)
        drone_layout.addWidget(self.drone_status_label)
        
        layout.addWidget(drone_group)
        
        # ========== HUSKY NAV2 GOAL SECTION ==========
        nav_group = QGroupBox("🎯 Husky Nav2 Goal Control")
        nav_layout = QVBoxLayout(nav_group)
        
        # Goal input fields
        goal_input_layout = QGridLayout()
        
        goal_input_layout.addWidget(QLabel('X (m):'), 0, 0)
        self.goal_x_input = QLineEdit('0.0')
        self.goal_x_input.setMaximumWidth(100)
        goal_input_layout.addWidget(self.goal_x_input, 0, 1)
        
        goal_input_layout.addWidget(QLabel('Y (m):'), 0, 2)
        self.goal_y_input = QLineEdit('0.0')
        self.goal_y_input.setMaximumWidth(100)
        goal_input_layout.addWidget(self.goal_y_input, 0, 3)
        
        goal_input_layout.addWidget(QLabel('Yaw (rad):'), 1, 0)
        self.goal_yaw_input = QLineEdit('0.0')
        self.goal_yaw_input.setMaximumWidth(100)
        goal_input_layout.addWidget(self.goal_yaw_input, 1, 1)
        
        nav_layout.addLayout(goal_input_layout)
        
        # Goal action buttons
        goal_btn_layout = QHBoxLayout()
        
        self.send_goal_btn = QPushButton("Send Goal")
        self.send_goal_btn.setFont(QFont('Arial', 11, QFont.Bold))
        self.send_goal_btn.setStyleSheet("""
            QPushButton { 
                background-color: #27AE60; color: white; 
                border-radius: 5px; padding: 10px;
            }
            QPushButton:pressed { 
                background-color: #229954; 
            }
        """)
        self.send_goal_btn.clicked.connect(self.send_nav_goal)
        goal_btn_layout.addWidget(self.send_goal_btn)
        
        self.cancel_goal_btn = QPushButton("Cancel Goal")
        self.cancel_goal_btn.setFont(QFont('Arial', 11, QFont.Bold))
        self.cancel_goal_btn.setStyleSheet("""
            QPushButton { 
                background-color: #E67E22; color: white; 
                border-radius: 5px; padding: 10px;
            }
            QPushButton:pressed { 
                background-color: #D35400; 
            }
        """)
        self.cancel_goal_btn.clicked.connect(self.cancel_nav_goal)
        goal_btn_layout.addWidget(self.cancel_goal_btn)
        
        nav_layout.addLayout(goal_btn_layout)
        
        # Goal status display
        self.goal_status_label = QLabel('Status: Ready')
        self.goal_status_label.setFont(QFont('Arial', 10))
        self.goal_status_label.setStyleSheet("""
            QLabel { 
                background-color: #F8F9F9; 
                padding: 8px; 
                border-radius: 5px;
                color: #566573;
            }
        """)
        self.goal_status_label.setWordWrap(True)
        nav_layout.addWidget(self.goal_status_label)
        
        layout.addWidget(nav_group)
        
        # ========== KEYBOARD CONTROL SECTION ==========
        keyboard_group = QGroupBox("⌨️ Manual Control (WASD or Click)")
        keyboard_layout = QVBoxLayout(keyboard_group)
        
        # Arrow key buttons
        arrow_layout = QVBoxLayout()
        
        # Forward button
        forward_layout = QHBoxLayout()
        forward_layout.addStretch()
        self.forward_btn = QPushButton("↑")
        self.forward_btn.setFont(QFont('Arial', 36, QFont.Bold))
        self.forward_btn.setFixedSize(120, 80)
        self.forward_btn.setStyleSheet("""
            QPushButton { 
                background-color: #3498DB; color: white; border-radius: 10px; 
            }
            QPushButton:pressed { 
                background-color: #2980B9; 
            }
        """)
        self.forward_btn.pressed.connect(lambda: self.set_key_state('forward', True))
        self.forward_btn.released.connect(lambda: self.set_key_state('forward', False))
        forward_layout.addWidget(self.forward_btn)
        forward_layout.addStretch()
        arrow_layout.addLayout(forward_layout)
        
        # Left/Right buttons
        middle_layout = QHBoxLayout()
        self.left_btn = QPushButton("←")
        self.left_btn.setFont(QFont('Arial', 36, QFont.Bold))
        self.left_btn.setFixedSize(120, 80)
        self.left_btn.setStyleSheet("""
            QPushButton { 
                background-color: #E74C3C; color: white; border-radius: 10px; 
            }
            QPushButton:pressed { 
                background-color: #C0392B; 
            }
        """)
        self.left_btn.pressed.connect(lambda: self.set_key_state('left', True))
        self.left_btn.released.connect(lambda: self.set_key_state('left', False))
        middle_layout.addWidget(self.left_btn)
        
        middle_layout.addStretch()
        
        self.right_btn = QPushButton("→")
        self.right_btn.setFont(QFont('Arial', 36, QFont.Bold))
        self.right_btn.setFixedSize(120, 80)
        self.right_btn.setStyleSheet("""
            QPushButton { 
                background-color: #E74C3C; color: white; border-radius: 10px; 
            }
            QPushButton:pressed { 
                background-color: #C0392B; 
            }
        """)
        self.right_btn.pressed.connect(lambda: self.set_key_state('right', True))
        self.right_btn.released.connect(lambda: self.set_key_state('right', False))
        middle_layout.addWidget(self.right_btn)
        arrow_layout.addLayout(middle_layout)
        
        # Backward button
        backward_layout = QHBoxLayout()
        backward_layout.addStretch()
        self.backward_btn = QPushButton("↓")
        self.backward_btn.setFont(QFont('Arial', 36, QFont.Bold))
        self.backward_btn.setFixedSize(120, 80)
        self.backward_btn.setStyleSheet("""
            QPushButton { 
                background-color: #3498DB; color: white; border-radius: 10px; 
            }
            QPushButton:pressed { 
                background-color: #2980B9; 
            }
        """)
        self.backward_btn.pressed.connect(lambda: self.set_key_state('backward', True))
        self.backward_btn.released.connect(lambda: self.set_key_state('backward', False))
        backward_layout.addWidget(self.backward_btn)
        backward_layout.addStretch()
        arrow_layout.addLayout(backward_layout)
        
        keyboard_layout.addLayout(arrow_layout)
        
        # Speed controls
        speed_layout = QVBoxLayout()
        speed_layout.addWidget(QLabel('Speed Settings:'))
        
        # Linear speed
        lin_speed_layout = QHBoxLayout()
        lin_speed_layout.addWidget(QLabel('Linear Speed:'))
        self.linear_speed_slider = QSlider(Qt.Horizontal)
        self.linear_speed_slider.setRange(10, 100)
        self.linear_speed_slider.setValue(50)
        self.linear_speed_slider.valueChanged.connect(self.update_speed_labels)
        lin_speed_layout.addWidget(self.linear_speed_slider)
        self.linear_speed_label = QLabel('0.50 m/s')
        self.linear_speed_label.setMinimumWidth(80)
        lin_speed_layout.addWidget(self.linear_speed_label)
        speed_layout.addLayout(lin_speed_layout)
        
        # Angular speed
        ang_speed_layout = QHBoxLayout()
        ang_speed_layout.addWidget(QLabel('Angular Speed:'))
        self.angular_speed_slider = QSlider(Qt.Horizontal)
        self.angular_speed_slider.setRange(10, 100)
        self.angular_speed_slider.setValue(50)
        self.angular_speed_slider.valueChanged.connect(self.update_speed_labels)
        ang_speed_layout.addWidget(self.angular_speed_slider)
        self.angular_speed_label = QLabel('1.00 rad/s')
        self.angular_speed_label.setMinimumWidth(80)
        ang_speed_layout.addWidget(self.angular_speed_label)
        speed_layout.addLayout(ang_speed_layout)
        
        keyboard_layout.addLayout(speed_layout)
        layout.addWidget(keyboard_group)
        
        # Emergency stop
        stop_layout = QHBoxLayout()
        stop_layout.addStretch()
        self.emergency_stop_btn = QPushButton("EMERGENCY STOP")
        self.emergency_stop_btn.setFont(QFont('Arial', 14, QFont.Bold))
        self.emergency_stop_btn.setFixedSize(220, 80)
        self.emergency_stop_btn.setStyleSheet("""
            QPushButton { 
                background-color: #FF0000; color: white; border-radius: 10px;
                border: 3px solid #CC0000; text-align: center;
            }
            QPushButton:pressed { 
                background-color: #CC0000; 
            }
        """)
        self.emergency_stop_btn.clicked.connect(self.emergency_stop)
        stop_layout.addWidget(self.emergency_stop_btn)
        stop_layout.addStretch()
        layout.addLayout(stop_layout)
        
        # Monitoring section
        monitor_group = QGroupBox("📊 Current Velocities")
        monitor_layout = QHBoxLayout(monitor_group)
        
        self.linear_x_label = QLabel('Linear: 0.000 m/s')
        self.linear_x_label.setFont(QFont('Arial', 11, QFont.Bold))
        self.linear_x_label.setStyleSheet("QLabel { color: #2E86C1; background-color: #EBF5FB; padding: 8px; border-radius: 5px; }")
        monitor_layout.addWidget(self.linear_x_label)
        
        self.angular_z_label = QLabel('Angular: 0.000 rad/s')
        self.angular_z_label.setFont(QFont('Arial', 11, QFont.Bold))
        self.angular_z_label.setStyleSheet("QLabel { color: #D35400; background-color: #FDF2E9; padding: 8px; border-radius: 5px; }")
        monitor_layout.addWidget(self.angular_z_label)
        
        self.status_label = QLabel('Status: Stopped')
        self.status_label.setFont(QFont('Arial', 11))
        self.status_label.setStyleSheet("QLabel { color: #E74C3C; font-style: italic; padding: 8px; }")
        monitor_layout.addWidget(self.status_label)
        
        layout.addWidget(monitor_group)
        
        # Update speed labels initially
        self.update_speed_labels()

    def init_ros(self):
        """Initialize ROS2 in a separate thread"""
        def ros_spin():
            rclpy.init()
            self.node = CmdVelNode(self.signals)
            try:
                rclpy.spin(self.node)
            except Exception as e:
                print(f"ROS spinning error: {e}")
            finally:
                if self.node:
                    self.node.destroy_node()
                rclpy.shutdown()
        
        self.ros_thread = threading.Thread(target=ros_spin, daemon=True)
        self.ros_thread.start()

    def send_drone_goal(self):
        """Send 3D goal to drone"""
        try:
            x = float(self.drone_x_input.text())
            y = float(self.drone_y_input.text())
            z = float(self.drone_z_input.text())
            yaw = float(self.drone_yaw_input.text())
            
            if self.node:
                self.node.publish_drone_goal(x, y, z, yaw)
            else:
                QMessageBox.warning(self, "Error", "ROS node not initialized")
                
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid numbers for X, Y, Z, and Yaw")

    def send_codes_bool(self):
        """Send True to /CODES/parrot/active topic"""
        if self.node:
            self.node.publish_codes_bool()
        else:
            QMessageBox.warning(self, "Error", "ROS node not initialized")

    def update_drone_status(self, message):
        """Update drone status display"""
        self.drone_status_label.setText(f'Drone: {message}')
        self.drone_status_label.setStyleSheet("""
            QLabel { 
                background-color: #E8DAEF; 
                padding: 8px; 
                border-radius: 5px;
                color: #6C3483;
                font-weight: bold;
            }
        """)

    def update_codes_bool_status(self, message):
        """Update CODES bool status display"""
        self.drone_status_label.setText(f'CODES: {message}')
        self.drone_status_label.setStyleSheet("""
            QLabel { 
                background-color: #D1F2EB; 
                padding: 8px; 
                border-radius: 5px;
                color: #117A65;
                font-weight: bold;
            }
        """)

    def send_nav_goal(self):
        """Send navigation goal from GUI inputs"""
        try:
            x = float(self.goal_x_input.text())
            y = float(self.goal_y_input.text())
            yaw = float(self.goal_yaw_input.text())
            
            if self.node:
                self.node.send_nav_goal(x, y, yaw)
            else:
                QMessageBox.warning(self, "Error", "ROS node not initialized")
                
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid numbers for X, Y, and Yaw")

    def cancel_nav_goal(self):
        """Cancel current navigation goal"""
        if self.node:
            if not self.node.cancel_nav_goal():
                QMessageBox.information(self, "Info", "No active goal to cancel")
        else:
            QMessageBox.warning(self, "Error", "ROS node not initialized")

    def update_goal_status(self, status, message):
        """Update the goal status display"""
        status_colors = {
            'sending': '#3498DB',
            'accepted': '#F39C12',
            'succeeded': '#27AE60',
            'aborted': '#E74C3C',
            'canceled': '#95A5A6',
            'error': '#C0392B',
            'canceling': '#E67E22',
            'rejected': '#E74C3C',
            'unknown': '#7F8C8D'
        }
        
        color = status_colors.get(status, '#566573')
        self.goal_status_label.setText(f'Nav2 Status: {message}')
        self.goal_status_label.setStyleSheet(f"""
            QLabel {{ 
                background-color: #F8F9F9; 
                padding: 8px; 
                border-radius: 5px;
                color: {color};
                font-weight: bold;
            }}
        """)

    def set_key_state(self, direction, pressed):
        """Set the state of directional keys"""
        if direction == 'forward':
            self.key_forward = pressed
        elif direction == 'backward':
            self.key_backward = pressed
        elif direction == 'left':
            self.key_left = pressed
        elif direction == 'right':
            self.key_right = pressed

    def update_speed_labels(self):
        """Update speed setting labels"""
        linear_speed = self.linear_speed_slider.value() / 100.0
        angular_speed = self.angular_speed_slider.value() / 50.0
        
        self.linear_speed_label.setText(f'{linear_speed:.2f} m/s')
        self.angular_speed_label.setText(f'{angular_speed:.2f} rad/s')

    def send_movement_command(self):
        """Calculate and send movement command based on current key states"""
        if not self.node:
            return
            
        linear_speed = self.linear_speed_slider.value() / 100.0
        angular_speed = self.angular_speed_slider.value() / 50.0
        
        linear_x = 0.0
        if self.key_forward:
            linear_x += linear_speed
        if self.key_backward:
            linear_x -= linear_speed
            
        angular_z = 0.0
        if self.key_left:
            angular_z += angular_speed
        if self.key_right:
            angular_z -= angular_speed
            
        self.node.publish_velocity(linear_x, 0.0, angular_z)

    def send_continuous_commands(self):
        """Send commands continuously in real-time"""
        self.send_movement_command()

    def emergency_stop(self):
        """Emergency stop - immediately stop robot and reset all states"""
        if self.node:
            self.node.publish_velocity(0.0, 0.0, 0.0)
            self.node.cancel_nav_goal()
        
        self.key_forward = False
        self.key_backward = False
        self.key_left = False
        self.key_right = False
        
        print("EMERGENCY STOP activated")

    def update_velocity_display(self, linear_x, linear_y, angular_z):
        """Update the GUI with new velocity values from topic"""
        self.current_linear_x = linear_x
        self.current_linear_y = linear_y
        self.current_angular_z = angular_z
        
        self.linear_x_label.setText(f'Linear: {linear_x:+.3f} m/s')
        self.angular_z_label.setText(f'Angular: {angular_z:+.3f} rad/s')
        
        velocity_magnitude = (linear_x**2 + linear_y**2)**0.5
        
        if abs(linear_x) > 0.01 or abs(linear_y) > 0.01 or abs(angular_z) > 0.01:
            if velocity_magnitude > 0.01:
                self.status_label.setText(f'Status: Moving ({velocity_magnitude:.2f} m/s)')
            else:
                self.status_label.setText('Status: Rotating')
            self.status_label.setStyleSheet("QLabel { color: #27AE60; font-style: italic; font-weight: bold; padding: 8px; }")
        else:
            self.status_label.setText('Status: Stopped')
            self.status_label.setStyleSheet("QLabel { color: #E74C3C; font-style: italic; font-weight: bold; padding: 8px; }")

    def keyPressEvent(self, event):
        """Handle keyboard input for WASD control"""
        if event.key() == Qt.Key_W:
            self.set_key_state('forward', True)
        elif event.key() == Qt.Key_S:
            self.set_key_state('backward', True)
        elif event.key() == Qt.Key_A:
            self.set_key_state('left', True)
        elif event.key() == Qt.Key_D:
            self.set_key_state('right', True)
        elif event.key() == Qt.Key_Space:
            self.emergency_stop()

    def keyReleaseEvent(self, event):
        """Handle keyboard release for WASD control"""
        if event.key() == Qt.Key_W:
            self.set_key_state('forward', False)
        elif event.key() == Qt.Key_S:
            self.set_key_state('backward', False)
        elif event.key() == Qt.Key_A:
            self.set_key_state('left', False)
        elif event.key() == Qt.Key_D:
            self.set_key_state('right', False)

    def closeEvent(self, event):
        """Clean up when closing the application"""
        if self.node:
            self.node.publish_velocity(0.0, 0.0, 0.0)
            self.node.destroy_node()
        if self.ros_thread and self.ros_thread.is_alive():
            rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = CmdVelGUI()
    window.show()
    
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("\nShutting down...")
        rclpy.shutdown()

if __name__ == '__main__':
    main()