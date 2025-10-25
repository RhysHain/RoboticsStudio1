#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QWidget, QLabel, QPushButton, QGroupBox,
                             QLineEdit, QGridLayout, QMessageBox)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, Qt
from PyQt5.QtGui import QFont
import threading
import math

class ParrotSignals(QObject):
    """Signals for thread-safe GUI updates"""
    goal_sent = pyqtSignal(str)
    command_sent = pyqtSignal(str)
    position_updated = pyqtSignal(float, float, float, float)  # x, y, z, yaw

class ParrotNode(Node):
    def __init__(self, signals):
        super().__init__('parrot_gui_node')
        self.signals = signals

        # Publisher for drone 3D goals
        self.drone_goal_publisher = self.create_publisher(
            Pose,
            '/CODES/parrot/goals',
            10
        )

        # Publisher for /CODES/parrot/active
        self.codes_active_publisher = self.create_publisher(
            Bool,
            '/CODES/parrot/active',
            10
        )

        # Publisher for /CODES/parrot/start_search
        self.codes_start_search_publisher = self.create_publisher(
            Bool,
            '/CODES/parrot/start_search',
            10
        )

        # Subscribe to parrot odometry for position
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/parrot/odometry',
            self.odom_callback,
            10)

        self.get_logger().info('Parrot Drone GUI node started')

    def odom_callback(self, msg):
        """Extract position from odometry and emit signal"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.signals.position_updated.emit(x, y, z, yaw)

    def publish_drone_goal(self, x, y, z, yaw=0.0):
        """Publish 3D goal for drone to /CODES/parrot/goals"""
        goal_msg = Pose()

        goal_msg.position.x = x
        goal_msg.position.y = y
        goal_msg.position.z = z

        goal_msg.orientation.x = 0.0
        goal_msg.orientation.y = 0.0
        goal_msg.orientation.z = math.sin(yaw / 2.0)
        goal_msg.orientation.w = math.cos(yaw / 2.0)

        self.drone_goal_publisher.publish(goal_msg)
        self.get_logger().info(f'Drone goal published: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}')

        self.signals.goal_sent.emit(f'Goal sent: ({x:.2f}, {y:.2f}, {z:.2f})')

    def publish_active(self):
        """Publish True to /CODES/parrot/active topic"""
        msg = Bool()
        msg.data = True
        self.codes_active_publisher.publish(msg)
        self.get_logger().info('Published True to /CODES/parrot/active')
        self.signals.command_sent.emit('Drone activated')

    def publish_start_search(self):
        """Publish True to /CODES/parrot/start_search topic"""
        msg = Bool()
        msg.data = True
        self.codes_start_search_publisher.publish(msg)
        self.get_logger().info('Published True to /CODES/parrot/start_search')
        self.signals.command_sent.emit('Search started')

class ParrotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('🚁 Parrot Drone Control Center')
        self.setGeometry(100, 100, 700, 650)

        self.signals = ParrotSignals()
        self.signals.goal_sent.connect(self.update_goal_status)
        self.signals.command_sent.connect(self.update_command_status)
        self.signals.position_updated.connect(self.update_position_display)

        self.ros_thread = None
        self.node = None
        self.init_ros()

        self.init_ui()

        # Position tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Title
        title = QLabel('🚁 Parrot Drone Control Center')
        title.setFont(QFont('Arial', 18, QFont.Bold))
        title.setStyleSheet("QLabel { color: #8E44AD; margin: 15px; }")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # ========== POSITION DISPLAY ==========
        pos_group = QGroupBox("📍 Current Position")
        pos_layout = QGridLayout(pos_group)

        self.pos_x_label = QLabel('X: 0.000 m')
        self.pos_x_label.setFont(QFont('Arial', 12, QFont.Bold))
        self.pos_x_label.setStyleSheet("QLabel { color: #7D3C98; background-color: #F4ECF7; padding: 10px; border-radius: 5px; }")
        pos_layout.addWidget(self.pos_x_label, 0, 0)

        self.pos_y_label = QLabel('Y: 0.000 m')
        self.pos_y_label.setFont(QFont('Arial', 12, QFont.Bold))
        self.pos_y_label.setStyleSheet("QLabel { color: #7D3C98; background-color: #F4ECF7; padding: 10px; border-radius: 5px; }")
        pos_layout.addWidget(self.pos_y_label, 0, 1)

        self.pos_z_label = QLabel('Z: 0.000 m')
        self.pos_z_label.setFont(QFont('Arial', 12, QFont.Bold))
        self.pos_z_label.setStyleSheet("QLabel { color: #148F77; background-color: #E8F8F5; padding: 10px; border-radius: 5px; }")
        pos_layout.addWidget(self.pos_z_label, 1, 0)

        self.pos_yaw_label = QLabel('Yaw: 0.00 rad')
        self.pos_yaw_label.setFont(QFont('Arial', 12, QFont.Bold))
        self.pos_yaw_label.setStyleSheet("QLabel { color: #148F77; background-color: #E8F8F5; padding: 10px; border-radius: 5px; }")
        pos_layout.addWidget(self.pos_yaw_label, 1, 1)

        layout.addWidget(pos_group)

        # ========== DRONE 3D GOAL SECTION ==========
        drone_group = QGroupBox("🎯 Drone 3D Goal Control")
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

        # Goal status display
        self.goal_status_label = QLabel('Status: Ready to send goal')
        self.goal_status_label.setFont(QFont('Arial', 10))
        self.goal_status_label.setStyleSheet("""
            QLabel {
                background-color: #F4ECF7;
                padding: 8px;
                border-radius: 5px;
                color: #6C3483;
            }
        """)
        drone_layout.addWidget(self.goal_status_label)

        layout.addWidget(drone_group)

        # ========== DRONE COMMANDS SECTION ==========
        cmd_group = QGroupBox("⚡ Drone Commands")
        cmd_layout = QVBoxLayout(cmd_group)

        # Activate Drone button
        self.activate_btn = QPushButton("Activate Drone (/CODES/parrot/active)")
        self.activate_btn.setFont(QFont('Arial', 11, QFont.Bold))
        self.activate_btn.setStyleSheet("""
            QPushButton {
                background-color: #16A085; color: white;
                border-radius: 5px; padding: 10px;
            }
            QPushButton:pressed {
                background-color: #138D75;
            }
        """)
        self.activate_btn.clicked.connect(self.activate_drone)
        cmd_layout.addWidget(self.activate_btn)

        # Start Search button
        self.start_search_btn = QPushButton("Start Search (/CODES/parrot/start_search)")
        self.start_search_btn.setFont(QFont('Arial', 11, QFont.Bold))
        self.start_search_btn.setStyleSheet("""
            QPushButton {
                background-color: #2874A6; color: white;
                border-radius: 5px; padding: 10px;
            }
            QPushButton:pressed {
                background-color: #1B4F72;
            }
        """)
        self.start_search_btn.clicked.connect(self.start_search)
        cmd_layout.addWidget(self.start_search_btn)

        # Command status display
        self.command_status_label = QLabel('Command Status: Ready')
        self.command_status_label.setFont(QFont('Arial', 10))
        self.command_status_label.setStyleSheet("""
            QLabel {
                background-color: #D1F2EB;
                padding: 8px;
                border-radius: 5px;
                color: #117A65;
            }
        """)
        cmd_layout.addWidget(self.command_status_label)

        layout.addWidget(cmd_group)

        # ========== INFO SECTION ==========
        info_group = QGroupBox("ℹ️ Topic Information")
        info_layout = QVBoxLayout(info_group)

        info_text = QLabel(
            "• Goals: /CODES/parrot/goals (Pose)\n"
            "• Active: /CODES/parrot/active (Bool)\n"
            "• Search: /CODES/parrot/start_search (Bool)\n"
            "• Odometry: /parrot/odometry (Odometry)"
        )
        info_text.setFont(QFont('Arial', 9))
        info_text.setStyleSheet("QLabel { color: #566573; padding: 5px; }")
        info_layout.addWidget(info_text)

        layout.addWidget(info_group)

        # Add stretch to push everything to the top
        layout.addStretch()

    def init_ros(self):
        def ros_spin():
            rclpy.init()
            self.node = ParrotNode(self.signals)
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

    def update_position_display(self, x, y, z, yaw):
        """Update position display with current drone pose"""
        self.current_x = x
        self.current_y = y
        self.current_z = z
        self.current_yaw = yaw

        self.pos_x_label.setText(f'X: {x:+.3f} m')
        self.pos_y_label.setText(f'Y: {y:+.3f} m')
        self.pos_z_label.setText(f'Z: {z:+.3f} m (Altitude)')
        self.pos_yaw_label.setText(f'Yaw: {yaw:+.2f} rad ({math.degrees(yaw):+.1f}°)')

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

    def activate_drone(self):
        """Send True to /CODES/parrot/active topic"""
        if self.node:
            self.node.publish_active()
        else:
            QMessageBox.warning(self, "Error", "ROS node not initialized")

    def start_search(self):
        """Send True to /CODES/parrot/start_search topic"""
        if self.node:
            self.node.publish_start_search()
        else:
            QMessageBox.warning(self, "Error", "ROS node not initialized")

    def update_goal_status(self, message):
        """Update drone goal status display"""
        self.goal_status_label.setText(f'Goal: {message}')
        self.goal_status_label.setStyleSheet("""
            QLabel {
                background-color: #E8DAEF;
                padding: 8px;
                border-radius: 5px;
                color: #6C3483;
                font-weight: bold;
            }
        """)

    def update_command_status(self, message):
        """Update command status display"""
        self.command_status_label.setText(f'Command: {message}')
        self.command_status_label.setStyleSheet("""
            QLabel {
                background-color: #D1F2EB;
                padding: 8px;
                border-radius: 5px;
                color: #117A65;
                font-weight: bold;
            }
        """)

    def closeEvent(self, event):
        if self.node:
            self.node.destroy_node()
        if self.ros_thread and self.ros_thread.is_alive():
            rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = ParrotGUI()
    window.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("\nShutting down...")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
