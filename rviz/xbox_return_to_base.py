#!/usr/bin/env python3
"""
Xbox Controller Full Teleop + Navigation for Leo Rover (ROS 2 Jazzy)
Cross-platform: Windows 11 or Ubuntu Linux
Tested with Xbox One/Series wired controller

CONTROLS:
  Manual Driving:
    - Left Trigger: Forward throttle
    - Right Trigger: Backward throttle
    - Left Stick X: Steering (left/right)
    - LB (hold): Deadman switch - must hold to enable movement
  
  Navigation:
    - RB: Return to home position
    - Left Stick Click: Set current location as home
    - A Button: Cancel/Stop active navigation
    - B Button: Reset home to default (map origin 0,0)
  
  Safety:
    - Xbox Button (center): Toggle E-Stop
    - Start Button: Release E-Stop

INSTALLATION:
  Ubuntu: pip install pygame
  Windows: Follow ROS2 Jazzy + pixi setup, then pip install pygame
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import pygame
import math
import sys
import signal
import os

# ---------------------- Configuration ----------------------
CMD_VEL_TOPIC = "/cmd_vel"
E_STOP_TOPIC = "/e_stop"
GOAL_POSE_TOPIC = "/goal_pose"

# Speed limits
MAX_LINEAR_SPEED = 1.0   # m/s
MAX_ANGULAR_SPEED = 1.5  # rad/s

# Deadzones
TRIGGER_DEADZONE = 0.1
STICK_DEADZONE = 0.15

# Button mappings (Xbox controller via pygame)
BTN_A = 0
BTN_B = 1
BTN_X = 2
BTN_Y = 3
BTN_LB = 4
BTN_RB = 5
BTN_BACK = 6
BTN_START = 7
BTN_XBOX = 8
BTN_LEFT_STICK = 9
BTN_RIGHT_STICK = 10

# Axis mappings
AXIS_LEFT_STICK_X = 0
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 2
AXIS_RIGHT_STICK_Y = 3
AXIS_LEFT_TRIGGER = 4
AXIS_RIGHT_TRIGGER = 5

# -----------------------------------------------------------

class XboxLeoTeleopFull(Node):
    def __init__(self):
        super().__init__('xbox_leo_teleop_full')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_POSE_TOPIC, 10)
        
        # E-stop with transient local QoS
        e_stop_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.e_stop_pub = self.create_publisher(Bool, E_STOP_TOPIC, e_stop_qos)
        
        # TF2 for position tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize pygame
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Xbox controller found! Connect it and try again.")
            sys.exit(1)
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Connected to: {self.joystick.get_name()}")
        
        # State tracking
        self.e_stop_active = False
        self.btn_state = {}  # Track button press states to detect edges
        
        # Home position (default: map origin)
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw = 0.0
        self.home_is_custom = False
        
        # Initialize E-Stop to safe state
        self.publish_e_stop(False)
        
        # Main control loop at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)
        
        # Graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.print_startup_info()
    
    def print_startup_info(self):
        """Print control scheme to console"""
        self.get_logger().info('=' * 70)
        self.get_logger().info('Xbox Leo Rover Full Control - Ready!')
        self.get_logger().info('=' * 70)
        self.get_logger().info('MANUAL DRIVING:')
        self.get_logger().info('  Left Trigger:     Forward throttle')
        self.get_logger().info('  Right Trigger:    Backward throttle')
        self.get_logger().info('  Left Stick X:     Steering (left/right)')
        self.get_logger().info('  LB (hold):        Deadman switch - MUST HOLD to move')
        self.get_logger().info('')
        self.get_logger().info('NAVIGATION:')
        self.get_logger().info('  RB:               Return to home')
        self.get_logger().info('  Left Stick Click: Set current location as home')
        self.get_logger().info('  A Button:         Cancel/Stop navigation')
        self.get_logger().info('  B Button:         Reset home to map origin (0,0)')
        self.get_logger().info('')
        self.get_logger().info('SAFETY:')
        self.get_logger().info('  Xbox Button:      Toggle E-Stop')
        self.get_logger().info('  Start Button:     Release E-Stop')
        self.get_logger().info('')
        self.get_logger().info(f'Home position: ({self.home_x:.2f}, {self.home_y:.2f}) - map origin')
        self.get_logger().info('=' * 70)
    
    def publish_e_stop(self, state: bool):
        """Publish E-Stop state"""
        if state != self.e_stop_active:
            msg = Bool()
            msg.data = state
            self.e_stop_pub.publish(msg)
            self.e_stop_active = state
            status = "ENGAGED" if state else "RELEASED"
            self.get_logger().warn(f"🚨 E-STOP {status}")
    
    def get_current_position(self):
        """Get current robot position in map frame using TF2"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Convert quaternion to yaw
            quat = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )
            
            return x, y, yaw
            
        except TransformException as ex:
            self.get_logger().debug(f'TF lookup failed: {ex}')
            return None, None, None
    
    def button_pressed(self, btn_index):
        """Detect button press (rising edge only)"""
        current = self.joystick.get_button(btn_index)
        previous = self.btn_state.get(btn_index, 0)
        self.btn_state[btn_index] = current
        return current == 1 and previous == 0
    
    def set_home_to_current(self):
        """Set home position to current robot location"""
        x, y, yaw = self.get_current_position()
        
        if x is None:
            self.get_logger().error('❌ Cannot set home - unable to get current position!')
            self.get_logger().info('   Make sure navigation is running and map->base_link transform exists')
            return
        
        self.home_x = x
        self.home_y = y
        self.home_yaw = yaw
        self.home_is_custom = True
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ Home position SET to current location:')
        self.get_logger().info(f'  X: {self.home_x:.2f} m')
        self.get_logger().info(f'  Y: {self.home_y:.2f} m')
        self.get_logger().info(f'  Yaw: {math.degrees(self.home_yaw):.1f}°')
        self.get_logger().info('=' * 60)
    
    def reset_home_to_default(self):
        """Reset home position to map origin"""
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw = 0.0
        self.home_is_custom = False
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🔄 Home position RESET to map origin:')
        self.get_logger().info(f'  X: {self.home_x:.2f} m')
        self.get_logger().info(f'  Y: {self.home_y:.2f} m')
        self.get_logger().info('=' * 60)
    
    def return_to_home(self):
        """Send navigation goal to return to home position"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = self.home_x
        goal.pose.position.y = self.home_y
        goal.pose.position.z = 0.0
        
        # Yaw to quaternion
        goal.pose.orientation.z = math.sin(self.home_yaw / 2.0)
        goal.pose.orientation.w = math.cos(self.home_yaw / 2.0)
        
        self.goal_pub.publish(goal)
        
        home_type = "custom location" if self.home_is_custom else "map origin"
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'🏠 RETURNING to home ({home_type}):')
        self.get_logger().info(f'  Target: ({self.home_x:.2f}, {self.home_y:.2f})')
        self.get_logger().info(f'  Yaw: {math.degrees(self.home_yaw):.1f}°')
        self.get_logger().info('=' * 60)
    
    def cancel_navigation(self):
        """Cancel active navigation by sending current position as goal"""
        x, y, yaw = self.get_current_position()
        
        if x is None:
            self.get_logger().warn('⚠ Cannot cancel - unable to get position')
            return
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.goal_pub.publish(goal)
        
        self.get_logger().info('🛑 Navigation CANCELLED - stopping at current location')
    
    def control_loop(self):
        """Main control loop - handles all input and publishes commands"""
        # Process pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
        
        # --- Button handling (edge detection) ---
        
        # E-Stop controls
        if self.button_pressed(BTN_XBOX):
            self.publish_e_stop(not self.e_stop_active)  # Toggle
        if self.button_pressed(BTN_START):
            self.publish_e_stop(False)  # Release
        
        # Navigation controls
        if self.button_pressed(BTN_RB):
            self.return_to_home()
        if self.button_pressed(BTN_LEFT_STICK):
            self.set_home_to_current()
        if self.button_pressed(BTN_A):
            self.cancel_navigation()
        if self.button_pressed(BTN_B):
            self.reset_home_to_default()
        
        # --- Manual driving with deadman switch ---
        
        # LB must be held for manual control
        deadman_active = self.joystick.get_button(BTN_LB)
        
        if self.e_stop_active or not deadman_active:
            # Force zero velocity
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # Read controller axes
        left_trigger = (self.joystick.get_axis(AXIS_LEFT_TRIGGER) + 1.0) / 2.0
        right_trigger = (self.joystick.get_axis(AXIS_RIGHT_TRIGGER) + 1.0) / 2.0
        steer = -self.joystick.get_axis(AXIS_LEFT_STICK_X)
        
        # Apply deadzones
        if left_trigger < TRIGGER_DEADZONE:
            left_trigger = 0.0
        if right_trigger < TRIGGER_DEADZONE:
            right_trigger = 0.0
        if abs(steer) < STICK_DEADZONE:
            steer = 0.0
        
        # Compute velocities
        linear_x = (left_trigger - right_trigger) * MAX_LINEAR_SPEED
        angular_z = steer * MAX_ANGULAR_SPEED
        
        # Publish command
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)
    
    def signal_handler(self, sig, frame):
        """Graceful shutdown"""
        self.get_logger().info("Shutting down - stopping robot and releasing E-Stop")
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.publish_e_stop(False)
        pygame.quit()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    # Set ROS domain (adjust for your network)
    os.environ.setdefault('ROS_DOMAIN_ID', '42')
    
    rclpy.init(args=args)
    
    node = XboxLeoTeleopFull()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.signal_handler(None, None)


if __name__ == '__main__':
    main()
