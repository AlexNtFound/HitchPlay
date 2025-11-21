#!/usr/bin/env python3
"""
Xbox Controller Teleop + Navigation for Leo Rover (ROS 2 Jazzy)
Works on both Windows and Ubuntu
Tested with Xbox One/Series wired controller

Controls:
- Right Stick Y-axis: Forward/Backward (linear motion)
- Left Stick X-axis: Rotate Left/Right (angular motion)
- Nexus/Guide Button: Toggle E-Stop
- Start Button: Release E-Stop
- RB Button: Return to home
- Left Stick Click: Set current location as home
- A Button: Cancel/Stop active navigation
- B Button: Reset home to default (map origin)

PC-side Python dependencies:
Ubuntu: ROS2 Jazzy + pip install pygame
Windows: ROS2 Jazzy via pixi + pip install pygame
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
E_STOP_TOPIC  = "/e_stop"
GOAL_POSE_TOPIC = "/goal_pose"

# Speed limits
MAX_LINEAR_SPEED  = 1.0   # m/s
MAX_ANGULAR_SPEED = 1.5   # rad/s

# Deadzone for sticks
STICK_DEADZONE = 0.15

# Button mappings (pygame indices for Xbox controller)
NEXUS_BUTTON = 8   # Xbox Guide/Nexus button - E-Stop toggle
START_BUTTON = 7   # Start button - Release E-Stop
RB_BUTTON = 5      # RB - Return to home
LEFT_STICK_CLICK = 9  # Left stick click - Set home to current
A_BUTTON = 0       # A button - Cancel navigation
B_BUTTON = 1       # B button - Reset home to default

# Axis mappings
LEFT_STICK_X = 0   # Steering (angular)
RIGHT_STICK_Y = 3  # Throttle (linear) - Note: inverted by default
# -----------------------------------------------------------

class XboxLeoTeleopNav(Node):
    def __init__(self):
        super().__init__('xbox_leo_teleop_nav')

        # ROS 2 publishers
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_POSE_TOPIC, 10)

        # E-stop publisher with "latched" behavior (transient local)
        e_stop_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.e_stop_pub = self.create_publisher(Bool, E_STOP_TOPIC, e_stop_qos)

        # TF2 for getting current position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Xbox controller found! Plug it in and try again.")
            sys.exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Connected to: {self.joystick.get_name()}")

        # E-Stop state
        self.e_stop_active = False
        self.publish_e_stop(False)  # initial safe state

        # Button state tracking (to detect press, not hold)
        self.button_states = {
            NEXUS_BUTTON: False,
            START_BUTTON: False,
            RB_BUTTON: False,
            LEFT_STICK_CLICK: False,
            A_BUTTON: False,
            B_BUTTON: False,
        }

        # Home position (default: map origin)
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw = 0.0
        self.home_is_custom = False

        # Timer to publish at ~50 Hz
        self.timer = self.create_timer(0.02, self.teleop_loop)

        # Graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

        # Print startup info
        self.print_startup_info()

    def print_startup_info(self):
        """Print control scheme and current settings"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Xbox Leo Rover Teleop + Navigation')
        self.get_logger().info('=' * 60)
        self.get_logger().info('TELEOP CONTROLS:')
        self.get_logger().info('  Right Stick Y-axis: Forward/Backward')
        self.get_logger().info('  Left Stick X-axis: Rotate Left/Right')
        self.get_logger().info('  Nexus/Guide Button: Toggle E-Stop')
        self.get_logger().info('  Start Button: Release E-Stop')
        self.get_logger().info('')
        self.get_logger().info('NAVIGATION CONTROLS:')
        self.get_logger().info('  RB Button: Return to home')
        self.get_logger().info('  Left Stick Click: Set current location as home')
        self.get_logger().info('  A Button: Cancel/Stop active navigation')
        self.get_logger().info('  B Button: Reset home to default (map origin)')
        self.get_logger().info('')
        self.get_logger().info(f'Current home: ({self.home_x:.2f}, {self.home_y:.2f}, {math.degrees(self.home_yaw):.1f}°)')
        self.get_logger().info('=' * 60)

    def publish_e_stop(self, state: bool):
        """Publish E-Stop state"""
        if state != self.e_stop_active:
            msg = Bool()
            msg.data = state
            self.e_stop_pub.publish(msg)
            self.e_stop_active = state
            status = "ENGAGED" if state else "RELEASED"
            self.get_logger().warn(f"E-STOP {status}")

    def get_current_position(self):
        """Get current robot position in map frame using TF2"""
        try:
            # Look up transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
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
            self.get_logger().warn(f'Could not get current position: {ex}')
            return None, None, None

    def set_home_to_current(self):
        """Set home position to current robot location"""
        x, y, yaw = self.get_current_position()
        
        if x is None:
            self.get_logger().error('Cannot set home - unable to get current position!')
            return
        
        self.home_x = x
        self.home_y = y
        self.home_yaw = yaw
        self.home_is_custom = True
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'✓ Home position updated to current location:')
        self.get_logger().info(f'  X: {self.home_x:.2f} m')
        self.get_logger().info(f'  Y: {self.home_y:.2f} m')
        self.get_logger().info(f'  Yaw: {math.degrees(self.home_yaw):.1f}°')
        self.get_logger().info('=' * 60)

    def reset_home_to_default(self):
        """Reset home position to map origin (0, 0, 0)"""
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw = 0.0
        self.home_is_custom = False
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🔄 Home position RESET to default (map origin):')
        self.get_logger().info(f'  X: {self.home_x:.2f} m')
        self.get_logger().info(f'  Y: {self.home_y:.2f} m')
        self.get_logger().info(f'  Yaw: {math.degrees(self.home_yaw):.1f}°')
        self.get_logger().info('=' * 60)

    def return_to_home(self):
        """Send navigation goal to return to home position"""
        
        # Create goal message
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal.pose.position.x = self.home_x
        goal.pose.position.y = self.home_y
        goal.pose.position.z = 0.0
        
        # Set orientation (yaw to quaternion)
        goal.pose.orientation.z = math.sin(self.home_yaw / 2.0)
        goal.pose.orientation.w = math.cos(self.home_yaw / 2.0)
        
        # Publish goal
        self.goal_pub.publish(goal)
        
        home_type = "custom location" if self.home_is_custom else "map origin"
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'🏠 Returning to home ({home_type}):')
        self.get_logger().info(f'  X: {self.home_x:.2f} m')
        self.get_logger().info(f'  Y: {self.home_y:.2f} m')
        self.get_logger().info(f'  Yaw: {math.degrees(self.home_yaw):.1f}°')
        self.get_logger().info('=' * 60)

    def cancel_navigation(self):
        """Cancel active navigation by sending current position as goal"""
        x, y, yaw = self.get_current_position()
        
        if x is None:
            self.get_logger().warn('Cannot cancel - unable to get position')
            return
        
        # Send current position as the goal (effectively "stay here")
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.goal_pub.publish(goal)
        
        self.get_logger().info('🛑 Navigation CANCELLED - Robot will stop at current location')

    def handle_button_press(self, button):
        """Handle button press events (detect press, not hold)"""
        current_state = self.joystick.get_button(button)
        
        # Detect rising edge (button just pressed)
        if current_state == 1 and not self.button_states[button]:
            self.button_states[button] = True
            return True
        elif current_state == 0:
            self.button_states[button] = False
        
        return False

    def teleop_loop(self):
        """Main control loop running at ~50 Hz"""
        # Process pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

        # Handle button presses
        if self.handle_button_press(NEXUS_BUTTON):
            self.publish_e_stop(not self.e_stop_active)  # toggle
        
        if self.handle_button_press(START_BUTTON):
            self.publish_e_stop(False)
        
        if self.handle_button_press(LEFT_STICK_CLICK):
            self.set_home_to_current()
        
        if self.handle_button_press(RB_BUTTON):
            self.return_to_home()
        
        if self.handle_button_press(A_BUTTON):
            self.cancel_navigation()
        
        if self.handle_button_press(B_BUTTON):
            self.reset_home_to_default()

        # If E-Stop is active, send zero velocity and return
        if self.e_stop_active:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return

        # --- Read stick axes ---
        # Right stick Y (axis 3): -1.0 = up (forward), +1.0 = down (backward)
        # Left stick X (axis 0): -1.0 = left, +1.0 = right
        throttle = -self.joystick.get_axis(RIGHT_STICK_Y)  # Invert so up is positive
        steer = -self.joystick.get_axis(LEFT_STICK_X)      # Invert so left is positive rotation

        # Apply deadzones
        if abs(throttle) < STICK_DEADZONE:
            throttle = 0.0
        if abs(steer) < STICK_DEADZONE:
            steer = 0.0

        # Scale to max speeds
        linear_x = throttle * MAX_LINEAR_SPEED
        angular_z = steer * MAX_ANGULAR_SPEED

        # Publish velocity command
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(angular_z)

        self.cmd_vel_pub.publish(twist)

        # Optional: print debug info (uncomment if needed)
        # self.get_logger().debug(f"Linear: {linear_x:.2f} m/s   Angular: {angular_z:.2f} rad/s")

    def signal_handler(self, sig, frame):
        """Graceful shutdown handler"""
        self.get_logger().info("Shutting down – sending zero velocity and releasing E-Stop")
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.publish_e_stop(False)
        pygame.quit()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    # Set ROS domain ID (adjust as needed for your rover)
    os.environ['ROS_DOMAIN_ID'] = '0'
    
    # Uncomment if you need to force specific middleware for network issues
    # os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'  # Better for WiFi

    rclpy.init(args=args)

    node = XboxLeoTeleopNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.signal_handler(None, None)


if __name__ == '__main__':
    main()
