#!/usr/bin/env python3
"""
Xbox Controller Return to Base for Leo Rover
- RB: Return to home (default: map origin 0,0)
- Left Stick Click: Set current location as home
- Left Stick Y-axis: Linear motion (forward/backward) only
- Right Stick X-axis: Angular motion (rotation) only
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class XboxReturnToBase(Node):
    def __init__(self):
        super().__init__('xbox_return_to_base')
        
        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publisher for navigation goals
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # TF2 for getting current position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Button state tracking (to detect press, not hold)
        self.rb_pressed = False
        self.left_stick_click_pressed = False
        
        # Home position (default: map origin)
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw = 0.0
        self.home_is_custom = False
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Xbox Return to Base - Controls:')
        self.get_logger().info('  Left Stick (Y-axis): Forward/Backward')
        self.get_logger().info('  Right Stick (X-axis): Rotate Left/Right')
        self.get_logger().info('  LB (hold): Enable movement (deadman switch)')
        self.get_logger().info('  RB (press): Return to home')
        self.get_logger().info('  Left Stick Click: Set current location as home')
        self.get_logger().info(f'  Current home: ({self.home_x:.2f}, {self.home_y:.2f})')
        self.get_logger().info('=' * 60)
    
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
    
    def joy_callback(self, msg):
        """Handle Xbox controller button presses"""
        
        # Button indices for Xbox controller
        rb_button = msg.buttons[5] if len(msg.buttons) > 5 else 0  # RB
        left_stick_click = msg.buttons[9] if len(msg.buttons) > 9 else 0  # Left stick click
        
        # Left Stick Click: Set current location as home
        if left_stick_click == 1 and not self.left_stick_click_pressed:
            self.set_home_to_current()
            self.left_stick_click_pressed = True
        elif left_stick_click == 0:
            self.left_stick_click_pressed = False
        
        # RB: Return to home
        if rb_button == 1 and not self.rb_pressed:
            self.return_to_home()
            self.rb_pressed = True
        elif rb_button == 0:
            self.rb_pressed = False
    
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

def main(args=None):
    rclpy.init(args=args)
    node = XboxReturnToBase()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
