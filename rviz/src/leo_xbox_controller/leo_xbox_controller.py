#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped

class LeoXboxController(Node):
    def __init__(self):
        super().__init__('leo_xbox_controller')
        
        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Velocity settings
        self.max_linear = 0.5   # m/s
        self.max_angular = 1.0  # rad/s
        self.velocity_step = 0.1  # adjustment step
        
        # Button states (to detect press, not hold)
        self.rb_pressed = False
        self.dpad_up_pressed = False
        self.dpad_down_pressed = False
        
        # Home position (map frame)
        self.home_x = 0.0
        self.home_y = 0.0
        
        self.get_logger().info('Leo Xbox Controller Started!')
        self.get_logger().info('Controls:')
        self.get_logger().info('  LB (hold): Enable movement')
        self.get_logger().info('  Left Stick: Forward/Backward')
        self.get_logger().info('  Right Stick: Rotate Left/Right')
        self.get_logger().info('  RB: Return to home (0,0)')
        self.get_logger().info('  D-pad Up: Increase max speed')
        self.get_logger().info('  D-pad Down: Decrease max speed')
        self.get_logger().info(f'Current max speeds - Linear: {self.max_linear} m/s, Angular: {self.max_angular} rad/s')
    
    def joy_callback(self, msg):
        # Xbox Controller Mapping:
        # Axes: 0=LStick-LR, 1=LStick-UD, 2=LT, 3=RStick-LR, 4=RStick-UD, 5=RT
        # Buttons: 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB, 6=Back, 7=Start, etc.
        # D-pad: axes 6=LR, 7=UD (on some controllers it's buttons 13,14,15,16)
        
        # Check deadman switch (LB = button 4)
        deadman = msg.buttons[4] if len(msg.buttons) > 4 else 0
        
        if deadman:
            # Teleop mode
            twist = Twist()
            
            # Left stick vertical (axis 1) for linear velocity
            linear_axis = msg.axes[1] if len(msg.axes) > 1 else 0.0
            twist.linear.x = linear_axis * self.max_linear
            
            # Right stick horizontal (axis 3) for angular velocity
            angular_axis = msg.axes[3] if len(msg.axes) > 3 else 0.0
            twist.angular.z = angular_axis * self.max_angular
            
            self.cmd_vel_pub.publish(twist)
        else:
            # No deadman, send zero velocity
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
        
        # RB button (button 5) - Return to home
        rb_button = msg.buttons[5] if len(msg.buttons) > 5 else 0
        if rb_button == 1 and not self.rb_pressed:
            self.return_to_home()
            self.rb_pressed = True
        elif rb_button == 0:
            self.rb_pressed = False
        
        # D-pad for velocity adjustment
        # Try axes first (most common), then buttons
        if len(msg.axes) > 7:
            dpad_ud = msg.axes[7]  # D-pad up/down
            
            # D-pad Up (increase speed)
            if dpad_ud > 0.5 and not self.dpad_up_pressed:
                self.adjust_velocity(increase=True)
                self.dpad_up_pressed = True
            elif dpad_ud <= 0.5:
                self.dpad_up_pressed = False
            
            # D-pad Down (decrease speed)
            if dpad_ud < -0.5 and not self.dpad_down_pressed:
                self.adjust_velocity(increase=False)
                self.dpad_down_pressed = True
            elif dpad_ud >= -0.5:
                self.dpad_down_pressed = False
    
    def adjust_velocity(self, increase):
        if increase:
            self.max_linear = min(2.0, self.max_linear + self.velocity_step)
            self.max_angular = min(3.0, self.max_angular + self.velocity_step)
        else:
            self.max_linear = max(0.1, self.max_linear - self.velocity_step)
            self.max_angular = max(0.1, self.max_angular - self.velocity_step)
        
        self.get_logger().info(
            f'Speed adjusted - Linear: {self.max_linear:.1f} m/s, Angular: {self.max_angular:.1f} rad/s'
        )
    
    def return_to_home(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = self.home_x
        goal.pose.position.y = self.home_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Returning to home ({self.home_x}, {self.home_y})')

def main(args=None):
    rclpy.init(args=args)
    node = LeoXboxController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
