#!/usr/bin/env python3
"""
Simple Navigation Node for Leo Rover (No Nav2 Required)
========================================================

A lightweight proportional controller that navigates to goal poses
using only TF from slam_toolbox. Replaces the entire Nav2 stack
for simple point-to-point navigation without obstacle avoidance.

Subscribes to:
  /goal_pose  (geometry_msgs/PoseStamped)  — target coordinate
  /cancel_nav (std_msgs/Empty)             — cancel active goal
  /e_stop     (std_msgs/Bool)              — emergency stop (from Xbox controller)

Publishes to:
  /cmd_vel    (geometry_msgs/Twist)        — velocity commands
  /nav_status (std_msgs/String)            — status updates

Requires:
  - slam_toolbox running (provides map → odom TF)
  - odom_filter running (provides odom → base_link TF)

Usage:
  ros2 run <your_package> simple_nav_node

  # Send goal from command line:
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}"

  # Go to origin:
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}"

  # Cancel navigation:
  ros2 topic pub --once /cancel_nav std_msgs/msg/Empty "{}"

WARNING: No obstacle avoidance! Use in open spaces only.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Empty, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math


# ======================== Configuration ========================

# Controller gains
KP_LINEAR  = 0.5    # Proportional gain for linear velocity
KP_ANGULAR = 2.0    # Proportional gain for angular velocity

# Speed limits
MAX_LINEAR_SPEED  = 0.3   # m/s (conservative for safety)
MAX_ANGULAR_SPEED = 1.0   # rad/s
MIN_LINEAR_SPEED  = 0.05  # m/s (overcome static friction)

# Goal tolerances
DISTANCE_TOLERANCE = 0.10  # meters — close enough to goal
YAW_TOLERANCE      = 0.10  # radians (~6 degrees)
ROTATE_THRESHOLD   = 0.30  # radians — rotate in place if heading error exceeds this

# Timeout
GOAL_TIMEOUT = 120.0  # seconds — give up after this

# ==============================================================


class SimpleNavNode(Node):
    def __init__(self):
        super().__init__('simple_nav_node')

        # ---- TF2 ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Publishers ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        status_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, '/nav_status', status_qos)

        # ---- Subscribers ----
        # /goal_pose: RELIABLE + VOLATILE (default QoS, depth=10)
        # Compatible with: Xbox controller, rosbridge, ros2 topic pub
        # Nav2 bt_navigator is not used — no need for BEST_EFFORT
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.cancel_sub = self.create_subscription(
            Empty, '/cancel_nav', self.cancel_callback, 10
        )

        # ---- E-Stop subscriber (from Xbox controller or external safety) ----
        # Matches the TRANSIENT_LOCAL + RELIABLE QoS published by the controller
        e_stop_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.e_stop_sub = self.create_subscription(
            Bool, '/e_stop', self.e_stop_callback, e_stop_qos
        )
        self.e_stop_active = False

        # ---- Navigation state ----
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.nav_active = False
        self.nav_phase = 'idle'  # 'rotate', 'drive', 'final_rotate', 'idle'
        self.goal_start_time = None

        # ---- Control loop at 20 Hz ----
        self.timer = self.create_timer(0.05, self.control_loop)

        self.publish_status('idle', 'Simple nav node ready. Waiting for /goal_pose...')
        self.get_logger().info('=' * 55)
        self.get_logger().info('  Simple Navigation Node (No Nav2)')
        self.get_logger().info('=' * 55)
        self.get_logger().info('  Subscribes: /goal_pose, /cancel_nav, /e_stop')
        self.get_logger().info('  Publishes:  /cmd_vel, /nav_status')
        self.get_logger().info(f'  Tolerances: dist={DISTANCE_TOLERANCE}m, '
                               f'yaw={math.degrees(YAW_TOLERANCE):.0f}°')
        self.get_logger().info(f'  Max speed:  linear={MAX_LINEAR_SPEED}m/s, '
                               f'angular={MAX_ANGULAR_SPEED}rad/s')
        self.get_logger().info('  ⚠  NO OBSTACLE AVOIDANCE')
        self.get_logger().info('=' * 55)

    # =================== Helpers ==========================

    def publish_status(self, state: str, message: str):
        msg = String()
        msg.data = f'{state}: {message}'
        self.status_pub.publish(msg)

    def get_current_pose(self):
        """Return (x, y, yaw) in map frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return x, y, yaw
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}')
            return None, None, None

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def quaternion_to_yaw(self, orientation):
        """Extract yaw from a Quaternion message."""
        q = orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    # =================== Callbacks ========================

    def goal_callback(self, msg: PoseStamped):
        """Receive a new goal pose."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = self.quaternion_to_yaw(msg.pose.orientation)

        self.nav_active = True
        self.nav_phase = 'rotate'
        self.goal_start_time = self.get_clock().now()

        self.get_logger().info(
            f'New goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, '
            f'{math.degrees(self.goal_yaw):.0f}°)'
        )
        self.publish_status('navigating',
            f'Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def cancel_callback(self, msg: Empty):
        """Cancel active navigation."""
        self.stop_nav('cancelled', 'Navigation cancelled by user')

    def e_stop_callback(self, msg: Bool):
        """Handle E-Stop from Xbox controller or external safety system."""
        if msg.data and not self.e_stop_active:
            self.e_stop_active = True
            self.get_logger().warn('E-STOP RECEIVED — halting navigation')
            if self.nav_active:
                self.stop_nav('cancelled', 'Navigation cancelled by E-Stop')
            else:
                # Even if not navigating, ensure zero velocity
                self.cmd_vel_pub.publish(Twist())
        elif not msg.data and self.e_stop_active:
            self.e_stop_active = False
            self.get_logger().info('E-Stop released')

    def stop_nav(self, state: str, message: str):
        """Stop navigation and publish zero velocity."""
        self.nav_active = False
        self.nav_phase = 'idle'
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info(message)
        self.publish_status(state, message)

    # =================== Control Loop =====================

    def control_loop(self):
        if not self.nav_active:
            return

        # E-Stop check — do not move if E-Stop is active
        if self.e_stop_active:
            self.cmd_vel_pub.publish(Twist())
            return

        # Check timeout
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        if elapsed > GOAL_TIMEOUT:
            self.stop_nav('timeout',
                f'Goal timed out after {GOAL_TIMEOUT:.0f}s')
            return

        # Get current pose
        x, y, yaw = self.get_current_pose()
        if x is None:
            self.get_logger().warn('Lost localization — pausing navigation')
            self.cmd_vel_pub.publish(Twist())
            return

        # Compute errors
        dx = self.goal_x - x
        dy = self.goal_y - y
        distance = math.sqrt(dx * dx + dy * dy)
        bearing = math.atan2(dy, dx)
        heading_error = self.normalize_angle(bearing - yaw)
        final_yaw_error = self.normalize_angle(self.goal_yaw - yaw)

        twist = Twist()

        # ---- Phase: ROTATE to face goal ----
        if self.nav_phase == 'rotate':
            if distance < DISTANCE_TOLERANCE:
                # Already close enough, just fix orientation
                self.nav_phase = 'final_rotate'
            elif abs(heading_error) < YAW_TOLERANCE:
                self.nav_phase = 'drive'
                self.get_logger().info(
                    f'Heading aligned, driving... dist={distance:.2f}m')
            else:
                angular = KP_ANGULAR * heading_error
                angular = max(-MAX_ANGULAR_SPEED,
                              min(MAX_ANGULAR_SPEED, angular))
                twist.angular.z = angular

        # ---- Phase: DRIVE toward goal ----
        elif self.nav_phase == 'drive':
            if distance < DISTANCE_TOLERANCE:
                self.nav_phase = 'final_rotate'
                self.get_logger().info(
                    f'Reached position, adjusting heading...')
            else:
                # If heading drifts too much, go back to rotating
                if abs(heading_error) > ROTATE_THRESHOLD:
                    self.nav_phase = 'rotate'
                    self.get_logger().info('Heading drift, re-rotating...')
                else:
                    # Linear: proportional to distance, clamped
                    linear = KP_LINEAR * distance
                    linear = max(MIN_LINEAR_SPEED,
                                 min(MAX_LINEAR_SPEED, linear))
                    twist.linear.x = linear

                    # Angular: correct heading while driving
                    angular = KP_ANGULAR * heading_error * 0.5
                    angular = max(-MAX_ANGULAR_SPEED,
                                  min(MAX_ANGULAR_SPEED, angular))
                    twist.angular.z = angular

        # ---- Phase: FINAL ROTATE to goal orientation ----
        elif self.nav_phase == 'final_rotate':
            if abs(final_yaw_error) < YAW_TOLERANCE:
                self.stop_nav('arrived',
                    f'Goal reached! ({x:.2f}, {y:.2f}, '
                    f'{math.degrees(yaw):.0f}°)')
                return
            else:
                angular = KP_ANGULAR * final_yaw_error
                angular = max(-MAX_ANGULAR_SPEED,
                              min(MAX_ANGULAR_SPEED, angular))
                twist.angular.z = angular

        self.cmd_vel_pub.publish(twist)


# ========================== Entry =============================

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
