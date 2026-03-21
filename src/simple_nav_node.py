#!/usr/bin/env python3
"""
Simple Navigation Node for Leo Rover with Obstacle Avoidance
==============================================================

A lightweight navigation + safety node that:
  1. Filters ALL velocity commands through LiDAR obstacle detection
  2. Navigates to goal poses using proportional control with SLAM TF
  3. Passes through manual teleop commands with obstacle safety
  4. Supports slope line-following mode (auto-steer, manual speed)
  5. Replays recorded waypoints using pure pursuit (from Xbox controller)
  6. Falls back to IMU-only tilt detection when camera/slope_pilot unavailable

Architecture:
  Xbox/keyboard -> /cmd_vel_teleop -> [this node] -> /cmd_vel -> rover motors
  /goal_pose    -> [this node]     -> /cmd_vel -> rover motors
                       ^                |
                     /scan              v
                     /slope_mode      /nav_mode -> Xbox display
                     /line_steer      /nav_status
                     /imu/data
                     /waypoint_replay
                     /line_follow_cmd

Three navigation modes (determined internally, published on /nav_mode):
  "slam"         - SLAM TF available + flat ground → full coordinate nav
  "dead_reckon"  - SLAM unavailable OR tilted → EKF-only, timed drive
  "line_follow"  - RT trigger active → camera steering from slope_pilot_node

EKF (odom→base_footprint) fusing IMU + wheel encoders runs continuously
in ALL modes. SLAM (map→odom) adds correction when available.
Mode is published on /nav_mode for Xbox display.

Line following is NOT auto-triggered by tilt. It is activated by:
  1. RT trigger on Xbox controller (autonomous line follow)
  2. During waypoint replay of "line_follow" segments
Teleop is always full manual regardless of mode.

Hardware degradation:
  No camera:  line_follow unavailable, slam/dead_reckon work normally
  No LiDAR:   obstacle avoidance disabled, all modes still work
  No SLAM:    automatic dead_reckon mode, nav waypoints skipped in replay
  Tilt:       automatic dead_reckon mode (SLAM unreliable on slopes)

Subscribes to:
  /cmd_vel_teleop (geometry_msgs/Twist)       - manual driving (from Xbox/keyboard)
  /goal_pose      (geometry_msgs/PoseStamped) - navigation target
  /cancel_nav     (std_msgs/Empty)            - cancel active goal
  /e_stop         (std_msgs/Bool)             - emergency stop
  /scan           (sensor_msgs/LaserScan)     - LiDAR for obstacle detection
  /slope_mode     (std_msgs/String)           - camera line detection from slope_pilot_node
  /line_steer     (std_msgs/Float64)          - line-follow steering command
  /imu/data       (sensor_msgs/Imu)           - IMU for fallback tilt detection
  /waypoint_replay (std_msgs/String)          - JSON replay commands from Xbox

Publishes to:
  /cmd_vel    (geometry_msgs/Twist)  - filtered velocity commands (sole publisher)
  /nav_status (std_msgs/String)      - status updates
  /nav_mode   (std_msgs/String)      - current mode: slam/dead_reckon/line_follow

Obstacle Avoidance:
  - Checks front/rear LiDAR sectors before allowing linear motion
  - Slows down approaching obstacles, full stop at safety distance
  - Turning (angular velocity) always allowed (helps avoid obstacles)
  - Applies to BOTH teleop and autonomous navigation

Requires (mandatory):
  - odom_filter running (provides odom -> base_link TF, systemd)

Optional (graceful degradation):
  - slam_toolbox (provides map -> odom TF; without: dead_reckon mode)
  - sllidar_node (provides /scan; without: no obstacle avoidance)
  - slope_pilot_node (provides /slope_mode + /line_steer; without: no line following)

Usage:
  python3 simple_nav_node.py

  # Manual driving is via /cmd_vel_teleop (published by xbox_leo_unified.py)
  # Navigation goals via /goal_pose (published by xbox_leo_unified.py or CLI)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Bool, Empty, String, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math
import time
import json


# ======================== Configuration ========================

# Controller gains (for navigation)
KP_LINEAR  = 0.5    # Proportional gain for linear velocity
KP_ANGULAR = 2.0    # Proportional gain for angular velocity

# Speed limits (for navigation)
MAX_LINEAR_SPEED  = 0.3   # m/s (conservative for safety)
MAX_ANGULAR_SPEED = 1.0   # rad/s
MIN_LINEAR_SPEED  = 0.05  # m/s (overcome static friction)
MIN_ANGULAR_SPEED = 0.12  # rad/s (overcome static friction during rotation)

# Goal tolerances
DISTANCE_TOLERANCE = 0.10  # meters -- close enough to goal
YAW_TOLERANCE      = 0.10  # radians (~6 degrees)
ROTATE_THRESHOLD   = 0.30  # radians -- rotate in place if heading error exceeds this

# Timeout / fault handling
GOAL_TIMEOUT       = 120.0  # seconds -- give up after this
TF_LOSS_ABORT_TIME = 3.0    # seconds -- abort nav if localization lost this long
LOG_THROTTLE_SEC   = 1.0    # seconds -- prevent TF warning spam

# ---------- Pure Pursuit (waypoint replay) ----------
LOOKAHEAD_DIST   = 0.30  # meters -- how far ahead on path to aim
CONSUME_RADIUS   = 0.15  # meters -- waypoint considered "passed"
PURSUIT_SPEED    = 0.25  # m/s -- cruise speed during replay
PURSUIT_MIN_SPEED = 0.10 # m/s -- minimum speed on tight curves
CURVATURE_SLOW   = 0.10  # speed reduction per rad/s of angular command

# ---------- Autonomous Line Following ----------
LINE_FOLLOW_SPEED = 0.20  # m/s -- forward speed during autonomous line follow

# ---------- Obstacle Avoidance ----------
OBSTACLE_STOP_DIST = 0.40  # meters -- full stop (rover radius + margin)
OBSTACLE_SLOW_DIST = 0.60  # meters -- start slowing down

# Detection arc (degrees, half-width from center)
OBSTACLE_ARC_HALF_DEG = 30.0  # front/rear arcs: +/-30 = 60 total cone

# LiDAR frame: static_transform_publisher uses yaw=pi from base_link to laser.
# So laser angle 0 = robot rear, laser angle +/-pi = robot front.
FRONT_CENTER_RAD = math.pi   # robot front in laser frame
REAR_CENTER_RAD  = 0.0       # robot rear in laser frame

# Teleop passthrough
TELEOP_TIMEOUT = 0.25  # seconds -- stop if no fresh teleop command

# ---------- IMU Tilt Detection ----------
TILT_THRESHOLD_DEG  = 10.0   # degrees -- auto switch to dead_reckon
TILT_RESUME_DEG     = 8.0    # degrees -- hysteresis band

# ---------- Hardware Degradation Detection ----------
SCAN_TIMEOUT        = 5.0    # seconds -- no /scan = LiDAR unavailable
SLAM_CHECK_INTERVAL = 10.0   # seconds -- how often to check map→odom TF

# Navigation blocked by obstacle
NAV_BLOCKED_TIMEOUT = 3.0  # seconds -- cancel nav if obstacle blocks forward for this long

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

        # /nav_mode: current navigation mode (slam/dead_reckon/line_follow)
        self.nav_mode_pub = self.create_publisher(String, '/nav_mode', 10)

        # ---- Subscribers ----
        # /goal_pose: default RELIABLE QoS
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.cancel_sub = self.create_subscription(
            Empty, '/cancel_nav', self.cancel_callback, 10
        )

        # /cmd_vel_teleop: manual driving from Xbox/keyboard controller
        self.teleop_sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.teleop_callback, 10
        )

        # /scan: LiDAR for obstacle detection
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # /slope_mode: INPUT from slope_pilot_node — tells us if camera sees a line
        # (we read this but do NOT set our mode from it; mode is internal)
        self.slope_mode_sub = self.create_subscription(
            String, '/slope_mode', self.slope_mode_callback, 10
        )

        # /line_steer: angular.z steering from slope_pilot_node line follower
        self.line_steer_sub = self.create_subscription(
            Float64, '/line_steer', self.line_steer_callback, 10
        )

        # /waypoint_replay: JSON commands from Xbox controller (start/stop replay)
        self.replay_cmd_sub = self.create_subscription(
            String, '/waypoint_replay', self.replay_cmd_callback, 10
        )

        # /line_follow_cmd: JSON commands from Xbox controller (start/stop line follow)
        self.line_follow_cmd_sub = self.create_subscription(
            String, '/line_follow_cmd', self.line_follow_cmd_callback, 10
        )

        # /imu/data: IMU for independent tilt detection (fallback when no camera)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # /e_stop: emergency stop
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

        # ---- LiDAR state ----
        self._latest_scan = None
        self._scan_stamp = 0.0           # time.monotonic() of last /scan msg
        self._lidar_available = False     # True when scans are fresh
        self._lidar_warned = False        # throttle "no LiDAR" warnings
        self._front_blocked = False       # for throttled logging
        self._rear_blocked = False

        # ---- SLAM state ----
        self._slam_available = False      # True when map→odom TF works
        self._slam_last_check = 0.0       # time.monotonic() of last check
        self._slam_warned = False         # throttle "no SLAM" warnings

        # ---- Navigation obstacle-blocked tracking ----
        self._nav_blocked_since = None  # time.monotonic() when nav first blocked by obstacle

        # ---- TF loss fault tracking ----
        self._tf_loss_start = None       # rclpy.time when TF first failed during nav
        self._last_tf_warn_time = None   # throttled TF warning timestamp (seconds)

        # ---- Teleop state ----
        self._teleop_cmd = Twist()
        self._teleop_stamp = 0.0  # time.monotonic() of last teleop msg

        # ---- Navigation Mode (3-mode state machine) ----
        # Modes: "slam", "dead_reckon", "line_follow"
        #   slam:        SLAM TF available + not tilted → full coordinate nav
        #   dead_reckon: SLAM unavailable OR tilted → timed replay only
        #   line_follow: RT trigger active → camera steering (user-triggered)
        # EKF (odom→base_footprint) always runs regardless of mode.
        self._nav_mode = "slam"
        self._prev_nav_mode = "slam"

        # Camera line detection (from slope_pilot_node)
        self._camera_sees_line = False   # True when slope_pilot publishes "line_follow"
        self._line_steer = 0.0          # angular.z from slope_pilot line follower
        self._slope_pilot_last_msg = 0.0
        self._slope_pilot_alive = False

        # IMU tilt detection
        self._imu_is_tilted = False
        self._imu_roll_deg = 0.0
        self._imu_pitch_deg = 0.0

        # Line follow (RT trigger state, separate from mode)
        self._line_follow_requested = False  # True while RT held on Xbox

        # ---- Navigation state ----
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.nav_active = False
        self.nav_phase = 'idle'  # 'rotate', 'drive', 'final_rotate', 'idle'
        self.goal_start_time = None

        # ---- Waypoint replay state (received from Xbox controller) ----
        self._replay_active = False
        self._replay_waypoints = []          # list of typed waypoint tuples
        self._replay_index = 0               # current waypoint index
        self._replay_driving = False         # True during timed segment
        self._replay_segment_type = None     # "dead_reckon" or "line_follow"
        self._replay_speed = 0.0             # forward speed for timed segment
        self._replay_steer = 0.0             # angular.z for dead_reckon segment
        self._replay_end_time = 0.0          # when timed segment ends

        # ---- Control loop at 20 Hz ----
        self.timer = self.create_timer(0.05, self.control_loop)

        self.publish_status('idle', 'Simple nav node ready. Waiting for /goal_pose...')
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Simple Navigation Node with Obstacle Avoidance')
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Subscribes: /cmd_vel_teleop, /goal_pose, /cancel_nav,')
        self.get_logger().info('              /e_stop, /scan, /slope_mode, /line_steer,')
        self.get_logger().info('              /waypoint_replay, /line_follow_cmd, /imu/data')
        self.get_logger().info('  Publishes:  /cmd_vel, /nav_status, /nav_mode')
        self.get_logger().info(f'  Modes:      slam / dead_reckon / line_follow')
        self.get_logger().info(f'  EKF:        always active (odom→base_footprint)')
        self.get_logger().info(f'  Obstacle:   stop={OBSTACLE_STOP_DIST}m, '
                               f'slow={OBSTACLE_SLOW_DIST}m, '
                               f'arc=+/-{OBSTACLE_ARC_HALF_DEG:.0f}deg')
        self.get_logger().info(f'  Nav tol:    dist={DISTANCE_TOLERANCE}m, '
                               f'yaw={math.degrees(YAW_TOLERANCE):.0f}deg')
        self.get_logger().info(f'  Nav speed:  linear={MAX_LINEAR_SPEED}m/s, '
                               f'angular={MAX_ANGULAR_SPEED}rad/s')
        self.get_logger().info(f'  Pursuit:    lookahead={LOOKAHEAD_DIST}m, '
                               f'cruise={PURSUIT_SPEED}m/s, '
                               f'consume={CONSUME_RADIUS}m')
        self.get_logger().info(f'  Tilt:       threshold={TILT_THRESHOLD_DEG}deg, '
                               f'IMU fallback if no camera')
        self.get_logger().info(f'  Degrade:    no LiDAR=no obstacles, '
                               f'no SLAM=teleop+slope only')
        self.get_logger().info('=' * 60)

    # =================== Helpers ==========================

    def publish_status(self, state: str, message: str):
        msg = String()
        msg.data = f'{state}: {message}'
        self.status_pub.publish(msg)

    def get_current_pose(self):
        """Return (x, y, yaw) in map frame. Returns (None, None, None) if SLAM unavailable."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.35),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            if not self._slam_available:
                self._slam_available = True
                self._slam_warned = False
                self.get_logger().info('SLAM online — coordinate navigation enabled')
            self._tf_loss_start = None  # TF recovered
            return x, y, yaw
        except TransformException as ex:
            self._slam_available = False
            # Throttled TF warning (once per LOG_THROTTLE_SEC)
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if (self._last_tf_warn_time is None or
                    (now_sec - self._last_tf_warn_time) >= LOG_THROTTLE_SEC):
                self.get_logger().warn(f'TF lookup failed: {ex}')
                self._last_tf_warn_time = now_sec
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

    def scaled_linear_speed(self, distance: float) -> float:
        """Softer near-goal speed profile to reduce overshoot."""
        speed = KP_LINEAR * distance
        speed = max(0.0, min(MAX_LINEAR_SPEED, speed))
        # Only enforce minimum speed when still meaningfully far away
        if distance > max(DISTANCE_TOLERANCE * 2.0, 0.25):
            speed = max(speed, MIN_LINEAR_SPEED)
        return speed

    def scaled_angular_speed(self, error: float,
                              gain: float = KP_ANGULAR) -> float:
        """Angular speed with minimum threshold to overcome static friction."""
        speed = gain * error
        speed = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, speed))
        # Apply minimum angular speed when error is significant
        if abs(error) > YAW_TOLERANCE and abs(speed) < MIN_ANGULAR_SPEED:
            speed = math.copysign(MIN_ANGULAR_SPEED, error)
        return speed

    # =================== Obstacle Detection ===============

    def _get_sector_min_dist(self, center_rad: float) -> float:
        """
        Get minimum obstacle distance in a sector of the LiDAR scan.

        Args:
            center_rad: center angle of sector in laser frame (radians)

        Returns:
            Minimum valid range in the sector, or inf if no data.
        """
        scan = self._latest_scan
        if scan is None:
            return float('inf')

        half_arc = math.radians(OBSTACLE_ARC_HALF_DEG)
        min_dist = float('inf')

        angle = scan.angle_min
        for r in scan.ranges:
            angle_diff = abs(self.normalize_angle(angle - center_rad))
            if angle_diff < half_arc:
                if scan.range_min < r < scan.range_max and math.isfinite(r):
                    min_dist = min(min_dist, r)
            angle += scan.angle_increment

        return min_dist

    def _filter_obstacles(self, twist: Twist) -> Twist:
        """
        Apply obstacle safety filtering to a velocity command.
        If LiDAR is unavailable, passes through unfiltered (no safety net).

        - Forward motion: blocked/slowed if front sector has obstacle
        - Backward motion: blocked/slowed if rear sector has obstacle
        - Turning (angular.z): ALWAYS passes through unfiltered
        """
        # Check LiDAR freshness
        if (time.monotonic() - self._scan_stamp) > SCAN_TIMEOUT:
            if self._lidar_available:
                self._lidar_available = False
                self.get_logger().warn(
                    'LiDAR offline — obstacle avoidance DISABLED')
            return twist  # pass through unfiltered

        filtered = Twist()
        filtered.angular.z = twist.angular.z  # turning always allowed

        if abs(twist.linear.x) < 0.001:
            return filtered

        if twist.linear.x > 0:
            # Moving forward -- check front sector
            front_dist = self._get_sector_min_dist(FRONT_CENTER_RAD)

            if front_dist < OBSTACLE_STOP_DIST:
                filtered.linear.x = 0.0
                if not self._front_blocked:
                    self._front_blocked = True
                    self.get_logger().warn(
                        f'OBSTACLE FRONT {front_dist:.2f}m -- forward blocked')
            elif front_dist < OBSTACLE_SLOW_DIST:
                scale = (front_dist - OBSTACLE_STOP_DIST) / \
                        (OBSTACLE_SLOW_DIST - OBSTACLE_STOP_DIST)
                filtered.linear.x = twist.linear.x * scale
                self._front_blocked = False
            else:
                filtered.linear.x = twist.linear.x
                self._front_blocked = False

        elif twist.linear.x < 0:
            # Moving backward -- check rear sector
            rear_dist = self._get_sector_min_dist(REAR_CENTER_RAD)

            if rear_dist < OBSTACLE_STOP_DIST:
                filtered.linear.x = 0.0
                if not self._rear_blocked:
                    self._rear_blocked = True
                    self.get_logger().warn(
                        f'OBSTACLE REAR {rear_dist:.2f}m -- backward blocked')
            elif rear_dist < OBSTACLE_SLOW_DIST:
                scale = (rear_dist - OBSTACLE_STOP_DIST) / \
                        (OBSTACLE_SLOW_DIST - OBSTACLE_STOP_DIST)
                filtered.linear.x = twist.linear.x * scale
                self._rear_blocked = False
            else:
                filtered.linear.x = twist.linear.x
                self._rear_blocked = False

        return filtered

    # =================== Callbacks ========================

    def scan_callback(self, msg: LaserScan):
        """Store latest LiDAR scan for obstacle detection."""
        self._latest_scan = msg
        self._scan_stamp = time.monotonic()
        if not self._lidar_available:
            self._lidar_available = True
            self._lidar_warned = False
            self.get_logger().info('LiDAR online — obstacle avoidance enabled')

    def slope_mode_callback(self, msg: String):
        """Track camera line detection from slope_pilot_node."""
        self._slope_pilot_alive = True
        self._slope_pilot_last_msg = time.monotonic()
        was_seeing = self._camera_sees_line
        self._camera_sees_line = (msg.data == "line_follow")
        if self._camera_sees_line != was_seeing:
            self.get_logger().info(
                f'Camera line: {"DETECTED" if self._camera_sees_line else "LOST"}')

    def line_steer_callback(self, msg: Float64):
        """Update line-follow steering from slope_pilot_node."""
        self._line_steer = msg.data

    def imu_callback(self, msg: Imu):
        """IMU tilt detection for automatic dead_reckon mode switching."""
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        self._imu_roll_deg = abs(math.degrees(roll))
        self._imu_pitch_deg = abs(math.degrees(pitch))
        tilt_deg = max(self._imu_roll_deg, self._imu_pitch_deg)

        # Tilt detection with hysteresis
        if not self._imu_is_tilted and tilt_deg > TILT_THRESHOLD_DEG:
            self._imu_is_tilted = True
        elif self._imu_is_tilted and tilt_deg < TILT_RESUME_DEG:
            self._imu_is_tilted = False

    def _update_nav_mode(self):
        """
        Determine current navigation mode. Called every control loop tick.

        Priority:
          1. line_follow  — RT trigger held (user-requested, overrides everything)
          2. dead_reckon  — SLAM unavailable OR tilted
          3. slam         — SLAM available AND flat ground

        EKF (odom→base_footprint) runs in ALL modes.
        SLAM (map→odom) only contributes in 'slam' mode.
        """
        if self._line_follow_requested:
            new_mode = "line_follow"
        elif not self._slam_available or self._imu_is_tilted:
            new_mode = "dead_reckon"
        else:
            new_mode = "slam"

        if new_mode != self._nav_mode:
            self.get_logger().info(
                f'Nav mode: {self._nav_mode} -> {new_mode}'
                f' (slam={"ok" if self._slam_available else "lost"}'
                f', tilt={"yes" if self._imu_is_tilted else "no"}'
                f', line={"RT" if self._line_follow_requested else "off"})')
            self._nav_mode = new_mode

            # Publish mode change
            msg = String()
            msg.data = new_mode
            self.nav_mode_pub.publish(msg)

    def get_ekf_pose(self):
        """Return (x, y, yaw) in odom frame from EKF. Always available."""
        try:
            t = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return x, y, yaw
        except TransformException:
            return None, None, None

    def teleop_callback(self, msg: Twist):
        """Store latest teleop command with timestamp."""
        self._teleop_cmd = msg
        self._teleop_stamp = time.monotonic()

    def goal_callback(self, msg: PoseStamped):
        """Receive a new goal pose. Validates input, requires SLAM."""
        # Validate frame_id
        frame_id = (msg.header.frame_id or '').strip()
        if frame_id not in ('', 'map'):
            self.get_logger().error(
                f'Rejecting goal: unsupported frame_id={frame_id!r} (expected "map")')
            self.publish_status('failed', f'Unsupported goal frame: {frame_id}')
            return

        # Validate position values
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        if not (math.isfinite(gx) and math.isfinite(gy)):
            self.get_logger().error('Rejecting goal: non-finite position values')
            self.publish_status('failed', 'Goal contains invalid position values')
            return

        # Check SLAM availability before accepting coordinate goal
        x, y, _ = self.get_current_pose()
        if x is None:
            self.get_logger().warn(
                'Goal rejected — SLAM not available (no map→base_link TF). '
                'Teleop and slope driving still work.')
            self.publish_status('failed',
                'Goal rejected: no SLAM localization')
            return

        self.goal_x = float(gx)
        self.goal_y = float(gy)
        self.goal_yaw = self.quaternion_to_yaw(msg.pose.orientation)

        self.nav_active = True
        self.nav_phase = 'rotate'
        self.goal_start_time = self.get_clock().now()
        self._tf_loss_start = None

        self.get_logger().info(
            f'New goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, '
            f'{math.degrees(self.goal_yaw):.0f}deg)'
        )
        self.publish_status('navigating',
            f'Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def cancel_callback(self, msg: Empty):
        """Cancel active navigation, replay, and/or line following."""
        if self._line_follow_requested:
            self._stop_line_follow('Line follow cancelled by user')
        if self._replay_active:
            self._stop_replay('Replay cancelled by user')
        else:
            self.stop_nav('cancelled', 'Navigation cancelled by user')

    def e_stop_callback(self, msg: Bool):
        """Handle E-Stop from Xbox controller or external safety system."""
        if msg.data and not self.e_stop_active:
            self.e_stop_active = True
            self.get_logger().warn('E-STOP RECEIVED -- halting all motion')
            if self._line_follow_requested:
                self._stop_line_follow('Line follow cancelled by E-Stop')
            if self._replay_active:
                self._stop_replay('Replay cancelled by E-Stop')
            elif self.nav_active:
                self.stop_nav('cancelled', 'Navigation cancelled by E-Stop')
            else:
                self.cmd_vel_pub.publish(Twist())
        elif not msg.data and self.e_stop_active:
            self.e_stop_active = False
            self.get_logger().info('E-Stop released')

    def stop_nav(self, state: str, message: str):
        """Stop navigation and publish zero velocity."""
        was_replay = self._replay_active and self.nav_active
        self.nav_active = False
        self.nav_phase = 'idle'
        self._nav_blocked_since = None
        self._tf_loss_start = None
        self.cmd_vel_pub.publish(Twist())

        if was_replay:
            if state == 'arrived':
                # Nav waypoint reached — advance to next in replay
                self._replay_advance()
            else:
                # Error during replay (blocked, timeout, failed)
                self._stop_replay(f'Replay stopped: {message}')
        else:
            self.get_logger().info(message)
            self.publish_status(state, message)

    # =================== Waypoint Replay Engine ===========

    def replay_cmd_callback(self, msg: String):
        """Handle waypoint replay commands from Xbox controller."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid replay command JSON')
            return

        cmd = data.get('cmd', '')

        if cmd == 'start':
            raw_waypoints = data.get('waypoints', [])
            if not raw_waypoints:
                self.get_logger().warn('Replay start with empty waypoint list')
                return

            # Mutual exclusion: stop line following if active
            if self._line_follow_requested:
                self._stop_line_follow('Line follow stopped for waypoint replay')

            # Cancel any existing navigation or replay
            if self.nav_active:
                self.nav_active = False
                self.nav_phase = 'idle'
                self.cmd_vel_pub.publish(Twist())

            # Store waypoints (not persisted — cleared on stop)
            self._replay_waypoints = [tuple(wp) for wp in raw_waypoints]
            start_index = data.get('start_index', 0)
            start_index = max(0, min(start_index, len(self._replay_waypoints) - 1))
            self._replay_index = start_index
            self._replay_active = True
            self._replay_driving = False

            n = len(self._replay_waypoints)
            nav_count = sum(1 for wp in self._replay_waypoints if wp[0] == 'nav')
            dr_count = sum(1 for wp in self._replay_waypoints
                           if wp[0] in ('dead_reckon', 'slope'))
            lf_count = sum(1 for wp in self._replay_waypoints if wp[0] == 'line_follow')
            self.get_logger().info(
                f'Replay started from WP {start_index + 1}/{n}: '
                f'{nav_count} nav + {dr_count} dead_reckon + {lf_count} line_follow, looping')
            self.publish_status('replaying',
                f'Replaying {n} waypoints from #{start_index + 1}')

            # Start first waypoint
            self._replay_advance()

        elif cmd == 'stop':
            self._stop_replay('Replay stopped by user')

    def _stop_replay(self, reason: str):
        """Stop waypoint replay and return to teleop."""
        if not self._replay_active:
            return

        self._replay_active = False
        self._replay_driving = False
        self._replay_segment_type = None
        self._replay_steer = 0.0
        self._replay_waypoints = []
        self.nav_active = False
        self.nav_phase = 'idle'
        self.cmd_vel_pub.publish(Twist())

        self.get_logger().info(reason)
        self.publish_status('replay_stopped', reason)

    def _replay_advance(self):
        """
        Begin executing the waypoint at _replay_index.
        Three waypoint types:
          nav:          pure pursuit (needs SLAM, skip if unavailable)
          dead_reckon:  timed forward drive, no steering
          line_follow:  timed forward drive with camera steering
        """
        if not self._replay_active or not self._replay_waypoints:
            return

        n = len(self._replay_waypoints)
        wp = self._replay_waypoints[self._replay_index]
        wp_type = wp[0]

        # Backward compat: treat old "slope" as "dead_reckon"
        if wp_type == 'slope':
            wp_type = 'dead_reckon'

        if wp_type == 'nav':
            # Check SLAM before starting nav waypoints
            x, y, _ = self.get_current_pose()
            if x is None:
                # No SLAM — skip all consecutive nav waypoints
                skipped = 0
                while skipped < n:
                    self._replay_index = (self._replay_index + 1) % n
                    skipped += 1
                    next_wp = self._replay_waypoints[self._replay_index]
                    next_type = next_wp[0]
                    if next_type == 'slope':
                        next_type = 'dead_reckon'
                    if next_type != 'nav':
                        self.get_logger().warn(
                            f'No SLAM — skipped {skipped} nav waypoint(s)')
                        self._replay_advance()
                        return
                self._stop_replay(
                    'Replay stopped: all waypoints are nav but SLAM unavailable')
                return

            # SLAM available — pure pursuit through nav waypoints
            self.nav_active = True
            self.nav_phase = 'drive'
            self.goal_start_time = self.get_clock().now()
            self._replay_driving = False
            self._nav_blocked_since = None

            self.get_logger().info(
                f'Pursuit from WP {self._replay_index + 1}/{n} [nav]')

        elif wp_type in ('dead_reckon', 'line_follow'):
            # Timed segment: drive at recorded speed (and steering) for duration
            # dead_reckon: 4-tuple (type, speed, angular_z, duration) or old 3-tuple
            # line_follow: 3-tuple (type, speed, duration) — steering from camera
            if wp_type == 'dead_reckon' and len(wp) >= 4:
                speed = float(wp[1])
                steer = float(wp[2])
                duration = float(wp[3])
            else:
                # line_follow 3-tuple, or old dead_reckon/slope 3-tuple
                speed = float(wp[1])
                steer = 0.0
                duration = float(wp[2])

            self._replay_driving = True
            self._replay_segment_type = wp_type
            self._replay_speed = speed
            self._replay_steer = steer
            self._replay_end_time = time.monotonic() + duration
            self.nav_active = False

            # Only log first segment in a sequence of same type
            prev_idx = (self._replay_index - 1) % n
            prev_type = self._replay_waypoints[prev_idx][0]
            if prev_type == 'slope':
                prev_type = 'dead_reckon'
            if prev_type != wp_type:
                self.get_logger().info(
                    f'WP {self._replay_index + 1}/{n} [{wp_type}]: '
                    f'entering {wp_type} section')

            # Advance index past this timed segment
            self._replay_index = (self._replay_index + 1) % n

    # =================== Autonomous Line Follow Engine =====

    def line_follow_cmd_callback(self, msg: String):
        """Handle line follow commands from Xbox controller.
        No tilt requirement. If no camera or no line, car stops immediately."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid line follow command JSON')
            return

        cmd = data.get('cmd', '')

        if cmd == 'start':
            # Mutual exclusion: stop replay if active
            if self._replay_active:
                self._stop_replay('Replay stopped for line follow')
            if self.nav_active:
                self.nav_active = False
                self.nav_phase = 'idle'
                self.cmd_vel_pub.publish(Twist())

            self._line_follow_requested = True

            # Check if line is currently detected
            if self._camera_sees_line:
                self.get_logger().info(
                    f'Line follow started at {LINE_FOLLOW_SPEED} m/s (line detected)')
                self.publish_status('line_following',
                    f'Following line at {LINE_FOLLOW_SPEED} m/s')
            else:
                # No line visible — stop car, but keep mode active
                # (will resume when line appears, or user releases RT)
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().warn(
                    'Line follow active but no line detected — car stopped. '
                    'Will resume when line appears or release RT to cancel.')
                self.publish_status('line_following',
                    'Waiting for line (car stopped)')

        elif cmd == 'stop':
            self._stop_line_follow('Line follow stopped by user')

    def _stop_line_follow(self, reason: str):
        """Stop autonomous line following and return to teleop."""
        if not self._line_follow_requested:
            return

        self._line_follow_requested = False
        self.cmd_vel_pub.publish(Twist())

        self.get_logger().info(reason)
        self.publish_status('line_follow_stopped', reason)

    # =================== Control Loop =====================

    def control_loop(self):
        """
        Main loop at 20 Hz.

        Navigation modes (determined by _update_nav_mode):
          slam:        SLAM TF available + flat → coordinate nav
          dead_reckon: SLAM lost OR tilted → timed drive replay
          line_follow: RT trigger held → camera steering

        Control priority:
          1. E-Stop → zero velocity
          2. Nav active (go-home or replay pursuit) → nav controller
          3. Replay timed segment (dead_reckon/line_follow) → recorded commands
          4. Autonomous line following (RT trigger) → camera + preset speed
          5. Teleop passthrough → full manual + obstacle filter

        EKF (odom→base_footprint) runs continuously in all modes.
        """
        # Update navigation mode every tick
        self._update_nav_mode()

        # E-Stop overrides everything
        if self.e_stop_active:
            self.cmd_vel_pub.publish(Twist())
            return

        # ---- Mode 1: Autonomous navigation (go-home or replay nav waypoint) ----
        if self.nav_active:
            twist = self._compute_nav_velocity()
            if twist is not None:
                filtered = self._filter_obstacles(twist)
                self.cmd_vel_pub.publish(filtered)

                # Detect if obstacle is blocking navigation progress
                wanted_forward = twist.linear.x > 0.01
                got_blocked = abs(filtered.linear.x) < 0.01
                if wanted_forward and got_blocked:
                    now = time.monotonic()
                    if self._nav_blocked_since is None:
                        self._nav_blocked_since = now
                    elif (now - self._nav_blocked_since) > NAV_BLOCKED_TIMEOUT:
                        front_dist = self._get_sector_min_dist(FRONT_CENTER_RAD)
                        self.stop_nav('blocked',
                            f'Obstacle at {front_dist:.2f}m blocking path for '
                            f'{NAV_BLOCKED_TIMEOUT:.0f}s')
                else:
                    self._nav_blocked_since = None
            return

        # ---- Mode 2: Replay timed segment (dead_reckon or line_follow) ----
        if self._replay_active and self._replay_driving:
            now = time.monotonic()
            if now >= self._replay_end_time:
                # Segment complete — advance to next waypoint
                self._replay_driving = False
                self._replay_advance()
            else:
                cmd = Twist()
                cmd.linear.x = self._replay_speed

                if self._replay_segment_type == "line_follow":
                    # Line follow replay: use camera steering if line visible
                    if self._camera_sees_line:
                        cmd.angular.z = self._line_steer
                    else:
                        # No line visible during line_follow replay — stop car
                        self.cmd_vel_pub.publish(Twist())
                        return
                elif self._replay_segment_type == "dead_reckon":
                    # Dead reckon replay: use recorded steering
                    cmd.angular.z = self._replay_steer

                filtered = self._filter_obstacles(cmd)
                self.cmd_vel_pub.publish(filtered)
            return

        # ---- Mode 3: Autonomous line following (RT-triggered) ----
        if self._line_follow_requested:
            if self._camera_sees_line:
                # Line detected — drive forward with camera steering
                cmd = Twist()
                cmd.linear.x = LINE_FOLLOW_SPEED
                cmd.angular.z = self._line_steer
                filtered = self._filter_obstacles(cmd)
                self.cmd_vel_pub.publish(filtered)
            else:
                # No line detected — stop car but keep mode active
                # (will resume when line appears, or user releases RT)
                self.cmd_vel_pub.publish(Twist())
            return

        # ---- Mode 4: Teleop passthrough ----
        now = time.monotonic()
        if (now - self._teleop_stamp) >= TELEOP_TIMEOUT:
            # No fresh teleop -- don't publish
            return

        # Full manual teleop with obstacle filtering (all modes)
        filtered = self._filter_obstacles(self._teleop_cmd)
        self.cmd_vel_pub.publish(filtered)

    def _compute_nav_velocity(self):
        """
        Compute navigation velocity based on current phase.
        Returns Twist, or None if navigation ended this tick.

        During replay: pure pursuit controller with lookahead. Drives smoothly
        through waypoints without stopping. Consumes waypoints as passed.
        During single-goal (go-home): full 3-phase (rotate, drive, final_rotate).
        """
        # Check timeout
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        if elapsed > GOAL_TIMEOUT:
            self.stop_nav('timeout',
                f'Goal timed out after {GOAL_TIMEOUT:.0f}s')
            return None

        # Get current pose
        x, y, yaw = self.get_current_pose()
        if x is None:
            # TF loss — track duration for timed abort
            now = self.get_clock().now()
            if self._tf_loss_start is None:
                self._tf_loss_start = now
            else:
                loss_sec = (now - self._tf_loss_start).nanoseconds / 1e9
                if loss_sec >= TF_LOSS_ABORT_TIME:
                    self.stop_nav('failed',
                        f'Localization lost for {loss_sec:.1f}s — aborting')
                    return None
            self.cmd_vel_pub.publish(Twist())
            return None
        else:
            self._tf_loss_start = None  # TF recovered

        # ============================================================
        # REPLAY MODE: Pure pursuit through waypoint sequence
        # ============================================================
        if self._replay_active:
            return self._compute_pursuit_velocity(x, y, yaw)

        # ============================================================
        # SINGLE GOAL MODE (go-home): full 3-phase controller
        # ============================================================

        # Compute errors to goal
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
                self.nav_phase = 'final_rotate'
            elif abs(heading_error) < YAW_TOLERANCE:
                self.nav_phase = 'drive'
                self.get_logger().info(
                    f'Heading aligned, driving... dist={distance:.2f}m')
            else:
                twist.angular.z = self.scaled_angular_speed(heading_error)

        # ---- Phase: DRIVE toward goal ----
        elif self.nav_phase == 'drive':
            if distance < DISTANCE_TOLERANCE:
                self.nav_phase = 'final_rotate'
                self.get_logger().info('Reached position, adjusting heading...')
            else:
                if abs(heading_error) > ROTATE_THRESHOLD:
                    self.nav_phase = 'rotate'
                    self.get_logger().info('Heading drift, re-rotating...')
                else:
                    twist.linear.x = self.scaled_linear_speed(distance)
                    twist.angular.z = self.scaled_angular_speed(
                        heading_error, gain=KP_ANGULAR * 0.5)

        # ---- Phase: FINAL ROTATE to goal orientation ----
        elif self.nav_phase == 'final_rotate':
            if abs(final_yaw_error) < YAW_TOLERANCE:
                self.stop_nav('arrived',
                    f'Goal reached! ({x:.2f}, {y:.2f}, '
                    f'{math.degrees(yaw):.0f}deg)')
                return None
            else:
                twist.angular.z = self.scaled_angular_speed(final_yaw_error)

        return twist

    # =================== Pure Pursuit (Replay) ====================

    def _compute_pursuit_velocity(self, x, y, yaw):
        """
        Pure pursuit controller for waypoint replay.
        Drives smoothly through nav waypoints without stopping.
        Returns Twist, or None if transitioning to slope segment.
        """
        n = len(self._replay_waypoints)

        # 1. Consume waypoints we've already passed
        consumed = 0
        while consumed < n:
            wp = self._replay_waypoints[self._replay_index]

            # If we hit a slope waypoint, hand off to slope driving (no stop)
            if wp[0] != 'nav':
                self.nav_active = False
                self.nav_phase = 'idle'
                self._replay_advance()  # sets up slope driving immediately
                return None

            _, wx, wy, _ = wp
            dist_to_wp = math.sqrt((wx - x) ** 2 + (wy - y) ** 2)

            if dist_to_wp < CONSUME_RADIUS:
                # Passed this waypoint — advance
                self._replay_index = (self._replay_index + 1) % n
                consumed += 1
            else:
                break

        # After consuming, check if we're now at a slope waypoint
        wp = self._replay_waypoints[self._replay_index]
        if wp[0] != 'nav':
            self.nav_active = False
            self.nav_phase = 'idle'
            self._replay_advance()
            return None

        # 2. Find lookahead point along the path ahead
        lx, ly = self._find_lookahead_point(x, y)

        # 3. Pure pursuit geometry
        dx_l = lx - x
        dy_l = ly - y
        ld = math.sqrt(dx_l * dx_l + dy_l * dy_l)

        if ld < 0.01:
            # Already at lookahead point (shouldn't happen normally)
            return Twist()

        # Angle from robot heading to lookahead point
        alpha = self.normalize_angle(math.atan2(dy_l, dx_l) - yaw)

        # Pure pursuit curvature: kappa = 2 * sin(alpha) / L
        curvature = 2.0 * math.sin(alpha) / ld

        # 4. Speed: cruise with curvature-based slowdown for smooth cornering
        angular = curvature * PURSUIT_SPEED
        angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular))

        speed = PURSUIT_SPEED - CURVATURE_SLOW * abs(angular)
        speed = max(PURSUIT_MIN_SPEED, min(MAX_LINEAR_SPEED, speed))

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angular
        return twist

    def _find_lookahead_point(self, rx, ry):
        """
        Find a point on the path LOOKAHEAD_DIST ahead of current position.
        Walks forward along nav waypoints, interpolating between segments.
        Returns (lx, ly) in map frame.
        """
        n = len(self._replay_waypoints)
        idx = self._replay_index

        # Start from the current target waypoint
        wp = self._replay_waypoints[idx]
        if wp[0] != 'nav':
            return rx, ry  # shouldn't happen

        px, py = float(wp[1]), float(wp[2])

        # Distance from robot to first waypoint counts toward lookahead
        remaining = LOOKAHEAD_DIST - math.sqrt((px - rx) ** 2 + (py - ry) ** 2)

        if remaining <= 0:
            # First waypoint is already beyond lookahead distance — aim at it
            return px, py

        # Walk forward along path segments
        for _ in range(n):
            next_idx = (idx + 1) % n
            nwp = self._replay_waypoints[next_idx]

            if nwp[0] != 'nav':
                # Slope waypoint ahead — use last nav point as lookahead
                return px, py

            nx, ny = float(nwp[1]), float(nwp[2])
            seg_len = math.sqrt((nx - px) ** 2 + (ny - py) ** 2)

            if seg_len > 0.001 and seg_len > remaining:
                # Interpolate along this segment
                t = remaining / seg_len
                return px + t * (nx - px), py + t * (ny - py)

            remaining -= seg_len
            px, py = nx, ny
            idx = next_idx

        # Ran out of path (very short loop) — return last point
        return px, py


# ========================== Entry =============================

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_vel_pub.publish(Twist())
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
