#!/usr/bin/env python3
"""
Xbox Controller Teleop + Navigation for Leo Rover (ROS 2 Jazzy)
Works on both Windows (WSL2/native) and Ubuntu 24.04
Tested with Xbox One/Series wired controller

===============================================================
CONTROLLER BUTTON / AXIS MAP
===============================================================

  MOVEMENT (left stick only):
    Left Stick Y-axis     : Forward / Backward (proportional, push up/down)
    Left Stick X-axis     : Rotate Left / Right (proportional)
    Left Stick diagonal   : Forward + Turn simultaneously (arcade drive)

  SAFETY:
    Right Stick (any dir) : E-Stop ON  (any tilt or click)
    Right Stick Click     : E-Stop ON  (press = instant stop)
    Left Stick Click      : E-Stop OFF (release)

  NAVIGATION:
    X button              : Set current robot location as home
    Y button              : Navigate to home position (via /goal_pose)
    A button              : Cancel navigation / replay / line-follow
    B button              : Reset home to map origin (0, 0, 0 deg)

  WAYPOINT RECORDING:
    LB (left bumper) PRESS : Start recording waypoints at 10Hz
    RB (right bumper) PRESS: Stop recording and save to file

  AUTONOMOUS MODES:
    LT (left trigger) HOLD  : Replay waypoints while held (starts from nearest)
    RT (right trigger) HOLD : Line following while held

  TUNING (D-Pad, live while driving):
    D-Pad Up / Down       : Increase / decrease max linear speed
    D-Pad Left / Right    : Increase / decrease max angular speed

  NOT USED:
    Nexus / Guide button  : (unmapped)
    Start button          : (unmapped)

===============================================================
AXIS MAPPING (Ubuntu 24.04 xpad driver + pygame, verified)
===============================================================

  Axis 0 : Left Stick X       Axis 1 : Left Stick Y
  Axis 2 : Left Trigger (LT)  Axis 3 : Right Trigger (RT)
  Axis 4 : Right Stick Y      Axis 5 : (unused)

  Triggers rest at -1.0 and go to +1.0 when fully pressed.
  Left Stick Y: up = -1.0, down = +1.0.
  LT/RT are now used as action buttons (edge-detected at 50% threshold).
  Use --debug-axes to print raw values and verify your mapping.

===============================================================
STARTUP HEALTH CHECKS
===============================================================

  Before entering the controller loop, this script verifies:
    1. /cmd_vel  — can reach rover motor controller
    2. /scan     — LiDAR is publishing (Pi5 driver running)
    3. TF chain  — map->odom->base_link (SLAM is healthy)
    4. /goal_pose — navigation node is listening

  If any critical check fails, the script alerts and exits.
  Use --skip-checks to bypass (e.g. for teleop-only use).

===============================================================
NAVIGATION ARCHITECTURE
===============================================================

  Teleop driving publishes to /cmd_vel_teleop (NOT /cmd_vel).
  simple_nav_node filters all commands through LiDAR obstacle
  detection before forwarding to /cmd_vel.

  Navigation goals publish to /goal_pose with default RELIABLE QoS.
  Cancel publishes to /cancel_nav (Empty) and zero /cmd_vel.

  Waypoint replay sends sequential /goal_pose goals, looping
  until cancelled. Status tracked via /nav_status.

===============================================================
KEYBOARD CONTROLS (works with or without Xbox controller)
===============================================================

  MOVEMENT:
    W                     : Forward  (at current max linear speed)
    S                     : Backward (at current max linear speed)
    A                     : Rotate Left  (at current max angular speed)
    D                     : Rotate Right (at current max angular speed)
    W+A / W+D             : Forward + turn (simultaneous)

  SAFETY:
    Space                 : E-Stop ON
    Shift + Space         : E-Stop OFF

  NAVIGATION:
    X                     : Set current robot location as home
    Y                     : Navigate to home position (via /goal_pose)
    C                     : Cancel navigation / replay / line-follow
    B                     : Reset home to map origin (0, 0, 0 deg)

  WAYPOINT & AUTONOMOUS:
    R                     : Toggle recording (press to start/stop)
    T                     : Toggle waypoint replay (C to stop)
    L                     : Toggle line following (C to stop)

  TUNING:
    Up / Down arrow       : Increase / decrease max linear speed
    Left / Right arrow    : Increase / decrease max angular speed

  The pygame window must be focused to receive keyboard input.
  If an Xbox controller is also connected, both work simultaneously.

===============================================================
USAGE
===============================================================

  python3 xbox_leo_unified.py                 # controller + keyboard
  python3 xbox_leo_unified.py --keyboard-only # keyboard only (no controller required)
  python3 xbox_leo_unified.py --debug-axes    # print raw axis values
  python3 xbox_leo_unified.py --skip-checks   # skip startup checks

===============================================================
PC-side Python dependencies
===============================================================

  Ubuntu : ROS 2 Jazzy + pip install pygame
  Windows: ROS 2 Jazzy via pixi + pip install pygame
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Empty, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import pygame
import math
import sys
import signal
import os
import platform
import time
import json

# ======================== Configuration ========================

CMD_VEL_TOPIC        = "/cmd_vel"
CMD_VEL_TELEOP_TOPIC = "/cmd_vel_teleop"
E_STOP_TOPIC         = "/e_stop"
GOAL_POSE_TOPIC      = "/goal_pose"
CANCEL_NAV_TOPIC     = "/cancel_nav"
NAV_STATUS_TOPIC     = "/nav_status"
NAV_MODE_TOPIC      = "/nav_mode"

# Speed limits (initial - adjustable via D-Pad at runtime)
MAX_LINEAR_SPEED  = 1.0   # m/s
MAX_ANGULAR_SPEED = 1.5   # rad/s

# Speed adjustment step per D-Pad press
SPEED_STEP_LINEAR  = 0.1  # m/s per click
SPEED_STEP_ANGULAR = 0.1  # rad/s per click

# Hard ceiling
SPEED_CEILING_LINEAR  = 2.0
SPEED_CEILING_ANGULAR = 3.0

# Deadzone for analog sticks
STICK_DEADZONE = 0.15

# Threshold for "right stick moved" -> E-Stop trigger
ESTOP_STICK_THRESHOLD = 0.20

# ---------- Button mappings (pygame indices) ----------
X_BUTTON             = 2   # X  -> Set home to current location
Y_BUTTON             = 3   # Y  -> Navigate to home
A_BUTTON             = 0   # A  -> Cancel navigation / replay / line-follow
B_BUTTON             = 1   # B  -> Reset home to map origin
LB_BUTTON            = 4   # LB -> Start waypoint recording (press)
RB_BUTTON            = 5   # RB -> Stop waypoint recording (press)
LEFT_STICK_CLICK     = 9   # Left stick press -> E-Stop OFF (release)
RIGHT_STICK_CLICK    = 10  # Right stick press -> E-Stop ON

# Trigger "press" threshold (analog 0-1 range after normalization)
TRIGGER_PRESS_THRESHOLD = 0.50  # treat trigger as "pressed" above this

# ---------- Axis mappings ----------
# Verified on Ubuntu 24.04 with xpad driver via --debug-axes:
#   0=LX  1=LY  2=LT  3=RT  4=RY  5=(unused)
# Triggers rest at -1.0, fully pressed = +1.0

if platform.system() == "Windows":
    LEFT_STICK_X   = 0
    LEFT_STICK_Y   = 1
    LEFT_TRIGGER   = 2
    RIGHT_TRIGGER  = 5   # May differ on Windows; verify with --debug-axes
    RIGHT_STICK_X  = 4   # May differ on Windows
    RIGHT_STICK_Y  = 3   # May differ on Windows
else:  # Linux / Ubuntu (xpad driver, verified)
    LEFT_STICK_X   = 0
    LEFT_STICK_Y   = 1
    LEFT_TRIGGER   = 2   # LT -> forward
    RIGHT_TRIGGER  = 3   # RT -> backward
    RIGHT_STICK_X  = 4   # Checked for E-Stop (may overlap with Y on some drivers)
    RIGHT_STICK_Y  = 4   # Verified via --debug-axes
    # NOTE: If your controller reports separate X and Y axes for the right
    # stick, update RIGHT_STICK_X to the correct index. For E-Stop both
    # are checked, so even if X == Y the E-Stop still triggers correctly.

# D-Pad hat index
DPAD_HAT = 0

# ---------- Navigation watchdog ----------
# Auto-cancel navigation if the rover stalls or times out.
# This ensures teleop is returned to the user automatically.
NAV_TIMEOUT       = 120.0  # seconds — max navigation duration (matches simple_nav_node)
NAV_STALL_TIMEOUT = 8.0    # seconds — cancel if rover hasn't moved this long
NAV_STALL_DIST    = 0.05   # meters  — minimum movement to count as "not stalled"
NAV_STALL_CHECK   = 1.0    # seconds — how often to sample position for stall check

# ---------- Waypoint persistence ----------
# Save/load waypoint list as JSON alongside this script
WAYPOINT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'waypoint_list.json')

# ======================== Startup Health Checks ========================

class HealthCheckError(Exception):
    """Raised when a critical ROS2 health check fails."""
    pass


def run_startup_checks(timeout_sec: float = 15.0):
    """
    Verify critical ROS2 services are running before starting the controller.
    Creates a temporary node, checks topics and TF, then destroys it.

    Checks:
      1. /cmd_vel  has at least one subscriber (motor controller listening)
      2. /scan     has at least one publisher  (LiDAR driver running)
      3. TF chain  map->odom (SLAM running) and odom->base_link (EKF running)
      4. /goal_pose has at least one subscriber (navigation node listening)

    Raises HealthCheckError with details if any critical check fails.
    """
    print("=" * 60)
    print("  ROS2 Pre-Flight Health Checks")
    print("=" * 60)
    print()

    # Create a temporary node for checking
    check_node = rclpy.create_node('_preflight_check')

    # Give DDS time to discover topics across the network
    print(f"  Waiting for DDS topic discovery ({timeout_sec:.0f}s max)...")
    print()

    # We need a TF buffer to check transforms
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, check_node)

    # Spin briefly to populate topic graph and TF buffer
    discovery_start = time.time()
    while time.time() - discovery_start < min(timeout_sec, 8.0):
        rclpy.spin_once(check_node, timeout_sec=0.5)

    results = {}
    critical_failures = []
    warnings = []

    # --- Check 1: /cmd_vel subscribers (motor controller) ---
    cmd_vel_info = check_node.get_publishers_info_by_topic(CMD_VEL_TOPIC)
    cmd_vel_subs = check_node.get_subscriptions_info_by_topic(CMD_VEL_TOPIC)
    if len(cmd_vel_subs) > 0:
        results['cmd_vel'] = ('PASS', f'{len(cmd_vel_subs)} subscriber(s)')
    else:
        results['cmd_vel'] = ('FAIL', 'No subscribers - rover motor controller not running')
        critical_failures.append(
            '/cmd_vel has no subscribers.\n'
            '    -> Is leo_system running on the Pi5?\n'
            '    -> Check: ros2 topic info /cmd_vel'
        )

    # --- Check 2: /scan publishers (LiDAR) ---
    scan_pubs = check_node.get_publishers_info_by_topic('/scan')
    if len(scan_pubs) > 0:
        results['scan'] = ('PASS', f'{len(scan_pubs)} publisher(s)')
    else:
        results['scan'] = ('FAIL', 'No publishers - LiDAR driver not running')
        critical_failures.append(
            '/scan has no publishers.\n'
            '    -> Is sllidar_node running on the Pi5?\n'
            '    -> Check: ros2 topic list | grep scan'
        )

    # --- Check 3: TF chain (SLAM + EKF) ---
    # Check map->odom (SLAM toolbox)
    slam_ok = False
    try:
        t = tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time(),
                                        timeout=rclpy.duration.Duration(seconds=2.0))
        slam_ok = True
        results['tf_slam'] = ('PASS', 'map->odom transform available')
    except TransformException:
        results['tf_slam'] = ('FAIL', 'map->odom not available - SLAM not running')
        critical_failures.append(
            'TF: map->odom transform not found.\n'
            '    -> Is slam_toolbox running on the Pi5?\n'
            '    -> Check: ros2 run tf2_ros tf2_monitor map odom'
        )

    # Check odom->base_link (EKF / odometry filter)
    ekf_ok = False
    try:
        t = tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(),
                                        timeout=rclpy.duration.Duration(seconds=2.0))
        ekf_ok = True
        results['tf_ekf'] = ('PASS', 'odom->base_link transform available')
    except TransformException:
        results['tf_ekf'] = ('FAIL', 'odom->base_link not available - EKF not running')
        critical_failures.append(
            'TF: odom->base_link transform not found.\n'
            '    -> Is odom_filter (robot_localization) running?\n'
            '    -> Check: ros2 run tf2_ros tf2_echo odom base_link'
        )

    # --- Check 4: /goal_pose subscribers (navigation node) ---
    goal_subs = check_node.get_subscriptions_info_by_topic(GOAL_POSE_TOPIC)
    if len(goal_subs) > 0:
        results['goal_pose'] = ('PASS', f'{len(goal_subs)} subscriber(s)')
    else:
        results['goal_pose'] = ('WARN', 'No subscribers - navigation node not running')
        warnings.append(
            '/goal_pose has no subscribers.\n'
            '    -> simple_nav_node or Nav2 bt_navigator not running.\n'
            '    -> Y button (navigate home) will publish but nothing will act on it.\n'
            '    -> Teleop driving will still work.'
        )

    # --- Check 5: /nav_status publishers (simple_nav_node feedback) ---
    nav_status_pubs = check_node.get_publishers_info_by_topic(NAV_STATUS_TOPIC)
    if len(nav_status_pubs) > 0:
        results['nav_status'] = ('PASS', f'{len(nav_status_pubs)} publisher(s)')
    else:
        results['nav_status'] = ('INFO', 'No publishers - no navigation feedback available')

    # Destroy temporary node
    tf_listener.unregister()
    check_node.destroy_node()

    # --- Print results ---
    for name, (status, detail) in results.items():
        icon = {'PASS': '+', 'FAIL': 'X', 'WARN': '!', 'INFO': '-'}[status]
        label = {'PASS': 'PASS', 'FAIL': 'FAIL', 'WARN': 'WARN', 'INFO': 'INFO'}[status]
        print(f"  [{icon}] {label:4s}  {name:15s}  {detail}")

    print()

    # --- Report warnings ---
    if warnings:
        print("  WARNINGS:")
        for w in warnings:
            for line in w.split('\n'):
                print(f"    {line}")
        print()

    # --- Report critical failures ---
    if critical_failures:
        print("  CRITICAL FAILURES:")
        for f in critical_failures:
            for line in f.split('\n'):
                print(f"    {line}")
        print()
        print("=" * 60)
        print("  ABORTING: Critical ROS2 services are not running.")
        print("  Fix the issues above and try again.")
        print("  Or use --skip-checks to bypass (teleop may not work).")
        print("=" * 60)
        raise HealthCheckError(
            f"{len(critical_failures)} critical check(s) failed"
        )

    print("=" * 60)
    if warnings:
        print("  All critical checks passed (with warnings). Starting controller.")
    else:
        print("  All checks passed. Starting controller.")
    print("=" * 60)
    print()


# ==============================================================

class XboxLeoTeleopNav(Node):
    def __init__(self, debug_axes: bool = False, keyboard_only: bool = False):
        super().__init__('xbox_leo_teleop_nav')

        self._debug_axes = debug_axes
        self._keyboard_only = keyboard_only

        # ---- ROS 2 publishers ----
        # /cmd_vel: direct to motors (used ONLY for E-Stop zero / emergency)
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # /cmd_vel_teleop: manual driving (filtered by simple_nav_node before motors)
        self.teleop_pub = self.create_publisher(Twist, CMD_VEL_TELEOP_TOPIC, 10)

        # /goal_pose: RELIABLE + VOLATILE (default QoS)
        # simple_nav_node subscribes with matching default QoS.
        # Nav2 bt_navigator is not used — no need for BEST_EFFORT.
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_POSE_TOPIC, 10)

        # Cancel navigation publisher for simple_nav_node (/cancel_nav)
        self.cancel_nav_pub = self.create_publisher(Empty, CANCEL_NAV_TOPIC, 10)

        # Waypoint replay command publisher (JSON payload to Pi5)
        self.waypoint_replay_pub = self.create_publisher(String, '/waypoint_replay', 10)

        # Line follow command publisher (JSON payload to Pi5)
        self.line_follow_pub = self.create_publisher(String, '/line_follow_cmd', 10)

        # E-stop publisher with "latched" behaviour (transient local)
        e_stop_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.e_stop_pub = self.create_publisher(Bool, E_STOP_TOPIC, e_stop_qos)

        # ---- Navigation status subscriber (from simple_nav_node) ----
        self._last_nav_status = "idle"
        self._nav_status_msg = ""
        # Match simple_nav_node's publisher: RELIABLE + TRANSIENT_LOCAL
        # Ensures terminal status messages (arrived, cancelled) are not
        # lost over WiFi, and late-join gets the last status.
        nav_status_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.nav_status_sub = self.create_subscription(
            String, NAV_STATUS_TOPIC, self._nav_status_callback, nav_status_qos
        )

        # ---- Slope mode subscriber (from slope_pilot_node) ----
        self._nav_mode = "slam"  # "slam", "dead_reckon", "line_follow"
        self.nav_mode_sub = self.create_subscription(
            String, NAV_MODE_TOPIC, self._nav_mode_callback, 10
        )

        # ---- TF2 for current position ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Pygame init ----
        pygame.init()
        pygame.joystick.init()

        # ---- Joystick (optional) ----
        self.joystick = None
        if not keyboard_only and pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f"Controller: {self.joystick.get_name()}")
            self.get_logger().info(
                f"  Axes: {self.joystick.get_numaxes()}  "
                f"Buttons: {self.joystick.get_numbuttons()}  "
                f"Hats: {self.joystick.get_numhats()}"
            )
        elif keyboard_only:
            self.get_logger().info("Keyboard-only mode (--keyboard-only)")
        else:
            self.get_logger().info(
                "No Xbox controller found — using keyboard only. "
                "Plug in a controller and restart for dual input."
            )

        # ---- Pygame window for keyboard input ----
        # Keyboard events require a focused display surface.
        self._kbd_window = pygame.display.set_mode((420, 120))
        pygame.display.set_caption("Leo Rover Teleop — WASD + XYBC")
        self._draw_kbd_window()

        # ---- E-Stop state ----
        self.e_stop_active = False
        self._publish_e_stop_unconditional(False)  # publish initial safe state

        # ---- Button edge-detection state ----
        self.button_states: dict[int, bool] = {
            X_BUTTON: False,
            Y_BUTTON: False,
            A_BUTTON: False,
            B_BUTTON: False,
            LB_BUTTON: False,
            RB_BUTTON: False,
            LEFT_STICK_CLICK: False,
            RIGHT_STICK_CLICK: False,
        }
        self._prev_dpad = (0, 0)

        # ---- Speed limits (mutable at runtime via D-Pad) ----
        self.max_linear_speed  = MAX_LINEAR_SPEED
        self.max_angular_speed = MAX_ANGULAR_SPEED

        # ---- Home position (default = map origin) ----
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw = 0.0
        self.home_is_custom = False

        # ---- Navigation state tracking ----
        self._nav_active = False
        self._nav_start_time = None          # when Y was pressed
        self._nav_last_check_time = None     # last stall check timestamp
        self._nav_last_pos = (0.0, 0.0)      # (x, y) at last stall check
        self._nav_stall_start = None         # when the rover stopped moving

        # ---- Waypoint recording ----
        # Waypoints are typed tuples:
        #   ("nav", x, y, yaw)           — flat ground, absolute SLAM coordinate
        #   ("slope", speed, duration)    — slope, line-follow at speed for duration
        self._waypoints = []                 # list of typed waypoint tuples
        self._waypoint_recording = False     # LB/R currently held = recording
        self._record_last_time = 0.0         # last 10Hz record timestamp
        self._record_last_slope_time = 0.0   # when the current slope segment began

        # Track current teleop speed and steering (for waypoint recording)
        self._teleop_cmd_speed = 0.0
        self._teleop_cmd_steer = 0.0

        # Trigger edge detection (analog → digital press)
        self._lt_was_pressed = False
        self._rt_was_pressed = False

        # Line follow state
        self._line_follow_active = False  # True while Pi5 is line following
        self._replay_active = False       # True while Pi5 is replaying waypoints

        # ---- Keyboard edge-detection state ----
        # Tracks which action keys were pressed last frame to detect rising edges
        self._kbd_prev: dict[int, bool] = {}

        # ---- Shutdown guard ----
        self._shutdown_done = False

        # ---- Main loop timer at ~50 Hz ----
        self.timer = self.create_timer(0.02, self.teleop_loop)

        # ---- Graceful shutdown ----
        signal.signal(signal.SIGINT, self._signal_handler)

        self._print_startup_info()

    # ======================= Helpers ==========================

    def _draw_kbd_window(self):
        """Draw a simple status reminder in the pygame keyboard window."""
        if self._kbd_window is None:
            return
        try:
            self._kbd_window.fill((30, 30, 30))
            font = pygame.font.SysFont('monospace', 14)
            lines = [
                "WASD: Move  |  Space: E-Stop  |  Shift+Space: Release",
                "X: Set Home  Y: Go Home  C: Cancel  B: Reset Home",
                "R: Record  T: Replay  L: Line-follow  |  Arrows: Speed",
            ]
            for i, line in enumerate(lines):
                surf = font.render(line, True, (200, 200, 200))
                self._kbd_window.blit(surf, (10, 10 + i * 30))
            pygame.display.flip()
        except pygame.error:
            pass

    def _kbd_pressed(self, key: int, keys) -> bool:
        """Return True on rising edge of a keyboard key (press, not hold).
        Args:
            key: pygame key constant
            keys: pre-read pygame.key.get_pressed() snapshot
        """
        current = keys[key]
        prev = self._kbd_prev.get(key, False)
        self._kbd_prev[key] = current
        if current and not prev:
            return True
        return False

    def _nav_status_callback(self, msg: String):
        """Receive navigation status from simple_nav_node."""
        # Message format: "state: detail text"
        # e.g. "navigating: Heading to (1.00, 2.00)"
        #      "arrived: Goal reached! (1.00, 2.00, 45°)"
        parts = msg.data.split(': ', 1)
        new_status = parts[0] if parts else msg.data
        detail = parts[1] if len(parts) > 1 else ""

        prev = self._last_nav_status
        self._last_nav_status = new_status
        self._nav_status_msg = detail

        # Track whether navigation is actively running
        if new_status in ('navigating', 'rotating', 'driving', 'final_rotate'):
            self._nav_active = True
        elif new_status == 'replaying':
            # Pi5 is executing waypoint replay
            self._replay_active = True
            self._nav_active = False
        elif new_status == 'line_following':
            # Pi5 is doing autonomous line following
            self._line_follow_active = True
            self._nav_active = False
        elif new_status in ('idle', 'arrived', 'cancelled', 'failed',
                            'timeout', 'blocked', 'replay_stopped',
                            'line_follow_stopped'):
            if self._nav_active or self._replay_active or self._line_follow_active:
                self._nav_active = False
                self._nav_start_time = None
                self._nav_stall_start = None

                if new_status == 'replay_stopped':
                    self._replay_active = False
                    self.get_logger().info(
                        f'Waypoint replay stopped: {detail}')
                elif new_status == 'line_follow_stopped':
                    self._line_follow_active = False
                    self.get_logger().info(
                        f'Line following stopped: {detail}')
                elif new_status == 'arrived':
                    self.get_logger().info(
                        'Navigation ARRIVED — teleop resumed')
                elif new_status == 'cancelled':
                    self._replay_active = False
                    self._line_follow_active = False
                    self.get_logger().info(
                        'Navigation cancelled — teleop resumed')
                elif new_status == 'blocked':
                    self.get_logger().warn(
                        f'Navigation BLOCKED by obstacle: {detail} — teleop resumed')
                elif new_status == 'failed':
                    self.get_logger().warn(
                        f'Navigation FAILED: {detail} — teleop resumed')
                elif new_status == 'timeout':
                    self.get_logger().warn(
                        f'Navigation TIMEOUT: {detail} — teleop resumed')

    def _nav_mode_callback(self, msg: String):
        """Track navigation mode from simple_nav_node."""
        if msg.data != self._nav_mode:
            self.get_logger().info(
                f'Nav mode: {self._nav_mode} -> {msg.data}')
        self._nav_mode = msg.data

    def _print_startup_info(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Leo Rover Teleop + Navigation')
        self.get_logger().info('=' * 60)
        if self.joystick:
            self.get_logger().info('XBOX CONTROLLER:')
            self.get_logger().info('  Left Stick     : Forward/Back + Turn (arcade drive)')
            self.get_logger().info('  LT/RT Triggers : Forward / Backward (alternative)')
            self.get_logger().info('  Right Stick    : E-Stop ON  |  L-Stick Click: E-Stop OFF')
            self.get_logger().info('  X/Y/A/B        : Home set / Go / Cancel / Reset')
            self.get_logger().info('  LB / RB        : Start / Stop recording')
            self.get_logger().info('  LT hold / RT   : Replay waypoints / Line follow')
            self.get_logger().info('  L-Stick click  : E-Stop release')
            self.get_logger().info('  D-Pad          : +/- speed tuning')
            self.get_logger().info('')
        self.get_logger().info('KEYBOARD (pygame window must be focused):')
        self.get_logger().info('  W/S            : Forward / Backward')
        self.get_logger().info('  A/D            : Rotate Left / Right')
        self.get_logger().info('  Space          : E-Stop ON  |  Shift+Space: OFF')
        self.get_logger().info('  X/Y/C/B        : Home set / Go / Cancel / Reset')
        self.get_logger().info('  R              : Toggle recording')
        self.get_logger().info('  T / L          : Replay waypoints / Line follow')
        self.get_logger().info('  Arrow keys     : +/- speed tuning')
        self.get_logger().info('')
        self.get_logger().info(
            f'Speeds: linear={self.max_linear_speed:.1f} m/s  '
            f'angular={self.max_angular_speed:.1f} rad/s'
        )
        self.get_logger().info(
            f'Home: ({self.home_x:.2f}, {self.home_y:.2f}, '
            f'{math.degrees(self.home_yaw):.1f} deg)'
        )
        if self._debug_axes:
            self.get_logger().info('*** DEBUG AXES MODE ENABLED ***')
        self.get_logger().info('=' * 60)

    # ---------- E-Stop ----------

    def _publish_e_stop_unconditional(self, state: bool):
        """Publish E-Stop regardless of current state (used at init)."""
        msg = Bool()
        msg.data = state
        self.e_stop_pub.publish(msg)
        self.e_stop_active = state

    def publish_e_stop(self, state: bool):
        """Publish E-Stop only on state change."""
        if state != self.e_stop_active:
            self._publish_e_stop_unconditional(state)
            status = "ENGAGED" if state else "RELEASED"
            self.get_logger().warn(f"E-STOP {status}")
            # If E-Stop engaged while navigating, cancel the navigation
            if state and self._nav_active:
                self.cancel_navigation()

    def _check_right_stick_estop(self) -> bool:
        """Return True if the right stick is moved or clicked -> E-Stop ON."""
        try:
            ry = self.joystick.get_axis(RIGHT_STICK_Y)
            rx = self.joystick.get_axis(RIGHT_STICK_X)
        except pygame.error:
            return False

        # Any tilt beyond threshold triggers E-Stop
        if abs(ry) > ESTOP_STICK_THRESHOLD or abs(rx) > ESTOP_STICK_THRESHOLD:
            return True

        # Right stick click also triggers E-Stop
        if self._button_pressed(RIGHT_STICK_CLICK):
            return True

        return False

    def _get_trigger_pressed(self, trigger_axis) -> bool:
        """Return True if a trigger axis is pressed (above threshold)."""
        try:
            raw = self.joystick.get_axis(trigger_axis)
        except pygame.error:
            return False
        # Normalize from [-1, +1] to [0, 1]
        norm = (raw + 1.0) / 2.0
        return norm >= TRIGGER_PRESS_THRESHOLD

    # ---------- TF2 position ----------

    def get_current_position(self):
        """Return (x, y, yaw) in map frame, or (None, None, None)."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
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
            self.get_logger().warn(f'Could not get position: {ex}')
            return None, None, None

    # ---------- Home management ----------

    def set_home_to_current(self):
        x, y, yaw = self.get_current_position()
        if x is None:
            self.get_logger().error(
                'Cannot set home - TF lookup failed!'
            )
            return
        self.home_x, self.home_y, self.home_yaw = x, y, yaw
        self.home_is_custom = True
        self.get_logger().info(
            f'Home SET -> ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg)'
        )

    def reset_home_to_default(self):
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw = 0.0
        self.home_is_custom = False
        self.get_logger().info('Home RESET -> map origin (0, 0, 0 deg)')

    # ---------- Navigation ----------

    def _make_pose(self, x, y, yaw) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def return_to_home(self):
        """
        Send navigation goal to home position via /goal_pose topic.

        Navigation flow:
          1. This publishes PoseStamped to /goal_pose
          2. simple_nav_node receives it and begins navigating
          3. Status updates arrive on /nav_status
          4. Press A to cancel, or E-Stop to emergency stop

        Auto-cancel (returns teleop to user automatically):
          - simple_nav_node reports arrived/cancelled/timeout via /nav_status
          - Navigation exceeds NAV_TIMEOUT (120s)
          - Rover stalls (< 5cm movement for NAV_STALL_TIMEOUT seconds)
          - TF becomes unavailable for NAV_STALL_TIMEOUT seconds
        """
        # Block on slopes — navigation unreliable during dead reckoning
        if self._nav_mode != "slam":
            self.get_logger().warn(
                f'Cannot navigate home in {self._nav_mode} mode — '
                'need SLAM mode (flat ground + SLAM active).')
            return

        # Cancel active waypoint replay if running
        if self._replay_active:
            self.get_logger().info('Cancelling waypoint replay for go-home')
            self.cancel_navigation()

        # Check if we can determine current position (SLAM is working)
        cur_x, cur_y, cur_yaw = self.get_current_position()
        if cur_x is None:
            self.get_logger().error(
                'Cannot navigate home - TF lookup failed! '
                'Is SLAM running? Check: ros2 run tf2_ros tf2_monitor map odom'
            )
            return

        # Compute distance to home for user info
        dx = self.home_x - cur_x
        dy = self.home_y - cur_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Build and publish the goal pose
        pose = self._make_pose(self.home_x, self.home_y, self.home_yaw)
        self.goal_pub.publish(pose)

        self._nav_active = True
        self._nav_start_time = time.monotonic()
        self._nav_last_check_time = time.monotonic()
        self._nav_last_pos = (cur_x, cur_y)
        self._nav_stall_start = None  # not stalled yet
        label = "custom" if self.home_is_custom else "origin"
        self.get_logger().info(
            f'NAVIGATING HOME ({label}): '
            f'({self.home_x:.2f}, {self.home_y:.2f}, '
            f'{math.degrees(self.home_yaw):.1f} deg) '
            f'[{dist:.2f}m away]'
        )
        self.get_logger().info(
            f'  Press A to cancel, E-Stop (right stick) for emergency stop'
        )

    def cancel_navigation(self):
        """
        Cancel active navigation, waypoint replay, AND line following on Pi5.

        Sends cancel signal via:
          - /cancel_nav (Empty) for simple_nav_node
          - /waypoint_replay stop command for Pi5 replay engine
          - /line_follow_cmd stop command for Pi5 line follower
          - Zero velocity on /cmd_vel as immediate stop
        """
        # Immediate zero velocity
        self.cmd_vel_pub.publish(Twist())

        # Publish cancel to simple_nav_node
        self.cancel_nav_pub.publish(Empty())

        # Stop replay on Pi5
        if self._replay_active:
            stop_msg = String()
            stop_msg.data = json.dumps({"cmd": "stop"})
            self.waypoint_replay_pub.publish(stop_msg)
            self._replay_active = False

        # Stop line following on Pi5
        if self._line_follow_active:
            stop_msg = String()
            stop_msg.data = json.dumps({"cmd": "stop"})
            self.line_follow_pub.publish(stop_msg)
            self._line_follow_active = False

        self._nav_active = False
        self._nav_start_time = None
        self._nav_stall_start = None

        self.get_logger().info('Navigation/replay/line-follow CANCELLED — teleop resumed')

    # ---------- Waypoint Recording & Replay ----------

    def _start_recording(self):
        """Called on LB press: start a new waypoint list."""
        # Cancel active replay or line following if running
        if self._replay_active or self._line_follow_active:
            self.cancel_navigation()

        self._waypoints = []
        self._waypoint_recording = True
        self._record_last_time = 0.0
        self._record_last_slope_time = time.monotonic()
        self.get_logger().info('Recording waypoints... press RB to stop.')

    def _record_tick(self):
        """Called every frame while recording is active. Records at 10Hz.

        Three waypoint types based on current state:
          1. RT held (line following): ("line_follow", speed, duration)
             Steering from camera during replay — not recorded.
          2. SLAM available: ("nav", x, y, yaw)
          3. No SLAM, no line follow: ("dead_reckon", speed, angular_z, duration)
             Records both forward speed AND steering for faithful replay.
        """
        if not self._waypoint_recording:
            return

        now = time.monotonic()
        if (now - self._record_last_time) < 0.10:  # 10Hz
            return
        self._record_last_time = now

        if self._line_follow_active:
            # LINE FOLLOW: record camera line-following segment
            # Steering comes from camera during replay, not recorded here
            speed = abs(self._teleop_cmd_speed) if self._teleop_cmd_speed != 0.0 \
                else self.max_linear_speed * 0.5

            # Merge with previous line_follow segment if same speed
            if (self._waypoints and self._waypoints[-1][0] == "line_follow" and
                    abs(self._waypoints[-1][1] - speed) < 0.05):
                prev = self._waypoints[-1]
                self._waypoints[-1] = ("line_follow", prev[1], prev[2] + 0.1)
            else:
                self._waypoints.append(("line_follow", speed, 0.1))

        else:
            # Try SLAM coordinate first
            x, y, yaw = self.get_current_position()
            if x is not None:
                # NAV: absolute SLAM coordinate
                self._waypoints.append(("nav", x, y, yaw))
            else:
                # DEAD RECKON: record speed + steering + duration
                speed = self._teleop_cmd_speed   # signed (fwd/back)
                steer = self._teleop_cmd_steer   # signed (left/right)

                # Merge with previous dead_reckon if speed AND steer are similar
                if (self._waypoints and self._waypoints[-1][0] == "dead_reckon" and
                        abs(self._waypoints[-1][1] - speed) < 0.05 and
                        abs(self._waypoints[-1][2] - steer) < 0.10):
                    prev = self._waypoints[-1]
                    self._waypoints[-1] = ("dead_reckon", prev[1], prev[2], prev[3] + 0.1)
                else:
                    self._waypoints.append(("dead_reckon", speed, steer, 0.1))

    def _stop_recording(self):
        """Called on RB press: finalize and save."""
        if not self._waypoint_recording:
            return
        self._waypoint_recording = False

        if not self._waypoints:
            self.get_logger().warn('No waypoints recorded (too short?)')
            return

        nav_count = sum(1 for wp in self._waypoints if wp[0] == "nav")
        dr_count = sum(1 for wp in self._waypoints if wp[0] == "dead_reckon")
        lf_count = sum(1 for wp in self._waypoints if wp[0] == "line_follow")
        self.get_logger().info(
            f'Recording complete: {len(self._waypoints)} waypoint(s) '
            f'({nav_count} nav + {dr_count} dead_reckon + {lf_count} line_follow)')
        self._save_waypoints()

    def _find_nearest_nav_index(self) -> int:
        """Find index of nearest nav waypoint to current position."""
        x, y, _ = self.get_current_position()
        if x is None:
            return 0  # fallback to start

        best_idx = 0
        best_dist = float('inf')
        for i, wp in enumerate(self._waypoints):
            if wp[0] == "nav":
                dx = wp[1] - x
                dy = wp[2] - y
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
        return best_idx

    def _send_replay_start(self):
        """
        Called on LT hold: send full waypoint list to Pi5 starting from
        the nearest nav waypoint to current position.
        """
        # If we were recording, save first
        if self._waypoint_recording and self._waypoints:
            self._stop_recording()

        # If no waypoints in memory, try loading from file
        if not self._waypoints:
            if not self._load_waypoints():
                self.get_logger().warn(
                    'No waypoints recorded or saved! Press LB to record first.')
                return

        # Cancel any active go-home navigation or line following
        if self._nav_active or self._line_follow_active:
            self.cancel_nav_pub.publish(Empty())
            self.cmd_vel_pub.publish(Twist())
            self._nav_active = False
            if self._line_follow_active:
                stop_msg = String()
                stop_msg.data = json.dumps({"cmd": "stop"})
                self.line_follow_pub.publish(stop_msg)
                self._line_follow_active = False

        # Find nearest waypoint
        start_idx = self._find_nearest_nav_index()

        # Send full waypoint list + start index to Pi5
        payload = {
            "cmd": "start",
            "waypoints": [list(wp) for wp in self._waypoints],
            "start_index": start_idx,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.waypoint_replay_pub.publish(msg)

        self._replay_active = True
        n = len(self._waypoints)
        nav_count = sum(1 for wp in self._waypoints if wp[0] == "nav")
        dr_count = sum(1 for wp in self._waypoints if wp[0] == "dead_reckon")
        lf_count = sum(1 for wp in self._waypoints if wp[0] == "line_follow")
        self.get_logger().info(
            f'Replay from WP {start_idx + 1}/{n} (nearest). '
            f'({nav_count} nav + {dr_count} dead_reckon + {lf_count} line_follow). '
            f'Release LT to stop.')

    def _send_replay_stop(self):
        """Called on LT release: stop replay on Pi5."""
        if not self._replay_active:
            return
        msg = String()
        msg.data = json.dumps({"cmd": "stop"})
        self.waypoint_replay_pub.publish(msg)
        self._replay_active = False
        self.get_logger().info('Replay stopped (LT released)')

    def _send_line_follow_start(self):
        """
        Called on RT hold: start autonomous line following on Pi5.
        Pi5 drives forward at preset speed, camera provides steering.
        No tilt requirement — works on any terrain if camera sees a line.
        If no camera or no line detected, Pi5 stops the car.
        """
        # Cancel any active replay or go-home (but NOT recording —
        # line follow segments get recorded as waypoints)
        if self._replay_active or self._nav_active:
            self.cancel_navigation()

        # Send line follow start command to Pi5
        # Pi5 checks camera/line availability and stops car if unavailable
        msg = String()
        msg.data = json.dumps({"cmd": "start"})
        self.line_follow_pub.publish(msg)

        self._line_follow_active = True
        self.get_logger().info(
            'Line following requested. Release RT to stop.')

    def _send_line_follow_stop(self):
        """Called on RT release: stop line following on Pi5."""
        if not self._line_follow_active:
            return
        msg = String()
        msg.data = json.dumps({"cmd": "stop"})
        self.line_follow_pub.publish(msg)
        self._line_follow_active = False
        self.get_logger().info('Line following stopped (RT released)')

    def _save_waypoints(self):
        """Save waypoint list to JSON file for persistence across restarts."""
        try:
            data = {
                'version': 1,
                'count': len(self._waypoints),
                'waypoints': [list(wp) for wp in self._waypoints],
            }
            with open(WAYPOINT_FILE, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(
                f'Saved {len(self._waypoints)} waypoint(s) to {WAYPOINT_FILE}')
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')

    def _load_waypoints(self) -> bool:
        """Load waypoint list from JSON file. Returns True if loaded.
        Backward compatible: converts old 'slope' type to 'dead_reckon'."""
        if not os.path.exists(WAYPOINT_FILE):
            self.get_logger().warn(
                f'No saved waypoint file found at {WAYPOINT_FILE}')
            return False
        try:
            with open(WAYPOINT_FILE, 'r') as f:
                data = json.load(f)

            raw = data.get('waypoints', [])
            if not raw:
                self.get_logger().warn('Saved waypoint file is empty')
                return False

            # Convert old formats for backward compatibility:
            #   "slope" 3-tuple → "dead_reckon" 4-tuple (add angular_z=0)
            #   "dead_reckon" 3-tuple → 4-tuple (add angular_z=0)
            converted = []
            for wp in raw:
                wp = tuple(wp)
                if wp[0] == "slope":
                    # Old slope: (type, speed, duration) → (type, speed, 0.0, duration)
                    converted.append(("dead_reckon", wp[1], 0.0, wp[2]))
                elif wp[0] == "dead_reckon" and len(wp) == 3:
                    # Old dead_reckon: (type, speed, duration) → (type, speed, 0.0, duration)
                    converted.append(("dead_reckon", wp[1], 0.0, wp[2]))
                else:
                    converted.append(wp)
            self._waypoints = converted

            nav_count = sum(1 for wp in self._waypoints if wp[0] == 'nav')
            dr_count = sum(1 for wp in self._waypoints if wp[0] == 'dead_reckon')
            lf_count = sum(1 for wp in self._waypoints if wp[0] == 'line_follow')
            self.get_logger().info(
                f'Loaded {len(self._waypoints)} waypoint(s) from file '
                f'({nav_count} nav + {dr_count} dead_reckon + {lf_count} line_follow)')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return False

    # ---------- D-Pad speed tuning ----------

    def _handle_dpad(self):
        if self.joystick.get_numhats() == 0:
            return
        hat = self.joystick.get_hat(DPAD_HAT)
        if hat == self._prev_dpad:
            return  # no change
        self._prev_dpad = hat

        dx, dy = hat  # dx: -1 left, +1 right; dy: +1 up, -1 down

        if dy == 1:  # D-Pad Up
            self.max_linear_speed = min(
                self.max_linear_speed + SPEED_STEP_LINEAR,
                SPEED_CEILING_LINEAR,
            )
            self.get_logger().info(
                f'Max linear speed -> {self.max_linear_speed:.1f} m/s'
            )
        elif dy == -1:  # D-Pad Down
            self.max_linear_speed = max(
                self.max_linear_speed - SPEED_STEP_LINEAR,
                SPEED_STEP_LINEAR,
            )
            self.get_logger().info(
                f'Max linear speed -> {self.max_linear_speed:.1f} m/s'
            )

        if dx == 1:  # D-Pad Right
            self.max_angular_speed = min(
                self.max_angular_speed + SPEED_STEP_ANGULAR,
                SPEED_CEILING_ANGULAR,
            )
            self.get_logger().info(
                f'Max angular speed -> {self.max_angular_speed:.1f} rad/s'
            )
        elif dx == -1:  # D-Pad Left
            self.max_angular_speed = max(
                self.max_angular_speed - SPEED_STEP_ANGULAR,
                SPEED_STEP_ANGULAR,
            )
            self.get_logger().info(
                f'Max angular speed -> {self.max_angular_speed:.1f} rad/s'
            )

    # ---------- Button edge detection ----------

    def _button_pressed(self, button: int) -> bool:
        """Return True on rising edge only (press, not hold)."""
        try:
            current = self.joystick.get_button(button)
        except pygame.error:
            return False
        if current == 1 and not self.button_states.get(button, False):
            self.button_states[button] = True
            return True
        elif current == 0:
            self.button_states[button] = False
        return False

    # =================== Main Loop ============================

    def teleop_loop(self):
        """Main control loop running at ~50 Hz."""
        # Pump pygame event queue (handles both joystick and keyboard)
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.shutdown()
                    return
        except pygame.error:
            if self.joystick:
                self.get_logger().error('Joystick disconnected!')
            self.cmd_vel_pub.publish(Twist())
            return

        # --- Debug axes mode (joystick only) ---
        if self._debug_axes and self.joystick:
            try:
                n = self.joystick.get_numaxes()
                vals = [f'{self.joystick.get_axis(i):+.2f}' for i in range(n)]
                nb = self.joystick.get_numbuttons()
                btns = [self.joystick.get_button(i) for i in range(nb)]
                self.get_logger().info(f'Axes [{n}]: {vals}  Btns: {btns}')
            except pygame.error:
                pass
            return  # don't drive in debug mode

        # ============================================================
        # Read keyboard state ONCE per frame (avoids inconsistent
        # snapshots from multiple get_pressed() calls)
        # ============================================================
        keys = pygame.key.get_pressed()

        # ============================================================
        # E-Stop input (controller OR keyboard)
        # ============================================================

        # Controller: right stick any direction or click -> E-Stop ON
        if self.joystick and self._check_right_stick_estop():
            self.publish_e_stop(True)

        # Controller: left stick click -> E-Stop OFF (release)
        if self.e_stop_active and self.joystick:
            if self._button_pressed(LEFT_STICK_CLICK):
                self.publish_e_stop(False)

        # Keyboard: Space -> E-Stop ON, Shift+Space -> E-Stop OFF
        if self._kbd_pressed(pygame.K_SPACE, keys):
            if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                self.publish_e_stop(False)
            else:
                self.publish_e_stop(True)

        # ============================================================
        # Navigation actions (controller OR keyboard, work during E-Stop)
        # ============================================================

        # Controller buttons (edge-detected)
        if self.joystick:
            if self._button_pressed(X_BUTTON):
                self.set_home_to_current()
            if self._button_pressed(Y_BUTTON):
                self.return_to_home()
            if self._button_pressed(A_BUTTON):
                self.cancel_navigation()
            if self._button_pressed(B_BUTTON):
                self.reset_home_to_default()

        # Keyboard action keys (edge-detected)
        if self._kbd_pressed(pygame.K_x, keys):
            self.set_home_to_current()
        if self._kbd_pressed(pygame.K_y, keys):
            self.return_to_home()
        if self._kbd_pressed(pygame.K_c, keys):
            self.cancel_navigation()
        if self._kbd_pressed(pygame.K_b, keys):
            self.reset_home_to_default()

        # ============================================================
        # Waypoint recording: LB = start, RB = stop (edge-detected)
        # ============================================================

        if self.joystick:
            if self._button_pressed(LB_BUTTON):
                self._start_recording()
            if self._button_pressed(RB_BUTTON):
                self._stop_recording()
        if self._kbd_pressed(pygame.K_r, keys):
            if not self._waypoint_recording:
                self._start_recording()
            else:
                self._stop_recording()  # R toggles on keyboard

        # Continuous recording tick (10Hz while recording is active)
        if self._waypoint_recording:
            self._record_tick()

        # ============================================================
        # Waypoint replay: LT HOLD = replay, release = stop
        # Mutual exclusion: blocked while line following is active
        # ============================================================

        if self.joystick:
            lt_pressed = self._get_trigger_pressed(LEFT_TRIGGER)
            if lt_pressed and not self._lt_was_pressed:
                if not self._line_follow_active:
                    self._send_replay_start()
            elif not lt_pressed and self._lt_was_pressed:
                self._send_replay_stop()
            self._lt_was_pressed = lt_pressed
        # Keyboard T: toggle replay
        if self._kbd_pressed(pygame.K_t, keys):
            if self._replay_active:
                self._send_replay_stop()
            elif not self._line_follow_active:
                self._send_replay_start()

        # ============================================================
        # Line following: RT HOLD = follow, release = stop
        # Mutual exclusion: blocked while replay is active
        # ============================================================

        if self.joystick:
            rt_pressed = self._get_trigger_pressed(RIGHT_TRIGGER)
            if rt_pressed and not self._rt_was_pressed:
                if not self._replay_active:
                    self._send_line_follow_start()
            elif not rt_pressed and self._rt_was_pressed:
                self._send_line_follow_stop()
            self._rt_was_pressed = rt_pressed
        # Keyboard L: toggle line follow
        if self._kbd_pressed(pygame.K_l, keys):
            if self._line_follow_active:
                self._send_line_follow_stop()
            elif not self._replay_active:
                self._send_line_follow_start()

        # ============================================================
        # Speed tuning (controller D-Pad OR keyboard arrow keys)
        # ============================================================

        if self.joystick:
            self._handle_dpad()

        # Keyboard arrow keys (edge-detected)
        if self._kbd_pressed(pygame.K_UP, keys):
            self.max_linear_speed = min(
                self.max_linear_speed + SPEED_STEP_LINEAR,
                SPEED_CEILING_LINEAR,
            )
            self.get_logger().info(
                f'Max linear speed -> {self.max_linear_speed:.1f} m/s')
        if self._kbd_pressed(pygame.K_DOWN, keys):
            self.max_linear_speed = max(
                self.max_linear_speed - SPEED_STEP_LINEAR,
                SPEED_STEP_LINEAR,
            )
            self.get_logger().info(
                f'Max linear speed -> {self.max_linear_speed:.1f} m/s')
        if self._kbd_pressed(pygame.K_RIGHT, keys):
            self.max_angular_speed = min(
                self.max_angular_speed + SPEED_STEP_ANGULAR,
                SPEED_CEILING_ANGULAR,
            )
            self.get_logger().info(
                f'Max angular speed -> {self.max_angular_speed:.1f} rad/s')
        if self._kbd_pressed(pygame.K_LEFT, keys):
            self.max_angular_speed = max(
                self.max_angular_speed - SPEED_STEP_ANGULAR,
                SPEED_STEP_ANGULAR,
            )
            self.get_logger().info(
                f'Max angular speed -> {self.max_angular_speed:.1f} rad/s')

        # ============================================================
        # E-Stop guard: send zero velocity and skip driving
        # ============================================================

        if self.e_stop_active:
            self.cmd_vel_pub.publish(Twist())
            return

        # ============================================================
        # Navigation watchdog (auto-cancel if stalled or timed out)
        # ============================================================

        if self._nav_active:
            now = time.monotonic()

            # Watchdog 1: overall navigation timeout
            elapsed = now - self._nav_start_time if self._nav_start_time else 0
            if elapsed > NAV_TIMEOUT:
                self.get_logger().warn(
                    f'Navigation auto-cancelled: timed out after {elapsed:.0f}s'
                )
                self.cancel_navigation()
                return

            # Watchdog 2: stall detection (rover not moving)
            if now - self._nav_last_check_time >= NAV_STALL_CHECK:
                self._nav_last_check_time = now
                x, y, _ = self.get_current_position()
                if x is not None:
                    dx = x - self._nav_last_pos[0]
                    dy = y - self._nav_last_pos[1]
                    moved = math.sqrt(dx * dx + dy * dy)

                    if moved >= NAV_STALL_DIST:
                        # Rover is moving — reset stall timer
                        self._nav_last_pos = (x, y)
                        self._nav_stall_start = None
                    else:
                        # Rover hasn't moved enough
                        if self._nav_stall_start is None:
                            self._nav_stall_start = now
                        elif now - self._nav_stall_start > NAV_STALL_TIMEOUT:
                            self.get_logger().warn(
                                f'Navigation auto-cancelled: rover stalled for '
                                f'{NAV_STALL_TIMEOUT:.0f}s '
                                f'(moved {moved:.3f}m < {NAV_STALL_DIST}m threshold)'
                            )
                            self.cancel_navigation()
                            return
                else:
                    # TF lookup failed — count as stalled
                    if self._nav_stall_start is None:
                        self._nav_stall_start = now
                    elif now - self._nav_stall_start > NAV_STALL_TIMEOUT:
                        self.get_logger().warn(
                            'Navigation auto-cancelled: TF unavailable for '
                            f'{NAV_STALL_TIMEOUT:.0f}s'
                        )
                        self.cancel_navigation()
                        return

            # Navigation still active and healthy — let simple_nav_node drive
            return

        # ============================================================
        # Driving input: keyboard WASD takes priority if any held,
        # otherwise fall through to controller triggers + stick
        # (uses 'keys' snapshot from top of loop)
        # ============================================================

        kbd_fwd  = keys[pygame.K_w]
        kbd_back = keys[pygame.K_s]
        kbd_left = keys[pygame.K_a]
        kbd_right = keys[pygame.K_d]
        kbd_active = kbd_fwd or kbd_back or kbd_left or kbd_right

        if kbd_active:
            # Keyboard: binary at current max speed
            throttle = 0.0
            if kbd_fwd:
                throttle += 1.0
            if kbd_back:
                throttle -= 1.0
            steer = 0.0
            if kbd_left:
                steer += 1.0
            if kbd_right:
                steer -= 1.0

            linear_x  = throttle * self.max_linear_speed
            angular_z = steer * self.max_angular_speed

        elif self.joystick:
            # Controller: left stick arcade drive
            #   Left Stick Y (up=forward) + X (left=turn left)
            # Triggers are now used for replay (LT) and line-follow (RT).
            try:
                stick_y = self.joystick.get_axis(LEFT_STICK_Y)   # up = negative
                steer   = -self.joystick.get_axis(LEFT_STICK_X)  # left = positive
            except pygame.error:
                self.get_logger().error('Joystick read error - sending zero vel')
                self.cmd_vel_pub.publish(Twist())
                return

            # Stick Y throttle: up (negative axis) = forward, down = backward
            throttle = -stick_y  # invert so up = positive
            if abs(throttle) < STICK_DEADZONE:
                throttle = 0.0

            if abs(steer) < STICK_DEADZONE:
                steer = 0.0

            linear_x  = throttle * self.max_linear_speed
            angular_z = steer * self.max_angular_speed

        else:
            # No input — zero velocity
            linear_x  = 0.0
            angular_z = 0.0

        # Publish to /cmd_vel_teleop (simple_nav_node filters for obstacles)
        twist = Twist()
        twist.linear.x  = float(linear_x)
        twist.angular.z = float(angular_z)
        self.teleop_pub.publish(twist)

        # Track current drive speed and steering for waypoint recording
        self._teleop_cmd_speed = float(linear_x)
        self._teleop_cmd_steer = float(angular_z)

    # =================== Shutdown =============================

    def _signal_handler(self, sig, frame):
        self.shutdown()

    def shutdown(self):
        """Safe shutdown - idempotent, can be called multiple times."""
        if self._shutdown_done:
            return
        self._shutdown_done = True

        self.get_logger().info(
            'Shutting down - zero velocity, cancelling navigation, releasing E-Stop'
        )
        try:
            # Cancel any active navigation
            self.cancel_nav_pub.publish(Empty())
            self.cmd_vel_pub.publish(Twist())
            self._publish_e_stop_unconditional(False)
        except Exception:
            pass

        try:
            pygame.quit()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        sys.exit(0)


# ========================== Entry =============================

def main(args=None):
    # Set ROS domain ID (adjust for your rover if needed)
    os.environ.setdefault('ROS_DOMAIN_ID', '0')

    # Uncomment for better WiFi reliability:
    # os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    rclpy.init(args=args)

    debug_axes = '--debug-axes' in sys.argv
    skip_checks = '--skip-checks' in sys.argv
    keyboard_only = '--keyboard-only' in sys.argv

    # --- Pre-flight health checks ---
    if not skip_checks and not debug_axes:
        try:
            run_startup_checks(timeout_sec=15.0)
        except HealthCheckError as e:
            print(f"\nStartup aborted: {e}")
            try:
                rclpy.shutdown()
            except Exception:
                pass
            sys.exit(1)
    elif skip_checks:
        print("  --skip-checks: Bypassing ROS2 health checks")
        print()

    node = XboxLeoTeleopNav(debug_axes=debug_axes, keyboard_only=keyboard_only)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
