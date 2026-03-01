#!/usr/bin/env python3
"""
Xbox Controller Teleop + Navigation for Leo Rover (ROS 2 Jazzy)
Works on both Windows and Ubuntu 24.04
Tested with Xbox One/Series wired controller

===============================================================
CONTROLLER BUTTON / AXIS MAP
===============================================================

  MOVEMENT:
    Left Trigger (LT)    : Forward  (proportional throttle)
    Right Trigger (RT)    : Backward (proportional throttle)
    Left Stick X-axis     : Rotate Left / Right (angular velocity)

  SAFETY:
    Right Stick (any dir) : E-Stop ON  (any tilt or click)
    Right Stick Click     : E-Stop ON  (press = instant stop)
    LT + RT fully pressed : E-Stop OFF (hold both triggers >90%)

  NAVIGATION:
    X button              : Set current robot location as home
    Y button              : Navigate to home position (Nav2 goal)
    A button              : Cancel active navigation (robot stops)
    B button              : Reset home to map origin (0, 0, 0°)

  TUNING (D-Pad, live while driving):
    D-Pad Up / Down       : Increase / decrease max linear speed
    D-Pad Left / Right    : Increase / decrease max angular speed

  NOT USED:
    Nexus / Guide button  : (unmapped)
    Start button          : (unmapped)
    RB / LB buttons       : (unmapped)
    Left Stick Click      : (unmapped)

===============================================================
AXIS MAPPING (Ubuntu 24.04 xpad driver + pygame, verified)
===============================================================

  Axis 0 : Left Stick X       Axis 1 : Left Stick Y
  Axis 2 : Left Trigger (LT)  Axis 3 : Right Trigger (RT)
  Axis 4 : Right Stick Y      Axis 5 : (unused)

  Triggers rest at -1.0 and go to +1.0 when fully pressed.
  Use --debug-axes to print raw values and verify your mapping.

===============================================================
USAGE
===============================================================

  ros2 run <your_package> xbox_leo_unified
  python3 xbox_leo_unified.py
  python3 xbox_leo_unified.py --debug-axes

===============================================================
PC-side Python dependencies
===============================================================

  Ubuntu : install ROS 2 Jazzy + pip install pygame
  Windows: install ROS 2 Jazzy via pixi + pixi add pygame + install pixi Code extension in Code
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

try:
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    HAS_NAV2_ACTION = True
except ImportError:
    HAS_NAV2_ACTION = False

import pygame
import math
import sys
import signal
import os
import platform

# ======================== Configuration ========================

CMD_VEL_TOPIC   = "/cmd_vel"
E_STOP_TOPIC    = "/e_stop"
GOAL_POSE_TOPIC = "/goal_pose"

# Speed limits (initial – adjustable via D-Pad at runtime)
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

# Threshold for "right stick moved" → E-Stop trigger
ESTOP_STICK_THRESHOLD = 0.20

# Threshold for "trigger fully pressed" (normalised 0–1)
TRIGGER_FULL_THRESHOLD = 0.90

# ---------- Button mappings (pygame indices) ----------
X_BUTTON             = 2   # X  → Set home to current location
Y_BUTTON             = 3   # Y  → Navigate to home
A_BUTTON             = 0   # A  → Cancel navigation
B_BUTTON             = 1   # B  → Reset home to map origin
RIGHT_STICK_CLICK    = 10  # Right stick press → E-Stop ON

# ---------- Axis mappings ----------
# Verified on Ubuntu 24.04 with xpad driver via --debug-axes:
#   0=LX  1=LY  2=LT  3=RT  4=RY  5=(unused)
# Triggers rest at -1.0, fully pressed = +1.0

if platform.system() == "Windows":
    LEFT_STICK_X   = 0
    LEFT_TRIGGER   = 2
    RIGHT_TRIGGER  = 5   # May differ on Windows; verify with --debug-axes
    RIGHT_STICK_X  = 4   # May differ on Windows
    RIGHT_STICK_Y  = 3   # May differ on Windows
else:  # Linux / Ubuntu (xpad driver, verified)
    LEFT_STICK_X   = 0
    LEFT_TRIGGER   = 2   # LT → forward
    RIGHT_TRIGGER  = 3   # RT → backward
    RIGHT_STICK_X  = 4   # Checked for E-Stop (may overlap with Y on some drivers)
    RIGHT_STICK_Y  = 4   # Verified via --debug-axes
    # NOTE: If your controller reports separate X and Y axes for the right
    # stick, update RIGHT_STICK_X to the correct index. For E-Stop both
    # are checked, so even if X == Y the E-Stop still triggers correctly.

# D-Pad hat index
DPAD_HAT = 0

# ==============================================================


class XboxLeoTeleopNav(Node):
    def __init__(self, debug_axes: bool = False):
        super().__init__('xbox_leo_teleop_nav')

        self._debug_axes = debug_axes

        # ---- ROS 2 publishers ----
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_POSE_TOPIC, 10)

        # E-stop publisher with "latched" behaviour (transient local)
        e_stop_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.e_stop_pub = self.create_publisher(Bool, E_STOP_TOPIC, e_stop_qos)

        # ---- Nav2 action client (optional, for proper cancel) ----
        self._nav_client = None
        self._nav_goal_handle = None
        if HAS_NAV2_ACTION:
            self._nav_client = ActionClient(
                self, NavigateToPose, 'navigate_to_pose'
            )

        # ---- TF2 for current position ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Pygame / joystick init ----
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error(
                "No Xbox controller found! Plug it in and try again."
            )
            sys.exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Connected to: {self.joystick.get_name()}")
        self.get_logger().info(
            f"  Axes: {self.joystick.get_numaxes()}  "
            f"Buttons: {self.joystick.get_numbuttons()}  "
            f"Hats: {self.joystick.get_numhats()}"
        )

        # ---- E-Stop state ----
        self.e_stop_active = False
        self._publish_e_stop_unconditional(False)  # publish initial safe state

        # ---- Button edge-detection state ----
        self.button_states: dict[int, bool] = {
            X_BUTTON: False,
            Y_BUTTON: False,
            A_BUTTON: False,
            B_BUTTON: False,
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

        # ---- Shutdown guard ----
        self._shutdown_done = False

        # ---- Main loop timer at ~50 Hz ----
        self.timer = self.create_timer(0.02, self.teleop_loop)

        # ---- Graceful shutdown ----
        signal.signal(signal.SIGINT, self._signal_handler)

        self._print_startup_info()

    # ======================= Helpers ==========================

    def _print_startup_info(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Xbox Leo Rover Teleop + Navigation')
        self.get_logger().info('=' * 60)
        self.get_logger().info('MOVEMENT:')
        self.get_logger().info('  Left Trigger   : Forward (proportional)')
        self.get_logger().info('  Right Trigger  : Backward (proportional)')
        self.get_logger().info('  Left Stick X   : Rotate Left / Right')
        self.get_logger().info('')
        self.get_logger().info('SAFETY:')
        self.get_logger().info('  Right Stick    : E-Stop ON (any movement or click)')
        self.get_logger().info('  LT + RT fully  : E-Stop OFF (hold both >90%)')
        self.get_logger().info('')
        self.get_logger().info('NAVIGATION:')
        self.get_logger().info('  X Button       : Set current location as home')
        self.get_logger().info('  Y Button       : Navigate to home')
        self.get_logger().info('  A Button       : Cancel active navigation')
        self.get_logger().info('  B Button       : Reset home to map origin')
        self.get_logger().info('')
        self.get_logger().info('TUNING (D-Pad):')
        self.get_logger().info('  Up / Down      : +/- max linear speed')
        self.get_logger().info('  Left / Right   : +/- max angular speed')
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

    def _check_right_stick_estop(self) -> bool:
        """Return True if the right stick is moved or clicked → E-Stop ON."""
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

    def _check_dual_trigger_release(self) -> bool:
        """Return True if BOTH triggers are fully pressed → E-Stop OFF."""
        try:
            lt_raw = self.joystick.get_axis(LEFT_TRIGGER)
            rt_raw = self.joystick.get_axis(RIGHT_TRIGGER)
        except pygame.error:
            return False

        # Normalise from [-1, +1] to [0, 1]
        lt_norm = (lt_raw + 1.0) / 2.0
        rt_norm = (rt_raw + 1.0) / 2.0

        return lt_norm >= TRIGGER_FULL_THRESHOLD and rt_norm >= TRIGGER_FULL_THRESHOLD

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
                'Cannot set home – TF lookup failed!'
            )
            return
        self.home_x, self.home_y, self.home_yaw = x, y, yaw
        self.home_is_custom = True
        self.get_logger().info(
            f'Home SET → ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg)'
        )

    def reset_home_to_default(self):
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw = 0.0
        self.home_is_custom = False
        self.get_logger().info('Home RESET → map origin (0, 0, 0 deg)')

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
        """Send Nav2 goal to home position."""
        pose = self._make_pose(self.home_x, self.home_y, self.home_yaw)

        # Try proper Nav2 action client first, fall back to topic
        if self._nav_client is not None and self._nav_client.server_is_ready():
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = pose
            future = self._nav_client.send_goal_async(nav_goal)
            future.add_done_callback(self._nav_goal_response_cb)
            self.get_logger().info('Nav2 goal sent via action client')
        else:
            self.goal_pub.publish(pose)
            self.get_logger().info('Nav2 goal published to topic (action server unavailable)')

        label = "custom" if self.home_is_custom else "origin"
        self.get_logger().info(
            f'Returning home ({label}): '
            f'({self.home_x:.2f}, {self.home_y:.2f}, '
            f'{math.degrees(self.home_yaw):.1f} deg)'
        )

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal was REJECTED')
            self._nav_goal_handle = None
            return
        self._nav_goal_handle = goal_handle
        self.get_logger().info('Nav2 goal ACCEPTED')

    def cancel_navigation(self):
        """Cancel the active Nav2 goal properly."""
        if self._nav_goal_handle is not None:
            self.get_logger().info('Cancelling Nav2 goal via action client ...')
            cancel_future = self._nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_cb)
        else:
            # Fallback: publish zero-velocity to at least stop motion
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(
                'Navigation cancel: no active goal handle – sent zero velocity'
            )

    def _cancel_done_cb(self, future):
        self.get_logger().info('Nav2 goal cancel acknowledged')
        self._nav_goal_handle = None

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
                f'Max linear speed → {self.max_linear_speed:.1f} m/s'
            )
        elif dy == -1:  # D-Pad Down
            self.max_linear_speed = max(
                self.max_linear_speed - SPEED_STEP_LINEAR,
                SPEED_STEP_LINEAR,
            )
            self.get_logger().info(
                f'Max linear speed → {self.max_linear_speed:.1f} m/s'
            )

        if dx == 1:  # D-Pad Right
            self.max_angular_speed = min(
                self.max_angular_speed + SPEED_STEP_ANGULAR,
                SPEED_CEILING_ANGULAR,
            )
            self.get_logger().info(
                f'Max angular speed → {self.max_angular_speed:.1f} rad/s'
            )
        elif dx == -1:  # D-Pad Left
            self.max_angular_speed = max(
                self.max_angular_speed - SPEED_STEP_ANGULAR,
                SPEED_STEP_ANGULAR,
            )
            self.get_logger().info(
                f'Max angular speed → {self.max_angular_speed:.1f} rad/s'
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
        # Pump pygame event queue
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
        except pygame.error:
            self.get_logger().error('Joystick disconnected!')
            self.cmd_vel_pub.publish(Twist())
            return

        # --- Debug axes mode ---
        if self._debug_axes:
            try:
                n = self.joystick.get_numaxes()
                vals = [f'{self.joystick.get_axis(i):+.2f}' for i in range(n)]
                nb = self.joystick.get_numbuttons()
                btns = [self.joystick.get_button(i) for i in range(nb)]
                self.get_logger().info(f'Axes [{n}]: {vals}  Btns: {btns}')
            except pygame.error:
                pass
            return  # don't drive in debug mode

        # --- E-Stop: right stick any direction or click → ON ---
        if self._check_right_stick_estop():
            self.publish_e_stop(True)

        # --- E-Stop release: both triggers fully pressed → OFF ---
        if self.e_stop_active and self._check_dual_trigger_release():
            self.publish_e_stop(False)

        # --- Button actions (work even during E-Stop) ---
        if self._button_pressed(X_BUTTON):
            self.set_home_to_current()

        if self._button_pressed(Y_BUTTON):
            self.return_to_home()

        if self._button_pressed(A_BUTTON):
            self.cancel_navigation()

        if self._button_pressed(B_BUTTON):
            self.reset_home_to_default()

        # --- D-Pad speed tuning (works even during E-Stop) ---
        self._handle_dpad()

        # --- E-Stop guard: send zero velocity and skip driving ---
        if self.e_stop_active:
            self.cmd_vel_pub.publish(Twist())
            return

        # --- Read triggers and stick for driving ---
        try:
            lt_raw = self.joystick.get_axis(LEFT_TRIGGER)    # forward
            rt_raw = self.joystick.get_axis(RIGHT_TRIGGER)   # backward
            steer  = -self.joystick.get_axis(LEFT_STICK_X)   # left = positive
        except pygame.error:
            self.get_logger().error('Joystick read error – sending zero vel')
            self.cmd_vel_pub.publish(Twist())
            return

        # Normalise triggers: -1.0 (rest) → 0.0, +1.0 (full) → 1.0
        forward  = (lt_raw + 1.0) / 2.0
        backward = (rt_raw + 1.0) / 2.0
        throttle = forward - backward  # positive = forward, negative = backward

        # Apply deadzone to steering only (triggers have natural zero at rest)
        if abs(steer) < STICK_DEADZONE:
            steer = 0.0

        # Scale to current max speeds
        linear_x  = throttle * self.max_linear_speed
        angular_z = steer * self.max_angular_speed

        # Publish
        twist = Twist()
        twist.linear.x  = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    # =================== Shutdown =============================

    def _signal_handler(self, sig, frame):
        self.shutdown()

    def shutdown(self):
        """Safe shutdown – idempotent, can be called multiple times."""
        if self._shutdown_done:
            return
        self._shutdown_done = True

        self.get_logger().info(
            'Shutting down – zero velocity, releasing E-Stop'
        )
        try:
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

    node = XboxLeoTeleopNav(debug_axes=debug_axes)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
