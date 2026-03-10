#!/usr/bin/env python3
"""
Tilt Gate Node for Leo Rover
==============================

Gates LiDAR scans to SLAM based on IMU roll/pitch. When the rover
tilts beyond a threshold (slope or banking), LiDAR scans are
suppressed to prevent SLAM from ingesting distorted scans that would
corrupt the map. During suppression, SLAM freezes its last good
map->odom correction while the EKF continues dead-reckoning with
wheel encoders + IMU.

When the rover returns to near-horizontal, scans are ramped back in
gradually to give SLAM time to re-localize against the existing map.

Architecture:
  /scan -> [tilt_gate_node] -> /scan_gated -> slam_toolbox
               ^
           /imu/data (roll/pitch from Madgwick filter)

  /scan remains available raw for obstacle detection in simple_nav_node.

Gate triggers on:
  - Pitch > threshold (driving up/down a slope)
  - Roll > threshold (banking on a curve)
  - Any combination of the two

Subscribes to:
  /scan     (sensor_msgs/LaserScan)  - raw LiDAR scans
  /imu/data (sensor_msgs/Imu)        - orientation for tilt detection

Publishes to:
  /scan_gated (sensor_msgs/LaserScan) - filtered scans for SLAM

Usage:
  python3 tilt_gate_node.py

  SLAM must subscribe to /scan_gated instead of /scan:
    ros2 launch slam_toolbox online_async_launch.py \
        use_sim_time:=false scan_topic:=/scan_gated
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu

import math
import time


# ======================== Configuration ========================

# Tilt threshold (degrees) -- suppress scans when roll OR pitch exceeds this
TILT_GATE_DEG = 10.0

# Hysteresis (degrees) -- resume scans when tilt drops below (threshold - hysteresis)
# Prevents rapid toggling on the boundary
TILT_HYSTERESIS_DEG = 2.0

# Ramp-in: after gate reopens, only pass 1 in N scans for this many seconds
# Gives SLAM time to re-localize before full scan rate resumes
RAMP_IN_DURATION = 3.0   # seconds
RAMP_IN_DIVISOR  = 3     # pass every 3rd scan during ramp-in (5Hz -> ~1.7Hz)

# Logging throttle -- don't spam the log every scan
LOG_INTERVAL = 2.0  # seconds between repeated gate open/close messages

# ==============================================================


class TiltGateNode(Node):
    def __init__(self):
        super().__init__('tilt_gate_node')

        # ---- Publishers ----
        self.scan_pub = self.create_publisher(LaserScan, '/scan_gated', 10)

        # ---- Subscribers ----
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # ---- State ----
        self._current_roll = 0.0   # radians
        self._current_pitch = 0.0  # radians
        self._gate_open = True     # True = passing scans, False = suppressing
        self._ramp_start = None    # time.monotonic() when gate reopened
        self._scan_count = 0       # counter for ramp-in divisor
        self._last_log_time = 0.0  # throttle repeated log messages

        self._tilt_rad = math.radians(TILT_GATE_DEG)
        self._resume_rad = math.radians(TILT_GATE_DEG - TILT_HYSTERESIS_DEG)

        self.get_logger().info('=' * 55)
        self.get_logger().info('  Tilt Gate Node')
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'  Gate threshold:  {TILT_GATE_DEG:.0f} deg '
                               f'(roll OR pitch)')
        self.get_logger().info(f'  Resume below:    '
                               f'{TILT_GATE_DEG - TILT_HYSTERESIS_DEG:.0f} deg')
        self.get_logger().info(f'  Ramp-in:         '
                               f'{RAMP_IN_DURATION:.0f}s at 1/{RAMP_IN_DIVISOR} rate')
        self.get_logger().info(f'  /scan -> /scan_gated (for slam_toolbox)')
        self.get_logger().info('=' * 55)

    # =================== Helpers ==========================

    @staticmethod
    def quaternion_to_rpy(q):
        """Extract roll, pitch from quaternion."""
        # Roll (x-axis rotation)
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        return roll, pitch

    def _log_throttled(self, msg: str, warn: bool = False):
        """Log a message at most once per LOG_INTERVAL seconds."""
        now = time.monotonic()
        if now - self._last_log_time >= LOG_INTERVAL:
            self._last_log_time = now
            if warn:
                self.get_logger().warn(msg)
            else:
                self.get_logger().info(msg)

    # =================== Callbacks ========================

    def imu_callback(self, msg: Imu):
        """Update current tilt from IMU orientation."""
        self._current_roll, self._current_pitch = self.quaternion_to_rpy(
            msg.orientation
        )

    def scan_callback(self, msg: LaserScan):
        """Gate scan based on current tilt state."""
        roll_deg = math.degrees(abs(self._current_roll))
        pitch_deg = math.degrees(abs(self._current_pitch))
        tilt = max(abs(self._current_roll), abs(self._current_pitch))

        # ---- State transitions ----
        if self._gate_open and tilt > self._tilt_rad:
            # Close gate -- rover is tilted beyond threshold
            self._gate_open = False
            self._ramp_start = None
            self._scan_count = 0
            self.get_logger().warn(
                f'TILT GATE CLOSED: roll={roll_deg:.1f} deg, '
                f'pitch={pitch_deg:.1f} deg '
                f'(threshold={TILT_GATE_DEG:.0f} deg). '
                f'SLAM using last good map->odom, EKF dead-reckoning.')
            self._last_log_time = time.monotonic()
            return  # suppress this scan

        elif not self._gate_open and tilt < self._resume_rad:
            # Reopen gate -- rover is back to near-horizontal
            self._gate_open = True
            self._ramp_start = time.monotonic()
            self._scan_count = 0
            self.get_logger().info(
                f'TILT GATE OPEN: roll={roll_deg:.1f} deg, '
                f'pitch={pitch_deg:.1f} deg. '
                f'Ramping scans back in over {RAMP_IN_DURATION:.0f}s...')
            self._last_log_time = time.monotonic()
            # Fall through to publish this scan

        # ---- Gate closed: suppress all scans ----
        if not self._gate_open:
            self._log_throttled(
                f'Tilt gate closed: roll={roll_deg:.1f}, '
                f'pitch={pitch_deg:.1f} -- scans suppressed',
                warn=True)
            return

        # ---- Gate open: check if we're in ramp-in period ----
        if self._ramp_start is not None:
            elapsed = time.monotonic() - self._ramp_start
            if elapsed < RAMP_IN_DURATION:
                # Only pass every Nth scan during ramp-in
                self._scan_count += 1
                if (self._scan_count % RAMP_IN_DIVISOR) != 1:
                    return  # skip this scan
            else:
                # Ramp-in complete -- resume full rate
                self._ramp_start = None
                self.get_logger().info(
                    'Tilt gate ramp-in complete -- full scan rate resumed')

        # ---- Pass scan through to SLAM ----
        self.scan_pub.publish(msg)


# ========================== Entry =============================

def main(args=None):
    rclpy.init(args=args)
    node = TiltGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
