#!/usr/bin/env python3
"""
Slope Pilot Node for Leo Rover
=================================

Detects slope/banking via IMU tilt and provides line-following steering
when a white center line is visible on the blue floor.

Three modes (published on /slope_mode):
  "flat"         - rover is level, LiDAR SLAM active, full manual control
  "line_follow"  - tilted + white line detected, auto-steer follows line
  "dead_reckon"  - tilted + no line visible, full manual control, IMU-only

Line detection:
  - Camera image is resized to 320x240 for speed
  - Bottom half of image (floor region) is analyzed
  - HSV filter isolates white pixels against blue/dark background
  - Contour centroid offset from image center -> steering command
  - Proportional controller: steer to keep line centered

Subscribes to:
  /imu/data          (sensor_msgs/Imu)     - tilt detection
  /camera/image_rect_color  (sensor_msgs/Image)   - floor line detection

Publishes to:
  /slope_mode   (std_msgs/String)  - current mode for simple_nav_node
  /line_steer   (std_msgs/Float64) - angular.z steering when in line_follow

Requirements:
  pip install opencv-python  (or apt install python3-opencv)
  apt install ros-jazzy-cv-bridge

Usage:
  python3 slope_pilot_node.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import String, Float64

import math
import time
import cv2
import numpy as np
from cv_bridge import CvBridge


# ======================== Configuration ========================

# Tilt threshold (must match tilt_gate_node)
TILT_THRESHOLD_DEG = 10.0
TILT_HYSTERESIS_DEG = 2.0

# Camera processing
PROCESS_WIDTH  = 320
PROCESS_HEIGHT = 240
FLOOR_ROI_TOP  = 0.50  # analyze bottom 50% of image (floor region)

# White line detection (HSV thresholds)
# White on blue/dark floor: high value, low saturation
WHITE_H_MIN, WHITE_H_MAX = 0, 180      # any hue
WHITE_S_MIN, WHITE_S_MAX = 0, 60       # low saturation
WHITE_V_MIN, WHITE_V_MAX = 180, 255    # high value (bright)

# Minimum contour area to count as a line (filters noise)
MIN_LINE_AREA = 200  # pixels at 320x240

# Line-following steering
KP_STEER = 2.0           # proportional gain (tunable)
MAX_STEER = 1.0           # max angular.z rad/s
LINE_CENTER_DEADZONE = 15 # pixels from center: no steering needed

# Processing rate (don't process every frame)
PROCESS_INTERVAL = 0.10  # seconds (10 Hz max)

# Mode persistence: require N consecutive detections/losses before switching
LINE_DETECT_COUNT = 2    # frames with line before entering line_follow
LINE_LOST_COUNT   = 5    # frames without line before exiting line_follow

# Camera hardware detection
CAMERA_TOPIC = '/camera/image_rect_color'
CAMERA_CHECK_INTERVAL = 5.0  # seconds between camera availability checks

# ==============================================================


class SlopePilotNode(Node):
    def __init__(self):
        super().__init__('slope_pilot_node')

        # Camera hardware detected at runtime (not package availability)
        self._camera_available = False
        self._camera_last_check = 0.0

        # ---- Publishers (always created) ----
        self.mode_pub = self.create_publisher(String, '/slope_mode', 10)
        self.steer_pub = self.create_publisher(Float64, '/line_steer', 10)

        # ---- Subscribers ----
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Camera subscription is created lazily — only when tilted AND camera available.
        # This avoids deserializing full camera frames (~20% CPU) when flat.
        self.image_sub = None  # created on tilt, destroyed on flat

        # ---- CV Bridge ----
        self.bridge = CvBridge()

        # ---- State ----
        self._current_roll = 0.0
        self._current_pitch = 0.0
        self._mode = "flat"
        self._prev_mode = "flat"

        self._tilt_rad = math.radians(TILT_THRESHOLD_DEG)
        self._resume_rad = math.radians(TILT_THRESHOLD_DEG - TILT_HYSTERESIS_DEG)
        self._is_tilted = False

        # Line detection state
        self._line_seen_count = 0
        self._line_lost_count = 0
        self._last_process_time = 0.0

        # Publish initial mode
        self._publish_mode("flat")

        # Check camera hardware at startup
        self._check_camera_hardware()

        self.get_logger().info('=' * 55)
        self.get_logger().info('  Slope Pilot Node')
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'  Tilt threshold: {TILT_THRESHOLD_DEG:.0f} deg')
        self.get_logger().info(f'  Line detect:    white on blue/dark floor')
        self.get_logger().info(f'  Steer gain:     Kp={KP_STEER}')
        if self._camera_available:
            self.get_logger().info(f'  Camera:         DETECTED on {CAMERA_TOPIC}')
            self.get_logger().info(f'  Modes: flat / line_follow / dead_reckon')
        else:
            self.get_logger().warn(f'  Camera:         NOT DETECTED on {CAMERA_TOPIC}')
            self.get_logger().warn(f'  Modes: flat / dead_reckon (will retry every {CAMERA_CHECK_INTERVAL:.0f}s)')
        self.get_logger().info('=' * 55)

    # =================== Helpers ==========================

    @staticmethod
    def quaternion_to_rpy(q):
        """Extract roll, pitch from quaternion."""
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        return roll, pitch

    def _check_camera_hardware(self):
        """Check if camera topic has active publishers (hardware available)."""
        now = time.monotonic()
        if (now - self._camera_last_check) < CAMERA_CHECK_INTERVAL:
            return  # don't check too often
        self._camera_last_check = now

        try:
            pubs = self.get_publishers_info_by_topic(CAMERA_TOPIC)
            was_available = self._camera_available
            self._camera_available = len(pubs) > 0

            if self._camera_available and not was_available:
                self.get_logger().info(
                    f'Camera DETECTED on {CAMERA_TOPIC} — line_follow enabled')
            elif not self._camera_available and was_available:
                self.get_logger().warn(
                    f'Camera LOST on {CAMERA_TOPIC} — switching to dead_reckon only')
                # Tear down camera subscription if active
                if self.image_sub is not None:
                    self.destroy_subscription(self.image_sub)
                    self.image_sub = None
                if self._is_tilted:
                    self._publish_mode("dead_reckon")
                    self._publish_steer(0.0)
        except Exception:
            pass  # topic introspection not available yet during early startup

    def _publish_mode(self, mode: str):
        """Publish mode and log on change."""
        self._mode = mode
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

        if mode != self._prev_mode:
            roll_deg = math.degrees(abs(self._current_roll))
            pitch_deg = math.degrees(abs(self._current_pitch))
            self.get_logger().info(
                f'Mode: {self._prev_mode} -> {mode} '
                f'(roll={roll_deg:.1f}, pitch={pitch_deg:.1f})')
            self._prev_mode = mode

    def _publish_steer(self, angular_z: float):
        """Publish steering command."""
        msg = Float64()
        msg.data = float(angular_z)
        self.steer_pub.publish(msg)

    # =================== Line Detection ===================

    def _detect_line(self, cv_image):
        """
        Detect white center line in camera image.

        Returns:
            (found, offset) where:
              found: bool - white line detected
              offset: float - normalized offset from center [-1.0, +1.0]
                      negative = line is left, positive = line is right
        """
        # Resize for speed
        frame = cv2.resize(cv_image, (PROCESS_WIDTH, PROCESS_HEIGHT))

        # Crop to bottom half (floor region)
        roi_top = int(PROCESS_HEIGHT * FLOOR_ROI_TOP)
        roi = frame[roi_top:, :]
        roi_h, roi_w = roi.shape[:2]

        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Threshold for white
        lower_white = np.array([WHITE_H_MIN, WHITE_S_MIN, WHITE_V_MIN])
        upper_white = np.array([WHITE_H_MAX, WHITE_S_MAX, WHITE_V_MAX])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Morphological cleanup
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return False, 0.0

        # Find largest contour (most likely the line)
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < MIN_LINE_AREA:
            return False, 0.0

        # Centroid of the line
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return False, 0.0

        cx = int(M["m10"] / M["m00"])

        # Normalize offset: 0 = center, -1 = far left, +1 = far right
        center_x = roi_w // 2
        offset = (cx - center_x) / center_x  # [-1.0, +1.0]

        return True, offset

    # =================== Callbacks ========================

    def imu_callback(self, msg: Imu):
        """Update tilt state from IMU. Manages camera subscription lifecycle."""
        self._current_roll, self._current_pitch = self.quaternion_to_rpy(
            msg.orientation
        )
        tilt = max(abs(self._current_roll), abs(self._current_pitch))

        # Periodically recheck camera hardware (detects hot-plug or late startup)
        self._check_camera_hardware()

        # Tilt state with hysteresis
        if not self._is_tilted and tilt > self._tilt_rad:
            self._is_tilted = True
            self._line_seen_count = 0
            self._line_lost_count = 0

            if self._camera_available:
                # Subscribe to camera (lazy — avoids 20% CPU on flat ground)
                if self.image_sub is None:
                    self.image_sub = self.create_subscription(
                        Image, CAMERA_TOPIC, self.image_callback, 10
                    )
                    self.get_logger().info('Tilted — camera subscription activated')
            else:
                # No camera hardware: go straight to dead_reckon
                self._publish_mode("dead_reckon")
                self._publish_steer(0.0)

        elif self._is_tilted and tilt < self._resume_rad:
            self._is_tilted = False
            self._line_seen_count = 0
            self._line_lost_count = 0
            self._publish_mode("flat")
            self._publish_steer(0.0)
            # Unsubscribe from camera (saves CPU on flat ground)
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
                self.get_logger().info('Flat — camera subscription deactivated')

        elif self._is_tilted and not self._camera_available:
            # No camera hardware: keep publishing dead_reckon so simple_nav_node
            # knows we're alive (prevents its IMU fallback from taking over)
            now = time.monotonic()
            if (now - self._last_process_time) >= 0.20:
                self._last_process_time = now
                self._publish_mode("dead_reckon")

    def image_callback(self, msg: Image):
        """Process camera image for line detection (only called when tilted)."""

        # Rate limit processing
        now = time.monotonic()
        if (now - self._last_process_time) < PROCESS_INTERVAL:
            return
        self._last_process_time = now

        # Convert ROS image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return

        # Detect white line
        found, offset = self._detect_line(cv_image)

        if found:
            self._line_seen_count += 1
            self._line_lost_count = 0

            if self._line_seen_count >= LINE_DETECT_COUNT:
                # Line confirmed -- enter line_follow mode
                self._publish_mode("line_follow")

                # Proportional steering: steer toward line center
                if abs(offset) * (PROCESS_WIDTH // 2) < LINE_CENTER_DEADZONE:
                    steer = 0.0
                else:
                    # Negative offset (line left) -> steer left (positive angular.z)
                    # Positive offset (line right) -> steer right (negative angular.z)
                    steer = -KP_STEER * offset
                    steer = max(-MAX_STEER, min(MAX_STEER, steer))

                self._publish_steer(steer)

        else:
            self._line_lost_count += 1
            self._line_seen_count = 0

            if self._line_lost_count >= LINE_LOST_COUNT:
                # Line lost -- switch to dead reckoning
                self._publish_mode("dead_reckon")
                self._publish_steer(0.0)


# ========================== Entry =============================

def main(args=None):
    rclpy.init(args=args)

    node = SlopePilotNode()
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
