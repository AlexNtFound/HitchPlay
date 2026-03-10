#!/usr/bin/env bash
# start_all.sh - Launch all Leo Rover ROS2 components with smart health checks
# =============================================================================
# Revised 2025: Navigation handled by lightweight simple_nav_node (~2% CPU).
# No Nav2 on Pi5 — keeps CPU load under 60%.
# =============================================================================

set -euo pipefail

# Store PIDs and log files
pids=()
log_dir="/tmp/rover_logs"
rm -rf "$log_dir"
mkdir -p "$log_dir"

# Persistent log setup
persistent_log_dir="$HOME/leo_ws/src/rover_logs_persistent"
mkdir -p "$persistent_log_dir"
persistent_log="$persistent_log_dir/startup_$(date +%Y%m%d_%H%M%S).log"

# Log function - writes to both console and persistent log
log() {
    echo "$1"
    echo "[$(date +%H:%M:%S)] $1" >> "$persistent_log"
}

cleanup_done=false

# =============================================================================
# Startup Safety Check
# =============================================================================

log "Checking for leftover processes from previous runs..."
log "Cleaning up leftover processes and ports..."

# Kill processes launched by start_all.sh only
# DO NOT kill leo_system, odom_filter, imu_filter, robot_state_publisher,
# firmware_message_converter, web_video_server — these are managed by systemd
# (ros-nodes.service / leo_bringup.launch.xml) and will NOT restart if killed.
pkill -KILL -f "slam_toolbox" 2>/dev/null || true
pkill -KILL -f "sllidar_node" 2>/dev/null || true
pkill -KILL -f "simple_nav_node" 2>/dev/null || true
pkill -KILL -f "tilt_gate_node" 2>/dev/null || true
pkill -KILL -f "slope_pilot_node" 2>/dev/null || true
pkill -KILL -f "static_transform_publisher" 2>/dev/null || true
pkill -KILL -f "rosbridge" 2>/dev/null || true
pkill -KILL -f "drive_service" 2>/dev/null || true
pkill -KILL -f "uvicorn" 2>/dev/null || true
pkill -KILL -f "api-server/main.py" 2>/dev/null || true

# Kill by port (catches processes missed by name matching)
fuser -k 8000/tcp 2>/dev/null || true   # FastAPI
fuser -k 9090/tcp 2>/dev/null || true   # ROSBridge WebSocket

# Clean SLAM shared memory only (NOT generic fastrtps — systemd nodes use those)
rm -f /dev/shm/sem.slam_toolbox* 2>/dev/null || true
rm -f /dev/shm/slam_toolbox* 2>/dev/null || true

sleep 3
log "  ✓ Cleanup complete"

echo ""

# =============================================================================
# Helper Functions
# =============================================================================

launch_and_wait() {
    local description="$1"
    local command="$2"
    local wait_pattern="$3"
    local timeout="${4:-30}"
    local log_file="$log_dir/$(echo "$description" | tr ' ' '_').log"

    log "Starting: $description"
    (
        set +u
        setsid bash -c "$command" 2>&1 | tee "$log_file"
    ) &
    local pid=$!
    pids+=($pid)

    if [ -n "$wait_pattern" ]; then
        echo "  Waiting for: $wait_pattern"
        local elapsed=0
        while [ $elapsed -lt $timeout ]; do
            if grep -q "$wait_pattern" "$log_file" 2>/dev/null; then
                log "  ✓ Ready! (${elapsed}s)"
                return 0
            fi
            sleep 1
            elapsed=$((elapsed + 1))
            if ! kill -0 $pid 2>/dev/null; then
                log "  ✗ Process died! Check logs: $log_file"
                tail -10 "$log_file" 2>/dev/null | tee -a "$persistent_log" || true
                return 1
            fi
        done
        log "  ✗ Timeout after ${timeout}s (continuing anyway...)"
        tail -10 "$log_file" 2>/dev/null | tee -a "$persistent_log" || true
        return 1
    fi

    return 0
}

cleanup() {
    if [ "$cleanup_done" = true ]; then
        return
    fi
    cleanup_done=true
    echo ""
    echo "=========================================="
    echo "Stopping all processes..."
    echo "=========================================="
    echo "[$(date +%H:%M:%S)] Shutdown initiated" >> "$persistent_log"

    # Send zero velocity to stop the rover immediately
    set +u
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    set -u
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true

    echo "  Stopping ROSBridge WebSocket..."
    pkill -INT -f "rosbridge" 2>/dev/null || true
    sleep 2

    echo "  Stopping FastAPI server..."
    pkill -INT -f "main.py" 2>/dev/null || true
    sleep 1

    echo "  Stopping Drive service..."
    pkill -INT -f "drive_service" 2>/dev/null || true
    sleep 1

    echo "  Stopping simple navigation node..."
    pkill -INT -f "simple_nav_node" 2>/dev/null || true
    sleep 1

    echo "  Stopping tilt gate node..."
    pkill -INT -f "tilt_gate_node" 2>/dev/null || true
    sleep 1

    echo "  Stopping slope pilot node..."
    pkill -INT -f "slope_pilot_node" 2>/dev/null || true
    sleep 1

    echo "  Stopping SLAM toolbox..."
    pkill -INT -f "slam_toolbox" 2>/dev/null || true
    sleep 5
    pkill -KILL -f "slam_toolbox" 2>/dev/null || true
    sleep 2

    echo "  Stopping transform publisher..."
    pkill -INT -f "static_transform_publisher" 2>/dev/null || true
    sleep 1

    echo "  Stopping LiDAR..."
    pkill -INT -f "sllidar_node" 2>/dev/null || true
    sleep 8
    pkill -KILL -f "sllidar_node" 2>/dev/null || true

    # NOTE: Do NOT kill leo_system, odom_filter, imu_filter, robot_state_publisher,
    # firmware_message_converter, web_video_server. These are managed by systemd.

    for pid in "${pids[@]}"; do
        kill -KILL -$pid 2>/dev/null || true
    done

    # Only kill ros2 launch/run started by THIS script
    pkill -KILL -f "ros2 launch sllidar" 2>/dev/null || true
    pkill -KILL -f "ros2 launch slam_toolbox" 2>/dev/null || true
    pkill -KILL -f "ros2 launch rosbridge" 2>/dev/null || true
    pkill -KILL -f "ros2 run custom_drive_pkg" 2>/dev/null || true
    pkill -P $$ 2>/dev/null || true

    rm -f /dev/shm/sem.slam_toolbox* 2>/dev/null || true
    rm -f /dev/shm/slam_toolbox* 2>/dev/null || true

    echo "Logs saved in: $log_dir"
    echo "Cleanup complete"
    echo "[$(date +%H:%M:%S)] Cleanup complete" >> "$persistent_log"
}

trap cleanup SIGINT SIGTERM EXIT

# =============================================================================
# Check for LiDAR
# =============================================================================

LIDAR_CONNECTED=false
if [ -e /dev/ttyUSB0 ] || [ -e /dev/ttyUSB1 ]; then
    LIDAR_CONNECTED=true
    log "✓ LiDAR detected"
else
    log "⚠ LiDAR not detected - skipping LiDAR driver, SLAM, obstacle avoidance"
    log "  Navigation will use IMU dead-reckoning only (no SLAM localization)"
fi

echo ""

# =============================================================================
# Core Robot Systems
# =============================================================================

LEO_OK=false
LIDAR_OK=false
SLAM_OK=false
TILTGATE_OK=false
SLOPEPILOT_OK=false
SIMPLENAV_OK=false
DRIVE_OK=false
FASTAPI_OK=false
ROSBRIDGE_OK=false
WEB_VIDEO_OK=false

launch_and_wait "Leo base system" \
    'source /opt/ros/jazzy/setup.bash; set -u;
     if ros2 node list 2>/dev/null | grep -q "/leo_system"; then
       echo "Leo system node started! (already running via systemd)";
     else
       ros2 run leo_bringup leo_system;
     fi' \
    "Leo system node started!" \
    15 && LEO_OK=true

# =============================================================================
# Sensors & Navigation (Conditional on LiDAR)
# =============================================================================

if [ "$LIDAR_CONNECTED" = true ]; then

    # =========================================================================
    # LiDAR driver (5 Hz via custom launch file - see INSTRUCTIONS.md Sec 4)
    # =========================================================================
    launch_and_wait "LiDAR driver (5 Hz)" \
        'source ~/ws_lidar/install/setup.bash; set -u;
         ros2 launch sllidar_ros2 view_sllidar_a2m12_nogui_launch.py' \
        "current scan mode" \
        15 && LIDAR_OK=true

    launch_and_wait "LiDAR transform (base_link → laser)" \
        'source /opt/ros/jazzy/setup.bash; set -u;
         ros2 run tf2_ros static_transform_publisher \
           --x 0.03 --y 0 --z 0.02 --yaw 3.14159 --pitch 0 --roll 0 \
           --frame-id base_link --child-frame-id laser' \
        "Spinning until stopped - publishing transform" \
        10

    # Let LiDAR and transforms stabilize
    log "  Stabilizing LiDAR and transforms (8 seconds)..."
    sleep 8

    # =========================================================================
    # Tilt Gate Node (gates LiDAR scans on slopes for SLAM stability)
    # When rover tilts >10 deg, suppresses scans so SLAM freezes
    # and EKF dead-reckons with IMU+wheels until flat again.
    # =========================================================================
    launch_and_wait "Tilt gate node" \
        'source /opt/ros/jazzy/setup.bash; set -u;
         python3 ~/leo_ws/src/tilt_gate_node.py' \
        "Tilt Gate Node" \
        10 && TILTGATE_OK=true

    # =========================================================================
    # SLAM Toolbox (localization + mapping)
    # Uses default config (subscribes to /scan, base_frame=base_footprint).
    # Tilt gate publishes /scan_gated for future use but SLAM reads /scan directly.
    # =========================================================================
    launch_and_wait "SLAM Toolbox" \
        'source /opt/ros/jazzy/setup.bash; set -u;
         ros2 launch slam_toolbox online_async_launch.py \
           use_sim_time:=false' \
        "Registering sensor" \
        30 && SLAM_OK=true

    # Let SLAM build initial map before starting navigation
    log "  Stabilizing SLAM (10 seconds)..."
    sleep 10

    # =========================================================================
    # Verify SLAM TF health before proceeding
    # =========================================================================
    log "  Checking SLAM TF health..."
    set +u
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    set -u
    TF_CHECK=$(timeout 5 ros2 run tf2_ros tf2_echo map odom --wait-for-transform 3 2>&1 || true)
    if echo "$TF_CHECK" | grep -q "Translation"; then
        log "  ✓ SLAM TF (map→odom) is publishing"
    else
        log "  ⚠ SLAM TF (map→odom) not yet available - navigation may fail initially"
    fi

fi  # end LIDAR_CONNECTED

# =============================================================================
# Core Navigation Nodes (always run — degrade gracefully without LiDAR/SLAM)
# =============================================================================

# =========================================================================
# Slope Pilot Node (camera line detection for slope navigation)
# =========================================================================
# Requires: python3-opencv, ros-jazzy-cv-bridge (mandatory packages).
# Camera hardware is OPTIONAL — if no camera publishers detected on
# /camera/image_rect_color, node runs in IMU-only mode (flat/dead_reckon,
# no line_follow). Camera is re-detected every 5s for hot-plug support.
# If node fails entirely, simple_nav_node falls back to its own IMU tilt.
# =========================================================================
launch_and_wait "Slope pilot node" \
    'source /opt/ros/jazzy/setup.bash; set -u;
     python3 ~/leo_ws/src/slope_pilot_node.py' \
    "Slope Pilot Node" \
    10 && SLOPEPILOT_OK=true

if [ "$SLOPEPILOT_OK" != true ]; then
    log "  ⚠ Slope pilot node failed — simple_nav_node will use IMU fallback"
fi

# =========================================================================
# Simple Navigation Node
# Provides teleop passthrough, go-home, waypoint replay, obstacle avoidance.
# Without LiDAR: obstacle avoidance disabled, teleop still works.
# Without SLAM: coordinate navigation disabled, dead-reckoning only.
# =========================================================================
launch_and_wait "Simple navigation node" \
    'source /opt/ros/jazzy/setup.bash; set -u;
     python3 ~/leo_ws/src/simple_nav_node.py' \
    "Simple Navigation Node" \
    10 && SIMPLENAV_OK=true

# =========================================================================
# Drive Service
# =========================================================================
launch_and_wait "Drive service" \
    'source /opt/ros/jazzy/setup.bash;
     source ~/leo_ws/install/setup.bash; set -u;
     ros2 run custom_drive_pkg drive_service' \
    "Drive Service Ready" \
    15 && DRIVE_OK=true

# =============================================================================
# API & Communication
# =============================================================================

launch_and_wait "FastAPI server" \
    'cd ~/projects/api-server; python3 main.py' \
    "Uvicorn running on" \
    30 && FASTAPI_OK=true

launch_and_wait "ROSBridge WebSocket server" \
    'source /opt/ros/jazzy/setup.bash; set -u;
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml' \
    "Rosbridge WebSocket server started on port" \
    40 && ROSBRIDGE_OK=true

# Web video server is launched by leo_bringup.launch.xml (systemd).
# Just verify it's running — do NOT launch a duplicate.
if pgrep -f "web_video_server" > /dev/null 2>&1; then
    log "Starting: Web video server"
    log "  ✓ Ready! (already running via systemd)"
    WEB_VIDEO_OK=true
else
    log "  ⚠ Web video server not detected — camera streaming unavailable"
fi

# =============================================================================
# Post-Startup LiDAR Frequency Verification
# =============================================================================

if [ "$LIDAR_OK" = true ]; then
    log ""
    log "Verifying LiDAR scan frequency..."
    set +u
    source ~/ws_lidar/install/setup.bash 2>/dev/null || true
    set -u
    # Sample /scan topic for 5 seconds to measure actual Hz
    HZ_OUTPUT=$(timeout 6 ros2 topic hz /scan --window 5 2>&1 || true)
    ACTUAL_HZ=$(echo "$HZ_OUTPUT" | grep -oP 'average rate: \K[0-9.]+' | head -1)
    if [ -n "$ACTUAL_HZ" ]; then
        # Check if frequency is above 7 Hz (should be ~5 Hz)
        TOO_FAST=$(echo "$ACTUAL_HZ > 7.0" | bc -l 2>/dev/null || echo "0")
        if [ "$TOO_FAST" = "1" ]; then
            log "  ⚠ LiDAR running at ${ACTUAL_HZ} Hz (expected ~5 Hz)"
            log "    SLAM may fall behind! Check scan_frequency param in launch file."
            log "    See INSTRUCTIONS.md Section 4 for fix."
        else
            log "  ✓ LiDAR scan rate: ${ACTUAL_HZ} Hz"
        fi
    else
        log "  ⚠ Could not measure LiDAR frequency"
    fi
fi

# =============================================================================
# Success Summary
# =============================================================================

echo ""
echo "=========================================="
echo "Startup Summary"
echo "=========================================="
echo "=========================================" >> "$persistent_log"
echo "Startup Summary" >> "$persistent_log"

[ "$LEO_OK" = true ]       && log "  ✓ Leo base system"          || log "  ✗ Leo base system"

if [ "$LIDAR_CONNECTED" = true ]; then
    [ "$LIDAR_OK" = true ]     && log "  ✓ LiDAR driver"             || log "  ✗ LiDAR driver"
    [ "$TILTGATE_OK" = true ]  && log "  ✓ Tilt gate node"          || log "  ✗ Tilt gate node"
    [ "$SLAM_OK" = true ]      && log "  ✓ SLAM Toolbox"             || log "  ✗ SLAM Toolbox"
else
    log "  ⚠ LiDAR not connected (no obstacle avoidance, no SLAM)"
fi

[ "$SLOPEPILOT_OK" = true ] && log "  ✓ Slope pilot node"        || log "  ⚠ Slope pilot node (IMU fallback)"
[ "$SIMPLENAV_OK" = true ] && log "  ✓ Simple navigation node"   || log "  ✗ Simple navigation node"
[ "$DRIVE_OK" = true ]     && log "  ✓ Drive service"            || log "  ✗ Drive service"

[ "$FASTAPI_OK" = true ]   && log "  ✓ FastAPI server"            || log "  ✗ FastAPI server"
[ "$ROSBRIDGE_OK" = true ] && log "  ✓ ROSBridge WebSocket"       || log "  ✗ ROSBridge WebSocket"
[ "$WEB_VIDEO_OK" = true ] && log "  ✓ Web video server"          || log "  ✗ Web video server"

echo "=========================================="
echo "=========================================" >> "$persistent_log"

if [ "$LIDAR_CONNECTED" = true ]; then
    echo ""
    log "Navigation: SLAM + obstacle avoidance + pure pursuit replay"
else
    echo ""
    log "Navigation: IMU dead-reckoning only (no LiDAR, no SLAM, no obstacle avoidance)"
fi

echo ""
echo "Logs (temp):       $log_dir"
echo "Logs (persistent): $persistent_log"
echo "Press Ctrl-C to stop"
echo ""

wait
