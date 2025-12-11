#!/usr/bin/env bash
# start_all.sh - Launch all Leo Rover ROS2 navigation components with smart health checks
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
if pgrep -f "slam_toolbox" > /dev/null 2>&1 || pgrep -f "nav2" > /dev/null 2>&1 || pgrep -f "sllidar_node" > /dev/null 2>&1; then
    log "  Found leftover processes - cleaning up..."
    
    pkill -KILL -f "slam_toolbox" 2>/dev/null || true
    pkill -KILL -f "nav2" 2>/dev/null || true
    pkill -KILL -f "sllidar_node" 2>/dev/null || true
    pkill -KILL -f "static_transform_publisher" 2>/dev/null || true
    pkill -KILL -f "leo_system" 2>/dev/null || true
    pkill -KILL -f "rosbridge" 2>/dev/null || true
    pkill -KILL -f "drive_service" 2>/dev/null || true
    
    rm -f /dev/shm/sem.slam_toolbox* 2>/dev/null || true
    rm -f /dev/shm/slam_toolbox* 2>/dev/null || true
    rm -rf /tmp/fastrtps_* 2>/dev/null || true
    rm -rf /tmp/.ros 2>/dev/null || true
    
    sleep 3
    log "  ✓ Cleanup complete"
else
    log "  ✓ No leftover processes found"
fi
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
    
    echo "  Stopping Nav2..."
    pkill -INT -f "nav2" 2>/dev/null || true
    sleep 3
    
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
    
    echo "  Stopping Leo base system..."
    pkill -INT -f "leo_system" 2>/dev/null || true
    sleep 2
    
    for pid in "${pids[@]}"; do
        kill -KILL -$pid 2>/dev/null || true
    done
    
    pkill -KILL -f "ros2 launch" 2>/dev/null || true
    pkill -KILL -f "ros2 run" 2>/dev/null || true
    pkill -P $$ 2>/dev/null || true
    
    rm -f /dev/shm/sem.slam_toolbox* 2>/dev/null || true
    rm -f /dev/shm/slam_toolbox* 2>/dev/null || true
    rm -rf /tmp/fastrtps_* 2>/dev/null || true
    
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
    log "⚠ LiDAR not detected - skipping SLAM and Nav2"
fi
echo ""

# =============================================================================
# Core Robot Systems
# =============================================================================
LEO_OK=false
LIDAR_OK=false
SLAM_OK=false
NAV2_OK=false
DRIVE_OK=false
FASTAPI_OK=false
ROSBRIDGE_OK=false

launch_and_wait "Leo base system" \
    'source /opt/ros/jazzy/setup.bash; set -u; ros2 run leo_bringup leo_system' \
    "Leo system node started!" \
    15 && LEO_OK=true

# =============================================================================
# Sensors (Conditional)
# =============================================================================
if [ "$LIDAR_CONNECTED" = true ]; then
    launch_and_wait "LiDAR driver" \
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
    # Navigation Stack
    # =========================================================================
    launch_and_wait "SLAM Toolbox" \
        'source /opt/ros/jazzy/setup.bash; set -u;
        cd ~/leo_ws/src/LeoRover-SLAM-ROS2;
        ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false' \
        "Registering sensor" \
        30 && SLAM_OK=true
    
    # Let SLAM stabilize before Nav2
    log "  Stabilizing SLAM (10 seconds)..."
    sleep 10

    launch_and_wait "Nav2 navigation stack" \
        'source /opt/ros/jazzy/setup.bash;
         source ~/leo_ws/install/setup.bash; set -u;
         ros2 launch nav2_bringup navigation_launch.py \
             params_file:="$HOME/leo_ws/src/LeoRover-SLAM-ROS2/nav2_simple.yaml" \
             slam:=true' \
        "Creating bond timer" \
        30 && NAV2_OK=true

    sleep 5

    # =========================================================================
    # Drive Service
    # =========================================================================
    launch_and_wait "Drive service" \
        'source /opt/ros/jazzy/setup.bash;
         source ~/leo_ws/install/setup.bash; set -u;
         ros2 run custom_drive_pkg drive_service' \
        "Drive Service Ready" \
        15 && DRIVE_OK=true
fi

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

# =============================================================================
# Success Summary
# =============================================================================
echo ""
echo "=========================================="
echo "Startup Summary"
echo "=========================================="
echo "=========================================" >> "$persistent_log"
echo "Startup Summary" >> "$persistent_log"

[ "$LEO_OK" = true ] && log "  ✓ Leo base system" || log "  ✗ Leo base system"
if [ "$LIDAR_CONNECTED" = true ]; then
    [ "$LIDAR_OK" = true ] && log "  ✓ LiDAR driver" || log "  ✗ LiDAR driver"
    [ "$SLAM_OK" = true ] && log "  ✓ SLAM Toolbox" || log "  ✗ SLAM Toolbox"
    [ "$NAV2_OK" = true ] && log "  ✓ Nav2 navigation" || log "  ✗ Nav2 navigation"
    [ "$DRIVE_OK" = true ] && log "  ✓ Drive service" || log "  ✗ Drive service"
fi
[ "$FASTAPI_OK" = true ] && log "  ✓ FastAPI server" || log "  ✗ FastAPI server"
[ "$ROSBRIDGE_OK" = true ] && log "  ✓ ROSBridge WebSocket" || log "  ✗ ROSBridge WebSocket"

echo "=========================================="
echo "=========================================" >> "$persistent_log"
echo ""
echo "Logs (temp): $log_dir"
echo "Logs (persistent): $persistent_log"
echo "Press Ctrl-C to stop"
echo ""

wait
