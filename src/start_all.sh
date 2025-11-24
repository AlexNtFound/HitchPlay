#!/usr/bin/env bash
# start_all.sh - Launch all Leo Rover ROS2 navigation components with smart health checks
set -euo pipefail

# Store PIDs and log files
pids=()
log_dir="/tmp/rover_logs"
rm -rf "$log_dir"  # Clear old logs to prevent false positives
mkdir -p "$log_dir"

# Flag to prevent double cleanup
cleanup_done=false

# =============================================================================
# Startup Safety Check - Clean up any leftover processes from previous runs
# =============================================================================
echo "Checking for leftover processes from previous runs..."
if pgrep -f "slam_toolbox" > /dev/null 2>&1 || pgrep -f "nav2" > /dev/null 2>&1 || pgrep -f "sllidar_node" > /dev/null 2>&1; then
    echo "  Found leftover processes - cleaning up..."
    
    # Kill any leftover processes
    pkill -KILL -f "slam_toolbox" 2>/dev/null || true
    pkill -KILL -f "nav2" 2>/dev/null || true
    pkill -KILL -f "sllidar_node" 2>/dev/null || true
    pkill -KILL -f "static_transform_publisher" 2>/dev/null || true
    pkill -KILL -f "leo_system" 2>/dev/null || true
    pkill -KILL -f "rosbridge" 2>/dev/null || true
    
    # Clean up shared memory and DDS
    echo "  Cleaning shared memory and DDS artifacts..."
    rm -f /dev/shm/sem.slam_toolbox* 2>/dev/null || true
    rm -f /dev/shm/slam_toolbox* 2>/dev/null || true
    rm -rf /tmp/fastrtps_* 2>/dev/null || true
    rm -rf /tmp/.ros 2>/dev/null || true
    
    # Wait for cleanup to complete
    sleep 3
    echo "  ✓ Cleanup complete"
else
    echo "  ✓ No leftover processes found"
fi
echo ""

# =============================================================================
# Helper Functions
# =============================================================================
launch_and_wait() {
    local description="$1"
    local command="$2"
    local wait_pattern="$3"
    local timeout="${4:-30}"  # Timeout in seconds
    local log_file="$log_dir/$(echo "$description" | tr ' ' '_').log"
    
    echo "Starting: $description"
    # Use setsid to create a new process group
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
                echo "  ✓ Ready! (${elapsed}s)"
                return 0
            fi
            sleep 1
            elapsed=$((elapsed + 1))
            
            # Check if process died
            if ! kill -0 $pid 2>/dev/null; then
                echo "  ✗ Process died! Check logs: $log_file"
                cleanup
                exit 1
            fi
        done
        echo "  ✗ Timeout after ${timeout}s waiting for ready signal"
        echo "    Check logs: $log_file"
        cleanup
        exit 1
    fi
}

cleanup() {
    # Prevent double cleanup
    if [ "$cleanup_done" = true ]; then
        return
    fi
    cleanup_done=true
    
    echo ""
    echo "=========================================="
    echo "Stopping all processes..."
    echo "=========================================="
    
    # Emergency stop - send zero velocity command
    set +u
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    set -u
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true
    
    # Shutdown in reverse order of startup
    
    # Step 1: Stop ROSBridge WebSocket
    echo "  Stopping ROSBridge WebSocket..."
    pkill -INT -f "rosbridge" 2>/dev/null || true
    sleep 2
    
    # Step 2: Stop FastAPI server
    echo "  Stopping FastAPI server..."
    pkill -INT -f "main.py" 2>/dev/null || true
    sleep 1
    
    # Step 3: Stop Drive service
    echo "  Stopping Drive service..."
    pkill -INT -f "drive_service" 2>/dev/null || true
    sleep 1
    
    # Step 4: Stop Nav2 (BEFORE SLAM!)
    echo "  Stopping Nav2 navigation stack..."
    pkill -INT -f "nav2" 2>/dev/null || true
    sleep 3
    
    # Step 5: Stop SLAM toolbox (give it time to save map)
    echo "  Stopping SLAM toolbox..."
    local slam_pids=$(pgrep -f "slam_toolbox" || true)
    if [ -n "$slam_pids" ]; then
        for spid in $slam_pids; do
            echo "    Stopping SLAM PID: $spid"
            kill -INT $spid 2>/dev/null || true
        done
        echo "    Waiting for SLAM to save and exit (5 seconds)..."
        sleep 5
        
        # Verify SLAM is actually stopped
        for spid in $slam_pids; do
            if ps -p $spid > /dev/null 2>&1; then
                echo "    Force stopping SLAM PID: $spid"
                kill -KILL $spid 2>/dev/null || true
            fi
        done
        
        # Wait to ensure it's really dead
        sleep 2
    fi
    
    # Step 6: Stop static transform publisher
    echo "  Stopping transform publisher..."
    pkill -INT -f "static_transform_publisher" 2>/dev/null || true
    sleep 1
    
    # Step 7: Stop LiDAR motor
    echo "  Stopping LiDAR motor..."
    local lidar_pids=$(pgrep -f "sllidar_node" || true)
    if [ -n "$lidar_pids" ]; then
        for lpid in $lidar_pids; do
            kill -INT $lpid 2>/dev/null || true
        done
        sleep 8
        
        # Force kill if still running
        for lpid in $lidar_pids; do
            if ps -p $lpid > /dev/null 2>&1; then
                kill -KILL $lpid 2>/dev/null || true
            fi
        done
    fi
    
    # Step 8: Stop Leo base system
    echo "  Stopping Leo base system..."
    pkill -INT -f "leo_system" 2>/dev/null || true
    sleep 2
    
    # Step 9: Send SIGINT to any remaining tracked process groups
    echo "  Stopping any remaining processes..."
    for pid in "${pids[@]}"; do
        if ps -p $pid > /dev/null 2>&1; then
            kill -INT -$pid 2>/dev/null || true
        fi
    done
    
    # Step 10: Wait for graceful shutdown
    sleep 2
    
    # Step 11: Send SIGTERM to stubborn processes
    for pid in "${pids[@]}"; do
        if ps -p $pid > /dev/null 2>&1; then
            kill -TERM -$pid 2>/dev/null || true
        fi
    done
    
    sleep 1
    
    # Step 12: Force kill any remaining tracked processes
    for pid in "${pids[@]}"; do
        if ps -p $pid > /dev/null 2>&1; then
            kill -KILL -$pid 2>/dev/null || true
        fi
    done
    
    # Step 13: Final cleanup - force kill any remaining by name
    pkill -KILL -f "ros2 launch" 2>/dev/null || true
    pkill -KILL -f "ros2 run" 2>/dev/null || true
    pkill -KILL -f "python3.*ros2" 2>/dev/null || true
    
    # Step 14: Kill any child processes of this script
    pkill -P $ 2>/dev/null || true
    
    # Step 15: CRITICAL - Clean up SLAM toolbox shared memory/semaphores
    echo "  Cleaning SLAM toolbox artifacts..."
    rm -f /dev/shm/sem.slam_toolbox* 2>/dev/null || true
    rm -f /dev/shm/slam_toolbox* 2>/dev/null || true
    
    echo ""
    echo "Logs saved in: $log_dir"
    echo "Cleanup complete - safe to restart"
}

trap cleanup SIGINT SIGTERM EXIT

# =============================================================================
# Check for LiDAR
# =============================================================================
LIDAR_CONNECTED=false
if [ -e /dev/ttyUSB0 ] || [ -e /dev/ttyUSB1 ]; then
    LIDAR_CONNECTED=true
    echo "✓ LiDAR detected"
else
    echo "⚠ LiDAR not detected - skipping SLAM and Nav2"
fi
echo ""

# =============================================================================
# Core Robot Systems
# =============================================================================
launch_and_wait "Leo base system" \
    'source /opt/ros/jazzy/setup.bash; set -u; ros2 run leo_bringup leo_system' \
    "Leo system node started!" \
    15

# =============================================================================
# Sensors (Conditional)
# =============================================================================
if [ "$LIDAR_CONNECTED" = true ]; then
    launch_and_wait "LiDAR driver" \
        'source ~/ws_lidar/install/setup.bash; set -u;
         ros2 launch sllidar_ros2 view_sllidar_a2m12_nogui_launch.py' \
        "current scan mode" \
        15

    launch_and_wait "LiDAR transform (base_link → laser)" \
        'source /opt/ros/jazzy/setup.bash; set -u;
         ros2 run tf2_ros static_transform_publisher \
             --x 0.03 --y 0 --z 0.08 --yaw 3.14159 --pitch 0 --roll 0 \
             --frame-id base_link --child-frame-id laser' \
        "Spinning until stopped - publishing transform" \
        10
    
    # Critical: Let LiDAR scans stabilize before SLAM starts subscribing
    echo "  Letting LiDAR scans stabilize (5 seconds)..."
    sleep 5

    # =========================================================================
    # Navigation Stack
    # =========================================================================
    launch_and_wait "SLAM Toolbox" \
        'source /opt/ros/jazzy/setup.bash; set -u;
        cd ~/leo_ws/src/LeoRover-SLAM-ROS2;
        ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false' \
        "Registering sensor" \
        45
    
    # Let SLAM fully process initial scan queue before Nav2 connects
    echo "  Letting SLAM stabilize (8 seconds)..."
    sleep 8

    launch_and_wait "Nav2 navigation stack" \
        'source /opt/ros/jazzy/setup.bash;
         source ~/leo_ws/install/setup.bash; set -u;
         ros2 launch nav2_bringup navigation_launch.py \
             params_file:="$HOME/leo_ws/src/LeoRover-SLAM-ROS2/nav2_simple.yaml" \
             slam:=true' \
        "Creating bond timer" \
        80

    sleep 5

    # =========================================================================
    # Drive Service
    # =========================================================================
    launch_and_wait "Drive service (relative movement commands)" \
        'source /opt/ros/jazzy/setup.bash;
         source ~/leo_ws/install/setup.bash; set -u;
         ros2 run custom_drive_pkg drive_service' \
        "Drive Service Ready" \
        15
fi

# =============================================================================
# API & Communication
# =============================================================================
launch_and_wait "FastAPI server" \
    'cd ~/api-server; python3 main.py' \
    "Uvicorn running on" \
    30

launch_and_wait "ROSBridge WebSocket server" \
    'source /opt/ros/jazzy/setup.bash; set -u;
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml' \
    "Rosbridge WebSocket server started on port" \
    40

# =============================================================================
# Success Summary
# =============================================================================
echo ""
echo "=========================================="
if [ "$LIDAR_CONNECTED" = true ]; then
    echo "✓ All systems launched successfully!"
    echo "  ✓ Leo base system"
    echo "  ✓ LiDAR and transforms"
    echo "  ✓ SLAM Toolbox"
    echo "  ✓ Nav2 navigation"
    echo "  ✓ Drive service"
    echo "  ✓ FastAPI server"
    echo "  ✓ ROSBridge WebSocket"
    echo ""
    echo "Drive service ready!"
    echo "  Usage: ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand \"{forward: 1.0, rotate: 0.0}\""
else
    echo "✓ Core systems launched successfully!"
    echo "  ✓ Leo base system"
    echo "  ✓ FastAPI server"
    echo "  ✓ ROSBridge WebSocket"
fi
echo "=========================================="

echo ""
echo "Logs directory: $log_dir"
echo "Press Ctrl-C to stop all processes"
echo ""

# Keep the script running
wait
