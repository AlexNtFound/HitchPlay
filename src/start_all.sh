#!/usr/bin/env bash
# start-all.sh - Launch all Leo Rover ROS2 navigation components
set -euo pipefail

# Store PIDs of all launched processes
pids=()

# =============================================================================
# Helper Functions
# =============================================================================

launch() {
    local description="$1"
    local command="$2"
    local wait_time="${3:-2}"  # Default 2 seconds, or use provided value
    
    echo "Starting: $description"
    (
        set +u
        eval "$command"
    ) &
    pids+=($!)
    sleep "$wait_time"
}

cleanup() {
    echo ""
    echo "Stopping all processes..."
    for pid in "${pids[@]}"; do
        kill -- -"$pid" 2>/dev/null || true
    done
    pkill -P $$ 2>/dev/null || true
    echo "Cleanup complete"
    exit
}

trap cleanup SIGINT SIGTERM

# =============================================================================
# Core Robot Systems
# =============================================================================

launch "Leo base system" \
    'source /opt/ros/jazzy/setup.bash; set -u; ros2 run leo_bringup leo_system'

# =============================================================================
# Sensors
# =============================================================================

launch "LiDAR driver" \
    'source ~/ws_lidar/install/setup.bash; set -u; 
     ros2 launch sllidar_ros2 view_sllidar_a2m12_nogui_launch.py'

launch "LiDAR transform (base_link → laser)" \
    'source /opt/ros/jazzy/setup.bash; set -u;
     ros2 run tf2_ros static_transform_publisher \
         --x 0.03 --y 0 --z 0.08 --yaw 3.14159 --pitch 0 --roll 0 \
         --frame-id base_link --child-frame-id laser'

# Wait for sensors to stabilize before starting SLAM
echo "Waiting for sensors to stabilize..."
sleep 2

# =============================================================================
# Navigation Stack
# =============================================================================

launch "SLAM Toolbox" \
    'sudo cpufreq-set -g performance;
     source /opt/ros/jazzy/setup.bash; set -u;
     cd ~/leo_ws/src/LeoRover-SLAM-ROS2;
     ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false'

# Wait for SLAM to initialize and start publishing map→odom transform
echo "Waiting for SLAM to initialize..."
sleep 3

launch "Nav2 navigation stack" \
    'source /opt/ros/jazzy/setup.bash;
     source ~/leo_ws/install/setup.bash; set -u;
     ros2 launch nav2_bringup navigation_launch.py \
         params_file:="$HOME/leo_ws/src/LeoRover-SLAM-ROS2/nav2_simple.yaml"'

# Wait for Nav2 to initialize
echo "Waiting for Nav2 to initialize..."
sleep 8

# =============================================================================
# API & Communication
# =============================================================================

launch "FastAPI server" \
    'cd ~/api-server; python3 main.py'

launch "ROSBridge WebSocket server" \
    'source /opt/ros/jazzy/setup.bash; set -u;
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml'

# =============================================================================
# Keep Running
# =============================================================================

echo ""
echo "=========================================="
echo "All systems launched successfully!"
echo "Press Ctrl-C to stop all processes"
echo "=========================================="
echo ""

wait
