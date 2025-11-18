#!/usr/bin/env bash
# start_all.sh - Launch all Leo Rover ROS2 navigation components with smart health checks
set -euo pipefail


# Store PIDs and log files
pids=()
log_dir="/tmp/rover_logs"
mkdir -p "$log_dir"


# Flag to prevent double cleanup
cleanup_done=false


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


celebration_wiggle() {
   echo ""
   echo "🎉 Rover ready! Performing quick forward bump..."
  
   # Wait for Nav2 to stabilize
   sleep 5
  
   # Simple forward bump - just one command
   set +u
   source /opt/ros/jazzy/setup.bash
   set -u
  
   # Move forward briefly
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
       "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" &
   sleep 0.5
  
   # Stop
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
       "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" &
  
   wait  # Wait for both commands to finish
  
   echo "✓ Ready!"
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
  
   # Special handling for LiDAR - find and stop it first with extra time
   echo "  Stopping LiDAR motor..."
   local lidar_pids=$(pgrep -f "sllidar_node" || true)
   if [ -n "$lidar_pids" ]; then
       for lpid in $lidar_pids; do
           echo "    Found sllidar_node PID: $lpid"
           kill -INT $lpid 2>/dev/null || true
       done
       echo "    Waiting for LiDAR motor to stop (8 seconds)..."
       sleep 8
      
       # Force kill if still running
       for lpid in $lidar_pids; do
           if ps -p $lpid > /dev/null 2>&1; then
               echo "    Force stopping sllidar_node PID: $lpid"
               kill -KILL $lpid 2>/dev/null || true
           fi
       done
   fi
  
   # Step 1: Send SIGINT (Ctrl+C) to all process groups
   echo "  Stopping other ROS2 processes..."
   for pid in "${pids[@]}"; do
       if ps -p $pid > /dev/null 2>&1; then
           kill -INT -$pid 2>/dev/null || true
       fi
   done
  
   # Step 2: Wait for graceful shutdown
   sleep 3
  
   # Step 3: Send SIGTERM to stubborn processes
   for pid in "${pids[@]}"; do
       if ps -p $pid > /dev/null 2>&1; then
           kill -TERM -$pid 2>/dev/null || true
       fi
   done
  
   sleep 2
  
   # Step 4: Force kill any remaining processes
   for pid in "${pids[@]}"; do
       if ps -p $pid > /dev/null 2>&1; then
           kill -KILL -$pid 2>/dev/null || true
       fi
   done
  
   # Step 5: Additional cleanup by process name
   pkill -KILL -f "ros2 launch" 2>/dev/null || true
   pkill -KILL -f "ros2 run" 2>/dev/null || true
   pkill -KILL -f "slam_toolbox" 2>/dev/null || true
   pkill -KILL -f "nav2" 2>/dev/null || true
   pkill -KILL -f "rosbridge" 2>/dev/null || true
   pkill -KILL -f "main.py" 2>/dev/null || true
   pkill -KILL -f "drive_service" 2>/dev/null || true
  
   # Final check for any remaining ROS2 processes
   pkill -KILL -f "python3.*ros2" 2>/dev/null || true
  
   # Kill any child processes of this script
   pkill -P $$ 2>/dev/null || true
  
   # Cleanup temp files
   rm -f /tmp/wiggle.py
  
   echo ""
   echo "Logs saved in: $log_dir"
   echo "Cleanup complete"
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


   # =========================================================================
   # Navigation Stack
   # =========================================================================
   launch_and_wait "SLAM Toolbox" \
       'sudo cpufreq-set -g performance;
        source /opt/ros/jazzy/setup.bash; set -u;
        cd ~/leo_ws/src/LeoRover-SLAM-ROS2;
        ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false' \
       "Registering sensor" \
       30


   launch_and_wait "Nav2 navigation stack" \
       'source /opt/ros/jazzy/setup.bash;
        source ~/leo_ws/install/setup.bash; set -u;
        ros2 launch nav2_bringup navigation_launch.py \
            params_file:="$HOME/leo_ws/src/LeoRover-SLAM-ROS2/nav2_simple.yaml" \
            slam:=true' \
       "Creating bond timer" \
       80


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


# Perform celebration wiggle
# celebration_wiggle


echo ""
echo "Logs directory: $log_dir"
echo "Press Ctrl-C to stop all processes"
echo ""


# Keep the script running
wait

