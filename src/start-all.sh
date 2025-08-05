#!/usr/bin/env bash
# start_all_safe.sh
set -euo pipefail
pids=()                            # array to store top-level PIDs we launch
# ----------- helper: launch commands in a subshell -----------
launch() {        # Usage：launch "<commands>"
  (
    set +u
    eval "$1"
  ) &
  pids+=($!)      # save PIDs
  sleep 3
}
# ----------- Stop everything on Ctrl-C -----------
cleanup() {
  echo "Stopping all processes…"
  for pid in "${pids[@]}"; do
    kill -- -"$pid" 2>/dev/null || true
  done
  pkill -P $$ 2>/dev/null || true         #kill any child still owned by this script
  exit
}
trap cleanup SIGINT SIGTERM
# ---------- 1. leo_bringup ----------
launch 'source /opt/ros/jazzy/setup.bash; set -u; ros2 run leo_bringup leo_system'
# ---------- 2. LiDAR driver ----------
launch 'source ~/ws_lidar/install/setup.bash; set -u; ros2 launch sllidar_ros2 view_sllidar_a2m12_launch.py'
# ---------- 3. static transform ----------
launch 'source /opt/ros/jazzy/setup.bash; set -u; ros2 run tf2_ros static_transform_publisher \
        --x 0.1 --y 0 --z 0.02 --yaw 3.14159 --pitch 0 --roll 0 \
        --frame-id base_link --child-frame-id laser'
# ---------- 4. RViz viewer ----------
launch 'cd ~/leo_ws; set +u; source install/setup.bash; set -u; ros2 launch leo_viz rviz.launch.xml'
# ---------- 5. SLAM Toolbox ----------
launch 'sudo cpufreq-set -g performance; source /opt/ros/jazzy/setup.bash; set -u; \
        cd ~/leo_ws/src/LeoRover-SLAM-ROS2; ros2 launch slam_toolbox online_async_launch.py \
        params_file:=mapper_params_main.yaml'
# ---------- 6. EKF localization ----------
launch 'source /opt/ros/jazzy/setup.bash; set -u; \
        ros2 launch robot_localization ekf.launch.py \
        params_file:="$HOME/leo_ws/src/LeoRover-SLAM-ROS2/ekf.yaml"'
# ---------- 7. Nav2 stack ----------
launch 'source /opt/ros/jazzy/setup.bash; source ~/leo_ws/install/setup.bash; set -u; \
        ros2 launch nav2_bringup navigation_launch.py \
        params_file:="$HOME/leo_ws/src/LeoRover-SLAM-ROS2/nav2_simple.yaml"'
# ---------- 8. FastAPI server ----------
launch 'cd ~/api-server; python3 main.py'
# ---------- 9. rosbridge websocket ----------
launch 'source /opt/ros/jazzy/setup.bash; set -u; \
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
echo "All processes launched.  Press Ctrl-C to stop."
wait
