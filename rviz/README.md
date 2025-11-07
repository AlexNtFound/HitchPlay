## Overview

This README guides you through setting up a **PC User Interface** for full remote control of the Leo Rover. By following these instructions, you'll be able to:

- **Visualize** Leo Rover's sensor data, navigation, and real-time status using RViz2
- **Control** Leo Rover remotely using an Xbox controller
- **Navigate autonomously** with return-to-base functionality
- **Monitor** all robot systems from your computer without needing to interact directly with the rover

This setup creates a complete operator station on your PC, allowing you to operate Leo Rover over the network as if you were sitting at the robot itself.

## Setup

### Remote Setup (Separate Computer)

Follow these steps to visualize Leo Rover from another computer:

#### Prerequisites
```bash
# Install RViz2 and Leo Rover packages
sudo apt update
sudo apt install ros-jazzy-rviz2 ros-jazzy-leo-description

# If leo-description is not available, build from source:
cd ~/ros2_ws/src
git clone https://github.com/LeoRover/leo_common-ros2.git
cd ~/ros2_ws && colcon build && source install/setup.bash
```

#### Network Configuration

**On both Leo Rover and remote computer**, add to `~/.bashrc`:
```bash
# ROS 2 Network Configuration
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # Use Fast DDS (ensures consistency)
export ROS_DOMAIN_ID=0                      # Match domain ID across devices
export ROS_LOCALHOST_ONLY=0                 # Allow network communication
```

Apply changes:
```bash
source ~/.bashrc
```

#### Verify Connection
```bash
# 1. Find Leo Rover's IP address (on Leo Rover)
hostname -I
# Example: 10.0.0.1

# 2. On remote computer, verify network connection
ping <leo_rover_ip>

# 3. Check if you can see Leo Rover's topics
ros2 topic list
# Should show: /scan, /wheel_odom, /imu/data_raw, etc.
```

#### Troubleshooting Connection

If `ros2 topic list` shows no topics:

1. **Verify both machines use same middleware:**
```bash
   echo $RMW_IMPLEMENTATION
   # Should be: rmw_fastrtps_cpp on both
```

2. **Check domain ID matches:**
```bash
   echo $ROS_DOMAIN_ID
   # Should be: 0 on both
```

3. **Ensure both computers are on same network/subnet**

4. **Check firewall settings:**
```bash
   sudo ufw allow from <leo_rover_ip>
```

5. **Try Cyclone DDS (better for WiFi):**
```bash
   sudo apt install ros-jazzy-rmw-cyclonedds-cpp
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   # Must change on BOTH computers
```

## Xbox Controller Setup

### Prerequisites
```bash
# Install joystick packages
sudo apt update
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

### Connect Xbox Controller

Connect the Xbox controller via USB to your remote computer (not the Leo Rover).

Verify it's detected:
```bash
ls /dev/input/js*
# Should show: /dev/input/js0
```

### Setup Return to Base Script

Make the script executable:

```bash
# Navigate to repository directory
cd /path/to/repository

# Make the script executable
chmod +x xbox_return_to_base.py
```

### Controller Layout

```
Xbox Controller Map:
┌─────────────────────────────────────┐
│                                     │
│   [LB]                       [RB]   │  
│    ↑                          ↑     │
│  Hold to                   Return   │
│  enable                    to home  │
│  driving                            │
│                                     │
│   [A]               [B]             │
│    ↑                 ↑              │
│   Cancel/Stop    Reset home         │
│   navigation     to default         │
│                                     │
│  Left Stick          Right Stick    │
│  ┌────────┐          ┌────────┐     │
│  │   ↑    │          │        │     │
│  │   +    │ Forward/ │ ← + →  │ Rotate
│  │   ↓    │ Backward │        │ only│
│  └────────┘  only    └────────┘     │
│  (click to                          │
│   set home)                         │
│                                     │
│                                     │
│                                     │
│                                     │
│                                     │
│                                     │
└─────────────────────────────────────┘
```

### Controller Functions

**Manual Control:**
- **LB (Hold)**: Enable movement (deadman switch)
- **Left Stick Y-axis**: Drive forward/backward only
- **Right Stick X-axis**: Rotate left/right only
- **Release LB**: Immediate stop

**Autonomous Navigation:**
- **RB (Press)**: Return to home position
- **Left Stick Click**: Set current location as new home
- **B Button**: Reset home to default (map origin 0,0)
- **A Button**: Cancel/stop active navigation

### Testing Controller

Monitor velocity commands being sent:
```bash
ros2 topic echo /cmd_vel
```

Hold LB and move the sticks - you should see velocity messages and Leo should respond.

## Usage

### Quick Start - Full PC User Interface

**On Leo Rover:**

> **💡 Quick Deploy Tip:** If you understand what the script does, you can use the automated startup script instead of running the commands below manually:
> ```bash
> cd ~/path/to/start_all.sh
> .start_all.sh
> ```
> This will launch all necessary nodes in separate terminal windows. Otherwise, follow the manual steps below:

```bash
# Terminal 1: Start Leo Rover base system
source /opt/ros/jazzy/setup.bash
ros2 run leo_bringup leo_system
```

```bash
# Terminal 2: Start Sllidar
source ~/ws_lidar/install/setup.bash
ros2 launch sllidar_ros2 view_sllidar_a2m12_nogui_launch.py
```

```bash
# Terminal 3: Add tf transform
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher --x 0.03 --y 0 --z 0.08 --yaw 3.14159 --pitch 0 --roll 0 --frame-id base_link --child-frame-id laser
```

```bash
# Terminal 4: Launch Slam_toolbox
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

```bash
# Terminal 5: Launch Nav2
source /opt/ros/jazzy/setup.bash
source ~/leo_ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py params_file:=$HOME/leo_ws/src/LeoRover-SLAM-ROS2/nav2_simple.yaml slam:=true
```

**On Your PC:**
```bash
# Terminal 1: Launch RViz visualization
cd /path/to/repository
ros2 run rviz2 rviz2 -d rviz/leo_rover.rviz

# Terminal 2: Start Xbox controller joy node
ros2 run joy joy_node --ros-args -p device_id:=0

# Terminal 3: Start Xbox controller teleop node
ros2 run teleop_twist_joy teleop_node --ros-args \
  -p enable_button:=4 \
  -p axis_linear.x:=1 \
  -p axis_angular.yaw:=3 \
  -p scale_linear.x:=0.5 \
  -p scale_angular.yaw:=1.0

# Terminal 4: Start return to base script
python3 /path/to/repository/xbox_return_to_base.py
```

**You now have a complete operator station:** RViz shows you what Leo sees, Xbox controller lets you drive manually, and autonomous return-to-base functionality!

### Typical Workflow

1. **Manual Exploration**
   - Hold LB and use sticks to drive around
   - Build map using SLAM
   
2. **Set Home Location**
   - Drive to desired home/base location
   - Click left stick to set as home
   
3. **Explore Further**
   - Drive away from home base
   - Use RViz to monitor map building
   
4. **Return Home**
   - Press RB to autonomously navigate back
   - Press A if you need to cancel navigation
   
5. **Reset Home (if needed)**
   - Press B to reset home to map origin (0,0)

### What's Included

**Configuration:** `leo_rover.rviz`

**Fixed Frame:** `odom` (or `map` when using SLAM)

**Displays:**
- Robot model
- LiDAR scan (`/scan`)
- IMU data (`/imu/data_raw`)
- Wheel odometry (`/wheel_odom`)
- Filtered odometry (`/odometry/filtered`)
- Map (`/map`)
- Navigation paths (`/plan`, `/local_plan`)
- Transform frames (TF)

### Updating Configuration
1. Launch RViz with existing config
2. Make your changes (add/remove displays, adjust settings)
3. **File → Save Config** (Ctrl+S)

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "No messages received" on displays | Verify Leo Rover nodes are running: `ros2 topic list` |
| Transform (TF) errors | Check TF tree: `ros2 run tf2_ros tf2_echo odom base_link` |
| Robot model not visible (remote) | Install `ros-jazzy-leo-description` on remote computer |
| RViz won't load config | Verify file exists: `ls -lh rviz/leo_rover.rviz` |
| Can't see topics (remote) | See [Troubleshooting Connection](#troubleshooting-connection) |
| Poor WiFi performance | Try Cyclone DDS instead of Fast DDS (change `RMW_IMPLEMENTATION`) |
| Xbox controller not detected | Check: `ls /dev/input/js*` and install `joystick` package |
| Controller moves but Leo doesn't | Verify `/cmd_vel` is being published: `ros2 topic echo /cmd_vel` |
| Permission denied on controller | Add user to input group: `sudo usermod -a -G input $USER` (logout/login) |
| Wrong stick controls movement | Verify axis mapping: `ros2 topic echo /joy` and check axes values |
| Return to base doesn't work | Ensure SLAM and Nav2 are running; check `/goal_pose` topic |
| Can't cancel navigation | Press A button; check script is running: `ps aux | grep xbox_return` |

## Notes

- **Both computers must use the same RMW implementation** (`rmw_fastrtps_cpp` or `rmw_cyclonedds_cpp`)
- Ensure both Leo Rover and remote computer use ROS 2 Jazzy
- Configuration is version-controlled for team consistency
- For custom configs, use descriptive names (e.g., `leo_slam_debug.rviz`)
- Xbox controller connects to the remote computer, not the Leo Rover
- Always hold the deadman switch (LB) when operating the rover for safety
- This setup enables full remote operation: visualization + control from a single PC workstation
- **Return to base uses `/map` frame** - home location is fixed in the map coordinate system
- Default home is map origin (0,0) where SLAM was initialized
- Custom home locations persist until reset with B button or script restart
