# RViz Configuration Files

This directory contains RViz2 configuration files for visualizing the Leo Rover's sensor data, navigation, and robot state.

## Overview

This README guides you through setting up a **PC User Interface** for full remote control of the Leo Rover. By following these instructions, you'll be able to:

- **Visualize** Leo Rover's sensor data, navigation, and real-time status using RViz2
- **Control** Leo Rover remotely using an Xbox controller
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
# Example: 192.168.1.100

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

### Launch Controller Nodes

**Terminal 1** - Start the joy node:
```bash
ros2 run joy joy_node --ros-args -p device_id:=0
```

**Terminal 2** - Start the teleop node:
```bash
ros2 run teleop_twist_joy teleop_node --ros-args -p enable_button:=4 -p axis_linear.x:=1 -p axis_angular.yaw:=0 -p scale_linear.x:=0.5 -p scale_angular.yaw:=1.0
```

### Controller Layout

- **Left Bumper (LB)**: Hold to enable movement (deadman switch)
- **Left Stick Up/Down**: Drive forward/backward
- **Right Stick Left/Right**: Rotate left/right
- **Release LB**: Emergency stop

### Testing Controller

Monitor velocity commands being sent:
```bash
ros2 topic echo /cmd_vel
```

Hold LB and move the sticks - you should see velocity messages and Leo should respond.

## Usage

### Quick Start - Full PC User Interface
```bash
# Terminal 1: Start Leo Rover (on Leo Rover)
source /opt/ros/jazzy/setup.bash
ros2 run leo_bringup leo_system
```
```bash
# Terminal 2: Launch RViz visualization (on your PC)
cd /path/to/repository
ros2 run rviz2 rviz2 -d rviz/leo_rover.rviz

# Terminal 3: Start Xbox controller joy node (on your PC)
ros2 run joy joy_node --ros-args -p device_id:=0

# Terminal 4: Start Xbox controller teleop node (on your PC)
ros2 run teleop_twist_joy teleop_node --ros-args -p enable_button:=4 -p axis_linear.x:=1 -p axis_angular.yaw:=0 -p scale_linear.x:=0.5 -p scale_angular.yaw:=1.0
```

**You now have a complete operator station:** RViz shows you what Leo sees, and the Xbox controller lets you drive it remotely!

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

## Notes

- **Both computers must use the same RMW implementation** (`rmw_fastrtps_cpp` or `rmw_cyclonedds_cpp`)
- Ensure both Leo Rover and remote computer use ROS 2 Jazzy
- Configuration is version-controlled for team consistency
- For custom configs, use descriptive names (e.g., `leo_slam_debug.rviz`)
- Xbox controller connects to the remote computer, not the Leo Rover
- Always hold the deadman switch (LB) when operating the rover for safety
- This setup enables full remote operation: visualization + control from a single PC workstation
