## Overview

This README guides you through setting up a **PC User Interface** for full remote control of the Leo Rover. By following these instructions, you'll be able to:

- **Visualize** Leo Rover's sensor data, navigation, and real-time status using RViz2
- **Control** Leo Rover remotely using an Xbox controller
- **Navigate autonomously** with return-to-base functionality
- **Monitor** all robot systems from your computer without needing to interact directly with the rover

This setup creates a complete operator station on your PC (Windows or Ubuntu), allowing you to operate Leo Rover over the network as if you were sitting at the robot itself.

## Choose Your Platform

**Click your operating system to jump to setup instructions:**

- **[Ubuntu 24.04 Setup](#ubuntu-setup)** - Native ROS 2 Jazzy installation
- **[Windows 11 Setup](#windows-setup)** - ROS 2 Jazzy via pixi (WSL2 not supported)

---

## Ubuntu Setup

### Prerequisites
```bash
# Install RViz2 and Leo Rover packages
sudo apt update
sudo apt install ros-jazzy-rviz2 ros-jazzy-leo-description

# If leo-description is not available, build from source:
cd ~/ros2_ws/src
git clone https://github.com/LeoRover/leo_common-ros2.git
cd ~/ros2_ws && colcon build && source install/setup.bash

# Install Python dependencies for Xbox controller
pip install pygame
```

### Network Configuration

**On both Leo Rover and Ubuntu computer**, add to `~/.bashrc`:
```bash
# ROS 2 Network Configuration
export ROS_DOMAIN_ID=0                      # Match domain ID across devices
export ROS_LOCALHOST_ONLY=0                 # Allow network communication
```

Apply changes:
```bash
source ~/.bashrc
```

### Verify Connection
```bash
# 1. Find Leo Rover's IP address (on Leo Rover)
hostname -I
# Example: 10.0.0.1

# 2. On Ubuntu computer, verify network connection
ping <leo_rover_ip>

# 3. Check if you can see Leo Rover's topics
ros2 topic list
# Should show: /scan, /wheel_odom, /imu/data_raw, etc.
```

### Xbox Controller Setup (Ubuntu)

```bash
# Connect via USB, verify detection:
ls /dev/input/js*
# Should show: /dev/input/js0

# Navigate to repository directory
cd /path/to/repository

# Make the script executable
chmod +x xbox_leo_unified.py

# Test the script
python3 xbox_leo_unified.py
```

### Testing Controller (Ubuntu)

Monitor velocity commands:
```bash
ros2 topic echo /cmd_vel
```

Move the sticks - you should see velocity messages and Leo should respond.

**[Jump to Usage Instructions](#usage)**

---

## Windows Setup

### Prerequisites

1. **Install ROS 2 Jazzy using pixi**
   
   Follow the official Windows installation guide:
   - https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html
   
   Quick summary:
   ```cmd
   # Download and install pixi
   # Create workspace
   mkdir C:\pixi_ws
   cd C:\pixi_ws
   
   # Initialize pixi and install ROS 2 Jazzy
   # (Follow official docs for detailed steps)
   ```

2. **Install Python dependencies**
   
   Open Command Prompt and activate your pixi environment:
   ```cmd
   cd C:\pixi_ws
   pixi shell
   call C:\pixi_ws\ros2-windows\local_setup.bat
   
   # Install pygame for Xbox controller support
   python -m pip install pygame
   ```

### Network Configuration

**Important:** ROS 2 environment variables on Windows must be set each time you open a new Command Prompt, or added to your system environment variables.

**Option 1: Set per session (temporary)**
```cmd
set ROS_DOMAIN_ID=0
set ROS_LOCALHOST_ONLY=0
```

**Option 2: Set permanently (recommended)**
1. Open System Properties → Environment Variables
2. Add these User Variables:
   - `ROS_DOMAIN_ID` = `0`
   - `ROS_LOCALHOST_ONLY` = `0`

### Verify Connection
```cmd
# 1. Find Leo Rover's IP address (on Leo Rover)
# From Leo Rover terminal: hostname -I
# Example: 10.0.0.1

# 2. On Windows, verify network connection
ping <leo_rover_ip>

# 3. Activate ROS 2 and check if you can see Leo Rover's topics
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
ros2 topic list
# Should show: /scan, /wheel_odom, /imu/data_raw, etc.
```

### Xbox Controller Setup (Windows)

```cmd
# Connect via USB or Bluetooth
# Windows will automatically detect Xbox controllers
# Verify in Device Manager under "Xbox Peripherals"

# Navigate to repository directory
cd C:\path\to\repository

# Test the script
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
python xbox_leo_unified.py
```

### Testing Controller (Windows)

Monitor velocity commands:
```cmd
ros2 topic echo /cmd_vel
```

Move the sticks - you should see velocity messages and Leo should respond.

**[Jump to Usage Instructions](#usage)**

---

## Troubleshooting Connection (Both Platforms)

If `ros2 topic list` shows no topics:

1. **Check domain ID matches:**
   - Ubuntu: `echo $ROS_DOMAIN_ID`
   - Windows: `echo %ROS_DOMAIN_ID%`
   - Should be: `0` on both

2. **Ensure both computers are on same network/subnet**

3. **Check firewall settings:**
   - Ubuntu: `sudo ufw allow from <leo_rover_ip>`
   - Windows: Allow ROS 2 through Windows Firewall

4. **Try Cyclone DDS (better for WiFi/different networks):**
   - Ubuntu: 
     ```bash
     sudo apt install ros-jazzy-rmw-cyclonedds-cpp
     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
     ```
   - Windows: 
     ```cmd
     set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
     ```
   - Must change on BOTH computers

5. **Verify ROS_LOCALHOST_ONLY is disabled:**
   - Should be `0` to allow network communication

---

## Controller Layout

```
Xbox Controller Map:
┌─────────────────────────────────────┐
│         [Nexus/Guide Button]        │
│              ↑                      │
│         Toggle E-Stop               │
│                                     │
│   [LB]        [Start]        [RB]   │  
│    (unused)     ↑             ↑     │
│            Release         Return   │
│            E-Stop          to home  │
│                                     │
│   [A]               [B]             │
│    ↑                 ↑              │
│   Cancel/Stop    Reset home         │
│   navigation     to default         │
│                                     │
│  Left Stick          Right Stick    │
│  ┌────────┐          ┌────────┐     │
│  │        │          │   ↑    │     │
│  │ ← + →  │ Rotate   │   +    │ Forward/
│  │        │ only     │   ↓    │ Backward
│  └────────┘          └────────┘     │
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
- **Right Stick Y-axis**: Drive forward/backward
- **Left Stick X-axis**: Rotate left/right
- **Nexus/Guide Button**: Toggle E-Stop (emergency stop)
- **Start Button**: Release E-Stop

**Autonomous Navigation:**
- **RB Button**: Return to home position
- **Left Stick Click**: Set current location as new home
- **B Button**: Reset home to default (map origin 0,0)
- **A Button**: Cancel/stop active navigation

> **Safety Note:** The E-Stop function immediately stops all robot movement. Press the Nexus/Guide button to toggle E-Stop on/off, or press Start to release it.

---

## Usage

### Quick Start - Full PC User Interface

**On Leo Rover:**

> **💡 Quick Deploy Tip:** If you understand what the script does, you can use the automated startup script instead of running the commands below manually:
> ```bash
> cd ~/path/to/start_all.sh
> ./start_all.sh
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

**On Your PC (Ubuntu):**
```bash
# Terminal 1: Launch RViz visualization
cd /path/to/repository
ros2 run rviz2 rviz2 -d rviz/leo_rover.rviz

# Terminal 2: Start Xbox controller with nav script
python3 /path/to/repository/xbox_leo_unified.py
```

**On Your PC (Windows):**
```cmd
REM Terminal 1: Setup environment and launch RViz
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
cd C:\path\to\repository
ros2 run rviz2 rviz2 -d rviz/leo_rover.rviz

REM Terminal 2: Setup environment and start Xbox controller
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
cd C:\path\to\repository
python xbox_leo_unified.py
```

**You now have a complete operator station:** RViz shows you what Leo sees, Xbox controller lets you drive manually with E-Stop safety, and autonomous return-to-base functionality!

### Typical Workflow

1. **Start Operation**
   - Ensure E-Stop is released (press Start button if needed)
   - Confirm in terminal: "E-STOP RELEASED"

2. **Manual Exploration**
   - Use right stick to drive forward/backward
   - Use left stick to rotate
   - Build map using SLAM visible in RViz
   
3. **Set Home Location**
   - Drive to desired home/base location
   - Click left stick to set as home
   - Confirm in terminal: "✓ Home position updated"
   
4. **Explore Further**
   - Drive away from home base
   - Use RViz to monitor map building
   - Monitor robot position in real-time
   
5. **Return Home**
   - Press RB to autonomously navigate back
   - Watch robot follow planned path in RViz
   - Press A if you need to cancel navigation
   
6. **Emergency Stop**
   - Press Nexus/Guide button for immediate E-Stop
   - Press Start or Nexus again to release
   
7. **Reset Home (if needed)**
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

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **General Issues** | |
| "No messages received" on displays | Verify Leo Rover nodes are running: `ros2 topic list` |
| Transform (TF) errors | Check TF tree: `ros2 run tf2_ros tf2_echo odom base_link` |
| Robot model not visible (remote) | Install `ros-jazzy-leo-description` on remote computer |
| RViz won't load config | Verify file exists: `ls -lh rviz/leo_rover.rviz` (Ubuntu) or `dir rviz\leo_rover.rviz` (Windows) |
| Can't see topics (remote) | See [Troubleshooting Connection](#troubleshooting-connection-both-platforms) |
| Poor WiFi performance | Try Cyclone DDS: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (must set on BOTH devices) |
| **Xbox Controller Issues** | |
| Xbox controller not detected | Ubuntu: Check `ls /dev/input/js*` and install `joystick` package<br>Windows: Check Device Manager under "Xbox Peripherals" |
| Controller moves but Leo doesn't | Verify `/cmd_vel` is being published: `ros2 topic echo /cmd_vel` |
| Wrong stick controls movement | Verify in script: Right Stick Y = forward/back, Left Stick X = rotation |
| pygame import error | Ubuntu: `pip install pygame`<br>Windows: `python -m pip install pygame` in pixi shell |
| "No Xbox controller found" error | Verify controller is connected and detected by OS before starting script |
| **Navigation Issues** | |
| Return to base doesn't work | Ensure SLAM and Nav2 are running; check `/goal_pose` topic |
| Can't cancel navigation | Press A button; verify script running: `ps aux \| grep xbox` (Ubuntu) or Task Manager (Windows) |
| Home position not updating | Check TF transforms available: `ros2 topic echo /tf` |
| Navigation goals ignored | Verify Nav2 is running and map is available in RViz |
| **Windows-Specific Issues** | |
| ROS 2 commands not found | Ensure pixi shell is activated and `local_setup.bat` is called |
| Environment variables not set | Set permanently in System Properties or set per session before running |
| Firewall blocking connection | Allow Python and ROS 2 through Windows Firewall |
| **Ubuntu-Specific Issues** | |
| Permission denied on controller | Add user to input group: `sudo usermod -a -G input $USER` (logout/login) |
| Script not executable | Make executable: `chmod +x xbox_leo_unified.py` |

---

## Notes

- **Both computers must use the same ROS_DOMAIN_ID** (default: `0`)
- Ensure both Leo Rover and remote computer use ROS 2 Jazzy
- Configuration is version-controlled for team consistency
- For custom configs, use descriptive names (e.g., `leo_slam_debug.rviz`)
- Xbox controller connects to the remote computer, not the Leo Rover
- The unified script (`xbox_leo_unified.py`) works on both Windows and Ubuntu
- E-Stop feature provides immediate safety shutdown
- **Return to base uses `/map` frame** - home location is fixed in the map coordinate system
- Default home is map origin (0,0) where SLAM was initialized
- Custom home locations persist until reset with B button or script restart
- **Windows users:** Always activate pixi shell and source ROS 2 setup before running any ROS 2 commands
- **Ubuntu users:** Source ROS 2 setup in your `~/.bashrc` for convenience
- **RMW Implementation:** By default, ROS 2 will use the available middleware. Only set `RMW_IMPLEMENTATION` if you experience connection issues (try Cyclone DDS)
