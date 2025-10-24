# Instruction to Set Up Leo Rover with Raspberry Pi5 (2025 version) as ROS2 server
This document is based on an earlier version implemented with Raspberry Pi 4 together with a NVidia Jetson Nano in 2024. This early version can be found here: (https://github.com/Arthios09/LeoRover-SLAM-ROS2). Please note that the 2024 version is not compatible with the latest Leo Rover OS nor ROS2 Jazzy.

## 1. Preparation for the Raspberry Pi 5 and Leo Rover Hardware

First we need to examine the condition of the onboard Pi5 board and its attachment with the LeoCore controller board. Because we intent to run compute-intensive SLAM and Nav2 ROS nodes directly on Pi5, the condition of the Pi5 board is critical for the correct runtime execution:

1. Please make sure that Pi5 needs to have active cooling solution onboard, such as
   * Official Pi active cooling: https://www.raspberrypi.com/products/active-cooler/

2. Please make sure that the Sllidar is working properly. The Sllidar can be powered sufficiently by its single USB port connected with its adapter box. When it is powered, the adapter box should light up with a green indicator. Also the bandwidth switch should be selected to 256000 for a high rate suitable for the A2M12 model.

3. Charging the batteries: The Leo Rover with a single Pi5 onboard can be powered by a single Leo battery pack. The battery with the button connects to the internal power cable and powers the Raspberry pi, wheel motors, controller, etc. 

When the battery's indicator is blinking green, it indicates that the charge is low and a recharge is needed. The charging is done by plugging in its dedicated Leo Battery Charger (which has higher Amp than regular chargers). Once plugged in, press the indicator button so that the Leo Battery Charger light will turn red, which indicates the charging has started. When the batter is sufficiently charged, the charger indicator will turn green again.

** WARNING: Do not attempt to disassemble the Leo battery pack or the charging cable or connector. Leo Rover uses a special type of waterproof battery connection solution that is difficult to replace. The only disassembly point should be only the connection of the battery pack with the Rover or the charger, under normal conditions **

4. I/O Connection: It is recommended that the onboard Wi-Fi antenna connects to one of the USB2 port; the onboard Sllidar connects to the other USB2 port. 

Leo Rover has a built-in exposed USB connection port. This will really come in handy if we need to connect a keyboard and mouse for debugging. So we recommend to keep this exposed external USB port connect internally to one of the USB3.1 port. The second USB3.1 port is recommended to be connected to the SSD drive for booting the OS (see Software Scheme below).

5. Verification of bootable SSD drive partition.

Custom SSD drive may have different capacity, while the official LeoOS image may only create one very small boot segment and one root segment. To fully utilize the SSD capacity, please install gnome-disks GUI app, and resize the the root segment to the max capacity. This will allow the root directory to be able to store larger ROS packages and data later.

Finally, if connecting an external monitor is needed, we recommend connecting the mini-HDMI with its onboard HDMI0 port.

## 2. Software Setup and Development

### Software Scheme:

The 2025 version of Leo Rover runs on Raspberry Pi5 with Ubuntu 24.04 and ROS2 Jazzy. The rover’s connection to external devices is permitted through its onboard wifi chip. Although the onboard wifi does not have to be connected to another wifi network/the internet, connection to the rover’s wifi is necessary to ssh into either computer and transmit information via the api server/websockets.

* The default Leo Rover WiFi password: password
* The default Pi5 login through the Leo Rover WiFi is: pi@10.0.0.1, Password: raspberry

If flashing a new Leo OS to a Pi5 is needed, please follow the [LeoRover Ros2 (experimental) guide here](https://docs.fictionlab.pl/leo-rover/advanced-guides/ros-2-support). It is strongly recommended to launch the Leo OS (a custome Ubuntu 24.04) from a USB SSD connecting to the USB 3.1 port, but one can also use a microSD.

### Ubuntu Installation for Pi5:

1. Sync computer clock

Before update the system, we need to fix the Pi 5 clock to establish secure connection, since Pi 5 does not have a hardware clock.
First time, we need to manually hard reset the system clock to the current local time (please replace with the correct present local time):
sudo date -s "2025-01-22 12:00:00"

```bash
sudo apt update
sudo apt upgrade
#install chrony
sudo apt install chrony -y
# Stop the service 
sudo systemctl stop systemd-timesyncd 
# Disable it from starting at boot 
sudo systemctl disable systemd-timesyncd
sudo systemctl enable chrony 
sudo systemctl start chrony
```

Finally check the chrony sync status

```bash
timedatectl status 
chronyc sources
```

If chronyc sources display some clock services are correctly connected, then the system clock should be reset automatically in the future (so long as there is WiFi connection).

2. Boost CPU performance for high-intensity apps

```bash
sudo apt update
sudo apt upgrade
sudo apt install cpufrequtils
sudo cpufreq-set -g performance
```
To automatically set cpu to performance, write the last line above into */etc/rc.local*. Then do

```bash
sudo chmod +x /etc/rc.local
```

3. Install and max root segment capacity

```bash
sudo apt install gnome-disk-utility
gnome-disks
```bash

4. Install rplidar ROS package and driver

```bash
# Create workspace
mkdir -p ~/ws_lidar/src
cd ~/ws_lidar/src

# Clone the official Slamtec ROS2 driver
git clone https://github.com/Slamtec/sllidar_ros2.git
# Build the workspace
cd ~/ws_lidar
colcon build --symlink-install
# Source the workspace
source ~/ws_lidar/install/setup.bash
echo "source ~/ws_lidar/install/setup.bash" >> ~/.bashrc
```

5. Install Nav2 and SLAM toolbox
   
```bash
# Update package lists
sudo apt update
# Install Nav2
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
# Install additional Nav2 packages (optional but useful)
sudo apt install ros-$ROS_DISTRO-nav2-*
# Install dependencies
sudo apt install ros-$ROS_DISTRO-robot-localization
sudo apt install ros-$ROS_DISTRO-joint-state-publisher
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
```

SLAM Toolbox should be installed by default together with ROS2 Jazzy and Nav2 above. In case it is not installed, try
```bash
sudo apt update
# Install SLAM Toolbox
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan
# Install additional dependencies
sudo apt install ros-jazzy-nav2-map-server
sudo apt install ros-jazzy-nav2-lifecycle-manager
```

## 3. Build the Leo Rover SLAM server and API server


1. To set up to workspace, first clone the Hitch Play project, then move the *src* folder to the *~\leo_ws* folder
   
```bash
mkdir ~/projects
cd ~/projects
git clone https://github.com/intelligentracing/HitchPlay

mkdir ~/leo_ws
cd HitchPlay
mv src ~/leo_ws
```

2. Before we can build from source, need to install some ROS2 Jazzy packages
```bash
sudo apt update
sudo apt install ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-sensor-msgs
cd ~/leo_ws
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```

3. Install the ROS2 API server

We first install an open-source API server

```bash
cd ~/projects
git clone -b alex_cmd https://gitlab.com/roar-gokart/api-server.git
cd api-server/
sudo apt install python3-fastapi
python3 main.py
```

Next, because Leo OS launches a default rosbridge_websocket, we need to remove it from startup. Specifically, edit:
```bash
sudo nano /opt/ros/jazzy/share/leo_bringup/launch/leo_bringup.launch.xml
```

Comment out the following lines
```script
<!--
<node name="rosbridge_server"
      pkg="rosbridge_server"
      …
      exec="rosbridge_websocket">
</node>
-->
```

The change will take effect after reboot

## 4. Launch the ROS nodes and API server

The entire startup sequence has been coded conveniently in a script file *start-all.sh* inside *HitchPlay/src*. 

The following is a more detailed breakdown of all the services:

1. Start Leo system

```bash
source /opt/ros/jazzy/setup.bash
ros2 run leo_bringup leo_system
```
   
2. Start Sllidar

Before launching sllidar, we need to customize the sllidar's scan frequency, and we want to disable the RViz to conserve CPU cycle. To do that, save a new view_sllidar_a2m12_nogui_launch.py by changing the original launch file view_sllidar_a2m12_launch.py:

First, remove the rviz2 node:

```python
Note (
   package = 'rviz2',
   executable='rviz2',
   name='rviz2',
   arguments=['-d', rviz_config_dir],
   output='screen'),
```
Second, add the scan frequency to the sllidar_ros2 node:

```python
'scan_frequency': 5.0
```

Finally, launch the sllidar node:
```bash
source ~/ws_lidar/install/setup.bash
ros2 launch sllidar_ros2 view_sllidar_a2m12_nogui_launch.py
```

3. Add tf transform
   
```bash
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher --x 0.03 --y 0 --z 0.08 --yaw 3.14159 --pitch 0 --roll 0 --frame-id base_link --child-frame-id laser
```

4. Start RVIZ
   
```bash
source install/setup.bash
ros2 launch leo_viz rviz.launch.xml
```

5. Start Nav2 and SLAM_Toolbox
   
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch robot_localization ekf.launch.py params_file:=$HOME/leo_ws/src/LeoRover-SLAM-ROS2/ekf.yaml
```

```bash
source /opt/ros/jazzy/setup.bash
source ~/leo_ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py params_file:=$HOME/leo_ws/src/LeoRover-SLAM-ROS2/nav2_simple.yaml slam:=true
```

6. Start the customized websocket

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
