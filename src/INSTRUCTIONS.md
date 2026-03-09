# Instruction to Set Up Leo Rover with Raspberry Pi5 (2026 version) as ROS2 server

This document is based on an earlier version implemented with Raspberry Pi 4 together with a NVidia Jetson Nano in 2024. This early version can be found here: (https://github.com/Arthios09/LeoRover-SLAM-ROS2). Please note that the 2024 version is not compatible with the latest Leo Rover OS nor ROS2 Jazzy.

## 1. Preparation for the Raspberry Pi 5 and Leo Rover Hardware

First we need to examine the condition of the onboard Pi5 board and its attachment with the LeoCore controller board. Because we intent to run compute-intensive SLAM ROS nodes directly on Pi5, the condition of the Pi5 board is critical for the correct runtime execution:

1. Please make sure that Pi5 needs to have active cooling solution onboard, such as
   - Official Pi active cooling: https://www.raspberrypi.com/products/active-cooler/

2. Please make sure that the Sllidar is working properly. The Sllidar can be powered sufficiently by its single USB port connected with its adapter box. When it is powered, the adapter box should light up with a green indicator. Also the bandwidth switch should be selected to 256000 for a high rate suitable for the A2M12 model.

3. Charging the batteries: The Leo Rover with a single Pi5 onboard can be powered by a single Leo battery pack. The battery with the button connects to the internal power cable and powers the Raspberry pi, wheel motors, controller, etc.

   When the battery indicator is blinking green, it indicates that the charge is low and a recharge is needed. The charging is done by plugging in its dedicated Leo Battery Charger (which has higher Amp than regular chargers). Once plugged in, press the indicator button so that the Leo Battery Charger light will turn red, which indicates the charging has started. When the battery is sufficiently charged, the charger indicator will turn green again.

   **WARNING: Do not attempt to disassemble the Leo battery pack or the charging cable or connector. Leo Rover uses a special type of waterproof battery connection solution that is difficult to replace. The only disassembly point should be only the connection of the battery pack with the Rover or the charger, under normal conditions.**

4. I/O Connection: It is recommended that the onboard Wi-Fi antenna connects to one of the USB2 port; the onboard Sllidar connects to the other USB2 port.

   Leo Rover has a built-in exposed USB connection port. This will really come in handy if we need to connect a keyboard and mouse for debugging. So we recommend to keep this exposed external USB port connect internally to one of the USB3.1 port.

5. (Optional) Bootable SSD drive partition.

   The original Leo Rover package comes with the Leo OS on a microSD card. MicroSD cards typically have lower I/O bandwidth and shorter lifespan and endurance compared to SSD memory. Therefore, it is recommended to replace the factory microSD card with a custom SSD boot drive. The SSD drive to boot the OS can be installed to the second USB3.1 port on the Pi 5 board.

   Custom SSD drive may have different capacity, while the official LeoOS image may only create one very small boot segment and one root segment. To fully utilize the SSD capacity, please install gnome-disks GUI app, and resize the root segment to the max capacity. This will allow the root directory to be able to store larger ROS packages and data later.

   Finally, if connecting an external monitor is needed, we recommend connecting the mini-HDMI with its onboard HDMI0 port.

## 2. Software Setup and Development

### Software Scheme

The 2025 version of Leo Rover runs on Raspberry Pi5 with Ubuntu 24.04 and ROS2 Jazzy. The rover connection to external devices is permitted through its onboard wifi chip. Although the onboard wifi does not have to be connected to another wifi network/the internet, connection to the rover wifi is necessary to ssh into either computer and transmit information via the api server/websockets.

- The default Leo Rover WiFi password: password
- The default Pi5 login through the Leo Rover WiFi is: pi@10.0.0.1, Password: raspberry

If flashing a new Leo OS to a Pi5 is needed, please follow the [LeoRover Ros2 (experimental) guide here](https://docs.fictionlab.pl/leo-rover/advanced-guides/ros-2-support). It is strongly recommended to launch the Leo OS (a custom Ubuntu 24.04) from a USB SSD connecting to the USB 3.1 port, but one can also use a microSD.

In case the user prefers a different default WiFi name, especially there are multiple Leo Rovers in the same space, the default WiFi name can be reset by editing hostapd.conf:

```bash
sudo nano /etc/hostapd/hostapd.conf
```

then finding and editing the ssid entry as: ssid=LeoRover-XXXX, where XXXX is the customized WiFi name that should be unique to each Leo Rover. Restart the system to make the change to take effect.

### Ubuntu Installation for Pi5

**Step 1. Sync computer clock**

Before updating the system, we need to fix the Pi 5 clock to establish secure connection, since Pi 5 does not have a hardware clock. First time, we need to manually hard reset the system clock to the current local time (please replace with the correct present local time):

```bash
sudo date -s "2025-01-22 12:00:00"
sudo apt update
sudo apt upgrade
sudo apt install chrony -y
sudo systemctl stop systemd-timesyncd
sudo systemctl disable systemd-timesyncd
sudo systemctl enable chrony
sudo systemctl start chrony
```

Finally check the chrony sync status:

```bash
timedatectl status
chronyc sources
```

If chronyc sources display some clock services are correctly connected, then the system clock should be reset automatically in the future (so long as there is WiFi connection).

**Step 2. Boost CPU performance for high-intensity apps**

```bash
sudo apt update
sudo apt upgrade
sudo apt install cpufrequtils
sudo cpufreq-set -g performance
```

To automatically set cpu to performance, write the last line above into `/etc/rc.local`. Then do:

```bash
sudo chmod +x /etc/rc.local
```

**Step 3. Install and max root segment capacity**

```bash
sudo apt install gnome-disk-utility
gnome-disks
```

**Step 4. Install rplidar ROS package and driver**

```bash
mkdir -p ~/ws_lidar/src
cd ~/ws_lidar/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ws_lidar
colcon build --symlink-install
source ~/ws_lidar/install/setup.bash
echo "source ~/ws_lidar/install/setup.bash" >> ~/.bashrc
```

**Step 5. Install SLAM Toolbox**

> **Architecture Note:** In this configuration, the Pi5 runs only SLAM for localization and mapping. Navigation commands (go-to-coordinate, go-home) are handled by a lightweight `simple_nav_node` that uses proportional control with SLAM TF data. This approach keeps CPU usage under 60%, leaving headroom for stability.

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-robot-state-publisher
```

**Step 6. Update Leo board firmware**

Please follow the Documentation to update the Leo board firmware:
[How to update Leo Rover firmware](https://docs.fictionlab.pl/leo-rover/guides/firmware-update)

## 3. Build the Leo Rover SLAM Server and API Server

**3.1** To set up the workspace, first clone the Hitch Play project, then move the `src` folder to the `~/leo_ws` folder:

```bash
mkdir ~/projects
cd ~/projects
git clone https://github.com/intelligentracing/HitchPlay
mkdir ~/leo_ws
cd HitchPlay
mv src ~/leo_ws
```

**3.2** Before we can build from source, install some ROS2 Jazzy packages:

```bash
sudo apt update
sudo apt install ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-sensor-msgs
cd ~/leo_ws
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```

Next, because Leo OS launches a default rosbridge_websocket, we need to remove it from startup. Specifically, edit:

```bash
sudo nano /opt/ros/jazzy/share/leo_bringup/launch/leo_bringup.launch.xml
```

Comment out the following lines:

```xml
<!--
<node name="rosbridge_server"
      pkg="rosbridge_server"
      exec="rosbridge_websocket">
</node>
-->
```

Change the rosbridge server setting:

```bash
sudo nano /opt/ros/jazzy/share/rosbridge_server/launch/rosbridge_websocket_launch.xml
```

Change the delay between messages from 0 to 0.0:

```xml
<arg name="delay_between_messages" default="0.0" />
```

The change will take effect after reboot.

## 4. Configure LiDAR Scan Frequency (5 Hz)

Running the LiDAR at 5 Hz (instead of the default ~10-11 Hz) significantly reduces CPU load on the Pi5, which is critical for keeping SLAM responsive. Higher scan rates cause SLAM message queue to overflow, leading to stale TF transforms and degraded localization accuracy.

Create a custom launch file that sets the correct scan frequency and disables the RViz GUI:

**4.1** Copy the original launch file to create a custom version:

```bash
cp ~/ws_lidar/src/sllidar_ros2/launch/view_sllidar_a2m12_launch.py \
   ~/ws_lidar/src/sllidar_ros2/launch/view_sllidar_a2m12_nogui_launch.py
```

**4.2** Edit the custom launch file:

```bash
nano ~/ws_lidar/src/sllidar_ros2/launch/view_sllidar_a2m12_nogui_launch.py
```

**4.3** Remove the entire rviz2 node block:

```python
# DELETE this entire block:
Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_dir],
    output='screen'),
```

**4.4** Add the scan frequency parameter to the sllidar_ros2 node parameters section:

```python
'scan_frequency': 5.0,
```

**4.5** Rebuild the workspace for the changes to take effect:

```bash
cd ~/ws_lidar
colcon build --symlink-install
```

**4.6** Verify the scan frequency after launching (in a separate terminal):

```bash
# Should show approximately 5 Hz, NOT 10-11 Hz
ros2 topic hz /scan

# Also verify the parameter was set correctly
ros2 param get /sllidar_node scan_frequency
```

> **Why 5 Hz?** At 10+ Hz on the Pi5, SLAM toolbox cannot process scans fast enough, causing its message queue to fill up. This results in the map to odom TF transform becoming stale (delays of 100+ seconds observed in testing), which causes navigation commands to fail with "Initial robot pose is not available." At 5 Hz, SLAM has sufficient CPU headroom to keep TF delays under 1 second.

## 5. Launch the ROS Nodes and API Server

The entire startup sequence has been coded conveniently in a script file `start_all.sh` inside `~/leo_ws/src/`.

The following is a more detailed breakdown of all the services:

**5.1** Start Leo system

```bash
source /opt/ros/jazzy/setup.bash
ros2 run leo_bringup leo_system
```

**5.2** Start Sllidar (at 5 Hz, no GUI)

```bash
source ~/ws_lidar/install/setup.bash
ros2 launch sllidar_ros2 view_sllidar_a2m12_nogui_launch.py
```

**5.3** Add tf transform

```bash
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher --x 0.03 --y 0 --z 0.02 --yaw 3.14159 --pitch 0 --roll 0 --frame-id base_link --child-frame-id laser
```

**5.4** Start SLAM Toolbox

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

**5.5** Start the Simple Navigation Node

The simple navigation node provides go-to-coordinate and go-home functionality using proportional control with SLAM TF data. It is a lightweight (~2% CPU) Python node with no external dependencies beyond the core ROS 2 packages already installed.

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/leo_ws/src/simple_nav_node.py
```

> **WARNING:** The simple navigation node does **not** provide obstacle avoidance. The rover will drive in a straight line toward the goal. Use in open spaces only.

Sending navigation commands from the command line:

```bash
# Go to specific coordinates (x=1.0, y=2.0)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}"

# Go home (return to origin)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}"

# Cancel active navigation
ros2 topic pub --once /cancel_nav std_msgs/msg/Empty "{}"

# Monitor navigation status
ros2 topic echo /nav_status
```

**5.6** Start the customized websocket

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**5.7** Start API server

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/leo_ws/src/api-server/main.py
```

**5.8** Start RViz (optional, on a remote PC only)

> **Note:** Do not run RViz on the Pi5 as it consumes significant CPU and GPU resources. Run it on a remote PC connected to the same ROS2 network.

```bash
source ~/leo_ws/install/setup.bash
ros2 launch leo_viz rviz.launch.xml
```

### Expected Pi5 CPU Load

With this configuration, the Pi5 should maintain a load average of 2.0 to 3.0 on 4 cores:

| Process | Approx CPU | Purpose |
|---------|:----------:|---------|
| leo_system + firmware | ~10% | Motor control, hardware I/O |
| sllidar_node (5 Hz) | ~10% | LiDAR driver |
| odom_filter | ~10% | EKF odometry fusion |
| slam_toolbox | ~15% | Localization and mapping |
| robot_state_publisher | ~2% | TF tree for URDF links |
| static_transform_publisher | ~1% | base_link to laser TF |
| simple_nav_node | ~2% | Go-to-coordinate controller |
| rosbridge_websocket | ~5% | WebSocket bridge for API/web clients |
| web_video_server | ~5% | Camera streaming |
| **Total** | **~60%** | **Leaves headroom for stability** |

### Verifying SLAM Health

After startup, verify that the SLAM TF transform is healthy:

```bash
ros2 run tf2_ros tf2_monitor map odom
```

If the average delay exceeds 5 seconds, SLAM is falling behind. Common causes include the LiDAR running above 5 Hz, other CPU-intensive processes, or Pi5 thermal throttling.

## 6. Setting Up Automatic Startup on Boot

To make the Leo Rover start all services automatically when powered on, create a systemd service that runs `start_all.sh` on boot.

### Creating the Systemd Service

1. Create the service file:

```bash
sudo nano /etc/systemd/system/leo-startup.service
```

2. Add the following content:

```ini
[Unit]
Description=Leo Rover Auto-Start
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/leo_ws
ExecStartPre=/bin/sleep 10
ExecStart=/bin/bash /home/pi/leo_ws/src/start_all.sh
Restart=on-failure
RestartSec=15
StandardOutput=journal
StandardError=journal
TimeoutStartSec=180

[Install]
WantedBy=multi-user.target
```

3. Reload systemd, enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable leo-startup.service
sudo systemctl start leo-startup.service
```

### Managing the Service

```bash
sudo systemctl status leo-startup.service
sudo systemctl stop leo-startup.service
sudo systemctl restart leo-startup.service
sudo systemctl disable leo-startup.service
```

## 7. Custom Drive Package for Robot Control

The custom_drive_pkg provides services for controlling the Leo Rover with relative movement commands and sequential navigation automation.

**For complete documentation, usage examples, and API integration guide, see:**

[custom_drive_pkg/README.md](./custom_drive_pkg/README.md)

The package includes:
- **Original Drive Service** - Single-command navigation with immediate response
- **Sequential Drive Service** - Multi-step automated navigation for AI agent integration

Both services are built automatically with step 3.2 above and can be started with `start_all.sh` or individually as documented in the README.

## 8. Saving and Reusing Maps

SLAM toolbox can save the current map for later reuse. First create the maps directory, then use the slam_toolbox service to serialize the map:

```bash
mkdir -p ~/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/pi/maps/my_map'}"
```

To load a saved map on next startup, add the `map_file_name` parameter to the SLAM launch.

## Appendix A: Troubleshooting

### SLAM TF Delay

If navigation fails with "Initial robot pose is not available", check SLAM TF delay:

```bash
ros2 run tf2_ros tf2_monitor map odom
```

Average delay should be under 1 second. If higher, check LiDAR frequency (`ros2 topic hz /scan` should show ~5 Hz), CPU load (`top`), and Pi5 temperature (`vcgencmd measure_temp` should be under 80°C).

### QoS Mismatch on /goal_pose

If publishing to `/goal_pose` has no effect, check for QoS incompatibility:

```bash
ros2 topic info /goal_pose --verbose
```

The `simple_nav_node` uses default RELIABLE QoS. Ensure publishers also use RELIABLE (the default for `ros2 topic pub`, rosbridge, and `xbox_leo_unified.py`).

### Verifying the Full TF Chain

Required chain: `map` → `odom` → `base_link` → `laser`. Verify each link:

```bash
ros2 run tf2_ros tf2_echo map odom          # SLAM (~1 Hz)
ros2 run tf2_ros tf2_echo odom base_link    # EKF (~50 Hz)
ros2 run tf2_ros tf2_echo base_link laser   # Static (constant)
```

If any link shows "Waiting for transform" indefinitely, the corresponding node (`slam_toolbox`, `odom_filter`, or `static_transform_publisher`) is not running.
