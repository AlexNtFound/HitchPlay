# Leo Rover 搭配 Raspberry Pi5（2026版）作为 ROS2 服务器的安装说明

本文档基于 2024 年使用 Raspberry Pi 4 与 NVidia Jetson Nano 实现的早期版本。早期版本可在此处找到：(https://github.com/Arthios09/LeoRover-SLAM-ROS2)。请注意，2024 版本与最新的 Leo Rover OS 和 ROS2 Jazzy 不兼容。

## 1. Raspberry Pi 5 与 Leo Rover 硬件准备

首先我们需要检查板载 Pi5 主板及其与 LeoCore 控制板的连接状态。由于我们计划直接在 Pi5 上运行计算密集型 SLAM ROS 节点，Pi5 主板的状态对于正确的运行时执行至关重要：

1. 请确保 Pi5 上有主动散热解决方案，例如
   - 官方 Pi 主动散热器：https://www.raspberrypi.com/products/active-cooler/

2. 请确保 Sllidar 工作正常。Sllidar 可以通过其与适配器盒连接的单个 USB 端口充分供电。通电后，适配器盒应亮起绿色指示灯。此外，带宽开关应选择 256000，以适应 A2M12 型号所需的高速率。

3. 电池充电：搭载单个 Pi5 的 Leo Rover 可由单个 Leo 电池组供电。带按钮的电池连接到内部电源线，为 Raspberry Pi、轮毂电机、控制器等供电。

   当电池指示灯闪烁绿色时，表示电量不足，需要充电。充电时插入专用 Leo 电池充电器（比普通充电器安培数更高）。插入后，按下指示灯按钮，Leo 电池充电器灯变为红色，表示充电已开始。当电池充满电后，充电器指示灯将再次变为绿色。

   **警告：请勿尝试拆卸 Leo 电池组或充电线缆或接头。Leo Rover 使用特殊的防水电池连接方案，难以更换。在正常情况下，唯一的拆卸点应仅限于电池组与 Rover 或充电器的连接处。**

4. I/O 连接：建议板载 Wi-Fi 天线连接到其中一个 USB2 端口；板载 Sllidar 连接到另一个 USB2 端口。

   Leo Rover 有一个内置的外露 USB 连接端口。如果我们需要连接键盘和鼠标进行调试，这将非常方便。因此我们建议将此外露的外部 USB 端口内部连接到其中一个 USB3.1 端口。

5. （可选）可启动 SSD 驱动器分区。

   原装 Leo Rover 包装附带 Leo OS 安装在 microSD 卡上。microSD 卡通常具有较低的 I/O 带宽和较短的使用寿命及耐久性。因此，建议用自定义 SSD 启动驱动器替换出厂 microSD 卡。SSD 驱动器可以安装到 Pi 5 主板上的第二个 USB3.1 端口。

   自定义 SSD 驱动器可能具有不同的容量，而官方 LeoOS 镜像可能只创建一个很小的启动分区和一个根分区。要充分利用 SSD 容量，请安装 gnome-disks GUI 应用程序，并将根分区大小调整为最大容量。这将允许根目录能够存储更大的 ROS 包和数据。

   最后，如果需要连接外部显示器，我们建议使用 mini-HDMI 连接到其板载 HDMI0 端口。

## 2. 软件安装与开发

### 软件方案

2026 版 Leo Rover 运行在 Raspberry Pi5 上，使用 Ubuntu 24.04 和 ROS2 Jazzy。Rover 通过其板载 wifi 芯片与外部设备连接。虽然板载 wifi 不必连接到另一个 wifi 网络/互联网，但连接到 Rover wifi 对于 SSH 进入任一计算机以及通过 API 服务器/WebSocket 传输信息是必要的。

- 默认 Leo Rover WiFi 密码：password
- 通过 Leo Rover WiFi 登录 Pi5：pi@10.0.0.1，密码：raspberry

如果需要为 Pi5 刷新 Leo OS，请按照 [LeoRover Ros2（实验性）指南](https://docs.fictionlab.pl/leo-rover/advanced-guides/ros-2-support)操作。强烈建议从连接到 USB 3.1 端口的 USB SSD 启动 Leo OS（自定义 Ubuntu 24.04），但也可以使用 microSD。

如果用户希望使用不同的默认 WiFi 名称，特别是在同一空间内有多台 Leo Rover 时，可以通过编辑 hostapd.conf 重置默认 WiFi 名称：

```bash
sudo nano /etc/hostapd/hostapd.conf
```

找到并编辑 ssid 条目为：ssid=LeoRover-XXXX，其中 XXXX 是每台 Leo Rover 应唯一的自定义 WiFi 名称。重启系统使更改生效。

### Pi5 Ubuntu 安装

**步骤 1. 同步系统时钟**

在更新系统之前，我们需要修正 Pi 5 的时钟以建立安全连接，因为 Pi 5 没有硬件时钟。首次使用时，需要手动将系统时钟硬重置为当前本地时间（请替换为正确的当前本地时间）：

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

最后检查 chrony 同步状态：

```bash
timedatectl status
chronyc sources
```

如果 chronyc sources 显示某些时钟服务已正确连接，那么系统时钟将在未来自动重置（只要有 WiFi 连接）。

**步骤 2. 提升 CPU 性能以支持高强度应用**

```bash
sudo apt update
sudo apt upgrade
sudo apt install cpufrequtils
sudo cpufreq-set -g performance
```

要自动设置 CPU 为性能模式，将上面最后一行写入 `/etc/rc.local`。然后执行：

```bash
sudo chmod +x /etc/rc.local
```

**步骤 3. 安装并最大化根分区容量**

```bash
sudo apt install gnome-disk-utility
gnome-disks
```

**步骤 4. 安装 rplidar ROS 包和驱动**

```bash
mkdir -p ~/ws_lidar/src
cd ~/ws_lidar/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ws_lidar
colcon build --symlink-install
source ~/ws_lidar/install/setup.bash
echo "source ~/ws_lidar/install/setup.bash" >> ~/.bashrc
```

**步骤 5. 安装 SLAM 工具箱**

> **架构说明：** Pi5 运行 SLAM 进行定位，并运行 `simple_nav_node` 提供带 LiDAR 避障功能的导航。系统在三种导航模式下运行：**slam**（SLAM TF 可用 + 平地 → 坐标导航）、**dead_reckon**（SLAM 不可用或倾斜 → 仅 EKF 定时驱动）和 **line_follow**（RT 扳机 → 相机转向）。EKF（odom→base_footprint）融合 IMU + 轮编码器在所有模式下持续运行。`slope_pilot_node` 检测地面白色中心线以提供相机转向。`tilt_gate_node` 监控 IMU 倾斜并发布 `/scan_gated` 供未来 SLAM 集成使用。

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install python3-opencv ros-jazzy-cv-bridge
```

> **必需软件包：** `python3-opencv` 和 `ros-jazzy-cv-bridge` 是 `slope_pilot_node` 的必需依赖。没有它们节点将无法启动。相机**硬件**是可选的——如果未连接相机，节点将自动回退到仅 IMU 倾斜检测模式（flat/dead_reckon 模式，无循线跟踪）。

**步骤 6. 更新 Leo 控制板固件**

请按照文档更新 Leo 控制板固件：
[如何更新 Leo Rover 固件](https://docs.fictionlab.pl/leo-rover/guides/firmware-update)

## 3. 构建 Leo Rover SLAM 服务器和 API 服务器

**3.1** 设置工作空间，首先克隆 Hitch Play 项目，然后将 `src` 文件夹移至 `~/leo_ws` 文件夹：

```bash
mkdir ~/projects
cd ~/projects
git clone https://github.com/intelligentracing/HitchPlay
mkdir ~/leo_ws
cd HitchPlay
mv src ~/leo_ws
```

**3.2** 在从源码构建之前，安装一些 ROS2 Jazzy 包：

```bash
sudo apt update
sudo apt install ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-sensor-msgs
cd ~/leo_ws
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```

接下来，由于 Leo OS 启动时会默认启动 rosbridge_websocket，我们需要将其从启动项中移除。具体编辑：

```bash
sudo nano /opt/ros/jazzy/share/leo_bringup/launch/leo_bringup.launch.xml
```

注释掉以下行：

```xml
<!--
<node name="rosbridge_server"
      pkg="rosbridge_server"
      exec="rosbridge_websocket">
</node>
-->
```

修改 rosbridge 服务器设置：

```bash
sudo nano /opt/ros/jazzy/share/rosbridge_server/launch/rosbridge_websocket_launch.xml
```

将消息间延迟从 0 改为 0.0：

```xml
<arg name="delay_between_messages" default="0.0" />
```

更改将在重启后生效。

## 4. 配置 LiDAR 扫描频率（5 Hz）

将 LiDAR 运行在 5 Hz（而不是默认的约 10-11 Hz）可以显著降低 Pi5 上的 CPU 负载，这对保持 SLAM 响应性至关重要。较高的扫描速率会导致 SLAM 消息队列溢出，造成 TF 变换过时和定位精度下降。

创建一个设置正确扫描频率并禁用 RViz GUI 的自定义启动文件：

**4.1** 复制原始启动文件以创建自定义版本：

```bash
cp ~/ws_lidar/src/sllidar_ros2/launch/view_sllidar_a2m12_launch.py \
   ~/ws_lidar/src/sllidar_ros2/launch/view_sllidar_a2m12_nogui_launch.py
```

**4.2** 编辑自定义启动文件：

```bash
nano ~/ws_lidar/src/sllidar_ros2/launch/view_sllidar_a2m12_nogui_launch.py
```

**4.3** 删除整个 rviz2 节点块：

```python
# 删除整个此块：
Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_dir],
    output='screen'),
```

**4.4** 在 sllidar_ros2 节点参数部分添加扫描频率参数：

```python
'scan_frequency': 5.0,
```

**4.5** 重新构建工作空间使更改生效：

```bash
cd ~/ws_lidar
colcon build --symlink-install
```

**4.6** 启动后验证扫描频率（在另一个终端中）：

```bash
# 应该显示约 5 Hz，而不是 10-11 Hz
ros2 topic hz /scan

# 同时验证参数是否设置正确
ros2 param get /sllidar_node scan_frequency
```

> **为什么选择 5 Hz？** 在 Pi5 上以 10+ Hz 运行时，SLAM 工具箱无法足够快地处理扫描数据，导致其消息队列填满。这会导致 map 到 odom 的 TF 变换过时（测试中观察到延迟超过 100 秒），从而导致导航命令失败并提示"Initial robot pose is not available"。在 5 Hz 下，SLAM 有足够的 CPU 余量将 TF 延迟保持在 1 秒以下。

## 5. 启动 ROS 节点和 API 服务器

整个启动序列已编码在 `~/leo_ws/src/` 中的脚本文件 `start_all.sh` 中。

以下是所有服务的详细说明：

**5.1** Leo 系统（由 systemd 管理）

Leo 基础系统（`leo_system`、`odom_filter`、`imu_filter`、`robot_state_publisher`、`firmware_message_converter`）在启动时由 `ros-nodes.service` systemd 用户单元自动启动。`start_all.sh` 检测到此情况后会跳过重复启动。

> **警告：** 切勿手动杀死这些 systemd 管理的服务（`pkill leo_system`、`pkill odom_filter` 等）。它们不会重新启动，Rover 将失去其 TF 链。如需重启：`systemctl --user restart ros-nodes.service`

**5.2** 启动 Sllidar（5 Hz，无 GUI）

```bash
source ~/ws_lidar/install/setup.bash
ros2 launch sllidar_ros2 view_sllidar_a2m12_nogui_launch.py
```

**5.3** 添加 tf 变换

```bash
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher --x 0.03 --y 0 --z 0.02 --yaw 3.14159 --pitch 0 --roll 0 --frame-id base_link --child-frame-id laser
```

**5.4** 启动倾斜门控节点（斜坡监控）

倾斜门控节点监控 IMU 横滚/俯仰角并将门控扫描发布到 `/scan_gated`。当前 SLAM 直接读取原始 `/scan`（门控话题可供未来集成使用）。倾斜检测还与 `slope_pilot_node` 协调以触发斜坡模式。

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/leo_ws/src/tilt_gate_node.py
```

**5.5** 启动斜坡导航节点（相机循线检测）

斜坡导航节点使用相机检测地面上的白色中心线。它在 `/slope_mode` 上发布线检测状态，在 `/line_steer` 上发布转向角。导航模式不由 slope_pilot_node 控制——`simple_nav_node` 在内部确定模式。循线跟踪通过用户在 Xbox 控制器上按下 RT 扳机触发。

> **相机硬件是可选的。** 如果 `/camera/image_rect_color` 上没有相机在发布数据，节点将以仅 IMU 模式运行：在平地上发布 `flat`，在斜坡上发布 `dead_reckon`（无 `line_follow`）。每 5 秒重新检测一次相机，因此支持运行期间热插拔相机。如果 `slope_pilot_node` 完全失败，`simple_nav_node` 有其自身的 IMU 回退机制。

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/leo_ws/src/slope_pilot_node.py
```

> **调参：** 如果循线检测不可靠，请在 `slope_pilot_node.py` 中调整 `WHITE_V_MIN`（亮度阈值，默认 180）、`MIN_LINE_AREA`（最小色块面积，默认 200）和 `KP_STEER`（转向灵敏度，默认 2.0）。

**5.6** 启动 SLAM 工具箱

SLAM 使用默认配置（订阅 `/scan`，`base_frame: base_footprint`）：

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

**5.7** 启动简易导航节点

简易导航节点是 Rover 的核心速度管控器。它管理三种导航模式（slam/dead_reckon/line_follow），提供目标坐标导航和回家功能、LiDAR 避障以及 Pi5 侧的路径点回放引擎（使用纯跟踪路径跟随）。所有速度命令（来自遥控、导航和路径点回放）在到达 `/cmd_vel` 之前都经过避障过滤。Xbox/键盘控制器发布到 `/cmd_vel_teleop`，`simple_nav_node` 过滤后转发到 `/cmd_vel`。节点在 `/nav_mode` 上发布当前模式。路径点回放命令以 JSON 格式通过 `/waypoint_replay` 到达。循线跟踪命令通过 `/line_follow_cmd` 到达（Xbox RT 扳机）。

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/leo_ws/src/simple_nav_node.py
```

> **注意：** 避障功能在 LiDAR 检测到行驶方向有障碍物时会减速并停止 Rover。转向始终允许。导航目标使用直线路径（不围绕障碍物进行路径规划）。

从命令行发送导航命令：

```bash
# 前往特定坐标（x=1.0, y=2.0）
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}"

# 回家（返回原点）
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}"

# 取消当前导航
ros2 topic pub --once /cancel_nav std_msgs/msg/Empty "{}"

# 监控导航状态
ros2 topic echo /nav_status
```

**5.8** 启动自定义 WebSocket

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**5.9** 启动 API 服务器

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/leo_ws/src/api-server/main.py
```

**5.10** Web 视频服务器（相机流）

> **注意：** Web 视频服务器由 `leo_bringup.launch.xml`（systemd）自动启动。`start_all.sh` 检测到此情况后不会重复启动。无需手动操作。

**5.11** 启动 RViz（可选，仅在远程 PC 上）

> **注意：** 请勿在 Pi5 上运行 RViz，因为它会消耗大量 CPU 和 GPU 资源。在连接到同一 ROS2 网络的远程 PC 上运行。

```bash
source ~/leo_ws/install/setup.bash
ros2 launch leo_viz rviz.launch.xml
```

### 预期 Pi5 CPU 负载

使用此配置，Pi5 应在 4 核上保持 2.0 到 3.0 的平均负载：

| 进程 | 约占 CPU | 用途 |
|------|:--------:|------|
| leo_system + firmware | ~10% | 电机控制、硬件 I/O（systemd） |
| sllidar_node (~13 Hz) | ~10% | LiDAR 驱动 |
| odom_filter | ~10% | EKF 里程计融合（systemd） |
| slam_toolbox | ~15% | 定位与建图 |
| tilt_gate_node | ~1% | IMU 倾斜监控、/scan_gated 发布 |
| slope_pilot_node | ~1% | 相机循线检测（懒加载：平地 0%，斜坡约 3%） |
| robot_state_publisher | ~2% | URDF 连杆 TF 树（systemd） |
| static_transform_publisher | ~1% | base_link 到 laser TF |
| simple_nav_node | ~5% | 导航 + 避障 + 三模式 + 回放 |
| rosbridge_websocket | ~5% | API/Web 客户端 WebSocket 桥接 |
| web_video_server | ~5% | 相机流（systemd） |
| **总计** | **~65%** | **为稳定性留有余量** |

### 验证 SLAM 健康状态

启动后，验证 SLAM TF 变换是否健康：

```bash
ros2 run tf2_ros tf2_monitor map odom
```

如果平均延迟超过 5 秒，则 SLAM 正在落后。常见原因包括 LiDAR 运行频率超过 5 Hz、其他 CPU 密集型进程或 Pi5 热节流。

## 6. 设置开机自启动

要使 Leo Rover 在开机时自动启动所有服务，创建一个在启动时运行 `start_all.sh` 的 systemd 服务。

### 创建 Systemd 服务

1. 创建服务文件：

```bash
sudo nano /etc/systemd/system/leo-startup.service
```

2. 添加以下内容：

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

3. 重新加载 systemd，启用并启动：

```bash
sudo systemctl daemon-reload
sudo systemctl enable leo-startup.service
sudo systemctl start leo-startup.service
```

### 管理服务

```bash
sudo systemctl status leo-startup.service
sudo systemctl stop leo-startup.service
sudo systemctl restart leo-startup.service
sudo systemctl disable leo-startup.service
```

## 7. 自定义驱动包用于机器人控制

custom_drive_pkg 提供使用相对移动命令和顺序导航自动化控制 Leo Rover 的服务。

**完整文档、使用示例和 API 集成指南请参见：**

[custom_drive_pkg/README.md](./custom_drive_pkg/README.md)

该包包括：
- **原始驱动服务** — 单命令导航，立即响应
- **顺序驱动服务** — 多步骤自动化导航，用于 AI 智能体集成

两种服务都通过上述步骤 3.2 自动构建，可以通过 `start_all.sh` 或按照 README 中的文档单独启动。

## 8. 保存和重用地图

SLAM 工具箱可以保存当前地图以供后续使用。首先创建地图目录，然后使用 slam_toolbox 服务序列化地图：

```bash
mkdir -p ~/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/pi/maps/my_map'}"
```

要在下次启动时加载已保存的地图，请在 SLAM 启动命令中添加 `map_file_name` 参数。

## 附录 A：故障排除

### SLAM TF 延迟

如果导航失败并提示"Initial robot pose is not available"，检查 SLAM TF 延迟：

```bash
ros2 run tf2_ros tf2_monitor map odom
```

平均延迟应低于 1 秒。如果更高，请检查 CPU 负载（`top`）、Pi5 温度（`vcgencmd measure_temp` 应低于 80°C），以及 systemd 基础服务是否正在运行（`systemctl --user status ros-nodes.service`）。

### /goal_pose 的 QoS 不匹配

如果发布到 `/goal_pose` 没有效果，检查 QoS 不兼容：

```bash
ros2 topic info /goal_pose --verbose
```

`simple_nav_node` 使用默认的 RELIABLE QoS。确保发布者也使用 RELIABLE（这是 `ros2 topic pub`、rosbridge 和 `xbox_leo_unified.py` 的默认设置）。

### 验证完整 TF 链

所需链：`map` → `odom` → `base_footprint` → `base_link` → `laser`。验证每个链接：

```bash
ros2 run tf2_ros tf2_echo map odom                # SLAM
ros2 run tf2_ros tf2_echo odom base_footprint      # EKF (odom_filter)
ros2 run tf2_ros tf2_echo base_footprint base_link # robot_state_publisher
ros2 run tf2_ros tf2_echo base_link laser          # static_transform_publisher
```

如果任何链接无限期显示"Waiting for transform"，则相应的节点未运行。基础服务（`odom_filter`、`robot_state_publisher`）由 `systemctl --user` 管理，可通过以下命令重启：`systemctl --user restart ros-nodes.service`

### 导航模式系统（三模式）

导航模式由 `simple_nav_node` 内部确定并发布在 `/nav_mode` 上：

| 模式 | 条件 | 转向 | 定位 |
|------|------|------|------|
| `slam` | SLAM TF 可用 + 平地 | 全手动 | SLAM + EKF（IMU + 轮编码器） |
| `dead_reckon` | SLAM 不可用或倾斜 > 10° | 全手动 | 仅 EKF（IMU + 轮编码器） |
| `line_follow` | Xbox 控制器 RT 扳机按住 | 相机自动转向 | EKF（IMU + 轮编码器） |

EKF（odom→base_footprint）在所有模式下持续运行。循线跟踪由用户显式触发（RT 扳机），而不是由倾斜自动触发。

监控导航模式：

```bash
ros2 topic echo /nav_mode            # 当前模式：slam/dead_reckon/line_follow
ros2 topic echo /slope_mode          # slope_pilot_node 的相机循线检测
ros2 topic echo /line_steer          # 循线转向输出
ros2 topic echo /nav_status          # 导航状态
```

**循线检测调参：** 如果白线检测不可靠，请在 `slope_pilot_node.py` 中调整：`WHITE_V_MIN`（亮度阈值，默认 180 — 光线暗时降低）、`MIN_LINE_AREA`（最小色块面积，默认 200 — 细线时降低）、`FLOOR_ROI_TOP`（图像中地板区域比例，默认 0.50 — 相机更朝下时增大）。

### 路径点录制与回放

路径点系统以 10Hz 录制驾驶员路径，并在 Pi5 侧使用**纯跟踪**控制器自主回放，实现通过导航路径点时的平滑连续运动。

**三种路径点类型**（根据当前状态自动录制）：

| 类型 | 录制时机 | 格式 | 回放行为 |
|------|----------|------|----------|
| `nav` | slam 模式（SLAM 可用） | `(nav, x, y, yaw)` | 通过坐标进行纯跟踪 |
| `dead_reckon` | dead_reckon 模式（无 SLAM 或倾斜） | `(dead_reckon, speed, steer, duration)` | 定时驱动 + 录制的转向 |
| `line_follow` | 录制期间按住 RT 扳机 | `(line_follow, speed, duration)` | 相机转向，无线时停车 |

**录制（LB = 开始，RB = 停止）：**
- 按 LB 开始新录制（清除之前的列表，以 10Hz 录制）
- 正常驾驶 — 路径点类型由当前模式自动确定
- 如果录制期间按住 RT，这些片段将被录制为 `line_follow`
- 按 RB 停止并保存到 Windows/PC 端的 `waypoint_list.json`

**回放（LT 按住 = 回放，松开 = 停止）：**
- 按住 LT 将完整路径点列表发送到 Pi5 并从最近的路径点开始回放
- Pi5 `simple_nav_node` 以 20Hz 执行并带有避障过滤
- 导航路径点：纯跟踪（前瞻 30cm，15cm 内消耗，平滑弧线）
- 航位推算路径点：回放录制的速度和转向以忠实再现转弯
- 循线路径点：以录制速度前进 + 相机转向（无线可见时停车）
- 如果 SLAM 不可用，导航路径点将被跳过（dead_reckon 和 line_follow 仍然执行）
- 到达最后一个路径点后循环回第一个
- 松开 LT 停止回放。也可通过 A/C 或 E-Stop 取消

**循线跟踪（RT 按住 = 跟踪，松开 = 停止）：**
- 按住 RT 以预设速度开始自主循线跟踪
- 如果相机检测到白线，Rover 将前进并使用相机转向
- 如果无线可见或无相机，Rover 停止但保持模式激活（出现线时恢复）
- 松开 RT 停止。也可通过 A/C 或 E-Stop 取消
- 可以与录制同时激活（line_follow 片段会被录制）

**纯跟踪调参**（`simple_nav_node.py` 顶部常量）：

| 常量 | 默认值 | 效果 |
|------|--------|------|
| `LOOKAHEAD_DIST` | 0.30m | 更大 = 更平滑的曲线但会切弯 |
| `CONSUME_RADIUS` | 0.15m | 路径点被视为"已通过"的距离 |
| `PURSUIT_SPEED` | 0.25 m/s | 回放时的巡航速度 |
| `PURSUIT_MIN_SPEED` | 0.10 m/s | 急弯时的最低速度 |
| `CURVATURE_SLOW` | 0.10 | 每 rad/s 转向速率的速度降低量 |
| `LINE_FOLLOW_SPEED` | 0.20 m/s | 自主循线跟踪时的前进速度 |
| `OBSTACLE_STOP_DIST` | 0.40m | 避障全停距离 |

**持久化：** 路径点以 JSON 格式保存在 Windows 端。重启后，按住 LT 从已保存的文件加载并重新发送到 Pi5。旧文件中的 `"slope"` 类型在加载时自动转换为 `"dead_reckon"`。

```bash
# 在 Pi5 上监控回放状态
ros2 topic echo /nav_status
# 显示："replaying: Replaying N waypoints from #K" 或 "replay_stopped: ..."
```
