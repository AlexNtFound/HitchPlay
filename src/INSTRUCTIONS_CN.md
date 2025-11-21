# 使用 Raspberry Pi 5（2025 版）将 Leo Rover 搭建为 ROS2 服务器的安装说明

本文档基于 2024 年在 Raspberry Pi 4 搭配 NVIDIA Jetson Nano 的早期实现版本编写。早期版本见：<https://github.com/Arthios09/LeoRover-SLAM-ROS2>。请注意：2024 版与最新的 Leo Rover OS 以及 ROS2 Jazzy **不兼容**。

## 1. 为 Raspberry Pi 5 与 Leo Rover 硬件做准备

首先检查车载 Pi 5 主板的状态以及与 LeoCore 控制板的连接情况。因为我们计划在 Pi 5 上直接运行计算密集型的 SLAM 与 Nav2 ROS 节点，所以 Pi 5 的工作状态对正确运行至关重要：

1. 请确保 Pi 5 **必须**配备有效的主动散热方案，例如：  
   * 官方主动散热：<https://www.raspberrypi.com/products/active-cooler/>

2. 请确认 **Sllidar** 工作正常。Sllidar 通过其适配盒的 **单个 USB 接口** 供电即可满足功耗需求。上电后，适配盒应有绿色指示灯常亮；并将 **带宽开关** 调到 **256000**，以匹配 A2M12 机型的高频率输出需求。

3. **电池充电**：仅搭载一块 Pi 5 的 Leo Rover 可由一块 Leo 电池包供电。带按键的电池通过内部供电线为 Raspberry Pi、轮毂电机、控制器等供电。  
   当电池指示灯 **绿色闪烁** 时，表示电量较低，需要充电。充电时请使用专用的 **Leo 电池充电器**（电流更大，优于普通充电器）。接上后按下电量指示按钮，充电器指示灯会变为 **红色**（开始充电）。当电量充足后，指示灯会重新变为 **绿色**。

   **警告**：请勿尝试拆解 Leo 电池包或其充电线缆/连接器。Leo Rover 采用特殊的**防水电池连接方案**，更换困难。正常情况下，唯一可拆卸的位置仅限于电池包与 Rover 或充电器之间的连接。

4. **I/O 连接建议**：建议将车载 **Wi‑Fi 天线** 接在 **一个 USB 2.0 口**；将 **Sllidar** 接在 **另一个 USB 2.0 口**。  
   Leo Rover 机身外部预留了一个 **USB 直通接口**，用于现场调试时外接键鼠，强烈建议将该外部 USB 口内部连接到 **其中一个 USB 3.1 口**。

<span style="color:red;">
5. （可选）**可启动的 SSD 分区**。

Leo Rover 原厂镜像默认写在 microSD 卡上。相较于 SSD，microSD 的 I/O 带宽与寿命/耐久度都较差。推荐**使用 SSD 取代 microSD** 作为系统启动盘，并将 SSD 插在 Pi 5 的 **第二个 USB 3.1 口**。

自制 SSD 的容量可能不同，官方 LeoOS 镜像通常只会创建一个很小的 **boot 分区**和一个 **root 分区**。为充分利用 SSD 容量，建议安装 **gnome‑disks** 图形工具，将 **root 分区扩容至最大**，以便后续能在根目录下存放更大的 ROS 包与数据。

若需要连接外接显示器，建议将 **mini‑HDMI** 连接到板载 **HDMI0** 端口。
</span>

## 2. 软件安装与开发

### 软件总体方案

2025 版 Leo Rover 运行环境为 **Raspberry Pi 5 + Ubuntu 24.04 + ROS2 Jazzy**。Rover 对外设备的连接通过板载 Wi‑Fi 完成。即使板载 Wi‑Fi 不连接到其他网络/互联网，也需要连接到 **Rover 自身的 Wi‑Fi**，以便通过 SSH 登录并通过 API 服务器/WebSocket 传输信息。

* Leo Rover **默认 Wi‑Fi 密码**：`password`
* 通过 Leo Rover Wi‑Fi 登录 Pi 5 的 **默认账号**：`pi@10.0.0.1`，**密码**：`raspberry`

如果需要为 Pi 5 重新刷入 Leo OS，请参考[官方 LeoRover ROS2（实验性）指南](https://docs.fictionlab.pl/leo-rover/advanced-guides/ros-2-support)。**强烈建议**将 **Leo OS（基于 Ubuntu 24.04 的定制版）** 安装并从 **USB 3.1 接口的 SSD** 启动；也可使用 microSD。

### 在 Pi 5 上安装 Ubuntu 的要点

1. **同步系统时钟**

在更新系统之前，我们需要先修正 Pi 5 的系统时钟以建立安全连接，因为 Pi 5 没有硬件时钟。首次启动时，请**手动**将系统时间设置为**当前本地时间**（替换为你的实际时间）：  
`sudo date -s "2025-01-22 12:00:00"`

```bash
sudo apt update
sudo apt upgrade
# install chrony
sudo apt install chrony -y
# 停止并禁用自带的 timesyncd
sudo systemctl stop systemd-timesyncd
sudo systemctl disable systemd-timesyncd
# 启用并启动 chrony
sudo systemctl enable chrony
sudo systemctl start chrony
```

最后检查 chrony 同步状态：

```bash
timedatectl status
chronyc sources
```

若 `chronyc sources` 显示已成功连接到时间服务，则今后（只要有 Wi‑Fi 连接）系统时钟会自动校正。

2. **为高强度应用提升 CPU 性能**

```bash
sudo apt update
sudo apt upgrade
sudo apt install cpufrequtils
sudo cpufreq-set -g performance
```
如需在开机自动设置为 performance，可将上一行写入 **/etc/rc.local**，然后：

```bash
sudo chmod +x /etc/rc.local
```

3. **安装并最大化 root 分区容量**

```bash
sudo apt install gnome-disk-utility
gnome-disks
```

4. **安装 rplidar 的 ROS 包与驱动**

```bash
# 创建工作区
mkdir -p ~/ws_lidar/src
cd ~/ws_lidar/src

# 克隆 Slamtec 官方 ROS2 驱动
git clone https://github.com/Slamtec/sllidar_ros2.git
# 编译工作区
cd ~/ws_lidar
colcon build --symlink-install
# 加载环境
source ~/ws_lidar/install/setup.bash
echo "source ~/ws_lidar/install/setup.bash" >> ~/.bashrc
```

5. **安装 Nav2 与 SLAM Toolbox**

```bash
# 更新软件源
sudo apt update
# 安装 Nav2
sudo apt install ros-jazzy-navigation2 ros-$ROS_DISTRO-nav2-bringup
#（可选但推荐）安装更多 Nav2 组件
sudo apt install ros-jazzy-nav2-*
# 依赖项
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-robot-state-publisher
```

SLAM Toolbox 通常会随 ROS2 Jazzy 与 Nav2 一并安装；若未安装，可执行：
```bash
sudo apt update
# 安装 SLAM Toolbox
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan
# 其他依赖
sudo apt install ros-jazzy-nav2-map-server
sudo apt install ros-jazzy-nav2-lifecycle-manager
```

6. **可选：安装 RustDesk 远程桌面共享软件**
```bash
cd ~
wget https://github.com/rustdesk/rustdesk/releases/download/1.4.3/rustdesk-1.4.3-aarch64.deb
sudo apt install ./rustdesk-1.4.3-aarch64.deb
```


## 3. 构建 Leo Rover 的 SLAM 服务器与 API 服务器

1. **准备工作区**：先克隆 Hitch Play 项目，再将其中的 **src** 目录移动到 **~/leo_ws**

```bash
mkdir ~/projects
cd ~/projects
git clone https://github.com/intelligentracing/HitchPlay

mkdir ~/leo_ws
cd HitchPlay
mv src ~/leo_ws
```

2. **从源码构建前，安装 ROS2 Jazzy 依赖**

```bash
sudo apt update
sudo apt install ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-sensor-msgs
cd ~/leo_ws
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```

3. **安装 ROS2 API 服务器**

先安装一个开源 API 服务器：

```bash
cd ~/projects
git clone -b alex_cmd https://gitlab.com/roar-gokart/api-server.git
cd api-server/
sudo apt install python3-fastapi
python3 main.py
```

随后，由于 Leo OS 会在启动时默认拉起 **rosbridge_websocket**，我们需要将其从自启动中移除。编辑：
```bash
sudo nano /opt/ros/jazzy/share/leo_bringup/launch/leo_bringup.launch.xml
```

将以下内容注释掉：
```script
<!--
<node name="rosbridge_server"
      pkg="rosbridge_server"
      …
      exec="rosbridge_websocket">
</node>
-->
```

重启后生效。

## 4. 启动 ROS 节点与 API 服务器

我们已将完整的启动顺序封装在 **HitchPlay/src** 中的脚本 **start-all.sh** 里。下面给出各服务的详细拆解步骤：

1. **启动 Leo 系统**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run leo_bringup leo_system
```

2. **启动 Sllidar**

在启动前，我们希望自定义 Sllidar 的扫描频率，并禁用 RViz 以节省 CPU。做法是基于官方的 `view_sllidar_a2m12_launch.py` 新增一个 **view_sllidar_a2m12_nogui_launch.py**：

首先，删除 rviz2 节点：

```python
Note (
   package = 'rviz2',
   executable='rviz2',
   name='rviz2',
   arguments=['-d', rviz_config_dir],
   output='screen'),
```
然后，在 `sllidar_ros2` 节点中加入扫描频率参数：

```python
'scan_frequency': 5.0
```

最后，启动 sllidar 节点：
```bash
source ~/ws_lidar/install/setup.bash
ros2 launch sllidar_ros2 view_sllidar_a2m12_nogui_launch.py
```

3. **增加 tf 变换**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher --x 0.03 --y 0 --z 0.08 --yaw 3.14159 --pitch 0 --roll 0 --frame-id base_link --child-frame-id laser
```

4. **启动 RVIZ**

```bash
source install/setup.bash
ros2 launch leo_viz rviz.launch.xml
```

5. **启动 Nav2 与 SLAM_Toolbox**

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

6. **启动自定义的 WebSocket**

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
