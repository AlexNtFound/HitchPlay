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

4. I/O Connection: It is recommended that the onboard Wi-Fi antenna connects to one of the Type-2 USB port; the onboard Sllidar connects to one of the Type-3 USB port. 

Leo Rover has a built-in exposed USB connection port. This will really come in handy if we need to connect a keyboard and mouse for debugging. So we recommend to keep this exposed external USB port connect internally to the second Type-3 USB port. 

Finally, if connecting an external monitor is needed, we recommend connecting the mini-HDMI with its onboard HDMI0 port.

## 2. Software Setup and Development

### Software Scheme:

The 2025 version of Leo Rover runs on Raspberry Pi5 with Ubuntu 24.04 and ROS2 Jazzy. The rover’s connection to external devices is permitted through its onboard wifi chip. Although the onboard wifi does not have to be connected to another wifi network/the internet, connection to the rover’s wifi is necessary to ssh into either computer and transmit information via the api server/websockets.

### Setting Up a New Rover or Computer:

To flash the base onboard RaspberryPi please follow the [LeoRover Ros2 (experimental) guide here](https://docs.fictionlab.pl/leo-rover/advanced-guides/ros-2-support). A microSD, microSD-SD adapter, and a computer are required. 


### Rover Activation and Use:

For instructions and specific commands used to activate the rover, please see the README in the rover repo linked at the start of this document. 

Although many commands, functions, and software abilities are available for use, the essential ROS2 nodes for the rover are:

Raspi - leo_system and rplidar_a2m12_launch.py

Jetson - static transforms, navigation_launch.py, online_async_launch.py (SLAM), ekf.launch.py, zed_camera.launch.py, rosbridge_websocket_launch.xml, and the api server

To activate the api server, cd into api_folder/api_server and run python main.py. See linked documentation on repo. 
All of these functions except the api-server can be started with a single command, “python3 start_rover.py” or “ros2 launch leo_common-ros2 rover_comp_launch.xml“. For development and testing, it is recommended to launch nodes individually to monitor output and restart nodes in the event of failure. 

### Control Computer Setup:

To use a control computer, create a partition and install Ubuntu 22.04, ros2 humble, and necessary dependencies and packages (instructions of ROS2 Humble website). Once this is complete, see the control computer section in the README of the repo to visualize with RVIZ. All nodes can be run from a control computer if desired.

### Development and Troubleshooting:

If you are having issues with running new packages, remember to colcon build, source your workspace, look for online resources (ie: stack exchange, github forums) for solutions, and ask those in the lab for help. Contact me using any of the contacts listed at the top of the document for help. 
For future development purposes, please fork the main repository (or duplicate it under your ownership). Features including object/body detection have great potential for future projects, so consider utilizing these for demos. If the Nvidia Jetson is substituted for the VIA AI Transforma, these functions, and integration with the Zed are no longer possible. In this case, remove the Zed and do not run the Zed activation node. 
Sometimes, the rover’s onboard packages are ahead of the committed/pushed github repo. Please login to your github/create a github key on the rover so that you can check git status and make new commits/pulls from both your computer and the rover.


## 4. Recommendations

### Useful Software Tools:
Nmap - Allows you to scan ports (finding new jetson address on network)

[Apt package manager](https://documentation.ubuntu.com/server/how-to/software/package-management/index.html) - Frequently update your linux system using the apt package management command line tools. 

Solidworks/Tinkercad/Any cad software - Make new attachments for the rover

### Useful Hardware Tools:
Full Hexkey Set - A full set of hex wrenches are necessary for modifying the rover and replacing batteries. These should be accessible in the lab.

Phillips Head Screwdriver - Necessary to dismantle the batter quarters and main electronics box of the rover.

### Qualifications:

The most effective individuals to work on the project will be those that have completed at least EE106a [(see my 106a project using the rover here!)](https://docs.google.com/presentation/d/1WYj237CU580oSIGjfUOlJ4bm8z5D5LlKhrZEYEyIiic/edit?usp=sharing) and ideally 106b, although due to the limited amount of new software developments necessary to fulfill needs for the Qualcomm demo, I do not believe these are strict requirements. If you are looking to recruit more robotics students to work on the team, the spare rover/the rover project in general could potentially be offered to those in 106a looking for a final project (under the research category). Please clear this with Allen, Franco, or the active advisor and team lead.
The most important qualifications for working with the rover are an interest in robotics and a desire to learn ROS/CAD/Network architecture. When I began working with the rover, I had not taken either robotics course and had never used Linux, ROS, or C++. The rover is a great platform for learning and experimenting with, so it is a great tool for those looking to gain new skills and begin involvement in lab research. 
Make sure to upkeep the physical components that see stress. The wheels/wheel attachment bar but have their screws tightened and be readjusted every so often so as to prevent failure.

### Suggested Changes/Additions:

Remove SSH password 
Some of the nodes, especially SLAM toolbox, occasionally fail/are unreliable beyond 30 minutes to an hour. This should ideally be fixed.
Request the purchase of the LeoRover charging station for in use charging and battery hotswapping. Will save time not having to reactivate rover (~5-6 minute process) and will reduce stress caused by battery charging/life during live demos.
Do something cool with object/body detection. Read through the zed ROS object/body tracking sections for topic information and use. See my 106a project for potential inspiration on flashy use cases.
Nav2 is occasionally clunky, so it may be worthwhile long term to write a custom nav stack. Please ask for Michael Wu’s contact as he may have suggestions/advice.
Deadman’s switch - As soon as connection via ssh/websocket/apiserver is lost for X time, auto run return to base. Shouldn't be very hard but would be a nice feature to implement. 
Robotic arm attachment would allow for more complex tasks, however would increase compute load significantly.
If switching permanently to VIA Transforma, investment in a new 3D LiDAR (as opposed to the current 2D) may be necessary to allow for object/body tracking functions.



