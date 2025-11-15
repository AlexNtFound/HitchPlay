## 5. Using Custom Drive Package for Relative Movement Commands

The `custom_drive_pkg` provides a persistent service that converts relative movement commands (e.g., "move forward 2m" or "turn right 90°") into absolute navigation goals for Nav2.

### Building the Package

Already built with step 3.2, or rebuild separately:
```bash
cd ~/leo_ws
colcon build --symlink-install --packages-select custom_drive_pkg
source install/setup.bash
```

### Starting the Drive Service

The drive service runs continuously and accepts multiple commands without restarting:
```bash
source ~/leo_ws/install/setup.bash
ros2 run custom_drive_pkg drive_service
```

**Note:** If using `start_all.sh`, the drive service starts automatically with all other systems.

### Usage

**Basic syntax:**
```bash
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand "{forward: <meters>, rotate: <degrees>}"
```

**Parameters:**
- `forward`: Distance in meters (positive = forward, negative = backward)
- `rotate`: Angle in degrees (positive = left/counter-clockwise, negative = right/clockwise)

### Examples
```bash
# Move forward 2m
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand "{forward: 2.0, rotate: 0.0}"

# Turn right 90°
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand "{forward: 0.0, rotate: -90.0}"

# Turn left 90°
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand "{forward: 0.0, rotate: 90.0}"

# Move forward 1.5m and turn left 45°
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand "{forward: 1.5, rotate: 45.0}"

# Move backward 1m
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand "{forward: -1.0, rotate: 0.0}"
```

### Important Notes

1. **Prerequisites:** Ensure Leo system, Sllidar, TF transform, SLAM, and Nav2 are running (use `start_all.sh`)

2. **Service must be running:** The drive service must be active to accept commands. Check with:
```bash
   ros2 service list | grep drive_command
```

3. **Wait between commands:** Allow 10-15 seconds between commands for navigation to complete

4. **Advantages over drive_publisher:**
   - No connection delay - service stays running
   - Faster command execution
   - Single command per movement
   - Automatically cancels previous navigation

5. **Troubleshooting:**
   - **"Service not available"** → Check drive service is running: `ros2 node list | grep drive_service`
   - **"Failed to get pose"** → Ensure SLAM is running: `ros2 node list | grep slam`
   - **Robot doesn't move** → Check Nav2 terminal for "Reached the goal!" message
   - **"Could not find connection between 'map' and 'base_link'"** → Verify TF tree: `ros2 run tf2_ros tf2_echo map base_link`

6. **Coordinate system:** Robot moves forward relative to its current orientation in the map frame. Nav2 handles all path planning and obstacle avoidance automatically.
