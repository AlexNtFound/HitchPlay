# Custom Drive Package

The `custom_drive_pkg` provides ROS2 services that convert relative movement commands into absolute navigation goals for Nav2. The package includes two services for different use cases:

1. **Original Drive Service** - Single-command quick navigation
2. **Sequential Drive Service** - Multi-step automated navigation sequences

---

## 📁 Package Structure

```
custom_drive_pkg/
├── msg/
│   └── DriveStep.msg                    # Single command message type
├── srv/
│   └── DriveCommand.srv                 # Service definition (shared by both services)
├── src/
│   ├── drive_publisher.cpp              # Goal publisher node
│   ├── drive_service.cpp                # Original single-command service
│   └── sequential_drive_service.cpp     # Sequential execution service
├── CMakeLists.txt
└── package.xml
```

---

## 🔧 Building the Package

```bash
cd ~/leo_ws
colcon build --symlink-install --packages-select custom_drive_pkg
source install/setup.bash
```

---

## 📋 Service 1: Original Drive Service

### Overview
Persistent service that converts relative movement commands (e.g., "move forward 2m" or "turn right 90°") into absolute navigation goals for Nav2. Returns immediately after publishing the goal.

### Starting the Service

```bash
source ~/leo_ws/install/setup.bash
ros2 run custom_drive_pkg drive_service
```

**Note:** If using `start_all.sh`, the drive service starts automatically with all other systems.

### Usage

**Basic Syntax:**
```bash
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand \
  "{forward: <meters>, rotate: <degrees>}"
```

**Parameters:**
- `forward`: Distance in meters (positive = forward, negative = backward)
- `rotate`: Angle in degrees (positive = left/CCW, negative = right/CW)

### Examples

```bash
# Move forward 2m
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand \
  "{forward: 2.0, rotate: 0.0}"

# Turn right 90°
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand \
  "{forward: 0.0, rotate: -90.0}"

# Turn left 90°
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand \
  "{forward: 0.0, rotate: 90.0}"

# Move forward 1.5m and turn left 45°
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand \
  "{forward: 1.5, rotate: 45.0}"

# Move backward 1m
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand \
  "{forward: -1.0, rotate: 0.0}"
```

### Characteristics
- ✅ Single command per call
- ✅ Immediate response (doesn't wait for completion)
- ✅ Multiple commands require multiple service calls
- ✅ Perfect for manual control and quick movements
- ⚠️ No automatic sequencing or completion tracking

---

## 📋 Service 2: Sequential Drive Service

### Overview
Advanced service that accepts **multiple sequential commands** in a single call and executes them one by one, waiting for each step to complete before moving to the next. Designed for autonomous navigation and AI agent integration.

### Starting the Service

```bash
source ~/leo_ws/install/setup.bash
ros2 run custom_drive_pkg sequential_drive_service
```

**With Custom Parameters:**
```bash
ros2 run custom_drive_pkg sequential_drive_service --ros-args \
  -p base_frame:=base_link \
  -p goal_timeout:=300.0
```

**Parameters:**
- `base_frame`: Robot's base frame (default: `base_link`)
- `goal_timeout`: Max seconds to wait for each goal (default: `300.0`)

### Usage

**Basic Syntax:**
```bash
ros2 service call /sequential_drive_command custom_drive_pkg/srv/DriveCommand \
  "{commands: [
    {forward: <meters>, rotate: <degrees>},
    {forward: <meters>, rotate: <degrees>},
    ...
  ]}"
```

**Command Parameters (same as original):**
- `forward`: Distance in meters (positive = forward, negative = backward)
- `rotate`: Angle in degrees (positive = left/CCW, negative = right/CW)

### Examples

```bash
# Simple sequence: Forward 2m, turn right 90°, forward 1m
ros2 service call /sequential_drive_command custom_drive_pkg/srv/DriveCommand \
  "{commands: [
    {forward: 2.0, rotate: 0.0},
    {forward: 0.0, rotate: -90.0},
    {forward: 1.0, rotate: 0.0}
  ]}"
```

```bash
# Complex path: Navigate a square
ros2 service call /sequential_drive_command custom_drive_pkg/srv/DriveCommand \
  "{commands: [
    {forward: 2.0, rotate: 0.0},
    {forward: 0.0, rotate: -90.0},
    {forward: 2.0, rotate: 0.0},
    {forward: 0.0, rotate: -90.0},
    {forward: 2.0, rotate: 0.0},
    {forward: 0.0, rotate: -90.0},
    {forward: 2.0, rotate: 0.0},
    {forward: 0.0, rotate: -90.0}
  ]}"
```

```bash
# Mixed movements
ros2 service call /sequential_drive_command custom_drive_pkg/srv/DriveCommand \
  "{commands: [
    {forward: 3.0, rotate: 0.0},
    {forward: 0.0, rotate: 90.0},
    {forward: 1.5, rotate: 45.0},
    {forward: -1.0, rotate: 0.0}
  ]}"
```

### Response Format

```python
response.success         # bool: True if all commands completed
response.message         # str: Status message
response.completed_steps # int: Number of steps completed before any failure
```

### Characteristics
- ✅ Unlimited commands in single call
- ✅ Sequential execution with completion waiting
- ✅ Progress tracking (reports completed steps)
- ✅ Failure handling (stops and reports which step failed)
- ✅ Perfect for autonomous navigation and AI agents
- ✅ Uses Nav2 action client for reliable navigation

### Monitoring

The service provides detailed logging for each step:

```
════════════════════════════════════════
📋 Received 4 sequential commands
════════════════════════════════════════
→ Step 1/4: forward=3.00m, rotate=0.0°
  Goal: (3.50, 2.10) yaw: 45.0°
✓ Step 1/4 COMPLETED
→ Step 2/4: forward=0.00m, rotate=-90.0°
  Goal: (3.50, 2.10) yaw: -45.0°
✓ Step 2/4 COMPLETED
→ Step 3/4: forward=1.00m, rotate=0.0°
  Goal: (4.20, 1.40) yaw: -45.0°
✓ Step 3/4 COMPLETED
→ Step 4/4: forward=0.00m, rotate=180.0°
  Goal: (4.20, 1.40) yaw: 135.0°
✓ Step 4/4 COMPLETED
════════════════════════════════════════
✓ ALL 4 STEPS COMPLETED SUCCESSFULLY
════════════════════════════════════════
```

---

## 🔄 Service Comparison

| Feature | `/drive_command` (Original) | `/sequential_drive_command` (New) |
|---------|----------------------------|-----------------------------------|
| **Commands per call** | 1 | Unlimited |
| **Execution** | Immediate return | Waits for each completion |
| **Progress tracking** | ❌ No | ✅ Yes (`completed_steps`) |
| **Failure handling** | Simple success/fail | Reports exact failure point |
| **Use case** | Quick manual moves | Complex automation |
| **AI agent ready** | ⚠️ Limited | ✅ Perfect |
| **Multiple goals** | Requires multiple calls | Single call for sequence |

---

## 🤖 Python API (AI Agent Integration)

### Original Service

```python
from custom_drive_pkg.srv import DriveCommand

# Single command
request = DriveCommand.Request()
request.forward = 2.0
request.rotate = -90.0

response = client.call(request)
```

### Sequential Service

```python
from custom_drive_pkg.srv import DriveCommand
from custom_drive_pkg.msg import DriveStep

# Create request with multiple commands
request = DriveCommand.Request()

commands = [
    (3.0, 0.0),      # forward 3m
    (0.0, -90.0),    # turn right 90°
    (1.0, 0.0),      # forward 1m
    (0.0, 180.0),    # turn left 180°
]

for forward, rotate in commands:
    step = DriveStep()
    step.forward = float(forward)
    step.rotate = float(rotate)
    request.commands.append(step)

# Call service
response = client.call(request)

# Handle response
if response.success:
    print(f"✓ All {len(commands)} commands completed")
else:
    print(f"✗ Failed at step {response.completed_steps + 1}")
    print(f"Message: {response.message}")
```

### Complete Python Client Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_drive_pkg.srv import DriveCommand
from custom_drive_pkg.msg import DriveStep

class DriveClient(Node):
    def __init__(self):
        super().__init__('drive_client')
        self.client = self.create_client(
            DriveCommand, 
            'sequential_drive_command'
        )
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_sequence(self, commands):
        request = DriveCommand.Request()
        
        for forward, rotate in commands:
            step = DriveStep()
            step.forward = float(forward)
            step.rotate = float(rotate)
            request.commands.append(step)
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = DriveClient()
    
    # Example: Navigate in a pattern
    commands = [
        (3.0, 0.0),      # Forward 3m
        (0.0, -90.0),    # Right 90°
        (1.0, 0.0),      # Forward 1m
        (0.0, 180.0),    # Left 180°
    ]
    
    response = client.send_sequence(commands)
    
    if response.success:
        print(f"✓ Success: {response.message}")
    else:
        print(f"✗ Failed: {response.message}")
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## ⚠️ Prerequisites

Both services require:
- ✅ Nav2 navigation stack running
- ✅ SLAM or localization active (for TF transforms)
- ✅ Map available (if using pre-built map)

**Check if ready:**
```bash
# Verify Nav2 is running
ros2 node list | grep -E "controller_server|planner_server"

# Check TF transforms
ros2 run tf2_ros tf2_echo map base_link

# Verify action server (for sequential service)
ros2 action list | grep navigate_to_pose
```

---

## 🎯 Which Service Should I Use?

### Use **Original Drive Service** (`/drive_command`) when:
- ✅ You need quick manual control
- ✅ Single movements at a time
- ✅ Testing individual navigation commands
- ✅ Immediate feedback required (doesn't wait)

### Use **Sequential Drive Service** (`/sequential_drive_command`) when:
- ✅ Automating complex movement patterns
- ✅ AI agent is controlling the rover
- ✅ Need guaranteed execution order
- ✅ Want progress tracking and failure reporting
- ✅ Building autonomous navigation behaviors

---

## 🚀 Quick Start Guide

### Test Original Service
```bash
# Terminal 1: Start service
ros2 run custom_drive_pkg drive_service

# Terminal 2: Send commands
ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand \
  "{forward: 2.0, rotate: 0.0}"
```

### Test Sequential Service
```bash
# Terminal 1: Start service
ros2 run custom_drive_pkg sequential_drive_service

# Terminal 2: Send sequence
ros2 service call /sequential_drive_command custom_drive_pkg/srv/DriveCommand \
  "{commands: [
    {forward: 2.0, rotate: 0.0},
    {forward: 0.0, rotate: -90.0},
    {forward: 1.0, rotate: 0.0}
  ]}"
```

---

## 💡 Tips & Best Practices

### For Original Service:
- Use for teleoperation and manual testing
- Good for quick position adjustments
- Combine with visualization tools like RViz

### For Sequential Service:
- Plan your entire path before calling
- Monitor the completion logs for debugging
- Use appropriate timeout values for long sequences
- Handle failures gracefully in your AI agent code

### General:
- Always ensure SLAM/localization is running first
- Check TF transforms before sending commands
- Use RViz to visualize goal poses
- Start with small movements to verify setup

---

## 🐛 Troubleshooting

**Service not responding:**
```bash
# Check if service is running
ros2 service list | grep drive_command

# Check node status
ros2 node list | grep drive
```

**"Failed to get pose" error:**
```bash
# Verify TF chain
ros2 run tf2_ros tf2_echo map base_link

# Check SLAM is running
ros2 node list | grep slam
```

**Sequential service timeout:**
```bash
# Increase timeout parameter
ros2 run custom_drive_pkg sequential_drive_service --ros-args \
  -p goal_timeout:=600.0
```

**Goals not executing:**
```bash
# Verify Nav2 action server
ros2 action list | grep navigate_to_pose

# Check controller server
ros2 node list | grep controller_server
```

---

## 📚 Additional Resources

- **Nav2 Documentation**: https://navigation.ros.org
- **Leo Rover Docs**: https://www.leorover.tech/documentation
- **ROS2 Services**: https://docs.ros.org/en/humble/Tutorials/Services.html
- **ROS2 Actions**: https://docs.ros.org/en/humble/Tutorials/Actions.html

---

**Package Version**: 1.0.0  
**ROS2 Distribution**: Humble  
**License**: MIT