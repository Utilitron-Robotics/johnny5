# AlohaMini ROS 2 Package (`alohamini_ros2`)

This package provides a unified ROS 2 interface for both the **AlohaMini** (Omni-directional) and **Johnny 5** (Differential Drive) robots. It acts as a bridge between the LeRobot-compatible Python drivers and the ROS 2 ecosystem (Nav2, MoveIt, RViz).

## 📦 Installation

This package is designed to be built in a ROS 2 workspace (Humble recommended).

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Go to your workspace src folder (assuming this repo is checked out there)
# cd ~/ros2_ws/src

# 3. Build
colcon build --packages-select alohamini_ros2
source install/setup.bash
```

## 🚀 Usage

### Launching the Driver
You can launch the driver for either robot platform using the `robot_type` argument.

**For AlohaMini (Default):**
```bash
ros2 launch alohamini_ros2 aloha.launch.py robot_type:=aloha
```

**For Johnny 5:**
```bash
ros2 launch alohamini_ros2 aloha.launch.py robot_type:=johnny5
```

### Configuration Arguments
| Argument | Default | Description |
|----------|---------|-------------|
| `robot_type` | `aloha` | Robot platform: `aloha` or `johnny5`. |
| `left_port` | `/dev/am_arm_follower_left` | USB port for Left Arm/Base bus. |
| `right_port` | `/dev/am_arm_follower_right` | USB port for Right Arm bus. |

## 📡 Topics

### Subscribed (Input)
*   **`/cmd_vel`** (`geometry_msgs/Twist`)
    *   Controls the mobile base.
    *   **AlohaMini**: Uses `linear.x` (forward), `linear.y` (lateral), `angular.z` (rotation).
    *   **Johnny 5**: Uses `linear.x` (forward), `angular.z` (rotation). `linear.y` is ignored.
*   **`/joint_commands`** (`sensor_msgs/JointState`)
    *   Direct control for arm joints and lift.
    *   Expects standard names: `arm_left_shoulder_pan`, `lift_joint`, etc.

### Published (Output)
*   **`/joint_states`** (`sensor_msgs/JointState`)
    *   Real-time position feedback from all motors.
    *   Frequency: 30Hz.
    *   Units: Radians (arms), Meters (lift).

## 🏗️ Architecture
The node dynamically loads the appropriate underlying Python driver (`AlohaMiniDivergent` or `Johnny5Robot`) based on the launch configuration, ensuring a consistent ROS API regardless of the physical hardware.