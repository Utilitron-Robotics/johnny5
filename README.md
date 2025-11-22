# Open-Source Mobile Manipulation: Choose Your Path

<div align="center">

![Johnny 5](docs/media/alohamini_git.png)

[![LeRobot](https://img.shields.io/badge/Hugging%20Face-LeRobot-orange?logo=huggingface)](https://github.com/huggingface/lerobot)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python)](https://www.python.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green?logo=ros)](https://docs.ros.org/en/humble/)
[![Jetson](https://img.shields.io/badge/NVIDIA-Jetson-76B900?logo=nvidia)](https://developer.nvidia.com/embedded/jetson-modules)
[![OAK-D](https://img.shields.io/badge/Luxonis-OAK--D-blue)](https://docs.luxonis.com/)
[![Gun.js](https://img.shields.io/badge/Database-Gun.js-red)](https://gun.eco/)

</div>

This repository hosts two distinct robot platforms derived from the LeKiwi/SO-ARM100 lineage. Whether you are an educator looking for an affordable teleoperation platform or a researcher building autonomous agents, there is a path for you.  Upgrade your existing LeKiwi to a more useful platform or rebuild it entirely to get a production grade, entry level robotics platform capable of real work.

## 🤖 Path A: AlohaMini (The Entry Level - Easiest LeKiwi Upgrade Path)
**"The Affordable Teleoperator"**

*   **Best For**: Education, Hobbyists, Teleoperation collection.
*   **Brain**: Raspberry Pi 5.
*   **Base**: 3-Wheel Omni-directional (3D printed).
*   **Key Feature**: Low barrier to entry, highly documented assembly.
*   **Documentation**: [Assembly Guide](docs/hardware_assembly.md) | [BOM](docs/BOM.md) | [LeKiwi Upgrade Guide](docs/lekiwi_upgrade_guide.md)

## 🤖 Path B: Johnny 5 (The Autonomous Pro - Real Robot for Real Work)
**"Input! More Input!"**

*   **Best For**: AI Research, VSLAM, Outdoor Utility.
*   **Brain**: NVIDIA Jetson Orin Nano (8GB+).
*   **Base**: Differential Drive with Rubber Tires (Off-the-shelf).
*   **Key Feature**: Rugged navigation, "Hanging Shoulder" arms (270° ROM), Towing capability.
*   **Documentation**: [Specification](docs/johnny5_spec.md) | [Migration Notes](docs/johnny5_migration_notes.md)

---

## 📂 Repository Structure

### Hardware
*   `hardware/arms/`: Shared SO-ARM100 follower arm files (used by both).
*   `hardware/alohamini_base/`: 3D printed parts for the AlohaMini Omni-base.
*   `hardware/johnny5_base/`: Mounting plates and adapters for the Johnny 5 Differential Drive base.

### Software
*   `software/src/lerobot/robots/alohamini/`: Python drivers for AlohaMini (RPi5/Omni).
*   `software/src/johnny5/`: Python drivers for Johnny 5 (Jetson/Diff-Drive).

## 🚀 Getting Started
## 🐢 ROS 2 Integration
The AlohaMini now supports ROS 2 Humble via the `alohamini_ros2` wrapper.

**Features:**
*   **Base Control**: Subscribe to `/cmd_vel` (Twist) for omni-directional movement.
*   **Joint Feedback**: Publish `/joint_states` for all arms and wheels.
*   **Visualization**: Compatible with RViz for real-time monitoring.

**Usage:**
```bash
# 1. Source your ROS 2 installation
source /opt/ros/humble/setup.bash

# 2. Build the package
colcon build --packages-select alohamini_ros2
source install/setup.bash

# 3. Launch the node
ros2 launch alohamini_ros2 aloha.launch.py
```
See [Upgrade Paths](docs/upgrade_paths.md) for a detailed comparison and decision guide.

## 🗺️ Roadmap
*   **Split Chassis Design**: We are developing a 2-piece interlocking chassis to support smaller 3D printers (currently requires >325mm bed).
*   **Johnny 5 Differential Drive**: Finalizing the off-the-shelf base integration.
*   **LeRobot Policy Training**: Releasing pre-trained weights for common household tasks.

## Acknowledgements
Originally based on [LeKiwi](https://github.com/TheRobotStudio/SO-ARM100) and the work of the LeRobot community.
