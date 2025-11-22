# Open-Source Mobile Manipulation: Choose Your Path

This repository hosts two distinct robot platforms derived from the LeKiwi/SO-ARM100 lineage. Whether you are an educator looking for an affordable teleoperation platform or a researcher building autonomous agents, there is a path for you.

## 🤖 Path A: AlohaMini (The Entry Level)
**"The Affordable Teleoperator"**

*   **Best For**: Education, Hobbyists, Teleoperation collection.
*   **Brain**: Raspberry Pi 5.
*   **Base**: 3-Wheel Omni-directional (3D printed).
*   **Key Feature**: Low barrier to entry, highly documented assembly.
*   **Documentation**: [Assembly Guide](docs/hardware_assembly.md) | [BOM](docs/BOM.md) | [LeKiwi Upgrade Guide](docs/lekiwi_upgrade_guide.md)

## 🤖 Path B: Johnny 5 (The Autonomous Pro)
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
See [Upgrade Paths](docs/upgrade_paths.md) for a detailed comparison and decision guide.

## Acknowledgements
Originally based on [LeKiwi](https://github.com/TheRobotStudio/SO-ARM100) and the work of the LeRobot community.
