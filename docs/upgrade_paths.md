# Choose Your Character: Upgrade Paths

This repository supports two distinct robot platforms. Both start from the **LeKiwi** (SO-ARM100) lineage but diverge based on cost, capability, and complexity.

## Path A: AlohaMini (The "LeKiwi Pro")
*   **Target Audience**: Educators, Hobbyists, Teleoperation Research.
*   **Core Philosophy**: Low-cost upgrade. Keep the Raspberry Pi 5, keep the Omni-wheels, just add a Z-axis and better frame.
*   **Compute**: Raspberry Pi 5.
*   **Base**: 3-Wheel Holonomic (Omni-wheels).
*   **Kinematics**: Standard forward-facing arms.
*   **Cost**: $ (Low).

## Path B: Johnny 5 (The "Autonomous Beast")
*   **Target Audience**: AI Researchers, Field Robotics, Autonomy Developers.
*   **Core Philosophy**: Capability first. Replace the brain and legs for real-world utility.
*   **Compute**: NVIDIA Jetson Orin Nano (8GB+).
*   **Base**: Differential Drive (Rubber Tires) for outdoor traction.
*   **Sensors**: OAK-D / RealSense Depth Camera (VSLAM).
*   **Kinematics**: "Hanging Shoulder" arms (270° ROM) + Enclosed Lift.
*   **Cost**: $$$ (Medium/High).

## Comparison Table

| Feature | AlohaMini | Johnny 5 |
| :--- | :--- | :--- |
| **OS** | Raspberry Pi OS | Ubuntu 20.04 (JetPack) |
| **Navigation** | Dead Reckoning (Basic) | VSLAM / ROS 2 Nav2 |
| **Traction** | Indoor Only (Slippery) | Indoor/Outdoor (Rugged) |
| **Payload** | Light Manipulation | Towing + Heavy Manipulation |
| **Complexity** | Medium (3D Print + Assemble) | High (Custom Electronics + ROS) |

## Repository Structure
*   `software/src/lerobot/robots/alohamini`: Code for Path A.
*   `software/src/johnny5`: Code for Path B.