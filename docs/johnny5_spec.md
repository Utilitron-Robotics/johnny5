# Johnny 5: Autonomous Mobile Manipulator Specification

## 1. Design Philosophy
*   **Identity**: "Johnny 5" - A personality-driven, autonomous-first mobile manipulator.
*   **Core Divergence**: Unlike the LeKiwi/AlohaMini clones which focus on low-cost teleoperation via omni-wheels and RPi5, Johnny 5 prioritizes robust navigation, autonomy, and practical utility (towing/hauling).
*   **Ecosystem**: Built on the **LeRobot** foundation for data/training compatibility but swapping the physical hardware backend entirely.

## 2. Hardware Architecture

### 2.1 Compute & Sensors
*   **Main Brain**: **NVIDIA Jetson Orin Nano (8GB)**.
    *   *Reasoning*: Required for on-edge inference, VSLAM, and depth processing which RPi5 struggles with.
*   **Vision (The "Head")**:
    *   **Primary**: **Luxonis OAK-D** or **Intel RealSense** (Depth + RGB).
    *   *Placement*: Mounted on the "Face" (Tablet) assembly.
*   **HMI (Human-Machine Interface)**:
    *   **Tablet Position**: Front-facing. "Robots need faces."
    *   *Function*: Display eyes/emotions, debug info, and interactive UI.

### 2.2 Manipulators (Arms)
*   **Kinematics**: **"Hanging Shoulder" Configuration**.
    *   *Description*: Arms mount from a top overhang rather than protruding forward from a chest.
    *   *Range of Motion*: Target **270° rotation** capability.
*   **Hardware**: Modified SO-ARM100 core (servos/links) but with completely redesigned mounting points and root kinematics.

### 2.3 Mobility (The Base)
*   **Drive Train**: **Differential Drive** (Rubber Tires).
    *   *Requirement*: Off-the-shelf reliable base. NO Holonomic/Omni wheels (lack of traction, complexity).
*   **Utility**:
    *   **Tow Hitch**: Integrated clip for carts/trash cans.
    *   **Capabilities**: Towing, hauling, outdoor/threshold traversal (superior to omni).

### 2.4 Vertical Lift (Z-Axis)
*   **Design**: **Enclosed Gear Housing**.
    *   *Critique of Fork*: Plastic rack-and-pinion suspending 4kg+ on a single gear step is a failure point.
    *   *New Design*: Gearbox enclosed in housing.
    *   *Track*: Dual track design (one offset half-step from the other) for stability and load distribution.
    *   *Future Path*: Telescoping lift (Phase 2).

## 3. Software Stack Changes

### 3.1 Architecture Shift
*   **OS**: Ubuntu 20.04/22.04 (JetPack).
*   **Middleware**: ROS 2 (Humble) highly recommended for Nav2 (navigation stack) integration with the Diff Drive base + RealSense/Oak-D.
*   **LeRobot Integration**:
    *   Custom `Johnny5Policy` adapter.
    *   Kinematics solver for "Hanging Shoulder" configuration.
    *   Translation layer: `Cmd_Vel` (ROS2) <-> `LeRobot` Action Space.

### 3.2 Fork Strategy ("Going Off-Grid")
*   **Purge**: Remove `LeKiwi` / `AlohaMini` omni-wheel kinematics.
*   **Purge**: Remove Raspberry Pi specific GPIO/Camera hacks.
*   **Keep**: `FeetechMotorsBus` (Low-level servo driver is solid).
*   **Refactor**: `LiftAxis` to support the new enclosed dual-track mechanism.

## 4. Action Plan
1.  **Hardware Selection**: Confirm specific "Off-the-shelf" base model.
2.  **CAD**: Design "Hanging Shoulder" mounts and "Face" assembly.
3.  **Software**: Spin up Jetson environment and test `lerobot` compatibility with ARM64 JetPack.