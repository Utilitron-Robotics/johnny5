# Project Review: AlohaMini

## 1. Executive Summary
This project is a sophisticated open-source hardware and software fork, derived from the **LeKiwi** platform, effectively transforming it into **AlohaMini**. The primary differentiator is the addition of a **motorized vertical lift axis (Z-axis)** to the mobile manipulator. The codebase is currently in a transitional state: it resides in an `alohamini` directory but retains `LeKiwi` class names and terminology throughout the Python source files.

## 2. Architecture & Codebase Analysis
*   **Structure**: The project follows the `lerobot` directory structure (`lerobot/robots/alohamini`), ensuring compatibility with the Hugging Face LeRobot ecosystem.
*   **Hardware Interface**:
    *   **Base**: Omni-directional 3-wheel base driven by Feetech STS3215 servos.
    *   **Arms**: Dual SO-ARM100 follower arms (6 DOF each).
    *   **Lift**: A custom `LiftAxis` class managing a single servo (ID 11) for vertical travel.
*   **Software Stack**:
    *   **`LeKiwiHost`**: Runs on the robot (Raspberry Pi), managing hardware communication via `LeKiwi` class and exposing a ZMQ server.
    *   **`LeKiwiClient`**: Runs on a remote workstation, sending teleoperation commands (VR or keyboard) and receiving observations.
    *   **`LiftAxis`**: A well-encapsulated class handling the Z-axis logic (homing, velocity control, position targeting), integrated directly into the main robot class.

## 3. Documentation & BOM
*   **Strengths**:
    *   **Visuals**: Excellent assembly documentation with clear photos (`docs/hardware_assembly.md`).
    *   **BOM**: Detailed and priced, with links for US/CN sourcing.
    *   **Transparency**: Clearly states the relationship with LeKiwi and SO-ARM100.
*   **Weaknesses**:
    *   **Naming Consistency**: The documentation refers to "AlohaMini", but the code refers almost exclusively to "LeKiwi". This creates cognitive load for new developers trying to map the docs to the code.

## 4. The "LeKiwi Upgrade Path" Strategy
The project currently presents itself as a standalone clone, but the underlying structure suggests a clear "Upgrade Path" from LeKiwi:
1.  **Mechanical**: Addition of the T-Bracket, Linear Rails/Lead Screw mechanism, and the Lift Servo (ID 11).
2.  **Electrical**: Integration of the Lift Servo into the bus.
3.  **Software**: The `LiftAxis` class and its inclusion in the observation/action space.

To formalize this, the project requires a strict separation in the code between the base `LeKiwi` functionality and the `AlohaMini` extensions, likely controlled via configuration flags or subclassing, allowing a single codebase to drive both robot variants.