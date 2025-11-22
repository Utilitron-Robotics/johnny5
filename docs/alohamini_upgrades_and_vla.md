# AlohaMini Upgrades & VLA/LeRobot Integration Strategy

This document outlines suggested upgrades for the AlohaMini platform (Tier 2) and details how to leverage it for VLA (Vision-Language-Action) research using the LeRobot ecosystem.

## 1. Codebase Review & Upgrades

### Current State Analysis
*   **`lekiwi.py`**: Handles the core robot logic. Uses `FeetechMotorsBus` for communication.
    *   *Limitation*: Hardcoded kinematic matrix for 3-wheel omni drive.
    *   *Limitation*: Basic P-controller for Lift Axis.
*   **`lekiwi_client.py`**: Remote teleoperation client.
    *   *Limitation*: Keyboard-only teleop is clunky for complex manipulation.
*   **`lift_axis.py`**: Custom driver for the Z-axis.
    *   *Strength*: Good abstraction for the linear rail mechanism.

### Suggested Upgrades

#### A. Motion Smoothing (S-Curve Profiles)
The current velocity commands are sent raw to the wheels. This causes jerky movement, especially for the Z-axis lift which carries the arms.
*   **Proposal**: Implement an S-Curve acceleration ramp or a simple low-pass filter on velocity commands in `lekiwi.py`.

#### B. Battery Monitoring & Safety
The code reads currents (`Present_Current`) but lacks a robust battery voltage monitor. Li-ion batteries can be permanently damaged if drained too low.
*   **Proposal**: Add a `check_battery_voltage()` loop in `LeKiwi.read_and_check_currents()`. If voltage drops below 10.5V (for 3S), trigger an audio alarm or auto-shutdown.

#### C. Dataset Recorder ("The Data Harvester")
To train VLA models, we need data. Lots of it.
*   **Proposal**: Enhance `lekiwi_client.py` to log synchronized `(observation, action)` tuples to a LeRobot-compatible dataset format (Hugging Face Dataset) automatically during teleoperation.

## 2. LeRobot Ecosystem Integration

### Data Collection Pipeline
1.  **Teleoperation**: Use the Leader Arms (from LeKiwi kit) to control the Follower Arms on AlohaMini.
2.  **Recording**:
    *   Capture RGB streams from all 5 cameras (Top, Front, Back, Left Wrist, Right Wrist).
    *   Capture Joint Positions (14 DOF: 6+6 arms, 1 lift, 3 base).
    *   Frequency: 30Hz or 50Hz.
3.  **Format**: Save as `.parquet` or `.hdf5` files compatible with `lerobot.common.datasets`.

### Autonomous Policy Training (ACT / Diffusion)
Once data is collected:
1.  **Upload**: Push dataset to Hugging Face Hub.
2.  **Train**: Use `lerobot` training scripts to fine-tune an ACT (Action Chunking Transformer) policy.
    *   *Input*: Camera images + Current State.
    *   *Output*: Future Action Chunk (next k steps).
3.  **Deploy**: Load the trained policy onto the Raspberry Pi 5 (AlohaMini) or Jetson Orin (Johnny 5).

## 3. Vision-Language-Action (VLA) Strategy

VLA models (like RT-2 or OpenVLA) take text instructions ("Pick up the red apple") and visual input to generate robot actions.

### Integration Plan
1.  **VLM as High-Level Planner**:
    *   Use a Vision-Language Model (e.g., GPT-4o, Gemini, or local Llava) to analyze the scene.
    *   *Prompt*: "What steps are needed to clean this table?"
    *   *Output*: "1. Move to the soda can. 2. Grasp the can. 3. Move to the trash bin."
2.  **LeRobot as Low-Level Controller**:
    *   The VLM selects a specific primitive policy (e.g., `pick_object(can)`).
    *   The AlohaMini executes the `pick_object` policy trained via LeRobot/ACT.

### Roadmap
1.  **Phase 1**: Reliable Teleoperation & Data Collection (Human controls everything).
2.  **Phase 2**: Policy Cloning (Robot mimics human tasks).
3.  **Phase 3**: Language Conditioning (Robot selects task based on text command).