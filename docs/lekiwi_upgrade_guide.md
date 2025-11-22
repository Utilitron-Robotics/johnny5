# LeKiwi to AlohaMini Upgrade Guide

**From Stationary Single-Arm to Mobile Manipulation.**

This guide covers upgrading a **LeKiwi** (Single SO-ARM100 on a desk stand) to the **AlohaMini** platform.

## 🛤️ Choose Your Path

### Path 1: The "Mobile Kiwi" (Single Arm)
*   **Goal**: Get your LeKiwi moving.
*   **Cost**: Low (Only need Base parts + Electronics).
*   **Result**: A 3-wheeled mobile robot with one arm and a vertical lift.
*   **Pros**: Cheaper, uses existing hardware.
*   **Cons**: Reduced manipulation capability (bimanual tasks impossible).

### Path 2: Full AlohaMini (Dual Arm)
*   **Goal**: Full bimanual teleoperation.
*   **Cost**: Medium (Base parts + Cost of a 2nd SO-ARM100).
*   **Result**: The standard AlohaMini configuration.
*   **Pros**: Capable of complex human-like tasks.
*   **Cons**: More assembly, higher cost.

---

## 1. Fabrication (The Chassis)

**⚠️ PRINTER WARNING**: The main chassis plates require a large print bed (**>325mm x 325mm**). If you have a standard Ender 3 or Prusa MK3/4, you will need to wait for the "Split Chassis" update or find a printing service.

*   **Print Files**: `hardware/alohamini_base/stl/`
*   **Material**: PETG (Recommended) or PLA+.
*   **Infill**: 25% Gyroid for strength.

## 2. Assembly Steps

### Step A: The Mobile Base (Common)
1.  **Servos**: Install the 3x Feetech STS3215 servos into the `OB_Chassis_Servo_Mounts`.
2.  **Wheels**: Mount the Omni Wheels to the servos using the axle connectors.
3.  **Frame**: Bolt the mounts between the `Lower` and `Upper` chassis plates using M3x12 screws.

### Step B: The Z-Axis Tower (Common)
1.  Assemble the 4 main posts (`O_Main_Assembly_Post`) to create the central spine.
2.  Install linear bearings/rails into the spine channels.
3.  Mount the Lift Servo (ID 11) and Rack Gear at the base.
4.  Install the **T-Connector Cross Bar** onto the rails. This is the critical mount point for the arms.

### Step C: Arm Migration

#### For "Mobile Kiwi" (Single Arm)
1.  **Unmount**: Remove your SO-ARM100 from its desk stand. Keep the arm assembled.
2.  **Mount**: Bolt the arm to the **Right Side** of the T-Connector Cross Bar using 4x M3x30 screws.
3.  **Counterweight (Optional)**: The base is wide enough to be stable with one arm, but adding a dummy weight (or even your battery pack) to the Left side of the Cross Bar will improve movement smoothness.

#### For Full AlohaMini (Dual Arm)
1.  **Unmount**: Remove existing arm from desk stand.
2.  **Build/Prep 2nd Arm**: Assemble your second SO-ARM100 (Refer to SO-ARM100 docs).
3.  **Mount**: Bolt one arm to the Left and one to the Right of the T-Connector Cross Bar.

### Step D: Electronics
1.  **Brain**: Mount the Raspberry Pi 5 to the back of the Tower.
2.  **Power**:
    *   Battery 1 (Motors): Connect to the Waveshare Servo Driver.
    *   Battery 2 (Compute): Connect to the DC-DC converter -> Pi 5 USB-C.
3.  **Bus Cabling**:
    *   Daisy chain the Arm servos.
    *   Connect the Base servos (IDs 8, 9, 10) and Lift servo (ID 11) to the same bus.

## 3. Software Configuration

### Config Adjustment
If running **Path 1 (Single Arm)**, you must modify `config_lekiwi.py`.

By default, the code expects two arms (`left_port` and `right_port`). For a single arm setup:

1.  Open `software/src/lerobot/robots/alohamini/config_lekiwi.py`.
2.  Locate the `LeKiwiConfig` class.
3.  Change the unused port to `None`:
    ```python
    @dataclass
    class LeKiwiConfig(RobotConfig):
        # For Single Arm (Right side only)
        left_port: str | None = None 
        right_port: str = "/dev/am_arm_follower_right"
    ```
4.  The `AlohaMiniDivergent` driver will automatically detect the `None` value and skip initialization for that arm.

### Calibration
Run the calibration script to zero the Z-axis and new wheels:
```bash
python3 -m lerobot.robots.alohamini.lekiwi_calibrate