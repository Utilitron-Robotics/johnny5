# LeKiwi to AlohaMini Upgrade Guide

**Transform your static SO-ARM100 robot into a mobile manipulator with a vertical lift.**

This guide covers the physical transformation of a LeKiwi station (or standalone SO-ARM100 arms) into an AlohaMini mobile robot.

## 📋 Prerequisites
*   **Existing Hardware**: 1 or 2 SO-ARM100 Arms (LeKiwi setup).
*   **Fabrication**: Access to a 3D Printer (PETG recommended).
*   **Parts**: See the **[Detailed Upgrade BOM](lekiwi_to_alohamini_bom.md)** for the full shopping list.

## 1. Base Fabrication (Printing & Repurposing)
The AlohaMini uses a custom 3D-printed chassis. Unlike the LeKiwi which clamps to a table, this robot carries its own weight.

*   **Print List**: See `hardware/alohamini_base/stl/`
*   **Critical Part**: `O_T_Connector_Cross_Bar.stl`. This part replaces your desk clamps. It is the central "spine" that holds both arms and connects them to the Z-axis lift.

## 2. Assembly Steps

### Step A: Construct the Mobile Base
1.  Install the 3x Feetech STS3215 servos into the `OB_Chassis_Servo_Mounts`.
2.  Attach the Omni Wheels to the servos using the `OB_Chassis_Wheel_Axle_Connector`.
3.  Sandwich the mounts between the `Lower` and `Upper` chassis plates using M3x12 screws.

### Step B: Build the Z-Axis Tower
1.  Assemble the 4 main posts (`O_Main_Assembly_Post 1-4`) to form the central tower.
2.  Install the linear rails/bearings into the tower channels.
3.  Mount the Lift Servo (ID 11) at the base of the tower using `OB_Z_Axis_Servo_Mount`.
4.  Attach the Rack Gear (`OB_Z_Axis_Servo_Gear`) to the servo horn.

### Step C: Migrate the Arms
**WARNING**: Ensure arms are powered off and disconnected.

1.  **Unmount**: Remove your SO-ARM100 arms from their current desk/table mounts. You will keep the arms intact but discard the table clamp base.
2.  **Remount**: Bolt the base of each arm to the `O_T_Connector_Cross_Bar` using 4x M3x30 screws.
    *   *Single Arm Setup*: If you only have one arm, mount it to the Right side (standard leader). The base is stable enough to support an unbalanced load, though adding a counterweight to the Left side is recommended for smoother motion.
3.  **Install**: Slide the `O_T_Connector_Cross_Bar` (now holding the arms) onto the Z-Axis tower rails.

### Step D: Electronics & Wiring
1.  Mount the Raspberry Pi 5 and Servo Driver to the back of the tower.
2.  **Cable Management**: Route the arm bus cables down the tower to the driver. Ensure enough slack for the Z-axis to move up and down (approx 30cm travel).
3.  **Power**: Connect the 12V battery to the DC-DC converter (for Pi) and directly to the Servo Driver (for Motors).

## 3. Software Setup
Once assembled, flash the **AlohaMini** software image to the Raspberry Pi.
*   **Config**: Update `config_alohamini.py` to reflect if you are using 1 or 2 arms.
*   **Calibration**: Run the Z-axis homing script immediately to prevent the lift from crashing into the chassis.

## 4. Next Steps
*   [Software Setup Guide](software_setup.md)
*   [Teleoperation Tutorial](../README.md)