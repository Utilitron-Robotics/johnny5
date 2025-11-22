# BOM: LeKiwi to AlohaMini Upgrade

This Bill of Materials is specifically for upgrading an existing **LeKiwi** (stationary robot with 1 or 2 SO-ARM100 arms) to the **AlohaMini** (mobile manipulator).

## 1. Base Fabrication (3D Printed)
You must print the following parts to create the mobile chassis and Z-axis tower.
*   **Material**: PLA, PETG, or ABS (Recommended: PETG for durability)
*   **Infill**: 20-30% Gyroid
*   **Printer Requirement**: Bed size > 325mm x 325mm (e.g., Voron 350, Prusa XL) for single-piece chassis.
*   **Files Location**: `hardware/alohamini_base/stl/`

| Part Name | Qty | Description |
|-----------|-----|-------------|
| `O_Chassis_Lower_Plate.stl` | 1 | Bottom chassis plate |
| `O_Chassis_Upper_Plate.stl` | 1 | Top chassis plate |
| `O_Main_Assembly_Post[1-4].stl` | 1 ea | Main tower structural posts |
| `O_T_Connector_Cross_Bar.stl` | 1 | **CRITICAL**: Mounts arms to the lift |
| `OB_Chassis_Servo_Mount.stl` | 3 | Mounts for wheel servos |
| `OB_Chassis_Wheel_Axle_Connector.stl` | 3 | Connects wheels to servos |
| `OB_Chassis_Wheel_Guard.stl` | 3 | Protective guards for wheels |
| `OB_Z_Axis_Servo_Mount.stl` | 1 | Mount for the lift servo |
| `OB_Z_Axis_Servo_Gear.stl` | 1 | Gear for the lift mechanism |
| `OB_Z_Axis_Bracket.stl` | 1 | Moving bracket for the lift |

## 2. Motion Hardware
| Component | Spec | Qty | Notes |
|-----------|------|-----|-------|
| **Wheel Servos** | Feetech STS3215 | 3 | Must be 12V Bus Servos |
| **Lift Servo** | Feetech STS3215 | 1 | Must be 12V Bus Servo |
| **Omni Wheels** | 100mm (4") | 3 | Plastic or Aluminum hub |
| **Linear Bearings** | 4x13x5 mm | 8 | For the Z-axis slide |
| **Axle Bearings** | 12x18x4 mm | 3 | Optional, for smoother wheel rotation |

## 3. Electronics Upgrade
| Component | Qty | Notes |
|-----------|-----|-------|
| **Raspberry Pi 5** | 1 | 4GB or 8GB recommended for LeRobot |
| **Servo Driver** | 1 | Waveshare Bus Servo Adapter (A) |
| **DC-DC Buck Converter** | 1 | 12V to 5V 5A (Powers the RPi) |
| **Battery** | 2 | 11.1V (3S) Li-ion Packs (XT60) |
| **Cameras** | 3 | USB 720p (Top, Front, Back) - *In addition to wrist cams* |

## 4. Arm Situation
*   **If you have 2 Arms**: Remove them from their desk stands. Mount them directly to the `O_T_Connector_Cross_Bar` using 4x M3x30 screws each.
*   **If you have 1 Arm**: Mount it to either the Left or Right side of the Cross Bar. You can print a "Dummy Weight" or counter-balance for the other side if stability is an issue, but the wide omni-base is generally stable enough for single-arm operation.

## 5. Fasteners Checklist
*   **M3 Heat-set Inserts**: ~50x (Standard 5mmOD x 4mmL)
*   **M3x12 Screws**: ~40x
*   **M3x18 Screws**: 12x
*   **M3x30 Screws**: 8x (For Arm Mounting)
*   **M3 Nuts**: 8x
*   **M4x12 Screws**: 20x
*   **M4 Heat-set Inserts**: 12x