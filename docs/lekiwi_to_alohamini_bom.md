# BOM: LeKiwi to AlohaMini Upgrade

**Transform your single-arm LeKiwi into a mobile AlohaMini.**

Since the LeKiwi is a **single-arm** stationary robot, and the AlohaMini is a **dual-arm** mobile robot, you have two upgrade choices:
1.  **Single-Arm Mobile**: Move your existing arm to the new base.
2.  **Dual-Arm Mobile (Full Upgrade)**: Move your existing arm AND build a second arm.

---

## 1. Base Upgrade Kit (Required for All)
*This kit builds the mobile chassis and the Z-axis lift tower.*

### 3D Printed Parts (PETG Recommended)
*   **Files Location**: `hardware/alohamini_base/stl/`
*   **Printer Requirement**: Bed size > 325mm x 325mm for single-piece chassis.

| Part | Qty | Notes |
|------|-----|-------|
| `O_Chassis_Lower_Plate.stl` | 1 | |
| `O_Chassis_Upper_Plate.stl` | 1 | |
| `O_Main_Assembly_Post[1-4].stl` | 1 ea | |
| `O_T_Connector_Cross_Bar.stl` | 1 | **The Spine**: Holds arms and lift |
| `OB_Chassis_Servo_Mount.stl` | 3 | |
| `OB_Chassis_Wheel_Axle_Connector.stl`| 3 | |
| `OB_Z_Axis_Servo_Mount.stl` | 1 | |
| `OB_Z_Axis_Servo_Gear.stl` | 1 | |
| `OB_Z_Axis_Bracket.stl` | 1 | |

### Electronics & Motion
| Component | Spec | Qty | Notes |
|-----------|------|-----|-------|
| **Wheel Servos** | Feetech STS3215 | 3 | 12V Bus Servos |
| **Lift Servo** | Feetech STS3215 | 1 | 12V Bus Servo |
| **Omni Wheels** | 100mm (4") | 3 | |
| **Linear Bearings** | 4x13x5 mm | 8 | Z-Axis Slide |
| **Raspberry Pi 5** | 4GB/8GB | 1 | Upgrades the LeKiwi's RPi4 or PC |
| **Battery** | 11.1V (3S) Li-ion | 2 | Motor Power + Compute Power |

---

## 2. Second Arm Expansion (Optional)
*Required only if you want the full dual-arm AlohaMini experience. If you are sticking to one arm for now, skip this.*

| Component | Qty | Notes |
|-----------|-----|-------|
| **Feetech STS3215** | 6 | For the second arm joints (Follower) |
| **SO-ARM100 Hardware Kit** | 1 | Bearings, screws, horns for 1 arm |
| **3D Printed Arm Parts** | 1 set | Print all `F_*.stl` and `D_*.stl` files |
| **Camera** | 1 | Wrist camera for the new arm |

---

## 3. Fasteners Checklist (Base Only)
*   **M3 Heat-set Inserts**: ~50x
*   **M3x12 Screws**: ~40x
*   **M3x30 Screws**: 8x (Critical for mounting arms to the spine)
*   **M4x12 Screws**: 20x