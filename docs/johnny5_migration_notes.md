# Johnny 5 Migration Notes (Break The Fork)

## 1. Deletion List (Purge "AlohaMini/LeKiwi")
These files are specific to the 3-wheel omni base or the RPi5 architecture and should be removed or heavily archived.

*   `software/src/lerobot/robots/alohamini/` (The entire directory is target for refactoring/replacement)
    *   `lekiwi.py` (Contains Omni kinematics -> DELETE)
    *   `config_lekiwi.py` (Contains Omni/RPi config -> DELETE)
    *   `lekiwi_host.py` / `lekiwi_client.py` (Architecture specific -> DELETE)
*   `hardware/mobile_base/` (STL files for the omni base -> DELETE/ARCHIVE)
*   `docs/hardware_assembly.md` (Refers to Omni base -> DELETE/ARCHIVE)

## 2. Preservation List (The "Organs")
We need to harvest these parts before burning the rest.

*   **Motor Drivers**: `software/src/lerobot/motors/feetech/` (Assuming this exists in the parent `lerobot` repo, or if it was vendored here, we keep the bus communication logic).
*   **Lift Logic**: The `LiftAxis` class logic (PID, Homing) is valuable, even if the mechanical implementation changes to an enclosed gear. Save snippets.
*   **Arm Kinematics**: SO-ARM100 forward/inverse kinematics (if present in library) are still relevant, just the mounting transform changes.

## 3. New Structure (`software/src/johnny5`)

```
software/src/johnny5/
├── __init__.py
├── robot.py             # Main Johnny5 class (Differential Drive)
├── config.py            # Jetson Orin + RealSense Config
├── drivers/
│   ├── diff_drive.py    # Wrapper for off-the-shelf base
│   ├── lift_controller.py # New Enclosed Z-Axis controller
│   └── head_display.py  # Tablet face UI control
├── vision/
│   ├── depth_camera.py  # OAK-D / RealSense wrapper
│   └── vslam.py         # Placeholder for future autonomy
└── arms/
    └── hanging_arm.py   # Modified kinematics for top-mount shoulders
```

## 4. Migration Steps
1.  **Create Directory**: `mkdir -p software/src/johnny5`
2.  **Scaffold**: Create empty `__init__.py` and basic `robot.py`.
3.  **Harvest**: Copy `FeetechMotorsBus` usage patterns from `lekiwi.py` into `johnny5/robot.py` but strip the wheel logic.
4.  **Destruction**: Remove `software/src/lerobot/robots/alohamini`.
5.  **Rename**: Update root `README.md` to reflect "Johnny 5".