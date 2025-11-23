# Project: Fleet Unification ("One Brain, Many Bodies")

**Status**: Draft (Revision 2 - Bare Metal Strategy)
**Date**: 2025-11-23
**Objective**: Unify the diverse fleet of 16+ robots under a single "WhoAmI" intelligence architecture using a lightweight, bare-metal deployment strategy (GitOps + Venv + Systemd).

---

## 1. The Fleet (The "Moon Alignment")

We have a unique opportunity with a massive, aligned fleet.

### **Group A: The Swarm (Light Mobile Manipulation)**
*Shared DNA: LeKiwi/SO-ARM100 lineage. High quantity, perfect for data collection and swarm behavior.*
*   **2x AlohaMini**: The reference standard for teleoperation.
*   **2x XLeRobot**: The "Greeter" specialized build.
*   **6x LeKiwi**: The precursors. **Strategy**: Upgrade these using the **20x SE101 Arms** inventory to match AlohaMini/XLeRobot capabilities.
    *   *Total Potential*: **10x Unified Mobile Manipulators**.

### **Group B: The Heavy Lifters (Advanced/Humanoid)**
*Shared DNA: High-end compute (Jetson Orin), complex kinematics.*
*   **4x K-1 Booster**: Commercial-grade humanoids. The "Face" of the fleet.
*   **1x OpenDroid R2D3**: Dual-arm lifting specialist with telescoping torso.

### **Group C: The Scout (Autonomous Research)**
*   **1x Johnny 5**: (Converted from AlohaMini). High-spec sensors (OAK-D), advanced compute. The pathfinder for VSLAM and autonomy.

---

## 2. Unification Strategy: "Bare Metal" (Lightweight)

To manage 16+ robots without the overhead of Docker, we will use a **Universal Installer** and **GitOps** workflow.

### A. The Universal Installer (`unified_setup.sh`)
A single script that auto-detects the hardware and configures the environment natively.

*   **Platform Detection**:
    *   *If Jetson (Tegra)*: Installs `torch` with CUDA support, JetPack libs, `pyttsx3` (or Piper for ARM).
    *   *If RPi5/Laptop (x86/ARM)*: Installs standard CPU `torch`, lightweight inference models.
*   **Isolation**: Uses standard Python `venv` (`~/whoami_env`) to keep dependencies clean without container overhead.
*   **Persistence**: Generates `systemd` services (`whoami-core.service`) for auto-start and restart-on-failure.

### B. Hardware Abstraction Layer (HAL)
Instead of hardcoding ports, each robot has a local identity file: `~/whoami_config.json`.

```json
{
  "robot_id": "alohamini_01",
  "robot_type": "alohamini",  // or "k1", "johnny5", "r2d3"
  "capabilities": ["vision", "voice", "mobile_base"],
  "hardware": {
    "camera_port": "/dev/video0",  // or "realsense_serial_number"
    "arm_left": "/dev/ttyUSB0",
    "arm_right": "/dev/ttyUSB1",
    "mic_device": "hw:2,0"
  }
}
```
*The code reads this at startup to know "Who Am I?" and loads the correct drivers.*

### C. The "Hive Mind" (Distributed Memory)
*   **Gun.js Mesh**: All 16 robots sync to a local Gun.js relay.
*   **Shared Experience**: Face encodings and interaction logs are synced via the mesh.
*   **Update Mechanism**: `git pull` + `systemctl restart whoami-core` via Ansible/SSH.

---

## 3. Implementation Roadmap

### Phase 1: Foundation (Tie the Rope)
*   [ ] **Repo Consolidation**: Finalize `alohamini`, `whoami`, `booster_k1`, `R2D3_ros2` updates.
*   [ ] **Create Universal Installer**: Write `unified_setup.sh` to handle dependency branching (Jetson vs. Standard).
*   [ ] **Define HAL Schema**: Finalize `robot_config.json` structure.

### Phase 2: The Reference Trio
*   [ ] **Select Prototypes**: Pick 1x AlohaMini, 1x K-1, 1x R2D3.
*   [ ] **Deploy**: Run the universal installer on all three.
*   [ ] **Verify Abstraction**: Ensure the same `main.py` runs on all three, adapting to their config files.

### Phase 3: Mass Production (The Swarm)
*   [ ] **Assembly Line**: Upgrade the 6 LeKiwis using spare SE101 arms.
*   [ ] **Fleet Management**: Use a simple Ansible playbook to trigger `git pull` across the subnet.
*   [ ] **Swarm Test**: Activate "Greeter" mode on all robots simultaneously.

---

## 4. Immediate Next Steps
1.  **Scripting**: Create the `unified_setup.sh` script in the `whoami` repo.
2.  **Config**: Create the `config/robot_identity_template.json`.
3.  **Network**: Ensure all robots are on the local subnet with static IPs (or mDNS hostnames like `k1-01.local`).