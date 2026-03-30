# Development Environment Setup

PX4 SITL with Gazebo for autoresearch-drone development. This gives you a MAVLink-over-UDP environment nearly identical to the competition sim.

## Prerequisites

- Windows 11 with WSL2 (Ubuntu 24.04)
- ~20GB disk space
- A GPU helps for Gazebo rendering but isn't required for headless runs

## 1. WSL2 Setup

All PX4/Gazebo commands run inside WSL2. Open a WSL terminal:

```powershell
wsl -d Ubuntu-24.04
```

## 2. Create Python Virtual Environment

Ubuntu 24.04 uses PEP 668 (externally managed Python), so a venv is required:

```bash
python3 -m venv ~/drone-venv
source ~/drone-venv/bin/activate
```

You must `source ~/drone-venv/bin/activate` each time you open a new WSL terminal.

## 3. Install PX4 Toolchain

PX4 must live on the Linux filesystem (not /mnt/c/) for build performance:

```bash
cd /home/brian
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Run the setup script (installs dependencies, Gazebo, etc.)
bash ./Tools/setup/ubuntu.sh
```

## 4. Install Python Build Dependencies

The venv needs PX4's build-time Python packages:

```bash
source ~/drone-venv/bin/activate
pip install empy==3.3.4 pyyaml jsonschema kconfiglib jinja2 pyros-genmsg
```

**Important**: `empy` must be version 3.x (not 4.x) — PX4 uses `import em` which changed in v4.

## 5. Build PX4 SITL

```bash
cd ~/PX4-Autopilot
source ~/drone-venv/bin/activate
make px4_sitl gz_x500
```

The first build takes ~10 min. Once it launches, you should see Gazebo open with a quadrotor. Kill it with Ctrl+C.

## 6. Install MAVSDK and Project Dependencies

```bash
source ~/drone-venv/bin/activate
pip install mavsdk aioconsole numpy scipy
```

## 7. Verify the Stack

Terminal 1 — Launch PX4 SITL:
```bash
cd ~/PX4-Autopilot
source ~/drone-venv/bin/activate
make px4_sitl gz_x500
```

Terminal 2 — Test MAVSDK connection (save as a .py file and run):
```python
import asyncio
from mavsdk import System

async def test():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    async for position in drone.telemetry.position():
        print(f"Alt: {position.relative_altitude_m:.1f}m")
        break

asyncio.run(test())
```

If this prints "Connected!" and an altitude, you're good to go.

## 8. Set Up the Gate Course

Copy the gate world into the Gazebo worlds directory:

```bash
# Option A: Use our custom world with PX4's launch
# Set the world file path when launching PX4 SITL
PX4_GZ_WORLD=racecourse make px4_sitl gz_x500
```

Alternatively, you can place gates into the default empty world by spawning SDF models. See `world/` directory for gate models and the course layout.

## 9. Running Headless (for overnight autoresearch)

For the autoresearch loop, you don't need the Gazebo GUI:

```bash
HEADLESS=1 make px4_sitl gz_x500
```

This runs the full physics sim without rendering. Much lighter on CPU/GPU.

## Key Paths

| What | Path |
|------|------|
| PX4-Autopilot | `/home/brian/PX4-Autopilot` (WSL2 Linux filesystem) |
| Python venv | `~/drone-venv` |
| Project code | `C:\Users\brian\OneDrive\Desktop\AI_Grand_Prix` (Windows) |
| Project code from WSL | `/mnt/c/Users/brian/OneDrive/Desktop/AI_Grand_Prix` |

## Architecture

```
┌─────────────────────────────────────────────────────┐
│  Your Code (pilot.py via prepare.py)                │
│  MAVSDK Python ←→ UDP :14540                        │
└──────────────────────┬──────────────────────────────┘
                       │ MAVLink v2
┌──────────────────────┴──────────────────────────────┐
│  PX4 SITL                                           │
│  - EKF2 state estimator → ODOMETRY                  │
│  - Attitude controller → motor mixing               │
│  - Accepts SET_ATTITUDE_TARGET                       │
│  - Accepts SET_POSITION_TARGET_LOCAL_NED             │
│  - Publishes ATTITUDE, HIGHRES_IMU, ODOMETRY        │
└──────────────────────┬──────────────────────────────┘
                       │ Gazebo transport
┌──────────────────────┴──────────────────────────────┐
│  Gazebo                                             │
│  - Rigid body physics (1000 Hz)                     │
│  - Drone model (x500 quadrotor)                     │
│  - Gate models (collision detection)                │
│  - Camera sensor (for Phase B+)                     │
└─────────────────────────────────────────────────────┘
```

This is the same stack the competition almost certainly uses. Code targeting this environment will transfer to the DCL sim with changes only in prepare.py (connection details, gate detection mechanism).

## Swapping to the Competition Sim

When the DCL sim drops in May:
1. Replace the PX4 SITL launch commands in `prepare.py` with the DCL sim launcher.
2. Update gate detection if the competition sim reports gate passage directly (likely).
3. Update connection address/port if different from :14540.
4. Everything else — `pilot.py`, `score.py`, `notebook.md`, `program.md`, references — carries over unchanged.
