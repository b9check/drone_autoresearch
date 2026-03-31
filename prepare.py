"""
prepare.py — Evaluation harness for autoresearch-drone.

INFRASTRUCTURE FILE — maintained by the human, not the agent.
The agent modifies pilot.py and score.py only.

This file:
1. Manages PX4 SITL lifecycle (launch, connect, teardown)
2. Defines the gate course (positions, normals, dimensions)
3. Connects to PX4 SITL via MAVSDK
4. Arms, takes off, enters offboard mode
5. Calls pilot.run(drone, gates) — the agent's code
6. Monitors gate passage and enforces timeout
7. Logs raw metrics, computes score, saves trajectory
8. Prints summary in a grep-friendly format
"""

import asyncio
import time
import sys
import os
import subprocess
import signal
import importlib
import traceback
import numpy as np
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
from mavsdk.action import ActionError

# ============================================================================
# CONSTANTS
# ============================================================================

MAX_RUN_DURATION = 480.0        # 8 minutes, matching competition spec
TAKEOFF_ALTITUDE = 2.0          # meters above ground
GATE_PASS_RADIUS = 1.5          # meters — how close to gate center counts as passage
TELEMETRY_RATE_HZ = 50          # how often we poll position for gate detection
CONNECTION_TIMEOUT = 45.0       # seconds to wait for MAVSDK connection

MAVSDK_ADDRESS = "udpin://0.0.0.0:14540"

PX4_AUTOPILOT_DIR = os.path.expanduser("~/PX4-Autopilot")
SIM_STARTUP_TIMEOUT = 60.0      # seconds to wait for PX4 SITL to become ready

# Trajectory logging — every run is saved for visualization
TRAJECTORY_DIR = "trajectories"

# ============================================================================
# GATE COURSE DEFINITION
#
# Each gate is a dict with:
#   position: [x, y, z] in NED (z is negative = above ground)
#   normal:   [nx, ny, nz] — direction to fly through the gate (unit vector)
#   width:    gate width in meters
#   height:   gate height in meters
#   label:    human-readable name
#
# This course is for the PX4 SITL development environment.
# It will be replaced when the competition sim drops.
# ============================================================================

GATES = [
    {
        "position": np.array([10.0, 0.0, -2.0]),
        "normal": np.array([1.0, 0.0, 0.0]),
        "width": 2.0,
        "height": 2.0,
        "label": "Gate 1 — straight ahead",
    },
    {
        "position": np.array([20.0, 5.0, -2.0]),
        "normal": np.array([0.87, 0.5, 0.0]),
        "width": 2.0,
        "height": 2.0,
        "label": "Gate 2 — gentle right",
    },
    {
        "position": np.array([25.0, 15.0, -2.5]),
        "normal": np.array([0.5, 0.87, 0.0]),
        "width": 2.0,
        "height": 2.0,
        "label": "Gate 3 — right turn",
    },
    {
        "position": np.array([20.0, 25.0, -3.0]),
        "normal": np.array([-0.5, 0.87, 0.0]),
        "width": 2.0,
        "height": 2.0,
        "label": "Gate 4 — hard right, climbing",
    },
    {
        "position": np.array([10.0, 28.0, -3.0]),
        "normal": np.array([-1.0, 0.0, 0.0]),
        "width": 2.0,
        "height": 2.0,
        "label": "Gate 5 — heading back",
    },
    {
        "position": np.array([0.0, 22.0, -2.5]),
        "normal": np.array([-0.5, -0.87, 0.0]),
        "width": 2.0,
        "height": 2.0,
        "label": "Gate 6 — left turn",
    },
    {
        "position": np.array([-5.0, 12.0, -2.0]),
        "normal": np.array([-0.5, -0.87, 0.0]),
        "width": 2.0,
        "height": 2.0,
        "label": "Gate 7 — continuing back",
    },
    {
        "position": np.array([0.0, 5.0, -2.0]),
        "normal": np.array([0.5, -0.87, 0.0]),
        "width": 2.0,
        "height": 2.0,
        "label": "Gate 8 — final turn",
    },
]

TOTAL_GATES = len(GATES)

# ============================================================================
# GATE PASSAGE DETECTION
# ============================================================================


class GateTracker:
    """Tracks which gates have been passed, in order."""

    def __init__(self, gates):
        self.gates = gates
        self.next_gate_idx = 0
        self.passed = []
        self.prev_position = None

    def update(self, position: np.ndarray) -> bool:
        """
        Check if the drone has passed through the next gate.
        Returns True if a new gate was just passed.

        Gate passage is detected when:
        1. The drone crosses the gate's plane (dot product with normal changes sign)
        2. The drone is within GATE_PASS_RADIUS of the gate center at the crossing point
        """
        if self.next_gate_idx >= len(self.gates):
            return False

        gate = self.gates[self.next_gate_idx]
        gate_pos = gate["position"]
        gate_normal = gate["normal"]

        if self.prev_position is not None:
            # Vector from gate center to previous and current position
            to_prev = self.prev_position - gate_pos
            to_curr = position - gate_pos

            # Check if we crossed the gate plane
            dot_prev = np.dot(to_prev, gate_normal)
            dot_curr = np.dot(to_curr, gate_normal)

            if dot_prev <= 0 and dot_curr > 0:
                # We crossed the plane in the forward direction
                # Interpolate to find crossing point
                t = dot_prev / (dot_prev - dot_curr)
                crossing = self.prev_position + t * (position - self.prev_position)
                dist_to_center = np.linalg.norm(crossing - gate_pos)

                if dist_to_center < GATE_PASS_RADIUS:
                    self.passed.append({
                        "gate_idx": self.next_gate_idx,
                        "label": gate["label"],
                        "time": time.time(),
                        "distance_from_center": dist_to_center,
                    })
                    self.next_gate_idx += 1
                    self.prev_position = position
                    return True

        self.prev_position = position.copy()
        return False

    @property
    def gates_passed(self):
        return len(self.passed)

    @property
    def all_passed(self):
        return self.gates_passed >= len(self.gates)


# ============================================================================
# TRAJECTORY LOGGING
# ============================================================================


def save_trajectory(results, trajectory):
    """Save trajectory and run metadata for visualization."""
    os.makedirs(TRAJECTORY_DIR, exist_ok=True)

    # Count existing files to get run number
    existing = [f for f in os.listdir(TRAJECTORY_DIR) if f.endswith(".npz")]
    run_num = len(existing) + 1

    traj_array = np.array(trajectory) if trajectory else np.zeros((0, 4))
    gate_positions = np.array([g["position"] for g in GATES])
    gate_normals = np.array([g["normal"] for g in GATES])

    filename = os.path.join(TRAJECTORY_DIR, f"run_{run_num:04d}.npz")
    np.savez_compressed(
        filename,
        trajectory=traj_array,          # (N, 4): time, x, y, z
        gate_positions=gate_positions,   # (M, 3): x, y, z per gate
        gate_normals=gate_normals,       # (M, 3): nx, ny, nz per gate
        score=results["score"],
        lap_time=results["lap_time"],
        gates_passed=results["gates_passed"],
        total_gates=results["total_gates"],
        crashed=results["crashed"],
    )
    print(f"Trajectory saved: {filename}")


# ============================================================================
# PX4 SITL LIFECYCLE MANAGEMENT
# ============================================================================

_sim_process = None


def kill_sim():
    """Kill any existing PX4 and Gazebo processes."""
    global _sim_process
    if _sim_process is not None:
        try:
            os.killpg(os.getpgid(_sim_process.pid), signal.SIGKILL)
        except Exception:
            pass
        _sim_process = None
    # Also kill any strays
    for pattern in ["px4", "gz", "ruby"]:
        subprocess.run(
            ["pkill", "-9", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    # Kill anything holding the MAVSDK port to prevent bind errors
    subprocess.run(
        ["fuser", "-k", "14540/udp"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(2)


def launch_sim():
    """Launch PX4 SITL headless and return the subprocess."""
    global _sim_process
    print("Launching PX4 SITL (headless)...")
    env = os.environ.copy()
    env["HEADLESS"] = "1"
    _sim_process = subprocess.Popen(
        ["make", "px4_sitl", "gz_x500"],
        cwd=PX4_AUTOPILOT_DIR,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    return _sim_process


async def wait_for_sim_ready():
    """Wait for PX4 SITL to start up. The actual MAVSDK connection is handled by run_experiment."""
    print("Waiting for sim to initialize...")
    # PX4 SITL typically takes 15-25s to be ready. We give it a generous window.
    # The MAVSDK connection_state loop in run_experiment() handles the final handshake.
    await asyncio.sleep(20)
    if _sim_process is not None and _sim_process.poll() is not None:
        print(f"ERROR: Sim process exited with code {_sim_process.returncode}")
        return False
    print("Sim startup wait complete.")
    return True


def restart_sim():
    """Kill existing sim, launch fresh, return when ready."""
    kill_sim()
    launch_sim()


# ============================================================================
# MAIN RUN HARNESS
# ============================================================================


async def run_experiment():
    """
    Run one experiment:
    1. Connect to sim
    2. Arm, takeoff, enter offboard
    3. Call pilot.run()
    4. Monitor gates and timeout
    5. Return raw results
    """
    # Import pilot module (reload to pick up agent's changes)
    if "pilot" in sys.modules:
        importlib.reload(sys.modules["pilot"])
    import pilot

    # Import score module
    if "score" in sys.modules:
        importlib.reload(sys.modules["score"])
    import score

    # Restart PX4 SITL for a clean state
    for attempt in range(3):
        restart_sim()
        if await wait_for_sim_ready():
            break
        print(f"Sim startup attempt {attempt + 1}/3 failed, retrying...")
    else:
        kill_sim()
        return make_error_result("sim_startup_timeout")

    try:
        return await _fly_experiment(pilot, score)
    finally:
        kill_sim()


async def _fly_experiment(pilot, score):
    """Inner experiment logic — always wrapped by kill_sim() in the caller."""
    drone = System()
    print(f"Connecting to {MAVSDK_ADDRESS}...")
    await drone.connect(system_address=MAVSDK_ADDRESS)

    # Wait for connection
    connected = False
    deadline = time.time() + CONNECTION_TIMEOUT
    async for state in drone.core.connection_state():
        if state.is_connected:
            connected = True
            break
        if time.time() > deadline:
            break
    if not connected:
        print("ERROR: Connection timeout")
        return make_error_result("connection_timeout")

    print("Connected. Waiting for GPS fix / position estimate...")

    # Wait for position estimate (PX4 needs this before arming)
    health_deadline = time.time() + CONNECTION_TIMEOUT
    position_ok = False
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            position_ok = True
            print("Position estimate OK.")
            break
        if time.time() > health_deadline:
            break
    if not position_ok:
        print("ERROR: Position estimate timeout")
        return make_error_result("position_timeout")

    # Arm (with retry — PX4 SITL sometimes needs a few seconds after health OK)
    armed = False
    for arm_attempt in range(3):
        print(f"Arming (attempt {arm_attempt + 1}/3)...")
        try:
            await drone.action.arm()
            armed = True
            break
        except ActionError as e:
            print(f"  Arm attempt {arm_attempt + 1} failed: {e}")
            if arm_attempt < 2:
                await asyncio.sleep(3)
    if not armed:
        print("ERROR: Arming failed after 3 attempts")
        return make_error_result("arm_failed")

    # Takeoff
    print(f"Taking off to {TAKEOFF_ALTITUDE}m...")
    try:
        await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
        await drone.action.takeoff()
    except ActionError as e:
        print(f"ERROR: Takeoff failed: {e}")
        return make_error_result("takeoff_failed")

    # Wait for takeoff
    await asyncio.sleep(5)

    # Enter offboard mode with an initial setpoint
    print("Entering offboard mode...")
    try:
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -TAKEOFF_ALTITUDE, 0.0)
        )
        await drone.offboard.start()
    except OffboardError as e:
        print(f"ERROR: Offboard mode failed: {e}")
        return make_error_result("offboard_failed")

    print("Offboard mode active. Starting pilot...")

    # Set up gate tracking
    tracker = GateTracker(GATES)
    trajectory = []  # list of (timestamp, x, y, z) for visualization
    start_time = time.time()
    crashed = False
    timeout = False

    # Prepare gate info for pilot (read-only copy)
    gate_info = [
        {
            "position": g["position"].copy(),
            "normal": g["normal"].copy(),
            "width": g["width"],
            "height": g["height"],
            "label": g["label"],
        }
        for g in GATES
    ]

    GATE_PROGRESS_TIMEOUT = 30.0  # abort if no new gate in this many seconds

    # Run pilot and gate monitor concurrently
    async def monitor_gates():
        """Poll position and check gate passage."""
        nonlocal crashed, timeout
        last_gate_time = time.time()
        while not tracker.all_passed:
            elapsed = time.time() - start_time
            if elapsed > MAX_RUN_DURATION:
                timeout = True
                return
            if time.time() - last_gate_time > GATE_PROGRESS_TIMEOUT:
                print(f"No gate progress in {GATE_PROGRESS_TIMEOUT}s, aborting.")
                timeout = True
                return

            try:
                async for odom in drone.telemetry.odometry():
                    pos = np.array([
                        odom.position_body.x_m,
                        odom.position_body.y_m,
                        odom.position_body.z_m,
                    ])
                    trajectory.append((elapsed, pos[0], pos[1], pos[2]))
                    passed = tracker.update(pos)
                    if passed:
                        last_gate_time = time.time()
                        g = tracker.passed[-1]
                        elapsed = time.time() - start_time
                        print(
                            f"  GATE {g['gate_idx']+1}/{TOTAL_GATES} "
                            f"({g['label']}) at {elapsed:.1f}s "
                            f"[{g['distance_from_center']:.2f}m from center]"
                        )
                    break  # only read one position per poll
            except Exception:
                pass

            await asyncio.sleep(1.0 / TELEMETRY_RATE_HZ)

    async def run_pilot():
        """Run the agent's pilot code."""
        nonlocal crashed
        try:
            await pilot.run(drone, gate_info)
        except Exception as e:
            print(f"PILOT CRASHED: {e}")
            traceback.print_exc()
            crashed = True

    # Run both concurrently — pilot flies, monitor watches gates
    pilot_task = asyncio.create_task(run_pilot())
    monitor_task = asyncio.create_task(monitor_gates())

    # Wait for either: all gates passed, pilot finishes, timeout, or crash
    done, pending = await asyncio.wait(
        [pilot_task, monitor_task],
        timeout=MAX_RUN_DURATION,
        return_when=asyncio.FIRST_COMPLETED,
    )

    # Cancel remaining tasks
    for task in pending:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    # Compute timing
    end_time = time.time()
    total_elapsed = end_time - start_time

    # If all gates passed, lap_time is the time the last gate was passed
    if tracker.all_passed:
        lap_time = tracker.passed[-1]["time"] - start_time
    else:
        lap_time = 0.0

    # Land (skip the wait on failed/timed-out runs — sim is about to be killed anyway)
    print("Landing...")
    try:
        await drone.action.land()
        if tracker.all_passed:
            await asyncio.sleep(2)
    except Exception:
        pass

    # Disarm
    try:
        await drone.action.disarm()
    except Exception:
        pass

    # Build raw results
    results = {
        "lap_time": lap_time,
        "gates_passed": tracker.gates_passed,
        "total_gates": TOTAL_GATES,
        "crashed": crashed,
        "timeout": timeout,
        "total_elapsed": total_elapsed,
        "gate_details": tracker.passed,
    }

    # Compute score
    try:
        final_score = score.compute(results)
    except Exception as e:
        print(f"SCORE FUNCTION ERROR: {e}")
        final_score = 9999.0

    results["score"] = final_score

    # Save trajectory for visualization
    save_trajectory(results, trajectory)

    return results


def make_error_result(reason):
    """Create a result dict for pre-flight errors."""
    return {
        "lap_time": 0.0,
        "gates_passed": 0,
        "total_gates": TOTAL_GATES,
        "crashed": True,
        "timeout": False,
        "total_elapsed": 0.0,
        "gate_details": [],
        "score": 9999.0,
        "error": reason,
    }


def print_summary(results):
    """Print results in a grep-friendly format (mirrors Karpathy's autoresearch)."""
    print("\n---")
    print(f"score:            {results['score']:.3f}")
    print(f"lap_time:         {results['lap_time']:.1f}")
    print(f"gates_passed:     {results['gates_passed']}/{results['total_gates']}")
    print(f"crashed:          {results['crashed']}")
    print(f"timeout:          {results.get('timeout', False)}")
    print(f"total_elapsed:    {results['total_elapsed']:.1f}")
    if results.get("error"):
        print(f"error:            {results['error']}")
    print("---")


async def main():
    transient_errors = {"arm_failed", "connection_timeout", "position_timeout", "sim_startup_timeout"}
    for attempt in range(3):
        results = await run_experiment()
        if results.get("error") not in transient_errors:
            break
        print(f"Transient failure ({results['error']}), retrying ({attempt + 1}/3)...")
    print_summary(results)


def _cleanup_on_exit(signum, frame):
    """Ensure PX4/Gazebo processes are killed on interrupt."""
    print(f"\nCaught signal {signum}, cleaning up...")
    kill_sim()
    sys.exit(1)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, _cleanup_on_exit)
    signal.signal(signal.SIGTERM, _cleanup_on_exit)
    asyncio.run(main())
