"""
prepare.py — FROZEN evaluation harness for autoresearch-drone.

DO NOT MODIFY THIS FILE. The agent modifies pilot.py and score.py only.

This file:
1. Defines the gate course (positions, normals, dimensions)
2. Connects to PX4 SITL via MAVSDK
3. Arms, takes off, enters offboard mode
4. Calls pilot.run(drone, gates) — the agent's code
5. Monitors gate passage and enforces timeout
6. Logs raw metrics and computes score
7. Prints summary in a grep-friendly format
"""

import asyncio
import time
import sys
import importlib
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
GATE_PASS_DOT_THRESHOLD = 0.3   # cos(angle) threshold for direction check
TELEMETRY_RATE_HZ = 50          # how often we poll position for gate detection
CONNECTION_TIMEOUT = 30.0       # seconds to wait for MAVSDK connection
TAKEOFF_TIMEOUT = 15.0          # seconds to wait for takeoff completion

MAVSDK_ADDRESS = "udp://:14540"

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
        2. The drone is within the gate's radius at the crossing point
        3. The drone is moving roughly in the gate's forward direction
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
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("Position estimate OK.")
            break

    # Arm
    print("Arming...")
    try:
        await drone.action.arm()
    except ActionError as e:
        print(f"ERROR: Arming failed: {e}")
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

    # Run pilot and gate monitor concurrently
    async def monitor_gates():
        """Poll position and check gate passage."""
        nonlocal crashed, timeout
        while not tracker.all_passed:
            elapsed = time.time() - start_time
            if elapsed > MAX_RUN_DURATION:
                timeout = True
                return

            try:
                async for odom in drone.telemetry.odometry():
                    pos = np.array([
                        odom.position_body.x_m,
                        odom.position_body.y_m,
                        odom.position_body.z_m,
                    ])
                    passed = tracker.update(pos)
                    if passed:
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

    # Land
    print("Landing...")
    try:
        await drone.action.land()
        await asyncio.sleep(3)
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
    results = await run_experiment()
    print_summary(results)


if __name__ == "__main__":
    asyncio.run(main())
