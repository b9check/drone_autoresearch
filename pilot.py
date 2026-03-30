"""
pilot.py — Autonomy stack for autoresearch-drone.

Velocity-based gate navigation with two-phase targeting:
- Far from gate: aim at gate center (correct approach angle)
- Close to gate: aim past gate along normal (ensures plane crossing)
"""

import asyncio
import math
import numpy as np
from mavsdk.offboard import VelocityNedYaw


# ============================================================================
# CONFIGURATION
# ============================================================================

APPROACH_SPEED = 5.0        # m/s — cruise speed toward gates
GATE_REACHED_DIST = 2.0     # meters — advance to next gate when past plane and this close
CLOSE_DIST = 5.0            # meters — switch from center to through-point targeting
THROUGH_OFFSET = 3.0        # meters past gate center along normal
COMMAND_RATE_HZ = 30        # command loop rate
MAX_TIME_PER_GATE = 15.0    # seconds before skipping a gate
RUN_TIMEOUT = 300.0         # total pilot runtime limit


async def run(drone, gates):
    """Fly through all gates using velocity commands."""
    import time as _time

    current_gate_idx = 0
    run_start = _time.time()
    gate_start = _time.time()

    while current_gate_idx < len(gates):
        if _time.time() - run_start > RUN_TIMEOUT:
            break
        if _time.time() - gate_start > MAX_TIME_PER_GATE:
            current_gate_idx += 1
            gate_start = _time.time()
            continue

        gate = gates[current_gate_idx]
        position = await get_position(drone)
        if position is None:
            await asyncio.sleep(1.0 / COMMAND_RATE_HZ)
            continue

        # Gate passage check: past plane and close to center
        to_gate = gate["position"] - position
        dist_to_gate = np.linalg.norm(to_gate)
        past_plane = np.dot(position - gate["position"], gate["normal"]) > 0

        if past_plane and dist_to_gate < GATE_REACHED_DIST:
            current_gate_idx += 1
            gate_start = _time.time()
            continue

        # Two-phase targeting:
        # Far away -> aim at gate center (correct approach angle)
        # Close or past -> aim at through-point (fly through the plane)
        if dist_to_gate < CLOSE_DIST or past_plane:
            target = gate["position"] + gate["normal"] * THROUGH_OFFSET
        else:
            target = gate["position"]

        # Velocity toward target
        to_target = target - position
        dist_to_target = np.linalg.norm(to_target)
        if dist_to_target > 0.1:
            direction = to_target / dist_to_target
        else:
            direction = gate["normal"]

        vel = direction * APPROACH_SPEED
        yaw_deg = math.degrees(math.atan2(direction[1], direction[0]))

        await drone.offboard.set_velocity_ned(VelocityNedYaw(
            north_m_s=vel[0],
            east_m_s=vel[1],
            down_m_s=vel[2],
            yaw_deg=yaw_deg,
        ))

        await asyncio.sleep(1.0 / COMMAND_RATE_HZ)

    # Hold briefly to let gate tracker register final passage
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    for _ in range(60):
        await asyncio.sleep(1.0 / COMMAND_RATE_HZ)


async def get_position(drone) -> np.ndarray:
    """Get current position from odometry. Returns np.array([x, y, z]) in NED."""
    try:
        async for odom in drone.telemetry.odometry():
            return np.array([
                odom.position_body.x_m,
                odom.position_body.y_m,
                odom.position_body.z_m,
            ])
    except Exception:
        return None
