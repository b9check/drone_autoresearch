"""
pilot.py — Autonomy stack for autoresearch-drone.

Position-based waypoint following with plane-crossing gate detection.
Targets a point past each gate center (along approach direction from
the previous gate) to ensure the drone flies through the gate plane.
"""

import asyncio
import math
import numpy as np
from mavsdk.offboard import PositionNedYaw


# ============================================================================
# CONFIGURATION
# ============================================================================

GATE_REACHED_DIST = 2.5     # meters — advance when past plane and within this distance
THROUGH_OFFSET = 2.0        # meters past gate center along approach direction
COMMAND_RATE_HZ = 30        # command loop rate
MAX_TIME_PER_GATE = 15.0    # seconds before skipping a gate
RUN_TIMEOUT = 300.0         # total pilot runtime limit


async def run(drone, gates):
    """Fly through all gates using position commands with plane-crossing detection."""
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

        # Gate passage: require crossing the gate plane AND being close
        to_pos = position - gate["position"]
        past_plane = np.dot(to_pos, gate["normal"]) > 0
        dist = np.linalg.norm(to_pos)

        if past_plane and dist < GATE_REACHED_DIST:
            current_gate_idx += 1
            gate_start = _time.time()
            continue

        # Target: point past gate center along approach direction
        # This ensures the drone flies THROUGH the gate, not just to it
        if current_gate_idx > 0:
            prev_pos = gates[current_gate_idx - 1]["position"]
        else:
            prev_pos = np.array([0.0, 0.0, -2.0])
        approach = gate["position"] - prev_pos
        approach_len = np.linalg.norm(approach)
        if approach_len > 0.01:
            approach_dir = approach / approach_len
        else:
            approach_dir = gate["normal"]

        target = gate["position"] + approach_dir * THROUGH_OFFSET

        delta = target - position
        yaw_rad = math.atan2(delta[1], delta[0])
        yaw_deg = math.degrees(yaw_rad)

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                north_m=target[0],
                east_m=target[1],
                down_m=target[2],
                yaw_deg=yaw_deg,
            )
        )

        await asyncio.sleep(1.0 / COMMAND_RATE_HZ)

    # Hold position briefly to let gate tracker register final passage
    for _ in range(90):
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
