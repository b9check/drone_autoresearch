"""
pilot.py — Approach+through waypoint strategy for reliable gate passage.

For each gate, two waypoints align the drone with the gate normal:
1. Approach point: 3m before gate along -normal (align heading)
2. Through point: 2m past gate along +normal (ensure passage)

The approach->through line passes exactly through gate center.
"""

import asyncio
import math
import numpy as np
from mavsdk.offboard import PositionNedYaw


# ============================================================================
# CONFIGURATION
# ============================================================================

APPROACH_DIST = 3.0     # meters before gate along -normal
THROUGH_DIST = 2.0      # meters past gate along +normal
GATE_REACHED_DIST = 2.0 # switch to next waypoint when this close
COMMAND_RATE_HZ = 30


async def run(drone, gates):
    """Fly through all gates using approach+through waypoints."""
    # Build waypoint sequence: approach + through per gate
    waypoints = []
    for gate in gates:
        n = gate["normal"]
        c = gate["position"]
        waypoints.append(c - APPROACH_DIST * n)
        waypoints.append(c + THROUGH_DIST * n)

    idx = 0
    while idx < len(waypoints):
        target = waypoints[idx]
        position = await get_position(drone)
        if position is None:
            await asyncio.sleep(1.0 / COMMAND_RATE_HZ)
            continue

        delta = target - position
        distance = np.linalg.norm(delta)

        if distance < GATE_REACHED_DIST:
            idx += 1
            continue

        yaw_deg = math.degrees(math.atan2(delta[1], delta[0]))
        await drone.offboard.set_position_ned(
            PositionNedYaw(
                north_m=target[0],
                east_m=target[1],
                down_m=target[2],
                yaw_deg=yaw_deg,
            )
        )

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
