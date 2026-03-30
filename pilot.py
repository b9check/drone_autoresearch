"""
pilot.py — Targeted approach+through for gate 8 only.

Gates 1-7: target gate centers (fast, already reliable).
Gate 8: approach waypoint (3m before) + through waypoint (2m past) along
normal to fix the PX4 controller curvature issue.
"""

import asyncio
import math
import numpy as np
from mavsdk.offboard import PositionNedYaw


# ============================================================================
# CONFIGURATION
# ============================================================================

GATE_REACHED_DIST = 2.0
COMMAND_RATE_HZ = 30
GATE8_APPROACH = 3.0    # meters before gate 8 along -normal
GATE8_THROUGH = 2.0     # meters past gate 8 along +normal


async def run(drone, gates):
    """Fly through gates 1-7 directly, use approach+through for gate 8."""
    waypoints = []
    for i, gate in enumerate(gates):
        if i == 7:  # Gate 8: approach + through
            n = gate["normal"]
            c = gate["position"]
            waypoints.append(c - GATE8_APPROACH * n)
            waypoints.append(c + GATE8_THROUGH * n)
        else:
            waypoints.append(gate["position"])

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
