"""
pilot.py — MUTABLE autonomy stack for autoresearch-drone.

The agent modifies this file to improve lap times.

This baseline implementation: simple waypoint following through gate centers
using SET_POSITION_TARGET_LOCAL_NED. It is intentionally naive — the agent
should improve everything about it.
"""

import asyncio
import math
import numpy as np
from mavsdk.offboard import PositionNedYaw


# ============================================================================
# CONFIGURATION — tune these or replace the entire approach
# ============================================================================

APPROACH_SPEED = 3.0        # m/s — how fast to fly toward gates
GATE_REACHED_DIST = 2.0     # meters — how close before switching to next gate
COMMAND_RATE_HZ = 30        # how often to send commands


async def run(drone, gates):
    """
    Fly through all gates in sequence.

    Args:
        drone: MAVSDK System object, already in offboard mode.
        gates: list of gate dicts, each with:
            - position: np.array([x, y, z]) in NED
            - normal: np.array([nx, ny, nz]) — direction to fly through
            - width: float
            - height: float
            - label: str
    """
    current_gate_idx = 0

    while current_gate_idx < len(gates):
        gate = gates[current_gate_idx]
        target = gate["position"]

        # Get current position
        position = await get_position(drone)
        if position is None:
            await asyncio.sleep(1.0 / COMMAND_RATE_HZ)
            continue

        # Distance to gate
        delta = target - position
        distance = np.linalg.norm(delta)

        # Check if we've reached this gate
        if distance < GATE_REACHED_DIST:
            current_gate_idx += 1
            continue

        # Compute yaw to face the gate
        yaw_rad = math.atan2(delta[1], delta[0])
        yaw_deg = math.degrees(yaw_rad)

        # Send position setpoint
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
