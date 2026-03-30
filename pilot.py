"""
pilot.py — Lookahead waypoint follower with gate-alignment protection.

For each gate, two waypoints align the drone with the gate normal:
1. Approach point: 3m before gate along -normal (align heading)
2. Through point: 2m past gate along +normal (ensure passage)

Lookahead blending only on through->approach segments (between gates),
never on approach->through (critical alignment).
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
LOOKAHEAD_DIST = 6.0    # meters ahead on path for blending
COMMAND_RATE_HZ = 30


async def run(drone, gates):
    """Fly through all gates using lookahead waypoint following."""
    # Build waypoint sequence: approach + through per gate
    # Even indices = approach, odd indices = through
    waypoints = []
    for gate in gates:
        n = gate["normal"]
        c = gate["position"]
        waypoints.append(c - APPROACH_DIST * n)  # approach (even idx)
        waypoints.append(c + THROUGH_DIST * n)   # through (odd idx)

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

        # Lookahead blending toward next waypoint
        # Through->approach (between gates): aggressive blend (0.8)
        # Approach->through (gate passage): gentle blend (0.3) to preserve alignment
        is_through_wp = (idx % 2 == 1)
        cmd_target = target
        max_blend = 0.8 if is_through_wp else 0.3

        if idx + 1 < len(waypoints) and distance < LOOKAHEAD_DIST:
            remaining = LOOKAHEAD_DIST - distance
            next_wp = waypoints[idx + 1]
            to_next = next_wp - target
            seg_len = np.linalg.norm(to_next)
            if seg_len > 0:
                blend = min(remaining / seg_len, max_blend)
                cmd_target = target + blend * to_next

        cmd_delta = cmd_target - position
        yaw_deg = math.degrees(math.atan2(cmd_delta[1], cmd_delta[0]))
        await drone.offboard.set_position_ned(
            PositionNedYaw(
                north_m=cmd_target[0],
                east_m=cmd_target[1],
                down_m=cmd_target[2],
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
