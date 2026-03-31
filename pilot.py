"""
pilot.py — Multi-waypoint path lookahead with split position/velocity control.

Position target: 10m lookahead with hard stops at alignment-critical gates.
Velocity feedforward: 15m lookahead without stops, giving PX4 the overall
path direction to prevent unnecessary deceleration.
"""

import asyncio
import math
import numpy as np
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw


# ============================================================================
# CONFIGURATION
# ============================================================================

APPROACH_DIST = 3.0     # meters before gate along -normal
THROUGH_DIST = 2.0      # meters past gate along +normal
GATE_REACHED_DIST = 2.0 # switch to next waypoint when this close
POS_LOOKAHEAD = 10.0    # meters ahead for position target (with hard stops)
VEL_LOOKAHEAD = 15.0    # meters ahead for velocity direction (no stops)
VEL_FF_SPEED = 8.0      # m/s velocity feedforward magnitude
COMMAND_RATE_HZ = 50

EASY_TURN_THRESHOLD = 0.7  # cos(45°) — gates with gentler turns skip hard stop


async def run(drone, gates):
    """Fly through all gates using split position/velocity control."""
    # Build waypoint sequence: approach + through per gate
    waypoints = []
    for gate in gates:
        n = gate["normal"]
        c = gate["position"]
        waypoints.append(c - APPROACH_DIST * n)  # even = approach
        waypoints.append(c + THROUGH_DIST * n)   # odd = through

    # Precompute which gates have easy turns (don't need hard stop)
    hard_stop_gates = set()
    hard_stop_gates.add(0)  # first gate always needs alignment
    for i in range(1, len(gates)):
        cos_angle = np.dot(gates[i - 1]["normal"], gates[i]["normal"])
        if cos_angle <= EASY_TURN_THRESHOLD:
            hard_stop_gates.add(i)

    idx = 0
    while idx < len(waypoints):
        position = await get_position(drone)
        if position is None:
            await asyncio.sleep(1.0 / COMMAND_RATE_HZ)
            continue

        dist_to_wp = np.linalg.norm(waypoints[idx] - position)
        if dist_to_wp < GATE_REACHED_DIST:
            idx += 1
            continue

        # Position target: 10m with hard stops for alignment
        pos_target = walk_along_path(waypoints, idx, position, POS_LOOKAHEAD,
                                     hard_stop_gates)
        # Velocity direction: 15m without stops for speed
        vel_target = walk_along_path(waypoints, idx, position, VEL_LOOKAHEAD,
                                     set())

        delta = pos_target - position
        yaw_deg = math.degrees(math.atan2(delta[1], delta[0]))

        vel_dir = vel_target - position
        vel_dist = np.linalg.norm(vel_dir)
        if vel_dist > 1.0:
            vel_dir = vel_dir / vel_dist
            await drone.offboard.set_position_velocity_ned(
                PositionNedYaw(
                    north_m=pos_target[0],
                    east_m=pos_target[1],
                    down_m=pos_target[2],
                    yaw_deg=yaw_deg,
                ),
                VelocityNedYaw(
                    north_m_s=vel_dir[0] * VEL_FF_SPEED,
                    east_m_s=vel_dir[1] * VEL_FF_SPEED,
                    down_m_s=vel_dir[2] * VEL_FF_SPEED,
                    yaw_deg=0.0,
                ),
            )
        else:
            await drone.offboard.set_position_ned(
                PositionNedYaw(
                    north_m=pos_target[0],
                    east_m=pos_target[1],
                    down_m=pos_target[2],
                    yaw_deg=yaw_deg,
                )
            )

        await asyncio.sleep(1.0 / COMMAND_RATE_HZ)


def walk_along_path(waypoints, idx, position, lookahead, hard_stop_gates):
    """Walk lookahead meters along polyline. Stop only at hard-stop approach waypoints."""
    current = waypoints[idx]
    dist_to_wp = np.linalg.norm(current - position)

    if dist_to_wp >= lookahead:
        return current

    remaining = lookahead - dist_to_wp
    prev = current
    j = idx + 1

    while j < len(waypoints) and remaining > 0:
        # Only stop at approach waypoints for hard turns
        if j % 2 == 0 and j > idx + 1:
            gate_idx = j // 2
            if gate_idx in hard_stop_gates:
                return waypoints[j]

        seg = waypoints[j] - prev
        seg_len = np.linalg.norm(seg)
        if seg_len <= 0:
            j += 1
            continue
        if remaining <= seg_len:
            return prev + (remaining / seg_len) * seg
        remaining -= seg_len
        prev = waypoints[j]
        j += 1

    return waypoints[-1]


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
