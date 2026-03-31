"""
pilot.py — Multi-waypoint path lookahead with gate alignment protection.

For each gate, two waypoints align the drone with the gate normal:
1. Approach point: 3m before gate along -normal (align heading)
2. Through point: 2m past gate along +normal (ensure passage)

The drone follows the polyline path with a lookahead target. The lookahead
walks along the path but STOPS at approach waypoints (even indices) to
preserve gate alignment.
"""

import asyncio
import math
import numpy as np
from mavsdk.offboard import Attitude, PositionNedYaw


# ============================================================================
# CONFIGURATION
# ============================================================================

APPROACH_DIST = 3.0     # meters before gate along -normal
THROUGH_DIST = 2.0      # meters past gate along +normal
GATE_REACHED_DIST = 2.0 # switch to next waypoint when this close
LOOKAHEAD = 10.0        # meters ahead on polyline path
COMMAND_RATE_HZ = 50

EASY_TURN_THRESHOLD = 0.7  # cos(45°) — gates with gentler turns skip hard stop

# Phase C: attitude control with altitude hold
ATT_PITCH = -20.0       # degrees, forward pitch
ATT_HOVER = 0.37        # estimated hover thrust (PX4 x500)
ATT_KP_ALT = 0.3        # altitude correction gain
ATT_SWITCH_DIST = 5.0   # switch to position mode this close to next waypoint


async def run(drone, gates):
    """Fly through all gates using multi-waypoint path lookahead."""
    # Build waypoint sequence: approach + through per gate
    # Gate 8 (idx 7) gets longer approach for alignment (tight 30° heading change)
    waypoints = []
    for i, gate in enumerate(gates):
        n = gate["normal"]
        c = gate["position"]
        ad = 4.0 if i == len(gates) - 1 else APPROACH_DIST
        waypoints.append(c - ad * n)             # even = approach
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

        # Check if we should use attitude mode (easy inter-gate through segments)
        is_through = (idx % 2 == 1)
        next_gate_idx = (idx + 1) // 2
        next_is_easy = (next_gate_idx < len(gates)
                        and next_gate_idx not in hard_stop_gates)
        next_wp = waypoints[idx + 1] if idx + 1 < len(waypoints) else None

        if (is_through and next_is_easy and next_wp is not None
                and np.linalg.norm(next_wp - position) > ATT_SWITCH_DIST):
            # Attitude mode with closed-loop altitude hold
            delta = next_wp - position
            yaw_deg = math.degrees(math.atan2(delta[1], delta[0]))

            # Altitude correction: maintain target altitude
            alt_error = next_wp[2] - position[2]  # negative = too high
            pitch_cos = math.cos(math.radians(ATT_PITCH))
            thrust = (ATT_HOVER + ATT_KP_ALT * alt_error) / pitch_cos
            thrust = max(0.1, min(0.8, thrust))

            await drone.offboard.set_attitude(Attitude(
                roll_deg=0.0, pitch_deg=ATT_PITCH,
                yaw_deg=yaw_deg, thrust_value=thrust,
            ))
        else:
            # Position mode: standard lookahead
            cmd_target = walk_along_path(waypoints, idx, position, LOOKAHEAD,
                                         hard_stop_gates)
            delta = cmd_target - position
            yaw_deg = math.degrees(math.atan2(delta[1], delta[0]))

            await drone.offboard.set_position_ned(
                PositionNedYaw(cmd_target[0], cmd_target[1], cmd_target[2],
                               yaw_deg)
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
