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
from mavsdk.offboard import PositionNedYaw


# ============================================================================
# CONFIGURATION
# ============================================================================

APPROACH_DIST = 3.0     # meters before gate along -normal
THROUGH_DIST = 2.0      # meters past gate along +normal
GATE_REACHED_DIST = 2.0 # switch to next waypoint when this close
LOOKAHEAD = 10.0        # meters ahead on polyline path
COMMAND_RATE_HZ = 50


EASY_TURN_THRESHOLD = 0.7  # cos(45°) — gates with gentler turns skip hard stop

CORNER_CUT_MAX = 0.5     # max corner-cutting offset from gate center (m)


async def run(drone, gates):
    """Fly through all gates using multi-waypoint path lookahead."""
    # Precompute which gates have easy turns (don't need hard stop)
    hard_stop_gates = set()
    hard_stop_gates.add(0)  # first gate always needs alignment
    for i in range(1, len(gates)):
        cos_angle = np.dot(gates[i - 1]["normal"], gates[i]["normal"])
        if cos_angle <= EASY_TURN_THRESHOLD:
            hard_stop_gates.add(i)

    # Compute corner-cutting offsets: shift gate crossing toward inside of turn
    offsets = compute_corner_offsets(gates)

    # Build waypoint sequence: approach + through per gate
    waypoints = []
    for i, gate in enumerate(gates):
        n = gate["normal"]
        c = gate["position"] + offsets[i]
        ad = 4.0 if i == len(gates) - 1 else APPROACH_DIST
        waypoints.append(c - ad * n)             # even = approach
        waypoints.append(c + THROUGH_DIST * n)   # odd = through

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

        cmd_target = walk_along_path(waypoints, idx, position, LOOKAHEAD,
                                     hard_stop_gates)

        delta = cmd_target - position
        yaw_deg = math.degrees(math.atan2(delta[1], delta[0]))

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                north_m=cmd_target[0],
                east_m=cmd_target[1],
                down_m=cmd_target[2],
                yaw_deg=yaw_deg,
            )
        )

        await asyncio.sleep(1.0 / COMMAND_RATE_HZ)


def compute_corner_offsets(gates):
    """Compute per-gate offsets that cut corners by shifting toward inside of turns."""
    offsets = []
    for i, gate in enumerate(gates):
        if i == 0 or i == len(gates) - 1:
            offsets.append(np.zeros(3))  # no offset for first/last gate
            continue

        # Direction from previous gate to this gate, and this gate to next
        incoming = gates[i]["position"] - gates[i - 1]["position"]
        outgoing = gates[i + 1]["position"] - gates[i]["position"]
        incoming = incoming / np.linalg.norm(incoming)
        outgoing = outgoing / np.linalg.norm(outgoing)

        # Bisector points toward outside of turn; negate for inside
        bisector = incoming + outgoing
        bisector_norm = np.linalg.norm(bisector)
        if bisector_norm < 0.01:
            offsets.append(np.zeros(3))
            continue

        # Project onto gate plane (perpendicular to normal)
        n = gate["normal"]
        plane_component = bisector - np.dot(bisector, n) * n
        plane_norm = np.linalg.norm(plane_component)
        if plane_norm < 0.01:
            offsets.append(np.zeros(3))
            continue

        # Offset toward inside of turn (negate bisector), capped
        offset = -(plane_component / plane_norm) * CORNER_CUT_MAX
        offsets.append(offset)

    return offsets


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
