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

TARGET_SPEED = 10.0       # m/s — speed at which the target advances along the path
MAX_LEAD = 10.0           # max distance target can be ahead of drone


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

    # Hard stop gates need alignment; easy turns let lookahead flow through
    # Gate 3 is 60° turn but gate 4's approach handles alignment → relax gate 3
    # Gate 7 MUST stay hard — gate 8's tight margin requires full alignment
    hard_stop_gates = set()
    hard_stop_gates.add(0)  # first gate always needs alignment
    for i in range(1, len(gates)):
        cos_angle = np.dot(gates[i - 1]["normal"], gates[i]["normal"])
        if cos_angle <= EASY_TURN_THRESHOLD:
            hard_stop_gates.add(i)
    hard_stop_gates.discard(3)  # relax gate 3 — gate 4 hard stop handles alignment

    # Time-based target: advances along path at TARGET_SPEED, pauses at hard stops
    target_dist = 0.0  # cumulative distance along path the target has traveled
    idx = 0            # current waypoint index (for gate-reached tracking)
    dt = 1.0 / COMMAND_RATE_HZ

    # Precompute cumulative path distances for each waypoint
    cum_dist = [0.0]
    for j in range(1, len(waypoints)):
        seg_len = np.linalg.norm(waypoints[j] - waypoints[j - 1])
        cum_dist.append(cum_dist[-1] + seg_len)

    # Compute hard-stop distances (distance along path to hard-stop approach waypoints)
    hard_stop_dists = set()
    for gi in hard_stop_gates:
        wp_idx = gi * 2  # approach waypoint index
        if wp_idx < len(cum_dist):
            hard_stop_dists.add(cum_dist[wp_idx])

    while idx < len(waypoints):
        position = await get_position(drone)
        if position is None:
            await asyncio.sleep(dt)
            continue

        # Check if drone has reached current waypoint
        dist_to_wp = np.linalg.norm(waypoints[idx] - position)
        if dist_to_wp < GATE_REACHED_DIST:
            idx += 1
            continue

        # Advance target along path (time-based)
        # Find the next hard-stop distance
        next_stop = float('inf')
        for hd in hard_stop_dists:
            if hd > target_dist + 0.1:
                next_stop = min(next_stop, hd)

        # Advance target, but stop at hard-stop approach points
        advance = TARGET_SPEED * dt
        target_dist = min(target_dist + advance, next_stop)

        # Cap lead distance: target can't be more than MAX_LEAD ahead of drone
        drone_path_dist = cum_dist[idx] - np.linalg.norm(waypoints[idx] - position)
        if target_dist - drone_path_dist > MAX_LEAD:
            target_dist = drone_path_dist + MAX_LEAD

        # Release hard stop when drone reaches it
        if target_dist in hard_stop_dists:
            if drone_path_dist >= target_dist - 1.0:
                pass  # target advances on next iteration when drone catches up

        # Convert target_dist to 3D position
        cmd_target = point_at_cumulative_dist(waypoints, cum_dist, target_dist)

        delta = cmd_target - position
        yaw_deg = math.degrees(math.atan2(delta[1], delta[0]))

        await drone.offboard.set_position_ned(
            PositionNedYaw(cmd_target[0], cmd_target[1], cmd_target[2], yaw_deg)
        )

        await asyncio.sleep(dt)


def point_at_cumulative_dist(waypoints, cum_dist, target_dist):
    """Get 3D position at cumulative distance along polyline."""
    target_dist = max(0.0, min(target_dist, cum_dist[-1]))
    for j in range(1, len(waypoints)):
        if cum_dist[j] >= target_dist:
            seg_start = cum_dist[j - 1]
            seg_len = cum_dist[j] - seg_start
            if seg_len < 0.001:
                return waypoints[j].copy()
            frac = (target_dist - seg_start) / seg_len
            return waypoints[j - 1] + frac * (waypoints[j] - waypoints[j - 1])
    return waypoints[-1].copy()


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
