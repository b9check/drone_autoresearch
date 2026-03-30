# Lab Notebook

Empirical discoveries about the simulator, drone, track, and control problem. This file is append-only. Never delete previous entries.

---

## 2026-03-29 — Baseline Run

- MAVSDK connection works with `udp://:14540` (deprecation warning, harmless).
- `odom.position_body.x_m/y_m/z_m` fields are correct for NED position.
- Baseline waypoint follower passes 7/8 gates in ~14.7s elapsed.
- Gate 8 (final turn, position [0, 5, -2]) is missed — drone likely overshoots or approach angle is wrong.
- Gate passage detection works well: all passages within 0.3m of center.
- The sim's position controller tracks PositionNedYaw setpoints quickly — drone moves at roughly 5-7 m/s between gates with the baseline code.
- GATE_REACHED_DIST=2.0m in pilot.py switches to the next gate before actually passing through — this works because the gate tracker in prepare.py uses plane-crossing detection independently.

## 2026-03-29 — Gate 8 Trajectory Analysis (run_0004.npz)

- Drone crosses gate 8's plane at position ~[-2.2, 3.5, -2.0], which is **2.68m from gate center** — outside the 1.5m GATE_PASS_RADIUS. This is why gate 8 is never registered.
- The PX4 position controller curves the path when transitioning from gate 7 momentum to the gate 8 target. The drone doesn't fly a straight line — it arcs, and the arc crosses the gate plane off-center.
- After crossing the plane, the drone settles near the through-point (~[1.5, 2.4, -2]) and hovers there until the 15s per-gate timeout fires. It never re-crosses the plane.
- Gate 8 normal is [0.5, -0.87, 0] — nearly aligned with the approach direction from gate 7 (8° difference). The geometry is fine; it's the PX4 controller's smooth transition that causes the off-center crossing.

## 2026-03-29 — VelocityNedYaw is Unstable

- VelocityNedYaw at 5 m/s caused severe oscillation. Drone bounced between gates 2 and 3 for ~20s before slowly progressing. Only 2/8 gates in 73s.
- PositionNedYaw is far more stable — the sim's inner-loop position controller provides damping. Stick with PositionNedYaw for Phase A.

## 2026-03-29 — Approach+Through Waypoints Fix Gate 8

- Adding approach waypoints (3m before gate along -normal) and through waypoints (2m past gate along +normal) achieves 8/8 gates in 21.2s.
- All gate passages within 0.39m of center. Gate 8: 0.05m from center (was 2.68m with baseline).
- The approach→through line is collinear with the gate normal and passes through gate center by construction.
- Key insight: through-offsets alone don't fix misalignment. The APPROACH waypoint is what forces the PX4 controller to get the drone on the correct heading before the gate plane.
- Lap time increased from ~14.7s (7 gates) to 21.2s (8 gates) — the extra waypoints add path length. Average 2.65s/gate vs 2.1s/gate baseline.
- Consecutive through→approach distances range from 5.2m to 7.3m, all safely above GATE_REACHED_DIST=2.0m.
- Gate 8 alignment depends on gate 7's through waypoint setting up the heading. Approach+through for gate 8 alone (centers for 1-7) still misses — the drone's uncontrolled heading from gate 7 center causes curving. Must use approach+through for ALL gates.

## 2026-03-29 — Lookahead Blending Optimization

- Lookahead blending on through→approach (inter-gate) segments: huge win. 4m lookahead → 20.3s, 6m → 18.5s.
- Adding gentle blending (0.3→0.5) on approach→through also helps: 17.1s → 16.6s.
- Gate 8 is extremely sensitive to alignment changes. Reducing approach dist to 2.5m or through dist to 1.5m causes gate 8 miss. 3.0m approach and 2.0m through are minimum safe values.
- Switch radius 3.0m also misses gate 8 — too early transition from approach waypoint.
- Current best: 16.6s, 8/8 gates. Lookahead with 6m distance, 0.8/0.5 blend (through/approach).
- The PX4 position controller's smoothing is the main speed limiter — it decelerates toward each commanded position. Lookahead partially defeats this by commanding a blended position further ahead.

## 2026-03-30 — Multi-Waypoint Path Lookahead

- Multi-waypoint walk_along_path with alignment protection at approach waypoints: 10.5s, 8/8 gates. Major improvement from single-blend (13.2s).
- The walk function walks LOOKAHEAD=10m along the polyline starting from current waypoint. It stops when reaching an approach waypoint (even index) beyond the immediate next, forcing alignment before each gate.
- GATE_REACHED_DIST=2.0m is the right threshold for 3m/2m approach/through. Shorter waypoints (1.5m/1m) cause the drone to skip all waypoints within 2m radius. Reducing approach/through requires reducing GATE_REACHED_DIST proportionally.
- 12m lookahead with multi-wp walk: drone skips waypoints entirely and gets stuck. 10m is the sweet spot.
- 3m position overshoot: fast on straights (gates 1-4 in 5.2s) but overshoots corners. Gate 5 took 11.3s instead of ~7s.
- Gate passages at 10.5s: distances from center range 0.20-1.30m. Gates 1 (1.29m) and 8 (1.30m) are close to the 1.5m GATE_PASS_RADIUS limit. This is a risk factor for reliability.
- Sim has intermittent "bind error: Address in use" when port 14540 isn't released between runs. Wait 15-20s after killing sim before relaunching.
- Cubic spline through waypoints: 0/8 — spline cuts inside gate planes. Splines need gate-plane constraints.
- set_position_velocity_ned with velocity feedforward: unstable. 12m/s overshoots corners, 3m/s breaks position tracking. Stick with pure PositionNedYaw.
- Gate centers for gates 1-6 + approach/through for 7-8: still misses gate 8. ALL gates need approach/through.

## 2026-03-30 — Session End State

- **Current pilot.py (commit 71dec67) is UNTESTED.** It has "soft alignment: bleed 70% of remaining lookahead at approach points" — a change from hard-stop to soft-bleed at approach waypoints. Never got a clean run due to port conflicts.
- **Last validated code: commit dc7f8cc, 10.5s, 8/8 gates.** If 71dec67 fails, revert to dc7f8cc's logic (hard stop at approach waypoints: `return waypoints[j]`).
- **Best score progression**: 21.2s → 20.3s → 18.5s → 17.1s → 16.6s → 16.4s → 14.1s → 13.3s → 13.2s → 10.5s.
- The soft alignment experiment is worth testing first next session — it might be faster than 10.5s since it allows some blending past approach points instead of a hard stop.
