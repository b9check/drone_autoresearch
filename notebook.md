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

## 2026-03-30 — Session 2 Learnings

- **Soft alignment (bleed 70%)**: tested, missed gate 8 (7/8). Hard stops are necessary for gate 8.
- **Selective hard stops**: gates with <45° entry turns (2, 3, 7) skip hard stop. Improved 10.5→10.27s. This is a reliable optimization.
- **Lookahead > 10m**: 11m consistently fails at gate 4 (3/8 in 3 consecutive runs). 10.5m is inconsistent (9.9s and 10.4s). 10m is the safe maximum.
- **The root cause of >10m failure**: on easy-turn gates (2, 3, 7), the lookahead skips the approach waypoint, causing corner cutting. Gate 3 is the critical one — its exit feeds into gate 4's 60° hard turn. Any extra corner cutting at gate 3 misaligns gate 4.
- **Approach dist 2.5m**: 8/8 gates but SLOWER (10.7s vs 10.3s). Shorter approach reduces alignment room, PX4 takes wider arcs.
- **Through dist 1.5m**: fails badly (1/8). Gate passages too far from center. 2.0m is minimum safe through distance.
- **Velocity feedforward**: set_position_velocity_ned with 15m velocity lookahead fails (3/8). The velocity direction conflicts with position hard stop at turns — PX4 averages both targets, takes bad trajectory.
- **Adaptive lookahead** (speed-based): same failure mode as increased fixed lookahead. Any effective lookahead >10m on easy sections breaks gate 4.
- **Current best: 10.27s**, commit fed9ef0, 10m lookahead + selective hard stops. Gate 8 at 1.35m from center (0.15m margin).
- **Best score progression**: 21.2s → 20.3s → 18.5s → 17.1s → 16.6s → 16.4s → 14.1s → 13.3s → 13.2s → 10.5s → 10.27s.

## 2026-03-30 — Session 3: Phase A+ Attempts and Gate 8 Fix

- **Gate 8 approach 4m (was 3m)**: improved gate 8 margin from 1.48m to 1.23-1.25m (0.27m safety margin vs 0.02m). Score 10.1-10.2s. Reliable across 2 runs. The 30° heading mismatch from gate 7 needs more alignment room.
- **Velocity feedforward (set_position_velocity_ned)**: aligned velocity direction with position target, magnitude proportional to distance. 7/8 gates — gate 8 missed. Velocity causes PX4 to take wider arcs. Zeroing velocity at hard stops makes it WORSE (3/8) — creates discontinuity.
- **Hybrid VelocityNedYaw on easy through segments**: 12 m/s velocity on easy inter-gate segments. Improved speed slightly (10.2s vs 10.3s) but degraded gate 8 to 1.46-1.48m. Even with 4m gate 8 approach + velocity, gate 8 is inconsistent. Limiting velocity to early-course only (gates 0-2) showed no improvement.
- **set_maximum_speed(20)**: no effect on PX4 position controller speed.
- **Gate 0 optimization (1m approach, no hard stop)**: no effect — total path length unchanged because approach distance doesn't change total path when inter-gate direction roughly aligns with gate normal.
- **Key insight**: reducing approach distance doesn't shorten the total path — it just shifts where the waypoint is on the same-length path. Velocity feedforward in any form is incompatible with this course's tight gate 8 alignment. Phase A position mode is definitively at its ceiling (~10.1s).
- **Attitude control (set_attitude)**: tested -15° to -25° pitch. Even with closed-loop altitude correction, the drone drops or climbs uncontrollably. Altitude sign was wrong initially (NED z is down). After fixing, all gates are ~1s slower each. Attitude setpoints don't work reliably in this PX4 SITL — possibly the offboard attitude interface isn't fully supported or hover thrust (0.37) is wrong.
- **Gentle velocity feedforward (3 m/s)**: even 3 m/s constant velocity hint breaks gate 3→4 transition (3/8). set_position_velocity_ned is FUNDAMENTALLY incompatible with this course — ANY velocity feedforward prevents PX4 from decelerating at hard-stop approach waypoints, causing overshoot at gate 4.
- **Corner-cutting offsets**: shifts gate crossing toward inside of turns. Geometrically correct but directly adds to measured gate-center distance (offsets from ORIGINAL center). 0.5m offset pushed gate 3 to 1.37m (0.13m margin). Not viable unless combined with wider gates.
- **Position mode ceiling is ~10.1s.** Cannot be broken by: velocity feedforward, VelocityNedYaw hybrid, max speed command, lookahead increase, approach/through tuning, attitude control, or corner cutting. The only path forward is either a WORKING attitude controller (needs correct hover thrust measurement) or a fundamentally different control scheme.
- **Current best**: 10.1s, commit bc35de1, with 4m gate 8 approach.
- **Best score progression**: ... → 10.27s → 10.1s (gate 8 fix).
- **Hover thrust measured**: 0.387 via binary search calibration. Even with correct thrust, attitude control disrupts hard turns (3/8).
- **Gate 3 relaxation is the key finding**: removing gate 3's hard stop gives 10.0s (2/3 runs), vs 10.8s typical before. Gate 4's hard stop still handles alignment.
- **Gate 4, 5 relaxation**: no improvement (10.9s each). Only gate 3 benefits from relaxation.
- **Gate 7 through 3m**: slower (11.2s), marginal gate 8 gain. Not worth it.
- **5m gate 8 approach**: same margin as 4m but slower. 4m is optimal.
- **Sim variance**: ±0.8s (10.0 to 10.8 for the same config). Need 0.5s+ improvement to be detectable.
- **Session 3 end state**: commit 69988f1. Best config: 10m lookahead, gate 3 relaxed, gate 8 4m approach, selective hard stops {0, 4, 5, 7}. Best: 10.0s.
- **Next session**: try time-based target progression (target advances along path at fixed speed) or a completely different path planning approach (Dubins paths, pre-computed optimal path).
