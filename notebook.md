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
