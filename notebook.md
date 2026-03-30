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
