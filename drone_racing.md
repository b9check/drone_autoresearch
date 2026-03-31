# Drone Racing: Physics, Trajectories, and Strategy

Everything the agent needs to know about flying a quadrotor fast through gates. Reads as a single narrative: physical model → how physics constrains motion → how to plan trajectories within those constraints → how to race.

---

## 1. The Physical Model

A quadrotor is a rigid body with 6 degrees of freedom actuated by 4 motors producing thrust along the body Z-axis. The sim handles the inner-loop attitude controller and motor mixing. You command attitude targets or position/velocity setpoints. Understanding the physics matters because it determines what commands are physically achievable.

### State

The drone's full state is position (x, y, z), velocity (vx, vy, vz), attitude (roll φ, pitch θ, yaw ψ), and angular rates (p, q, r). Position and velocity are in the NED world frame. Angular rates are in the body frame.

### Coordinate Frames

**NED (North-East-Down) — World Frame**: X = North, Y = East, Z = Down. To fly at altitude h meters: z = -h. This is standard aerospace convention. The Z axis will trip you up if you're not careful — positive Z is toward the ground.

**FRD (Forward-Right-Down) — Body Frame**: X = Forward (nose), Y = Right, Z = Down (belly). IMU data is in this frame. Odometry is typically in NED.

**Euler Angles**: Roll (φ) = rotation about forward axis, positive = right side down. Pitch (θ) = rotation about right axis, positive = nose up. Yaw (ψ) = rotation about down axis, positive = clockwise from above. Use quaternions internally for anything involving aggressive attitudes — Euler angles have gimbal lock at ±90° pitch.

### Translational Dynamics

To accelerate horizontally, the quadrotor must tilt. To go forward, pitch nose-down. To go right, roll right. Horizontal acceleration is fundamentally coupled to attitude — a quadrotor cannot accelerate horizontally while remaining level.

The total thrust must support both gravity and any horizontal acceleration:

```
T_required = m * sqrt(ax² + ay² + (g + az)²)
```

At hover: T = m * g. During aggressive maneuvering: T > m * g.

### Thrust-to-Weight Ratio (TWR)

TWR = T_max / (m * g). This determines the drone's maximum acceleration capability. A racing drone typically has TWR of 2:1 to 8:1. The sim drone's TWR is unknown — **measure it experimentally in your first few runs**.

To measure: command maximum thrust while level, observe upward acceleration from ODOMETRY. TWR = (g + a_up) / g.

### Aerodynamic Drag

The sim models drag, which increases with v² and creates a terminal velocity at each pitch angle. You can't go infinitely fast by pitching more aggressively. To measure top speed: command maximum forward pitch in a straight line and observe when velocity plateaus.

### Collision

Hitting anything (gate, obstacle, boundary) likely ends the run. The tolerance between "brushed and recovered" and "crashed" is unknown. Be conservative with clearance margins until you've tested this empirically.

---

## 2. How Physics Constrains Motion

### Cornering

To turn, the drone banks. Banking redirects thrust horizontally but reduces vertical thrust. The key relationships:

```
centripetal_acceleration = g * tan(bank_angle)
turn_radius = v² / (g * tan(bank_angle))
```

**The faster you go, the wider you turn** at constant bank angle. **The more you bank, the tighter you turn** but the more altitude you lose unless you increase total thrust.

Minimum turn radius at speed v with max bank angle φ_max:
```
r_min = v² / (g * tan(φ_max))
```

Concrete examples:
- v=10 m/s, φ=45°: r_min ≈ 10.2m
- v=5 m/s, φ=45°: r_min ≈ 2.5m
- v=5 m/s, φ=60°: r_min ≈ 1.5m

**Halving speed quarters the minimum turn radius.** This is why slowing down before tight corners is essential and the single biggest factor in cornering performance.

**Important**: These are physics limits. In position mode, PX4's internal controller imposes tighter limits — lower max speed, shallower max bank angle. The physics formulas tell you what's theoretically possible; the PX4 controller determines what's actually achievable in position mode. See Section 7 for how to break through the controller ceiling.

### Altitude Loss in Turns

During banking, vertical thrust drops. To maintain altitude at bank angle φ:

```
T_level = m * g / cos(φ)
```

At 45° bank: need 141% of hover thrust. At 60° bank: need 200%. If the drone's TWR isn't high enough, it physically cannot maintain altitude in steep turns. This is an empirical limit to discover.

### Attitude Controller Bandwidth

The sim's inner-loop controller tracks your attitude commands with finite bandwidth. A step command (e.g., 0° to 30° roll) takes some time to achieve. This latency means you must command attitude changes BEFORE you need them — you're always planning ahead of the drone's current state.

**Measure this**: command a step change in attitude, observe delay in ATTITUDE telemetry. This determines how far ahead you need to plan.

### Energy and Altitude

Kinetic energy (½mv²) and potential energy (mgh) are interchangeable. Diving converts altitude to speed for free. Climbing costs speed. On courses with gates at different heights, exploit gravity:
- Dive toward lower gates for free speed.
- If the next gate is higher, arrive with excess speed and pull up rather than climbing slowly.

### Flight Envelope Summary

Measure these empirically in early experiments — they directly constrain all planning:

| Parameter | How to Measure |
|-----------|----------------|
| Thrust-to-weight ratio | Full throttle hover, measure upward acceleration |
| Max forward speed | Full pitch forward, observe terminal velocity |
| Max stable bank angle | Increase roll progressively, find instability onset |
| Attitude controller settling time | Step response measurement |
| Command-to-effect latency | Command a change, measure delay in telemetry |

**Populating these values is one of your first priorities.**

---

## 3. Trajectory Planning

Given a sequence of gates, find the fastest path through all of them. Start simple, add complexity only when it demonstrably improves lap times.

### Level 1: Waypoint Following

Place a waypoint at each gate center. Fly to each in sequence.

```
for each gate:
    while not close_enough(position, gate.position):
        command = fly_toward(gate.position)
```

Properties: works in ~30 lines. Produces terrible "stop-and-go" paths — the drone decelerates to near-zero at each gate. Use only as the Phase A baseline.

### Level 2: Lookahead (Carrot-on-a-Stick)

Instead of flying to the current gate, fly toward a point that's `lookahead_distance` ahead along the path. This smooths trajectories and reduces stop-and-go.

Tuning: typical lookahead 2–5m. Too short = oscillation. Too long = cuts corners too aggressively and misses gates.

### Level 2.5: Approach/Through Gate Alignment

The single most important pattern for reliable gate passage. For each gate, place two waypoints along the gate normal:

1. **Approach point**: `APPROACH_DIST` meters before the gate center along `-normal`. This forces the drone onto the correct heading before reaching the gate.
2. **Through point**: `THROUGH_DIST` meters past the gate center along `+normal`. This ensures the drone actually crosses the gate plane.

The approach→gate→through line is collinear with the gate normal by construction, so the drone passes through the gate center at the correct angle regardless of where it came from.

**This must be applied to ALL gates, not just the ones that are currently failing.** A gate that passes fine with straight-on approach will fail when the drone's exit heading from the previous gate changes due to upstream optimizations. The approach waypoint is what makes each gate independent of upstream path changes.

Typical starting values: APPROACH_DIST = 3.0m, THROUGH_DIST = 2.0m. These interact with GATE_REACHED_DIST (switch radius) — reducing approach/through distances requires proportionally reducing the switch radius.

### Level 2.5 + Lookahead: The Alignment Cascade Problem

Combining lookahead with approach/through waypoints creates an interaction that dominates the tuning challenge. A lookahead that walks past an approach waypoint lets the drone "skip" the alignment phase, causing the gate to be approached at the wrong angle. This can cascade: a slightly wrong exit from gate N changes the approach to gate N+1, which changes the exit from gate N+1, and so on.

The solution is **selective hard stops**: the lookahead walks freely along the path but stops at approach waypoints for gates that require precise alignment (tight turns). Gates with gentle turns between them can allow the lookahead to pass through. The turn threshold (e.g., cos(45°) between consecutive gate normals) determines which gates need hard stops.

Lookahead beyond ~10m tends to cause cascading alignment failures even with hard stops, because the drone "sees" too far ahead and starts cutting corners on easy gates in ways that affect harder gates downstream.

### Level 3: Spline Paths

Fit a smooth curve (cubic spline, B-spline) through gate positions. This produces a continuous, differentiable path with well-defined curvature everywhere — which enables speed profiling.

**Caution**: A naive spline through gate centers will cut inside gate planes on corners, causing misses. The fix is gate-normal-constrained splines: instead of fitting through gate centers only, fit through the approach and through points (from Level 2.5). This forces the spline to pass through each gate along its normal. Alternatively, add inequality constraints ensuring the spline crosses each gate plane within the gate opening.

### Level 4: Corner-Cutting Offsets

The shortest path doesn't go through gate centers. On turns, the inside of the gate is shorter. Add tunable offsets at each gate: on left turns, shift the waypoint right within the gate opening, and vice versa. The offset amount per gate is a parameter the agent can optimize.

### Level 5: Multi-Gate Lookahead

When planning approach to gate N, consider gates N+1 and N+2. Example: if gate N+1 requires a hard right, approach gate N on its left side to set up the turn. Planning just one gate ahead is locally optimal but globally suboptimal.

### Level 6: Offline Trajectory Optimization

Compute the full time-optimal path before the run using numerical optimization. Parameterize the trajectory (e.g., polynomial segments), minimize total time subject to dynamic constraints (max speed, max acceleration, max curvature, gate passage). Solve with scipy.optimize. Track the optimized path during the race.

This only works well when gate positions are precisely known and the drone can track the planned path accurately. It's the highest-performance approach but the most fragile.

**Don't skip levels.** Each provides the baseline understanding for the next. The agent should progress through these as experiments justify the added complexity.

---

## 4. Speed Profiling

Separate from the path shape, speed profiling determines how fast the drone flies at each point. This is crucial — the same path at different speeds produces vastly different lap times and crash rates.

### Curvature-Based Speed Limits

At any point along the path with curvature κ:

```
v_max(κ) = sqrt(a_lateral_max / κ)
a_lateral_max = g * tan(φ_max)
```

On straights (κ ≈ 0): speed limited by drag and thrust. On tight corners (large κ): speed must drop significantly.

### Forward-Backward Algorithm

A practical speed profiling method:

1. Compute curvature κ(s) along the entire path.
2. Compute cornering speed limit v_max(s) at every point.
3. **Forward pass**: from current speed, accelerate at a_max until hitting v_max(s).
4. **Backward pass**: from the end, decelerate at a_max backward.
5. Take the minimum of forward, backward, and v_max at each point.

This produces a profile that accelerates on straights and brakes before corners — exactly like a racing driver.

### Key Parameters to Tune

- `a_max_accel`: max forward acceleration (limited by pitch and thrust)
- `a_max_brake`: max braking deceleration (quadrotors brake by pitching backward)
- `a_lateral_max`: max centripetal acceleration (limited by max bank angle)
- `v_max_straight`: max straight-line speed (limited by thrust and drag)
- `safety_margin`: multiply v_max by a factor < 1.0. **Start at 0.7, increase toward 1.0 as the system proves reliable.**

---

## 5. Gate Approach Geometry

How you approach a gate affects path length, risk, and setup for the next gate.

### Gate Normal Alignment

Gates have a facing direction (normal vector). Passing through at a steep oblique angle is risky: the effective opening narrows, and small position errors become misses. Keep approach angle within ~30° of the gate normal for reliability. Relax this in Phase C if you have precise tracking.

### Corner Cutting

For a turn at gate B between gates A and C: the shortest path passes through gate B off-center, on the inside of the turn. How much to cut depends on gate width. This is a per-gate tunable parameter.

### Multi-Gate Setup

The optimal approach to gate N depends on gate N+1. If a hard right follows, approach gate N on its left side. This requires planning at least 2 gates ahead.

---

## 6. Racing Strategy

### The Speed-Reliability Tradeoff

At X% of theoretical max speed, completion rate is roughly:
- 60%: ~99% completion. Almost never crashes.
- 80%: ~90% completion.
- 95%: ~50% completion.
- 100%: ~0% completion. Any perturbation crashes.

Expected performance = speed × reliability. In the autoresearch loop, reliability dominates because crashes waste experiments. For the final competition submission, you can push toward the aggressive end because you only need one clean run.

### Where Lap Time Lives

The differences between fast and slow systems are primarily in:
1. **Corner speed** — how much you slow down (or don't) for each turn.
2. **Transition smoothness** — jerky paths waste time on transients.
3. **Gate precision** — reliable gate passage without excessive safety margins.

Straight-line speed matters less than you'd think. The time is in the corners.

### Failure Modes (ranked by frequency)

1. **Gate miss**: fly past/around instead of through. Cause: inaccurate gate position or poor approach angle.
2. **Corner overshoot**: enter too fast, can't make the turn. Cause: insufficient speed profiling.
3. **Collision**: hit a gate or obstacle. Cause: tight tolerances, controller lag.
4. **Instability**: unrecoverable tumble. Cause: exceeding attitude limits.
5. **Timeout**: don't finish in 8 minutes. Cause: stuck, wrong direction, too slow.

### Safety Margins

Start with large margins and tighten experimentally:
- Gate passage: aim for center, don't use full width.
- Speed: 70% of theoretical max initially.
- Bank angle: stay well within stable range.
- Obstacle clearance: maintain a buffer.

Each margin tightening is an experiment. If completion rate stays high, tighten more. If it drops, back off and understand why.

### Graceful Degradation

Build systems that handle small errors smoothly:
- Missed gate: recover and continue to the next, don't get stuck.
- Position drift: smooth correction, not oscillation.
- Dropped perception frame: controller smooths over the gap.

A system that degrades gracefully under perturbation is more valuable than a theoretically perfect system that catastrophically fails on any error.

### Progressive Improvement

Match strategy to capability:

**Can't finish**: focus entirely on reliability. Slow way down. Understand why you're failing.

**Finish slowly**: analyze where time is lost. Corners? Straights? Gate approach? Attack the biggest time sink.

**Finish reliably**: tighten margins by 5–10%. If completion holds, tighten more. If it drops, understand why.

**Near the limit**: obsessively optimize the 2–3 hardest corners. The last 5% comes from the worst sections.

This progression maps directly onto the Phase Curriculum in `program.md`.

---

## 7. Breaking the Position Controller Ceiling

In position mode (`PositionNedYaw`), PX4's internal controller decides how fast to fly. It's tuned for stability, not racing. Key internal limits:

- **MPC_XY_VEL_MAX**: Maximum horizontal velocity (typically ~12 m/s). You cannot exceed this in position mode regardless of how far ahead you target.
- **Internal bank angle limits**: PX4 limits roll/pitch to maintain stability, typically ~35°. This limits cornering acceleration.
- **Deceleration toward target**: PX4 decelerates as it approaches the commanded position. This creates stop-and-go behavior near waypoints.

**The position controller, not physics, is your speed ceiling in position mode.** The drone's physics may support 20+ m/s and 60° bank, but PX4 won't let you access that in position mode. When lap times plateau despite parameter tuning, this is almost certainly why.

### Strategy 1: Dynamic Lookahead (easiest, stays in position mode)

Vary lookahead distance based on upcoming path curvature. On straights, push lookahead to 15–20m — PX4 will fly faster toward the distant target. Before corners, pull lookahead back to 3–5m — PX4 will decelerate naturally.

This exploits PX4's speed-vs-distance behavior without changing control mode. It's the simplest way to add speed management and can be implemented as a modification of the existing walk_along_path function.

Compute curvature of the path at the current position. Map curvature to lookahead distance. Tune the mapping experimentally.

### Strategy 2: Position + Velocity Feedforward (moderate difficulty)

MAVSDK provides `set_position_velocity_ned` which combines a position target with a velocity hint. The position target provides gate alignment (PX4 tracks it), while the velocity feedforward tells PX4 what speed you want.

```python
from mavsdk.offboard import PositionGlobalYaw  # may need VelocityBodyYaw
# The exact MAVSDK method varies — experiment with the API
```

**The challenge**: velocity and position targets can conflict. If you command "go to position X" but "at velocity V pointing away from X," PX4 must choose. Typically position dominates near the target and velocity dominates far from it. This makes it useful for speed on straights (position far ahead, velocity high) but tricky near gates (position close, velocity toward the gate).

**Start with velocity feedforward on straights only**, keeping pure position mode near gates. Gradually extend the velocity region as you characterize the behavior.

### Strategy 3: Attitude Control (highest ceiling, hardest)

`SET_ATTITUDE_TARGET` bypasses PX4's position and velocity controllers entirely. You command roll, pitch, yaw, and thrust directly. PX4 only runs the inner-loop attitude rate controller.

This removes all speed and bank angle limits. The performance ceiling is now the drone's physical limits, not PX4's software limits. But you must manage everything yourself:

- **Thrust for altitude**: At level flight, hover thrust ≈ 0.5 (varies by drone). During banking, increase thrust to maintain altitude: `T = hover_thrust / cos(bank_angle)`.
- **Pitch for speed**: Nose down = accelerate forward. Nose up = decelerate. The pitch angle directly controls horizontal acceleration.
- **Roll for turning**: Bank angle controls turn rate. Steeper bank = tighter turn but requires more thrust to maintain altitude.
- **Yaw for heading**: Keep yaw aligned with velocity vector for efficient flight.

**How to start with attitude control**:
1. Measure hover thrust: command level attitude (0° roll, 0° pitch), sweep thrust from 0.3 to 0.7, find the value where vertical velocity is ~0. This is your hover baseline.
2. Implement straight-line flight: command slight nose-down pitch (e.g., -5° to -15°) with hover thrust. Measure resulting speed.
3. Implement turns: command bank angles with increased thrust to maintain altitude.
4. Combine: pitch for speed management, roll for cornering, thrust for altitude.

### Strategy 4: Hybrid Control (pragmatic)

Use different control modes for different sections of the course:
- **Position mode near gates**: Precise alignment, PX4 handles the details.
- **Velocity or attitude mode on straights**: Higher speed, you control the throttle.
- **Smooth transitions**: Switch mode ~5m after passing a gate, switch back ~5m before the next approach point.

This gets most of the speed benefit with much less crash risk than full attitude control.

### Recommended Progression

Position mode → dynamic lookahead → position+velocity feedforward → hybrid control → full attitude control. Each step unlocks more speed but adds complexity and crash risk. Don't skip steps.
