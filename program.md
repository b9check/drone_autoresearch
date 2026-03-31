# autoresearch-drone

Autonomous research loop for the AI Grand Prix — Anduril's global autonomous drone racing competition. An AI agent iterates on an autonomy stack to minimize lap time through a gate course, using the autoresearch pattern pioneered by Karpathy.

## Project Context

The AI Grand Prix virtual qualifier requires fully autonomous drone software that navigates a gate course using onboard sensors only. All teams use identical simulated drones. The only differentiator is the code. The simulator communicates via MAVLink v2 over UDP through MAVSDK. The drone is controlled by sending attitude targets or local NED position/velocity setpoints — the sim runs an inner-loop stabilized flight controller.

**Your job is to write the fastest, most reliable autonomous racing pilot possible.**

## Repository Structure

```
prepare.py              # FROZEN — sim launcher, MAVLink bridge, raw metric logger
score.py                # MUTABLE — your scoring/reward function
pilot.py                # MUTABLE — the full autonomy stack
notebook.md             # MUTABLE — your lab notebook. Write discoveries here.
program.md              # READ-ONLY — you are reading this now
references/             # READ-ONLY — domain knowledge and technical references
  mavlink_interface.md  # MAVLink message specs, MAVSDK usage, control modes
  drone_racing.md       # Physics, trajectories, and racing strategy
results.tsv             # Experiment log (do not commit)
run.log                 # Latest run output (do not commit)
```

### What you CAN modify

- `pilot.py` — this is your primary workspace. It contains the full autonomy pipeline: perception, planning, guidance, and command generation. Everything is fair game. You may restructure it entirely, add classes, remove them, change the approach wholesale.
- `score.py` — this defines how raw run data (lap time, gates passed, crashes) is converted into a single scalar score for the experiment loop. You may change the weighting, add penalty terms, reshape the reward however you see fit.
- `notebook.md` — your lab notebook. Write down discoveries about the sim, the drone, the track, and anything else you learn from experiments. The references tell you the theory; the notebook records the reality. See "Lab Notebook" section below.

### What you CANNOT modify

- `prepare.py` — the sim interface, MAVLink bridge, and raw data logger. This is the ground truth.
- `program.md` — these instructions.
- Anything in `references/` — these are curated domain knowledge written by the human. Do not overwrite theory with conclusions from a handful of experiments.
- You cannot change the simulator, the drone physics, or the MAVLink interface.
- You cannot add human input during a run. The drone must be fully autonomous.

### What you SHOULD read before experimenting

Read these files for full context before your first experiment:
- This file (`program.md`) — you're doing that now.
- `prepare.py` — understand the sim interface, what raw data is logged, how runs are launched and timed.
- `pilot.py` — understand the current baseline implementation.
- `score.py` — understand the current scoring function.
- `notebook.md` — read any previous discoveries before planning your next experiment.
- `references/mavlink_interface.md` — the MAVLink messages available to you, control modes, and timing constraints.
- `references/drone_racing.md` — physics, trajectory planning, speed profiling, and racing strategy.

## The Simulator Interface

Key facts from the technical specification:

- **Transport**: MAVLink v2 over UDP via MAVSDK
- **Physics rate**: 120 Hz
- **Recommended command rate**: 50–120 Hz
- **Minimum heartbeat rate**: 2 Hz
- **Maximum run duration**: 8 minutes
- **Coordinate system**: Local Cartesian NED. No GPS. No global position.

### Telemetry you receive

| Message | Content |
|---------|---------|
| HEARTBEAT | Connection status |
| ATTITUDE | Roll, pitch, yaw, angular rates |
| HIGHRES_IMU | Accelerometer, gyroscope, magnetometer |
| ODOMETRY | Local position and velocity estimates |
| TIMESYNC | Simulator time synchronization |

### Commands you send

| Message | Use case |
|---------|----------|
| SET_ATTITUDE_TARGET | Direct attitude control (roll/pitch/yaw targets + thrust) |
| SET_POSITION_TARGET_LOCAL_NED | Position/velocity setpoints in local NED frame |
| HEARTBEAT | Keep-alive (≥2 Hz) |

### Vision stream

A forward-facing FPV camera feed is available. Detailed vision stream parameters are specified separately. Expect approximately a 12MP wide-angle camera. Gates are visually distinctive from the environment.

### What the sim handles for you

The simulator runs an inner-loop stabilized flight controller. When you send SET_ATTITUDE_TARGET, the sim's flight controller tracks those attitude targets and handles motor mixing. You are NOT writing rate-loop PIDs or ESC commands. You are writing the outer loop: what attitude/position targets to command and when.

## Setup

To set up a new experiment run, work with the user to:

1. **Agree on a run tag**: propose a tag based on today's date (e.g. `mar29`). The branch `autoresearch/<tag>` must not already exist.
2. **Create the branch**: `git checkout -b autoresearch/<tag>` from current master.
3. **Read all in-scope files**: Read the files listed above under "What you SHOULD read."
4. **Verify sim connection**: Confirm the simulator is running and MAVLink connection can be established.
5. **Initialize results.tsv**: Create `results.tsv` with just the header row.
6. **Confirm and go**: Confirm setup looks good with the user.

Once you get confirmation, kick off experimentation.

## Scoring

### Ground truth (in prepare.py — frozen)

Every run produces raw data:
- `lap_time`: wall-clock time from start gate to finish gate (seconds). 0.0 if DNF.
- `gates_passed`: number of gates successfully passed (integer).
- `total_gates`: total number of gates on the course (integer).
- `crashed`: whether the drone crashed (boolean).
- `max_run_time`: 8 minutes (480 seconds).

### Your score function (in score.py — mutable)

`score.py` takes the raw data and returns a single float. **Lower is better.**

The score function is yours to design. It must satisfy one hard constraint: **the global optimum of your score function must correspond to the fastest possible complete lap with all gates passed and no crash.** Beyond that, you decide how to weight partial completions, crashes, gate misses, etc.

Think carefully about reward shaping. A good score function makes the optimization landscape smooth — small improvements in the pilot should produce small improvements in score. A bad score function has cliffs and plateaus that make it hard to hill-climb.

Some considerations:
- A drone that passes 7/10 gates and then crashes is better than one that crashes at gate 2. Your score should reflect this.
- A drone that completes the course in 60 seconds is better than one that completes it in 120 seconds. But a drone that completes the course in 120 seconds is MUCH better than one that DNFs.
- Finishing the course reliably matters more than finishing it fast, especially in early experiments. Speed comes from reliability pushed to its limits, not from reckless flying.
- You may change the score function at any time. If you notice experiments plateauing, consider whether your score function is the bottleneck.

### Logging the ground truth

`prepare.py` always logs the raw data regardless of what `score.py` does. This means you can always audit whether your score function is aligned with the true objective. If you see your composite score improving but lap times getting worse, your score function has diverged — fix it.

## The Experiment Loop

Each experiment is one complete run of the drone through the course.

### Handling Sim Variance

The sim is nominally deterministic, but in practice there is ~0.5s run-to-run variance from timing jitter and async scheduling. This matters for keep/discard decisions:

- **For improvements > 1.0s**: A single run is sufficient. The signal is well above the noise.
- **For improvements of 0.2–1.0s**: Run twice and take the better of two. This filters lucky outliers.
- **For improvements < 0.2s**: Be skeptical. If you need three runs to tell whether an experiment helped, the improvement is marginal and probably not worth the added complexity. Consider whether the change simplifies the code instead.
- **Never keep a change based on a single lucky run** that's substantially better than expected. If you go from 10.5s to 9.5s on one run, verify it with a second run before committing.

### Structural vs. Parameter Experiments

Recognize which type of experiment you're running and use the appropriate method:

**Structural changes** (new algorithm, new waypoint strategy, different control mode): Use the standard one-change-per-experiment loop. Change one thing, test it, keep or discard.

**Parameter tuning** (lookahead distance, speed threshold, approach offset): Don't burn one experiment per value. Write a parameter sweep within a single experiment — run the course N times with different values, pick the best, and commit with that value. A sweep of 5 values in one experiment is faster and more informative than 5 sequential experiments.

**Single-variable discipline**: Never combine two unrelated changes in one experiment. If you change approach distance AND speed limit simultaneously, you can't tell which one caused the result. The exception is when two changes are mechanically coupled (e.g., reducing approach distance requires reducing switch radius proportionally).

LOOP FOREVER:

1. **Examine state**: Look at the current code in `pilot.py` and `score.py`, the git log, `results.tsv`, and `notebook.md` to understand where you are.
2. **Form a hypothesis**: Based on your reading of the references, the notebook, previous results, and the current code, decide what to try. Write a one-line description of the experiment.
3. **Modify the code**: Edit `pilot.py` (and optionally `score.py`) to implement your idea.
4. **Git commit**: Commit the changes with a message describing the experiment.
5. **Run the experiment**: `python3 -u prepare.py > run.log 2>&1` (the `-u` flag disables output buffering so run.log is readable in real-time)
6. **Read results**: Extract the raw data and score from `run.log`. Use: `grep -a "^score:\|^lap_time:\|^gates_passed:\|^crashed:" run.log` (the `-a` flag handles binary data that MAVSDK sometimes writes to stderr). If the output is empty or malformed, `tail -n 50 run.log` to diagnose.
7. **Log results**: Append to `results.tsv`.
8. **Update notebook**: If you learned something generalizable about the sim, the drone, the track, or the control problem, write it in `notebook.md`. See "Lab Notebook" below.
9. **Keep or discard**:
   - If the score improved (lower): **keep**. The branch advances.
   - If the score is equal or worse: **discard**. Revert only the files you changed:
     ```
     git checkout HEAD~1 -- pilot.py score.py && git commit -m "discard: <reason>"
     ```
     This is surgical — `notebook.md`, `results.tsv`, and all other files are untouched. If you only changed pilot.py, you can omit score.py. Do NOT use `git reset --hard` as it destroys uncommitted changes to other files.
   - If the run crashed: use judgment. Fix trivial bugs and re-run. If the idea is fundamentally broken, discard and move on.
10. **Repeat**.

### results.tsv format

Tab-separated, 7 columns:

```
commit	score	lap_time	gates	min_margin	status	description
```

- `commit`: short git hash (7 chars)
- `score`: composite score from score.py (lower is better). Use 9999.0 for crashes.
- `lap_time`: raw lap time in seconds. Use 0.0 for DNF/crash.
- `gates`: gates passed / total gates (e.g. "7/10")
- `min_margin`: closest gate passage distance from center (meters). This is your reliability signal — if min_margin is approaching GATE_PASS_RADIUS (1.5m), you're one bad run from a missed gate. Use 0.0 for crashes.
- `status`: `keep`, `discard`, or `crash`
- `description`: short text describing what was tried

Example:
```
commit	score	lap_time	gates	min_margin	status	description
a1b2c3d	9999.000	0.0	0/10	0.0	keep	baseline — hover only
b2c3d4e	850.000	0.0	3/10	0.45	keep	basic waypoint following to first 3 gates
c3d4e5f	420.000	95.3	10/10	0.31	keep	complete course, conservative speed
d4e5f6g	415.000	88.7	10/10	0.52	keep	increased approach speed
e5f6g7h	9999.000	0.0	6/10	1.29	discard	aggressive cornering — crashed at gate 7
f6g7h8i	390.000	82.1	10/10	0.38	keep	moderate cornering with speed reduction before tight turns
```

## Phase Curriculum

You have broad freedom to restructure and rewrite the code however you want. However, the following phased approach is **strongly recommended** as a curriculum. You should advance phases when you meet the criteria, not before. Premature advancement wastes experiments.

### Phase A — Guidance and Planning

**Goal**: Achieve a complete lap (all gates passed, no crash) with the simplest possible perception. Use odometry data for state estimation. If gate positions are available or can be hardcoded from initial exploration, do that. Focus entirely on: how to plan a path through gates, how to generate smooth and fast setpoints, and how to command the drone.

**Advance when**: You can reliably complete the course (>90% completion rate across 3 consecutive runs) AND lap times have stopped improving with parameter tuning in position mode (typically 2–3 consecutive experiments with <0.3s improvement).

**What to experiment with**:
- Waypoint following → approach/through gate alignment → multi-waypoint lookahead (see `drone_racing.md` trajectory levels)
- Gate approach geometry: straight-on vs. cutting corners
- Lookahead distance and its interaction with gate alignment (see `drone_racing.md` Section 3)
- Per-gate parameter tuning (some gates need tighter alignment than others)

### Phase A+ — Breaking the Position Controller Ceiling

**Goal**: Add explicit speed management. Phase A uses `PositionNedYaw` which lets PX4 decide how fast to fly. This phase takes control of speed while keeping the path planning from Phase A.

**Advance when**: Lap times improve measurably over pure position mode, with maintained reliability.

**What to experiment with** (see `drone_racing.md` Section 7 for details):
- Dynamic lookahead: vary lookahead distance based on upcoming curvature (far on straights = faster, short before corners = slower)
- `set_position_velocity_ned`: position for alignment + velocity feedforward for speed
- Curvature-based speed profiling with the forward-backward algorithm
- Hybrid control: velocity/attitude on straights, position near gates

### Phase B — Perception Integration

**Goal**: Replace any hardcoded or simplified perception with vision-based gate detection. The drone should detect gates from the FPV camera and navigate based on what it sees, not where it knows gates are.

**Advance when**: The drone can complete the course using vision-based gate detection with comparable reliability to Phase A.

**What to experiment with**:
- Gate detection algorithms (color filtering, contour detection, learned detectors)
- Gate distance/angle estimation from monocular vision
- Fusing vision-based gate detection with IMU/odometry state estimates
- Handling missed detections and false positives gracefully
- Anticipating next gates before they're visible

### Phase C — Full Optimization

**Goal**: Minimize lap time. Everything is fair game. You have a working, reliable system from previous phases. Now push it to the limit.

**What to experiment with**:
- **Attitude control** (`SET_ATTITUDE_TARGET`): Direct roll/pitch/thrust commands bypass PX4's conservative position controller entirely. Highest performance ceiling but requires you to manage altitude, speed, and trajectory simultaneously. Start by measuring hover thrust (see `mavlink_interface.md`), then implement a simple attitude controller for straights, then extend to corners.
- Aggressive trajectory optimization (cutting gates closer, faster approach speeds)
- Predictive path planning (plan multiple gates ahead, not just the next one)
- Offline trajectory optimization (compute the time-optimal path once before the run, track it during the race)
- Aerodynamic exploitation (using momentum, gravity-assisted dives, banking efficiency)
- Any radical restructuring of the architecture, as long as it improves the score

## Lab Notebook

`notebook.md` is your persistent memory across experiments. Unlike `pilot.py` which gets reverted on failed experiments, the notebook always moves forward. Discoveries from failed experiments are often the most valuable — "banking past 45° crashes the drone" is critical knowledge even if the experiment that produced it was discarded.

### What to write

Record **empirical facts about the sim and drone** — things that aren't in the references because they're specific to this simulator, this track, this drone model. Examples:

- Measured properties: "Gates are ~1.5m wide, orange, rectangular. Gate 4 is a sharp left turn of approximately 90°."
- Sim behavior: "SET_ATTITUDE_TARGET has ~2 frame latency. SET_POSITION_TARGET_LOCAL_NED has built-in smoothing that limits max acceleration."
- Drone limits: "Max stable bank angle is approximately 40°. Beyond that, altitude drops rapidly. Thrust-to-weight ratio appears to be ~2:1."
- Track knowledge: "The course has 10 gates. Gates 1–3 are a gentle left curve. Gates 4–5 require a hard right. Gate 8 is elevated ~2m above gate 7."
- Odometry quality: "Odometry drift is <0.1m over a full lap. Position estimates are reliable enough for Phase A planning."
- What works: "Slowing to 3 m/s before turns tighter than 60° eliminates crashes. Lookahead of 3m gives best speed/stability tradeoff."
- What doesn't: "Pure proportional navigation toward gates causes oscillation at close range. Need a blended approach with position-based guidance inside 5m."

### What NOT to write

- Don't copy-paste code into the notebook. The code lives in `pilot.py`.
- Don't write vague impressions. "The drone feels slow" is useless. "Max straight-line speed before instability is ~8 m/s" is useful.
- Don't duplicate results.tsv. The notebook is for generalizable knowledge, not per-experiment metrics.

### When to read it

Read `notebook.md` before planning each experiment. Your past self already discovered things that should inform your next hypothesis. Don't rediscover them.

### Notebook survives reverts

The surgical discard method (`git checkout HEAD~1 -- pilot.py`) does not touch `notebook.md`, so it automatically survives. The notebook is append-only — never delete previous entries, only add new ones.

## Design Philosophy

### Simplicity criterion

Borrowed from Karpathy's autoresearch and adapted: **all else being equal, simpler is better.** A small lap time improvement that adds 50 lines of fragile code is probably not worth it. A lap time improvement from deleting code? Definitely keep. An equal lap time but much simpler code? Keep — simpler code is easier to improve in future experiments.

Complexity is debt. Every line of code is a line that could have a bug, and bugs mean crashes, and crashes mean wasted experiments.

### Reliability over speed

Especially in Phases A and B: a drone that finishes the course in 120 seconds beats a drone that DNFs at gate 8 in 40 seconds. You cannot optimize lap time if you cannot complete laps. Get reliability first, then squeeze speed.

### Use the references

Before trying something novel, check the references. `drone_racing.md` contains principles that took decades of human research to develop. Don't waste experiments rediscovering that you need to decelerate before tight corners. Use the references as a starting point and improve from there.

### Think before each experiment

Don't just randomly perturb numbers. Before each experiment, articulate a hypothesis: "I believe that increasing the lookahead distance from 2m to 4m will allow smoother trajectories and reduce lap time by approximately 5%, because the current drone oscillates when targets are too close." This discipline prevents wasted experiments and makes the results.tsv log more useful.

### Learn from failures

When an experiment fails (crash or worse score), don't just discard and move on. Spend a moment understanding WHY it failed. The failure mode often points directly at the next good experiment. If aggressive cornering caused a crash, the next experiment should be "slightly less aggressive cornering" or "add a safety margin on corner entry speed," not an unrelated change.

## Time Discipline

**Getting stuck is unacceptable.** The experiment loop must keep moving.

- **Thinking between experiments: MAX 60 seconds.** Form your hypothesis, edit the code, commit, and run. Do not over-analyze or second-guess. Quick iterations beat careful planning.
- **Sim run timeout: 2 minutes.** If `prepare.py` hasn't returned output in 2 minutes, something is wrong. Kill it and retry. (The harness auto-aborts stuck runs in 30s, so this is a last resort.)
- **Analysis after a run: MAX 30 seconds.** Read the results, log them, decide keep/discard, move on.
- **If stuck on a problem for 3+ failed experiments:** Step back and diagnose. The question isn't "what parameter to try next" — it's "am I hitting a fundamental ceiling?"
  - **Position controller ceiling**: If you're in position mode and lap times have plateaued despite trying different lookahead values, path shapes, and approach geometries, you've likely hit PX4's internal speed limits (~12 m/s horizontal, conservative bank angles). The fix is not more parameter tuning — it's switching to velocity control or attitude control. See `drone_racing.md` Section 7.
  - **Gate alignment ceiling**: If gate 8 (or the hardest gate) keeps failing at similar margins regardless of approach strategy, the issue is geometric — the approach/through waypoint pattern can only do so much. Consider corner-cutting offsets or per-gate parameter tuning.
  - **Architecture ceiling**: If every incremental tweak gives < 0.3s improvement, the current code structure may be maxed out. Consider a wholesale restructure rather than continued tuning.

## NEVER STOP

Once the experiment loop has begun, do NOT pause to ask the human if you should continue. Do NOT ask "should I keep going?" or "is this a good stopping point?" The human may be asleep, away from the computer, and expects you to continue working **indefinitely** until manually stopped.

If you run out of ideas:
- Re-read the reference materials for new angles
- Re-read `notebook.md` — your past discoveries often suggest the next experiment
- Look at the results.tsv for patterns (what kinds of changes helped most?)
- Try combining two previous near-misses
- Try the opposite of your last few experiments
- Try simplifying instead of adding complexity
- Revisit your score function — maybe the optimization landscape needs reshaping
- Try a radically different approach to planning or perception

The loop runs until the human interrupts you. Period.

You could be running while the human sleeps. If each experiment takes ~1 minute (sim startup + flight + teardown), you can run ~60/hour, ~480 overnight. The human wakes up to a results log and (hopefully) a faster drone.

## Timeout and Error Handling

prepare.py handles most failure modes automatically:
- **Arm retries**: 3 attempts with 3s backoff before giving up on COMMAND_DENIED.
- **Top-level retry**: If the entire run fails with a transient error (arm_failed, connection_timeout, etc.), prepare.py retries up to 3 times.
- **Port cleanup**: `fuser -k 14540/udp` runs during sim teardown to prevent bind errors.
- **Gate progress timeout**: If no new gate is passed in 30s, the run aborts early instead of waiting 8 minutes.

For failures that prepare.py can't handle:
- **If prepare.py hasn't returned in 2 minutes**: Something is wrong. Kill it and retry. (This is a last resort — the harness auto-aborts stuck runs.)
- **If 3 consecutive prepare.py invocations fail**: The sim environment is broken. Stop the loop and leave a note in results.tsv. The human will fix it.
- **If your code crashes**: Read the traceback (full stack trace is printed to run.log). If it's a trivial bug (typo, import error, off-by-one), fix it and re-run without counting as a separate experiment. If the idea itself is broken, log as crash and move on.
- **If score.py crashes**: Fix it immediately. A broken score function halts the entire loop.
- **NEVER run prepare.py in background mode.** Background runs create orphaned PX4/Gazebo processes that block future runs with port conflicts. Always run in foreground and wait for completion.
