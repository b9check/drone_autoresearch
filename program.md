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

LOOP FOREVER:

1. **Examine state**: Look at the current code in `pilot.py` and `score.py`, the git log, `results.tsv`, and `notebook.md` to understand where you are.
2. **Form a hypothesis**: Based on your reading of the references, the notebook, previous results, and the current code, decide what to try. Write a one-line description of the experiment.
3. **Modify the code**: Edit `pilot.py` (and optionally `score.py`) to implement your idea.
4. **Git commit**: Commit the changes with a message describing the experiment.
5. **Run the experiment**: `python prepare.py > run.log 2>&1`
6. **Read results**: Extract the raw data and score from `run.log`. Use: `grep "^score:\|^lap_time:\|^gates_passed:\|^crashed:" run.log`. If the output is empty or malformed, `tail -n 50 run.log` to diagnose.
7. **Log results**: Append to `results.tsv`.
8. **Update notebook**: If you learned something generalizable about the sim, the drone, the track, or the control problem, write it in `notebook.md`. See "Lab Notebook" below.
9. **Keep or discard**:
   - If the score improved (lower): **keep**. The branch advances.
   - If the score is equal or worse: **discard**. `git reset --hard` to the previous commit. (Note: `notebook.md` is never reverted — discoveries persist even from failed experiments. Stash it before reset if needed.)
   - If the run crashed: use judgment. Fix trivial bugs and re-run. If the idea is fundamentally broken, discard and move on.
10. **Repeat**.

### results.tsv format

Tab-separated, 6 columns:

```
commit	score	lap_time	gates	status	description
```

- `commit`: short git hash (7 chars)
- `score`: composite score from score.py (lower is better). Use 9999.0 for crashes.
- `lap_time`: raw lap time in seconds. Use 0.0 for DNF/crash.
- `gates`: gates passed / total gates (e.g. "7/10")
- `status`: `keep`, `discard`, or `crash`
- `description`: short text describing what was tried

Example:
```
commit	score	lap_time	gates	status	description
a1b2c3d	9999.000	0.0	0/10	keep	baseline — hover only
b2c3d4e	850.000	0.0	3/10	keep	basic waypoint following to first 3 gates
c3d4e5f	420.000	95.3	10/10	keep	complete course, conservative speed
d4e5f6g	415.000	88.7	10/10	keep	increased approach speed
e5f6g7h	9999.000	0.0	6/10	discard	aggressive cornering — crashed at gate 7
f6g7h8i	390.000	82.1	10/10	keep	moderate cornering with speed reduction before tight turns
```

## Phase Curriculum

You have broad freedom to restructure and rewrite the code however you want. However, the following phased approach is **strongly recommended** as a curriculum. You should advance phases when you meet the criteria, not before. Premature advancement wastes experiments.

### Phase A — Guidance and Planning

**Goal**: Achieve a complete lap (all gates passed, no crash) with the simplest possible perception. Use odometry data for state estimation. If gate positions are available or can be hardcoded from initial exploration, do that. Focus entirely on: how to plan a path through gates, how to generate smooth and fast setpoints, and how to command the drone.

**Advance when**: You can reliably complete the course (>90% completion rate across 3 consecutive runs) with a reasonable lap time.

**What to experiment with**:
- Waypoint following vs. trajectory optimization
- Speed profiles: constant speed vs. acceleration/deceleration around gates
- Gate approach geometry: straight-on vs. cutting corners
- SET_ATTITUDE_TARGET vs. SET_POSITION_TARGET_LOCAL_NED (which control mode works better?)
- Setpoint lookahead distance
- Cornering strategy: slow down and turn vs. bank and maintain speed

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

**Goal**: Minimize lap time. Everything is fair game. You have a working, reliable system from Phases A and B. Now push it to the limit.

**What to experiment with**:
- Aggressive trajectory optimization (cutting gates closer, faster approach speeds)
- Predictive path planning (plan multiple gates ahead, not just the next one)
- Dynamic speed adaptation based on upcoming track geometry
- End-to-end approaches (if they can beat your modular pipeline)
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

When you `git reset --hard` after a failed experiment, `notebook.md` must not be lost. Before any reset, ensure the notebook is preserved (e.g., stash it, copy it out, or keep it untracked by git). The notebook is append-only — never delete previous entries, only add new ones.

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

You could be running while the human sleeps. If each experiment takes ~3 minutes (sim run + overhead), you can run ~20/hour, ~160 overnight. The human wakes up to a results log and (hopefully) a faster drone.

## Timeout and Error Handling

- **Normal run**: Should complete in under 8 minutes (the sim's max run duration). Allow 10 minutes total including startup and teardown.
- **If a run exceeds 10 minutes**: Kill it, log as crash, discard and revert.
- **If the sim crashes**: Attempt to restart it. If you can't restart after 3 attempts, stop the loop and leave a note in results.tsv explaining the failure. The human will fix the sim when they return.
- **If your code crashes**: Read the traceback. If it's a trivial bug (typo, import error, off-by-one), fix it and re-run without counting as a separate experiment. If the idea itself is broken, log as crash and move on.
- **If score.py crashes**: Fix it immediately. A broken score function halts the entire loop.
