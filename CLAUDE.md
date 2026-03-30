# Claude Code Project Configuration

## Execution Environment

This project runs on Windows 11 with WSL2 (Ubuntu 24.04). All Python execution must happen inside WSL2.

To run prepare.py (or any experiment):
```bash
wsl -d Ubuntu-24.04 -- bash -c "source /home/brian/drone-venv/bin/activate && cd /mnt/c/Users/brian/OneDrive/Desktop/AI_Grand_Prix && python3 prepare.py"
```

Key paths:
- **PX4-Autopilot**: `/home/brian/PX4-Autopilot` (WSL2 Linux filesystem)
- **Python venv**: `~/drone-venv` (must activate before any Python command)
- **Project code from WSL**: `/mnt/c/Users/brian/OneDrive/Desktop/AI_Grand_Prix`

## prepare.py Manages the Sim

prepare.py handles the full PX4 SITL lifecycle automatically: kill, launch (headless), connect, fly, teardown. You do NOT need to manually start or stop the simulator. Just run prepare.py.

## Files You Can Modify

- `pilot.py` — the autonomy stack
- `score.py` — the scoring/reward function
- `notebook.md` — lab notebook (append-only, survives git reverts)

## Files You Cannot Modify

- `prepare.py` — evaluation harness (infrastructure, maintained by human)
- `program.md` — experiment loop instructions
- `references/` — domain knowledge
- `visualize.py` — trajectory visualization (infrastructure)

## Experiment Workflow

Follow the loop in program.md exactly. Key reminders:
- Redirect output: `python3 prepare.py > run.log 2>&1`
- Before `git reset --hard`, stash notebook.md so discoveries survive
- Each experiment takes ~1 min (25s sim startup + 30s flight + teardown)
- The experiment loop runs INDEFINITELY until interrupted by the human
