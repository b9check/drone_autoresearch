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
- Each experiment takes ~1 min (25s sim startup + 30s flight + teardown)
- The experiment loop runs INDEFINITELY until interrupted by the human

## Time Discipline

**Getting stuck is unacceptable.** The experiment loop must keep moving.

- **Thinking between experiments: MAX 60 seconds.** Form your hypothesis, edit the code, commit, and run. Do not over-analyze or second-guess. Quick iterations beat careful planning.
- **Sim run timeout: 3 minutes.** If `prepare.py` hasn't returned output in 3 minutes, something is wrong. Kill it and retry.
- **Analysis after a run: MAX 30 seconds.** Read the results, log them, decide keep/discard, move on.
- **If stuck on a problem for 3+ failed experiments:** Step back, try a completely different approach, or simplify. Do not keep tweaking the same parameter.

## Discarding Failed Experiments

Use `git reset --hard HEAD~1` to discard the latest experiment, as program.md instructs. Only ever reset ONE commit back. Never reset to an arbitrary old commit hash — that risks reverting infrastructure files.

## Transient Sim Failures

PX4 SITL occasionally fails with `COMMAND_DENIED` on arming or bind errors. These are transient — just retry the run. Do not change code in response to these. If 3 consecutive runs fail with sim errors, stop and leave a note in results.tsv.
