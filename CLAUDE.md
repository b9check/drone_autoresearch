# Claude Code Project Configuration

## Execution Environment

This project runs on Windows 11 with WSL2 (Ubuntu 24.04). All Python execution must happen inside WSL2.

To run an experiment (combines WSL wrapper + redirect from program.md step 5):
```bash
wsl -d Ubuntu-24.04 -- bash -c "source /home/brian/drone-venv/bin/activate && cd /mnt/c/Users/brian/OneDrive/Desktop/AI_Grand_Prix && python3 -u prepare.py > run.log 2>&1"
```

To run any other Python command in the project:
```bash
wsl -d Ubuntu-24.04 -- bash -c "source /home/brian/drone-venv/bin/activate && cd /mnt/c/Users/brian/OneDrive/Desktop/AI_Grand_Prix && <command>"
```

Key paths:
- **PX4-Autopilot**: `/home/brian/PX4-Autopilot` (WSL2 Linux filesystem)
- **Python venv**: `~/drone-venv` (must activate before any Python command)
- **Project code from WSL**: `/mnt/c/Users/brian/OneDrive/Desktop/AI_Grand_Prix`

## prepare.py Manages the Sim

prepare.py handles the full PX4 SITL lifecycle automatically: kill, launch (headless), connect, fly, teardown. You do NOT need to manually start or stop the simulator. Just run prepare.py.

## What to Read

Follow the instructions in `program.md`. It defines the experiment loop, scoring, phases, and all operational procedures. This file (CLAUDE.md) only adds environment-specific configuration.
