"""
visualize.py — Trajectory visualization for autoresearch-drone.

NOT part of the autoresearch loop. Run manually to inspect runs.

Usage:
    python visualize.py                     # plot the latest run
    python visualize.py 42                  # plot run #42
    python visualize.py all                 # plot all runs overlaid
    python visualize.py last 10             # plot the last 10 runs overlaid
    python visualize.py progress            # score progression chart
"""

import sys
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

TRAJECTORY_DIR = "trajectories"


def load_run(filepath):
    """Load a single run's data."""
    data = np.load(filepath, allow_pickle=True)
    return {
        "trajectory": data["trajectory"],
        "gate_positions": data["gate_positions"],
        "gate_normals": data["gate_normals"],
        "score": float(data["score"]),
        "lap_time": float(data["lap_time"]),
        "gates_passed": int(data["gates_passed"]),
        "total_gates": int(data["total_gates"]),
        "crashed": bool(data["crashed"]),
        "filepath": filepath,
    }


def get_run_files():
    """Get sorted list of trajectory files."""
    pattern = os.path.join(TRAJECTORY_DIR, "run_*.npz")
    files = sorted(glob.glob(pattern))
    return files


def draw_gate(ax, position, normal, width=2.0, height=2.0, color="orange", alpha=0.3):
    """Draw a gate as a rectangular frame in 3D."""
    # Create gate corners in the gate's local frame
    # The gate faces along 'normal', so we need two perpendicular vectors
    up = np.array([0, 0, -1])  # NED: up is -Z
    if abs(np.dot(normal, up)) > 0.9:
        up = np.array([1, 0, 0])

    right = np.cross(normal, up)
    right = right / np.linalg.norm(right)
    up = np.cross(right, normal)
    up = up / np.linalg.norm(up)

    hw = width / 2
    hh = height / 2

    # Four corners of the gate opening
    corners = [
        position + right * hw + up * hh,
        position - right * hw + up * hh,
        position - right * hw - up * hh,
        position + right * hw - up * hh,
    ]

    # Draw the frame as 4 lines
    for i in range(4):
        j = (i + 1) % 4
        ax.plot(
            [corners[i][0], corners[j][0]],
            [corners[i][1], corners[j][1]],
            [-corners[i][2], -corners[j][2]],  # flip Z for display (up = positive)
            color=color,
            linewidth=3,
            alpha=0.8,
        )

    # Draw a semi-transparent fill
    verts = [[(c[0], c[1], -c[2]) for c in corners]]
    poly = Poly3DCollection(verts, alpha=alpha, facecolor=color, edgecolor=color)
    ax.add_collection3d(poly)

    # Gate number label
    label_pos = position + up * (hh + 0.3)
    return label_pos


def plot_single_run(run_data, ax=None, color="dodgerblue", label=None, show_gates=True):
    """Plot a single run's trajectory with gates."""
    if ax is None:
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection="3d")
        standalone = True
    else:
        standalone = False

    traj = run_data["trajectory"]

    if len(traj) > 0:
        x = traj[:, 1]
        y = traj[:, 2]
        z = -traj[:, 3]  # flip Z so up is positive in the plot

        # Color by time
        if label:
            ax.plot(x, y, z, color=color, linewidth=1.5, alpha=0.7, label=label)
        else:
            ax.plot(x, y, z, color=color, linewidth=1.5, alpha=0.7)

        # Start and end markers
        ax.scatter(*[x[0]], *[y[0]], *[z[0]], color="green", s=100, marker="^", zorder=5)
        if run_data["crashed"]:
            ax.scatter(
                *[x[-1]], *[y[-1]], *[z[-1]],
                color="red", s=100, marker="x", zorder=5
            )

    # Draw gates
    if show_gates:
        for i, (pos, normal) in enumerate(
            zip(run_data["gate_positions"], run_data["gate_normals"])
        ):
            label_pos = draw_gate(ax, pos, normal)
            ax.text(
                label_pos[0], label_pos[1], -label_pos[2],
                f"G{i+1}", fontsize=8, ha="center", color="darkorange",
                fontweight="bold",
            )

    if standalone:
        status = "CRASHED" if run_data["crashed"] else "OK"
        title = (
            f"Score: {run_data['score']:.1f} | "
            f"Lap: {run_data['lap_time']:.1f}s | "
            f"Gates: {run_data['gates_passed']}/{run_data['total_gates']} | "
            f"{status}"
        )
        ax.set_title(title, fontsize=12)

    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_zlabel("Altitude (m)")

    return ax


def plot_latest():
    """Plot the most recent run."""
    files = get_run_files()
    if not files:
        print("No trajectory files found.")
        return

    run = load_run(files[-1])
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection="3d")
    plot_single_run(run, ax=ax)
    ax.set_title(
        f"Run #{len(files)} — "
        f"Score: {run['score']:.1f} | "
        f"Lap: {run['lap_time']:.1f}s | "
        f"Gates: {run['gates_passed']}/{run['total_gates']} | "
        f"{'CRASHED' if run['crashed'] else 'OK'}",
        fontsize=12,
    )
    plt.tight_layout()
    plt.savefig("latest_run.png", dpi=150, bbox_inches="tight")
    print(f"Saved: latest_run.png")
    plt.show()


def plot_run_number(num):
    """Plot a specific run by number."""
    files = get_run_files()
    if num < 1 or num > len(files):
        print(f"Run #{num} not found. Have {len(files)} runs.")
        return

    run = load_run(files[num - 1])
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection="3d")
    plot_single_run(run, ax=ax)
    ax.set_title(
        f"Run #{num} — "
        f"Score: {run['score']:.1f} | "
        f"Lap: {run['lap_time']:.1f}s | "
        f"Gates: {run['gates_passed']}/{run['total_gates']} | "
        f"{'CRASHED' if run['crashed'] else 'OK'}",
        fontsize=12,
    )
    plt.tight_layout()
    plt.savefig(f"run_{num:04d}.png", dpi=150, bbox_inches="tight")
    print(f"Saved: run_{num:04d}.png")
    plt.show()


def plot_overlay(files):
    """Plot multiple runs overlaid, colored by score."""
    runs = [load_run(f) for f in files]
    scores = [r["score"] for r in runs]
    valid_scores = [s for s in scores if s < 9999]
    min_score = min(valid_scores) if valid_scores else 0
    max_score = max(valid_scores) if valid_scores else 1

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection="3d")

    cmap = plt.cm.RdYlGn_r  # red = bad, green = good
    for i, run in enumerate(runs):
        if run["score"] >= 9999:
            color = "red"
            alpha = 0.2
        else:
            norm_score = (run["score"] - min_score) / max(max_score - min_score, 1)
            color = cmap(norm_score)
            alpha = 0.5

        run_num = int(os.path.basename(files[i]).split("_")[1].split(".")[0])
        plot_single_run(
            run, ax=ax, color=color,
            label=f"#{run_num} ({run['score']:.0f})",
            show_gates=(i == 0),  # only draw gates once
        )

    ax.legend(loc="upper left", fontsize=7, ncol=2)
    ax.set_title(f"{len(runs)} runs overlaid (green=better, red=worse)", fontsize=12)
    plt.tight_layout()
    plt.savefig("overlay.png", dpi=150, bbox_inches="tight")
    print(f"Saved: overlay.png")
    plt.show()


def plot_progress():
    """Plot score progression over experiments."""
    files = get_run_files()
    if not files:
        print("No trajectory files found.")
        return

    runs = [load_run(f) for f in files]
    scores = [r["score"] for r in runs]
    gates = [r["gates_passed"] for r in runs]
    lap_times = [r["lap_time"] for r in runs]
    crashed = [r["crashed"] for r in runs]

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    # Score over time
    ax = axes[0]
    colors = ["red" if c else "dodgerblue" for c in crashed]
    ax.scatter(range(1, len(scores) + 1), scores, c=colors, s=30, alpha=0.7)
    # Running best
    best = []
    curr_best = 9999
    for s in scores:
        curr_best = min(curr_best, s)
        best.append(curr_best)
    ax.plot(range(1, len(best) + 1), best, color="green", linewidth=2, label="Best score")
    ax.set_ylabel("Score (lower=better)")
    ax.set_ylim(bottom=0, top=min(max(scores) * 1.1, 10000))
    ax.legend()
    ax.set_title("Experiment Progress")
    ax.grid(True, alpha=0.3)

    # Gates passed
    ax = axes[1]
    ax.bar(range(1, len(gates) + 1), gates, color=colors, alpha=0.7)
    ax.set_ylabel("Gates Passed")
    total = runs[0]["total_gates"]
    ax.axhline(y=total, color="green", linestyle="--", alpha=0.5, label=f"All {total} gates")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Lap time (0 = DNF)
    ax = axes[2]
    valid_times = [(i + 1, t) for i, t in enumerate(lap_times) if t > 0]
    if valid_times:
        ax.scatter(
            [v[0] for v in valid_times],
            [v[1] for v in valid_times],
            color="dodgerblue", s=30, alpha=0.7,
        )
        # Running best lap
        best_lap = []
        curr_best_lap = 9999
        for _, t in valid_times:
            curr_best_lap = min(curr_best_lap, t)
            best_lap.append(curr_best_lap)
        ax.plot(
            [v[0] for v in valid_times], best_lap,
            color="green", linewidth=2, label="Best lap",
        )
        ax.legend()
    ax.set_ylabel("Lap Time (s)")
    ax.set_xlabel("Experiment #")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("progress.png", dpi=150, bbox_inches="tight")
    print(f"Saved: progress.png")
    plt.show()


if __name__ == "__main__":
    args = sys.argv[1:]

    if not args:
        plot_latest()
    elif args[0] == "progress":
        plot_progress()
    elif args[0] == "all":
        plot_overlay(get_run_files())
    elif args[0] == "last":
        n = int(args[1]) if len(args) > 1 else 10
        files = get_run_files()
        plot_overlay(files[-n:])
    elif args[0].isdigit():
        plot_run_number(int(args[0]))
    else:
        print(__doc__)
