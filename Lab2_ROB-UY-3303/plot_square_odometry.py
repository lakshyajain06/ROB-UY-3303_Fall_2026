#!/usr/bin/env python3
"""Create a top-down pose plot from the newest Assignment 02 odometry log.

Each recorded odometry position is drawn as a small circle with a short line
that shows the robot's orientation at that pose.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib

# Render directly to a file; this avoids requiring a desktop GUI backend.
matplotlib.use("Agg")
import matplotlib.pyplot as plt


LOG_PATTERN = "odom_log_*.csv"
OUTPUT_FILE = "assignment02_square_plot.png"
HEADING_LENGTH_M = 0.045  # Physical length represented by each heading arrow.


def newest_log(directory: Path) -> Path:
    """Return the most recently modified odometry CSV in *directory*."""
    logs = list(directory.glob(LOG_PATTERN))
    if not logs:
        raise FileNotFoundError(f"No files matching {LOG_PATTERN!r} in {directory}")
    return max(logs, key=lambda path: path.stat().st_mtime)


def load_poses(log_file: Path) -> tuple[list[float], list[float], list[float]]:
    """Read x position, y position, and heading (radians) from an odometry CSV."""
    with log_file.open(newline="") as csv_file:
        rows = csv.DictReader(csv_file)
        required = {"odom_x_m", "odom_y_m", "odom_theta_rad"}
        if not required.issubset(rows.fieldnames or []):
            raise ValueError(f"{log_file.name} is missing one of: {', '.join(sorted(required))}")

        poses = [
            (float(row["odom_x_m"]), float(row["odom_y_m"]), float(row["odom_theta_rad"]))
            for row in rows
        ]

    if not poses:
        raise ValueError(f"{log_file.name} contains no pose samples")
    x, y, theta = zip(*poses)
    return list(x), list(y), list(theta)


def main() -> None:
    directory = Path(__file__).resolve().parent
    log_file = newest_log(directory)
    x, y, theta = load_poses(log_file)

    figure, axis = plt.subplots(figsize=(7, 7), constrained_layout=True)

    # Every logged position is represented by a small circle, as requested.
    axis.plot(
        x, y, "o", markersize=3.5, markerfacecolor="#2878B5",
        markeredgecolor="white", markeredgewidth=0.35, linestyle="None",
        label="Predicted position",
    )

    # A short arrow in the robot-frame forward direction gives every pose its heading.
    heading_x = [HEADING_LENGTH_M * math.cos(angle) for angle in theta]
    heading_y = [HEADING_LENGTH_M * math.sin(angle) for angle in theta]
    axis.quiver(
        x, y, heading_x, heading_y,
        angles="xy", scale_units="xy", scale=1, pivot="tail",
        color="#E66100", width=0.0022, headwidth=3.7,
        headlength=4.8, headaxislength=4.2, zorder=3,
    )

    axis.plot(x[0], y[0], "o", color="#2CA02C", markersize=7, label="Start")
    axis.plot(x[-1], y[-1], "s", color="#D62728", markersize=6, label="End")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, linestyle="--", linewidth=0.6, alpha=0.55)
    axis.set_xlabel("Odometry x (m)")
    axis.set_ylabel("Odometry y (m)")
    axis.set_title(f"Assignment 02 Square Path: Predicted Pose\n{log_file.name}")
    axis.legend(loc="best")

    output_path = directory / OUTPUT_FILE
    figure.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"Saved {output_path}")


if __name__ == "__main__":
    main()
