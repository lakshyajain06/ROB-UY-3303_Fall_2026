#!/usr/bin/env python3
"""Plot odometry repeatability for the three Raw Timed Move experiment sets.

Each CSV is one run.  The figures show all ten odometry trajectories and an
``X`` at each run's final predicted position.
"""

from __future__ import annotations

import csv
import math
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # Save PNGs without needing a desktop display.
import matplotlib.pyplot as plt


# Folder name format: PWM_A_PWM_B_DURATION_TENTHS.  Set labels describe the
# observed trajectory; change them here if your physical motor naming differs.
EXPERIMENTS = (
    ("150_170_25", "Straight", "repeatability_straight.png"),
    ("200_170_25", "Curve (200 / 170)", "repeatability_curve_200_170.png"),
    ("200_150_25", "Curve (200 / 150)", "repeatability_curve_200_150.png"),
)


def natural_sort_key(path: Path) -> list[object]:
    """Sort trial_2 before trial_10 while retaining timestamp file order."""
    return [int(part) if part.isdigit() else part.lower() for part in re.split(r"(\d+)", path.name)]


def load_run(csv_path: Path) -> tuple[list[float], list[float]]:
    """Read the predicted x/y odometry trajectory from one recording."""
    with csv_path.open(newline="") as file:
        reader = csv.DictReader(file)
        required = {"odom_x_m", "odom_y_m"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{csv_path} does not include odom_x_m and odom_y_m")
        poses = [(float(row["odom_x_m"]), float(row["odom_y_m"])) for row in reader]

    if not poses:
        raise ValueError(f"{csv_path} has no odometry samples")
    x, y = zip(*poses)
    return list(x), list(y)


def parse_command(folder_name: str) -> tuple[int, int, float]:
    """Extract PWM A, PWM B, and duration in seconds from a set folder name."""
    pwm_a, pwm_b, duration_tenths = (int(value) for value in folder_name.split("_"))
    return pwm_a, pwm_b, duration_tenths / 10


def plot_experiment(directory: Path, folder_name: str, motion_name: str, output_name: str) -> None:
    run_files = sorted((directory / folder_name).glob("*.csv"), key=natural_sort_key)
    if len(run_files) != 10:
        raise ValueError(f"Expected 10 CSV runs in {folder_name}, found {len(run_files)}")

    pwm_a, pwm_b, duration_s = parse_command(folder_name)
    figure, axis = plt.subplots(figsize=(7, 7), constrained_layout=True)
    colors = plt.get_cmap("tab10").colors
    endpoint_distances: list[float] = []

    for run_number, csv_path in enumerate(run_files, start=1):
        x, y = load_run(csv_path)
        color = colors[(run_number - 1) % len(colors)]
        axis.plot(x, y, color=color, linewidth=1.5, alpha=0.85, label=f"Run {run_number}")
        axis.plot(x[-1], y[-1], "x", color=color, markersize=8, markeredgewidth=2, zorder=3)
        endpoint_distances.append(math.hypot(x[-1], y[-1]))

    # All trials are reset to this same initial pose before GO.
    axis.plot(0, 0, "ko", markersize=5, label="Start (0, 0)", zorder=4)
    mean_distance = sum(endpoint_distances) / len(endpoint_distances)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, linestyle="--", linewidth=0.6, alpha=0.55)
    axis.set_xlabel("Odometry x (m)")
    axis.set_ylabel("Odometry y (m)")
    axis.set_title(
        f"{motion_name} Repeatability (10 runs)\n"
        f"PWM A={pwm_a}, PWM B={pwm_b}; duration={duration_s:.1f} s\n"
        f"Mean final displacement={mean_distance:.2f} m"
    )
    # Keep the legend outside the data area and clear of both labels and title.
    axis.legend(
        loc="center left", bbox_to_anchor=(1.02, 0.5),
        fontsize=8, ncol=1,
    )
    figure.savefig(directory / output_name, dpi=300, bbox_inches="tight")
    plt.close(figure)
    print(f"Saved {output_name} from {folder_name} ({mean_distance:.3f} m mean endpoint displacement)")


def main() -> None:
    directory = Path(__file__).resolve().parent
    for folder_name, motion_name, output_name in EXPERIMENTS:
        plot_experiment(directory, folder_name, motion_name, output_name)


if __name__ == "__main__":
    main()
