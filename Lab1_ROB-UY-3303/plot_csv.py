"""Plot robot telemetry: python3 plot_csv.py [log.csv] [--yaw-only]."""

import argparse
import csv
import math
from pathlib import Path


def read_log(path):
    """Read and validate the telemetry columns produced by GUI_code.py."""
    columns = ("time_s", "enc_a", "enc_b", "gyro_rate_dps", "angle_deg")
    values = {name: [] for name in columns}
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        missing = set(columns) - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for row in reader:
            try:
                sample = {name: float(row[name]) for name in columns}
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}: invalid number on line {reader.line_num}") from exc
            if not all(math.isfinite(value) for value in sample.values()):
                raise ValueError(f"{path}: nonfinite number on line {reader.line_num}")
            for name, value in sample.items():
                values[name].append(value)
    if not values["time_s"]:
        raise ValueError(f"{path}: no telemetry samples")
    return values


def main():
    parser = argparse.ArgumentParser(description="Plot yaw, gyro rate, and encoder counts against time.")
    parser.add_argument("csv", nargs="?", type=Path, help="CSV file (default: latest nonempty raw_log_*.csv beside this script)")
    parser.add_argument("--yaw-only", action="store_true", help="Plot only the yaw angle")
    parser.add_argument("--save", type=Path, help="Save the figure, e.g. yaw.png or telemetry.pdf")
    parser.add_argument("--no-show", action="store_true", help="Skip opening a plot window (requires --save)")
    args = parser.parse_args()
    if args.no_show and not args.save:
        parser.error("--no-show requires --save")

    path = args.csv
    if path is None:
        logs = [p for p in Path(__file__).parent.glob("raw_log_*.csv") if p.stat().st_size > 0]
        if not logs:
            parser.error("No nonempty raw_log_*.csv found; supply a CSV path.")
        path = max(logs, key=lambda p: p.name)
    try:
        data = read_log(path)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    try:
        import matplotlib
        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        parser.error("Matplotlib is required. Install it with: python3 -m pip install matplotlib")

    rows = 1 if args.yaw_only else 3
    fig, axes = plt.subplots(rows, 1, sharex=True, squeeze=False,
                             figsize=(11, 4 if args.yaw_only else 9),
                             constrained_layout=True)
    axes = axes[:, 0]
    time = data["time_s"]
    axes[0].plot(time, data["angle_deg"], color="tab:blue", linewidth=2)
    axes[0].set(ylabel="Yaw (degrees)", title="Yaw angle (integrated Z-axis gyro)")
    if not args.yaw_only:
        axes[1].plot(time, data["gyro_rate_dps"], color="tab:orange")
        axes[1].set(ylabel="Rate (degrees/s)", title="Yaw angular velocity")
        axes[2].plot(time, data["enc_a"], label="Encoder A")
        axes[2].plot(time, data["enc_b"], label="Encoder B")
        axes[2].set(ylabel="Encoder counts", title="Wheel encoders")
        axes[2].legend()
    for axis in axes:
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Recording time (seconds)")
    fig.suptitle(path.name)
    if args.save:
        fig.savefig(args.save, dpi=180)
        print(f"Saved plot to {args.save.resolve()}")
    if not args.no_show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
