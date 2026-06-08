#!/usr/bin/env python3
"""Plot SampleMyTest speed-loop CSV data."""

import argparse
import csv
import sys
from pathlib import Path


def read_rows(path):
    rows = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required = {"time", "target_speed", "actual_speed", "filtered_speed"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"missing CSV columns: {', '.join(sorted(missing))}")

        for row in reader:
            rows.append(
                {
                    "time": float(row["time"]),
                    "target_speed": float(row["target_speed"]),
                    "actual_speed": float(row["actual_speed"]),
                    "filtered_speed": float(row["filtered_speed"]),
                }
            )

    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv",
        nargs="?",
        default="sample_my_speed.csv",
        help="speed CSV path written by the sample",
    )
    parser.add_argument(
        "-o",
        "--out",
        default=None,
        help="output PNG path, defaults to CSV path with .png suffix",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.is_file():
        sys.exit(f"error: CSV not found: {csv_path}")

    rows = read_rows(csv_path)
    if not rows:
        sys.exit(f"error: CSV has no samples: {csv_path}")

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("error: matplotlib is required. Install it with: python -m pip install matplotlib")

    out_path = Path(args.out) if args.out else csv_path.with_suffix(".png")

    time = [row["time"] for row in rows]
    target_speed = [row["target_speed"] for row in rows]
    actual_speed = [row["actual_speed"] for row in rows]
    filtered_speed = [row["filtered_speed"] for row in rows]

    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(time, target_speed, label="target_speed", linewidth=2.0)
    ax.plot(time, actual_speed, label="actual_speed", linewidth=1.2)
    ax.plot(time, filtered_speed, label="filtered_speed", linewidth=1.2, linestyle="--")
    ax.set_title("SampleMyTest Speed Loop")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
