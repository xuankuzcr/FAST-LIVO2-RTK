#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_time_offset(debug_dir: Path):
    values = {}
    path = debug_dir / "time_offset.txt"
    if not path.exists():
        return values
    for line in path.read_text().splitlines():
        parts = line.strip().split(maxsplit=1)
        if len(parts) != 2:
            continue
        key, value = parts
        try:
            values[key] = float(value)
        except ValueError:
            values[key] = value
    return values


def to_float(value):
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def read_velocity_csv(path: Path):
    rows = []
    if not path.exists():
        return rows
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            rows.append(
                {
                    "time_sec": to_float(row.get("time_sec")),
                    "livo_speed_mps": to_float(row.get("livo_speed_mps")),
                    "rtk_speed_coarse_mps": to_float(row.get("rtk_speed_coarse_mps")),
                    "rtk_speed_applied_mps": to_float(row.get("rtk_speed_applied_mps")),
                }
            )
    return rows


def read_score_csv(path: Path):
    rows = []
    if not path.exists():
        return rows
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            rows.append(
                {
                    "fine_offset_sec": to_float(row.get("fine_offset_sec")),
                    "score": to_float(row.get("score")),
                    "pairs": to_float(row.get("pairs")),
                }
            )
    return rows


def finite_series(rows, key):
    xs = []
    ys = []
    finite_times = [r["time_sec"] for r in rows if math.isfinite(r["time_sec"])]
    if not finite_times:
        return xs, ys
    t0 = finite_times[0]
    for row in rows:
        t = row["time_sec"]
        y = row[key]
        if math.isfinite(t) and math.isfinite(y):
            xs.append(t - t0)
            ys.append(y)
    return xs, ys


def plot(debug_dir: Path):
    velocity_rows = read_velocity_csv(debug_dir / "time_alignment_velocity.csv")
    score_rows = read_score_csv(debug_dir / "time_alignment_score.csv")
    offsets = read_time_offset(debug_dir)

    if not velocity_rows:
        raise RuntimeError("missing time_alignment_velocity.csv")

    fig, axes = plt.subplots(2, 1, figsize=(13, 8), constrained_layout=True)

    ax = axes[0]
    for key, label, color, alpha in [
        ("livo_speed_mps", "LIVO speed", "tab:blue", 0.9),
        ("rtk_speed_coarse_mps", "RTK speed, coarse", "tab:orange", 0.65),
        ("rtk_speed_applied_mps", "RTK speed, applied", "tab:green", 0.9),
    ]:
        xs, ys = finite_series(velocity_rows, key)
        if xs:
            ax.plot(xs, ys, label=label, linewidth=1.3, alpha=alpha, color=color)

    applied = offsets.get("applied_time_offset_sec", math.nan)
    fine = offsets.get("velocity_fine_time_offset_sec", math.nan)
    fine_used = int(offsets.get("velocity_fine_used", 0)) if "velocity_fine_used" in offsets else 0
    source = offsets.get("velocity_source", "unknown")
    score = offsets.get("velocity_best_score", math.nan)
    zero_score = offsets.get("velocity_zero_score", math.nan)
    ax.set_title(
        "Velocity alignment "
        f"(source={source}, applied={applied:.6f}s, fine={fine:.3f}s, used={fine_used}, "
        f"score={score:.3f}, zero={zero_score:.3f})"
    )
    ax.set_xlabel("Time from first LIVO velocity sample (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")

    ax = axes[1]
    xs = []
    ys = []
    for row in score_rows:
        x = row["fine_offset_sec"]
        y = row["score"]
        if math.isfinite(x) and math.isfinite(y):
            xs.append(x)
            ys.append(y)
    if xs:
        ax.plot(xs, ys, color="tab:purple", linewidth=1.4, label="velocity correlation")
        ax.axvline(0.0, color="0.35", linestyle="--", linewidth=1.0, label="zero fine offset")
        if math.isfinite(fine):
            ax.axvline(fine, color="tab:red", linestyle=":", linewidth=1.2, label="best fine offset")
    ax.set_title("Fine time-offset search")
    ax.set_xlabel("Fine offset applied to RTK time (s)")
    ax.set_ylabel("Normalized correlation")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")

    output = debug_dir / "time_alignment.png"
    fig.savefig(output, dpi=160)
    plt.close(fig)
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug-dir", required=True)
    args = parser.parse_args()
    plot(Path(args.debug_dir))


if __name__ == "__main__":
    main()
