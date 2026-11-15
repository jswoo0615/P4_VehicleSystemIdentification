
from pathlib import Path
import csv

import numpy as np
import matplotlib.pyplot as plt

if __package__:
    from .signals import CHANNELS, read_signals
else:  # Also support direct script execution.
    from signals import CHANNELS, read_signals

OUTPUT = Path("output")


def read_maneuvers(path):
    if not path.exists():
        return []

    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def finite_range(values):
    valid = values[np.isfinite(values)]

    if valid.size == 0:
        return np.nan

    return float(np.max(valid) - np.min(valid))


def main():
    files = sorted(OUTPUT.glob("vblog_*_signals.csv"))

    if not files:
        raise RuntimeError(
            "No VBO signal CSV files found."
        )

    runs = []
    summary = []

    for path in files:
        name = path.name.removesuffix("_signals.csv")

        signals = read_signals(path)

        maneuvers = read_maneuvers(
            OUTPUT / f"{name}_maneuvers.csv"
        )

        runs.append((name, signals, maneuvers))

        summary.append({
            "file": name,
            "samples": len(signals["time"]),
            "duration_s": float(
                signals["time"][-1]
                - signals["time"][0]
            ),
            "candidates": len(maneuvers),
            "steering_range_deg": finite_range(
                signals["steer_ang2"]
            ),
            "yawrate_range_raw": finite_range(
                signals["YawRate"]
            ),
            "yaw_rate_range_raw": finite_range(
                signals["Yaw_Rate"]
            ),
            "velocity_mean_raw": float(
                np.nanmean(signals["velocity"])
            ) if np.isfinite(
                signals["velocity"]
            ).any() else np.nan
        })

    # ---------------------------------------
    # 1. Overlay plot
    # ---------------------------------------
    fig, axes = plt.subplots(
        4, 1,
        figsize=(15, 12),
        sharex=True,
        constrained_layout=True
    )

    for ax, (channel, ylabel) in zip(axes, CHANNELS):
        for name, data, _ in runs:
            ax.plot(
                data["time"],
                data[channel],
                linewidth=1.1,
                label=name
            )

        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

    axes[-1].set_xlabel(
        "Time from each log start (s)"
    )

    fig.suptitle("VBO Run Comparison")

    output_plot = OUTPUT / "vbo_comparison.png"
    fig.savefig(output_plot, dpi=170)
    plt.close(fig)

    # ---------------------------------------
    # 2. Maneuver comparison table
    # ---------------------------------------
    maneuver_rows = []

    for name, _, maneuvers in runs:
        for m in maneuvers:
            maneuver_rows.append({
                "file": name,
                "maneuver_id": m["id"],
                "start_s": m["start"],
                "end_s": m["end"],
                "delta_deg": m["delta_deg"],
                "rise_s": m["rise_s"],
                "hold_s": m["hold_s"]
            })

    maneuver_path = (
        OUTPUT / "maneuver_comparison.csv"
    )

    with maneuver_path.open(
        "w", newline="", encoding="utf-8"
    ) as f:
        fields = [
            "file", "maneuver_id",
            "start_s", "end_s",
            "delta_deg", "rise_s", "hold_s"
        ]

        writer = csv.DictWriter(
            f, fieldnames=fields
        )

        writer.writeheader()
        writer.writerows(maneuver_rows)

    # ---------------------------------------
    # 3. Run summary table
    # ---------------------------------------
    summary_path = OUTPUT / "vbo_summary.csv"

    with summary_path.open(
        "w", newline="", encoding="utf-8"
    ) as f:
        writer = csv.DictWriter(
            f, fieldnames=list(summary[0])
        )

        writer.writeheader()
        writer.writerows(summary)

    # ---------------------------------------
    # 4. Console
    # ---------------------------------------
    print("\n=== VBO Comparison ===")

    for s in summary:
        print(
            f"{s['file']}: "
            f"{s['samples']} samples, "
            f"{s['duration_s']:.2f} s, "
            f"{s['candidates']} candidates"
        )

    print(f"\n[SAVED] {output_plot}")
    print(f"[SAVED] {summary_path}")
    print(f"[SAVED] {maneuver_path}")


if __name__ == "__main__":
    main()
