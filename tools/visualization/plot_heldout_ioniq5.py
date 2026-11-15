"""Plot held-out IONIQ5 predictions exported by the two C++ identifiers.

Diagnostics use test targets only for evaluation; no parameters are fed back
to the models. Run from the repository root after the --report-dir exports.
"""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load(path):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"Empty export: {path}")
    return {k: np.array([float(row[k]) for row in rows]) for k in rows[0]}


def rmse(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))


def bin_metrics(x, error, edges):
    results = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        mask = (x >= lo) & (x < hi if hi != edges[-1] else x <= hi)
        if np.any(mask):
            results.append((lo, hi, int(mask.sum()), float(np.sqrt(np.mean(error[mask] ** 2)))))
    return results


def steering_update_intervals(path):
    times = []
    last_delta = None
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            delta = float(row["delta_stm_rad"])
            if last_delta is None or delta != last_delta:
                times.append(float(row["time_s"]))
            last_delta = delta
    dt = np.diff(times)
    return (len(times), float(np.median(dt)),
            float(np.mean((dt >= 0.015) & (dt <= 0.025))))


def model_a(path, raw_dir, lines):
    d = load(path)
    name = path.stem.removesuffix("_model_a")
    segment = d["segment"].astype(int)
    valid = d["sample"] > 0  # initial yaw is measured, not predicted
    measured = d["yaw_measured_radps"]
    predicted = d["yaw_predicted_radps"]
    error = measured - predicted
    lines.append(f"## {name}: Model A")
    lines.append(f"Scored samples: {valid.sum()}, RMSE: {rmse(measured[valid], predicted[valid]):.6f} rad/s")
    for sid in np.unique(segment):
        m = (segment == sid) & valid
        t = d["time_s"][segment == sid]
        lines.append(f"Segment {sid}: {m.sum()} scored, {t[0]:.3f}–{t[-1]:.3f} s, RMSE {rmse(measured[m], predicted[m]):.6f} rad/s")
    fig, axes = plt.subplots(len(np.unique(segment)), 1, figsize=(14, 3 * len(np.unique(segment))), squeeze=False)
    for ax, sid in zip(axes[:, 0], np.unique(segment)):
        m = segment == sid
        ax.plot(d["time_s"][m], measured[m], label="Measured", lw=1)
        ax.plot(d["time_s"][m], predicted[m], label="Predicted", lw=1)
        ax.set(title=f"{name} · segment {sid}", ylabel="Yaw rate (rad/s)")
        ax.grid(alpha=.3)
        ax.legend()
    axes[-1, 0].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(path.with_suffix(".png"), dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for ax, x, label in ((axes[0], d["speed_mps"], "Speed (m/s)"),
                         (axes[1], np.abs(d["ay_mps2"]), "|Lateral acceleration| (m/s²)")):
        ax.scatter(x[valid], error[valid], s=2, alpha=.13)
        ax.axhline(0, color="black", lw=.8)
        ax.set(xlabel=label, ylabel="Measured − predicted (rad/s)")
        ax.grid(alpha=.3)
        edges = np.quantile(x[valid], [0, .25, .5, .75, 1])
        lines.append(f"{label} quartile RMSE (range, count, rad/s):")
        for lo, hi, n, score in bin_metrics(x[valid], error[valid], edges):
            lines.append(f"  [{lo:.3f}, {hi:.3f}]: {n}, {score:.6f}")
    fig.suptitle(name + " · held-out residuals")
    fig.tight_layout()
    fig.savefig(path.with_name(name + "_model_a_residuals.png"), dpi=160)
    plt.close(fig)

    # Compare equal interior samples for all offsets; positive shift means
    # the prediction is moved later relative to the measurement.
    shifted = {lag: [] for lag in range(-3, 4)}
    actual = []
    dt_all = []
    for sid in np.unique(segment):
        m = segment == sid
        y, p, t = measured[m], predicted[m], d["time_s"][m]
        dt_all.extend(np.diff(t))
        for i in range(4, len(y) - 3):
            actual.append(y[i])
            for lag in shifted:
                shifted[lag].append(p[i - lag])
    actual = np.asarray(actual)
    scores = {lag: rmse(actual, np.asarray(values)) for lag, values in shifted.items()}
    lines.append("Common-interior time-shift RMSE, prediction shifted later by lag × sample interval:")
    lines.append(", ".join(f"{lag:+d}: {score:.6f}" for lag, score in scores.items()))
    lines.append(f"Median selected sample interval: {np.median(dt_all):.5f} s")
    raw = raw_dir / (name + ".csv")
    if raw.exists():
        n, median, fraction = steering_update_intervals(raw)
        lines.append(f"Raw steering value changes: {n}, median interval {median:.5f} s, 15–25 ms fraction {fraction:.3f}")
    # Descriptive held-out amplitude check, never reused for prediction.
    slope, intercept = np.polyfit(predicted[valid], measured[valid], 1)
    adjusted = slope * predicted[valid] + intercept
    lines.append(f"Held-out affine diagnostic: measured ≈ {slope:.4f} × predicted + {intercept:.4f}; in-sample diagnostic RMSE {rmse(measured[valid], adjusted):.6f} rad/s")
    lines.append("")


def s_load(path, lines):
    d = load(path)
    name = path.stem.removesuffix("_s_load")
    lines.append(f"## {name}: S-Load")
    fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    for ax, side in zip(axes, ("left", "right")):
        measured = d[f"{side}_measured_N"]
        predicted = d[f"{side}_predicted_N"]
        ax.plot(d["time_s"], measured, lw=.8, label="Measured")
        ax.plot(d["time_s"], predicted, lw=.8, label="Predicted")
        ax.set(ylabel=f"{side.title()} force (N)")
        ax.grid(alpha=.3)
        ax.legend()
        lines.append(f"{side.title()}: {len(measured)} updates, RMSE {rmse(measured, predicted):.3f} N, mean residual {np.mean(measured-predicted):.3f} N")
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(name + " · held-out tie-rod force")
    fig.tight_layout()
    fig.savefig(path.with_suffix(".png"), dpi=160)
    plt.close(fig)
    lines.append("")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report-dir", type=Path, required=True)
    parser.add_argument("--raw-dir", type=Path, required=True)
    args = parser.parse_args()
    lines = ["# Held-out IONIQ5 diagnostics", "", "First sample of each Model A segment initializes the state and is excluded from RMSE. All fits remain training-only. Time-shift and affine checks are descriptive on held-out data.", ""]
    a_files = sorted(args.report_dir.glob("*_model_a.csv"))
    s_files = sorted(args.report_dir.glob("*_s_load.csv"))
    if len(a_files) != 2 or len(s_files) != 2:
        raise ValueError("Expected two Model A and two S-Load export files")
    for path in a_files:
        model_a(path, args.raw_dir, lines)
    for path in s_files:
        s_load(path, lines)
    output = args.report_dir / "heldout_metrics.md"
    output.write_text("\n".join(lines) + "\n")
    print(output)


if __name__ == "__main__":
    main()
