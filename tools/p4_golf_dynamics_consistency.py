#!/usr/bin/env python3
"""Check Golf lateral-acceleration kinematics without changing model fits."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.integrate import cumulative_trapezoid
from scipy.signal import savgol_filter


REQUIRED = ("t_s", "Vx_mps", "Vy_mps", "yawRate_radps", "ayCG_mps2", "bank_rad")


def rmse(left, right):
    return float(np.sqrt(np.mean(np.square(left - right))))


def summarize(t, kin, ay, bank_g, start, end):
    selected = (t >= start) & (t <= end)
    x, y, g = kin[selected], ay[selected], bank_g[selected]
    return {
        "start_s": start,
        "end_s": end,
        "sample_count": int(selected.sum()),
        "correlation_kin_vs_ay": float(np.corrcoef(x, y)[0, 1]),
        "rmse_kin_vs_ay_mps2": rmse(x, y),
        "mean_ay_minus_kin_mps2": float(np.mean(y - x)),
        "rmse_kin_plus_gsin_bank_mps2": rmse(x + g, y),
        "rmse_kin_minus_gsin_bank_mps2": rmse(x - g, y),
        "max_abs_gsin_bank_mps2": float(np.max(np.abs(g))),
    }


def analyze(csv, start, end, focus_start, focus_end, window):
    df = pd.read_csv(csv, comment="#", usecols=list(REQUIRED))
    missing = [name for name in REQUIRED if name not in df]
    if missing:
        raise ValueError(f"Missing columns: {missing}")
    df = df[(df.t_s >= start) & (df.t_s <= end)].reset_index(drop=True)
    if not start <= focus_start < focus_end <= end:
        raise ValueError("Focus interval must be inside the analysis interval")
    if len(df) < max(window, 20) or window < 5 or window % 2 != 1:
        raise ValueError("Use an odd Savitzky–Golay window of at least 5 samples")
    if not np.isfinite(df.to_numpy()).all():
        raise ValueError("Input contains nonfinite values")
    t = df.t_s.to_numpy()
    dt = float(np.median(np.diff(t)))
    if np.any(np.diff(t) <= 0) or np.max(abs(np.diff(t) - dt)) > 0.01 * dt:
        raise ValueError("Samples must be regular and strictly increasing")
    if (df.Vx_mps < 5).any():
        raise ValueError("Analysis interval contains speeds below 5 m/s")

    vx, vy = df.Vx_mps.to_numpy(), df.Vy_mps.to_numpy()
    yaw, ay_raw = df.yawRate_radps.to_numpy(), df.ayCG_mps2.to_numpy()
    vy_dot = savgol_filter(vy, window, 3, deriv=1, delta=dt)
    ay = savgol_filter(ay_raw, window, 3)
    vx_yaw = vx * yaw
    kin = vy_dot + vx_yaw
    bank_g = 9.80665 * np.sin(df.bank_rad.to_numpy())

    focus = (t >= focus_start) & (t <= focus_end)
    tf, vf, yf, qf = t[focus], vy[focus], vx_yaw[focus], ay_raw[focus]
    vy_reconstructed = vf[0] + cumulative_trapezoid(qf - yf, tf, initial=0)
    result = {
        "experiment": "P4-GOLF-DYNAMICS-CONSISTENCY-001",
        "source_csv": str(csv.resolve()),
        "method": {
            "kinematic_relation": "ay_kin = d(Vy_mps)/dt + Vx_mps*yawRate_radps",
            "derivative_and_ay_smoothing": "Savitzky-Golay, cubic, centered",
            "window_samples": window,
            "window_seconds": window * dt,
            "integration": "raw ayCG_mps2 - Vx_mps*yawRate_radps, trapezoidal, measured initial Vy",
        },
        "whole_interval": summarize(t, kin, ay, bank_g, start, end),
        "focus_interval": summarize(t, kin, ay, bank_g, focus_start, focus_end),
        "focus_vy_reconstruction_rmse_mps": rmse(vy_reconstructed, vf),
        "focus_vy_reconstruction_end_error_mps": float(vy_reconstructed[-1] - vf[-1]),
    }
    return result, (t, vy, vy_dot, vx_yaw, kin, ay_raw, ay, bank_g, focus, vy_reconstructed)


def plot_focus(data, start, end, output):
    t, vy, vy_dot, vx_yaw, kin, ay_raw, ay, bank_g, focus, vy_reconstructed = data
    x = t[focus]
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, layout="constrained")
    axes[0].plot(x, ay_raw[focus], color="0.75", lw=0.6, label="ayCG raw")
    axes[0].plot(x, ay[focus], color="black", lw=1.4, label="ayCG filtered")
    axes[0].plot(x, kin[focus], color="tab:blue", lw=1.3, label="dVy/dt + Vx*r")
    axes[0].set_ylabel("Lateral accel. (m/s²)")
    axes[0].legend(ncol=3, fontsize=8)
    axes[1].plot(x, vy_dot[focus], label="dVy/dt")
    axes[1].plot(x, vx_yaw[focus], label="Vx*r")
    axes[1].set_ylabel("Components (m/s²)")
    axes[1].legend(fontsize=8)
    axes[2].plot(x, vy[focus], color="black", label="Measured Vy")
    axes[2].plot(x, vy_reconstructed, label="Integrated ayCG - Vx*r")
    axes[2].set_ylabel("Vy (m/s)")
    axes[2].legend(fontsize=8)
    axes[3].plot(x, (ay - kin)[focus], label="ayCG filtered - kinematic")
    axes[3].plot(x, bank_g[focus], label="g*sin(bank)")
    axes[3].axhline(0, color="0.4", lw=0.7)
    axes[3].set_ylabel("Difference (m/s²)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(fontsize=8)
    for axis in axes:
        axis.grid(alpha=0.25)
    fig.suptitle(f"Golf dynamics consistency | {start:g}–{end:g} s")
    fig.savefig(output, dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--start", type=float, default=75)
    parser.add_argument("--end", type=float, default=100)
    parser.add_argument("--focus-start", type=float, default=85)
    parser.add_argument("--focus-end", type=float, default=88)
    parser.add_argument("--window", type=int, default=81)
    parser.add_argument("--out", type=Path, default=Path("output/golf_dynamics_consistency_001"))
    args = parser.parse_args()
    result, data = analyze(args.csv, args.start, args.end, args.focus_start, args.focus_end, args.window)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
    plot_focus(data, args.focus_start, args.focus_end, args.out.with_suffix(".png"))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
