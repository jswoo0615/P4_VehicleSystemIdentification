#!/usr/bin/env python3
"""Compare frozen Golf B0 yaw-moment terms with measured yaw acceleration."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.integrate import trapezoid
from scipy.signal import savgol_filter


def yaw_terms(vy, yaw, vx, steer, params):
    cf = params["cf_over_m_mps2_per_rad"]
    cr = params["cr_over_m_mps2_per_rad"]
    lf = params["lf_m"]
    lr = params["lr_m"]
    iz_m = params["iz_over_m_m2"]
    front = lf * cf / iz_m * (steer - (vy + lf * yaw) / vx)
    rear = -lr * cr / iz_m * (-(vy - lr * yaw) / vx)
    return front, rear


def rmse(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))


def score(t, measured_dot, measured_state_dot, free_dot, start, end):
    use = (t >= start) & (t <= end)
    a, m, f = measured_dot[use], measured_state_dot[use], free_dot[use]
    return {
        "interval_s": [start, end],
        "samples": int(use.sum()),
        "measured_state_yaw_accel_rmse_radps2": rmse(m, a),
        "free_run_yaw_accel_rmse_radps2": rmse(f, a),
        "measured_state_yaw_accel_bias_radps2": float(np.mean(m - a)),
        "free_run_yaw_accel_bias_radps2": float(np.mean(f - a)),
        "measured_state_yaw_accel_correlation": float(np.corrcoef(m, a)[0, 1]),
        "free_run_yaw_accel_correlation": float(np.corrcoef(f, a)[0, 1]),
    }


def analyze(raw_csv, replay_csv, baseline_json, focus_start, focus_end,
            ramp_start, ramp_end, window):
    params = json.loads(baseline_json.read_text())["model_b0"]
    replay = pd.read_csv(replay_csv)
    needed = ("t_s", "vx_mps", "steer_rad", "vy_measured_mps", "yaw_measured_radps",
              "vy_B0_mps", "yaw_B0_radps")
    missing = [key for key in needed if key not in replay]
    if missing:
        raise ValueError(f"Replay is missing columns: {missing}")
    if not np.isfinite(replay[list(needed)].to_numpy()).all():
        raise ValueError("Replay contains nonfinite values")
    t = replay.t_s.to_numpy()
    if len(t) < 2 or np.any(np.diff(t) <= 0) or np.any(np.diff(t) > 0.025):
        raise ValueError("Replay must have continuous, ordered 50 Hz samples")
    if np.any(replay.vx_mps.to_numpy() < 5):
        raise ValueError("Replay includes speeds below 5 m/s")
    if not t[0] < focus_start < ramp_start < ramp_end < focus_end < t[-1]:
        raise ValueError("Focus and ramp must be within the replay interval")
    if window < 5 or window % 2 != 1:
        raise ValueError("Filter window must be odd and at least 5 samples")
    raw = pd.read_csv(raw_csv, comment="#", usecols=["t_s", "yawRate_radps"])
    raw = raw[(raw.t_s >= t[0] - 1) & (raw.t_s <= t[-1] + 1)].reset_index(drop=True)
    raw_t = raw.t_s.to_numpy()
    raw_yaw = raw.yawRate_radps.to_numpy()
    if len(raw) < window or not np.isfinite(raw.to_numpy()).all():
        raise ValueError("Raw yaw signal is too short or nonfinite")
    if raw_t[0] > t[0] or raw_t[-1] < t[-1] or np.any(np.diff(raw_t) <= 0):
        raise ValueError("Raw yaw signal must span the ordered replay")
    dt_raw = float(np.median(np.diff(raw_t)))
    if np.max(abs(np.diff(raw_t) - dt_raw)) > 0.01 * dt_raw:
        raise ValueError("Raw yaw signal has irregular sample times")
    # Centered Savitzky–Golay derivative: no intentional phase shift.
    yaw_dot_raw = savgol_filter(raw_yaw, window, 3, deriv=1, delta=dt_raw)
    yaw_dot_measured = np.interp(t, raw_t, yaw_dot_raw)
    vx = replay.vx_mps.to_numpy()
    steer = replay.steer_rad.to_numpy()
    vy_m = replay.vy_measured_mps.to_numpy()
    yaw_m = replay.yaw_measured_radps.to_numpy()
    vy_f = replay.vy_B0_mps.to_numpy()
    yaw_f = replay.yaw_B0_radps.to_numpy()
    front_m, rear_m = yaw_terms(vy_m, yaw_m, vx, steer, params)
    front_f, rear_f = yaw_terms(vy_f, yaw_f, vx, steer, params)
    dot_m, dot_f = front_m + rear_m, front_f + rear_f

    # Midpoint step identity checks the equation and 50 Hz free-run trajectory.
    mid_inputs = (0.5 * (x[1:] + x[:-1]) for x in (vy_f, yaw_f, vx, steer))
    vy_mid, yaw_mid, vx_mid, steer_mid = mid_inputs
    front_mid, rear_mid = yaw_terms(vy_mid, yaw_mid, vx_mid, steer_mid, params)
    step_dot = np.diff(yaw_f) / np.diff(t)
    midpoint_residual = step_dot - (front_mid + rear_mid)

    ramp_use = np.flatnonzero((t >= ramp_start) & (t <= ramp_end))
    first, last = int(ramp_use[0]), int(ramp_use[-1])
    integral = lambda values: float(trapezoid(values[first:last + 1], t[first:last + 1]))
    ramp = score(t, yaw_dot_measured, dot_m, dot_f, ramp_start, ramp_end)
    ramp.update({
        "sampled_interval_s": [float(t[first]), float(t[last])],
        "yaw_error_start_radps": float(yaw_f[first] - yaw_m[first]),
        "yaw_error_end_radps": float(yaw_f[last] - yaw_m[last]),
        "integrated_measured_state_residual_radps": integral(dot_m - yaw_dot_measured),
        "integrated_front_state_difference_radps": integral(front_f - front_m),
        "integrated_rear_state_difference_radps": integral(rear_f - rear_m),
        "integrated_total_dot_difference_radps": integral(dot_f - yaw_dot_measured),
    })
    result = {
        "experiment": "P4-GOLF-YAW-MOMENT-001",
        "source_csv": str(raw_csv.resolve()),
        "replay_csv": str(replay_csv.resolve()),
        "baseline_json": str(baseline_json.resolve()),
        "method": "Frozen normalized B0; centered cubic Savitzky-Golay derivative on raw 200 Hz measured yaw; interpolate to 50 Hz replay",
        "yaw_derivative_window_samples": window,
        "yaw_derivative_window_s": window * dt_raw,
        "midpoint_step_identity_max_abs_radps2": float(np.max(abs(midpoint_residual))),
        "whole": score(t, yaw_dot_measured, dot_m, dot_f, float(t[0]), float(t[-1])),
        "focus": score(t, yaw_dot_measured, dot_m, dot_f, focus_start, focus_end),
        "ramp": ramp,
    }
    table = pd.DataFrame({
        "t_s": t,
        "yaw_measured_radps": yaw_m,
        "yaw_B0_radps": yaw_f,
        "yaw_accel_measured_radps2": yaw_dot_measured,
        "measured_state_front_radps2": front_m,
        "measured_state_rear_radps2": rear_m,
        "measured_state_total_radps2": dot_m,
        "free_run_front_radps2": front_f,
        "free_run_rear_radps2": rear_f,
        "free_run_total_radps2": dot_f,
        "measured_state_residual_radps2": dot_m - yaw_dot_measured,
        "free_run_residual_radps2": dot_f - yaw_dot_measured,
    })
    return result, table


def plot(table, focus_start, focus_end, ramp_start, ramp_end, output):
    q = table[(table.t_s >= focus_start) & (table.t_s <= focus_end)]
    x = q.t_s
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, layout="constrained")
    axes[0].plot(x, q.yaw_measured_radps, "k", label="Measured yaw")
    axes[0].plot(x, q.yaw_B0_radps, label="B0 free-run yaw")
    axes[0].set_ylabel("Yaw rate (rad/s)")
    axes[0].legend(fontsize=8)
    axes[1].plot(x, q.yaw_accel_measured_radps2, "k", label="Measured yaw accel.")
    axes[1].plot(x, q.measured_state_total_radps2, label="B0 at measured state")
    axes[1].plot(x, q.free_run_total_radps2, label="B0 free-run")
    axes[1].set_ylabel("Yaw accel. (rad/s²)")
    axes[1].legend(ncol=3, fontsize=8)
    axes[2].plot(x, q.measured_state_front_radps2, label="Front, measured state")
    axes[2].plot(x, q.measured_state_rear_radps2, label="Rear, measured state")
    axes[2].plot(x, q.free_run_front_radps2, "--", label="Front, free-run")
    axes[2].plot(x, q.free_run_rear_radps2, "--", label="Rear, free-run")
    axes[2].set_ylabel("Moment terms (rad/s²)")
    axes[2].legend(ncol=2, fontsize=8)
    axes[3].plot(x, q.measured_state_residual_radps2, label="Measured-state B0 - measured accel.")
    axes[3].plot(x, q.free_run_residual_radps2, label="Free-run B0 - measured accel.")
    axes[3].axhline(0, color="0.4", lw=0.7)
    axes[3].set_ylabel("Yaw accel. error (rad/s²)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(fontsize=8)
    for axis in axes:
        axis.axvspan(ramp_start, ramp_end, color="0.8", alpha=0.2)
        axis.grid(alpha=0.25)
    fig.suptitle("VW Golf B0 yaw-moment diagnosis | 85–88 s")
    fig.savefig(output, dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_csv", type=Path)
    parser.add_argument("replay_csv", type=Path)
    parser.add_argument("--baseline", type=Path, default=Path("config/golf_baselines.json"))
    parser.add_argument("--focus-start", type=float, default=85)
    parser.add_argument("--focus-end", type=float, default=88)
    parser.add_argument("--ramp-start", type=float, default=85.7)
    parser.add_argument("--ramp-end", type=float, default=86.3)
    parser.add_argument("--window", type=int, default=81)
    parser.add_argument("--out", type=Path, default=Path("output/golf_yaw_moment_001"))
    args = parser.parse_args()
    result, table = analyze(args.raw_csv, args.replay_csv, args.baseline,
                            args.focus_start, args.focus_end, args.ramp_start,
                            args.ramp_end, args.window)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
    table.to_csv(args.out.with_suffix(".csv"), index=False)
    plot(table, args.focus_start, args.focus_end, args.ramp_start, args.ramp_end,
         args.out.with_suffix(".png"))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
