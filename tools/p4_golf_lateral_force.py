#!/usr/bin/env python3
"""Compare frozen Golf B0 lateral-force terms at measured and free-run states."""

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


def rmse(left, right):
    return float(np.sqrt(np.mean((left - right) ** 2)))


def terms(vy, yaw, vx, steer, bank, params):
    alpha_f = steer - (vy + params["lf_m"] * yaw) / vx
    alpha_r = -(vy - params["lr_m"] * yaw) / vx
    front = params["cf_over_m_mps2_per_rad"] * alpha_f
    rear = params["cr_over_m_mps2_per_rad"] * alpha_r
    bank_accel = 9.80665 * np.sin(bank)
    yaw_term = -vx * yaw
    return front, rear, bank_accel, yaw_term


def metrics(t, ay, measured_total, free_total, start, end):
    use = (t >= start) & (t <= end)
    return {
        "interval_s": [start, end],
        "samples": int(use.sum()),
        "measured_state_ay_rmse_mps2": rmse(measured_total[use], ay[use]),
        "measured_state_ay_bias_mps2": float(np.mean(measured_total[use] - ay[use])),
        "free_run_ay_rmse_mps2": rmse(free_total[use], ay[use]),
        "free_run_ay_bias_mps2": float(np.mean(free_total[use] - ay[use])),
    }


def analyze(raw_csv, replay_csv, baseline_json, focus_start, focus_end, ramp_start, ramp_end, window):
    baseline = json.loads(baseline_json.read_text())
    params = baseline["model_b0"]
    replay = pd.read_csv(replay_csv)
    raw = pd.read_csv(raw_csv, comment="#", usecols=["t_s", "ayCG_mps2"])
    t = replay.t_s.to_numpy()
    if not np.all(np.diff(t) > 0) or (replay.vx_mps < 5).any():
        raise ValueError("Replay must have increasing time and Vx >= 5 m/s")
    if not t[0] <= focus_start < ramp_start < ramp_end <= focus_end <= t[-1]:
        raise ValueError("Focus and ramp intervals must be inside the replay")
    if window < 5 or window % 2 != 1 or len(raw) < window:
        raise ValueError("ay smoothing window must be odd and at least 5 samples")
    raw_t = raw.t_s.to_numpy()
    if np.any(np.diff(raw_t) <= 0) or t[0] < raw_t[0] or t[-1] > raw_t[-1]:
        raise ValueError("Raw acceleration must span the replay without time resets")
    raw_dt = float(np.median(np.diff(raw_t)))
    if np.max(abs(np.diff(raw_t) - raw_dt)) > 0.01 * raw_dt:
        raise ValueError("Raw acceleration sample times are irregular")
    ay = np.interp(t, raw_t, savgol_filter(raw.ayCG_mps2.to_numpy(), window, 3))
    vx, steer, bank = (replay[k].to_numpy() for k in ("vx_mps", "steer_rad", "bank_rad"))
    vy_m, yaw_m = (replay[k].to_numpy() for k in ("vy_measured_mps", "yaw_measured_radps"))
    vy_f, yaw_f = (replay[k].to_numpy() for k in ("vy_B0_mps", "yaw_B0_radps"))
    if not all(np.isfinite(x).all() for x in (t, ay, vx, steer, bank, vy_m, yaw_m, vy_f, yaw_f)):
        raise ValueError("Nonfinite input")
    mf, mr, mb, my = terms(vy_m, yaw_m, vx, steer, bank, params)
    ff, fr, fb, fy = terms(vy_f, yaw_f, vx, steer, bank, params)
    measured_total = mf + mr + mb
    free_total = ff + fr + fb

    # Exact pointwise decomposition of predicted dVy/dt minus (ayCG - Vx*r_measured).
    # Force differences depend on the free-run state and are not independent causes.
    force_state = (ff + fr) - (mf + mr)
    force_closure = measured_total - ay
    yaw_state = fy - my
    derivative_gap = force_state + force_closure + yaw_state
    focus = metrics(t, ay, measured_total, free_total, focus_start, focus_end)
    ramp = metrics(t, ay, measured_total, free_total, ramp_start, ramp_end)
    chosen = np.flatnonzero((t >= ramp_start) & (t <= ramp_end))
    first, last = int(chosen[0]), int(chosen[-1])
    integral = lambda values: float(trapezoid(values[first : last + 1], t[first : last + 1]))
    ramp.update({
        "sampled_interval_s": [float(t[first]), float(t[last])],
        "vy_error_start_mps": float(vy_f[first] - vy_m[first]),
        "vy_error_end_mps": float(vy_f[last] - vy_m[last]),
        "integrated_front_state_difference_mps": integral(ff - mf),
        "integrated_rear_state_difference_mps": integral(fr - mr),
        "integrated_force_closure_mps": integral(force_closure),
        "integrated_yaw_term_difference_mps": integral(yaw_state),
        "integrated_derivative_gap_mps": integral(derivative_gap),
    })
    result = {
        "experiment": "P4-GOLF-LATERAL-FORCE-001",
        "raw_csv": str(raw_csv.resolve()),
        "replay_csv": str(replay_csv.resolve()),
        "baseline_json": str(baseline_json.resolve()),
        "comparison_assumption": "ayCG and model front+rear+bank are in the same gravity-corrected frame; not independently established",
        "method": "Frozen B0 coefficients; measured-state and 75 s-initialized 50 Hz free-run states; raw 200 Hz ayCG smoothed before sampling",
        "ay_savgol_window_s": window * raw_dt,
        "full": metrics(t, ay, measured_total, free_total, float(t[0]), float(t[-1])),
        "focus": focus,
        "ramp": ramp,
    }
    table = pd.DataFrame({
        "t_s": t, "ayCG_filtered_mps2": ay,
        "measured_front_mps2": mf, "measured_rear_mps2": mr,
        "free_front_mps2": ff, "free_rear_mps2": fr,
        "bank_mps2": mb, "measured_yaw_term_mps2": my, "free_yaw_term_mps2": fy,
        "measured_state_model_ay_mps2": measured_total,
        "free_run_model_ay_mps2": free_total,
        "force_state_difference_mps2": force_state,
        "force_closure_mps2": force_closure,
        "yaw_term_difference_mps2": yaw_state,
        "derivative_gap_mps2": derivative_gap,
        "measured_vy_mps": vy_m, "free_vy_mps": vy_f,
        "measured_yaw_radps": yaw_m, "free_yaw_radps": yaw_f,
    })
    return result, table


def plot(table, focus_start, focus_end, ramp_start, ramp_end, output):
    q = table[(table.t_s >= focus_start) & (table.t_s <= focus_end)]
    t = q.t_s
    fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True, layout="constrained")
    axes[0].plot(t, q.ayCG_filtered_mps2, "k", label="ayCG filtered")
    axes[0].plot(t, q.measured_state_model_ay_mps2, label="B0 at measured state")
    axes[0].plot(t, q.free_run_model_ay_mps2, label="B0 free-run")
    axes[0].set_ylabel("Lateral accel. (m/s²)")
    axes[0].legend(ncol=3, fontsize=8)
    for axis, prefix, label in [(axes[1], "front", "Front"), (axes[2], "rear", "Rear")]:
        axis.plot(t, q[f"measured_{prefix}_mps2"], label="Measured state")
        axis.plot(t, q[f"free_{prefix}_mps2"], label="Free-run")
        axis.set_ylabel(f"{label} contribution\n(m/s²)")
        axis.legend(fontsize=8)
    axes[3].plot(t, q.force_state_difference_mps2, label="Front + rear state difference")
    axes[3].plot(t, q.force_closure_mps2, label="Measured-state force - ayCG")
    axes[3].plot(t, q.yaw_term_difference_mps2, label="-Vx*(r_free - r_measured)")
    axes[3].axhline(0, color="0.4", lw=0.6)
    axes[3].set_ylabel("dVy gap terms (m/s²)")
    axes[3].legend(ncol=2, fontsize=8)
    axes[4].plot(t, q.measured_vy_mps, "k", label="Measured Vy")
    axes[4].plot(t, q.free_vy_mps, label="B0 free-run Vy")
    axes[4].set_ylabel("Vy (m/s)")
    axes[4].set_xlabel("Time (s)")
    axes[4].legend(fontsize=8)
    for axis in axes:
        axis.axvspan(ramp_start, ramp_end, color="0.8", alpha=0.17)
        axis.grid(alpha=0.25)
    fig.suptitle("VW Golf B0 lateral-force diagnosis | 85–88 s")
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
    parser.add_argument("--out", type=Path, default=Path("output/golf_lateral_force_001"))
    args = parser.parse_args()
    result, table = analyze(args.raw_csv, args.replay_csv, args.baseline, args.focus_start,
                            args.focus_end, args.ramp_start, args.ramp_end, args.window)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
    table.to_csv(args.out.with_suffix(".csv"), index=False)
    plot(table, args.focus_start, args.focus_end, args.ramp_start, args.ramp_end,
         args.out.with_suffix(".png"))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
