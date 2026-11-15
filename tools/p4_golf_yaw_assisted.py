#!/usr/bin/env python3
"""Diagnose Golf B0 Vy prediction with measured yaw and measured-state resets."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def midpoint_vy(previous_vy, dt, vx, steer, yaw, bank, params):
    """Frozen B0 lateral equation with the same midpoint inputs as the replay."""
    cf = params["cf_over_m_mps2_per_rad"]
    cr = params["cr_over_m_mps2_per_rad"]
    lf = params["lf_m"]
    lr = params["lr_m"]
    a = -(cf + cr) / vx
    forcing = cf * steer + ((-lf * cf + lr * cr) / vx - vx) * yaw
    forcing += 9.80665 * np.sin(bank)
    return ((1 + dt * a / 2) * previous_vy + dt * forcing) / (1 - dt * a / 2)


def score(table, start, end):
    selected = table[(table.t_s >= start) & (table.t_s <= end) &
                     table.vy_one_step_mps.notna()]
    measured = selected.vy_measured_mps.to_numpy()
    def rmse(column):
        return float(np.sqrt(np.mean((selected[column].to_numpy() - measured) ** 2)))
    def bias(column):
        return float(np.mean(selected[column].to_numpy() - measured))
    baseline = rmse("vy_baseline_mps")
    assisted = rmse("vy_yaw_assisted_mps")
    return {
        "interval_s": [start, end],
        "scored_samples": len(selected),
        "baseline_vy_rmse_mps": baseline,
        "yaw_assisted_vy_rmse_mps": assisted,
        "one_step_vy_rmse_mps": rmse("vy_one_step_mps"),
        "baseline_vy_bias_mps": bias("vy_baseline_mps"),
        "yaw_assisted_vy_bias_mps": bias("vy_yaw_assisted_mps"),
        "rmse_change_assisted_vs_baseline_pct": 100 * (assisted / baseline - 1),
    }


def analyze(replay_csv, baseline_json, focus_start, focus_end, ramp_start, ramp_end):
    params = json.loads(baseline_json.read_text())["model_b0"]
    replay = pd.read_csv(replay_csv)
    columns = ("t_s", "steer_rad", "vx_mps", "bank_rad", "yaw_measured_radps",
               "yaw_B0_radps", "vy_measured_mps", "vy_B0_mps")
    missing = [column for column in columns if column not in replay]
    if missing:
        raise ValueError(f"Replay is missing columns: {missing}")
    if not np.isfinite(replay[list(columns)].to_numpy()).all():
        raise ValueError("Replay has nonfinite data")
    t = replay.t_s.to_numpy()
    if len(t) < 2 or np.any(np.diff(t) <= 0) or np.any(np.diff(t) > 0.025):
        raise ValueError("Replay must be continuous, ordered 50 Hz samples")
    if np.any(replay.vx_mps.to_numpy() < 5):
        raise ValueError("Replay includes speeds below 5 m/s")
    if not t[0] < focus_start < ramp_start < ramp_end < focus_end < t[-1]:
        raise ValueError("Focus and ramp must be within the replay interval")

    measured_vy = replay.vy_measured_mps.to_numpy()
    measured_yaw = replay.yaw_measured_radps.to_numpy()
    vx, steer, bank = (replay[k].to_numpy() for k in ("vx_mps", "steer_rad", "bank_rad"))
    yaw_assisted = np.empty_like(measured_vy)
    one_step = np.full_like(measured_vy, np.nan)
    yaw_assisted[0] = measured_vy[0]
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        midpoint_inputs = (0.5 * (x[i - 1] + x[i]) for x in (vx, steer, measured_yaw, bank))
        v_mid, steer_mid, yaw_mid, bank_mid = midpoint_inputs
        yaw_assisted[i] = midpoint_vy(yaw_assisted[i - 1], dt, v_mid, steer_mid,
                                      yaw_mid, bank_mid, params)
        one_step[i] = midpoint_vy(measured_vy[i - 1], dt, v_mid, steer_mid,
                                  yaw_mid, bank_mid, params)

    table = pd.DataFrame({
        "t_s": t,
        "steer_rad": steer,
        "vx_mps": vx,
        "bank_rad": bank,
        "yaw_measured_radps": measured_yaw,
        "yaw_baseline_radps": replay.yaw_B0_radps.to_numpy(),
        "vy_measured_mps": measured_vy,
        "vy_baseline_mps": replay.vy_B0_mps.to_numpy(),
        "vy_yaw_assisted_mps": yaw_assisted,
        "vy_one_step_mps": one_step,
    })
    # At the initial sample the two free runs have the same measured state.
    if abs(table.vy_baseline_mps.iloc[0] - measured_vy[0]) > 1e-12:
        raise ValueError("Baseline replay does not start at measured Vy")
    whole = score(table, float(t[0]), float(t[-1]))
    focus = score(table, focus_start, focus_end)
    ramp = score(table, ramp_start, ramp_end)
    result = {
        "experiment": "P4-GOLF-YAW-ASSISTED-001",
        "replay_csv": str(replay_csv.resolve()),
        "baseline_json": str(baseline_json.resolve()),
        "method": "50 Hz Python midpoint replay; frozen B0; measured yaw used in front/rear slip and -Vx*r; one-step resets measured Vy at each prior sample",
        "initialization": "Measured Vy and yaw at first sample (75 s)",
        "evaluation_kind": "In-sample diagnostic with measured yaw injection, not independent validation or controller performance",
        "whole": whole,
        "focus": focus,
        "ramp": ramp,
    }
    for instant in (86.14, 86.30):
        idx = int(np.argmin(abs(t - instant)))
        result[f"at_{instant:.2f}_s"] = {k: float(table[k].iloc[idx]) for k in
                                        ("t_s", "vy_measured_mps", "vy_baseline_mps",
                                         "vy_yaw_assisted_mps", "vy_one_step_mps",
                                         "yaw_measured_radps", "yaw_baseline_radps")}
    return result, table


def plot(table, start, end, ramp_start, ramp_end, output):
    q = table[(table.t_s >= start) & (table.t_s <= end)]
    t = q.t_s
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, layout="constrained")
    axes[0].plot(t, q.yaw_measured_radps, "k", label="Measured yaw")
    axes[0].plot(t, q.yaw_baseline_radps, label="B0 free-run yaw")
    axes[0].set_ylabel("Yaw rate (rad/s)")
    axes[0].legend(fontsize=8)
    axes[1].plot(t, q.vy_measured_mps, "k", label="Measured Vy")
    axes[1].plot(t, q.vy_baseline_mps, label="B0 baseline")
    axes[1].plot(t, q.vy_yaw_assisted_mps, label="Yaw-assisted")
    axes[1].plot(t, q.vy_one_step_mps, ":", label="One-step", alpha=0.8)
    axes[1].set_ylabel("Vy (m/s)")
    axes[1].legend(ncol=2, fontsize=8)
    axes[2].plot(t, q.vy_baseline_mps - q.vy_measured_mps, label="B0 baseline")
    axes[2].plot(t, q.vy_yaw_assisted_mps - q.vy_measured_mps, label="Yaw-assisted")
    axes[2].plot(t, q.vy_one_step_mps - q.vy_measured_mps, ":", label="One-step")
    axes[2].axhline(0, color="0.4", lw=0.7)
    axes[2].set_ylabel("Vy error (m/s)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(fontsize=8)
    for axis in axes:
        axis.axvspan(ramp_start, ramp_end, color="0.8", alpha=0.2)
        axis.grid(alpha=0.25)
    fig.suptitle("Golf B0 yaw-assisted diagnosis | 85–88 s")
    fig.savefig(output, dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay_csv", type=Path)
    parser.add_argument("--baseline", type=Path, default=Path("config/golf_baselines.json"))
    parser.add_argument("--focus-start", type=float, default=85)
    parser.add_argument("--focus-end", type=float, default=88)
    parser.add_argument("--ramp-start", type=float, default=85.7)
    parser.add_argument("--ramp-end", type=float, default=86.3)
    parser.add_argument("--out", type=Path, default=Path("output/golf_yaw_assisted_001"))
    args = parser.parse_args()
    result, table = analyze(args.replay_csv, args.baseline, args.focus_start,
                            args.focus_end, args.ramp_start, args.ramp_end)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
    table.to_csv(args.out.with_suffix(".csv"), index=False)
    plot(table, args.focus_start, args.focus_end, args.ramp_start, args.ramp_end,
         args.out.with_suffix(".png"))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
