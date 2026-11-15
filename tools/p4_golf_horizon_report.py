#!/usr/bin/env python3
"""Summarize C++ Golf finite-horizon file CV and full-fit local diagnostics."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ERROR_COLUMNS = ("a_yaw_sse", "as_yaw_sse", "b_yaw_sse", "b_vy_sse")
NAMES = ("A yaw", "A-Speed yaw", "B0 yaw", "B0 Vy")


def pooled(frame):
    n = int(frame.samples.sum())
    return {"samples": n, **{name.replace("_sse", "_rmse"): float(np.sqrt(frame[name].sum() / n))
                             for name in ERROR_COLUMNS}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cv", type=Path, default=Path("output/golf_prediction_horizon_cv_001.csv"))
    parser.add_argument("--diagnostic", type=Path,
                        default=Path("output/golf_prediction_horizon_85_88_001.csv"))
    parser.add_argument("--a-reference", type=Path, default=Path("output/golf_a_kfold_001.json"))
    parser.add_argument("--b-reference", type=Path, default=Path("output/golf_b_kfold_001.json"))
    parser.add_argument("--out", type=Path, default=Path("output/golf_prediction_horizon_001"))
    args = parser.parse_args()
    cv, local = pd.read_csv(args.cv), pd.read_csv(args.diagnostic)
    a = json.loads(args.a_reference.read_text())
    b = json.loads(args.b_reference.read_text())
    if len(cv) != 100 or cv.file.nunique() != 20 or set(cv.horizon_steps) != {0, 5, 10, 25, 50}:
        raise ValueError("Expected 20 held-out files at five prediction lengths")
    if len(local) != 5 or set(local.horizon_steps) != {0, 5, 10, 25, 50}:
        raise ValueError("Expected five local diagnostic rows")
    checks = {
        "a_yaw_rmse": a["results"]["model_a"]["pooled_cv_rmse_radps"],
        "as_yaw_rmse": a["results"]["model_a_speed"]["pooled_cv_rmse_radps"],
        "b_yaw_rmse": b["pooled_yaw_rmse_radps"],
        "b_vy_rmse": b["pooled_vy_rmse_mps"],
    }
    free = pooled(cv[cv.horizon_steps == 0])
    errors = {key: abs(free[key] - expected) for key, expected in checks.items()}
    if free["samples"] != b["cv_samples"] or max(errors.values()) > 1e-12:
        raise ValueError("C++ free-run CV does not reproduce the frozen references")

    horizons = []
    for steps in (5, 10, 25, 50):
        frame = cv[cv.horizon_steps == steps].copy()
        if len(frame) != 20 or frame.fold.nunique() != 5:
            raise ValueError("Incomplete file-fold horizon result")
        frame["as_file_rmse"] = np.sqrt(frame.as_yaw_sse / frame.samples)
        frame["b_file_rmse"] = np.sqrt(frame.b_yaw_sse / frame.samples)
        fold_wins = sum(np.sqrt(group.b_yaw_sse.sum() / group.samples.sum()) <
                        np.sqrt(group.as_yaw_sse.sum() / group.samples.sum())
                        for _, group in frame.groupby("fold"))
        result = pooled(frame)
        result.update({
            "horizon_steps": steps,
            "horizon_nominal_s": steps * 0.02,
            "b0_yaw_rmse_change_vs_a_speed_pct":
                100 * (result["b_yaw_rmse"] / result["as_yaw_rmse"] - 1),
            "b0_better_yaw_files": int((frame.b_file_rmse < frame.as_file_rmse).sum()),
            "b0_better_yaw_folds": int(fold_wins),
        })
        horizons.append(result)
    local_results = []
    for steps in (5, 10, 25, 50):
        record = pooled(local[local.horizon_steps == steps])
        record["horizon_steps"] = steps
        record["horizon_nominal_s"] = steps * 0.02
        local_results.append(record)
    report = {
        "experiment": "P4-GOLF-PREDICTION-HORIZON-001",
        "method": "C++17 ControllerRuntime; original GolfDataset 50 Hz preprocessing; reset from measured Vy and yaw at each origin; use recorded future steering, Vx and bank",
        "horizons_s": [0.1, 0.2, 0.5, 1.0],
        "target_scoring": "Every valid target sample within a segment; overlapping forecasts are pooled by forecast origin, never across a segment boundary",
        "independent_evaluation": "20 unique held-out files; each model uses its own corresponding fold's training-only coefficients",
        "reference_free_run_match_max_abs_rmse": max(errors.values()),
        "file_cv": horizons,
        "local_diagnostic": {
            "target_interval_s": [85, 88],
            "file": "2023-11_1.csv",
            "coefficients": "full-data refit from config/golf_baselines.json; not independent validation",
            "results": local_results,
        },
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    x = [r["horizon_nominal_s"] for r in horizons]
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True, layout="constrained")
    for column, rows, ylabel, ax in [
        ("yaw", horizons, "Yaw RMSE (rad/s)", axes[0, 0]),
        ("vy", horizons, "B0 Vy RMSE (m/s)", axes[1, 0]),
        ("yaw", local_results, "Yaw RMSE (rad/s)", axes[0, 1]),
        ("vy", local_results, "B0 Vy RMSE (m/s)", axes[1, 1]),
    ]:
        if column == "yaw":
            for key, label in (("a_yaw_rmse", "A"), ("as_yaw_rmse", "A-Speed"),
                               ("b_yaw_rmse", "B0")):
                ax.plot(x, [row[key] for row in rows], marker="o", label=label)
            ax.legend(fontsize=8)
        else:
            ax.plot(x, [row["b_vy_rmse"] for row in rows], marker="o", color="tab:green")
        ax.set_ylabel(ylabel)
        ax.set_xticks(x)
        ax.grid(alpha=0.25)
    axes[0, 0].set_title("20-file held-out CV")
    axes[0, 1].set_title("85–88 s full-fit diagnostic")
    axes[1, 0].set_xlabel("Forecast horizon (s)")
    axes[1, 1].set_xlabel("Forecast horizon (s)")
    fig.suptitle("VW Golf measured-input finite-horizon prediction")
    fig.savefig(args.out.with_suffix(".png"), dpi=170)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
