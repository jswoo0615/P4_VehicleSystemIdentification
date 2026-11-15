#!/usr/bin/env python3
"""Export frozen Golf A/A-Speed/B0 fits for a controller; never refit models."""

import argparse
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_json(path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def display_path(path):
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def require_close(actual, expected, label):
    if not math.isclose(actual, expected, rel_tol=1e-10, abs_tol=1e-12):
        raise ValueError(f"Source mismatch for {label}: {actual} != {expected}")


def build_export(baseline, validation, baseline_path, validation_path):
    if validation.get("schema") != "p4.validation_summary.v1":
        raise ValueError("Unsupported validation summary schema")
    if validation.get("assessment", {}).get("reference_model") != "B0":
        raise ValueError("Validation summary does not select B0")

    protocol = baseline["validation"]
    summary_data = validation["data"]
    if (protocol["folds"] != summary_data["folds"] or
            baseline["unique_csv_count"] != summary_data["unique_files"] or
            protocol["sample_count"] != summary_data["cv_samples_reported"]):
        raise ValueError("Baseline and validation summary describe different folds")
    require_close(protocol["sample_period_s"], summary_data["sample_period_s"], "sample period")

    a = baseline["model_a"]
    speed = baseline["model_a_speed"]
    b = baseline["model_b0"]
    metrics = validation["metrics"]
    full = validation["full_data_reference_NOT_cv"]["parameters_normalized"]
    require_close(b["pooled_file_cv_yaw_rmse_rad_per_s"], metrics["b0_yaw_rmse_radps"], "B0 yaw RMSE")
    require_close(b["pooled_file_cv_lateral_speed_rmse_m_per_s"], metrics["b0_vy_rmse_mps"], "B0 Vy RMSE")
    for baseline_key, summary_key in (
        ("cf_over_m_mps2_per_rad", "cf_over_m"),
        ("cr_over_m_mps2_per_rad", "cr_over_m"),
        ("iz_over_m_m2", "iz_over_m_m2"),
        ("lf_m", "lf_m"),
        ("lr_m", "lr_m"),
    ):
        require_close(b[baseline_key], full[summary_key], baseline_key)
    require_close(b["lf_m"] + b["lr_m"], b["nominal_wheelbase_m"], "B0 wheelbase")
    require_close(speed["nominal_wheelbase_m"], b["nominal_wheelbase_m"], "model wheelbase")
    if not all(math.isfinite(x) and x > 0 for x in (
            a["K_per_s"], a["tau_s"], speed["tau_s"], speed["nominal_wheelbase_m"],
            b["cf_over_m_mps2_per_rad"], b["cr_over_m_mps2_per_rad"],
            b["iz_over_m_m2"], b["lf_m"], b["lr_m"])):
        raise ValueError("Invalid controller coefficient")

    channels = baseline["channel_map"]
    signals = {
        "time": {"csv_column": channels["time"], "unit": "s", "role": "timestamp"},
        "steering": {"csv_column": channels["road_wheel_angle"], "unit": "rad", "role": "road-wheel input"},
        "yaw_rate": {"csv_column": channels["yaw_rate"], "unit": "rad/s", "role": "measured output"},
        "vx": {"csv_column": channels["longitudinal_speed"], "unit": "m/s", "role": "longitudinal speed input"},
        "vy": {"csv_column": channels["lateral_speed"], "unit": "m/s", "role": "Model B0 measured output"},
        "bank": {"csv_column": channels["road_bank"], "unit": "rad", "role": "Model B0 road-bank input"},
    }
    domain = baseline["observed_domain_raw_samples_after_speed_filter"]
    return {
        "schema": "p4.controller_model.v1",
        "vehicle": baseline["vehicle"],
        "models": {
            "A": {
                "equation": a["equation"],
                "inputs": ["steering"], "outputs": ["yaw_rate"],
                "parameters": {"K_r_per_s": a["K_per_s"], "tau_r_s": a["tau_s"],
                               "b_radps": a["bias_rad_per_s"]},
            },
            "A-Speed": {
                "equation": speed["equation"],
                "inputs": ["steering", "vx"], "outputs": ["yaw_rate"],
                "parameters": {"L_m": speed["nominal_wheelbase_m"],
                               "k_v_s2_per_m": speed["kv_s2_per_m"],
                               "tau_r_s": speed["tau_s"],
                               "b_radps": speed["bias_rad_per_s"]},
            },
            "B0": {
                "equation": b["equation"],
                "states": ["vy", "yaw_rate"],
                "inputs": ["steering", "vx", "bank"],
                "outputs": ["vy", "yaw_rate"],
                "bank_lateral_acceleration": "+9.80665*sin(bank)",
                "parameters": {"Cf_over_m_mps2_per_rad": b["cf_over_m_mps2_per_rad"],
                               "Cr_over_m_mps2_per_rad": b["cr_over_m_mps2_per_rad"],
                               "Iz_over_m_m2": b["iz_over_m_m2"],
                               "lf_m": b["lf_m"], "lr_m": b["lr_m"]},
                "parameter_interpretation": b["interpretation"],
            },
        },
        "signal_mapping": signals,
        "operating_domain": {
            "minimum_training_speed_mps": protocol["minimum_speed_m_s"],
            "observed_from": "raw unique CSV samples after speed filter",
            "raw_sample_count": domain["raw_sample_count"],
            "speed_mps_p05_p95": domain["speed_mps_p05_p95"],
            "abs_road_wheel_angle_rad_p95": domain["abs_road_wheel_angle_rad_p95"],
            "abs_yaw_rate_radps_p95": domain["abs_yaw_rate_radps_p95"],
            "abs_lateral_speed_mps_p95": domain["abs_lateral_speed_mps_p95"],
            "abs_bank_rad_p95": domain["abs_bank_rad_p95"],
            "interpretation": domain["note"],
        },
        "validation": {
            "method": f"{protocol['folds']}-fold grouped cross-validation by {protocol['group']}",
            "unique_files": baseline["unique_csv_count"],
            "scored_samples": protocol["sample_count"],
            "sample_period_s": protocol["sample_period_s"],
            "yaw_rmse_radps": {"A": a["pooled_file_cv_yaw_rmse_rad_per_s"],
                               "A-Speed": speed["pooled_file_cv_yaw_rmse_rad_per_s"],
                               "B0": b["pooled_file_cv_yaw_rmse_rad_per_s"]},
            "B0_lateral_speed_rmse_mps": b["pooled_file_cv_lateral_speed_rmse_m_per_s"],
            "interpretation": "Fold-specific fits were scored on held-out files; these RMSEs do not directly validate the full-data coefficient vectors above.",
        },
        "provenance": {
            "dataset": baseline["dataset"],
            "track": baseline["track"],
            "source_baseline": display_path(baseline_path),
            "source_baseline_sha256": sha256(baseline_path),
            "source_validation_summary": display_path(validation_path),
            "source_validation_summary_sha256": sha256(validation_path),
            "validation_experiment": validation["experiment"],
            "validation_source_cv_sha256": validation["input_sha256"],
            "coefficient_origin": "Full-data refits after cross-validation; not fold coefficients",
            "duplicate_files_excluded": baseline["excluded_exact_duplicates"],
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=ROOT / "config/golf_baselines.json")
    parser.add_argument("--validation", type=Path, default=ROOT / "output/golf_validation_summary.json")
    parser.add_argument("--output", type=Path, default=ROOT / "output/golf_controller_model.json")
    args = parser.parse_args()
    baseline_path, validation_path = args.baseline.resolve(), args.validation.resolve()
    document = build_export(read_json(baseline_path), read_json(validation_path),
                            baseline_path, validation_path)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=args.output.parent,
                                     prefix=".golf_controller_model.", suffix=".tmp",
                                     delete=False) as stream:
        temp_path = Path(stream.name)
        json.dump(document, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
    os.replace(temp_path, args.output)
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
