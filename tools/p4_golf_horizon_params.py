#!/usr/bin/env python3
"""Convert existing training-only Golf fold fits to a checked C++ parameter table."""

import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--a", type=Path, default=Path("output/golf_a_kfold_001.json"))
    parser.add_argument("--b", type=Path, default=Path("output/golf_b_kfold_001.json"))
    parser.add_argument("--out", type=Path, default=Path("output/golf_horizon_fold_params.csv"))
    args = parser.parse_args()
    a = json.loads(args.a.read_text())
    b = json.loads(args.b.read_text())
    af = a["results"]["model_a"]["fold_results"]
    sf = a["results"]["model_a_speed"]["fold_results"]
    bf = b["fold_results"]
    if len(af) != len(sf) or len(af) != len(bf) or len(af) != 5:
        raise ValueError("Expected five matching folds")
    rows = []
    all_files = set()
    for x, y, z in zip(af, sf, bf):
        names = x["test_files"]
        if x["fold"] != y["fold"] or x["fold"] != z["fold"]:
            raise ValueError("Fold IDs differ")
        if names != y["test_files"] or names != [r["file"] for r in z["file_results"]]:
            raise ValueError("Held-out file assignments differ")
        if all_files.intersection(names):
            raise ValueError("Held-out file occurs in multiple folds")
        all_files.update(names)
        p = z["parameters"]
        rows.append([x["fold"], "|".join(names), x["K_r_per_s"], x["tau_s"],
                     x["b_radps"], a["wheelbase_nominal_m"], y["k_v_s2_per_m"],
                     y["tau_s"], y["b_radps"], p["cf_over_m"], p["cr_over_m"],
                     p["iz_over_m_m2"], p["lf_m"], p["lr_m"]])
    if len(all_files) != a["unique_files"] or len(all_files) != b["unique_files"]:
        raise ValueError("Not all unique files are represented")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["fold", "held_files", "a_gain", "a_tau", "a_bias", "as_wheelbase",
                         "as_kv", "as_tau", "as_bias", "b_cf_over_m", "b_cr_over_m",
                         "b_iz_over_m", "b_lf", "b_lr"])
        writer.writerows(rows)
    print(f"Wrote {len(rows)} training-only folds and {len(all_files)} unique held-out files to {args.out}")


if __name__ == "__main__":
    main()
