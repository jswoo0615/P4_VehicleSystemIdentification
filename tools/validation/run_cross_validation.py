
#!/usr/bin/env python3

import argparse
import json
import math
import subprocess
from collections import defaultdict
from pathlib import Path

if __package__:
    from .csv_io import read_csv, write_csv
else:  # Also support direct script execution.
    from csv_io import read_csv, write_csv


def run(command, log_path):
    result = subprocess.run(
        [str(x) for x in command],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )

    log_path.write_text(
        result.stdout, encoding="utf-8"
    )

    if result.returncode != 0:
        raise RuntimeError(
            f"Command failed: {' '.join(map(str, command))}\n"
            f"{result.stdout}"
        )

    return result.stdout


def prediction_metrics(path):
    rows = read_csv(path)

    if not rows:
        raise RuntimeError(f"Empty prediction: {path}")

    errors = [float(r["error"]) for r in rows]
    measured = [
        float(r["yaw_measured"]) for r in rows
    ]

    n = len(errors)
    sse = sum(e * e for e in errors)
    sae = sum(abs(e) for e in errors)

    mean = sum(measured) / n
    sst = sum((y - mean) ** 2 for y in measured)

    return {
        "samples": n,
        "sse": sse,
        "sae": sae,
        "rmse": math.sqrt(sse / n),
        "mae": sae / n,
        "r2": (
            1.0 - sse / sst
            if sst > 1e-12 else float("nan")
        ),
    }


def aggregate(rows):
    # Same equal-event-weight RMSE as C++ Trainer.
    return {
        "events": len(rows),
        "rmse": math.sqrt(
            sum(r["rmse"] ** 2 for r in rows)
            / len(rows)
        ),
        "mae": (
            sum(r["mae"] for r in rows)
            / len(rows)
        ),
    }


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--cv-dir", type=Path,
        default=Path("output/cv")
    )
    parser.add_argument(
        "--build-dir", type=Path,
        default=Path("build")
    )
    parser.add_argument(
        "--folds", type=int, default=5
    )

    args = parser.parse_args()

    trainer = (
        args.build_dir / "model_a_train"
    ).resolve()
    evaluator = (
        args.build_dir / "model_a_evaluate"
    ).resolve()

    if not trainer.is_file() or not evaluator.is_file():
        raise FileNotFoundError(
            "Build model_a_train and "
            "model_a_evaluate first"
        )

    fold_results = []
    event_results = []
    seen_events = set()

    for fold in range(1, args.folds + 1):
        fold_dir = args.cv_dir / f"fold_{fold}"

        train_manifest = (
            fold_dir / "train_manifest.csv"
        )
        test_manifest = (
            fold_dir / "test_manifest.csv"
        )
        model_path = fold_dir / "model_a.json"
        predictions = fold_dir / "predictions"

        predictions.mkdir(
            parents=True, exist_ok=True
        )

        # Remove stale prediction files.
        for old_file in predictions.glob("*.csv"):
            old_file.unlink()

        print(f"\n=== RUN FOLD {fold} ===", flush=True)

        # 1. Train exclusively on this fold's train set.
        run(
            [trainer, train_manifest, model_path],
            fold_dir / "training.log"
        )

        # 2. Read the saved model.
        model = json.loads(
            model_path.read_text(encoding="utf-8")
        )
        kr = float(model["parameters"]["Kr"])
        tau = float(model["parameters"]["tau_r"])

        # 3. Evaluate without fitting.
        run(
            [
                evaluator,
                model_path,
                test_manifest,
                predictions
            ],
            fold_dir / "evaluation.log"
        )

        test_rows = read_csv(test_manifest)
        fold_event_metrics = []

        for event in test_rows:
            group = event["group_id"]
            event_id = event["event_id"]

            key = (group, event_id)

            if key in seen_events:
                raise RuntimeError(
                    f"Duplicate OOF event: {key}"
                )
            seen_events.add(key)

            prediction_path = predictions / (
                f"{group}_event_{event_id}.csv"
            )

            metrics = prediction_metrics(
                prediction_path
            )

            record = {
                "fold": fold,
                "group_id": group,
                "event_id": event_id,
                "maneuver": event["maneuver"],
                "Kr": kr,
                "tau_r": tau,
                **metrics,
            }

            fold_event_metrics.append(record)
            event_results.append(record)

        result = aggregate(fold_event_metrics)

        fold_results.append({
            "fold": fold,
            "train_events": len(
                read_csv(train_manifest)
            ),
            "test_events": result["events"],
            "Kr": kr,
            "tau_r": tau,
            "test_rmse": result["rmse"],
            "test_mae": result["mae"],
        })

        print(
            f"Kr={kr:.6f} | tau={tau:.6f}s | "
            f"RMSE={result['rmse']:.6f} deg/s | "
            f"Events={result['events']}"
        )

    # Save fold-level metrics.
    write_csv(
        args.cv_dir / "cross_validation_summary.csv",
        fold_results,
        [
            "fold", "train_events", "test_events",
            "Kr", "tau_r", "test_rmse", "test_mae"
        ],
    )

    # Save every event's out-of-fold prediction metrics.
    write_csv(
        args.cv_dir / "oof_event_metrics.csv",
        event_results,
        [
            "fold", "group_id", "event_id",
            "maneuver", "Kr", "tau_r",
            "samples", "sse", "sae",
            "rmse", "mae", "r2"
        ],
    )

    overall = aggregate(event_results)

    # Sample-weighted overall OOF RMSE as additional
    # information. This is not the training objective.
    total_samples = sum(
        r["samples"] for r in event_results
    )
    pooled_rmse = math.sqrt(
        sum(r["sse"] for r in event_results)
        / total_samples
    )

    by_maneuver = defaultdict(list)

    for record in event_results:
        by_maneuver[record["maneuver"]].append(record)

    maneuver_rows = []

    for name, records in sorted(by_maneuver.items()):
        metrics = aggregate(records)
        maneuver_rows.append({
            "maneuver": name,
            "events": metrics["events"],
            "rmse": metrics["rmse"],
            "mae": metrics["mae"],
        })

    write_csv(
        args.cv_dir / "oof_maneuver_summary.csv",
        maneuver_rows,
        ["maneuver", "events", "rmse", "mae"],
    )

    fold_rmses = [
        r["test_rmse"] for r in fold_results
    ]

    mean_fold_rmse = (
        sum(fold_rmses) / len(fold_rmses)
    )

    std_fold_rmse = math.sqrt(
        sum(
            (x - mean_fold_rmse) ** 2
            for x in fold_rmses
        ) / len(fold_rmses)
    )

    print("\n=== GROUP K-FOLD CV SUMMARY ===")
    print(f"OOF events          : {len(event_results)}")
    print(f"Equal-event OOF RMSE: {overall['rmse']:.6f}")
    print(f"Equal-event OOF MAE : {overall['mae']:.6f}")
    print(f"Pooled sample RMSE  : {pooled_rmse:.6f}")
    print(f"Mean fold RMSE      : {mean_fold_rmse:.6f}")
    print(f"Std fold RMSE       : {std_fold_rmse:.6f}")

    print("\n=== OOF BY MANEUVER ===")
    for r in maneuver_rows:
        print(
            f"{r['maneuver']} | "
            f"Events={r['events']} | "
            f"RMSE={r['rmse']:.6f} | "
            f"MAE={r['mae']:.6f}"
        )


if __name__ == "__main__":
    main()
