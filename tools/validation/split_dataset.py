
#!/usr/bin/env python3

import argparse
import random
from collections import Counter, defaultdict
from pathlib import Path

if __package__:
    from .csv_io import read_csv, write_csv
else:  # Also support direct script execution.
    from csv_io import read_csv, write_csv

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--input",
        type=Path,
        default=Path("output/dataset_manifest.csv"),
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output"),
    )

    parser.add_argument(
        "--test-ratio",
        type=float,
        default=0.20,
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=42,
    )

    args = parser.parse_args()

    if not 0 < args.test_ratio < 1:
        raise ValueError("test-ratio must be in (0,1)")

    rows = read_csv(args.input)

    if not rows:
        raise ValueError("Empty dataset manifest")

    fields = list(rows[0].keys())

    # All events from one file belong to one group.
    groups = defaultdict(list)

    for row in rows:
        groups[row["group_id"]].append(row)

    # Initial stratification:
    # use the most frequent maneuver within each group.
    # Mixed-maneuver groups remain intact.
    strata = defaultdict(list)

    for group_id, group_rows in groups.items():
        counts = Counter(
            row["maneuver"] for row in group_rows
        )

        dominant_maneuver = sorted(
            counts.items(),
            key=lambda item: (-item[1], item[0])
        )[0][0]

        strata[dominant_maneuver].append(group_id)

    rng = random.Random(args.seed)

    train_groups = set()
    test_groups = set()

    for maneuver in sorted(strata):
        ids = sorted(strata[maneuver])
        rng.shuffle(ids)

        if len(ids) == 1:
            # One group cannot simultaneously belong
            # to train and test.
            train_groups.update(ids)
            continue

        test_count = max(
            1,
            round(len(ids) * args.test_ratio)
        )
        test_count = min(test_count, len(ids) - 1)

        test_groups.update(ids[:test_count])
        train_groups.update(ids[test_count:])

    if train_groups & test_groups:
        raise RuntimeError("Train/test group leakage")

    train_rows = [
        row for row in rows
        if row["group_id"] in train_groups
    ]

    test_rows = [
        row for row in rows
        if row["group_id"] in test_groups
    ]

    if not train_rows or not test_rows:
        raise RuntimeError(
            "Could not create nonempty train/test sets"
        )

    train_path = args.output_dir / "train_manifest.csv"
    test_path = args.output_dir / "test_manifest.csv"

    write_csv(train_path, train_rows, fields)
    write_csv(test_path, test_rows, fields)

    print("\n=== DATASET SPLIT ===")
    print("Seed             :", args.seed)
    print("Train groups     :", len(train_groups))
    print("Test groups      :", len(test_groups))
    print("Train events     :", len(train_rows))
    print("Test events      :", len(test_rows))

    print("\n=== MANEUVER DISTRIBUTION ===")

    train_count = Counter(
        row["maneuver"] for row in train_rows
    )
    test_count = Counter(
        row["maneuver"] for row in test_rows
    )

    for maneuver in sorted(train_count.keys() | test_count.keys()):
        print(
            f"{maneuver:20s} "
            f"Train={train_count[maneuver]:3d} "
            f"Test={test_count[maneuver]:3d}"
        )

    print("\nTrain:", train_path)
    print("Test :", test_path)


if __name__ == "__main__":
    main()
