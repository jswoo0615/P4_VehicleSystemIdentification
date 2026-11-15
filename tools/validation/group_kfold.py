
#!/usr/bin/env python3

import argparse
import random

from collections import Counter, defaultdict
from pathlib import Path

if __package__:
    from .csv_io import read_csv, write_csv
else:  # Also support direct script execution.
    from csv_io import read_csv, write_csv


VERSION = "Balanced Group K-Fold V2"


# ============================================================
# 2. Balanced Group K-Fold
# ============================================================

def make_folds(rows, k, seed):

    groups = defaultdict(list)

    for row in rows:
        groups[row["group_id"]].append(row)

    if len(groups) < k:
        raise ValueError(
            "Number of groups must be >= K"
        )

    rng = random.Random(seed)

    group_ids = sorted(groups)
    rng.shuffle(group_ids)

    # Larger groups first.
    # Stable sort preserves seeded random ordering
    # for groups having the same size.
    group_ids.sort(
        key=lambda g: len(groups[g]),
        reverse=True
    )

    fold_groups = [[] for _ in range(k)]
    fold_events = [0 for _ in range(k)]
    fold_maneuvers = [
        Counter() for _ in range(k)
    ]

    total_maneuvers = Counter(
        row["maneuver"]
        for row in rows
    )

    target_maneuvers = {
        maneuver: count / k
        for maneuver, count
        in total_maneuvers.items()
    }

    # --------------------------------------------------------
    # Assign each group to exactly one fold.
    # --------------------------------------------------------

    for group_id in group_ids:

        group_rows = groups[group_id]

        group_size = len(group_rows)

        group_counts = Counter(
            row["maneuver"]
            for row in group_rows
        )

        # PRIORITY 1:
        # Choose only folds with minimum event load.
        minimum_load = min(fold_events)

        candidates = [
            f for f in range(k)
            if fold_events[f] == minimum_load
        ]

        # PRIORITY 2:
        # Among equally loaded folds,
        # choose the best maneuver balance.
        def maneuver_score(fold):

            score = 0.0

            for maneuver, target in target_maneuvers.items():

                after = (
                    fold_maneuvers[fold][maneuver]
                    + group_counts[maneuver]
                )

                score += (
                    (after - target) ** 2
                    / max(target, 1.0)
                )

            return score

        chosen = min(
            candidates,
            key=lambda f: (
                maneuver_score(f),
                len(fold_groups[f]),
                f
            )
        )

        fold_groups[chosen].append(group_id)

        fold_events[chosen] += group_size

        fold_maneuvers[chosen].update(
            group_counts
        )

    assignment = {}

    for fold in range(k):
        for group_id in fold_groups[fold]:

            if group_id in assignment:
                raise RuntimeError(
                    "Duplicate group assignment"
                )

            assignment[group_id] = fold

    if len(assignment) != len(groups):
        raise RuntimeError(
            "Some groups were not assigned"
        )

    return assignment


# ============================================================
# 3. Generate train/test manifests
# ============================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--input",
        type=Path,
        default=Path(
            "output/dataset_manifest.csv"
        )
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output/cv")
    )

    parser.add_argument(
        "--folds",
        type=int,
        default=5
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=42
    )

    args = parser.parse_args()

    print(f"\n=== {VERSION} ===")

    if args.folds < 2:
        raise ValueError(
            "K must be at least 2"
        )

    rows = read_csv(args.input)

    if not rows:
        raise ValueError(
            "Dataset manifest is empty"
        )

    required = {
        "file",
        "group_id",
        "event_id",
        "start_s",
        "end_s",
        "maneuver"
    }

    fields = list(rows[0].keys())

    if not required.issubset(fields):
        raise ValueError(
            "Missing manifest columns: "
            + str(required - set(fields))
        )

    # Check that a source file belongs to one group.
    file_to_group = {}

    for row in rows:

        file_name = row["file"]
        group_id = row["group_id"]

        if (
            file_name in file_to_group
            and file_to_group[file_name] != group_id
        ):
            raise RuntimeError(
                f"File belongs to multiple groups: "
                f"{file_name}"
            )

        file_to_group[file_name] = group_id

    # Ensure unique event identities.
    event_keys = [
        (row["group_id"], row["event_id"])
        for row in rows
    ]

    if len(event_keys) != len(set(event_keys)):
        raise RuntimeError(
            "Duplicate event identities"
        )

    # --------------------------------------------------------
    # Perform split
    # --------------------------------------------------------

    assignment = make_folds(
        rows=rows,
        k=args.folds,
        seed=args.seed
    )

    assignment_rows = [
        {
            "group_id": group_id,
            "fold": fold + 1
        }
        for group_id, fold
        in sorted(assignment.items())
    ]

    write_csv(
        args.output_dir / "fold_assignments.csv",
        assignment_rows,
        ["group_id", "fold"]
    )

    all_test_keys = set()
    fold_test_counts = []

    print(
        f"\nTotal events : {len(rows)}"
    )
    print(
        f"Total groups : {len(assignment)}"
    )
    print(
        f"Seed         : {args.seed}"
    )

    # --------------------------------------------------------
    # Write each fold
    # --------------------------------------------------------

    for fold in range(args.folds):

        fold_dir = (
            args.output_dir
            / f"fold_{fold + 1}"
        )

        train_rows = [
            row for row in rows
            if assignment[row["group_id"]] != fold
        ]

        test_rows = [
            row for row in rows
            if assignment[row["group_id"]] == fold
        ]

        if not train_rows or not test_rows:
            raise RuntimeError(
                f"Empty train/test in fold {fold + 1}"
            )

        train_groups = {
            row["group_id"]
            for row in train_rows
        }

        test_groups = {
            row["group_id"]
            for row in test_rows
        }

        if not train_groups.isdisjoint(test_groups):
            raise RuntimeError(
                "Train/Test leakage detected"
            )

        for row in test_rows:

            key = (
                row["group_id"],
                row["event_id"]
            )

            if key in all_test_keys:
                raise RuntimeError(
                    "Event tested more than once"
                )

            all_test_keys.add(key)

        write_csv(
            fold_dir / "train_manifest.csv",
            train_rows,
            fields
        )

        write_csv(
            fold_dir / "test_manifest.csv",
            test_rows,
            fields
        )

        fold_test_counts.append(
            len(test_rows)
        )

        print(
            f"\n=== FOLD {fold + 1} ==="
        )

        print(
            f"Train Groups : {len(train_groups)}"
            f" | Events : {len(train_rows)}"
        )

        print(
            f"Test Groups  : {len(test_groups)}"
            f" | Events : {len(test_rows)}"
        )

        maneuver_counts = Counter(
            row["maneuver"]
            for row in test_rows
        )

        for maneuver, count in sorted(
            maneuver_counts.items()
        ):
            print(
                f"  {maneuver}: {count}"
            )

    # --------------------------------------------------------
    # Final checks
    # --------------------------------------------------------

    if len(all_test_keys) != len(rows):
        raise RuntimeError(
            "Some events were not tested"
        )

    minimum = min(fold_test_counts)
    maximum = max(fold_test_counts)

    print(
        "\n=== FOLD BALANCE ==="
    )

    print(
        "Test Event Counts :",
        fold_test_counts
    )

    print(
        "Minimum           :", minimum
    )

    print(
        "Maximum           :", maximum
    )

    print(
        "Spread            :",
        maximum - minimum
    )

    print(
        f"\nPASS: All {len(rows)} events "
        "assigned to exactly one test fold."
    )


if __name__ == "__main__":
    main()
