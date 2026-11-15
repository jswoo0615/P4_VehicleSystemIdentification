
from pathlib import Path
from collections import defaultdict
import csv
import hashlib
import math

import numpy as np
import matplotlib.pyplot as plt

if __package__:
    from .vbo_dataset_audit import (
        load_columns_and_rows,
        split_fields,
        number,
        clock_seconds,
    )
else:  # Also support direct script execution.
    from vbo_dataset_audit import (
        load_columns_and_rows,
        split_fields,
        number,
        clock_seconds,
    )

# ============================================================
# Configuration — exploratory thresholds
# ============================================================

DATA_DIR = Path("data/old_vito_not_used")
OUT_DIR = Path("output/maneuver_segments")

SMOOTH_SECONDS = 0.15

STEER_ON_DEG = 3.0
YAW_ON_DEG_S = 3.0

PADDING_SECONDS = 0.35
MERGE_GAP_SECONDS = 1.0
MIN_EVENT_SECONDS = 0.40

CIRCLE_STEER_DEG = 10.0
CIRCLE_YAW_DEG_S = 8.0
CIRCLE_MIN_FRACTION = 0.70
CIRCLE_MIN_DURATION = 8.0


# ============================================================
# 1. Read VBO signals
# ============================================================

def read_signals(path):
    fmt, columns, raw_rows = load_columns_and_rows(path)

    indices = {}

    for name in ("time", "steer_ang2", "YawRate", "velocity"):
        if name not in columns:
            raise ValueError(f"Missing column: {name}")

    time_indices = [
        i for i, name in enumerate(columns)
        if name == "time"
    ]

    elapsed_mode = fmt == "headerless" and len(time_indices) == 2

    indices["time"] = (
        time_indices[1] if elapsed_mode else time_indices[0]
    )
    indices["steer"] = columns.index("steer_ang2")
    indices["yaw"] = columns.index("YawRate")
    indices["speed"] = columns.index("velocity")

    times, steer, yaw, speed = [], [], [], []

    for line in raw_rows:
        if not line.strip():
            continue

        values = split_fields(line)

        if len(values) != len(columns):
            raise ValueError("Invalid number of columns")

        raw_time = values[indices["time"]]

        t = (
            number(raw_time)
            if elapsed_mode
            else clock_seconds(raw_time)
        )

        values_now = [
            t,
            number(values[indices["steer"]]),
            number(values[indices["yaw"]]),
            number(values[indices["speed"]]),
        ]

        if not all(math.isfinite(v) for v in values_now):
            raise ValueError("Non-finite signal value")

        times.append(values_now[0])
        steer.append(values_now[1])
        yaw.append(values_now[2])
        speed.append(values_now[3])

    if len(times) < 3:
        raise ValueError("Insufficient samples")

    t = np.asarray(times, dtype=float)
    t -= t[0]

    if np.any(np.diff(t) <= 0):
        raise ValueError("Non-increasing time")

    return {
        "time": t,
        "steer": np.asarray(steer),
        "yaw": np.asarray(yaw),
        "speed": np.asarray(speed),
    }


# ============================================================
# 2. Signal smoothing
# ============================================================

def smooth(signal, time):
    dt = float(np.median(np.diff(time)))

    window = max(1, round(SMOOTH_SECONDS / dt))
    window = int(window)

    if window % 2 == 0:
        window += 1

    pad = window // 2
    padded = np.pad(signal, (pad, pad), mode="edge")

    return np.convolve(
        padded,
        np.ones(window) / window,
        mode="valid",
    )


# ============================================================
# 3. Continuous circle candidate
# ============================================================

def is_circle_candidate(t, steer, yaw):
    duration = float(t[-1] - t[0])

    if duration < CIRCLE_MIN_DURATION:
        return False

    active = (
        (np.abs(steer) >= CIRCLE_STEER_DEG)
        & (np.abs(yaw) >= CIRCLE_YAW_DEG_S)
    )

    fraction = float(np.mean(active))

    active_idx = np.flatnonzero(active)

    if len(active_idx) == 0:
        return False

    # Check directional consistency in active samples.
    s = np.sign(np.median(steer[active_idx]))
    r = np.sign(np.median(yaw[active_idx]))

    if s == 0 or r == 0:
        return False

    directional = np.mean(
        (np.sign(steer[active_idx]) == s)
        & (np.sign(yaw[active_idx]) == r)
    )

    return (
        fraction >= CIRCLE_MIN_FRACTION
        and directional >= 0.95
    )


# ============================================================
# 4. Event detection and temporal grouping
# ============================================================

def detect_events(t, steer, yaw):
    active = (
        (np.abs(steer) >= STEER_ON_DEG)
        | (np.abs(yaw) >= YAW_ON_DEG_S)
    )

    # Expand each active point by a time margin.
    expanded = active.copy()
    active_indices = np.flatnonzero(active)

    for idx in active_indices:
        lo = np.searchsorted(
            t, t[idx] - PADDING_SECONDS, side="left"
        )
        hi = np.searchsorted(
            t, t[idx] + PADDING_SECONDS, side="right"
        )
        expanded[lo:hi] = True

    indices = np.flatnonzero(expanded)

    if len(indices) == 0:
        return []

    # Initial contiguous groups.
    groups = []
    start = previous = int(indices[0])

    for idx in indices[1:]:
        idx = int(idx)

        if idx > previous + 1:
            groups.append([start, previous])
            start = idx

        previous = idx

    groups.append([start, previous])

    # Merge nearby events.
    merged = [groups[0]]

    for start, end in groups[1:]:
        previous = merged[-1]

        gap = t[start] - t[previous[1]]

        if gap <= MERGE_GAP_SECONDS:
            previous[1] = end
        else:
            merged.append([start, end])

    return [
        (s, e)
        for s, e in merged
        if t[e] - t[s] >= MIN_EVENT_SECONDS
    ]


# ============================================================
# 5. Event classification candidates
# ============================================================

def classify_event(t, steer, yaw, start, end):
    s = steer[start:end + 1]
    r = yaw[start:end + 1]

    duration = t[end] - t[start]

    large_steer = np.max(np.abs(s)) >= 20.0

    significant = s[np.abs(s) >= STEER_ON_DEG]

    signs = np.sign(significant)

    sign_changes = int(np.count_nonzero(
        signs[1:] != signs[:-1]
    ))

    if large_steer and sign_changes == 0:
        return "J-turn candidate"

    if sign_changes >= 2:
        return "Alternating steering / DLC candidate"

    if sign_changes == 1:
        return "Steering reversal / LC-DLC candidate"

    if np.max(np.abs(r)) < YAW_ON_DEG_S:
        return "Steering event / Unknown"

    if duration >= 3.0:
        return "Sustained turn / Unknown"

    return "Short turn / Unknown"


# ============================================================
# 6. Analyze one file
# ============================================================

def analyze(path, duplicate_of=""):
    signals = read_signals(path)

    t = signals["time"]
    speed = signals["speed"]

    steer = smooth(signals["steer"], t)
    yaw = smooth(signals["yaw"], t)

    if is_circle_candidate(t, steer, yaw):
        events = [(0, len(t) - 1)]
        labels = ["Circle candidate"]
    else:
        events = detect_events(t, steer, yaw)
        labels = [
            classify_event(t, steer, yaw, s, e)
            for s, e in events
        ]

    rows = []

    for event_id, ((start, end), label) in enumerate(
        zip(events, labels), start=1
    ):
        segment_steer = steer[start:end + 1]
        segment_yaw = yaw[start:end + 1]
        segment_speed = speed[start:end + 1]

        rows.append({
            "file": path.name,
            "duplicate_of": duplicate_of,
            "event_id": event_id,
            "start_s": round(float(t[start]), 3),
            "end_s": round(float(t[end]), 3),
            "duration_s": round(float(t[end] - t[start]), 3),
            "mean_speed_kmh": round(
                float(np.mean(segment_speed)), 3
            ),
            "max_abs_steer_deg": round(
                float(np.max(np.abs(segment_steer))), 3
            ),
            "max_abs_yaw_deg_s": round(
                float(np.max(np.abs(segment_yaw))), 3
            ),
            "candidate": label,
            "status": "UNVERIFIED",
        })

    # Diagnostic plot
    fig, axes = plt.subplots(
        3, 1, figsize=(12, 8), sharex=True
    )

    traces = [
        (steer, "Steering (deg)"),
        (yaw, "Yaw rate (deg/s)"),
        (speed, "Speed (km/h)"),
    ]

    for ax, (values, label) in zip(axes, traces):
        ax.plot(t, values, linewidth=1.0)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.25)

        for start, end in events:
            ax.axvspan(
                t[start], t[end],
                alpha=0.15,
                color="gray",
            )

    axes[-1].set_xlabel("Time (s)")

    fig.suptitle(
        f"{path.stem} | {len(events)} detected events"
    )
    fig.tight_layout()

    output = OUT_DIR / f"{path.stem}_segments.png"
    fig.savefig(output, dpi=130)
    plt.close(fig)

    return rows


# ============================================================
# 7. Batch process with duplicate detection
# ============================================================

def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    files = sorted(DATA_DIR.glob("vblog_*.vbo"))

    first_by_hash = {}

    all_rows = []
    failures = []
    duplicates = []

    print("=== P4 Maneuver Segmentation ===")
    print("Files found:", len(files))

    for path in files:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()

        if digest in first_by_hash:
            duplicate = first_by_hash[digest]
            duplicates.append((path.name, duplicate))

            print(f"DUP {path.name} = {duplicate}")
            continue

        first_by_hash[digest] = path.name

        try:
            rows = analyze(path)
            all_rows.extend(rows)

            print(
                f"OK  {path.name}: "
                f"{len(rows)} event(s)"
            )

        except Exception as exc:
            failures.append((path.name, str(exc)))
            print(f"FAIL {path.name}: {exc}")

    columns = [
        "file", "duplicate_of", "event_id",
        "start_s", "end_s", "duration_s",
        "mean_speed_kmh",
        "max_abs_steer_deg",
        "max_abs_yaw_deg_s",
        "candidate", "status",
    ]

    output_csv = OUT_DIR / "maneuver_segments.csv"

    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        writer.writerows(all_rows)

    duplicate_csv = OUT_DIR / "duplicate_files.csv"

    with duplicate_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["file", "duplicate_of"])
        writer.writerows(duplicates)

    print("\n=== Summary ===")
    print("Input files    :", len(files))
    print("Unique files   :", len(first_by_hash))
    print("Processed files:", len(first_by_hash) - len(failures))
    print("Total events   :", len(all_rows))
    print("Duplicates     :", len(duplicates))
    print("Failures       :", len(failures))
    print("CSV saved      :", output_csv)

    for file, reason in failures:
        print("FAIL:", file, reason)


if __name__ == "__main__":
    main()
