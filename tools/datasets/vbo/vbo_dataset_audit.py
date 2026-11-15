
from pathlib import Path
import csv
import math
import statistics
import sys


def split_fields(line):
    # Prefer tabs when the file is tab-delimited.
    if "\t" in line:
        return [x.strip() for x in line.split("\t")]
    return line.split()


def number(value):
    return float(value.replace(",", "."))


def clock_seconds(value):
    raw = number(value)
    hh = int(raw // 10000)
    mm = int(raw // 100) % 100
    ss = raw - hh * 10000 - mm * 100
    return hh * 3600 + mm * 60 + ss


def load_columns_and_rows(path):
    lines = path.read_text(
        encoding="utf-8-sig", errors="replace"
    ).splitlines()

    columns = None
    data_start = None
    fmt = None

    for i, line in enumerate(lines):
        s = " ".join(line.split())

        if s == "[column names]":
            fmt = "section"
            for j in range(i + 1, len(lines)):
                if lines[j].strip():
                    columns = split_fields(lines[j])
                    break

        if s == "[data]" and fmt == "section":
            data_start = i + 1
            break

    if fmt is None:
        # Headerless format: first nonempty line is columns.
        for i, line in enumerate(lines):
            if line.strip():
                columns = split_fields(line)
                data_start = i + 1
                fmt = "headerless"
                break

    if columns is None or data_start is None:
        raise ValueError("Missing columns or data section")

    return fmt, columns, lines[data_start:]


def audit(path):
    fmt, columns, raw_rows = load_columns_and_rows(path)

    time_indices = [
        i for i, c in enumerate(columns) if c == "time"
    ]

    for name in ("velocity", "steer_ang2", "YawRate"):
        if name not in columns:
            raise ValueError(f"Missing column: {name}")

    if fmt == "headerless":
        if len(time_indices) == 2:
            time_index = time_indices[1]
            elapsed_time = True

        elif len(time_indices) == 1:
            time_index = time_indices[0]
            elapsed_time = False

        else:
            raise ValueError(
                "Unsupported time column configuration"
            )
    else:
        if len(time_indices) != 1:
            raise ValueError(
                "Expected one time column in section format"
            )

        time_index = time_indices[0]
        elapsed_time = False

    speed_index = columns.index("velocity")
    steer_index = columns.index("steer_ang2")
    yaw_index = columns.index("YawRate")

    times = []
    speeds = []
    steers = []
    yaws = []

    invalid_rows = 0

    for line in raw_rows:
        if not line.strip():
            continue

        values = split_fields(line)

        if len(values) != len(columns):
            invalid_rows += 1
            continue

        try:
            t = (
                number(values[time_index])
                if elapsed_time
                else clock_seconds(values[time_index])
            )
            v = number(values[speed_index])
            steer = number(values[steer_index])
            yaw = number(values[yaw_index])

            if not all(
                math.isfinite(x) for x in (t, v, steer, yaw)
            ):
                raise ValueError("Non-finite value")

            times.append(t)
            speeds.append(v)
            steers.append(steer)
            yaws.append(yaw)

        except (ValueError, OverflowError):
            invalid_rows += 1

    if not times:
        raise ValueError("No valid data rows")

    nonmonotonic = sum(
        times[i] <= times[i - 1]
        for i in range(1, len(times))
    )

    return {
        "file": path.name,
        "format": fmt,
        "samples": len(times),
        "invalid_rows": invalid_rows,
        "nonmonotonic_dt": nonmonotonic,
        "duration_s": round(times[-1] - times[0], 3),
        "speed_min_kmh": round(min(speeds), 3),
        "speed_mean_kmh": round(statistics.mean(speeds), 3),
        "speed_max_kmh": round(max(speeds), 3),
        "steer_min_deg": round(min(steers), 3),
        "steer_max_deg": round(max(steers), 3),
        "yaw_min_deg_s": round(min(yaws), 3),
        "yaw_max_deg_s": round(max(yaws), 3),
    }


def main():
    data_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "data/old_vito_not_used")
    paths = sorted(data_dir.glob("*.vbo"))

    if not paths:
        raise SystemExit("No VBO files found")

    results = []
    failed = []

    for path in paths:
        try:
            result = audit(path)
            results.append(result)
            print(
                f"OK   {path.name:25s} "
                f"N={result['samples']:5d} "
                f"v_mean={result['speed_mean_kmh']:7.2f} km/h"
            )
        except Exception as e:
            failed.append((path.name, str(e)))
            print(f"FAIL {path.name}: {e}")

    if results:
        output = Path("vbo_dataset_summary.csv")
        with output.open("w", newline="") as f:
            writer = csv.DictWriter(
                f, fieldnames=results[0].keys()
            )
            writer.writeheader()
            writer.writerows(results)

    print("\n=== Dataset Audit Summary ===")
    print("Files found :", len(paths))
    print("Files passed:", len(results))
    print("Files failed:", len(failed))
    print("CSV saved   : vbo_dataset_summary.csv"
          if results else "No CSV saved")

    if failed:
        print("\nFailed files:")
        for name, reason in failed:
            print(f"  {name}: {reason}")


if __name__ == "__main__":
    main()
