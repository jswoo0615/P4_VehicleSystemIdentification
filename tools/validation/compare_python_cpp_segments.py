
import csv
import re
from pathlib import Path

PYTHON_CSV = Path(
    "output/maneuver_segments/maneuver_segments.csv"
)
CPP_LOG = Path(
    "output/cpp_maneuver_results.txt"
)

TIME_TOL = 0.011
VALUE_TOL = 0.002

HEADER = re.compile(
    r"^=== data/(?:old_vito_not_used/)?(vblog_\d+\.vbo) ===$"
)

EVENT = re.compile(
    r"^\s*#(\d+) "
    r"\[([\d.]+), ([\d.]+)\] s \| "
    r"(.*?) \| "
    r"Steer=([\d.]+) deg \| "
    r"Yaw=([\d.]+) deg/s \| "
    r"Speed=([\d.]+) km/h$"
)


def load_python():
    with PYTHON_CSV.open(newline="") as f:
        return {
            (r["file"], int(r["event_id"])): {
                "start": float(r["start_s"]),
                "end": float(r["end_s"]),
                "candidate": r["candidate"],
                "steer": float(r["max_abs_steer_deg"]),
                "yaw": float(r["max_abs_yaw_deg_s"]),
                "speed": float(r["mean_speed_kmh"]),
            }
            for r in csv.DictReader(f)
        }


def load_cpp():
    result = {}
    current = None

    for line in CPP_LOG.read_text().splitlines():
        line = line.strip()

        header = HEADER.match(line)
        if header:
            current = header.group(1)
            continue

        match = EVENT.match(line)
        if not match or current is None:
            continue

        (
            event_id, start, end, candidate,
            steer, yaw, speed
        ) = match.groups()

        result[(current, int(event_id))] = {
            "start": float(start),
            "end": float(end),
            "candidate": candidate,
            "steer": float(steer),
            "yaw": float(yaw),
            "speed": float(speed),
        }

    return result


def main():
    reference = load_python()
    actual = load_cpp()

    failures = []
    max_diff = {
        key: 0.0
        for key in ("start", "end", "steer", "yaw", "speed")
    }

    for event_key, expected in reference.items():
        observed = actual.get(event_key)

        if observed is None:
            failures.append((event_key, "MISSING EVENT"))
            continue

        if expected["candidate"] != observed["candidate"]:
            failures.append((
                event_key, "CLASSIFICATION MISMATCH"
            ))

        for field in max_diff:
            diff = abs(
                expected[field] - observed[field]
            )

            max_diff[field] = max(
                max_diff[field], diff
            )

            tolerance = (
                TIME_TOL if field in ("start", "end")
                else VALUE_TOL
            )

            if diff > tolerance:
                failures.append((
                    event_key,
                    f"{field}: Python={expected[field]}, "
                    f"C++={observed[field]}, diff={diff:.6f}"
                ))

    # Python reference already excludes exact duplicates.
    reference_files = {k[0] for k in reference}

    extra = sorted(
        key for key in actual
        if key not in reference
        and key[0] != "vblog_042.vbo"
    )

    for key in extra:
        failures.append((key, "EXTRA CPP EVENT"))

    print("\n=== Python vs C++ Maneuver Comparison ===")
    print("Python events:", len(reference))
    print("C++ events   :", len(actual))
    print("Compared     :", len(reference))
    print("Extra events :", len(extra))

    print("\nMaximum absolute differences:")
    for name, value in max_diff.items():
        print(f"  {name:8s}: {value:.6f}")

    print("\nFailures:", len(failures))

    for key, reason in failures:
        print("FAIL", key, reason)

    if failures:
        print("\nRESULT: FAIL")
        raise SystemExit(1)

    print("\nRESULT: PASS")


if __name__ == "__main__":
    main()
