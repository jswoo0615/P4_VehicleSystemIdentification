
#!/usr/bin/env python3

import argparse
import csv
import math
import re
from collections import Counter, defaultdict
from pathlib import Path

HEADER_RE = re.compile(
    r"^=== (.+\.vbo) ===$"
)

COUNT_RE = re.compile(
    r"^(Events|Excursions|Phases)\s*:\s*(\d+)$"
)

PHASE_RE = re.compile(
    r"^Event\s+(\d+)\s*/\s*Phase\s+(\d+)"
    r"\s+\[([-\d.]+),\s*([-\d.]+)\]\s*s"
    r"\s*\|\s*([A-Z_]+)"
    r"\s*\|\s*Steer=([-\d.]+)"
    r"\s*->\s*([-\d.]+)\s*deg$"
)

TIME_TOL = 0.002
SHORT_PHASE_S = 0.10
LONG_REVERSAL_S = 1.0
ACTIVE_STEER_DEG = 3.0


def issue(level, filename, event_id, phase_id, code, detail):
    return {
        "level": level,
        "file": filename,
        "event_id": event_id,
        "phase_id": phase_id,
        "code": code,
        "detail": detail,
    }


def parse_log(path):
    records = {}
    current = None

    for line_number, raw in enumerate(
        path.read_text(encoding="utf-8").splitlines(),
        start=1
    ):
        line = raw.strip()

        header = HEADER_RE.match(line)
        if header:
            current = Path(header.group(1)).name

            if current in records:
                raise ValueError(
                    f"Duplicate file section: {current}"
                )

            records[current] = {
                "counts": {},
                "phases": [],
            }
            continue

        if current is None:
            continue

        count = COUNT_RE.match(line)
        if count:
            records[current]["counts"][
                count.group(1)
            ] = int(count.group(2))
            continue

        match = PHASE_RE.match(line)
        if match:
            (
                event_id, phase_id,
                start, end, phase_type,
                steer_start, steer_end
            ) = match.groups()

            records[current]["phases"].append({
                "event_id": int(event_id),
                "phase_id": int(phase_id),
                "start": float(start),
                "end": float(end),
                "type": phase_type,
                "steer_start": float(steer_start),
                "steer_end": float(steer_end),
            })
            continue

        if line.startswith("Event "):
            raise ValueError(
                f"Unrecognized phase at line {line_number}: "
                f"{line}"
            )

    if not records:
        raise ValueError("No VBO sections found")

    return records


def validate_file(filename, record):
    problems = []
    phases = record["phases"]
    counts = record["counts"]

    # --------------------------------------------
    # 1. Count integrity
    # --------------------------------------------
    for name in ("Events", "Excursions", "Phases"):
        if name not in counts:
            problems.append(issue(
                "ERROR", filename, 0, 0,
                "MISSING_COUNT", name
            ))

    if counts.get("Phases") != len(phases):
        problems.append(issue(
            "ERROR", filename, 0, 0,
            "PHASE_COUNT",
            f"declared={counts.get('Phases')}, "
            f"parsed={len(phases)}"
        ))

    # --------------------------------------------
    # 2. Phase ID and intervals
    # --------------------------------------------
    for expected_id, phase in enumerate(phases, start=1):
        event_id = phase["event_id"]
        phase_id = phase["phase_id"]

        if phase_id != expected_id:
            problems.append(issue(
                "ERROR", filename, event_id, phase_id,
                "PHASE_ID",
                f"expected={expected_id}"
            ))

        start = phase["start"]
        end = phase["end"]
        duration = end - start

        if not all(
            math.isfinite(value)
            for value in (
                start, end,
                phase["steer_start"],
                phase["steer_end"]
            )
        ):
            problems.append(issue(
                "ERROR", filename, event_id, phase_id,
                "NONFINITE", "Non-finite phase value"
            ))
            continue

        if duration <= 0.0:
            problems.append(issue(
                "ERROR", filename, event_id, phase_id,
                "INVALID_DURATION",
                f"duration={duration:.3f}s"
            ))
            continue

        # ----------------------------------------
        # 3. Physical warning rules
        # ----------------------------------------
        if duration < SHORT_PHASE_S:
            problems.append(issue(
                "WARN", filename, event_id, phase_id,
                "SHORT_PHASE",
                f"duration={duration:.3f}s"
            ))

        phase_type = phase["type"]
        steer0 = phase["steer_start"]
        steer1 = phase["steer_end"]

        if phase_type == "HOLD":
            if max(abs(steer0), abs(steer1)) < ACTIVE_STEER_DEG:
                problems.append(issue(
                    "WARN", filename, event_id, phase_id,
                    "LOW_STEER_HOLD",
                    f"steer={steer0:.3f} -> "
                    f"{steer1:.3f} deg"
                ))

        elif phase_type == "REVERSAL":
            if steer0 * steer1 >= 0:
                problems.append(issue(
                    "WARN", filename, event_id, phase_id,
                    "REVERSAL_SIGN",
                    f"steer={steer0:.3f} -> "
                    f"{steer1:.3f} deg"
                ))

            if duration > LONG_REVERSAL_S:
                problems.append(issue(
                    "WARN", filename, event_id, phase_id,
                    "LONG_REVERSAL",
                    f"duration={duration:.3f}s"
                ))

        elif phase_type == "SUSTAINED_TURN":
            if steer0 * steer1 < 0:
                problems.append(issue(
                    "WARN", filename, event_id, phase_id,
                    "CIRCLE_SIGN_CHANGE",
                    f"steer={steer0:.3f} -> "
                    f"{steer1:.3f} deg"
                ))

    # --------------------------------------------
    # 4. Continuity within each event
    # --------------------------------------------
    by_event = defaultdict(list)

    for phase in phases:
        by_event[phase["event_id"]].append(phase)

    for event_id, event_phases in by_event.items():
        event_phases.sort(key=lambda p: p["start"])

        for previous, current in zip(
            event_phases, event_phases[1:]
        ):
            gap = current["start"] - previous["end"]

            if abs(gap) > TIME_TOL:
                code = (
                    "PHASE_GAP"
                    if gap > 0
                    else "PHASE_OVERLAP"
                )

                problems.append(issue(
                    "ERROR",
                    filename,
                    event_id,
                    current["phase_id"],
                    code,
                    f"delta={gap:.3f}s"
                ))

    # Compare declared event count with nonempty events.
    # Event count zero is allowed.
    if counts.get("Events") is not None:
        if len(by_event) != counts["Events"]:
            problems.append(issue(
                "ERROR", filename, 0, 0,
                "EVENT_COVERAGE",
                f"declared={counts['Events']}, "
                f"represented={len(by_event)}"
            ))

    return problems


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--input",
        type=Path,
        default=Path("output/cpp_phase_results.txt")
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("output/phase_qualification.csv")
    )

    args = parser.parse_args()

    records = parse_log(args.input)

    problems = []

    for filename, record in records.items():
        problems.extend(
            validate_file(filename, record)
        )

    # vblog_042 is a known exact duplicate of 035
    # from the previous dataset audit.
    unique_records = {
        name: record
        for name, record in records.items()
        if name != "vblog_042.vbo"
    }

    event_total = sum(
        record["counts"].get("Events", 0)
        for record in unique_records.values()
    )

    excursion_total = sum(
        record["counts"].get("Excursions", 0)
        for record in unique_records.values()
    )

    phase_total = sum(
        len(record["phases"])
        for record in unique_records.values()
    )

    errors = [
        item for item in problems
        if item["level"] == "ERROR"
    ]

    warnings = [
        item for item in problems
        if item["level"] == "WARN"
    ]

    args.output.parent.mkdir(
        parents=True, exist_ok=True
    )

    with args.output.open(
        "w", newline="", encoding="utf-8"
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "level", "file", "event_id",
                "phase_id", "code", "detail"
            ]
        )
        writer.writeheader()
        writer.writerows(problems)

    print("\n=== P4 Phase Qualification ===")
    print("Files      :", len(records))
    print("Unique     :", len(unique_records))
    print("Events     :", event_total)
    print("Excursions :", excursion_total)
    print("Phases     :", phase_total)

    print("\n=== Structural Integrity ===")
    print("Errors     :", len(errors))

    for item in errors:
        print(
            f"ERROR {item['file']} "
            f"E{item['event_id']} "
            f"P{item['phase_id']} "
            f"{item['code']}: {item['detail']}"
        )

    print("\n=== Physical Warnings ===")
    print("Warnings   :", len(warnings))

    by_code = Counter(
        item["code"] for item in warnings
    )

    for code, count in sorted(by_code.items()):
        print(f"  {code}: {count}")

    for item in warnings:
        print(
            f"WARN {item['file']} "
            f"E{item['event_id']} "
            f"P{item['phase_id']} "
            f"{item['code']}: {item['detail']}"
        )

    print("\nCSV:", args.output)

    if errors:
        print("RESULT: FAIL (structural)")
        raise SystemExit(1)

    if warnings:
        print("RESULT: PASS (structural), REVIEW (physical)")
    else:
        print("RESULT: PASS")


if __name__ == "__main__":
    main()
