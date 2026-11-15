"""Export all IONIQ 5 MCOS timeseries from MATLAB v5 files without MATLAB.

Run from the project root: python3 -m tools.datasets.kit_ioniq5.batch_export_ioniq5
Requires numpy and scipy. Fails closed if the observed MCOS layout changes.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat

if __package__:
    from .read_mat import extract_double_arrays
else:  # Also support direct script execution.
    from read_mat import extract_double_arrays


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_INPUT = ROOT / "data/IONIQ5_Data/data/dataset"
DEFAULT_OUTPUT = ROOT / "data/IONIQ5_Data/processed"
EXPECTED_CHANNEL_COUNTS = {6, 7, 10, 12}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def display_path(path):
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def convert(source, output_dir):
    mat = loadmat(source, variable_names=["data", "__function_workspace__"],
                  squeeze_me=True, struct_as_record=False)
    data = mat["data"]
    channels = list(data._fieldnames)
    if len(channels) not in EXPECTED_CHANNEL_COUNTS or len(set(channels)) != len(channels):
        raise ValueError(f"Unexpected channel set: {channels}")

    # The opaque object's fifth index advances by five in data-field order.
    # This independently checks the field-to-array ordering seen in the workspace.
    for i, name in enumerate(channels):
        opaque = getattr(data, name)
        if opaque.size != 1 or opaque[0][1] != b"MCOS" or opaque[0][2] != b"timeseries":
            raise ValueError(f"Unexpected MCOS object for {name}")
        if int(opaque[0][3][4]) != 1 + 5 * i:
            raise ValueError(f"MCOS reference order mismatch for {name}")

    arrays = extract_double_arrays(mat["__function_workspace__"].tobytes(), min_samples=2)
    if len(arrays) != 2 * len(channels):
        raise ValueError(f"Expected {2 * len(channels)} arrays, found {len(arrays)}")

    times = []
    signals = []
    channel_qa = {}
    for i, name in enumerate(channels):
        time_offset, time = arrays[2 * i]
        signal_offset, signal = arrays[2 * i + 1]
        if time.size != signal.size or time.size < 2:
            raise ValueError(f"Time/signal length mismatch for {name}")
        if not np.all(np.isfinite(time)) or not np.all(np.isfinite(signal)):
            raise ValueError(f"Nonfinite time/signal for {name}")
        dt = np.diff(time)
        if not np.all(dt > 0):
            raise ValueError(f"Time not strictly increasing for {name}")
        if times and not np.array_equal(time, times[0]):
            raise ValueError(f"Channel time vectors differ for {name}")
        times.append(time)
        signals.append(signal)
        channel_qa[name] = {
            "time_offset": time_offset, "signal_offset": signal_offset,
            "min": float(signal.min()), "max": float(signal.max()),
            "median_dt_s": float(np.median(dt)), "max_dt_s": float(dt.max()),
            "value_changes": int(np.count_nonzero(np.diff(signal))),
        }

    destination = output_dir / (source.stem + ".csv")
    tmp = destination.with_suffix(".csv.tmp")
    output_dir.mkdir(parents=True, exist_ok=True)
    try:
        np.savetxt(tmp, np.column_stack([times[0], *signals]), delimiter=",",
                   header=",".join(["time_s", *channels]), comments="", fmt="%.17g")
        with tmp.open() as check:
            header = next(check).rstrip("\n")
            rows = sum(1 for _ in check)
        if header != ",".join(["time_s", *channels]) or rows != len(times[0]):
            raise ValueError("CSV write verification failed")
        tmp.replace(destination)
    finally:
        tmp.unlink(missing_ok=True)

    return {
        "source": display_path(source), "csv": display_path(destination),
        "source_sha256": sha256(source), "csv_sha256": sha256(destination),
        "rows": len(times[0]), "channel_count": len(channels), "channels": channels,
        "time_start_s": float(times[0][0]), "time_end_s": float(times[0][-1]),
        "duration_s": float(times[0][-1] - times[0][0]),
        "time_vectors_equal": True, "mcos_reference_order_verified": True,
        "channels_qa": channel_qa,
        "model_a_ready": all(c in channels for c in
                             ("delta_stm_rad", "w_z_cor_radps", "v_x_cor_mps")),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    sources = sorted(args.input_dir.glob("*.mat"))
    if not sources:
        parser.error("No MAT files found")

    results = []
    failures = []
    for source in sources:
        try:
            result = convert(source, args.output_dir)
            results.append(result)
            print(f"OK {source.name}: {result['rows']} rows, {result['channel_count']} channels")
        except Exception as exc:
            failures.append({"source": str(source), "error": str(exc)})
            print(f"FAIL {source.name}: {exc}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "conversion_qa.json").write_text(
        json.dumps({"converted": results, "failures": failures}, indent=2) + "\n")
    with (args.output_dir / "conversion_manifest.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["source", "csv", "source_sha256", "csv_sha256", "rows",
                         "channel_count", "channels", "time_start_s", "time_end_s",
                         "duration_s", "model_a_ready", "status"])
        for row in results:
            writer.writerow([row[k] if k != "channels" else ";".join(row[k]) for k in
                             ("source", "csv", "source_sha256", "csv_sha256", "rows",
                              "channel_count", "channels", "time_start_s", "time_end_s",
                              "duration_s", "model_a_ready")] + ["ok"])
        for failure in failures:
            writer.writerow([failure["source"], *([""] * 10), failure["error"]])
    print(f"Converted {len(results)}/{len(sources)}; Model A ready: "
          f"{sum(r['model_a_ready'] for r in results)}")
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
