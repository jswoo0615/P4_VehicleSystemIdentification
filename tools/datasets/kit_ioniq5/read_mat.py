
from pathlib import Path
import struct

import numpy as np
from scipy.io import loadmat


MAT_PATH = Path(
    "data/IONIQ5_Data/data/dataset/"
    "dynamic_driving_asphalt_a_1.mat"
)

# MAT v5 data-element types
MI_MATRIX = 14
MI_DOUBLE = 9

# Only extract large double arrays.
MIN_SAMPLES = 100_000


def extract_double_arrays(buf, min_samples=MIN_SAMPLES):
    arrays = []

    def walk(start, end, depth=0):
        pos = start

        while pos + 8 <= end:
            tag0, tag1 = struct.unpack_from("<II", buf, pos)

            # MAT v5 small-data-element format
            if tag0 >> 16:
                dtype = tag0 & 0xFFFF
                nbytes = tag0 >> 16

                if nbytes > 4:
                    break

                payload = pos + 4
                next_pos = pos + 8
            else:
                dtype = tag0
                nbytes = tag1
                payload = pos + 8
                next_pos = payload + ((nbytes + 7) // 8) * 8

            if (
                nbytes < 0
                or payload + nbytes > end
                or next_pos > end
                or next_pos <= pos
            ):
                break

            if dtype == MI_MATRIX:
                walk(payload, payload + nbytes, depth + 1)

            elif dtype == MI_DOUBLE:
                if nbytes >= min_samples * 8:
                    if nbytes % 8 != 0:
                        raise ValueError("Invalid double array size")

                    values = np.frombuffer(
                        buf,
                        dtype="<f8",
                        count=nbytes // 8,
                        offset=payload,
                    ).copy()

                    arrays.append((payload, values))

            pos = next_pos

    # MCOS workspace begins with an 8-byte prefix.
    walk(8, len(buf))

    return sorted(arrays, key=lambda item: item[0])


def main():
    mat = loadmat(MAT_PATH)
    workspace = mat["__function_workspace__"].tobytes()

    arrays = extract_double_arrays(workspace)

    print("=== Extracted Double Arrays ===")
    print("Total:", len(arrays))

    output = {}

    for i, (offset, values) in enumerate(arrays):
        name = f"array_{i:02d}"
        output[name] = values

        finite = values[np.isfinite(values)]
        diff = np.diff(values)

        if len(finite) == 0:
            print(f"{name}: no finite values")
            continue

        monotonic = bool(
            len(diff) > 0
            and np.all(np.isfinite(diff))
            and np.all(diff > 0)
        )

        median_dt = (
            float(np.median(diff))
            if monotonic
            else float("nan")
        )

        print(
            f"\n{name}"
            f"\n  Offset    : {offset}"
            f"\n  Samples   : {values.size}"
            f"\n  Min       : {finite.min():.6f}"
            f"\n  Max       : {finite.max():.6f}"
            f"\n  Mean      : {finite.mean():.6f}"
            f"\n  Monotonic : {monotonic}"
            f"\n  Median dt : {median_dt:.9f}"
            f"\n  First 5   : {values[:5]}"
        )

    out_path = Path("data/IONIQ5_Data/extracted_arrays.npz")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    np.savez_compressed(out_path, **output)

    print("\nSaved:", out_path)


if __name__ == "__main__":
    main()
