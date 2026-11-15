
from pathlib import Path

import numpy as np


INPUT = Path(
    "data/IONIQ5_Data/extracted_arrays.npz"
)

OUTPUT = Path(
    "data/IONIQ5_Data/processed/"
    "dynamic_driving_asphalt_a_1.csv"
)

CHANNELS = [
    "delta_stm_rad",
    "F_trl_N",
    "F_trr_N",
    "a_x_ra_mps2",
    "a_y_ra_mps2",
    "w_z_cor_radps",
    "v_x_cor_mps",
    "v_y_cor_mps",
    "lat_ra_deg",
    "long_ra_deg",
]


def main():
    with np.load(INPUT) as raw:
        time_arrays = [
            raw[f"array_{2 * i:02d}"]
            for i in range(len(CHANNELS))
        ]

        t = time_arrays[0]

        if not np.all(np.isfinite(t)):
            raise ValueError("Invalid timestamps")

        if np.any(np.diff(t) <= 0):
            raise ValueError("Time is not strictly increasing")

        for other in time_arrays[1:]:
            if not np.array_equal(t, other):
                raise ValueError(
                    "Channel time vectors differ"
                )

        signals = {
            name: raw[f"array_{2 * i + 1:02d}"]
            for i, name in enumerate(CHANNELS)
        }

        for name, values in signals.items():
            if len(values) != len(t):
                raise ValueError(
                    f"Length mismatch: {name}"
                )

            if not np.all(np.isfinite(values)):
                raise ValueError(
                    f"Non-finite values: {name}"
                )

        columns = ["time_s"] + CHANNELS

        matrix = np.column_stack(
            [t] + [signals[name] for name in CHANNELS]
        )

    OUTPUT.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    np.savetxt(
        OUTPUT,
        matrix,
        delimiter=",",
        header=",".join(columns),
        comments="",
        fmt="%.12g",
    )

    print("Exported:", OUTPUT)
    print("Samples :", len(t))
    print("Duration:", t[-1] - t[0], "s")
    print("Channels:", len(CHANNELS))


if __name__ == "__main__":
    main()
