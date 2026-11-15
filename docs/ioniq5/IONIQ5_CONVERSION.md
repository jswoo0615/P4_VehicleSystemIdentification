# IONIQ 5 MAT to CSV conversion

Run from this project directory:

```sh
python3 -m tools.datasets.kit_ioniq5.batch_export_ioniq5
cmake -S . -B build
cmake --build build --target model_a_grouped_csv
build/model_a_grouped_csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_1.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_2.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_3.csv --test data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_4.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_5.csv
```

Requires Python 3, NumPy, and SciPy. The converter reads all `*.mat` files under
`data/IONIQ5_Data/data/dataset/` and writes CSVs to
`data/IONIQ5_Data/processed/`. It does not modify the MAT files. Generated CSVs
are ignored by Git because the batch occupies about 414 MB.

## Conversion outcome

- 41/41 MAT files converted; 2,535,459 total rows; 0 failed.
- Channel counts: 20 files with 6, 4 with 7, 6 with 10, and 11 with 12.
- 13 CSVs contain all three Model A inputs: `delta_stm_rad`,
  `w_z_cor_radps`, and `v_x_cor_mps`.
- All 41 original MAT files match the bundled BagIt MD5 manifest.
- All CSV numeric values round-trip exactly to the extracted double arrays.
- For every channel, timestamps are finite and strictly increasing, signal
  values are finite, and all channel time vectors within a file are identical.

`processed/conversion_manifest.csv` lists files, row and channel counts,
times, readiness for Model A, and SHA-256 hashes. `processed/conversion_qa.json`
adds per-channel offsets, ranges, sample intervals, and change counts.

## Mapping and limits

The converter uses the `data` struct field names as CSV headers. Every MCOS
timeseries reference follows the observed `1, 6, 11, ...` sequence in that
field order; the workspace contains exactly two numeric arrays per channel,
alternating time and signal. The converter rejects any file that violates
these conditions, has unequal channel time vectors, or has invalid samples.
This validates the observed MAT layout, but is not a general MCOS decoder.

CSV timestamps may have about 1 ms spacing while measured signal values
change less frequently. The CSV preserves both exactly; it does not imply a
new sensor reading at every row. Model A uses radians, radians per second,
meters per second, and seconds. `model_a_identify_csv` fits gain `K`, time
constant `tau`, and steady-state yaw bias `b` using exact first-order updates.
Select a maneuver interval before treating its fit as a vehicle parameter.
The same-file 0–60 s and 60–90 s fit was a loader smoke test only. For
file-group holdout, time-aware segmentation, and the separate S-Load experiment,
see [IONIQ5_EXPERIMENTS.md](IONIQ5_EXPERIMENTS.md). The full CTest suite now
passes 13/13 tests, including corrected paths for the legacy VBO fixtures.
