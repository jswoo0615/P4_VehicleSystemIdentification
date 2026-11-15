# IONIQ 5 exploratory identification (2026-11-15)

Follow-up A1 speed-gain and S1 friction cross-validation is documented in
`IONIQ5_REFINEMENT.md`.

Run commands below from the `P4_VehicleSystemIdentification` directory. The
original MAT files and converted CSV files are read only by these experiments.

## Data audit

The conversion manifest records 41 successful MAT-to-CSV exports, 2,535,459
rows, and 41 retained original MAT files. All 41 source SHA-256 hashes match
the manifest. Thirteen CSVs contain Model A's steering, yaw, and speed fields.
Nine of the thirteen standstill CSVs contain both tie-rod force fields. Four
instead have `M_eps_Nm`, so they are excluded from S-Load.

`time_s` is in seconds. `delta_stm_rad` is measured equivalent steering angle
in radians, `w_z_cor_radps` is measured yaw rate in rad/s, `v_x_cor_mps` is
longitudinal speed in m/s, and `F_trl_N`/`F_trr_N` are tie-rod forces in N.
No commanded steering angle is available. The converted CSVs retain radians;
degree conversion is useful for plotting only.

## S-Load: measured angle and rate to measured tie-rod forces

```sh
cmake -S . -B build
cmake --build build --target model_s_load_csv model_a_grouped_csv -j4
build/model_s_load_csv data/IONIQ5_Data/processed/standstill_concrete_1.csv data/IONIQ5_Data/processed/standstill_asphalt_a_1.csv data/IONIQ5_Data/processed/standstill_cobble_2.csv --test data/IONIQ5_Data/processed/standstill_asphalt_a_2.csv data/IONIQ5_Data/processed/standstill_cobble_3.csv
```

The program uses only actual steering value changes. A centered difference
uses their timestamps; it accepts neighboring update intervals of 10–200 ms
and rates of at most 5 rad/s. It fits independent intercept, angle, and rate
coefficients for the left and right force by least squares. Files, rather than
rows, are split between training and test.

This run used 5,674 training and 2,861 test steering updates:

| Force | Fit (N; angle in rad, rate in rad/s) | Train RMSE | Test RMSE | Test mean-only baseline |
|---|---|---:|---:|---:|
| Left | 432.58 + 3186.45 δ + 6795.87 δdot | 852.26 N | 904.48 N | 2686.73 N |
| Right | 629.93 − 3375.20 δ − 6844.18 δdot | 801.21 N | 855.24 N | 2585.35 N |

This is an empirical steering **load** relationship. Force signs, sensor
location, timing alignment, and road-dependent offsets still need independent
verification before physical interpretation. It is not command-to-angle
actuator Model S.

## Model A: measured angle to measured yaw rate

```sh
build/model_a_grouped_csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_1.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_2.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_3.csv --test data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_4.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_5.csv
```

The dedicated `IONIQ5CsvReader` reads named columns, validates time order and
finite numeric values, retains radians/rad/s/m/s, and builds forward-driving
segments at speed >=2 m/s. It samples at about 20 ms rather than treating each
~1 ms exported row as an independent steering measurement. Gaps over 80 ms,
low-speed periods, and yaw jumps over 0.25 rad/s break segments; 100 ms on
both sides of each jump is excluded. This removes the known drop/recovery at
about 97.647–97.667 s in `dynamic_driving_asphalt_a_1.csv`.

The pooled first-order fit uses exact elapsed-time propagation and resets its
state at each segment. Three training files yielded 13,538 samples in eight
segments; two held-out files yielded 6,099 samples in six segments.

| K | tau | b | Train RMSE | Test RMSE |
|---:|---:|---:|---:|---:|
| 2.36118 1/s | 0.07536 s | 0.03440 rad/s | 0.12773 rad/s | 0.21394 rad/s |

The held-out error is substantially larger. Model A uses one constant gain
across changing speeds and maneuvers, so these coefficients are exploratory,
not calibrated vehicle-wide values. Steering-angle sensor meaning and timing,
road dependence, and yaw bias remain open questions.

## Held-out plots and diagnostics

Generate predictions with the same C++ preprocessing, fitted coefficients,
segment resets, and propagation used for the scores above. The exports and
plots are written under `output/heldout_diagnostics/`; source CSVs are read
only. Python requires NumPy and Matplotlib.

```sh
cmake --build build --target model_a_grouped_csv model_s_load_csv -j4
build/model_a_grouped_csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_1.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_2.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_3.csv --test data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_4.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_5.csv --report-dir output/heldout_diagnostics
build/model_s_load_csv data/IONIQ5_Data/processed/standstill_concrete_1.csv data/IONIQ5_Data/processed/standstill_asphalt_a_1.csv data/IONIQ5_Data/processed/standstill_cobble_2.csv --test data/IONIQ5_Data/processed/standstill_asphalt_a_2.csv data/IONIQ5_Data/processed/standstill_cobble_3.csv --report-dir output/heldout_diagnostics
python3 -m tools.visualization.plot_heldout_ioniq5 --report-dir output/heldout_diagnostics --raw-dir data/IONIQ5_Data/processed
```

`heldout_metrics.md` gives exact per-segment and per-file scores, speed and
absolute lateral-acceleration quartile scores, common-interior time-shift
scores, sampling intervals, and S-Load scores. The PNGs show measured and
predicted traces for each held-out file and Model A residual scatter plots.
Model A excludes each segment's first sample from RMSE because its measured
yaw initializes that segment's prediction state.

Observed on these two held-out Model A files:

| File | RMSE (rad/s) | Segment RMSEs (rad/s) |
|---|---:|---|
| `dynamic_driving_asphalt_a_4.csv` | 0.25993 | 0.21379, 0.21710, 0.67174 |
| `dynamic_driving_asphalt_a_5.csv` | 0.16743 | 0.30885, 0.10701, 0.08581 |

The largest errors are localized: file 4's third segment (61.28–64.29 s)
and file 5's first segment (4.25–18.06 s) show predicted yaw magnitude much
larger than measured. In file 4's first segment, the model also overshoots
several long plateaus. A held-out affine diagnostic gives slopes 0.683 for
file 4 and 0.281 for file 5, indicating amplitude and offset mismatch;
those slopes are *diagnostics fitted to test targets*, not revised model
coefficients or valid test scores. The file 5 slope is especially sensitive
to its long narrow-range segment and should not be interpreted as a physical
gain estimate.

Moving the prediction by up to three ~21 ms samples barely changes RMSE:
best common-interior reductions are 0.25968→0.25933 for file 4 and
0.16732→0.16624 for file 5. A simple fixed time offset therefore does not
explain the principal errors. Errors vary with speed, but the pattern differs
between files. High absolute lateral acceleration is not a monotonic error
driver: the top quartile scores 0.119 in file 4 and 0.160 rad/s in file 5,
versus 0.352 and 0.277 rad/s in other quartiles. This does not establish
high-`ay` nonlinearity as the main cause. The selected median interval is
20.91 ms; raw steering-value changes have medians 20.01 and 20.14 ms.
The observed mismatch is therefore not explained by treating the ~1 ms CSV
rows as independent steering updates.

For S-Load, file 2 asphalt scores 947.5 N left and 870.0 N right; file 3
cobble scores 820.2 N left and 827.5 N right. The traces follow the broad
cycles but miss sharp turning peaks and force reversals, suggesting that
the linear angle/rate relationship leaves structured residuals. The plots
alone do not identify whether sensor timing, hysteresis, or another omitted
effect is responsible.

## Model boundaries and verification

Model S needs commanded steering and measured angle; these CSVs do not supply
the command. S-Load predicts tie-rod forces from measured angle and rate.
Model A predicts yaw rate from measured angle. Model B is separate; the later
bicycle-model reference in `parameter.m` is wheelbase 3 m, mass 2254 kg,
`Iz=2193 kg m²`, `lf=1.563 m`, `lr=1.437 m`, and `Cf=Cr=234008 N/rad`.
Those reference stiffnesses were not used in the fits here.

```sh
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

The CSV reader test checks 20 ms sampling, preserved radian values, and yaw
glitch exclusion on a synthetic file. The full test suite also covers the
existing Models A, S, and B and VBO parsing. Its old VBO test paths now point
to their actual location under `data/old_vito_not_used/`.
