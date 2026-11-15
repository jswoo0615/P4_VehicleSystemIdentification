# IONIQ 5 model refinement experiment (2026-11-15)

The two new C++17 programs use exactly the existing Model A segment reader
and S-Load guarded-update selection. They train only on the original three
training files per model. Leave-one-file-out CV is performed on those files;
the pre-existing `_a_4`/`_a_5` and standstill test files are never used to
choose parameters. Since those test files were already examined in the prior
diagnostic, their scores are exploratory. Additional files below were
evaluated after the candidate structures and search grids were fixed.

## Reproduce

From the project root:

```sh
cmake -S . -B build
cmake --build build --target model_a_speed_cv model_s_load_friction_cv -j4
build/model_a_speed_cv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_1.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_2.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_3.csv --test data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_4.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_5.csv
build/model_a_speed_cv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_1.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_2.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_3.csv --test data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_6.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_7.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_8.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_9.csv
build/model_a_speed_cv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_1.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_2.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_3.csv --test data/IONIQ5_Data/processed/dynamic_driving_cobble_1.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_b_1.csv
build/model_s_load_friction_cv data/IONIQ5_Data/processed/standstill_concrete_1.csv data/IONIQ5_Data/processed/standstill_asphalt_a_1.csv data/IONIQ5_Data/processed/standstill_cobble_2.csv --test data/IONIQ5_Data/processed/standstill_asphalt_a_2.csv data/IONIQ5_Data/processed/standstill_cobble_3.csv
build/model_s_load_friction_cv data/IONIQ5_Data/processed/standstill_concrete_1.csv data/IONIQ5_Data/processed/standstill_asphalt_a_1.csv data/IONIQ5_Data/processed/standstill_cobble_2.csv --test data/IONIQ5_Data/processed/standstill_asphalt_b_2.csv data/IONIQ5_Data/processed/standstill_plates_3.csv
```

The optional `--causal-rate` switch on `model_s_load_friction_cv` replaces
the centered angle difference with a preceding-update difference. It still
uses the same offline eligible-update selection, so it is a rate-estimator
sensitivity check, not a complete online deployment simulation.

## Model A: constant gain A0 versus speed-dependent A1

Both models use exact elapsed-time first-order propagation and reset the yaw
state to the measured first sample of each segment. A0 searches gain, bias,
and time constant. A1 uses

`tau * r_dot + r = [v_x / (3.0 + kv * v_x^2)] * delta + b`.

The fixed 3.0 m wheelbase comes from the supplied vehicle parameters. A1
searches nonnegative `kv` from 0 to 0.15 in 0.0025 steps; both models search
time constants from 0.03 to 3 s on the same 81-point logarithmic grid.

| Metric | A0 constant | A1 speed |
|---|---:|---:|
| Pooled leave-one-file-out CV RMSE | 0.193604 | **0.032506** rad/s |
| Full-train RMSE | 0.127732 | **0.030796** rad/s |
| Original test `_a_4` | 0.259934 | **0.038941** rad/s |
| Original test `_a_5` | 0.167434 | **0.033310** rad/s |
| Additional asphalt `_a_6` | 0.185342 | **0.043225** rad/s |
| Additional asphalt `_a_7` | 0.165929 | **0.038314** rad/s |
| Additional asphalt `_a_8` | 0.150564 | **0.045516** rad/s |
| Additional asphalt `_a_9` | 0.122948 | **0.033858** rad/s |
| Additional cobble `_1` | 0.398460 | **0.069482** rad/s |
| Additional asphalt B `_1` | 0.119064 | **0.032896** rad/s |

The full-training A1 fit is `tau=0.053348 s`, `kv=0.005000`, and
`b=0.005965 rad/s`. It clears the proposed improvement thresholds on both
original test files and improves all six additional driving files. The
previous worst `_a_4` segment falls from 0.671743 to 0.057844 rad/s.
Omitting the first 0.2 s of that segment does not repair A0: its error is
0.691206 rad/s after the cut, versus 0.059591 for A1. Thus the large A0
error is not confined to the measured-yaw initialization transient.
The fitted `kv=0.005000` is **inside** the searched interval `0–0.15`,
not at its upper or lower boundary; it is the second positive grid point.
The grid spacing is 0.0025, so the reported value is not a high-precision
physical estimate.
The full-precision A0/A1 values and training-file identities are recorded in
`config/ioniq5_baselines.json` for later Model B comparisons.

These scores support a speed-dependent response for these data. They do not
prove the fitted `kv` is a unique physical understeer coefficient: steering
angle meaning, sensor alignment, and other dynamics must be checked before
that interpretation or before using the fit to infer Model B stiffnesses.

## Model S-Load: S0 linear versus S1 tanh friction

S1 adds `Fc * tanh(delta_dot / vs)` to the existing intercept, angle, and
rate terms for each side. It chooses `vs` by three-file leave-one-file-out CV
from 0.01, 0.02, 0.04, 0.08, 0.16, 0.32, 0.64, and 1.28 rad/s. Both
models use the same guarded steering updates and centered difference as the
original S-Load fit. The centered difference uses a *future angle sample*;
these are offline scores, not real-time predictions.

| Side and evaluation | S0 linear | S1 tanh |
|---|---:|---:|
| Left pooled CV | 929.8 | **876.6** N |
| Right pooled CV | 869.4 | **806.1** N |
| Left asphalt A `_2` | **947.5** | 1013.4 N |
| Right asphalt A `_2` | **870.0** | 912.1 N |
| Left cobble `_3` | 820.2 | **746.5** N |
| Right cobble `_3` | 827.5 | **719.9** N |
| Left additional asphalt B `_2` | 752.6 | **705.8** N |
| Right additional asphalt B `_2` | 745.9 | **698.3** N |
| Left additional plates `_3` | **1546.3** | 1685.4 N |
| Right additional plates `_3` | **1679.2** | 1798.6 N |

S1 improves pooled training-file CV, but fails the proposed asphalt A
improvement criterion and worsens both sides on the additional plates file.
The 200 ms after a steering-rate sign reversal is also worse for S1 on both
original test files: asphalt A left 1988→2311 N, right 1641→2045 N; cobble
left 1479→1783 N, right 1154→1533 N. This simple memoryless tanh term is
therefore **not adopted** as a general S-Load replacement. It does not
resolve the reversal behavior it was intended to address.

Changing to a preceding-update rate estimate increases pooled CV RMSE for
both models and sides (S0 left/right 1022/972 N; S1 left/right 940/870 N).
That shows the offline centered derivative contributes to the reported
accuracy. A causal implementation needs its own train-only tuning and
independent evaluation.

## All asphalt A driving files: file-group 9-fold CV

The earlier A0/A1 cross-validation rotated only the three original training
files (`a_1`–`a_3`). The follow-up experiment rotates **all nine**
`dynamic_driving_asphalt_a_1.csv` through `_a_9.csv` files: each fold trains
on eight complete files and evaluates on the ninth. File boundaries and
segment resets remain intact; each file is evaluated exactly once. Run:

```sh
cmake --build build --target model_a_speed_cv -j4
build/model_a_speed_cv --all-file-cv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_[1-9].csv
```

| Held-out file | A0 RMSE | A1 RMSE | A1 `kv` fitted without that file |
|---|---:|---:|---:|
| `a_1` | 0.145426 | **0.039013** | 0.0025 |
| `a_2` | 0.293816 | **0.041229** | 0.0050 |
| `a_3` | 0.124344 | **0.033667** | 0.0050 |
| `a_4` | 0.250535 | **0.038468** | 0.0050 |
| `a_5` | 0.160967 | **0.032670** | 0.0050 |
| `a_6` | 0.163813 | **0.046772** | 0.0050 |
| `a_7` | 0.166131 | **0.038294** | 0.0050 |
| `a_8` | 0.149506 | **0.045960** | 0.0050 |
| `a_9` | 0.114095 | **0.031713** | 0.0050 |
| **Pooled, 29,971 scored samples** | **0.166039** | **0.038669** | — |

The pooled RMSE is computed from all held-out squared errors, not an
unweighted mean of the nine file RMSEs. A1 improves all nine files and
reduces pooled RMSE by 76.7%. Its fitted `kv` is 0.0050 in eight folds and
0.0025 in the `a_1` fold; the fitted time constants range from about
0.0424 to 0.0475 s. These are fold-specific fits and do **not** replace the
frozen A1 values in `config/ioniq5_baselines.json`.

This is a broader cross-validation estimate over the asphalt A sequence,
but `a_4`–`a_9` were previously inspected in separate evaluations. All nine
files now participate in model fitting in other folds, so none of them is an
untouched final test set. An independent claim for a later Model B comparison
requires a newly reserved file or dataset that is excluded from all model
choices.

## Refit using all nine asphalt A files

After the nine file-level CV runs, fit one coefficient set using all nine
files. This uses the same preprocessing and parameter grid as every fold:

```sh
build/model_a_speed_cv --fit-all data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_[1-9].csv
```

| Model | 3-file fit (`a_1`–`a_3`) | 9-file fit (`a_1`–`a_9`) | 9-file training RMSE |
|---|---|---|---:|
| A0 | `K=2.361176`, `tau=0.075357 s`, `b=0.034404` | `K=2.215679`, `tau=0.067162 s`, `b=0.003920` | 0.154129 rad/s |
| A1 | `kv=0.005000`, `tau=0.053348 s`, `b=0.005965` | `kv=0.005000`, `tau=0.044887 s`, `b=0.002889` | 0.035335 rad/s |

The 9-file A1 equation is

`0.044887069682833*r_dot + r = [v_x/(3.0 + 0.005*v_x^2)]*delta + 0.00288859811493495`.

These training RMSEs use data that fitted the coefficients. For predictive
performance over the nine files, use the preceding file-level CV RMSEs
(A0 0.166039, A1 0.038669 rad/s). The previous three-file coefficient set
is retained in `config/ioniq5_baselines.json` for provenance; the new
all-nine values are recorded separately there. Any Model B CV comparison
should also refit its parameters inside each fold and compare fold-matched
predictions, rather than compare its CV error with A1's 9-file training error.
