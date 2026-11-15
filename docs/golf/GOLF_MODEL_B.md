# VW Golf Model B: file-group validation

`GOLF-B-KFOLD-001` uses the same 20 distinct October/November 2023 VW Golf
CSVs and five whole-file folds as `GOLF-A-KFOLD-001`. Two byte-identical
October file pairs are omitted. The Stanford CSV reader supplies `t_s`,
`roadWheelAngle_rad`, `yawRate_radps`, `Vx_mps`, `Vy_mps`, and `bank_rad`.
`atan2(Vy_mps,Vx_mps)` agrees numerically with `slipAngle_rad` in the source
file, supporting use of `Vy_mps` as the bicycle-model lateral velocity. This
does not independently verify sensor placement or calibration.

## Model and fit

The existing `LinearBicycleModel` implicit-midpoint integrator is reused.
Setting its mass to one makes the four fitted parameters
`[Cf/m, Cr/m, Iz/m, lf]`; `lr=2.63-lf`. The road bank enters the lateral
acceleration equation as `+9.80665*sin(bank_rad)`. Its sign follows the
assumed Stanford CSV coordinate convention and should be checked against
dataset documentation before physical interpretation.

Samples below 5 m/s are excluded. Source timestamps are about 200 Hz; the
cross-file experiment selects a 50 Hz grid. Each continuous segment is
initialized from measured `Vy` and yaw rate. The fit minimizes the sum of
yaw and `Vy` squared errors, each divided by the corresponding training-file
variance, so neither held-out signals nor their scale enter the fit.
Bounds are `Cf/m, Cr/m ∈ [10,250] m/s²/rad`, `Iz/m ∈ [0.7,3.2] m²`, and
`lf ∈ [0.7,1.6] m`. A deterministic multistart pattern search includes
profile starts with `lf` fixed at several values; this limits dependence on
one starting guess. The fits are effective parameters of this linear model,
not independent measurements of tire stiffness, mass, or inertia.

## Results

| Fold | A-Speed yaw RMSE | B yaw RMSE | B `Vy` RMSE |
|---|---:|---:|---:|
| 1 | 0.06998 | 0.06929 | 0.32227 |
| 2 | 0.13621 | 0.13327 | 0.32320 |
| 3 | 0.08797 | 0.08556 | 0.35367 |
| 4 | 0.05637 | 0.05775 | 0.34172 |
| 5 | 0.07466 | 0.07150 | 0.31543 |
| **Pooled** | **0.08708** | **0.08543** | **0.33105** |

Yaw RMSE is in rad/s; `Vy` RMSE is in m/s. B improves pooled yaw RMSE by
about 1.9% against A-Speed and wins four of five folds. The worst single
file remains `2023-10_6.csv` (B yaw RMSE about 0.206 rad/s). The full-data
reference fit is `Cf/m=48.38`, `Cr/m=51.01 m/s²/rad`, `Iz/m=2.53 m²`,
`lf=1.00 m`, and `lr=1.63 m`. This full-data fit was not independently
validated; use fold results to judge predictive performance.

## Parameter sensitivity

Refitting `Cf/m`, `Cr/m`, and `Iz/m` with `lf` fixed gives:

| Fixed `lf` (m) | Training objective | Fitted `Iz/m` (m²) |
|---:|---:|---:|
| 0.80 | 0.34110 | 2.08 |
| 1.00 | 0.33995 | 2.53 |
| 1.04 | 0.34015 | 2.63 |
| 1.20 | 0.34224 | 2.90 |
| 1.40 | 0.34799 | 3.18 |

The 0.8–1.2 m profiles have similar loss while `Iz/m` moves substantially.
The search bound for `lf` is 0.7–1.6 m, so the 1.00 m full fit is not on a
constraint boundary. Its repeated appearance reflects the shallow objective
and the finite deterministic search, not proof of a precise axle location.
Thus these data and this objective do not sharply separate `lf` from
`Iz/m`. The fitted inertia ratio is also above the rough ratio implied by
the previously discussed mass and inertia ranges; model mismatch, signal
definition, or those provisional ranges may be responsible. Do not convert
these normalized coefficients to absolute `Cf`, `Cr`, or `Iz` without
session-specific measured mass and geometry.

## Reproduce

```sh
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target model_b_golf_cv -j4
./build-release/model_b_golf_cv data/VehicleDynamicsDataset output/golf_b_kfold_001.json
```

The JSON contains fold coefficients, fold and file RMSEs, the full-data fit,
constraints, and the fixed-`lf` loss profile.
