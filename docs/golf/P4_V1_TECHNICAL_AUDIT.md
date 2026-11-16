# P4-V1-TECHNICAL-AUDIT-001

Date: 2026-11-17. Scope: read-only review of the frozen Golf A/A-Speed/B0
implementation, local October/November 2023 CSV bundle, and tracked V1
results. This audit does not refit parameters or revise the controller export.

## 1. Equations and numerical implementation

The Golf A and A-Speed identification path in `apps/model_a_speed_cv.cpp`
uses the exact zero-order-hold update for
`tau * r_dot + r = u + b`, with `u = K_r * delta` for A and
`u = Vx * delta / (L + k_v * Vx^2)` for A-Speed. The input at sample `i-1`
is held over the actual timestamp interval to sample `i`. The controller
runtime implements the same form for both models. `K_r` has units of 1/s,
`tau` of s, `b` of rad/s, and `k_v` of s^2/m.

For Golf B0, the parameter tuple is
`[Cf/m, Cr/m, Iz/m, lf]`, with `lr = 2.63 - lf` and effective mass set to
1 in `apps/model_b_golf_cv.cpp`. The tire-force and state equations in
`src/models/LinearBicycleModel.cpp` expand to:

```text
alpha_f = delta - (Vy + lf*r)/Vx
alpha_r = -(Vy - lr*r)/Vx
Vy_dot = (Cf/m)*alpha_f + (Cr/m)*alpha_r - Vx*r + a_bank
r_dot  = (lf*(Cf/m)*alpha_f - lr*(Cr/m)*alpha_r)/(Iz/m)
a_bank = +9.80665*sin(bank_rad)
```

The coefficients of the implemented state matrix match this expansion,
including the signs of the front/rear yaw moments and `-Vx*r`. `Cf/m` and
`Cr/m` are in m/s^2/rad, `Iz/m` in m^2, and axle distances in m. The model
uses a small-angle linear tire approximation; no `cos(delta)` factor appears
in the lateral-force sum. This is consistent with that approximation, not a
verification that it is accurate at large steering/slip angles.

Golf identification and `ControllerRuntime` both call
`stepImplicitMidpoint(...)` with a maximum substep of 0.02 s and the same
bank term. That function applies the implicit midpoint update to the two
state linear system. The class also exposes `step(...)`, which uses explicit
Euler and has no bank input. The latter is exercised by the basic unit test
and must not be described as the Golf B0 identification integrator.
Reload/parity verifies numerical reproduction of the chosen implementation;
it cannot establish physical validity of the equations or sensor frame.

## 2. Input definitions and unresolved coordinate convention

`src/io/IONIQ5CsvReader.cpp` maps Golf `t_s`, `roadWheelAngle_rad`,
`yawRate_radps`, `Vx_mps`, `Vy_mps`, `ayCG_mps2`, and `bank_rad` to P4's
internal arrays. The source CSV has these headers, and the angle/rate/unit
suffixes are consistent with the reader. The CSV header does not specify
the axis orientation, steering sign, bank sign, CG transformation, or the
accelerometer's gravity compensation. The reader defaults a missing
`bank_rad` column to zero and missing `Vy_mps`/`ayCG_mps2` to NaN; therefore
the accepted channel set alone does not prove that all physical inputs are
present with the intended definition.

The [Stanford repository record](https://purl.stanford.edu/hh613qz0317/version/1)
identifies the Golf dataset, authors, track, and release, but its displayed
description does not define those signal conventions. The `+g*sin(bank)`
choice is thus an implementation assumption. The prior empirical agreement
between `Vy`, `r`, and `ayCG` is useful evidence for these recordings, but
does not by itself resolve the bank or gravity convention. Confirm the
data producer's signal dictionary or calibration notes before calling B0's
force decomposition physically validated.

## 3. Validation independence

`GolfDataset::load` recursively finds CSVs, sorts paths, and omits files
with an already-seen 64-bit content hash. A separate SHA-256 check of the
local 22 CSVs found 20 distinct byte contents and exactly two duplicate
pairs: `2023-10.csv` = `2023-10_2.csv`, and `2023-10_1.csv` =
`2023-10_3.csv`. The current code compares hash values without a confirming
byte comparison; a collision is unlikely but the implementation does not
mathematically guarantee exact duplicate detection.

`GolfDataset::fold` assigns every unique file wholly to one of five folds
using sorted file index modulo five. The A and B training routines receive
only the training files for each fold. B's yaw/`Vy` normalization is also
computed from training files; B1's supporting A-Speed fit uses those same
training files. This supports file-level separation and no direct test-fold
parameter fitting. It does not establish separate driving sessions: the
CSV columns include timestamps and positions but no session identifier,
and the fold rule does not group by recording day, lap, or continuous run.
No untouched external test set has yet been evaluated after model selection.

## 4. Result interpretation

The tracked Golf V1 file-fold results report pooled RMSE of 0.09230 rad/s
for A, 0.08708 rad/s for A-Speed, and 0.08543 rad/s yaw plus 0.33105 m/s
`Vy` for B0. These are predictive errors under the recorded-input,
file-level cross-validation protocol. The full-data coefficients in the
controller export are fitted with all unique files and are not separately
validated by the same pooled scores.

The B0 `lf` profile is shallow: fixing `lf` from 0.8 to 1.2 m changes the
training objective little while `Iz/m` moves substantially. The V1 tuple
is therefore an effective predictive parameter set, not unique measured
vehicle geometry/inertia. The finite-horizon tests support B0's relative
advantage at 0.1–0.2 s under known future logged inputs; the synthetic
closed-loop harness is not actual Golf control validation. The 85–88 s
free-run discrepancy shows a material transient limitation.

## Audit disposition

The implemented Golf equations, units, and midpoint call path are internally
consistent with the documented mathematical model. Keep V1 frozen. Before
asserting physical parameter accuracy or real-vehicle controller readiness,
obtain authoritative channel/frame definitions, confirm session grouping,
and evaluate unused driving data or an independent calibrated plant. Define
control-specific tolerances for yaw and lateral-velocity prediction.
