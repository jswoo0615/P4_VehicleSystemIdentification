# IONIQ 5 Model B identification (2026-11-15)

Model B is the existing 2DOF linear bicycle model with states `(v_y at CoG,
r)` and input effective road-wheel angle `delta_stm_rad`. The fixed vehicle
values come from `data/IONIQ5_Data/data/dataset/parameter.m`: mass 2254 kg,
yaw inertia 2193 kg m², front axle distance 1.563 m, and rear axle distance
1.437 m. Only effective axle cornering stiffnesses `Cf` and `Cr` are fitted.
The supplied `Cf=Cr=234008 N/rad` is evaluated separately as a reference;
it is not used as a target or a fixed fit value.

## Measurement frame and numerical method

The CSV signals `v_x_cor_mps` and `v_y_cor_mps` are at the Correvit sensor,
while the bicycle state is at the center of gravity (CoG). The dataset's
`parameter.m` states ISO 8855 axes (x forward, y left, z up) and translation
vectors `cor→rear axle=[0.958,-0.159,0] m` and
`rear axle→CoG=[1.437,0,0] m`. Rigid-body velocity translation gives

`vx_cog = vx_cor + 0.159*r` and `vy_cog = vy_cor + 2.395*r`.

Using raw `v_y_cor` as if it were at CoG produced an implausible joint fit
(`Cf≈16 kN/rad`, `Cr≈27 kN/rad`) and a reference-value lateral-velocity
RMSE of 1.471 m/s. Applying the documented translation reduces the
reference-value lateral-velocity RMSE to 0.230 m/s. The frame translation
comes from supplied geometry; it is not a fitted test-data offset.

Model A's existing reader supplies the same ~20 ms forward-driving segments,
speed threshold, glitch guard, and file boundaries. Every Model B segment is
initialized from its first measured CoG lateral velocity and yaw rate. Its
first sample is excluded from RMSE, as with Model A. The existing C++17
bicycle model now has an implicit-midpoint step with substeps no longer than
5 ms; the earlier explicit-Euler method remains available. This avoids
numerical instability from a 20 ms Euler step at low speed and high tire
stiffness.

For fitting, yaw and CoG lateral-velocity squared errors are each divided by
their standard deviation **computed from the training files only**; the two
normalized terms then receive equal weight. A deterministic two-parameter
log-space pattern search uses bounds 10–500 kN/rad. No test file enters the
fit, scale calculation, or search. The printed objective is a dimensionless
training objective, while yaw and lateral-velocity RMSEs retain their units.
The final search step is 0.0125 in log stiffness, so the many decimal places
in the run log are reproducibility values, not physical precision.

## Reproduce

Run from the project root:

```sh
cmake -S . -B build
cmake --build build --target model_b_ioniq5_cv -j4
build/model_b_ioniq5_cv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_[1-9].csv --test data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_cobble_1.csv data/IONIQ5_Data/processed/dynmic_asphalt_b_cobble_1.csv
```

The command fits Model B afresh in each leave-one-file-out fold, evaluates
the fixed supplied stiffnesses in the same folds, fits all nine asphalt A
files once, and applies that full fit to the two mixed-surface files.
Model A-Speed's matching nine-fold results are in `IONIQ5_REFINEMENT.md`.

## Nine-fold results on asphalt A files

| Held-out file | A-Speed yaw RMSE | B yaw RMSE | B CoG `vy` RMSE | Fitted `Cf` / `Cr` (kN/rad) |
|---|---:|---:|---:|---:|
| `a_1` | 0.039013 | 0.030873 | 0.147778 | 98.8 / 153.0 |
| `a_2` | 0.041229 | 0.041621 | 0.122596 | 93.9 / 143.7 |
| `a_3` | 0.033667 | 0.031195 | 0.089575 | 93.9 / 145.5 |
| `a_4` | 0.038468 | 0.037118 | 0.081107 | 93.9 / 143.7 |
| `a_5` | 0.032670 | 0.029363 | 0.087342 | 92.8 / 143.7 |
| `a_6` | 0.046772 | 0.033210 | 0.092358 | 93.9 / 145.5 |
| `a_7` | 0.038294 | 0.036449 | 0.113067 | 95.1 / 138.4 |
| `a_8` | 0.045960 | 0.033936 | 0.096175 | 93.9 / 143.7 |
| `a_9` | 0.031713 | 0.026230 | 0.127227 | 93.9 / 143.7 |
| **Pooled, 29,971 scored samples** | **0.038669** | **0.032793** | **0.115194** | — |

B has lower yaw RMSE in eight of nine files and reduces pooled yaw RMSE
by about 15.2% versus A-Speed. The supplied 234.008/234.008 kN/rad reference
scores 0.076564 rad/s yaw and 0.229535 m/s CoG lateral velocity in the
same folds. The fitted stiffnesses are stable across folds and do not reach
the search bounds. The all-nine fit is `Cf=93941.306 N/rad` and
`Cr=143718.324 N/rad`; its **training** RMSE is 0.032221 rad/s yaw and
0.109673 m/s CoG lateral velocity. Use the preceding fold scores for model
comparison.

## Stiffness identifiability check

The 2D map evaluates the **all-nine asphalt A training objective** on a
61 × 61 grid: each stiffness ranges from 40 to 280 kN/rad in 4 kN/rad steps.
The objective uses yaw and CoG lateral-velocity scales from these same nine
files. It is not a held-out score. The grid minimum is at 96/144 kN/rad
with objective 0.051268; the continuous-search result is 93.941/143.718
kN/rad with objective 0.051217. The supplied 234.008/234.008 kN/rad point
is well outside the low-loss contours. The grid shows a localized minimum
for the chosen model and data rather than a long, nearly flat valley.

![Model B stiffness loss map](output/model_b_loss_map.png)

The finite-difference Hessian of the normalized training objective, using
`log(Cf)` and `log(Cr)` coordinates and a 0.02 log-unit step at the all-nine
fit, has eigenvalues 0.272656 and 1.030321 (ratio 3.778836). Both are
positive. This supports local separation of the two parameters on this
objective; the ratio is coordinate- and objective-dependent and is not a
physical uncertainty interval. The optimizer's 0.0125 log-unit final step
also limits how precisely the minimum can be localized.

| Nine leave-one-file-out fits | Mean (kN/rad) | Sample SD (kN/rad) | Range (kN/rad) |
|---|---:|---:|---:|
| `Cf` | 94.478 | 1.709 | 92.774–98.758 |
| `Cr` | 144.562 | 3.775 | 138.429–152.987 |

Across the nine fitted pairs, Pearson `corr(Cf, Cr)=+0.665`. The folds share
eight of nine files, so their scatter and correlation are a stability check,
not independent parameter confidence intervals. The objective combines both
measured signals: optimizing yaw alone or lateral velocity alone can favor
different stiffness pairs.

| Nine-fold pooled score | Fitted stiffness per fold | Supplied stiffness 234.008/234.008 kN/rad |
|---|---:|---:|
| Yaw-rate RMSE (rad/s) | **0.032793** | 0.076564 |
| CoG lateral-velocity RMSE (m/s) | **0.115194** | 0.229535 |

The identified `Cf` and `Cr` are 59.9% and 38.6% lower than the supplied
values, respectively. They imply a positive steady-state speed-squared
denominator term (`kv_B≈0.003322 s²/m`), qualitatively consistent with
A-Speed's positive `kv`; the supplied pair yields `kv_B≈-0.000405 s²/m`.
This is a comparison of model implications, not proof of a physical tire
property.

Reproduce the grid, fold summary, local curvature, and plot from the project
root without writing to `data/`:

```sh
build/model_b_ioniq5_cv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_[1-9].csv --loss-map output/model_b_loss_map.csv > output/model_b_identifiability.txt
PYTHONDONTWRITEBYTECODE=1 python3 -m tools.visualization.plot_model_b_loss --input output/model_b_loss_map.csv --output output/model_b_loss_map.png
```

The numeric [loss grid](output/model_b_loss_map.csv) and
[run log](output/model_b_identifiability.txt) preserve the plotted values and
fold-level fits. These results support a locally distinguishable *effective
pair* within this model and asphalt A data. They do not establish that the
pair equals independently measured tire stiffness: steering compliance, the
linear tire approximation, sensor translation, and the tested speed and
surface range can all affect the fitted values. The 20 ms samples also have
temporal correlation, so a sample-count-based confidence interval would be
misleading.

## New mixed-surface evaluation

These two files were not used for the nine asphalt A fits or the A-Speed
baseline fit. They were selected before inspecting their target errors.
Both models use their full nine-file fit for these predictions.

| New file | A-Speed yaw RMSE | B yaw RMSE | B CoG `vy` RMSE |
|---|---:|---:|---:|
| `dynamic_driving_asphalt_a_cobble_1.csv` | 0.039630 | 0.038480 | 0.099122 |
| `dynmic_asphalt_b_cobble_1.csv` | 0.104487 | 0.073892 | 0.154994 |
| **Pooled, 2,703 scored samples** | 0.067847 | **0.052648** | — |

The fitted B model improves yaw prediction on both files. On the second
file, the supplied-stiffness reference has lower CoG lateral-velocity RMSE
(0.132072 versus 0.154994 m/s), so no single pair of stiffnesses dominates
every signal and surface. These test results must not be used to tune the
current fit. The identified `Cf` and `Cr` are effective parameters for this
linear model, sensor processing, speed range, and dry-surface dataset.
Their difference from the supplied values alone does not establish a physical
change in tire stiffness.

## Surface transfer and operating domain

The full nine-file asphalt A fits of Model B and A-Speed were held fixed
while evaluating each available driving file from other surface groups.
The supplied stiffness pair is a separate, fixed comparator. No result in
this section was used to refit either model or select a new coefficient.
The `a_1`–`a_9` row is leave-one-file-out validation; the other rows use
the full asphalt A fit and are separate evaluation files.

| Evaluation | Scored samples | A-Speed yaw RMSE (rad/s) | B yaw RMSE (rad/s) | B CoG `vy` RMSE (m/s) | Supplied-stiffness yaw / `vy` RMSE |
|---|---:|---:|---:|---:|---:|
| Asphalt A, 9-fold | 29,971 | 0.038669 | **0.032793** | 0.115194 | 0.076564 / 0.229535 |
| Asphalt B `_1` | 718 | 0.027500 | **0.018696** | 0.083706 | **0.016199 / 0.066370** |
| Cobble `_1` | 1,891 | 0.070858 | **0.054080** | 0.124458 | **0.045572 / 0.104700** |
| Asphalt A–cobble `_1` | 1,826 | 0.039630 | **0.038480** | **0.099122** | 0.056049 / 0.152237 |
| Asphalt B–cobble `_1` | 877 | 0.104487 | 0.073892 | 0.154994 | **0.059016 / 0.132072** |

Model B beats A-Speed on yaw rate in all four external files, but the
supplied stiffness pair beats the identified pair on **both** signals in
Asphalt B, cobble, and Asphalt B–cobble. The asphalt A coefficients therefore
do not transfer as the best stiffness pair across these files. A single file
per group cannot isolate a road-surface effect from speed, steering, and
other drive-condition changes.

The operating-domain report uses only scored samples from the same 20 ms
segments as the RMSE calculation. Its three signals are CoG longitudinal
speed, absolute effective steering angle, and absolute measured lateral
acceleration at the rear axle. The limits and 5th–95th percentile bands are
computed **only from asphalt A training files**. A test value outside a
training range flags extrapolation on that signal; being inside each range
does not establish joint coverage or guarantee accuracy.

| Group | Median speed (m/s) | Median `abs(delta)` (rad) | Steering samples above asphalt A maximum | Steering samples outside asphalt A 5th–95th band |
|---|---:|---:|---:|---:|
| Asphalt A training | 8.315 | 0.208 | — | 10.0% |
| Asphalt B `_1` | 6.085 | 0.142 | 0% | 12.5% |
| Cobble `_1` | 4.720 | 0.526 | 0.7% | 64.7% |
| Asphalt A–cobble `_1` | 6.333 | 0.181 | 0% | 26.0% |
| Asphalt B–cobble `_1` | 5.156 | 0.609 | **74.6%** | **100%** |

Asphalt A's observed speed range is 1.963–15.556 m/s and absolute steering
range is 0.00015–0.59188 rad. The Asphalt B–cobble steering input is often
outside that training range; its model errors should be read as an
extrapolation result. Cobble mostly lies within the absolute steering range
but is concentrated near its upper end. Asphalt B lies inside all three
one-dimensional training ranges, yet its median speed and steering differ
from asphalt A. These are descriptive comparisons, not causes established
by the data.

Reproduce the fixed-fit comparison and automatic domain report:

```sh
build/model_b_ioniq5_cv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_[1-9].csv --test data/IONIQ5_Data/processed/dynamic_driving_asphalt_b_1.csv data/IONIQ5_Data/processed/dynamic_driving_cobble_1.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_cobble_1.csv data/IONIQ5_Data/processed/dynmic_asphalt_b_cobble_1.csv --domain-report output/model_b_operating_domain.csv > output/model_b_surface_generalization.txt
build/model_a_speed_cv --fit-all data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_[1-9].csv --test data/IONIQ5_Data/processed/dynamic_driving_asphalt_b_1.csv data/IONIQ5_Data/processed/dynamic_driving_cobble_1.csv data/IONIQ5_Data/processed/dynamic_driving_asphalt_a_cobble_1.csv data/IONIQ5_Data/processed/dynmic_asphalt_b_cobble_1.csv > output/model_a_speed_surface_generalization.txt
```

The [domain report](output/model_b_operating_domain.csv) records all three
signals' ranges, quantiles, and fractions outside asphalt A coverage for
each file. The [Model B run log](output/model_b_surface_generalization.txt)
and [A-Speed run log](output/model_a_speed_surface_generalization.txt) retain
the exact fitted coefficients and errors. The
[baseline record](config/ioniq5_model_b_asphalt_a_baseline.json) stores the
fixed coefficients, vehicle geometry, coordinate correction, and training
domain. The asphalt A Model B fit is a fixed **asphalt A baseline**; current
evidence does not support
labeling it a universal road-surface model.
