# VW Golf B0 vs B1: using A-Speed information

This experiment compares two versions of the same normalized Model B on the
same 20 unique VW Golf CSVs and the same five whole-file folds. B0 fits the
four Model B parameters from yaw rate and lateral velocity alone. B1 uses
the same data objective plus a weak training-only A-Speed consistency term.
Both use the existing `LinearBicycleModel` integrator, road bank input,
5 m/s minimum speed, and 50 Hz sampling described in `GOLF_MODEL_B.md`.

The linear bicycle model implies

`kv_B = ((L-lf)/(Cf/m) - lf/(Cr/m))/L`, with `L=2.63 m`.

For **each fold**, A-Speed is fitted afresh on that fold's training files.
Its coarse `kv` grid is 0.0025 apart; a second training-only grid searches
the winning neighborhood at 0.00025 spacing. The held-out files do not enter
either search. B1 adds to the training objective

`0.05 * ((kv_B - kv_A_training)/0.0025)^2`.

The fixed weight and scale are not tuned against held-out scores. The term
guides the fit; it does not hard-code A-Speed's full-data `kv` or fix an
individual Model B coefficient. An A-Speed first-order time constant is not
equated to `Iz/m`, and its output bias is not treated as tire stiffness.

| Fold | Training-only A-Speed `kv` | B0 yaw RMSE | B1 yaw RMSE | B0 `Vy` RMSE | B1 `Vy` RMSE |
|---|---:|---:|---:|---:|---:|
| 1 | 0.00575 | 0.06929 | 0.06979 | 0.32227 | 0.32311 |
| 2 | 0.00500 | 0.13327 | 0.13359 | 0.32320 | 0.32358 |
| 3 | 0.00525 | 0.08556 | 0.08556 | 0.35367 | 0.35367 |
| 4 | 0.00575 | 0.05775 | 0.05827 | 0.34172 | 0.34233 |
| 5 | 0.00525 | 0.07150 | 0.07186 | 0.31543 | 0.31393 |
| **Pooled** | — | **0.08543** | 0.08576 | **0.33105** | 0.33120 |

Yaw RMSE is in rad/s; `Vy` RMSE is in m/s. B1 moves its implied `kv`
toward the training A-Speed target, but this does not improve pooled held-out
prediction. B1's pooled yaw RMSE is about 0.38% higher and its pooled `Vy`
RMSE about 0.04% higher. No clear parameter-stability gain appears: across
five overlapping fold fits, the sample SD of `Cf/m` and `Iz/m` rises, that
of `Cr/m` falls, and that of `lf` is unchanged. These fold SDs are a
stability diagnostic, not independent parameter confidence intervals.

The earlier coarse-grid A-Speed pooled yaw RMSE was 0.08708 rad/s. Finer
training-only `kv` search gives 0.08828 rad/s on the same folds. The small
held-out deterioration is consistent with coarse-grid regularization, but
five folds do not establish its cause. The full-data B0 fit already implies
`kv_B≈0.00536`, close to A-Speed's full-data fine-grid `kv=0.00550`, so the
weak prior contributes little new information here. Retain B0 as the
current reference; B1 is an evaluated alternative, not a validated gain.

## Reproduce

```sh
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target model_b_golf_cv -j4
./build-release/model_b_golf_cv --compare-a-speed data/VehicleDynamicsDataset output/golf_b01_kfold_001.json
```

The JSON includes fold and file RMSEs, training-only A-Speed coefficients,
both Model B parameter sets, and full-data reference fits. The full-data
fits are not used as priors in cross-validation.
