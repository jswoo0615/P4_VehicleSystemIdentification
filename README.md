# P4 Vehicle System Identification

This repository contains the C++17 identification and controller-model
runtime for the IONIQ 5 and Stanford VW Golf experiments. Raw driving logs
and generated build/output folders are kept locally. See `DATASETS.md` for
the expected input layout and `reference/README.md` for the tracked Golf V1
results. Nine C++ regression tests run without the raw datasets; five
additional VBO checks run when their original local files are present.

## Dataset acknowledgment

The Stanford VW Golf experiments use Mori et al. (2025), *Vehicle Dynamics
Dataset for Highly Dynamic Automated Driving*, Version 1, Stanford Digital
Repository ([DOI](https://doi.org/10.25740/hh613qz0317)). The dataset record
specifies [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/).
See [DATASET_ATTRIBUTION.md](DATASET_ATTRIBUTION.md) for the full citation,
source link, processing summary, and usage conditions. Original driving CSVs
are not included in this repository.

## Build and test

```sh
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release -j4
ctest --test-dir build-release --output-on-failure
```

## VW Golf Model A experiment

Run the existing Model A / A-Speed identifier on the Stanford-format CSV:

```sh
cmake --build build --target model_a_speed_cv -j4
./build/model_a_speed_cv --golf-single data/VehicleDynamicsDataset/Nov2023/2023-11_1.csv output/golf_a_001.json
```

`GOLF-A-001` trains on 20–70 s and validates on 75–100 s of the same
recording, at the source 200 Hz sample rate. It excludes speeds below 5 m/s
and uses nominal wheelbase 2.63 m. The JSON includes coefficients, RMSE,
and the training speed and road-wheel-angle ranges. This is a time-held-out
initial check; validation on another recording is still needed.

For five-fold validation across all October and November VW Golf CSVs:

```sh
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target model_a_speed_cv -j4
./build-release/model_a_speed_cv --golf-kfold data/VehicleDynamicsDataset output/golf_a_kfold_001.json
```

The program removes byte-identical files before assigning whole files to
folds. It uses 50 Hz samples for this cross-file experiment, with nominal
wheelbase 2.63 m and minimum speed 5 m/s. The JSON records fold and file
RMSE, plus a final fit using all unique files. The October bundle contains
two duplicate pairs; 20 unique CSVs remain. Pooled CV RMSE is 0.09230 rad/s
for Model A and 0.08708 rad/s for A-Speed.

Golf Model B is documented in [docs/golf/GOLF_MODEL_B.md](docs/golf/GOLF_MODEL_B.md),
including the normalized four-parameter fit, matching file-fold evaluation,
and parameter sensitivity. The training-only A-Speed assisted B1 comparison
is in [docs/golf/GOLF_B0_B1.md](docs/golf/GOLF_B0_B1.md).
The frozen reference coefficients and observed operating domain are in
`config/golf_baselines.json`; B0 is the retained Golf Model B baseline.
Run `python3 tools/p4_golf_export.py` to create the separate controller-facing
`output/golf_controller_model.json` without changing the identification fits.
The C++17 reload and 1,000-step numerical parity procedure is recorded in
`docs/golf/P4_EXPORT_PARITY.md`.
The separate controller-facing runtime, measured step timing, and existing-
data diagnostic predictions are recorded in `docs/golf/P4_CONTROLLER_INTEGRATION.md`.
`docs/golf/P4_GOLF_V1_STATUS.md` records the frozen artifact hashes, current validation
scope, and remaining gates. Finite-horizon file-fold prediction is in
`docs/golf/P4_PREDICTION_HORIZON.md`. The closed-loop harness in
`docs/golf/P4_CLOSED_LOOP_DEMO.md` has passed a synthetic feasibility check; it is not
VW Golf real-vehicle control validation.

The Golf A and B experiments share `GolfDataset` for byte-content duplicate
removal, CSV mapping, forward-driving preprocessing, and whole-file fold
membership. The identification algorithms and frozen V1 result files remain
separate. Re-running the three Golf cross-validation modes after this
refactor produced byte-identical JSON to the V1 results.

## Current IONIQ 5 status

| Model | Status | Entry point |
|---|---|---|
| S actuator (command → angle) | On hold: no measured command angle | `src/models/SteeringActuatorModel.cpp` is simulation only |
| S-Load S0 | Retained as an **offline** steering-load baseline | `apps/model_s_load_csv.cpp` |
| S-Load S1 tanh | Investigated, not adopted | `apps/model_s_load_friction_cv.cpp` |
| A0 constant gain | Comparison baseline | `apps/model_a_grouped_csv.cpp` |
| **A1 speed gain** | Frozen comparison reference for Model B | `apps/model_a_speed_cv.cpp` |
| B bicycle model | Asphalt A baseline frozen; surface transfer and operating domain reported | `apps/model_b_ioniq5_cv.cpp` |

A1 uses `tau*r_dot + r = v_x/(3.0 + kv*v_x²)*delta + b`. The original
three-file fit was `tau=0.053348 s`, `kv=0.005000`, `b=0.005965 rad/s`;
the later nine-file fit is `tau=0.044887 s`, `kv=0.005000`,
`b=0.002889 rad/s`.
The `kv` result is inside the `0–0.15` search range, not on a boundary.
Both coefficient sets and their file lists are recorded in
`config/ioniq5_baselines.json`. For Model B comparisons, use the same
file-level CV folds for both models; see `docs/ioniq5/IONIQ5_REFINEMENT.md` for results
and limitations.

## Project map

- `data/`: original and converted recordings. Keep intact.
- `include/`, `src/`: reusable C++17 readers and model simulators.
- `apps/`: command-line identification and diagnostic programs.
- `docs/`: vehicle-specific experiment reports and the earlier VBO audit.
- `tools/datasets/`: KIT IONIQ 5 and VBO conversion and inspection utilities.
- `tools/validation/`: file-group splits, cross-validation, and parity checks.
- `tools/visualization/`: model and maneuver plots.

- `tests/`: CTest regression checks.
- `output/`: generated reports and figures; ignored by Git.
- `build/`: generated CMake build; ignored by Git.

See [docs/README.md](docs/README.md) for the vehicle-specific experiment
reports. See `tools/README.md` for the current Python commands and the roles of the
older one-file and VBO utilities.

The older VBO workflow remains available for comparison and teaching. See
`docs/legacy/AUDIT.md` for its classification and
`docs/ioniq5/IONIQ5_CONVERSION.md` for the IONIQ 5 data conversion record.
`docs/ioniq5/IONIQ5_EXPERIMENTS.md` gives the first held-out A0 and S0 results.
`docs/ioniq5/IONIQ5_REFINEMENT.md` gives the A1/S1 comparison and
reproducible commands. `docs/ioniq5/IONIQ5_MODEL_B.md` gives the Model B frame correction,
identification method, and comparison with A-Speed.

## Verify

```sh
cmake -S . -B build
cmake --build build -j4
ctest --test-dir build --output-on-failure
```
