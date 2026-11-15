# P4 Vehicle System Identification — audit and safe refactor

## Baseline and scope

Before editing, the CMake build passed, and the 9 registered CTest cases passed. The
stored dataset contains 46 events in 38 recording groups. A fresh five-fold run
reproduced Model A OOF RMSE 2.148947 deg/s, OOF MAE 1.532943 deg/s, pooled RMSE
2.702530 deg/s, and fold RMSE standard deviation 0.188051 deg/s. The five test
folds contain 9, 9, 9, 9, and 10 events. `vblog_042.vbo` is excluded from the
manifest as a verified duplicate of `vblog_035.vbo`; both raw files remain in `data/`.

## File classification

Classification is about the original VBO pipeline. ARCHIVE means retain the
source file for later use. The later IONIQ 5 cleanup is recorded below.

| Classification | Files | Reason |
| --- | --- | --- |
| KEEP | `CMakeLists.txt`; `include/p4/io/VBOData.hpp`, `VBOReader.hpp`; `src/io/VBOData.cpp`, `VBOReader.cpp`, `VBOParsingUtils.hpp`; `src/io/parsers/SectionedVBOParser.hpp/.cpp`, `TabularVBOParser.hpp/.cpp` | One public VBO reader dispatches to two genuinely different formats. Both parser implementations and shared numeric/column helpers are needed. |
| KEEP | `include/p4/data/VehicleMeasurement.hpp`; `include/p4/maneuvers/Maneuver.hpp`, `ManeuverDetector.hpp`, `ManeuverSegmenter.hpp`, `ManeuverGrouper.hpp`; matching `src/maneuvers/*.cpp` for those three classes | Measurement and maneuver segmentation structures used by the pipeline. `VehicleMeasurement.hpp` has no current direct reference, but is a stable domain type and should be retained. |
| KEEP | `include/p4/models/FirstOrderYawModel.hpp`, `SteeringActuatorModel.hpp`, `LinearBicycleModel.hpp`, `ModelARollout.hpp`; matching `src/models/*.cpp`; `include/p4/identification/ManifestEvent.hpp` | Independent Model A, S, and B simulation classes; shared Model A rollout and split unit. |
| KEEP | `apps/dataset_manifest.cpp`, `model_a_train.cpp`, `model_a_evaluate.cpp`; `tools/validation/split_dataset.py`, `group_kfold.py`, `run_cross_validation.py`; `tools/visualization/plot_model_a.py` | Default manifest, split, training, frozen evaluation, CV, and plotting path. |
| KEEP | `tests/model_a_test.cpp`, `model_s_test.cpp`, `model_s_boundary_test.cpp`, `model_b_test.cpp`, `vbo_data_test.cpp`, `vbo_parsing_utils_test.cpp`, `sectioned_vbo_parser_test.cpp`, `tabular_vbo_parser_test.cpp`, `vbo_reader_integration_test.cpp`, `vbo_reader_test.cpp`, `vbo_reader_002_test.cpp`, `maneuver_detector_test.cpp` | Regression checks. The two standalone reader tests were previously not registered with CTest. |
| MERGE | Model A rollout in `apps/model_a_train.cpp`, `model_a_evaluate.cpp`, `model_a_identify_vbo.cpp` | These used the same exact discrete first-order update; now call `simulateModelA`. The parameter-identification basis recursion remains separate. |
| MERGE | Manifest event fields in `apps/model_a_train.cpp` and `model_a_evaluate.cpp` | Both now use `ManifestEvent`. Their CSV readers still duplicate parsing/validation and are a future merge candidate. |
| MERGE | VBO parser error formatting in `src/io/parsers/SectionedVBOParser.cpp` and `TabularVBOParser.cpp` | Small duplication; merge only if it simplifies diagnostics. Parser formats must remain separate. |
| ARCHIVE | `include/p4/maneuvers/MajorExcursionDetector.hpp`, `ManeuverPhaseAnalyzer.hpp`; matching `src/maneuvers/*.cpp`; `include/p4/identification/IdentificationWindowSelector.hpp`, `src/identification/IdentificationWindowSelector.cpp` | Exploratory phase and window analysis. The selector is not currently linked to a CMake target, so preserve it until its dependencies and intended API are confirmed. |
| ARCHIVE | `apps/model_a_identify_vbo.cpp`, `vbo_data_analysis.cpp`, `maneuver_analysis.cpp`, `maneuver_export.cpp`, `maneuver_segment_analysis.cpp`, `major_excursion_analysis.cpp`, `maneuver_phase_analysis.cpp`, `identification_window_analysis.cpp` | Standalone diagnostics or earlier single-event workflow, outside default batch CV. Keep runnable for teaching and comparison. |
| ARCHIVE | `tools/datasets/vbo/` analysis scripts, `tools/validation/validate_cpp_phases.py`, `compare_python_cpp_segments.py`, `tools/visualization/plot_maneuvers.py` | Independent analysis, parity checks, plots, and raw-data audit. |
| REMOVED GENERATED | Historical generated `output/cv_old/`, partial `output/cv_balanced/`, root `vbo_analysis.csv`, and root `maneuver_segments/` | Reviewed during the 2026-11-15 cleanup: no build or default pipeline consumer. Current `output/cv/` and IONIQ 5 results were retained. |
| KEEP | `data/old_vito_not_used/vblog_001.vbo` through `vblog_042.vbo` | Original recordings; never delete as part of cleanup. |

The original tree contained **no Model B source or header**. This refactor adds a
standalone 2DOF linear bicycle simulator with `(v_y, r)` states and a direct
unit test. It uses SI units, radians, linear axle tire forces, and explicit
Euler integration; its physical parameters are defaults, not identified values.
Model S has a simulation implementation and tests. Neither S nor B has a data
trainer, serializer, or frozen evaluator yet. Do not describe either as having
the full Train → Save → Load → Test workflow.

## Safe implementation sequence

1. Preserve raw data under `data/` and write generated manifests, models, CV
   results, predictions, and figures under `output/`. The VBO quality CSV and
   Python maneuver plots now use that directory. Superseded root analysis
   artifacts were removed in the later cleanup; `.gitignore` prevents new
   generated files entering Git.
2. Use `VBOReader` as the only public VBO import API. Keep its format dispatch
   and parser-specific behavior. Any future parser deduplication must retain the
   sectioned and tabular integration tests.
3. Use `ManifestEvent` for the train/test event identity and group boundary.
   Keep the split on `group_id`; all events from one recording stay in one fold.
4. Use the common Model A rollout for training diagnostics, frozen evaluation,
   and the legacy single-event identifier. Freeze parameters before test data.
5. Add Model S data identification and model persistence only after confirming
   measured command and actual steering channels. Identify Model B parameters
   only after checking lateral velocity and yaw signal units and sign conventions.
   Give each model a typed trainer, serializer,
   loader, and evaluator. Avoid forcing dissimilar model states into one `step`
   signature.

## Reproduction commands

From this directory:

```sh
cmake -S . -B build
cmake --build build -j4
ctest --test-dir build --output-on-failure
build/dataset_manifest output/dataset_manifest.csv data/old_vito_not_used/*.vbo
python3 -m tools.validation.group_kfold --input output/dataset_manifest.csv --output-dir output/cv --folds 5 --seed 42
python3 -m tools.validation.run_cross_validation --cv-dir output/cv --build-dir build
```

The reproduction uses the repository root as the working directory because
manifest file paths are relative to that directory. Check every fold for
disjoint train/test `group_id` sets and exactly one test appearance per event.

## IONIQ 5 cleanup update (2026-11-15)

S0 and S1 now call the same guarded-update reader in
`src/io/IONIQ5LoadSamples.cpp`. The shared reader preserves the original
centered-difference sample selection and also exposes the previous-update
rate sensitivity check. The original S0 coefficients and RMSEs were verified
unchanged after the refactor. The historical generated files listed above and
the root `.DS_Store` were removed. `data/` was not changed.
