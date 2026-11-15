# Python tools

Run the commands below from the project root. Module commands keep imports
independent of the caller's current Python path. CMake and CTest do not call
these scripts directly.

| Area | Purpose |
|---|---|
| `datasets/kit_ioniq5/` | MAT inspection and IONIQ 5 CSV conversion |
| `datasets/vbo/` | Legacy VBO audit, segmentation, run comparison, shared signal CSV reader |
| `validation/` | Grouped holdout and K-fold splits, cross-validation, parity checks |
| `visualization/` | Held-out IONIQ 5, Model A, and maneuver plots |

## Current commands

```sh
python3 -m tools.visualization.plot_heldout_ioniq5 --report-dir output/heldout_diagnostics --raw-dir data/IONIQ5_Data/processed
python3 -m tools.validation.group_kfold --input output/dataset_manifest.csv --output-dir output/cv --folds 5 --seed 42
python3 -m tools.validation.run_cross_validation --cv-dir output/cv --build-dir build
python3 tools/p4_golf_export.py
```

`p4_golf_export.py` reads the frozen Golf baseline and validation summary,
checks that their B0 values and fold metadata agree, and writes
`output/golf_controller_model.json`. It does not refit a model or modify the
source files. Use `--baseline`, `--validation`, and `--output` to select other
paths. The controller export contains A, A-Speed, and B0 only.

`batch_export_ioniq5.py` is the complete KIT conversion entry point and
**writes under `data/`** when run with defaults. It was not run during this
reorganization. `read_mat.py` supplies its MCOS array extractor and also has
a diagnostic command that writes `extracted_arrays.npz`. `export_ioniq5.py`
is an older, hard-coded one-file export from that NPZ. It remains available
for provenance, but the batch exporter is the normal conversion path.

`split_dataset.py` creates one grouped train/test split; `group_kfold.py`
creates multiple grouped folds. Their roles differ, so both remain. The
validation commands share `validation/csv_io.py` for CSV input and output. The
held-out IONIQ 5 plotter reads the C++ prediction exports; `plot_model_a.py`
plots the older VBO prediction schema. They are kept separate because their
CSV inputs and reports differ. VBO run comparison and maneuver plotting now
share `datasets/vbo/signals.py` for signal CSV loading.
