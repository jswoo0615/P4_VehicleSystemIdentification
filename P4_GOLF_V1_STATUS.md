# VW Golf A / A-Speed / B0 V1 status

The Golf V1 coefficients in `config/golf_baselines.json` remain frozen. The
controller export and validation outputs below are separate artifacts, not
new parameter fits. Byte-for-byte copies of the small frozen output JSONs
are tracked in `reference/v1/`; `reference/README.md` explains how to restore
them to the ignored `output/` directory after cloning. These SHA-256 values
were checked against both copies on 2026-11-16; the three source hashes
embedded in the controller export match the baseline, validation summary,
and B0/B1 CV JSON.

| Frozen artifact | SHA-256 |
|---|---|
| `config/golf_baselines.json` | `fb9f4bf3718aa0373fcd6d0f753d8afd8a83449864d435b9522ea2159f9eaff2` |
| `output/golf_controller_model.json` | `2a33c3829a5cba0b92798d8792b4b32243d1ea65944d1d1be512956191f9b015` |
| `output/golf_validation_summary.json` | `3ef6c84e50f6f48bb4143cb08a7f1216597826ba5f2a666d0d7c9b4bf878dba5` |
| `output/golf_a_kfold_001.json` | `dd0848aac55f1c62a16ae15cb3309f7aa3c8adfe3cefab3d52f6bc0a4ce6f122` |
| `output/golf_b_kfold_001.json` | `faaf9dc787f05839662609b05fefb9dd09493ee288d5ae68c4018a6d173cce66` |
| `output/golf_b01_kfold_001.json` | `c53cbd6898d056cc636e885f5b2f18c6fa0a7d3bcd01f7aa7d158c21be727646` |

## Evidence and scope

- The original C++17 Golf file-group five-fold RMSE is 0.09230 rad/s for A,
  0.08708 rad/s for A-Speed, and 0.08543 rad/s yaw plus 0.33105 m/s Vy for B0.
- Export reload and prediction parity, plus independent `ControllerRuntime`
  execution, are documented in `P4_EXPORT_PARITY.md` and
  `P4_CONTROLLER_INTEGRATION.md`.
- `P4_PREDICTION_HORIZON.md` reports independent 20-file, training-fold-only
  finite-horizon prediction. B0 is strongest relative to A-Speed at 0.1–0.2 s;
  A-Speed is better at 0.5 s in every fold. This uses recorded future inputs.
- `P4_CLOSED_LOOP_DEMO.md` records `P4-GOLF-CLOSED-LOOP-001 — Synthetic
  Feasibility PASS`. Its arbitrary nonlinear bicycle plants demonstrate a
  working controller comparison harness only. They do not establish VW Golf
  real-vehicle closed-loop performance.

## Open validation gates

1. Confirm Stanford bank-angle and acceleration coordinate definitions from
   authoritative dataset metadata and check the sign used by B0.
2. Revisit the weak separation of `lf_m` and `Iz/m` before interpreting them
   as physical vehicle parameters. The V1 values are effective predictors.
3. Evaluate new, unused driving files when available. The existing 20-file
   folds remain the independent evidence for this dataset.
4. Connect a separately calibrated vehicle plant or real control experiment
   before ranking A-Speed and B0 as controllers for the actual Golf. Include
   steering-command dynamics and online speed/bank forecasts.
5. Integrate Model C through the same import, validation, export, and runtime
   gates when its owner supplies a completed roll model.

The regression suite passed 14/14 after the horizon experiment. The synthetic
demo leaves all six frozen artifacts above unchanged.
