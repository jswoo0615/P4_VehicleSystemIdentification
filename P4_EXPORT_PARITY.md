# P4-EXPORT-PARITY-001

This check reloads `output/golf_controller_model.json` in C++17 and compares
its Model A, A-Speed, and B0 coefficients against the separately parsed
`config/golf_baselines.json`. The loader requires
`schema=p4.controller_model.v1`, finite coefficients, physically valid
positive parameters, `lf+lr=L`, and the exported B0 bank expression.
Missing or changed schema, missing parameters, NaN, infinity, overflowed
numbers, and a valid but altered coefficient are rejected by the test.

The two coefficient sets are then given identical initial states and 1,000
deterministic 20 ms steering, speed, and bank inputs. Model A uses the
existing `FirstOrderYawModel`; A-Speed uses the same exact-discrete-time
rollout formula as the identification app; B0 uses the existing
`LinearBicycleModel::stepImplicitMidpoint` with the same 20 ms maximum
substep and `9.80665*sin(bank)` external lateral acceleration as the Golf
identifier. No separate Euler implementation is used.

Acceptance limits: maximum parameter relative error `1e-12`, single-step
absolute prediction error `1e-10`, and maximum 1,000-step absolute error
`1e-8`. On the current export all three measured errors are zero. This is
a reload and numerical-path check, not new physical validation of the
underlying Golf model or the bank-angle sign convention.

Run from the project root:

```sh
python3 tools/p4_golf_export.py
cmake -S . -B build
cmake --build build --target controller_export_parity -j4
./build/controller_export_parity output/golf_controller_model.json config/golf_baselines.json output/golf_export_parity.json
ctest --test-dir build --output-on-failure
```

The actual-export result is `output/golf_export_parity.json`. CTest also
runs a tracked fixture against the frozen baseline, so the loader and
prediction parity check remain covered when generated `output/` files are
absent from a fresh checkout.
