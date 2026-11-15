# P4-CONTROLLER-INTEGRATION-001

`controller_integration` is a small C++17 executable separate from the
identification programs. It loads `golf_controller_model.json` through the
controller-model loader and drives `ControllerRuntime` with normalized
per-sample inputs: road-wheel steering angle (rad), longitudinal speed
(m/s), road bank (rad), and elapsed time (s). The runtime returns Model A
and A-Speed yaw rate and Model B0 yaw rate and lateral speed. It uses the
existing `FirstOrderYawModel` and `LinearBicycleModel::stepImplicitMidpoint`.
The latter receives `9.80665*sin(bank)` as external lateral acceleration;
the bank sign still needs physical verification against Stanford metadata.

The CSV adapter reads the Stanford channel names, initializes all states
from the first measured yaw rate and lateral speed in each continuous
forward-driving segment, and steps with the preceding sample's inputs.
Speeds below 5 m/s and gaps above 80 ms cause a reset. This is an adapter
for this diagnostic, while `ControllerRuntime::step` is the controller-facing
input interface. Parsing and file output are outside the timed step.

## Current-host diagnostics

| Existing-data interval | Scored steps | A yaw RMSE | A-Speed yaw RMSE | B0 yaw RMSE | B0 `Vy` RMSE |
|---|---:|---:|---:|---:|---:|
| `2023-11_1.csv`, 75–100 s | 4,999 | 0.16656 | 0.17983 | 0.17119 | 0.29971 |
| `2023-10_6.csv`, full recording | 21,111 | 0.20406 | 0.20834 | 0.20163 | 0.43135 |

Yaw RMSE is in rad/s and `Vy` RMSE in m/s. The second file had 1,003
low-speed rows excluded. On this host's Release build, both runs measured
an approximately 84 ns p99 `step` duration, no calls to C++ `operator new`
inside `step`, and no measured step above 5 ms. The timer's granularity,
the operating system, and the target controller hardware limit what these
numbers predict about hard real-time behavior. The allocation counter
covers C++ `new` calls in the measured step, not every possible allocator.

**These are in-sample diagnostics.** The exported coefficients were fitted
using all 20 unique CSVs, including both files above. The earlier whole-file
five-fold CV remains the evidence for generalization until a new recording
is available. The higher-error full `2023-10_6.csv` run illustrates the
remaining file-level variation.

The 1,000-step export parity test now also compares `ControllerRuntime`
against independently constructed baseline model instances; its maximum
absolute difference is zero on the current export.

## Reproduce

```sh
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target controller_integration -j4
./build-release/controller_integration output/golf_controller_model.json data/VehicleDynamicsDataset/Nov2023/2023-11_1.csv 75 100 output/golf_controller_integration_001.json output/golf_controller_integration_001.csv
./build-release/controller_integration output/golf_controller_model.json data/VehicleDynamicsDataset/Oct2023/2023-10_6.csv 0 111 output/golf_controller_integration_stress.json output/golf_controller_integration_stress.csv
```
