# Golf finite-horizon prediction experiment

`P4-GOLF-PREDICTION-HORIZON-001` evaluates 0.1, 0.2, 0.5, and 1.0 s predictions
at the original 50 Hz Golf sample rate. Every forecast starts from measured
`Vy_mps` and `yawRate_radps`. During each forecast, A, A-Speed, and B0 receive
the recorded future road-wheel angle, longitudinal speed, and road bank.
Forecasts never cross a preprocessed driving-segment boundary. RMSE pools
overlapping forecast origins, so adjacent errors are correlated.

The 20-file result uses the unchanged five whole-file folds. A, A-Speed, and
B0 each use their own corresponding fold's training-only coefficients. The
five fold parameter rows are extracted from the existing A and B0 cross-
validation JSONs. The C++ executable reuses `GolfDataset` preprocessing and
`ControllerRuntime` dynamics. Its `horizon_steps=0` rows repeat each original
full free run; their pooled RMSE agrees with the four frozen CV references to
within `5e-16`.

Run from the project root:

```sh
python3 tools/p4_golf_horizon_params.py
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target golf_prediction_horizon -j4
./build-release/golf_prediction_horizon --cv data/VehicleDynamicsDataset output/golf_horizon_fold_params.csv output/golf_prediction_horizon_cv_001.csv
./build-release/golf_prediction_horizon --diagnostic data/VehicleDynamicsDataset/Nov2023/2023-11_1.csv config/golf_baselines.json 75 100 85 88 output/golf_prediction_horizon_85_88_001.csv
python3 tools/p4_golf_horizon_report.py
```

The local 85–88 s result uses the full-data refit and is diagnostic only. It
scores forecasts whose **target** lies in 85–88 s; forecast origins may be
earlier. The file CV is the independent evaluation. These are measured-input
model checks, not online closed-loop MPC performance: an online controller
would need planned future steering and a speed/bank forecast.

| Horizon | A yaw RMSE | A-Speed yaw RMSE | B0 yaw RMSE | B0 Vy RMSE | B0 yaw wins over A-Speed |
|---:|---:|---:|---:|---:|---:|
| 0.1 s | 0.05517 | 0.04917 | 0.04316 | 0.13036 | 17/20 files |
| 0.2 s | 0.07646 | 0.06925 | 0.06531 | 0.21563 | 16/20 files |
| 0.5 s | 0.09117 | 0.08525 | 0.08791 | 0.33330 | 6/20 files |
| 1.0 s | 0.09269 | 0.08745 | 0.08670 | 0.34326 | 15/20 files |

Yaw units are rad/s and Vy units are m/s. B0 has its clearest yaw advantage
at 0.1 s. At 0.5 s, A-Speed is better in every fold. The 1.0 s pooled B0
advantage is small. These results do not change the frozen V1 coefficients.
