# P4 closed-loop feasibility demo

`P4-GOLF-CLOSED-LOOP-001` is recorded as **Synthetic Feasibility PASS**, not a
VW Golf closed-loop validation. The project has no independent calibrated
Golf plant or verified Golf steering-command actuator. The three evaluation
plants in `config/golf_closed_loop_demo_plants.json` are illustrative nonlinear
two-state bicycle systems with manually selected coefficients and `tanh`
tire-force saturation. They share the general bicycle structure with B0,
though their coefficients and tire law differ. No plant parameter was fitted
to the 85–88 s Golf data.

Both controllers use the same one-move receding-horizon grid search. The
candidate road-wheel angle is held constant over a 0.2 s prediction window;
only its first 20 ms is applied before replanning. Both controllers see the
same simulated `Vy` and yaw state, the same recorded 85–88 s yaw target, and
the same future recorded speed and bank. A-Speed predicts yaw only; B0 predicts
`Vy` and yaw. The control objective, candidate grid, steering limits, and
weights are identical. The plant has direct road-wheel steering: actuator
dynamics, command-angle uncertainty, sensor noise, and a real MPC optimizer
are absent.

Run from the project root:

```sh
python3 tools/p4_golf_closed_loop_demo.py data/VehicleDynamicsDataset/Nov2023/2023-11_1.csv --horizon-steps 10 --out output/golf_closed_loop_demo_001
python3 tools/p4_golf_closed_loop_demo.py data/VehicleDynamicsDataset/Nov2023/2023-11_1.csv --horizon-steps 5 --out output/golf_closed_loop_demo_h5
python3 tools/p4_golf_closed_loop_demo.py data/VehicleDynamicsDataset/Nov2023/2023-11_1.csv --horizon-steps 25 --out output/golf_closed_loop_demo_h25
```

At 0.2 s, the yaw tracking RMSE (rad/s) is:

| Synthetic plant | A-Speed | B0 |
|---|---:|---:|
| Nominal | 0.03943 | 0.04139 |
| Soft | 0.15316 | 0.15225 |
| Stiff | 0.04468 | 0.04702 |

Both controllers met the same absolute steering and steering-rate limits.
On the soft plant, both exceeded the illustrative 0.15 rad/s tracking-error
threshold on 25.5% of scored samples; B0 reached the absolute steering limit
on 2.7% of scored samples without violating it. B0 used about 31% more RMS
steering there, so its small tracking-RMSE advantage is not a control-
efficiency claim.

At 0.1 s, B0 is slightly better in all three synthetic plants. At 0.5 s,
A-Speed is slightly better in the nominal and stiff plants. This sensitivity
to plant assumptions and controller horizon is the result of the demo; it is
not evidence that either controller is superior on a real Golf. The
`tracking_error_over_0p15_fraction` report field means the fraction of steps
with absolute yaw tracking error above an illustrative 0.15 rad/s threshold,
not a safety or solver failure rate. The frozen V1 model coefficients remain
unchanged.

Before claiming Golf control performance, connect both controllers to the
same independently calibrated vehicle plant or CARLA setup, define the
command-to-wheel-angle actuator and state-estimation interface, and replace
recorded future speed/bank with values available to an online controller.
