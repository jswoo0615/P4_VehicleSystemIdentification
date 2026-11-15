#!/usr/bin/env python3
"""Compare A-Speed and B0 receding-horizon steering on synthetic plants."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def plant_derivative(state, steer, vx, bank, p):
    vy, yaw = state
    front_slip = steer - (vy + p["lf_m"] * yaw) / vx
    rear_slip = -(vy - p["lr_m"] * yaw) / vx
    saturation = p["slip_saturation_rad"]
    front = p["cf_over_m"] * saturation * np.tanh(front_slip / saturation)
    rear = p["cr_over_m"] * saturation * np.tanh(rear_slip / saturation)
    return np.array([front + rear - vx * yaw + 9.80665 * np.sin(bank),
                     (p["lf_m"] * front - p["lr_m"] * rear) / p["iz_over_m"]])


def plant_step(state, steer, vx, bank, dt, p):
    k1 = plant_derivative(state, steer, vx, bank, p)
    k2 = plant_derivative(state + 0.5 * dt * k1, steer, vx, bank, p)
    k3 = plant_derivative(state + 0.5 * dt * k2, steer, vx, bank, p)
    k4 = plant_derivative(state + dt * k3, steer, vx, bank, p)
    return state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def b0_step(vy, yaw, steer, vx, bank, dt, p):
    cf = p["cf_over_m_mps2_per_rad"]
    cr = p["cr_over_m_mps2_per_rad"]
    iz = p["iz_over_m_m2"]
    lf, lr = p["lf_m"], p["lr_m"]
    a11 = -(cf + cr) / vx
    a12 = (-lf * cf + lr * cr) / vx - vx
    a21 = (-lf * cf + lr * cr) / (iz * vx)
    a22 = -(lf * lf * cf + lr * lr * cr) / (iz * vx)
    b1 = cf * steer + 9.80665 * np.sin(bank)
    b2 = lf * cf * steer / iz
    # Match ControllerRuntime's 20 ms maximum midpoint substep.
    steps = max(1, int(np.ceil(dt / 0.02)))
    h = dt / steps
    m11, m12 = 1 - h * a11 / 2, -h * a12 / 2
    m21, m22 = -h * a21 / 2, 1 - h * a22 / 2
    det = m11 * m22 - m12 * m21
    for _ in range(steps):
        rhs1 = (1 + h * a11 / 2) * vy + h * a12 / 2 * yaw + h * b1
        rhs2 = h * a21 / 2 * vy + (1 + h * a22 / 2) * yaw + h * b2
        vy, yaw = (rhs1 * m22 - m12 * rhs2) / det, (m11 * rhs2 - m21 * rhs1) / det
    return vy, yaw


def choose_steering(model, state, previous, index, times, reference, speed, bank,
                    horizon, limits, baseline):
    dt = times[index + 1] - times[index]
    lower = max(-limits["steering_abs_rad"], previous - limits["steering_rate_radps"] * dt)
    upper = min(limits["steering_abs_rad"], previous + limits["steering_rate_radps"] * dt)
    candidates = np.linspace(lower, upper, limits["candidate_count"])
    vy = np.full_like(candidates, state[0])
    yaw = np.full_like(candidates, state[1])
    cost = np.zeros_like(candidates)
    steps = min(horizon, len(times) - index - 1)
    for offset in range(steps):
        k = index + offset
        h = times[k + 1] - times[k]
        if model == "A-Speed":
            p = baseline["model_a_speed"]
            factor = np.exp(-h / p["tau_s"])
            steady = speed[k] * candidates / (
                p["nominal_wheelbase_m"] + p["kv_s2_per_m"] * speed[k] ** 2)
            yaw = factor * yaw + (1 - factor) * (steady + p["bias_rad_per_s"])
        else:
            vy, yaw = b0_step(vy, yaw, candidates, speed[k], bank[k], h,
                              baseline["model_b0"])
        cost += h * ((yaw - reference[k + 1]) ** 2 +
                     limits["steering_weight"] * candidates ** 2)
    cost += limits["move_weight"] * (candidates - previous) ** 2
    return float(candidates[np.argmin(cost)])


def simulate(model, plant, times, reference, speed, bank, initial, initial_steer,
             horizon, limits, baseline):
    states = np.empty((len(times), 2))
    steering = np.empty(len(times))
    states[0] = initial
    steering[0] = initial_steer
    for i in range(len(times) - 1):
        command = choose_steering(model, states[i], steering[i], i, times, reference,
                                  speed, bank, horizon, limits, baseline)
        steering[i + 1] = command
        states[i + 1] = plant_step(states[i], command, speed[i], bank[i],
                                   times[i + 1] - times[i], plant)
        if not np.isfinite(states[i + 1]).all():
            raise ValueError(f"Synthetic plant diverged for {plant['name']}")
    return states, steering


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--baseline", type=Path, default=Path("config/golf_baselines.json"))
    parser.add_argument("--plants", type=Path, default=Path("config/golf_closed_loop_demo_plants.json"))
    parser.add_argument("--start", type=float, default=85)
    parser.add_argument("--end", type=float, default=88)
    parser.add_argument("--horizon-steps", type=int, default=10)
    parser.add_argument("--out", type=Path, default=Path("output/golf_closed_loop_demo_001"))
    args = parser.parse_args()
    baseline = json.loads(args.baseline.read_text())
    plants = json.loads(args.plants.read_text())["plants"]
    source = pd.read_csv(args.csv, comment="#")
    source = source[(source.t_s >= args.start) & (source.t_s <= args.end)].iloc[::4]
    times = source.t_s.to_numpy()
    reference = source.yawRate_radps.to_numpy()
    speed = source.Vx_mps.to_numpy()
    bank = source.bank_rad.to_numpy()
    initial = np.array([source.Vy_mps.iloc[0], reference[0]])
    initial_steer = float(source.roadWheelAngle_rad.iloc[0])
    if len(times) < 60 or np.any(np.diff(times) <= 0) or np.max(np.diff(times)) > 0.025:
        raise ValueError("Need a continuous 50 Hz reference of at least 1.2 s")
    if np.min(speed) < 5 or not np.isfinite(source.to_numpy()).all():
        raise ValueError("Invalid reference or speed below 5 m/s")
    limits = {"steering_abs_rad": 0.18, "steering_rate_radps": 0.8,
              "candidate_count": 9, "steering_weight": 0.005, "move_weight": 0.03}
    horizon = args.horizon_steps
    if horizon < 1:
        raise ValueError("Prediction horizon must be positive")
    records, rows = [], []
    fig, axes = plt.subplots(len(plants), 2, figsize=(11, 3.3 * len(plants)),
                             sharex=True, layout="constrained")
    for plant_index, plant in enumerate(plants):
        axes[plant_index, 0].plot(times, reference, "k", label="Target yaw")
        for model in ("A-Speed", "B0"):
            states, steering = simulate(model, plant, times, reference, speed, bank,
                                        initial, initial_steer, horizon, limits, baseline)
            error = states[:, 1] - reference
            dt = np.diff(times)
            slew = np.abs(np.diff(steering) / dt)
            if np.max(abs(steering)) > limits["steering_abs_rad"] + 1e-12 or \
               np.max(slew) > limits["steering_rate_radps"] + 1e-12:
                raise AssertionError("Control constraints violated")
            records.append({
                "plant": plant["name"], "controller": model,
                "yaw_tracking_rmse_radps": float(np.sqrt(np.mean(error[1:] ** 2))),
                "yaw_tracking_max_abs_radps": float(np.max(abs(error[1:]))),
                "steering_rms_rad": float(np.sqrt(np.mean(steering[1:] ** 2))),
                "steering_total_variation_rad": float(np.sum(abs(np.diff(steering)))),
                "tracking_error_over_0p15_fraction": float(np.mean(abs(error[1:]) > 0.15)),
                "steering_abs_limit_fraction": float(np.mean(abs(steering[1:]) >=
                                                        limits["steering_abs_rad"] - 1e-9)),
            })
            for k in range(len(times)):
                rows.append({"plant": plant["name"], "controller": model,
                             "t_s": times[k], "target_yaw_radps": reference[k],
                             "plant_yaw_radps": states[k, 1],
                             "plant_vy_mps": states[k, 0], "steering_rad": steering[k]})
            axes[plant_index, 0].plot(times, states[:, 1], label=model)
            axes[plant_index, 1].plot(times, np.rad2deg(steering), label=model)
        axes[plant_index, 0].set_ylabel(f"{plant['name']}\nYaw (rad/s)")
        axes[plant_index, 1].set_ylabel("Road-wheel angle (deg)")
        for ax in axes[plant_index]:
            ax.grid(alpha=0.25)
            ax.legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("Time (s)")
    fig.suptitle("Synthetic closed-loop model comparison; not VW Golf control validation")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out.with_suffix(".png"), dpi=170)
    plt.close(fig)
    pd.DataFrame(rows).to_csv(args.out.with_suffix(".csv"), index=False)
    report = {
        "experiment": "P4-GOLF-CLOSED-LOOP-001 synthetic feasibility demo",
        "evaluation_kind": "synthetic_plant_only_not_real_golf_control_performance",
        "reference": f"{args.csv.resolve()} target yaw {args.start:g}-{args.end:g} s",
        "baseline": str(args.baseline.resolve()),
        "plant_config": str(args.plants.resolve()),
        "controller": "equal one-move receding-horizon grid search; 0.2 s prediction; full measured plant states",
        "forecast_inputs": "recorded future speed and bank, constant candidate steering over horizon",
        "sample_count": len(times),
        "prediction_horizon_steps": horizon,
        "prediction_horizon_nominal_s": horizon * 0.02,
        "limits_and_weights": limits,
        "tracking_threshold_radps": 0.15,
        "results": records,
    }
    args.out.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
