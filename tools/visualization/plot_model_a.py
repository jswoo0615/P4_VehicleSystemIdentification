
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def plot_prediction(csv_path: Path, output_dir: Path):
    df = pd.read_csv(csv_path)

    required = {
        "time_s",
        "steer",
        "yaw_measured",
        "yaw_predicted",
        "error",
    }

    if not required.issubset(df.columns):
        raise ValueError(
            f"Missing columns in {csv_path.name}: "
            f"{required - set(df.columns)}"
        )

    t = df["time_s"]
    steer = df["steer"]
    measured = df["yaw_measured"]
    predicted = df["yaw_predicted"]
    error = df["error"]

    rmse = (error.pow(2).mean()) ** 0.5
    mae = error.abs().mean()
    bias = error.mean()

    yaw_range = measured.max() - measured.min()

    nrmse = (
        100.0 * rmse / yaw_range
        if yaw_range > 1e-12
        else float("nan")
    )

    fig, axes = plt.subplots(
        3, 1,
        figsize=(12, 9),
        sharex=True,
    )

    axes[0].plot(t, steer, linewidth=1.3)
    axes[0].set_ylabel("Steering (raw channel unit)")
    axes[0].set_title(csv_path.stem)
    axes[0].grid(True)

    axes[1].plot(
        t, measured,
        label="Measured",
        linewidth=1.6,
    )
    axes[1].plot(
        t, predicted,
        label="Predicted",
        linewidth=1.4,
        linestyle="--",
    )
    axes[1].set_ylabel("Yaw rate (deg/s)")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(t, error, linewidth=1.3)
    axes[2].axhline(0, linestyle="--", linewidth=1)
    axes[2].set_ylabel("Error (deg/s)")
    axes[2].set_xlabel("Time from event start (s)")
    axes[2].grid(True)

    fig.suptitle(
        f"RMSE={rmse:.3f} deg/s | "
        f"MAE={mae:.3f} deg/s | "
        f"Bias={bias:+.3f} deg/s | "
        f"Range NRMSE={nrmse:.1f}%",
        fontsize=11,
    )

    fig.tight_layout()

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path = output_dir / (
        csv_path.stem + ".png"
    )

    fig.savefig(output_path, dpi=160)
    plt.close(fig)

    print(
        f"{csv_path.name:28s} "
        f"RMSE={rmse:7.3f} "
        f"Bias={bias:+8.3f} "
        f"NRMSE={nrmse:7.2f}%"
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("output/predictions"),
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output/plots"),
    )

    args = parser.parse_args()

    csv_files = sorted(
        args.input_dir.glob("*.csv")
    )

    if not csv_files:
        raise RuntimeError(
            "No prediction CSV files found"
        )

    for csv_path in csv_files:
        plot_prediction(
            csv_path,
            args.output_dir,
        )


if __name__ == "__main__":
    main()
