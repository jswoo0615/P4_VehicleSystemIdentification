
import argparse
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt

if not __package__:  # Also support direct script execution.
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.datasets.vbo.signals import CHANNELS, read_signals


def read_maneuvers(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def plot_signals(data, maneuvers, title, output, xlim=None):
    fig, axes = plt.subplots(
        4, 1,
        figsize=(13, 10),
        sharex=True,
        constrained_layout=True
    )

    time = data["time"]

    for ax, (channel, ylabel) in zip(axes, CHANNELS):
        values = data[channel]

        ax.plot(
            time,
            values,
            linewidth=1.2,
            label=channel
        )

        for event in maneuvers:
            start = float(event["start"])
            end = float(event["end"])

            ax.axvspan(
                start, end,
                alpha=0.15
            )

        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

    if xlim is not None:
        axes[-1].set_xlim(*xlim)

    axes[-1].set_xlabel("Time from log start (s)")
    fig.suptitle(title, fontsize=14)

    fig.savefig(output, dpi=160)
    plt.close(fig)

    print(f"[SAVED] {output}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix")
    parser.add_argument("--pad", type=float, default=0.7)

    args = parser.parse_args()

    prefix = Path(args.prefix)

    signals = read_signals(
        str(prefix) + "_signals.csv"
    )

    maneuvers = read_maneuvers(
        str(prefix) + "_maneuvers.csv"
    )

    # 전체 24초 기록
    plot_signals(
        signals,
        maneuvers,
        f"{prefix.name} - Full Run",
        str(prefix) + "_full.png"
    )

    # Maneuver 개별 확대
    for event in maneuvers:
        event_id = event["id"]

        start = float(event["start"])
        end = float(event["end"])

        lower = max(
            float(signals["time"][0]),
            start - args.pad
        )
        upper = min(
            float(signals["time"][-1]),
            end + args.pad
        )

        plot_signals(
            signals,
            [event],
            f"{prefix.name} - Maneuver {event_id}",
            str(prefix) + f"_maneuver_{event_id}.png",
            xlim=(lower, upper)
        )

    print(f"Detected candidates: {len(maneuvers)}")


if __name__ == "__main__":
    main()
