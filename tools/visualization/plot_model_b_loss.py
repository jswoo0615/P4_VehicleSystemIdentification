"""Plot the Model B two-stiffness loss grid exported by the C++ CV program."""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cf", type=float, default=93941.306281)
    parser.add_argument("--cr", type=float, default=143718.324315)
    args = parser.parse_args()

    table = np.genfromtxt(args.input, delimiter=",", names=True)
    cf = np.unique(table["cf_N_per_rad"])
    cr = np.unique(table["cr_N_per_rad"])
    if len(table) != len(cf) * len(cr):
        raise ValueError("Incomplete rectangular loss grid")
    values = table["objective"].reshape(len(cf), len(cr))
    x, y = np.meshgrid(cf / 1000, cr / 1000, indexing="ij")
    minimum = float(np.min(values))
    relative = np.log10(values / minimum)

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5), constrained_layout=True)
    views = (
        (axes[0], np.minimum(relative, 1.5), np.linspace(0, 1.5, 31),
         "log10(objective / grid minimum)"),
        (axes[1], np.minimum(100 * (values / minimum - 1), 20),
         np.linspace(0, 20, 31), "Objective above minimum (%)"),
    )
    for ax, surface, levels, label in views:
        filled = ax.contourf(x, y, surface, levels=levels, cmap="viridis", extend="max")
        contours = ax.contour(x, y, values / minimum,
                              levels=[1.01, 1.05, 1.20, 2.0],
                              colors="white", linewidths=.9)
        if ax is axes[1]:
            ax.clabel(contours, fmt=lambda value: f"+{100*(value-1):.0f}%", fontsize=8)
        ax.scatter(args.cf / 1000, args.cr / 1000, marker="*", s=150,
                   color="red", edgecolors="white", label="Identified")
        ax.scatter(234.008, 234.008, marker="x", s=90,
                   color="magenta", linewidths=2.5, label="Supplied reference")
        ax.set(xlabel="Front stiffness Cf (kN/rad)",
               ylabel="Rear stiffness Cr (kN/rad)")
        if ax is axes[0]:
            ax.legend(loc="upper left", fontsize=8)
        fig.colorbar(filled, ax=ax, label=label)
    axes[0].set_title("Full search region")
    axes[1].set(xlim=(70, 120), ylim=(110, 180))
    axes[1].set_title("Near the identified minimum")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    plt.close(fig)
    print(f"{args.output} (grid minimum objective={minimum:.6f})")


if __name__ == "__main__":
    main()
