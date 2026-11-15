"""Shared reader for the VBO signal CSVs produced by the old VBO workflow."""

import csv

import numpy as np


CHANNELS = [
    ("steer_ang2", "Steering angle (deg)"),
    ("YawRate", "YawRate (raw)"),
    ("Yaw_Rate", "Yaw_Rate (raw)"),
    ("velocity", "Velocity (raw)"),
]


def read_signals(path):
    with open(path, newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"Empty signal file: {path}")
    return {
        key: np.array([
            float(row[key]) if row.get(key, "").strip() else np.nan
            for row in rows
        ])
        for key in ["time", *[name for name, _ in CHANNELS]]
    }
