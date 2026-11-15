
from pathlib import Path
from collections import Counter
from scipy.io import loadmat

DATASET_DIR = Path(
    "data/IONIQ5_Data/data/dataset"
)

files = sorted(DATASET_DIR.glob("*.mat"))

print(f"MAT files found: {len(files)}")

channel_counts = Counter()
file_channels = {}

for path in files:
    try:
        mat = loadmat(
            path,
            variable_names=["data"],
        )

        data = mat["data"]

        fields = list(data.dtype.names or [])

        file_channels[path.name] = fields
        channel_counts.update(fields)

        print(f"\n=== {path.name} ===")
        print(f"Channels: {len(fields)}")

        for name in fields:
            print(f"  {name}")

    except Exception as e:
        print(f"\n[ERROR] {path.name}: {e}")

print("\n=== ALL CHANNELS ===")

for name, count in sorted(channel_counts.items()):
    print(
        f"{name:30s} "
        f"files={count}/{len(files)}"
    )

print("\n=== MODEL S CANDIDATES ===")

keywords = (
    "steer",
    "delta",
    "eps",
    "cmd",
    "command",
    "target",
    "ref",
    "angle",
    "torque",
)

candidates = [
    name for name in channel_counts
    if any(k in name.lower() for k in keywords)
]

for name in sorted(candidates):
    print(name)

if not candidates:
    print("No candidate steering channels found.")
