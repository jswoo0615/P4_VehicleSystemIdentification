# Local datasets

Raw vehicle logs are kept outside Git. The source recordings in this
workspace exceed 1 GB. See [DATASET_ATTRIBUTION.md](DATASET_ATTRIBUTION.md)
for the Stanford dataset citation and its verified CC BY-SA 4.0 license.
The repository contains code, frozen small JSON references, and selected
diagnostic figures. Do not commit the local `data/` directory.

For the VW Golf experiments, place the Stanford CSV bundle under:

```text
data/VehicleDynamicsDataset/Oct2023/*.csv
data/VehicleDynamicsDataset/Nov2023/*.csv
```

The scripts and C++ readers expect source headers including `t_s`,
`roadWheelAngle_rad`, `yawRate_radps`, `Vx_mps`, `Vy_mps`, and `bank_rad`.
The file-group CV removes byte-identical duplicates, leaving 20 unique
recordings from the 22 local October/November CSV files used for V1.
Use the original repository bundle without changing the CSV contents.

IONIQ 5 and older VBO experiments use other local folders under `data/`;
their commands and conversion notes are in `docs/ioniq5/IONIQ5_CONVERSION.md` and
`tools/README.md`. Nine C++ tests run without any raw vehicle logs. Five
additional VBO integration checks are registered when both
`data/old_vito_not_used/vblog_001.vbo` and `vblog_002.vbo` are present.
