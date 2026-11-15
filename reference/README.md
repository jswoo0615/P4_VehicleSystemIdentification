# Tracked reference artifacts

`v1/` contains byte-for-byte copies of the frozen Golf A/A-Speed/B0
validation, controller export, parity, and integration JSON reports. The
original command outputs live in ignored `output/` during local work.
The SHA-256 values for the V1 artifacts are recorded in
`docs/golf/P4_GOLF_V1_STATUS.md` in the repository root.

To restore the frozen inputs expected by export and parity commands after
cloning the repository:

```sh
mkdir -p output
cp reference/v1/*.json output/
```

`diagnostics/` contains the finite-horizon file CV and synthetic closed-loop
summary. Its synthetic results are a harness demonstration, not VW Golf
closed-loop validation. The matching selected plots are in `figures/`.
For diagnostics based on the Stanford VW Golf recordings, cite the source
dataset as described in [DATASET_ATTRIBUTION.md](../DATASET_ATTRIBUTION.md);
the plotted model outputs are P4-derived results.
