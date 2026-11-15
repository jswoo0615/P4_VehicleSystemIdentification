# Dataset Attribution

## Stanford VW Golf vehicle dynamics data

**Preferred citation:** Mori, D., Aggarwal, R. K., Broadbent, N. D.,
Kobayashi, T., and Gerdes, J. Christian (2025). *Vehicle Dynamics Dataset
for Highly Dynamic Automated Driving*. Version 1. Stanford Digital
Repository. [DOI: 10.25740/hh613qz0317](https://doi.org/10.25740/hh613qz0317).

**Source:** [Stanford Digital Repository record](https://purl.stanford.edu/hh613qz0317/version/1)

**Dataset license:** [Creative Commons Attribution-ShareAlike 4.0
International (CC BY-SA 4.0)](https://creativecommons.org/licenses/by-sa/4.0/).
The Stanford record identifies this license for the dataset. Its repository
use statement also notes that depositor-applied restrictions may apply; check
the source record before redistributing measurements or adaptations.

## Use in P4

P4 uses the VW Golf recordings for Model A, A-Speed, and B0 identification;
file-group five-fold validation; open-loop prediction and simulation
diagnostics; and controller-model export and runtime verification. The Golf
V1 experiments use 20 unique October/November 2023 driving CSV recordings
from Thunderhill Raceway Park. P4 excludes byte-identical duplicate files,
maps source channels, filters low-speed samples, and samples the file-group
experiments at 50 Hz. Individual experiments may use different sampling and
time windows; their procedures are documented separately.

The original measurements were created by the dataset authors, not by P4.
P4's fitted coefficients, predictions, validation summaries, and diagnostic
figures are derived results and must not be mistaken for source measurements.
The original CSV recordings are not distributed in this Git repository.

The dataset's CC BY-SA 4.0 terms and any applicable source-record conditions
govern use of the Stanford material. They are separate from permissions for
the P4 source code. Reusers should retain the citation and license link and
identify any changes to the original material.

P4 is an independent engineering project and is not affiliated with or
endorsed by Stanford University or the dataset authors.
