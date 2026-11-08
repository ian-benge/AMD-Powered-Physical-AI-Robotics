# Published evidence

These are selected historical outputs from the Halo development runs. Measurements are retained as recorded. A private Tailscale address is replaced with a placeholder in four early smoke reports; [manifest.json](manifest.json) identifies those redactions and records both source and published SHA-256 hashes. Other report copies are unchanged. Screenshots in `docs/media` are original captures, not generated illustrations.

| Folder | Contents |
| --- | --- |
| [capture-depth-boundary-20261108](capture-depth-boundary-20261108/README.md) | V10 cutoff recovery: preserved-frame replay, 20-view rendered regression, deployment/preservation checks and clean protected closeout |
| [development-20261107](development-20261107/README.md) | November rendering, terrain, mapping, pilot/collection and monitoring-controller summaries; separate provenance manifest and fresh source checks |
| `phase3-validation-20261021` | Three-trial summary, timing, source-test logs and browser regression evidence |
| `restart-20261021T212626Z-06f471` | Accepted tracking trial 003 and fresh-revisit/replay reports |
| `restart-20261021T212834Z-6fa6de` | Accepted tracking trial 004 and fresh-revisit/replay reports |
| `restart-20261021T212947Z-150fe2` | Accepted tracking trial 005 and fresh-revisit/replay reports |
| `gpu-terrain-staging-20261021` | Integrated stationary measurement and profiled ROCm execution |
| `npu-resume-20261016` | Original replay, held-out accuracy, reference and compile reports |
| Other dated session folders | Selected earlier smoke, diagnostic and recovery outputs linked from reports |

The complete dataset, checkpoints, vendor software, large raw RGB-D bundles, session databases and runtime caches are not redistributed. Some historical reports reference these as local artifacts; such references are explicitly labeled when the file is omitted.

The files establish what the recorded tests observed. They do not establish physical-rover readiness, general semantic accuracy, full collision coverage, or acceptance of unfinished autonomous milestones. [Verification conditions](../docs/VERIFICATION.md) describe those boundaries.

[publication-source.json](publication-source.json) identifies the source snapshot and imported hashes. Hardware reports predate publication; the publication date is not a new hardware test date.
