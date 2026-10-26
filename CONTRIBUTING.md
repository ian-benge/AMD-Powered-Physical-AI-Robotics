# Contributing

Use a feature branch and keep behavior changes separate from benchmark claims. Run `python scripts/run_checks.py` before opening a pull request. Changes to ROS interfaces or package dependencies also need the Docker ROS build.

Motion changes require bounded simulator acceptance in addition to unit tests. Preserve sole guard ownership of `/cmd_vel`, existing watchdogs, full-footprint checks and unknown-space blocking. Do not use simulator truth as a navigation input or relax thresholds to obtain a pass.

Record benchmark input/model/runtime identities and state what timing includes. Keep calibration data separate from held-out evaluation; retain failed experiments. Hardware-only checks must report skips rather than masquerading as CPU validation.

Do not commit generated datasets, weights, vendor binaries, caches, credentials or personal camera recordings. Add only deliberately selected evidence with provenance. Preserve contributor and third-party notices.
