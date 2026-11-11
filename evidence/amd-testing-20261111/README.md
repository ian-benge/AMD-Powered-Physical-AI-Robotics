# AMD testing handoff evidence

This snapshot publishes selected source-check results and earlier recorded
measurements. It does not start a new Halo hardware trial or establish a completed
autonomous mission. Begin with the [testing guide](../../docs/AMD_TESTING_GUIDE.md).

The [manifest](manifest.json) records the original and published SHA-256 and byte
count for each selected record, plus the exact publication transformation.
Metrics retain their original dates. Host paths/identities are omitted where
documented; raw source evidence remains preserved. No failure was converted to a
pass, and no missing shutdown or review was reconstructed.

| Evidence | Outcome and scope |
|---|---|
| [Fresh source checks](source-checks/checks.json) | 805 tests with 16 hardware/OS skips across six Python suites; six JavaScript browser harnesses passed; no accelerator or movement acceptance implied |
| [Frozen detector comparison](model/MODEL_METRICS_SUMMARY_V2.json) | Same 100 legacy validation / 100 legacy test frames; epoch-22 NPU test mAP50–95 78.90%, baseline 76.67%; mudstone regression retained |
| [Freeze findings](incident/FINDINGS_SELECTED.json) | Unclean interruption during first autonomous return; initiating cause unknown; no successful home return or sample |
| [GUI-off stationary interval](thermal/STATIONARY_INTERVAL.json) | Five steady minutes: mean 80.82°C, peak 83.125°C; worst reported rolling publication p95 165.65 ms; below-80°C target failed |
| [Original single-input result](thermal/ORIGINAL_SINGLE_INPUT_RESULT.json) | Original logging-gap failure retained; overlapping observer reconciliation is documented separately |
| [Native health selection](thermal/NATIVE_HEALTH_SELECTED.json) | 58 health snapshots; mean PPT 70.43 W and minimum 49.78 GiB available memory; no matched cooling or energy claim |
| [GUI-off release identities](gui-off/RELEASE_IDENTITIES.json) | Source-v13 application plus corrected runtime-v15 host viewer; retained installed ROS build unchanged |
| [Initial tool/viewer tests](gui-off/gui-tools.log), [scene-control tests](gui-off/scene-control.log) | Eight and thirteen recorded Linux cases respectively; actual detached-child cleanup and profile propagation |
| [Final viewer tests](gui-off/VIEWER_FRONTEND_V15_TESTS.log) | Five corrected frontend lifecycle/command cases using system Python |
| [Actual ROS service probe](gui-off/actual-ros.log), [DDS transport fixture](gui-off/transport.json) | Real service/best-effort odometry and 90/90 RGB/depth messages near 15 Hz; synthetic transport fixture, not live sensor acceptance |
| [Protected manual closeout](gui-off/MANUAL_SESSION_FINAL.json) | Current stationary case closed cleanly; no endurance/autonomous claim; earlier freeze teardown remains missing |

The GUI-off Halo archive's [183-file off-host verification](archives/GUI_OFF_HALO_OFFHOST_VERIFICATION.json)
and [combined archive verification](archives/GUI_OFF_EVIDENCE_V1.verification.json)
identify the complete private copies. Full journals, process inventories,
boot/session identifiers, model binaries, datasets, runtime caches and large
archives are not redistributed here.

The [thermal figure and measured CSV](../../docs/AUTONOMOUS_DEMO_BROWSER_PROFILE.md#november-10-stationary-results)
are included separately in `docs/figures/`. Historical rendering/terrain/mapping
acceptance remains in the [November 7 evidence](../development-20261107/README.md).
The former [capture-depth repair evidence](../capture-depth-boundary-20261108/README.md)
is preserved unchanged.
