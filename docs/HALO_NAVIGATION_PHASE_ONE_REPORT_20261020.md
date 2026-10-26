# Phase One mapping diagnostics — October 20, 2026

The mapping diagnostics are implemented and deployed on Halo in the navigation
checkout, branch `navigation/mapping-and-exploration`. Phase One's full terrain
and localization acceptance gate remains open: the physical steep-ramp fixture
could not initialize valid localization, and the edge run exposed substantial
pose disagreement. These results do not qualify autonomous Martian driving.

The operator page adds terrain-cause colors, the padded footprint, one-second
motion sweep, frozen rejected sweep, highlighted blocking cell, depth age,
localization freshness and mapped area. Per-frame hazard causes survive memory
fusion and map corrections. Missing diagnostic evidence never changes the
guard's blocking decisions. See [operation and capture instructions](NAVIGATION_PHASE_ONE.md).

The audit now saves exact-stamp RGB/depth/calibration/terrain bundles, image-time
transforms, latest guard state and timing. Failed mapped captures preserve
separately labelled sensor-only evidence when possible. Truth comparisons run
only in independent evaluators. Safety limits, camera pitch, hazard thresholds,
two-frame retention and command/sensor/clock watchdogs retain their settings.

## Recorded results

Artifacts use UTC October 21 names; this report uses the operator's October 20
America/Chicago date. Every test used a new session or output prefix.

| Check | Observation | Interpretation |
| --- | --- | --- |
| Source and Halo build | Navigation built; **114 tests passed on Windows and 114 on Linux**: 111 navigation, one detector-service, two camera tests | Passed |
| Stationary flat, 10 wall seconds | Ten synchronized bundles; no retained hazards and no hazards in the final single depth frame | Passed for this bounded baseline |
| Flat browser manual workflow | Forward hold, two turns separated by idle, explicit Stop and simulated network loss; manual selection persisted between commands; disconnect latched `command_timeout` | Passed after warm-up |
| Actual Restart button | New stopped session, zero known cells and zero candidates before explicit launch confirmation | Passed |
| Physical edge fixture | Independent outward displacement **0.577 m**; stopped with `unknown_terrain`; final zero command; no observed chassis contact | Passed for conservative stopping and cell diagnostics; edge approach remains unqualified |
| Physical step fixture | Final depth frame: 702 hazard cells, including 91 height-span violations; 197 retained hazards in the 5 cm map | Measured step geometry remains blocked; no driving trial |
| Physical steep-ramp fixture | Repeated startup visual-odometry loss; no complete synchronized mapped capture; stopped throughout | Failed localization acceptance; partial camera/depth evidence retained |
| Stationary scene 50000, 12 wall seconds | Eleven synchronized bundles; final map 2,994 known cells and 147 hazards; depth-frame causes include slope and height change | Baseline recorded; hazard truth accuracy is not established |
| Scene-50000 browser workflow | Repeated manual commands, idle, Stop, network-loss stopping and diagnostic image/footprint display | Passed |

Two early browser runs interrupted their control lease during startup/warm-up.
They are not passes. The command HTTP handler was changed to read only cached
guard health; overlay computation and supervisor HTTP requests run on `/status`.
The later flat and Martian runs completed with the original lease limits.
Startup control stability still warrants a longer repeatability run.

The edge observer compared a stopped estimated base position of approximately
`(0.680, 0.379)` m with independent truth `(0.577, 0.000)` m, a **0.393 m**
disagreement. Receipts were 0.011 s apart. The run stopped before the physical
platform edge; it does not prove accurate edge localization or approach distance.
The recorded rejection identifies odom cell centre `(0.9375, 0.5875)` m, local
row 128/column 131, observation stamp 84.944 s. The UI displays that cell and the
frozen attempted motion after the later operator Stop.

In the short stationary Martian timing capture, global terrain arrived at
**5.06 Hz**, local ground at **5.07 Hz**, and RGB/depth at approximately **30 Hz**.
Maximum observed delivery gaps were 0.253 s for the global map and 0.283 s for
local ground. These are twelve-second observations with the existing GPU
detector active, not sustained timing or inference qualification.

Most final single-frame Martian hazards were away from the evaluator's nominal
rock radii. This alone does not identify false hazards: terrain slopes and
height changes also block navigation. Review the synchronized evidence before
changing any threshold. Localization freshness on the page measures timely
poses/transforms; it does not establish positional accuracy.

## Evidence

Session evidence remains ignored by Git and is retained on Halo and copied to
the local navigation checkout. It includes full sensor bundles and fixture
configuration, not only the summary links below. The curated export contains
199 evidence/configuration files; all 52 recorded source hashes match between
the Windows checkout and Halo.

- [Flat baseline](../evidence/restart-20261021T030108Z-f702be/checks/phase1-static-001.json)
- [Flat manual UI](../evidence/phase1-validation/browser-flat-drive-003.json)
- [Restart UI](../evidence/phase1-validation/browser-flat-restart-004.json)
- [Edge independent observer](../evidence/phase1-edge-20261021/checks/edge-stop-001.json)
- [Edge synchronized evidence](../evidence/phase1-edge-20261021/checks/phase1-edge-001.json)
- [Edge rejection screenshot](../evidence/phase1-validation/browser-edge-diagnostics-001.png)
- [Step evidence](../evidence/phase1-step-20261021/checks/phase1-step-001.json)
- [Steep-ramp failed capture](../evidence/phase1-slope-20261021/checks/phase1-slope-002.json)
- [Martian baseline and timing](../evidence/phase1-mars-20261021/checks/phase1-mars-001.json)
- [Martian manual UI](../evidence/phase1-validation/browser-mars-drive-001.json)
- [Martian screenshot](../evidence/phase1-validation/browser-mars-drive-001.png)
- [Final diagnostics view](../evidence/phase1-validation/browser-mars-final-002.png)
- [Linux build/tests](../evidence/phase1-validation/final-build-tests-002.log)
- [Independent baseline summary](../evidence/phase1-validation/baseline-summary.json)

Earlier reports, the operator's populated session
`restart-20261021T021913Z-0c3f61`, and its pre-change audit/status are preserved.
No old maps or session evidence were deleted. Arms, face/NPU work, target
tracking quality, exploration and autonomous waypoint acceptance were outside
this phase.

The rover is left stopped and ready in scene 50000, session
`phase1-mars-20261021`, with a populated diagnostic map. Refresh the existing
control page to see **Show terrain causes** and the mapping-health panel.

## Remaining gate

Use the captured evidence to resolve steep-fixture localization initialization
and the measured edge drift, then repeat those checks and a longer manual
startup/timing run. Keep slope, step and watchdog limits unchanged while
investigating. Phase Two and autonomous waypoint qualification have not started.
