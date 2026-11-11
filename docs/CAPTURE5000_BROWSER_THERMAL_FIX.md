# Browser assessment thermal stop and repair

[Presentation graphs, data and reproduction](figures/browser-thermal-20261108/README.md)
show the measured temperature checks, repair iterations and implemented changes.

The epoch-22 browser assessment stopped on November 8, 2026 at 18:57 local time
(November 9 at 00:57 UTC). The terrain supervisor directly observed the retained
90 C Tctl limit and terminated its worker. The scene supervisor subsequently
reported the missing terrain PID (`/proc/74081/stat`) and cleaned up the scene
and new detector. That missing-PID error was a consequence of the thermal stop,
not evidence that an exited GPU worker should be ignored or automatically restarted.

The terrain stop record is `environment-v2-terrain-final.json` in the case
`oem617-7141-rocm-manual-20261109t001920z-da40-20261109T001931Z-7vexv9n0`.
The scene record is `application-v9/supervisor-sessions/phasecandidate22-control-20261109T005419Z/result.json`.
Monitoring showed roughly 84–90 W package PPT during the loaded interval;
temperature rose from about 71 C to nearly 90 C over three minutes. The direct
guard observation caused the stop; the slower sampled logs do not capture every
temperature peak. Protected finish retained cooldown and journal capture and
recorded `MANUAL_SESSION_CLOSED_CLEANLY` at 01:00:32 UTC.

The separately pinned assessment v3 disables the redundant **software-rendered
RViz window**. Gazebo's graphical preview and qualified Ogre sensor rendering
remain enabled. The browser still supplies the camera, maps and controls.
This removes an unnecessary display workload; the retained logs alone cannot
attribute the entire thermal load to RViz, so a fresh attended measurement is
required before claiming that the thermal problem is resolved.

The wrapper checks the terrain's final record before native PID lookup and
reports its actual stop reason. It never retries a finished terrain attempt or
bypasses the 90 C guard. A failed scene supervisor is also recorded as a failure
in the parent assessment receipt. The previous bounded retry remains limited
to a vanished `loginctl` inventory entry.

All three Linux assessment tests passed, including rejection of wrong model or
runtime identities, preservation of camera/mapping/motion settings, and explicit
thermal/operator stop handling. The new release and installer preserve the old
scripts, configurations and release metadata in `assessment-source-v3/previous-release`.

The model, float64 ROCm terrain, HIP flags 4, 40 m maps, freshness
requirements, motion limits, hazard thresholds, power profile and fan policy
are unchanged. Restart and Generate new scene inherit `rviz=false` and the new
detector endpoint through the existing scene settings.

Run the same checked starter described in the
[model assessment guide](CAPTURE5000_MODEL_ASSESSMENT.md). A fresh monitoring case,
native NPU reference and candidate terrain service are required after a stopped
terrain attempt. Confirm clearance yourself before enabling manual drive.

The first reduced-display test retained about 84.6 W mean PPT without RViz.
Its observation helper deliberately stopped the identified assessment at a
sampled 87.125 C after 80 seconds of active mapping, before the native 90 C
limit. The operator had driven during this interval; that was not a detector
crash. Results are retained in `thermal-v3-observation-20261109T010648Z/`.
RTAB-Map process samples grew from 671 to 888 threads, while the Gazebo GUI
used roughly two CPU cores. Removing RViz alone did not resolve the load.

The separate `assessment-application-v4` retains the original image and
application files while adding container-local CPU affinity to eight distinct
physical cores, OpenMP workers 2, thread limit 4, one active level, passive
waiting, and BLAS/NumExpr workers 2. The original launchers' hashes were verified
unchanged after installation. A CPU-only check in the actual image verified
eight visible CPUs and eight default OpenCV workers, compared with 32 before.
An environment-only OpenCV thread hint did not affect this installed TBB build
and was not used as evidence of a working cap. OpenCV's implementation also
accounts for process affinity when determining available CPUs.
([OpenCV 4.6 source](https://github.com/opencv/opencv/blob/4.6.0/modules/core/src/parallel.cpp))

The idle terrain supervisor then encountered the same vanished-login-session
race at 01:12:37 UTC. An isolated assessment terrain supervisor now applies
the existing three-attempt whole-guard retry in both parent and worker. Its
original HIP-before-Torch setup and compute implementations are preserved.
Temperature and stale-monitor exceptions were tested to remain immediate stops.
The original terrain supervisor was not edited.

Use this terrain starter after fresh capture/temperatures and native NPU readiness:

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/capture5000-training-20261108/start_capture5000_terrain.sh
```

The browser assessment command is unchanged. After inspecting launch clearance,
minimize the separate Gazebo GUI while assessing through the browser; the sensor
server continues rendering through the qualified Ogre desktop path.

The eight-core test also remained near 84 W. RTAB-Map dropped to 113 threads,
but the verification helper stopped the assessment at 86 C after ten seconds
of observed active mapping. That deliberate stop preserved fresh camera and
localization samples; it did not establish a thermal fix.

A brief GLX probe in the actual pinned image confirmed accelerated Radeon 8060S
`radeonsi` rendering. The remaining v6 candidate adds an explicit navigation-only
`camera_rate_hz` argument, default 30, with a finite allowed range of 10–30 Hz.
This browser assessment selects **15 Hz RGB-D** while retaining 960 x 540 pixels,
camera mount/FOV/tilt, clipping, geometry, maps and safeguards. Capture configuration
and legacy launch defaults remain 30 Hz. Restart and Generate new scene retain
the chosen live cadence. Reduced cadence requires actual freshness checks;
no timeout or hazard rule was relaxed.

All 16 Linux launch, scene-control and assessment tests passed, including actual
URDF/session rate checks, rejection of invalid rates, unchanged source scenes,
30 Hz defaults and Restart/Generate propagation. The first v6 staging installer
matched a test filename too broadly; its assertion stopped installation before
graphics. Pre-install files were restored and verified, the exact-basename match
was corrected, and both attempts were preserved.

The 15 Hz live check passed on November 8 at approximately 19:36–19:45 local
time (November 9 at 01:36–01:45 UTC):

| Check | Measured result |
| --- | --- |
| Actual RGB-D sensor/session cadence | 15 Hz, verified in generated URDF and session metadata |
| Stationary active mapping | 300.56 seconds, 31 samples, peak 84.75 C |
| Stationary package PPT | Mean 70.60 W, versus roughly 84–90 W in the original failing profile |
| Stationary freshness | Camera and localization healthy in all 31 samples; 4,535 additional new-model frames |
| Forward/turn driving at 0.05 m/s | 120-second observation window, six nonzero-command samples, peak 86.125 C |
| Drive freshness | Camera and localization healthy in all 25 status samples; no verification failure |

The operator reported completion after the requested driving and Stop sequence.
The final readback showed zero commands with manual mode still armed. The
verification helper therefore applied Stop and confirmed `mode=stopped`, zero
commands, `reason=operator_stop` and no fault. It sent no clearance or drive commands.
The drive observation began before the operator moved; it is not a claim of
120 seconds of continuous driving. The final stopped state is recorded separately
in `THERMAL_REPAIR_LIVE_VERIFICATION.json`. Native temperature protection remains
90 C. Diagnostic observers used earlier stops at 85 C for stationary checks and
87 C for the movement check; neither threshold changed the native policy.

Both the GPU terrain service and selected NPU model remained ready. The sensor
server used the qualified Ogre path, HIP requested/observed flags remained 4,
and the new source/model pins were checked again after driving. All measurements
and prior failed attempts are retained in the training follow-up directory.

The successful observations, final receipt, release identities, renderer/CPU
probes and Linux test logs are collected in
`THERMAL_REPAIR_EVIDENCE_20261109.tar.gz`, with per-file SHA-256 hashes in
`THERMAL_REPAIR_EVIDENCE_MANIFEST.json`.

Keep Gazebo minimized when using the browser and retain visible monitoring.
The assessment starter has a 30-minute limit; expiry is an intentional shutdown,
not a new crash. Protected finish is still required after operator assessment.

This bounded regression does not close the separate 40 m environment's
latency/density qualification or establish endurance stability, higher-speed
acceptance under this cadence, or global model promotion.
