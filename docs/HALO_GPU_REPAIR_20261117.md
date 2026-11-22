# Halo GPU repair investigation — November 17, 2026

The SDMA-off candidate passed bounded rendering, concurrent mapping and an
attended 0.05 m/s exploration/home return. A separate Nav2 progress-checker
correction resolved the reproduced return abort. Protected finish contains no
new journal incidents; the historical reset remains flagged. The off-host
archive and all 1,403 members are independently verified. No 0.10 m/s promotion, endurance or full sample-mission
acceptance is claimed. Earlier work and candidates remain preserved.

The operator has now requested the next tests. Fresh monitoring and the
separate [0.10 m/s qualification](AUTONOMOUS_010_QUALIFICATION_20261117.md)
are being prepared; this report retains the completed repair results above.

## Findings and uncertainties

The retained v22 archive independently reverified on Windows: all 31 member
hashes passed, and archive SHA-256 is
`c75580af574e44fb786649c809e5430422cba760bbd4cf6b7705a4309468c65a`.
Its protected kernel journal records SDMA timeout at 17:14:19.579625 UTC,
followed by reset/queue-removal failures and VRAM loss at 17:14:37.878572 UTC.
GNOME Shell and RDP report lost contexts afterward. This sequence supports
desktop failure as a consequence of the reset; it does not prove which client
or driver operation initiated the stall. Failed suspension of the terrain PID
is insufficient to assign cause.

No retained GPU coredump was found in the original case. The journal reports
devcoredump creation, but the preserved incident inputs contain text logs and
protected teardown rather than a dump that identifies the initiating submission.

| Hypothesis | Evidence and remaining test |
|---|---|
| ROCm copy-engine interaction, including simultaneous rendering | SDMA is the first recorded timed-out ring. Test a process-scoped copy policy while retaining GPU computation. A timeout alone does not identify the submitting client. |
| Rendering or Mesa/kernel/firmware defect | Rendering shared the GPU with ROCm and the desktop. Start with bounded rendering-only operation, then simultaneous production mapping. |
| Remote desktop interaction | Old desktop/RDP contexts failed after reset. Recovered Remote Login has a different service ownership model; verify it and the graphical session before rendering. No causal RDP claim. |
| Thermal, power or hardware fault | Sampled moving peak was 84.375°C, below the 87°C qualification stop. No thermal-cause proof. Retain power/fan policy and temperature guards. |
| November 10 hard freeze | Remains a distinct unresolved incident; this reset does not establish its cause. |

The current boot remains `b1c937a8-3133-4efe-90bb-648b1282b4a3`, with kernel
`6.17.0-1033-oem` and AMDGPU source version `6295BB68074049E9D137840`.
BIOS is EVO-X2 1.12, dated 12/09/2025 as reported by DMI.
Mesa DRI is `25.2.8-0ubuntu0.24.04.4`, libdrm-amdgpu
`2.4.125-1ubuntu0.1~24.04.2`, linux-firmware
`20240318.git3b128b60.0ubuntu3.1`, and amd64-microcode
`3.20251202.1ubuntu0.24.04.1`. Loaded firmware reports match the incident
preflight byte for byte, including SDMA0 `0x00000012`, MES `0x00000086`,
MES_KIQ `0x0000006f` and VBIOS `113-STRXLGEN-001`.
ROCm remains 7.14.1 / Torch 2.12.0+rocm7.14.1. No driver, kernel, firmware,
power or fan setting was changed; no reboot occurred.

## Implemented candidate

`application-v24` / `runtime-v24` retain v22's installed ROS bytes, three frozen
rare scenes, ten-Hz 960×540 RGB-D, GUI/RViz-off policy and pose-chain timestamp
repair. The separately pinned `terrain-sdma-off-v2` changes only the terrain
worker's `HSA_ENABLE_SDMA=0` environment and records that policy in startup and
HTTP metadata. GPU float64 computation and HIP blocking-sync flags 4 remain.
Other HSA/HIP/ROCR overrides are rejected. The new runtime starter requires the
explicit live SDMA-off ROCm service, preventing accidental use of the old worker.

V23 and `terrain-sdma-off-v1` remain preserved. The first terrain launch stopped
before HIP import because moving the retained wrapper exposed a parent-relative
bulk-helper import. V24 / `terrain-sdma-off-v2` use the original helper's absolute
path, passed an actual no-GPU import check and repeated the source/ROS checks.
The GPU copy policy is unchanged between these attempts. Actual v2 worker PID
356027 reported ROCm float64, `HSA_ENABLE_SDMA=0` in `/proc` and HIP flags 4
before and after warmup. Current-case CPU/ROCm classify, depth and projection
parity passed with exact costs/reasons. This verifies configured process policy;
effective SDMA engine activity has not been measured.

V25 adds stop-time deadline diagnostics. V26 retains those installed code bytes
and changes only the installed navigation configuration to use
`nav2_controller::PoseProgressChecker`. It counts measured rotation above
0.5 radians as progress, retaining the 0.10 m translation threshold and
20-second no-motion timeout. Goal tolerances and mission time budgets are
unchanged. This corrects the specific translation-only progress failure observed
during V25's return; it is separate from the GPU copy-policy experiment.

AMD documents this variable as a way to isolate copy-engine problems: it requests
compute blit copies in the ROCr process. It does not disable all SDMA use by the
renderer, desktop or kernel, and environment verification alone does not measure
effective engine activity. See [AMD's HIP debugging documentation](https://rocmdocs.amd.com/projects/HIP/en/latest/how-to/debugging.html)
and [ROCm 7.14 environment variables](https://rocm.docs.amd.com/en/docs-7.14.0/reference/environment-variables/).

| Artifact | SHA-256 |
|---|---|
| V23 application configuration | `16daa5e31823807b9d516b2d820c61d862bb0ad105e5dd3b66549200c4222be7` |
| V23 runtime release | `cad6090a286ea5295aa5f58a7e25f096a7c957a139e9d6d7b3f5fa2fc97788ad` |
| Terrain SDMA-off candidate | `c7ceac3418b61d571dacf365c08808c99bf2c245a4d2c20d78cbab95d4786c58` |
| Rendering-only input manifest | `c4dbd74d3aa8a42063d5e23a2c842d20f61e9ea72ca162b828b0346e46f3d5ad` |
| V24 application configuration | `c0ba19746c3a72937314659b06ac772639172159ad18f566b39d5256c93e444f` |
| V24 runtime release | `6475622dc6fb268181b7ebe215af050d31d91886e09a3428cd6b6b533f1ac6c8` |
| Terrain SDMA-off v2 candidate | `50e13cdb7d7c1c8a2b3ee1a6975a515813b8a9b58e48e0db15cf33bebfeb22fa` |
| V25 diagnostic application configuration | `85f4e9e16a3c1a5f32aa31877d6c73ce344cc06d5f0d1dbc55a986f8f6339c9e` |
| V25 diagnostic runtime release | `4a20d388c6e0378235d11d8edd1c9b3fa85c395262bf0ac5361f532eb4690d84` |
| V26 application configuration | `2dd79d3a13871baf745c42587f4da9377d3e8d8e2d25c61837095fffa62505de` |
| V26 runtime release | `476620f66bb5288432a7d6bb1b9d89f891308e6c92805f6a77b83ba02b2bbdbf` |

All identities are relative to Halo's retained demo base:
`/home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109`.
The terrain and rendering fixtures are under `gpu-repair-20261117/`.

## Checks and monitoring

Both V23 and V24 passed 262 navigation tests (four GPU-only skips), actual ROS/HTTP and
best-effort odometry reception, real tf2 interpolation, all three launch graphs,
and 60/60 RGB and depth transport frames at approximately 10 Hz (largest gap
101.12 ms). The copied installed/source pins and v22 rollback were verified.
Recovery, environment rejection and Remote Login identity checks have targeted
CPU regressions. These checks are separate from actual GPU qualification.

The first recovery helper required a stopped record, but the compositor crash
had left a stale `running` record. The revised review preserves that record
and checks that no native capture process remains. A later native preflight
correctly refused the historical incident anywhere in the current boot.
The separate reviewed-history helper accepts only the exact root-protected
incident records; missing or new incidents remain fatal. Full journals and all
failed attempts remain available. Live incident detection and native protected
finish are unchanged. Consequently native finish still reports this boot's
historical incident; it must not be presented as globally clean.

System Remote Login runs as `gnome-remote-desktop`, rather than the old user-owned
sharing daemon. A new pinned kit verifies its systemd MainPID, service user,
process UID, exact daemon arguments and start time, while preserving the existing
graphical-session selection and per-case identity checks. Read-only validation
passed against PID 336889; no RDP settings or credentials were changed.
The recovered display was re-detected as `:0`, 1920×1080, with current
Xauthority `/run/user/1001/.mutter-Xwaylandauth.VUOVY3`; this is an observation,
not a value to hard-code for future sessions.

Current fresh monitoring case:
`oem617-7141-rocm-manual-20261117t191727z-40a2-20261117T191727Z-0zntc8pv`.
The operator confirmed capture/temperatures and attends the desktop.
GET-only off-host logging fsyncs each received chunk and sends no heartbeat or
motion controls. The original incident's native monitoring gap remains recorded.
The new off-host SSH channel timed out after 19:59:08 UTC; collection resumed
at 20:07:20 UTC, leaving a documented **492.41-second off-host gap**. Native
health records continued across that interval (maximum observed interval
5.42 seconds), the boot was unchanged and no simulator was running when SSH
returned. This does not establish the cause of the connection interruption or
continuous operator desktop connectivity. The replacement observer remains
read-only; the original logs are retained.

## Hardware test sequence and current status

The first test used the separate `render-only-v1` camera fixture at ten Hz for
300 seconds, with no ROS bridge, detector, mapper or rover controller. It retains
the 87°C qualification stop, native 90°C protection and fixed GPU image/plugin.
It uses simpler evaluator geometry than the production rare scene; comparison
with production mapping cannot isolate scene effects causally.

The rendering-only test completed 19:19–19:24 UTC: **3,002 RGB frames and
3,002 depth frames**, 3,001 matched pairs, 300 saved synchronized frames,
zero uniform RGB/empty depth frames, and no capture or cleanup error.
Across 395 guard samples, mean Tctl was **32.38°C**, peak **33.625°C**.
Actual Ogre logs identify Radeon 8060S/radeonsi, Mesa 25.2.8 and the pinned
plugin SHA `f6e8842247198a2b34b25cfdb40a8f4bc4f156c2e90a54135a68698dc239a3e6`.
No new native incident or ROCm client was observed during this fixture.
This supplies a bounded rendering stability result; original fixture geometry
acceptance remains its separate `REVIEW_REQUIRED` result.

V24 production scene `restart-20261117T192850Z-847607` passed the five-minute
concurrent rendering/ROCm mapping check, 19:30:36–19:35:36 UTC. All 61 samples
showed fresh navigation inputs, zero motion and the stopped guard. Mean Tctl
was **78.96°C**, peak **80.75°C**, worst observed rolling publication p95
**121.02 ms**, median reported RTF **0.99798** and minimum **0.99711**.
The raw stationary log SHA-256 is
`25f8f4c9bc951e9cdce1dfd9482496cef2e163bef9650e78bfa8e274b16b13ec`.

The operator then started 0.05 m/s exploration at 19:36:16 UTC. After 5.76
seconds of initial rotation, the application stopped safely with “Unhealthy
navigation or operator heartbeat lost.” There was no new GPU incident. Nearby
status samples showed healthy inputs, but do not resolve a transient at the
precise stop time. The operator reports the dashboard remained visible with
no earlier browser error. Neither a heartbeat cause nor a GPU cause is proven.
The scene was closed; no home arrival or movement qualification was recorded.

V25 is a separate pinned diagnostic sibling. It records heartbeat age and
sequence, navigation health and each input age at a deadline stop, without
logging owner tokens. Its one-second heartbeat limit and all freshness, Stop,
motion and thermal rules remain unchanged. The only installed code delta
from V24 is this stop-time diagnostic. All 265 navigation tests passed (four
GPU-only skips), including regressions for heartbeat expiry, transient sensor
loss and healthy continuation; ROS/tf2/transport and all three launch checks
also passed. Fresh scene `restart-20261117T202038Z-8dbcab` passed ten-second
exterior inspection with clean cleanup, followed by an eleven-sample stopped
check at 20:24:55–20:25:25 UTC: all inputs healthy, peak Tctl **80.25°C**,
maximum rolling publication p95 **119.21 ms**, minimum RTF **0.99651**.
This short check verifies the diagnostics-only sibling; V24 supplies the
separate five-minute production hardware prerequisite. The operator started
the targeted sweep at approximately 20:26:06 UTC, entered exploration after
26.19 seconds and requested early return after 79.59 seconds. The rover stopped
at 99.79 seconds with **Nav2 error 105, FAILED_TO_MAKE_PROGRESS**, after about
2.55 m estimated travel. No heartbeat/navigation deadline event, movement guard
fault or verified home arrival was recorded. Across 33 active watch samples,
all navigation inputs were healthy, peak Tctl was **83.125°C**, maximum rolling
publication p95 **122.72 ms** and minimum RTF **0.99564**. No new GPU incident
flag appeared. The V24 transient deadline stop remains unexplained.

The controller log confirms cancellation completed and the new home goal was
accepted. The return then rotated about **1.33 radians** with only **0.094 m**
net translation before the 20-second timeout. The configured
`SimpleProgressChecker` counts translation only. The installed Nav2 action
definition and controller error identify the abort; the retained startup
odometry latch was cleared by explicit arming and no movement fault occurred.
Nav2's [Jazzy SimpleProgressChecker source](https://api.nav2.org/nav2-jazzy/html/simple__progress__checker_8cpp_source.html)
and [PoseProgressChecker source](https://api.nav2.org/nav2-jazzy/html/pose__progress__checker_8cpp_source.html)
document the translation-only and translation-or-rotation checks respectively.

V26 passed the 265 navigation tests (four GPU-only skips), ROS/tf2/transport,
three launch checks and an actual installed C++ plugin replay of the 101
recorded return poses. The replay reproduced the old timeout and accepted the
measured turn with the new checker. It held the final recorded pose for 0.25 s
to cover the abort instant just after the last trajectory sample. Both plugins
still rejected stationary and small-noise stalls at 20.001 s; translation
resets and rotation across yaw wrap were checked. This is CPU plugin evidence,
not a live home-return result by itself.

V26 scene `restart-20261117T203636Z-3f5b76` passed ten-second attended exterior
inspection and a stopped prerequisite. Live ROS parameter queries verified the
actual controller's PoseProgressChecker and exact 0.10 m / 0.50 rad / 20 s
settings. The operator started exploration at approximately 20:42:12 UTC and
requested early return after about 85 seconds. The rover completed the turn
that previously timed out and recorded **verified home arrival after 198.81 s**,
about **5.74 m** estimated travel: **0.06207 m position error**, **0.04268 rad
(2.45°) heading error**, zero command and no guard fault. The operator confirmed
completion. No manual rescue or rearm occurred. No sampling trip was attempted;
the post-home target-selection timeout remained active while the scene closed.

All **273 higher-frequency supervisor observations** during movement had
healthy navigation inputs and no guard fault. Mean sampled Tctl was
**83.31°C**, peak **85.125°C**, maximum rolling publication p95 **122.60 ms**
and minimum reported RTF **0.99396**. The independent three-second watch also
passed (66 active samples); its lower sampled peak of 84.75°C reflects its
sparser cadence. Neither log establishes continuous measurement of temperature
peaks. All retained thermal protections remained active. The movement-watch
SHA-256 is `3249ce7b0d5cecc973810599f018be0e4c6232507f760cda75824c57fe2e872f`.

The scene, terrain worker and NPU stopped, and native idle checks passed.
Protected finish at **20:50:24 UTC** retained `post_teardown_clean=false` and
`INCIDENT_REVIEW_REQUIRED`. Independently parsing its complete protected kernel
journal with the broader recovery matcher found exactly the **14** timestamp
and message pairs from the root-reviewed incident, with no new or missing
entries. This is scoped evidence of no additional incident, not a globally
clean boot or proof that SDMA copy policy fixed the initiating cause.
Capture closed at **20:53:37 UTC**, after protected finish; both capture and
temperature processes exited. The off-host log independently received the
`operator_capture_stop` event before its own identity-checked shutdown.
`PROTECTED_TEARDOWN_SCOPED_REVIEW_FINAL.json` passed. The earlier review remains
preserved as incomplete because capture was still running at that point.

After verifying all source/runtime pins and closed evidence, the separate
`runtime-v26/MOVEMENT_005_QUALIFIED.json` receipt was issued for the bounded
representative-1 exploration/early-return condition. Its SHA-256 is
`a86faabafe1b509a0916d5bc0331359fc65f130a0c0dcb7bfc3659cf1b260f20`.
This makes a separate 0.10 m/s qualification test eligible; it does not qualify
0.10 m/s, other scenes, endurance or the full sampling mission. The immutable
configuration's pre-test status is supplemented by this evidence-bound receipt.
The downloaded archive independently verified on Windows with .NET ZIP streams
and SHA-256 for every regular member; duplicate names, missing members and
manifest count mismatches are rejected. All **1,403 files** passed:

- Archive: `HALO_GPU_REPAIR_20261117T205952Z.zip` (**211,464,017 bytes**).
- SHA-256: `9d8a162cf8cb9e45b24a8858be52f62b03fbde5536885486274e16386cd96120`.
- Local directory: `navigation_sessions/autonomous-demo-20261109/gpu-repair-20261117/verified-archive/`.
- Independent receipt: `HALO_GPU_REPAIR_OFFHOST_VERIFICATION.json`.
- Final combined result: `gpu-repair-20261117/GPU_REPAIR_FINAL_RESULT.json`.

The bundle includes closed native case/kit evidence, preserved failed recovery
attempts, V23–V26 runtime and test/session evidence, rendering observations,
off-host logs, the original V22 archive, and a hash-verified 46-file snapshot of
the uncommitted working source. It explicitly lists excluded retained
localization databases and caches. It contains the pre-verification result
snapshot; the independent external receipt/final result records completion of
the download verification. Earlier evidence, incident flags and uncommitted
work remain preserved. No commit, push, driver/kernel/firmware, power/fan or
reboot change occurred.

Launch V26 from Halo's attended desktop:

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v26/start_autonomous_demo.sh 1 rehearsal 0.05
```

After the camera is ready, inspect from another terminal using the full path
(shell variables set in the starter terminal do not carry into another terminal):

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v26/view_autonomous_demo.sh 10
```

After inspection closes, explicitly confirm launch clearance in the browser
and keep the rover stopped until prerequisite checks are reviewed. Use the
browser for explicit Start and Return commands; observers send no heartbeat.

For another hardware session, use a fresh reviewed monitoring kit, qualify its
NPU and SDMA-off terrain worker, and re-detect the connected desktop. Preserve
the 87°C/90°C protections and explicit inspection, clearance, Start and heartbeat
rules. Keep monitoring through workload stop, protected finish and cooldown.
The next progression is a fresh attended 0.10 m/s qualification using the issued
exact-configuration 0.05 m/s receipt, on representative 1 first. Longer
attended operation, the remaining scenes and the full sampling mission remain
separate acceptance work. The original GPU initiating cause, V24's transient
deadline stop and November 10 hard freeze remain unresolved.
