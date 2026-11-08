# Phase 1 rendering implementation and acceptance

November 4, 2026 (UTC validation continued November 5). **Status: rendering accepted;
all three visibility trials and final protected/live review complete.**
Use the [operator guide](PHASE1_RENDERING_GUIDE.md) for exact baseline, candidate
and rollback commands. Phase 2 was subsequently authorized separately.

## Inventory and retained baseline

The Windows checkout is on `main` with substantial existing tracked and
untracked work. Its initial status, diff and Phase 1 sources were saved before
editing under `navigation_sessions/rendering-phase1-resume-20261104/`.
No reset, clean, stash, commit or push was performed.

Halo's `/home/ian/capstone-training/rover-navigation` is a source/install mirror
without `.git`; no branch identity can be inferred there. The retained launcher's
exact file pins and actual installed files were inspected instead. The local
source was not assumed to be deployed. The separate perception checkout,
models, scenes, datasets and baseline install remain in place.

The initial active manual session was stopped after the operator confirmed a
stopped rover. Its `MANUAL_SESSION_FINAL.json` reports clean protected teardown.
A fresh baseline session then passed startup, logging, temperature/fan,
NPU qualification, float64 CPU/GPU parity, HIP flag-4 readback and actual
Radeon/Mesa renderer/library review. The operator reported driving/turning at
the requested 0.05 m/s / <=0.30 rad/s check settings and stopping. The saved
post-check state was ready, stopped with `operator_stop`, command `[0,0]` and no
fault. This is readiness evidence, not an endurance pass.

| Identity | Value |
| --- | --- |
| Baseline image | `sha256:b976b898cc96ad898336310fcd630e2151af527656e549ed24b5f0967a8fc88b` |
| Final candidate image | `sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa` |
| Final candidate tag | `rover-navigation-phase1-resume-20261104-v4` |
| Baseline / candidate Ogre library SHA-256 | `ab53815ad7857af955710ec62da0a065f6d2ea9712767da9b6fcfa31ae02b360` / `f6e8842247198a2b34b25cfdb40a8f4bc4f156c2e90a54135a68698dc239a3e6` |
| Pinned upstream archive | gz-rendering 8.2.3, `5bfe18d164414e098b2ba6ef81bf9391f7ff8f2c39b8a4fa9f8543f778e921c8` |
| Source archive v6 SHA-256 | `97c988fa426db5ba0d7e5cc41f77defb7b760a4f59c592e2b543cf4b3be8f9be` |
| Final motion-tool source archive v8 SHA-256 | `8ed90cd814d0e0a5ef1553e098c269ef212aab7604f53dc373d5b98360e72778` |
| Final evaluator source archive v9 SHA-256 | `7153300a8d53e76c08dc9c76adacdb140ff674ad9fe80ff07cb89e388a4d4623` |
| Measured-pose recorder source archive v10 SHA-256 | `8bb01b0589237f77c50051d8759e6ef5d2a5030d162d8efa6908987925552977` |
| Candidate source/install/scene configuration SHA-256 | `a91ca3d7d94fca9cb9879affd637114c68704ead38d70ddfc439e55738836d1a` |
| Source built into application install | Application v5; runtime package sources match v6 and v8; later changes are evaluator/management/build-probe files |
| NPU model SHA-256 | `2d47f7591563869cc7cbd6f118342d82db3427dbee8e8eff5c5b3b0589a8b03a` |
| Kernel / driver | `6.17.0-1033-oem`, inbox amdgpu srcversion `6295BB68074049E9D137840` |
| Compute | Isolated terrain ROCm 7.14.1 / Torch 2.12, float64, requested/observed HIP flags 4 |

Docker BuildKit interprets a bare `sha256:ID` in `FROM` as a registry tag. The
build wrapper therefore creates a separate local baseline alias and verifies
its immutable image ID before and after the build. It never overwrites the
baseline tag or changes stored baseline source hashes. Both the failed initial
BuildKit attempt and subsequent guarded build records are retained.

The installed-file comparison found eight changed runtime files: camera/world
launches, annotation, camera description/configuration, world renderer selection,
restart image/range preservation, and an existing UI character-encoding fix.
Motion, hazard thresholds, localization and depth-supported positioning modules
match the baseline install. Quiet/Automatic fans, Linux power-saver, runtime
versions and the attended 90 C stop rule were retained.

## Implemented behavior

The pinned Ogre patch inspects decoded alpha instead of treating the presence
of a PBR alpha flag/channel as transparency. Opaque RGB and fully opaque RGBA
textures retain depth checking/writing. Genuine transparency keeps blending
and depth checking, with depth writing disabled. Texture replacement, clearing,
in-memory images and uniform material transparency are covered by the probe.
Alpha operations are reset on transitions. Unknown/unsupported decoded-color
access conservatively retains the original alpha path.

The explicit candidate profile uses **20 m axial RGB far clipping**, **8 m
usable depth**, near **0.30 m**, 960x540 and 0.40 rad downward tilt with Ogre.
Historical camera defaults remain unchanged. Future capture worlds can select
the same explicit engine/profile. RGB-only distant boxes still cannot create
positioned map objects without valid depth. Existing capture quality checks
continue to flag geometry labels without measured depth support.

Ogre's pinned RGB-D implementation renders a nearest surface below 0.30 m, then
substitutes invalid depth/background RGB. It still hides farther surfaces.
The new explicit `near_clip_mode=occlude` represents this measured behavior
instead of inventing farther-object/ground labels through that surface. Old
profiles retain their original clipping mode.

Guarded fixture recording saves exact-stamp RGB/depth, observed camera poses,
published calibration, per-frame health, actions, renderer mappings, source/image
and configuration identities. Stalled/invalid output and interruptions remain
failed/incomplete recordings. The original capture/device-dump watcher is
required throughout. The fixture has no ROS bridge or navigation controller;
geometry truth is evaluator-only.

## Separate controlled comparisons

Final-image evidence is in `qualification-v6/baseline8`, `candidate8` and
`candidate20` under the Halo evidence root below. All use identical geometry,
lighting, lens, near/depth limits and measured stationary camera poses.

| Check | Baseline 8/8 m | Candidate 8/8 m | Candidate 20/8 m |
| --- | --- | --- | --- |
| Opaque file/memory texture contract | Expected failure | Pass | Same backend |
| Partly buried rock in RGB | Incorrectly overdrawn by ground | Visible at predicted mask | Visible |
| Distant range targets | Clipped | Clipped | Rendered, with invalid depth beyond 8 m |
| Expected usable depth missing | 0 | 0 | 0 |
| Unexpected usable depth fraction | 0 | 0 | 0 |
| Sampled p95 depth error | 0.0277 mm | 0.0277 mm | 0.0401 mm |
| Below-near mask matches background | 100% | 100% | 100% |
| Translucent sky ROI intermediate-alpha fraction | 100% | 100% | 100% |

Saved views: baseline 8 m (`navigation_sessions/rendering-phase1-resume-20261104/final-baseline8.png`, local-only),
candidate 8 m (`navigation_sessions/rendering-phase1-resume-20261104/final-candidate8.png`, local-only),
candidate 20 m (`navigation_sessions/rendering-phase1-resume-20261104/final-candidate20.png`, local-only).
The partly buried yellow rock is absent in the baseline view and present in
both candidate views. The lower-left background patch is the explained
below-near occluder; it remains invalid in all three configurations.

This is image-level reproduction of one disappearing-object mechanism. It does
not establish that every previously reported rock had the same cause. Genuine
terrain/rock occlusion and the previous 8 m range limit explain other visibility
changes. Immediate detector boxes and delayed confirmed-map landmarks remain
separate outputs; detector classification accuracy was not changed or qualified.

The published intrinsics are fx/fy approximately 686.775, cx=480, cy=270. Using
these measured principal points removes the nominal half-pixel comparison bias.
The evaluator uses independent analytic ray/box intersections plus mesh geometry,
excluding silhouettes and transparent surfaces from opaque depth comparisons.
An off-axis target whose radial distance exceeds 20 m but axial depth is within
20 m verifies the axial far-plane convention. Its RGB is present and depth invalid.

## Validation and timed trials

| Validation | Result |
| --- | --- |
| Isolated final-backend image build | Pass; original core/Ogre2 backend retained |
| Isolated application build | Five ROS packages built successfully |
| Halo Linux navigation suite | 191 tests passed, four hardware-specific cases skipped |
| Halo Linux perception suite | 157 passed after supplying existing read-only/copied CPU test dependencies and packaging documentation |
| Halo Linux environment suite | 144 passed at application v5 |
| Final fixture/profile focused checks | 12 fixture/management checks and 7 camera/annotation checks passed |
| Visibility acceptance rejection checks | Three passed: RGB disappearance, bad depth, alpha/near behavior |
| Camera recovery/manual speed/control lifecycle harnesses | Pass |
| Windows full environment suite | Retained WinError 5 concurrent review-page replace failure; review code unchanged, Linux suite passed |

The expanded final material probe passed. An earlier v3 image completed a
600-second stationary recording before decode fallback was hardened. That
recording is preserved but does not count toward the final v4 image's required
three trials. Earlier probe-discovery failures and startup/test-environment
failures are also retained; none are relabeled as passing acceptance.

| Final v4 trial | Outcome | Geometry/RGB review |
| --- | --- | --- |
| Stationary, ten minutes | Pass, including protected teardown review | 600.001 s; all 553 saved frames and 18,183 RGB-D pairs reviewed; no camera/pose failure; peak Tctl 39.75 C |
| Approach/retreat, first attempt | Stopped after 3.825 s; not a pass | Pose-service request timed out; 26 matched pairs, no uniform RGB, empty depth or missing pose |
| Approach/retreat, ten-minute retry | 600.020 s completed; all offline gates pass, protected teardown pending | All 600 saved frames / 18,183 synchronized pairs reviewed with v9 evaluator |
| Rotation, first ten-minute attempt | 600.017 s completed; offline gates failed | Four saved frames fail depth-fraction/RGB gates; retained as failed qualification |
| Rotation, measured-bracket retry | 600.017 s completed; all offline gates pass, protected teardown pending | All 600 stable-bracket saved frames / 18,183 checked RGB-D pairs reviewed; 5,401 accepted requests; peak Tctl 37.125 C |

Across the complete stationary recording, worst per-frame p95 depth error is
0.0401 mm, with no missing/unexpected usable depth and 100% RGB support in
qualified object interiors. Across approach/retreat it is 4.5824 mm, with zero
missing expected depth, at most 0.00367% unexpected usable pixels and >=99.57%
RGB support. At most 0.07266% of compared pixels exceed 3 cm, within the 1%
evaluator gate. The approach trial's peak Tctl is 36.875 C. Its measured path
and request timing are retained in `measured-motion-review.json`; actions
continue through 599.94 seconds, with no interval longer than 0.233 seconds.
Final rotation has worst per-frame p95 depth error 0.08345 mm, zero missing
expected depth, at most 0.000193% unexpected usable pixels and >=99.99978% RGB
support. Its largest over-3-cm fraction is 0.000216%. All 600 saved measured
brackets validate. The CPU-only rotation analysis peaked at 72.125 C; this is
separate from the 37.125 C rendering-trial peak.

The first moving trial retained its failure and stopped only its fixture.
Capture recorded no GPU/incident marker at that point. The pose client shared
a Python process with high-rate image callbacks; callback/request contention
is a working hypothesis, not an established GPU failure. Revision v7 moves
pose RPCs into a separate transport-only process, restricted to the isolated
fixture partition, camera entity and bounded evaluator trajectory. Its focused
source checks passed; short motion checks precede another timed attempt.
The v7 source archive SHA-256 is
`781d3a4da367789b70571e734bcae84c9dcef682b19086e4e421679129946f7d`.
The final image and fixture geometry are unchanged. Stationary v6 evidence
remains separate from the moving-camera recordings. Both v7 30-second checks
completed with all pose requests accepted and all saved RGB/geometry gates
passing: 28 approach frames / 911 pairs and 27 rotation frames / 910 pairs.
Revision v8 anchors saving to one-second deadlines instead of adding writing
time to each interval. Its source archive SHA-256 is
`8ed90cd814d0e0a5ef1553e098c269ef212aab7604f53dc373d5b98360e72778`.
Its 30-second cadence check saved 30 frames, checked 910 synchronized pairs,
accepted all 280 pose requests and passed every saved RGB/geometry gate.

The complete approach recording accepted 5,583 pose requests and covered measured
x=0 through 1.799998 m with no camera/pose failure. Its first full analysis
flagged frame 244's near-background mask: 81.34% raw support across 836 pixels.
The complete two-pixel-eroded interior has 615 pixels and 100% correct background;
the discrepancy is confined to the moving silhouette. Measured pose timing is
18 ms from the frame, and a 1 mm pose shift explains the projected boundary.
This does not identify a renderer disappearance. Revision v9 applies the same
two-pixel silhouette exclusion already used by the other RGB/depth checks to
the near-background comparison. It retains raw and interior metrics separately,
keeps the 90% threshold, and adds a regression proving interior RGB loss still
fails. The original failed verdict and geometry analysis remain untouched;
the new full analysis writes versioned outputs beside them.

The first complete rotation recorded 600 frames / 18,183 checked pairs and
5,424 accepted requests, but frames 267, 270, 271 and 273 failed geometry/RGB
gates. A diagnostic found that all four images agree with the immediately next
requested pose: depth p95 becomes 0.0081-0.0629 mm, over-3-cm fraction becomes
zero, and overlap-target RGB support becomes 100%. Requested poses remain
diagnostic only; they were never substituted for measured acceptance truth.
`pose-timing-diagnostic.json` retains both comparisons. The failed rotation is
not counted as a pass.

Nearest-in-time measured poses can straddle a discrete fixture update even
with a 2-24 ms timing gap. Revision v10 requires matching measured camera poses
strictly before and after each geometry-qualified saved frame, both within
50 ms. All matched RGB-D pairs still receive camera-health checks; ambiguous
pose brackets are counted and excluded from saved geometry frames. Saved
brackets and the stricter policy are independently checked by the offline
evaluator. A regression covers the case where a close timestamp hides a pose
change. The image, trajectory and acceptance tolerances remain unchanged.
The one-minute v10 check saved 60 stable-bracket frames, checked 1,819 RGB-D
pairs and passed every RGB/geometry gate, with worst p95 depth error 0.0715 mm.
It counted 589 ambiguous transient brackets and used none for saved geometry.

The original capture reached its two-hour bound at 22:31 UTC while workloads
were stopped. Protected closure at 23:41 UTC reports
`MANUAL_SESSION_CLOSED_CLEANLY`, no kernel incident matches and no capture
incident/stop marker. `trial-stationary/protected-review.json` records the final
review. The operator then started fresh capture and temperatures for the moving
trials; the stopped capture was not reused.

Timed capture alone is not a pass. Review every saved pose/frame comparison,
all-frame camera health and capture/kernel evidence. Approaching/retreating and
rotating camera replays occur only in the isolated fixture. These do not qualify
guarded physical/manual rover traversal or autonomous navigation.

The offline evaluator requires a complete 600 seconds, >=540 saved frames, all
saved frames analyzed with published calibration, and zero uniform-camera,
empty-depth or missing-pose events. For opaque interior pixels it requires p95
depth error <=3 cm, <=1% beyond 3 cm, <=0.5% missing expected depth and <=0.1%
unexpected usable depth. Each predicted object mask >=100 pixels requires >=90%
RGB chromaticity support; smaller masks are retained as diagnostics. Below-near
background and translucent sky ROIs have separate gates. These are evaluator
tolerances; no navigation threshold was changed. A passing offline result still
requires protected kernel/capture teardown review.

## Evidence and remaining scope

Halo root:
`/home/ian/capstone-training/rover-navigation/navigation_sessions/rendering-phase1-resume-20261104T202728Z`.

- `build-v4/`: exact final image/build context hashes, logs and thermal samples.
- `qualification-v6/`: final fixtures, raw RGB/depth/pose/calibration, comparisons,
  material probes, process/library provenance, completed stationary trial and
  interrupted first approach/retreat attempt.
- `qualification-v7/`: separately hashed pose-client repair and new motion trials.
- `qualification-v8/`: final motion recorder with anchored one-second saving.
- `qualification-v9/`: consistent near-boundary evaluator and final rotation tools;
  its camera recorder is byte-identical to v8.
- `qualification-v10/`: stricter measured-pose brackets and fresh rotation retry.
- `application-v5/`: isolated source/install, build/regression logs and candidate
  launch configuration. Its pins are separate from the default launcher's pins.
- `baseline-*`, initial inventory and earlier versioned directories: readiness,
  original source/image identities and preserved diagnostic/interrupted work.
- Original boot-bound case: `oem617-7141-rocm-manual-20261104t203116z-fe48-20261104T203120Z-krzjphy8`;
  protected capture/temperature/kernel/device-dump workflow remains authoritative.
- Moving-trial case: `oem617-7141-rocm-manual-20261104t234127z-408e-20261104T234130Z-xcr6zve0`.
- `VISIBILITY_OFFLINE_REVIEW.json`: all three passing offline verdicts, exact
  configurations and totals: 1,753 saved geometry frames / 54,549 checked RGB-D pairs.
- `operator-kits/` and `retained-launcher/`: verified copies of the original
  guard sources and unchanged baseline launcher.

The off-host visibility checkpoint is
the evidence archive (`navigation_sessions/rendering-phase1-resume-20261104/rendering-phase1-resume-20261104-visibility-evidence.tar.gz`, local-only)
(1,753,769,326 bytes), SHA-256
`d8c5786b7cf65da4dd862893c5abb06971aed83c00645e1c6c0f607d322c5cf3`.
Local verification (`navigation_sessions/rendering-phase1-resume-20261104/visibility-archive-verification.json`, local-only)
checks the complete archive SHA-256, all 10,357 file hashes and four symbolic-link
entries. Its manifest explicitly excludes future live session directories and
Python caches. This checkpoint records offline success with protected/live
review pending; it does not assert final acceptance.

`PHASE1_ACCEPTANCE_FINAL.json`, recorded at 02:55:50 UTC on November 5, reports
`PHASE1_RENDERING_ACCEPTED`. The approach/retreat and rotation trials each have
a passing `protected-review.json`; the moving-trial case closed cleanly at
02:26:20 UTC without a kernel incident match or capture incident marker.

The operator started fresh logging for the real navigation preview at
02:44:45 UTC, inspected and drove the candidate, stopped it, and reported
"Phase one looks great." Actual mapped Radeon/Mesa/Ogre provenance passed;
the camera retained 960 × 540, 20 m RGB / 8 m usable depth, 0.30 m near and
0.4 rad pitch. The supervisor recorded an operator stop, no failure and no
cleanup error. Peak preview Tctl was 67.5 C. The preview evidence is under
`application-v5/supervisor-sessions/phase1-live-20261105T024445Z` and the matching
`application-v5/navigation_sessions` directory.

The fresh preview case, `oem617-7141-rocm-manual-20261105t024321z-3e79-20261105T024321Z-xuut7_lk`,
closed with `MANUAL_SESSION_CLOSED_CLEANLY` at 02:48:21 UTC and no kernel
incident matches. Capture ended after workloads stopped but before protected
cooldown finished. The protected journal covers cooldown; uninterrupted
capture through cooldown is therefore not claimed. Closed case copies and
the final acceptance record are retained under the Phase 1 evidence root.

The original off-host checkpoint above remains immutable. The separate
completion supplement (`navigation_sessions/rendering-phase1-resume-20261104/rendering-phase1-resume-20261104-completion-evidence.tar.gz`, local-only)
records the later protected reviews, live preview and final verdict: 132,490,444
bytes, SHA-256 `accb9c59ae65246db2c36b1a7dcd9744b2eab84de10c82472e34632f05ebbd7a`.
[Off-host verification](../evidence/development-20261107/rendering-phase1-resume-20261104/completion-archive-verification.json)
checks the archive and all 78 completion-file hashes against its manifest.
Long-duration full-stack stability and the initiating causes of previous whole-GPU
resets remain unconfirmed. These results qualify rendering and brief attended
manual use. They do not qualify later terrain/mapping phases, dataset expansion,
training, model export or autonomous approach.
