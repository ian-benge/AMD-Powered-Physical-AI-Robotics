# Environment v2 closeout

November 5, 2026 (closeout November 6 UTC).
**IMPLEMENTATION DELIVERED; 40 M EXPERIMENTAL DEFAULT BY USER REQUEST.**

The user ended further attended checks and explicitly requested the 40 m
default. This changes the separate candidate launcher; full qualification
remains incomplete. Failed and missing checks remain visible below. Accepted
Phase 1-3 evidence and the native 20 m launcher are retained. No further drives
are required for this closeout.

Halo evidence root:
`/home/ian/capstone-training/rover-navigation/navigation_sessions/environment-v2-20261105T225758Z`.
Local evidence: `navigation_sessions/environment-v2-20261105/`.

## Delivered implementation

- Separate `mixed_reachable` version 2: 40 x 40 m navigation area, shared
  257 x 257 visual/collision grid, continuous 20 m surrounding apron and
  8192-pixel ground composite. Physical texture/object scales, 1.3 m launch
  clearance, 0.8 m maximum relief and 22-degree generation limit are retained.
- Sparse / standard / cluttered rock presets (76-110 / 152-220 / 228-330),
  separately seeded appearance, lighting, dust, orientation and clustering.
  Near 3-7 m and outer 12-17 m four-class staging routes retain conservative
  corridors, turning clearance and slopes below 8 degrees. Evaluator geometry
  and route certificates are not controller inputs.
- Scene-derived map extent, 1600 x 1600 terrain memory at 2.5 cm and
  800 x 800 global grid at 5 cm. The 6 m local map and per-frame footprint
  remain unchanged; outside-map terrain remains unknown. Restart/Generate
  preserve the selected version, density and camera.
- Manual initial speed 0.05 m/s, cap 0.20 m/s and turn cap 0.30 rad/s.
  Restart restores the initial speed. Hazard thresholds, clearing requirements,
  swept footprint, explicit manual selection, rearming and expiry are retained.
- Capture profile: 960 x 540, existing mount/FOV, 0.40 rad downward tilt,
  qualified Ogre renderer, 20 m RGB, 8 m usable depth and existing clipping.
  Geometry labels include eligible RGB targets beyond usable depth. Legacy
  three-class source IDs retain their meaning; metadata performs the existing
  mudstone-slab export conversion.
- Ten future training scene groups reserved: eight populated, two backgrounds,
  20 views each, with five primary-target views per exported class in populated
  scenes. Held-out assets, appearance recipes and split assignments are reserved.
  Failed capture attempts retain separate outputs/logs; resume checks hashes and
  completeness. Capture does not silently select a software renderer.

**No 200-image pilot, bulk capture, retraining or autonomous mission occurred during this environment effort.**
The six rendered camera fixtures are diagnostic evidence only.

## Completed checks

| Check | Result and scope |
| --- | --- |
| Predeclared layouts | All 50 seeds 4311000-4311049 pass independent saved-mesh audits and exact regeneration; 400 near/outer class routes. Bounded retries/failures retained, no seed substitution. Density/sunlight buckets balanced. |
| Materialized scenes | Sparse 4311000, standard 4311001 and cluttered 4311002 match audited geometry. These three representatives use low sunlight; the 50-layout set covers other lighting buckets. |
| Linux source | Five packages build. 208 navigation checks (four GPU-only skips), 176 environment and 157 perception checks pass. V8/v9 repeat build/navigation; environment/perception source is unchanged from fully checked v6. |
| Compatibility and controls | Legacy scenes, Restart/Generate propagation, camera recovery and manual caps/lifecycle pass. Isolated 0.20 m/s guard cases cover unknown/hazardous ground, stale inputs and released controls. |
| CPU/ROCm mapping | All 14 actual ROS 40 m cases pass exact published-cell/clearing comparisons, repeated through v9: outer/negative positions, all four boundaries, weak clearing, retained hazards and sensor/localization loss. Native classify/depth/project float64 parity also passes for the resumed case. |
| Texture support | Desktop GL limit 16384; all three 8192 maps allocate, generate mipmaps and read back exactly. Logical base bytes 469762048; full-mipmap estimate 626349397. These are format sizes, not isolated physical allocations. |
| Rendered diagnostics | Six actual qualified-Ogre RGB-D/annotation fixtures pass per-object checks for four classes, burial/occlusion, backgrounds, outer views and clipping. RGB-only targets beyond 8 m retain labels. Images reviewed and copied off-host. |
| Review gates | Twelve read-only checks cover positive/known-bias cases and reject missing truth/outer evidence, excess latency, contact and missing/unstarted contact wiring. |
| Publication optimization | Four contiguous/noncontiguous 400/800-grid cases preserve all decoded fields, cells and encoded field bytes. Four known uninitialized CDR padding bytes are explicitly excluded. Contiguous 800-grid conversion median improves 8.626 ms to 0.029 ms; noncontiguous 8.82 ms to 1.07 ms. |

The v2 painter uses padded patch windows instead of processing whole 8192 images
for every patch. The v1 painter and random draw order remain intact. An earlier
whole-image attempt was stopped after over ten minutes/about 7 GB. Its partial
files, failed builds, initial benchmark failures and interrupted recorders remain
preserved.

## Live measurements and remaining limits

| Run | Timing | Traversal / synchronized review | Acceptance |
| --- | --- | --- | --- |
| V6 20 m stopped baseline | RTF 1.000006; p95 local-publication latency 137.54 ms | 109/109 geometry frames pass; worst per-frame ground p95 2.775 mm | Completed v6 baseline |
| V6 sparse 40 m | RTF 1.000002; p95 351.67 ms | Axis 7.440 m; return 0.233 m; three speeds without recorded inter-stage Stops; recorder interrupted, exit 137 | Not accepted |
| V8 sparse 40 m | RTF 1.000005; ratio to v6 baseline 0.999999; p95 **267.73 ms**, 1521 steady-state samples | Axis **13.1089 m**; return **0.6768 m**; 115/115 geometry frames pass, eight outer frames; worst ground p95 **4.685 mm**; no reported contact/live guard faults | Extent, return, rendering and RTF pass; latency fails; saved commands omit 0.05/0.10 stages |
| V9 partial 20 m baseline | Partial p95 128.23 ms, 215 post-warmup samples | 46 raw synchronized frames; stopped before completion, recorder exit 137/empty final audit | Partial observations, not an accepted matched baseline |

Latency measures mapper callback receipt through successful local publication,
including transform waiting and compute/publication work. It does not measure
the entire sensor-exposure-to-actuation path. The bound remains p95 <=200 ms;
V8 exceeds it by about 34% despite its smooth appearance.

V8's synchronized review reports maximum fixed-first-pose VIO position drift
44.246 mm and orientation drift 0.2186 degrees, using one fixed initial
alignment and no trajectory fit. Every reviewed frame meets the 15 mm ground
bound. Contact silence is supported by saved world, collision, sensor/bridge,
actual conversion and bridge startup evidence.

V9 retains optimized byte-array publication and polls pending transforms every
50 ms on 40 m maps; legacy maps retain 200 ms. Float64 results, frame admission,
publication order and safeguards remain unchanged. Its offline cases pass,
but **no v9 40 m live run validates that latency fix**. Standard and cluttered
attended outer-and-return runs have not occurred. These checks are deferred,
not passed. This effort establishes neither endurance nor autonomous navigation.

Receipts: `BASELINE_RUN_REVIEW_v4.json`, `BASELINE_RENDERED_REVIEW_v2.json`,
`SPARSE_V6_RUN_REVIEW.json`, `application-v9/PRIOR_V8_RUN_REVIEW.json`,
`application-v9/PRIOR_V8_RENDERED_REVIEW.json`,
`V9_BASELINE_PARTIAL_OBSERVATIONS.json`.

## Exact candidate and default decision

Final candidate: **application-v9**, 1508 pins, 338 source files.
Source archive SHA-256:
`8dea3b574bf6a44a399ff55c859d00a8a8af2a40ce6c4fe8b18648437537c70b`.
`SOURCE_V9_OFF_HOST_VERIFIED.json` verifies the archive and every source file.

Renderer image:
`sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa`.
Ogre plugin SHA-256:
`f6e8842247198a2b34b25cfdb40a8f4bc4f156c2e90a54135a68698dc239a3e6`.

The terrain worker remains pinned v6, explicitly referenced by v9: isolated
Torch 2.12.0+rocm7.14.1 / HIP 7.14.60850, float64, capacity 1600 and verified
requested/observed HIP flags 4. NPU/runtime, robot, gravity, friction and
power/fan policy are retained. Ubuntu 24.04.5 remains an experimental deviation
from the named AMD 24.04.4 runtime row.

NPU ONNX/model SHA-256:
`2d47f7591563869cc7cbd6f118342d82db3427dbee8e8eff5c5b3b0589a8b03a`.
Source `.pt` checkpoint SHA-256:
`89bbcf4ff124996fb6a520c5bf751287c2b5ea47bbd1dd7e2adaf7f530ee44ca3`.
These identify different artifacts.

The user's explicit instruction was: "done, lets make the 40m the default. it
seemed fine." The separate `start-environment-v2.sh` entrypoint now selects
the sparse 40 m v9 candidate with no argument. Its wrapper verifies the unchanged
v9 starter hash and delegates through existing pinned configuration, native
health, rendering and parity gates. Initial speed remains 0.05 m/s, cap 0.20,
explicit manual enable and the ten-minute bound remain. `baseline` selects 20 m;
`1`, `2`, `3` select sparse, standard, cluttered. The native accepted launcher
is unchanged.

`DEFAULT_LAUNCHER_IDENTITIES.json` records **EXPERIMENTAL_DEFAULT_BY_USER_REQUEST**
and supersedes historical default-entrypoint receipts. Historical starters and
receipts are preserved, including the previous entrypoint in `launcher-history`.
Pinned v9 configuration and its incomplete qualification status are unchanged.

## Resources and protected closure

Read-only resource sampling retained 554 observations from 03:50:56 through
04:39:11 UTC. Minimum host MemAvailable: 51526057984 bytes (47.99 GiB).
Peak observed simulator cgroup memory: 6037069824 bytes (5.62 GiB); whole-driver
VRAM accounting: 4446638080 bytes (4.14 GiB), GTT: 188264448 bytes. Host/GPU
values include other desktop/services and cgroup memory includes cache. These
are sampled observations, not isolated allocations or full-session peaks.
The full temperature log peaks at Tctl **79.125 C**, below the 90 C stop limit,
with no recorded temperature stop requirement. See `CLOSEOUT_MEASUREMENTS.json`
and the preserved native temperature log.

Resumed case:
`oem617-7141-rocm-manual-20261106t032820z-1d6d-20261106T032821Z-vod34yha`.
Protected finish at 04:40:39 UTC records `MANUAL_SESSION_CLOSED_CLEANLY` and
`post_teardown_clean: true`: empty containers/service ports, no KFD clients,
no pstore files and no matching new kernel incident. The underlying protected
report does not claim the unperformed timed/endurance qualification.

Temperatures ended at 04:39:20 UTC and capture at 04:39:26 UTC, before the final
protected capture at 04:40:38 UTC. The final approximately 72 seconds therefore
lack continuous capture; temperature coverage ends earlier still. The original
case also had a capture-expiry-to-delayed-finish gap. Both remain limitations.
Protected finish does not establish continuous monitoring or endurance.

## Evidence preservation

Checkpoint 1 is verified off-host: 1279632473 bytes, 10730 files, 11 symlinks,
SHA-256 `383586c7a8894096463e49fd789a47278edf04f3f6d026fab2feee4e2dd324c0`.
It predates resumed checks. The final closeout archive adds resumed data,
interrupted attempts, candidate/default identities, delivery documents and
protected closure. Its separate immutable receipt and off-host stream-verification
record provide the final archive hash without embedding a circular hash in its
own contents. Consult `FINAL_CLOSEOUT_OFF_HOST_VERIFIED.json` for completion.

The delivered outcome is the larger environment as the user-selected
experimental candidate default and a prepared capture pipeline. Full live
qualification and the uncollected training dataset remain open.
