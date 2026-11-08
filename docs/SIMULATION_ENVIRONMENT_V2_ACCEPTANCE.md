# Environment v2 acceptance record

**CLOSED: implementation delivered; 40 m experimental default by user request.**
Full qualification remains incomplete. The final measurements, deferred gates,
exact default decision, protected closure and archive references are in the
[closeout report](SIMULATION_ENVIRONMENT_V2_CLOSEOUT.md). That report supersedes
the working log below. The user explicitly ended further attended checks and
requested the separate candidate's 40 m default. No pilot images were collected.

## Preserved working log before closeout

The following is historical progress, including superseded source versions,
resolved transfer approval and pending checks. It is not the final status.

November 5, 2026. **IN PROGRESS — NOT PROMOTED.**

Authorized scope: implement and qualify a 40 x 40 m operating world, prepare the
capture pipeline, and qualify guarded manual driving up to 0.20 m/s. The
200-image pilot, bulk capture, retraining and autonomous missions remain outside
this task.

Halo evidence root:
`/home/ian/capstone-training/rover-navigation/navigation_sessions/environment-v2-20261105T225758Z`.
Local evidence: `navigation_sessions/environment-v2-20261105/`.

## Completed source work

- Versioned 40 m profile, shared 257-grid terrain, joined 20 m context ring,
  8192-pixel ground materials, density/lighting presets and independent seeds.
- Four-class near/outer staging certification with independent saved-mesh audit,
  deterministic replay and preserved rejected layout attempts.
- Scene-derived 40 m navigation extent, restart/generation propagation and a
  conservative initial manual speed with a separate 0.20 m/s cap.
- Four-class export targeting without changing legacy source IDs; uncaptured
  training-only pilot configuration and content-verified capture attempt resume.
- Candidate-only bounded terrain capacity extension; retained interpreter,
  libraries, HIP scheduling, baseline source and operator kit remain intact.

## Evidence so far

The initial three real-asset density probes passed geometry and exact replay.
Their random target placement needed several retries; the revised candidate
stratifies target placement into near/outer bands without weakening route
criteria. The corrected Linux candidate passed its five-package build and source
suites before entering the predeclared 50-layout run.

Earlier build failures are preserved. A minimal legacy fixture omitted terrain
metadata; the new context writer now accepts that existing fixture shape.
Browser checks passed for manual-speed defaults/caps, restart, control lifecycle
and camera recovery. Expanded CPU grid-bound and capture-attempt tests pass.

All 50 predeclared seeds (4311000 through 4311049) passed independent saved-mesh
audits and exact seeded regeneration: 400 near/outer class routes in total.
No failing seed was substituted. The first materialization attempt stopped when
an ancillary login session disappeared during native session enumeration;
capture and the active graphical desktop subsequently passed the unchanged gate.

The latest candidate passed a five-package Linux build, 205 navigation checks
(four GPU-only skips), 176 environment checks and 157 perception checks.
Additional checks cover all four local-map boundaries, latched 0.20 m/s guard
stops and released browser leases. RGB-visible geometry beyond 8 m retains its
labels; missing usable depth there does not create background. Legacy depth
and source-class policies retain their existing behavior.

The first full-image 8192 material attempt was explicitly stopped after over ten
minutes and about 7 GB of memory. Its partial files and guarded stop remain in
the evidence root. The v2 material painter now blurs padded patch windows at
the same physical scale; the v1 painter and random draw order remain intact.
The new recipe passes deterministic regeneration and category-mask checks.

Source-v5 archive SHA-256:
`9b4fd1b1cfff3c68998413e9806cb2602af25f0fd2197a248c53e75ed30556d1`.
Application source, later diagnostic tools and final installed files require
separate pins in the final checked configuration.

The final pinned source is now application-v6, archive SHA-256
`9d4327dc1d554ae0de48b3c2f1ca41645bf7d88e3669d47fa43fc99492f2d059`.
Its five-package build and 208 navigation / 176 environment / 157 perception
checks pass (four GPU-only navigation skips). All three actual ROS launch setups
pass, including the resolved 40 m size and conservative initial speed. The
Generate button supports the separate source layout and a bounded 600 s v2
material build; legacy generation keeps its original 180 s limit.

All three full-material scenes match the predeclared geometry. Fourteen actual
ROS CPU cases pass at outer/negative positions and all four boundaries, with
weak-artifact clearing, retained hazards and clearing suspended on sensor or
localization loss. The v6 mapping/geometry implementation is byte-identical to
the v5 implementation used for this replay; that derivation is recorded.

Desktop hardware qualification completed a 16384-pixel texture-limit query and
allocation/mipmap/exact readback of all three 8192 ground maps. Logical base
texture data totals 469762048 bytes, with a 626349397-byte logical full-mipmap
estimate; these are format sizes, not a claim about physical driver allocation.
Six isolated qualified-Ogre RGB-D fixtures cover all four classes, an outer
view, burial/occlusion and an unpainted target-free background. All per-object
visibility/depth checks pass, including RGB-only pixels beyond 8 m. Renderer
provenance is retained for each fixture. Diagnostic RGB/boxes were visually
reviewed and copied off-host. The background diagnostic uses unpainted
procedural dirt; future pilot backgrounds still use the configured 8192 ground
composite without target regions.

The candidate has 1506 source/install/scene/tool pins. Its checked starter is
`start-environment-v2.sh`; the candidate-only terrain starter is
`start-environment-v2-terrain.sh`. These checks rendered isolated camera worlds
without a rover. The resumed session independently verifies NPU and candidate
terrain readiness, exact process identities, float64, capacity 1600 and HIP
flags 4. Live speed and larger-world driving acceptance have not occurred.

The first capture expired during the usage interruption; the terrain supervisor
stopped its worker on capture loss. Protected finish later records a clean
teardown, but continuous capture does not cover expiry through that finish.
This monitoring gap remains a limitation of the old case. A fresh attended
capture and temperature display passed native guards before resumed workloads.

The checkpoint archive is now verified off-host: 1,279,632,473 bytes, 10,730
files, 11 symlinks, SHA-256
`383586c7a8894096463e49fd789a47278edf04f3f6d026fab2feee4e2dd324c0`.
This preserves completed implementation and diagnostics; the final archive must
also include resumed acceptance checks and their protected closure.

The resumed case passed the retained classify/depth/project float64 parity
check. All fourteen actual 40 m ROS cases then passed exact CPU/ROCm published
grid and clearing-evidence comparisons, including outer and negative poses,
all four boundaries, weak-artifact clearing and protected/lost-input cases.
Evidence: `application-v6/controlled-rocm-40-attempt-1/result.json` and
`rocm-mapping-step-1/result.json` (completed 03:36:56 UTC November 6).

The separate read-only review tools now require image-time truth for every
captured frame, outer-area geometry for v2 scenes and explicit stops between
the three manual speed stages. Nine acceptance-review checks pass, including
retained positive/known-3-cm-bias metrology and deliberately missing evidence,
excess latency and contact cases. These review tools are separately hashed in
`RESUMED_REVIEW_TOOLS.json`; the pinned v6 runtime remains unchanged.

`CAPTURE_PREPARATION.json` reserves the ten future training scene groups,
held-out assets and appearance families against the unchanged split function.
The configuration requests 200 views, five primary-target views per exported
class in each populated scene. Pilot scene generation, image collection and
retraining have not been performed.

## Outstanding acceptance

The resumed 20 m stopped baseline passed independent review of all 109
synchronized frames: median RTF 1.000006, steady-state p95 depth receipt to
local publication 137.54 ms, maximum per-frame ground p95 error 2.775 mm.
No contact events were reported. The event-only contact publisher is verified
through the saved world/robot/bridge configuration, actual URDF-to-SDF collision
conversion and the bridge startup log. Its silence is not treated as a stream
of empty contact messages. See `BASELINE_RUN_REVIEW_v4.json` and
`BASELINE_RENDERED_REVIEW_v2.json`.

The first sparse 40 m run is retained as **not accepted**. It had median RTF
1.000002, p95 publication latency 351.67 ms, no reported contact or live guard
fault, all three speed levels observed, and a final center distance of 0.233 m.
The largest axis coordinate was 7.440 m. Explicit stops between speed stages
were not recorded. The synchronized recorder was interrupted before completion
(exit 137; empty final audit JSON); its raw partial frames remain preserved.
Evidence: `SPARSE_V6_RUN_REVIEW.json` and
`application-v6/supervisor-sessions/phaseenvironment-v2-control-20261106T034948Z`.

A separate publication optimization uses the ROS signed-byte array setter for
int8 grids, retaining the previous validation path for other dtypes. It does
not change float64 terrain computation, cells, thresholds or publication order.
The first revised build passes all five packages and 208 navigation checks
(four GPU skips). The benchmark found non-deterministic CDR alignment padding;
independent diagnostics confirm identical decoded fields/cells. The corrected
benchmark compares every decoded field and all encoded bytes except the four
padding bytes. It still requires measured conversion improvement and final
mapping/live checks.

Source-v8 archive: `8b81fa800c088af49476ba5fab70a41c6a1cb6e105a721a2509ee2b85204db0c`.
The transfer to the existing Halo test workspace is awaiting explicit user
consent after automatic approval review rejected it as a private-source export.
No drive repetition or subsequent density check has been started with this fix.
The user's latest instruction restores full qualification after briefly asking
to skip the repeat; none of the missing gates has been waived.

- A matched 20 m baseline and three progressive-speed connected manual drives.
- RTF >=0.8 and >=80% of baseline; p95 depth-to-local publication <=200 ms;
  synchronized rendered-ground error <=15 mm under the Phase 3 review method.
- Protected finish/kernel review and verified off-host evidence archive.

No new environment or speed is promoted until these checks pass. Any monitoring
gap, failed run, rejected command or limitation must remain explicit.
