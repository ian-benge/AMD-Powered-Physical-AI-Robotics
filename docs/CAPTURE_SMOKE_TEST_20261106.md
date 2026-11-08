# Ten-image capture test — November 6, 2026

**Follow-up: the camera compatibility fix passed a ten-image repeat.** See the
[fix and repeat report](CAPTURE_OGRE_FIX_20261106.md) and its image previews.
The original failed run is preserved below. The 200-image pilot has not started.

**Original result: FAILED BEFORE IMAGE CAPTURE.**

The requested small test exposed a capture compatibility problem. The dataset
robot adds an auxiliary Gazebo bounding-box camera, which the retained Ogre
backend does not implement. All three preserved attempts report:

```text
BoundingBox camera not supported by: ogre
Segmentation fault
gazebo-1: process has died ... exit code 139
```

The stack points to `gz::sensors::v8::BoundingBoxCameraSensor::CreateCamera()`.
Gazebo exits before the capture node receives a usable initial sensor set.
The dataset orchestrator retains the failed attempts and refuses to commit
images without completed reports. ROS launch returns zero after shutting down,
but the missing capture report still makes the enclosing capture job fail.
No source labels, export labels or image previews were produced.

## Test scope and completed checks

- Planned ten frames in two separately generated 40 m scenes: eight populated
  views, two primary targets per exported class, and two background views.
- Reserved training groups: populated seed 4312000 and background seed 4312013.
  The derived eight/two-view test configuration is separate from the unchanged
  twenty-view-per-scene pilot. Test outputs are excluded from future training.
- Both scene plans pass scene validation and four-class quota audits. Background
  geometry is target-free. The camera retains 960 x 540, 0.40 rad downward tilt,
  20 m RGB, 8 m usable depth and the qualified Ogre clipping conventions.
- All 1508 v9 candidate pins were checked before execution. The application is
  mounted read-only; generated scenes and attempts use a separate writable root.
- Actual mapped Ogre/Mesa/Radeon renderer provenance passes for all three
  launches. This establishes the renderer identity, not support for the
  auxiliary bounding-box camera.
- Fresh native capture and visible temperature monitoring supervise the work.
  There are 327 guarded-step health observations; observed peak Tctl is
  80.875 C, below the 90 C stop limit. No later rendering attempt was started
  after identifying the compatibility failure. NPU inference and the terrain
  service were left off. No detector retraining or full pilot was started.

## Exact identities and preserved evidence

Halo test root:
`/home/ian/capstone-training/rover-navigation/navigation_sessions/capture-smoke10-20261106T132405Z`.

Native case:
`oem617-7141-rocm-manual-20261106t132002z-b959-20261106T132016Z-d3xybrbg`.

Application: `environment-v2-20261105T225758Z/application-v9`.
Source archive SHA-256:
`8dea3b574bf6a44a399ff55c859d00a8a8af2a40ce6c4fe8b18648437537c70b`.
Renderer image:
`sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa`.

Populated scene SHA-256:
`6122d2705af31851e2f19d5ad899727fec1c813e6a31313e068cba77cd49b199`.
Background scene SHA-256:
`d1c25a49593cb66e46248a075c3b245c9745ec97fd64fbc6404bda087335fd9e`.

The root contains the original generated bundles, separate attempt directories,
launch logs and stack traces, script identities, generation/capture guarded-step
results and health logs, renderer proofs and `SMOKE_TEST_RESULT.json`.
`COREDUMP_INDEX.txt` records that `coredumpctl` is unavailable; no raw core was
extracted. The actual simulator stack traces remain in every launch log.

All test containers are stopped. Protected finish at **07:37:59 CST**
(13:37:59 UTC) records `MANUAL_SESSION_CLOSED_CLEANLY`, empty workloads and no
matching new kernel incident. Capture continued through 07:38:12 CST and
temperatures through 07:38:13 CST, covering the protected cooldown/final capture.
The full temperature log's sampled Tctl peak is 79.875 C; the guarded step's
separate direct readings peak at 80.875 C. Neither exceeds the 90 C limit.
This clean teardown does not make the failed capture test pass.

`SMOKE_EVIDENCE_OFF_HOST_VERIFIED.json` records complete off-host evidence
verification after session closure, including both scene bundles and all
preserved failed attempts. The archive receipt provides its exact hash.
The complete 168343622-byte archive remains on Halo: C: had only about 176 MiB
free at copy time. Verification streams the full archive without storing it,
and retains a bounded diagnostic subset locally. Full off-host archive storage
has therefore not been established. Archive SHA-256:
`73077da21107796c943051e6b29f0f0eb3d2cc0ba4d5a58d45306f96d1e94c15`.

## Required next change

Implement and qualify a dataset capture sensor path compatible with Ogre,
retaining RGB-D synchronization, geometry labels, visibility and framing checks.
The independent bounding-box check needs an explicit compatible implementation
or a separately justified and tested replacement; disabling its readiness gate
alone still leaves the unsupported sensor in the robot description.

Keep the renderer, hardware/runtime policy and accepted simulation launchers
intact. Preserve these failed attempts. Repeat a small capture test after the
fix and review actual image/label previews before starting the 200-image pilot.
The experimental 40 m manual-driving default is unchanged by this test.
