# Ogre capture fix and ten-image repeat - November 6, 2026

**Result: PASS for the capture compatibility fix and ten-image smoke test.**
The 200-image pilot, bulk capture and training have not started.

The original failure and all three crashing attempts remain preserved in
[the first test report](CAPTURE_SMOKE_TEST_20261106.md). The repeat uses copies
of the same two prepared scene bundles, preserving their scene hashes and
eight populated/two background viewpoints. No geometry was regenerated to
make the repeat pass.

## Change and verification scope

Ogre capture now creates only its supported RGB-D camera. It omits the
unsupported auxiliary bounding-box sensor, its bridge and its readiness wait.
The Ogre2 capture path retains its existing bounding-box camera and comparison.
The renderer, robot configuration and accepted simulation launchers are unchanged.

Ogre uses the explicit `sensor_depth_v1` verification contract, recorded in
each frame and completion report. Geometry supplies object identity and the
visible mask; measured, synchronized sensor depth verifies surface support and
the usable-depth box extent. Full RGB box extents are checked against geometry.
This supplies independent surface evidence, **not an independent renderer
instance-ID camera**. Eligible RGB-visible targets beyond 8 m keep their labels
and an explicit geometry-only scope. Missing depth never makes a target background.

Existing visibility, framing, clipping, support thresholds and class IDs remain
in force. Hardware capture uses the configured pose-bracketing limit. Saved-data
validation rejects missing, mismatched or incomplete new checks. Rejected-frame
diagnostics include the contract and box-check details.

Fatal renderer errors now stop after their first preserved attempt, including
when ROS launch returns zero after a child segfault. Other capture failures
retain the existing bounded retries and content-verified resume behavior.

## Repeat results

- Ten 960 x 540 frames saved: eight populated and two target-free backgrounds.
- Both datasets have zero validation errors; both scene attempts committed.
- Two primary-target views for each of rock, pale soil patch, yellow crystal
  rock and mudstone slab. Draft export counts are 129, 10, 6 and 10 labels
  respectively. Source annotations retain the three-class schema; the four-class
  preview uses existing appearance metadata without changing source labels.
- All 155 object appearances pass box verification. Of these, 31 are beyond
  usable depth and explicitly retain geometry-only verification.
- Minimum in-range depth support: 0.95. Minimum measured-depth box IoU:
  0.9166666667. Largest pose-bracket gap: 0.018 s.
- Actual mapped Ogre/Mesa/Radeon provenance passes for both renderer launches.
- Assistant visual review covers the ten-frame contact sheet and full-resolution
  yellow-crystal/mudstone views. No obvious box/surface mismatch was observed.
  Dataset human-review acceptance remains false.

Open the ten-frame contact sheet (`navigation_sessions/capture-ogre-fix-20261106/review/contact-sheet.jpg`, local-only).

Five Linux packages build successfully. All **210 Linux source tests pass**
(186 environment, 24 training), including wrong boxes, missing/corrupt depth,
partial depth range, distant RGB targets, backgrounds, legacy compatibility,
real robot sensor expansion and fail-fast renderer errors. The actual ROS
launch setups pass without starting graphics. The broader Windows suite hits
the existing concurrent review-page replacement `WinError 5`; its Linux
equivalent passes. The focused new Windows regression checks pass.

The first fix attempt exposed a flat-list/2D depth adapter mismatch and was
stopped; its diagnostics remain alongside the corrected repeat. The adapter
now accepts the decoder's row-major values only after checking the exact pixel
count. An initial CPU launch-description check also lacked `DISPLAY`; that
harness failure is retained, and the corrected setup check passes.

These offline capture timings include pose changes, annotation and simulator
startup/shutdown: scene effective real-time factors are approximately 0.637
and 0.293. They do not establish live navigation latency or larger-environment
performance acceptance.

## Identity, closeout and evidence

Halo root:
`/home/ian/capstone-training/rover-navigation/navigation_sessions/capture-ogre-fix-20261106T140200Z`.
The successful candidate and outputs are in `repeat-2/` beneath that root.
All 786 candidate pins were rechecked after capture.

Candidate source archive SHA-256:
`d79a8bcabc756874d87ee74a090df6c1335d985175cb90199024f7d0175668ee`.

Renderer image:
`sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa`.

Populated scene SHA-256:
`6122d2705af31851e2f19d5ad899727fec1c813e6a31313e068cba77cd49b199`.
Background scene SHA-256:
`d1c25a49593cb66e46248a075c3b245c9745ec97fd64fbc6404bda087335fd9e`.
This matches `repeat-2/CAPTURE_RESULT.json`; archived Markdown copies retain
an earlier transcription error in the background hash.

Fresh native case:
`oem617-7141-rocm-manual-20261106t135115z-c11c-20261106T140502Z-bt7c3zr7`.
NPU inference and terrain services stayed off. Guarded readings peaked at
78.875 C; the temperature log peaked at 79.375 C, below the 90 C stop limit.
Protected finish records `MANUAL_SESSION_CLOSED_CLEANLY` at **08:15:29 CST**
(14:15:29 UTC), with clean teardown and zero matching kernel incidents.
Capture and temperature monitoring continued through 08:15:33 CST, covering
the cooldown and final capture. The final temperature `stop_required` marker
follows the operator stopping capture. Test containers and temporary display
authority files have been removed.

`FINAL_FIX_REVIEW.json`, candidate pins, reports, diagnostic attempts, provenance
and protected closeout records preserve the result. A bounded evidence archive
is retained off host with every captured RGB/depth/label/metadata output; the
larger archive also includes copied scene assets and application files on Halo.
Archive receipts and off-host verification are recorded in
`navigation_sessions/capture-ogre-fix-20261106/EVIDENCE_OFF_HOST_VERIFIED.json`.

This result establishes the small end-to-end capture path. It does not approve
a training dataset or fill the remaining 40 m live-performance/density checks.
