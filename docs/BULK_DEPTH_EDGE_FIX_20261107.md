# Batch 6 depth-support recovery

The repeat failure in `scene_4320092/view_000` is resolved in the separately
pinned **capture application v4**. A monitored resume completed batch 6 at
2026-11-07 17:00:59 UTC. The campaign now has **1,200 accepted images in six
completed batches**, with 8,800 images remaining. The verification stopped after
that batch; it did not start batch 7.

## Why capture stopped

The primary soil target passed its framing and depth checks. A thin incidental
slab, `cheyava_rock_001`, was partly clipped at the left image edge and spanned
about 7.54–8.14 m in forward depth. Within the 8 m usable depth range, its
measured support was only 59.6–67.3% in two preserved failures, below the required
90%. The existing mild-mismatch recovery correctly refused this larger
disagreement, so restarting repeated the same rejected composition.

Independent projection reproduced the original expected surface. Backface
culling did not resolve the discrepancy. All 173 and 203 badly mismatched valid
pixels in the two samples instead matched the terrain behind the slab within
3 mm; median terrain error was about 0.2 mm. The rest of the frame agreed
closely, with overall median errors around 14 micrometres. This establishes a
local renderer/evaluator visibility disagreement. Its lower-level Ogre
mechanism is not established by this investigation.

## Capture-only correction

The recovery recognizes this measured failure pattern and proposes a slightly
different viewpoint. Its eligibility requires a good primary target, local
incidental failures, valid depth coverage, a far object initially clipped at a
horizontal edge, and at least 95% independent terrain agreement at the rejected
pixels. Missing sensors, widespread errors and primary-target failures remain
stops. Existing mild-mismatch recovery remains supported.

It tries at most four turns toward the clipped object, at 0.035, 0.070, 0.105
and 0.140 radians relative to the original heading. Rover position, target
identity and export-class schedule are retained. Existing map-boundary,
footprint, lens clearance, target-distance and slope checks still apply.
Every proposed view must produce a fresh synchronized RGB-D/pose sample and
pass all original frame, visibility, box and framing checks before export.
Eligibility for a new pose does not authorize accepting the rejected frame.

The successful rendered case needed only the first **0.035 rad (2 degree)**
turn. The original slab remained labeled and moved fully inside the image:

| Measurement | Fresh original view | Accepted alternative |
| --- | ---: | ---: |
| Required depth support | 90% | 90% |
| Measured depth support | 69.6% | **98.1%** |
| Supported/expected usable pixels | Rejected | **364 / 371** |
| Median slab depth error | Rejected | About **0.2 mm** |
| Slab box | Clipped at the left edge | **[29, 32, 103, 45]**, unclipped |
| Primary target | `silica_soil_000` | Same target; full framing pass |

The slab's 276 RGB-visible pixels beyond usable depth kept their geometry
label. Its remaining 371 usable-depth pixels still underwent independent
sensor checks. The source three-class label and appearance metadata are
unchanged; no global mudstone class-ID conversion was introduced.

All six original failed scene attempts and the new rejected frame are retained.
The alternative pose and its diagnostic evidence appear in `view_recoveries`,
so the existing review page flags the image for review.

## Verification and identities

- **201 Linux environment tests passed**, including 13 focused recovery tests.
- Offline replay recognized both preserved failures while keeping their raw
  rejection intact. Tests cover missing sensors, primary/global errors,
  terrain disagreement, both edges, bounds, collisions and finite retry limits.
- The accepted batch has **200 frames**, **40 backgrounds**, zero validation
  errors and **200/200** sensor-depth box checks agreeing. It retains 1,037
  geometry-only object appearances beyond usable depth among 2,780 checked
  object appearances.
- All **4,550 preservation hashes** covering the previous **1,120 accepted
  frames** and their scene-plan evidence still match.
- The bounded run's guard returned exit 0 with no failure and peak Tctl
  **76.0 C**. All **17 retained batch-6 cleanup records**, including earlier
  attempts, have zero surviving processes. No workload container remained.

The original camera application, geometry evaluator, acceptance policy,
qualified Ogre/Mesa image, generated scenes and scene reservations remain
unchanged. The desktop-startup fix is retained. Candidate source and installed
module copies are separately pinned; the runtime extends the original 786-file
application to 789 pinned files.

| Identity | Value |
| --- | --- |
| Candidate | `bulk10000-capture-recovery-20261106/application-v4` |
| Runtime manifest SHA-256 | `e1f30c2652891f8329a0fb40b9303fc96726d01b2efe146ab6ee50cc17bc209a` |
| Source patch archive SHA-256 | `9bc5cb85938a6f8ac8eb519deaeddfb6a6373e485690ea59744ea0f8befc8bb3` |
| Reproducible controller archive SHA-256 | `e5960478b0422b536fa35718a1fba3cc6932995120b9c7fe7ee92e70b37f22e2` |
| Image | `sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa` |
| Reservation SHA-256 | `737554d0383ebe3eae574afb55f6c4e983b4e17283e2377e4312355e941c5558` |

Earlier v1–v3 candidates and the previous controller metadata are preserved for
review/rollback. The separate original pilot controller was not changed.

## Continue collection

The current native capture and temperature monitoring were kept running for
continuation. No protected finish was performed during this fix. Keep the Halo
desktop/RDP connected and temperatures visible, with NPU, terrain and manual
simulation off. In the desktop terminal:

```bash
python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-collection-20261106/start_bulk10000_collection.py --run
```

This resumes at **batch 7**. Monitoring retains its two-hour duration, and
another quality failure remains a stop to investigate. If monitoring has ended,
use the fresh-session procedure in the
[collection guide](FULL10000_COLLECTION_GUIDE.md). This fix qualifies the
observed failure and completed batch; it does not establish that all remaining
scenes will pass or that the unreviewed dataset is ready for training.

## Evidence

Final independent audit (`navigation_sessions/bulk-depth-edge-fix-20261107/FINAL_REVIEW_V4.json`, local-only),
Linux tests (`navigation_sessions/bulk-depth-edge-fix-20261107/LINUX_ENVIRONMENT_TESTS-v4.txt`, local-only),
preservation snapshot (`navigation_sessions/bulk-depth-edge-fix-20261107/PRESERVATION_BEFORE_V4.json`, local-only),
offline proposals (`navigation_sessions/bulk-depth-edge-fix-20261107/OFFLINE_RECOVERY_PROPOSALS.json`, local-only),
accepted annotated frame (`navigation_sessions/bulk-depth-edge-fix-20261107/accepted_scene4320092_view000.png`, local-only),
archive receipt (`navigation_sessions/bulk-depth-edge-fix-20261107/ARCHIVE_RECEIPT.json`, local-only),
and off-host verification (`navigation_sessions/bulk-depth-edge-fix-20261107/OFFHOST_VERIFICATION.json`, local-only).

The archive contains canonical batch-6 images/labels/previews, all rejected
diagnostics, scene-bound renderer/cleanup proofs, source/test identities and
the retained controller release. Native monitoring status is an active-session
snapshot, not a protected final-teardown certificate.
