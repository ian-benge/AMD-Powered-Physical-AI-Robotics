# Capture recovery at the usable-depth boundary

November 8, 2026 UTC. The v10 recovery candidate is installed after a passing
20-view rendered regression of the scene that stopped batch 11. The campaign's
**2,000 accepted images in ten batches** and all saved failures are preserved.
This repair does not change image acceptance thresholds or approve training.

## Cause

All three attempts of `scene_4320160` stopped at `view_015`. The primary target,
`veined_lookalike_000`, passed its framing and sensor checks. An incidental
`rock_009` crossed the 8 m usable-depth limit: 39 visible pixels were beyond
usable depth and nine were just inside it, at 7.987-7.995 m. Only three to five
of those nine had matching measured depth; the remainder were far-clipped.
Depth support and box verification therefore rejected the frame correctly.

The existing recovery required 90% valid depth coverage for every failing
incidental object. It also refused a depth-box extent mismatch. This particular
cutoff sliver could not enter its alternative-view search, so restarting the
scene repeated the same rejection. The continuous supervisor stopped on the
quality failure and completed its monitored cooldown and protected closeout.

## Repair

The new proof permits a different pose only when a mesh crosses the axial
far-depth boundary, all missing usable samples are far-clipped within a band
of at most 30 mm, and the remaining usable samples agree within 2 mm. The band
cannot exceed the existing depth-support tolerance. A valid primary target,
at least 95% global depth coverage, at least 98% global support, median error
at most 2 mm, p95 at most 10 mm, and at most 2% affected usable pixels are still
required. NaNs, near/zero dropouts, missing sensors, primary-target errors,
class errors and global disagreements remain stops.

The existing finite search remains: at most eight checked alternatives, with
collision, range, map-boundary and slope checks. Every replacement receives
fresh pose feedback and RGB-D, then passes all original export checks. The
failed frame remains rejected. Geometry labels, RGB-only labels beyond 8 m,
legacy class IDs, four-class export metadata and reserved splits are retained.

Only three application pins changed from v9: `capture_recovery.py` in source
and installed form, and its source test. The Ogre image/plugin, annotation,
box-verification, quality, cleanup, robot and native monitoring code remain
pinned. The collector and continuous supervisor programs are unchanged;
their manifests now identify the checked v10 runtime.

## Verification

- **240 Windows environment tests passed**, with five Linux process-test skips.
- The pinned candidate passed **223 Linux environment tests**, **33 collector
  tests** and **28 continuous-supervisor tests**.
- Saved-frame replay confirms that v9 refuses the case and v10 proposes a new
  pose without accepting the rejected frame.
- The copied scene completed **20/20 rendered views** with zero validation
  errors and measured-depth box agreement for all frames. These diagnostic
  images are excluded from the 10,000-image campaign.
- The original failure was reproduced during that diagnostic. Its first
  alternative, a **0.035 rad turn**, passed. `rock_009` retained its full RGB
  label and measured support rose to **38/42 pixels (90.48%)**, exceeding the
  unchanged 90% requirement. All 25 labels in that frame passed.
- Mapped hardware renderer provenance and cleanup passed, with no remaining
  owned processes. Peak guarded diagnostic Tctl was **74.125 C**.
- Before deployment, all ten completed batch receipts and **35,342 preserved
  files** were verified, including rejected attempts. Their hashes were checked
  again after deployment; scene reservations and splits were unchanged.

A diagnostic worker staging-path failure occurred before graphics started;
its partial tree is retained. An isolated-mode invocation of the legacy
collector's static check also failed before workloads; the normal invocation
passed. Neither required a change to the approved source payload or quality
policy. These are retained separately from the successful diagnostic.

The diagnostic session closed with `MANUAL_SESSION_CLOSED_CLEANLY` at
01:24:30 UTC. Temperatures and capture extended past the final journal capture;
the last cooldown minute has 30 temperature samples with a maximum 2.022 s gap.
The [closure review](../evidence/capture-depth-boundary-20261108/PROTECTED_CLOSURE_REVIEW.json)
records this bounded closeout without an endurance claim.

The continuous campaign then resumed with v10. The originally blocking scene
committed all 20 frames on `attempt-0004`, retaining the three failed attempts,
and collection proceeded to the next scene. This is a dated startup/blocker
verification, not a completed-batch or completed-dataset claim. See the
[live blocker review](../evidence/capture-depth-boundary-20261108/LIVE_BLOCKER_PASSED.json).

## Resume

The installed Halo supervisor uses v10 on its next launch. Close an existing
manual monitoring case with `~/bin/halo-rover finish`, keeping capture and
temperatures running through cooldown and final journal capture. Then stop
those manual monitors and start from the Halo graphical desktop:

```bash
sudo /usr/bin/python3 -I /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-continuous-20261107/start_bulk10000_continuous.py --run
```

The supervisor manages its own monitoring. It resumes saved campaign progress
and preserves the previous three rejected scene attempts. Keep the desktop
connected and terminal visible. Real hardware, monitoring and image-quality
faults still stop collection; this bounded regression does not guarantee that
every unseen scene will pass. See the [continuous guide](BULK_CONTINUOUS_COLLECTION_GUIDE.md).

Local evidence is under `navigation_sessions/bulk-depth-boundary-fix-20261108/`.
The [published review](../evidence/capture-depth-boundary-20261108/FINAL_REVIEW_V10.json)
and [deployment receipt](../evidence/capture-depth-boundary-20261108/DEPLOYMENT_V10.json)
record exact source, application and manifest identities. Raw diagnostic
captures, prior failure arrays and the full evidence archive remain external.

The **43,762,522-byte** archive was copied off Halo and verified against all
**305 member hashes**. SHA-256:
`8c1ed4592447760ab6c8f1169be95a8469ff45598e54e5007df25864a5e74b5e`.
It includes complete diagnostic outputs, original failed-frame arrays, source,
tests, preservation records and protected closeout. Large copied scene bundles
remain on Halo, identified by their saved hashes. The new active collection's
growing logs remain on Halo and are not given a final-teardown claim.
[Off-host verification](../evidence/capture-depth-boundary-20261108/OFFHOST_VERIFICATION_V10.json).
