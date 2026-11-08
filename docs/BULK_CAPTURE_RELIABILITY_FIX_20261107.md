# Bulk capture reliability fix — November 7, 2026

The separate capture v9 candidate fixes repeated framing and incidental-depth
rejections, stale pose feedback, a transient session-inventory race and
monitoring-budget interruptions. The collection has **1,600 accepted images in
eight completed batches**, with **8,400 images remaining**. Training remains
unapproved and has not started.

The original camera application, renderer, generated scenes, 500 reserved scene
groups, split assignments and acceptance thresholds are retained. Before final
deployment, all 1,600 accepted frames in 80 scenes were preserved through 6,498
file hashes. Resume repeats verification before reuse.

## Problems and corrections

`scene_4320104/view_011` planned exactly 70% primary-target visibility. Three
rendered attempts measured 69.84%, 69.81% and 69.69% after settling. The original
checks correctly rejected those frames. The new preflight searches for a safer
view of the same target, preferring four additional percentage points of
visibility, two training-size pixels and four border pixels. The actual export
floors remain 70% for ordinary views and 30% for difficult views.

The geometry search examines at most 33 poses, with two settling probes per
pose, and permits at most three framing adjustments. It preserves the target,
export schedule and difficult-view offset, while checking map bounds, slopes,
footprint, lens clearance and configured target distance. Fresh measured RGB-D
must still pass all original checks; predicted geometry never authorizes export.

Some incidental objects failed depth support despite excellent global depth
agreement. The new recovery evaluates their pixel footprint independently:

- At least 10,000 expected usable pixels and 95% valid measured-depth coverage.
- At least 98% global support, median error at most 2 mm and p95 at most 10 mm.
- Failing incidental objects occupy at most 2% of usable pixels; primary-target
  sensor support must pass.
- At most eight checked alternatives: two small turns followed by six orbits
  around the same target. Unsafe candidates are skipped and the finite search
  indices are recorded.

These conditions authorize another camera pose only. Every exported object
still requires the original 90% measured-depth support and 30 mm tolerance.
Rejected images stay rejected and their diagnostics remain preserved. During
an existing search, a new alternative with both localized depth rejection and
insufficient primary framing can continue to the next bounded candidate.
Missing targets, class errors, missing sensors and global failures still stop.

For the saved `scene_4320107/view_013` failure, three incidental objects occupied
0.326% of usable pixels. Global support was 99.907%, with median error 15 µm.
The accepted replacement retained all three objects and achieved sensor support
of 100%, 98.55% and 95.72%, without changing their labels.

Capture now drains queued sensor/odometry callbacks before setting each pose,
records a timestamp barrier, and requires newer odometry before starting the
existing settling interval. Accepted metadata records the barrier, arrival,
settling deadline and actual interpolated camera pose. The newest 60 campaign
frames passed the fresh-feedback timing audit. Earlier accepted labels remain
bound to their recorded actual pose and validated measured depth.

A native guard could also fail when an ephemeral SSH login session disappeared
between `loginctl list-sessions` and `show-session`. The separate pinned guard
runner retries the complete native check at most three times, catching only
that exact vanished-session assertion. Real desktop loss, stale inputs,
thermal limits, module changes and other errors still stop. The native kit and
its guard functions are unchanged. The runner survived twelve transient SSH
connections and handled one such race during a successful full batch.

## Progress and monitoring

`--status` now reports partial committed images, completed scenes and the active
attempt's saved views. Campaign totals continue to count only completed,
validated batches.

Before another batch starts, the collector budgets its work against the native
capture duration, recent measured timings and a safety margin. A partially
completed bundle receives a reduced budget only after its plans, completed
reports and evidence hashes are verified. Insufficient time produces
`SESSION_RENEWAL_REQUIRED` without starting another workload. The native
two-hour monitoring limit remains enforced.

Keep the graphical desktop/RDP connection active, with capture and temperatures
running and temperatures visible. Leave NPU, terrain and manual simulation off.
From a Halo desktop terminal:

```bash
python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-collection-20261106/start_bulk10000_collection.py --run
```

In another terminal:

```bash
watch -n 5 python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-collection-20261106/start_bulk10000_collection.py --status
```

On `SESSION_RENEWAL_REQUIRED`, run `~/bin/halo-rover finish`, keep both monitors
through the cooldown and final journal capture, then close them. Start fresh
capture and temperature terminals, wait for two capture health events and
updating temperatures, and repeat `--run`. Genuine hardware or quality failures
remain stops to investigate. This release cannot guarantee fault-free capture
of scenes that have not yet been rendered.

## Verification and delivery

**219 Linux environment tests and 33 Linux controller tests passed.** They cover
bounded recovery, settling sensitivity, collision/range/boundary constraints,
legacy behavior, retained class identities, rejected sensor/global failures,
fresh pose feedback, partial progress, session budgeting, login-session races
and cleanup identity checks.

Batches 7 and 8 completed with **400 accepted frames**, including 80 backgrounds,
zero validation errors and measured-depth box agreement for all 400 frames.
The successful guards and renderer cleanup records are retained. The earlier
failed invocations and rejected attempts are preserved separately. The two
recovered preview fixtures and the final diagnostic's recovered view were also
visually reviewed. Peak guarded temperature was 85.125 °C across the reviewed
batches and 83.0 °C in the final diagnostic; no native guard breach was recorded.
Batch 7 completed with v6 and batch 8 with v8. The additional v9 continuation
logic was qualified on the separate copied scene described below.

The final v9 diagnostic passed all 20 views of a copied scene. Its first scene
attempt exhausted eight alternatives and was correctly preserved as rejected;
the second completed with all original checks passing. The rendered evidence
also exercises continuation after a combined framing/depth rejection. This
remains a bounded recovery system, with three scene attempts per invocation.
The diagnostic's timing audit, final preservation hashes, exact identities and
off-host archive verification are recorded below. These diagnostic images are
excluded from the 10,000-image count.

The 671,430,055-byte evidence archive was downloaded and independently verified
off-host: all 4,583 member hashes matched, including the 400 canonical batch
images, labels and metadata. All ten patched workspace source files and the
local/deployed controller pins also matched. Archive SHA-256:
`89f8fd0ea2efb28e854f7888cdec2949a774de40c7e24f6a002404817afd22e3`.

| Identity | Value |
| --- | --- |
| Candidate | `bulk10000-capture-recovery-20261106/application-v9` |
| Pinned application files | 790 |
| Pinned controller files | 12 |
| Runtime manifest SHA-256 | `0ddc865b17e6949a9b70b16ee999d516193006787744d235090a50a9ea3512e6` |
| Controller manifest SHA-256 | `e9992a6e9ed7f9173d40a0de8c62a04dc3e6d44b11680ffb3c8af1777c4e21da` |
| Controller archive SHA-256 | `85bb5a6931669761898aa4f1e3c982ae91642072403097e287ba597e8c6b6f11` |
| Source patch archive SHA-256 | `df3b4391829eedd5155348dffff54d43a9b5f6ab5c13713b85195dd24dfc9a80` |
| Image | `sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa` |
| Reservation SHA-256 | `737554d0383ebe3eae574afb55f6c4e983b4e17283e2377e4312355e941c5558` |

Earlier candidates, controllers and accepted evidence are retained. The latest
monitoring session is left open for collection continuation; this report makes
no protected-finish claim for that active session. Previous closed sessions
retain their protected finish records.

[Final review](../evidence/development-20261107/bulk-framing-reliability-fix-20261107/FINAL_REVIEW_V9.json),
[deployment identities](../evidence/development-20261107/bulk-framing-reliability-fix-20261107/DEPLOYMENT_V9.json),
preservation hashes (`navigation_sessions/bulk-framing-reliability-fix-20261107/PRESERVATION_BEFORE_V9.json`, local-only),
[environment tests](../evidence/development-20261107/bulk-framing-reliability-fix-20261107/LINUX_ENVIRONMENT_TESTS-v9.txt),
[controller tests](../evidence/development-20261107/bulk-framing-reliability-fix-20261107/LINUX_CONTROLLER_TESTS-v9.txt),
[archive receipt](../evidence/development-20261107/bulk-framing-reliability-fix-20261107/ARCHIVE_RECEIPT.json),
and [off-host verification](../evidence/development-20261107/bulk-framing-reliability-fix-20261107/OFFHOST_VERIFICATION.json).

The campaign remains pending human review and metadata-based four-class export.
Collection completion does not automatically approve or start detector training.
