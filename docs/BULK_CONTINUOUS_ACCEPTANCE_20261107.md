# Automatic collection continuation — November 7, 2026

The separate continuous supervisor is installed and running batch 10. It
automatically closes and renews native monitoring windows until the existing
10,000-image campaign completes, or a fault/cancellation requires review.
At the live-start audit, nine batches and **1,800 accepted images** were verified.
All 1,800 prior images, receipt identities and reserved splits were preserved.

The operator launches the supervisor once with local sudo. Only the existing
protected capture, module-check and final-journal helpers use root privileges.
Preparation, temperature logging and the unchanged collector run as `ian`.
Live inspection confirmed the collector's effective UID was 1001 and the Docker
capture container retained `1001:1001`, the qualified image and application v9.
No persistent service or sudoers rule was installed.

## Verified behavior

The supervisor keeps the native 7,200-second capture duration. At a clean
collector `SESSION_RENEWAL_REQUIRED` pause, it requires newly completed images,
checks that workloads stopped, keeps monitoring fresh through 60 seconds of
cooldown, runs the original protected final-journal helper, verifies clean
teardown, stops its old monitors and prepares a fresh kit/case. The collector
then verifies saved progress and resumes automatically.

The final window also closes automatically. Full completion includes another
independent verification of all saved batch receipts and output hashes.
There is a finite limit of 50 windows, with forward progress required before
any automatic renewal. The default accommodates the 41 remaining batches.

Ctrl+C, a failed collector, a stale/missing monitor, changed pinned inputs,
thermal/fan/kernel/GPU incidents, renderer or image-quality failures, inconsistent
progress and failed teardown stop automatic continuation. The existing native
guards, capture application, renderer, labels and dataset thresholds are unchanged.
Missing final-journal capture leaves evidence monitoring open for local review.

## Qualification

**61 Linux controller tests passed.** These include the new supervisor tests
and both existing collector/starter suites. Tests cover renewal ordering,
cancellation, failed or zero-progress exits, bounded iteration, stale monitoring,
incident retention, teardown failure, source pins, user credentials and actual
Linux command resolution. The first protected check exposed a missing
`/usr/sbin` in the sanitized user PATH. Its failed attempt was preserved and
closed cleanly; the corrected PATH and a native-tool regression test passed.

Two real protected monitor cycles then passed without a renderer or training
image capture:

| Cycle | Cooldown | Temperature samples | Maximum sample gap | Peak Tctl | Teardown |
| --- | ---: | ---: | ---: | ---: | --- |
| 1 | 60.047 s | 32 | 2.021 s | 52.5 °C | Clean |
| 2 | 60.048 s | 32 | 2.108 s | 29.5 °C | Clean |

Independent review checked native health, temperature timestamps, protected
journal results and monitor-stop timestamps. Both cycles kept monitors through
cooldown and final journal capture, recorded no monitoring gap, and stopped
capture after user finalization. The compound operator command then started
normal collection automatically. Batch 10's active Docker job and user
credentials were independently inspected.

Qualification run: `bulk10000-continuous-20261107T225141Z-lpdysrvu`.
Live collection run: `bulk10000-continuous-20261107T225409Z-mvhu5r79`.
The current active capture is intentionally left running for collection; it
does not have a final-teardown claim in this report.

This verifies automatic lifecycle handling and live startup. It does not certify
unattended endurance stability, guarantee every future scene passes its quality
checks, or approve detector training. Image review and four-class export remain
required after collection.

## Identities and evidence

| Identity | SHA-256 |
| --- | --- |
| Supervisor source | `81398bf87b716ff5c43a97aa8021a7f5b9bbd36ac9b2dc4bbb03585ae478b573` |
| Source bundle | `93e412b2712820f0b3ea1c00538ed97b4a4523e081b74b98f88341ce685dcfc6` |
| Continuous input manifest | `24edbb1eb8840917d15dfd880f1121b208c2748b99617c7818e9b1bac5f3118e` |
| Qualified Docker image | `0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa` |
| Evidence archive | `ecc5b3f21461b5491531b75956c9391e1af564770e7b6403a67c1def308ea62e` |

The 968,558-byte evidence archive was downloaded and independently verified
off-host: all 119 member hashes matched, including the supervisor's three files
and 15 unchanged native/controller inputs. It includes both successful cycles,
the first rejected startup and its clean closeout, and immutable live-start
identities. Growing logs from the active collection remain on Halo.

[Linux tests](../evidence/development-20261107/bulk10000-continuous-20261107/LINUX_CONTROLLER_TESTS.txt),
[independent rotation/live-start audit](../evidence/development-20261107/bulk10000-continuous-20261107/AUTOMATIC_ROTATION_REVIEW.json),
[prior image verification](../evidence/development-20261107/bulk10000-continuous-20261107/PRIOR_CAMPAIGN_VERIFICATION.json),
[archive receipt](../evidence/development-20261107/bulk10000-continuous-20261107/ARCHIVE_RECEIPT.json),
and [off-host verification](../evidence/development-20261107/bulk10000-continuous-20261107/OFFHOST_VERIFICATION.json).

Use the [continuous collection guide](BULK_CONTINUOUS_COLLECTION_GUIDE.md) for
start, status and cancellation commands. Leave the current command running,
with its terminal visible and the graphical desktop/RDP connection active.
