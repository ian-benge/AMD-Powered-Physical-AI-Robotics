# Usable-depth cutoff recovery evidence

These selected records support the [November 8 repair report](../../docs/BULK_DEPTH_BOUNDARY_FIX_20261108.md).
The [manifest](manifest.json) binds original local source hashes to published
copies, with any CRLF-to-LF normalization recorded. Raw captures, failed-frame
arrays, runtime bundles and the full archive are not distributed here.

- [Rendered review](FINAL_REVIEW_V10.json): copied failing scene, 20 accepted
  diagnostic views, original rejection reproduced, bounded pose recovery,
  unchanged label/depth checks and renderer cleanup.
- [Offline replay](OFFLINE_REPLAY.json): v9 refuses the saved case; v10 proposes
  a fresh view while the original remains rejected.
- [Deployment](DEPLOYMENT_V10.json): 2,000 accepted campaign images and 35,342
  files verified unchanged; only recovery code/test pins and runtime manifests
  change. [Source payload identity](SOURCE_RELEASE_V10.json).
- [Windows environment tests](WINDOWS_ENVIRONMENT_TESTS-v10.txt): 240 cases,
  five Linux process-test skips. The separately pinned Halo candidate passed
  [223 Linux environment cases](LINUX_ENVIRONMENT_TESTS-v10.txt),
  [33 collector cases](LINUX_CONTROLLER_TESTS-v10.txt) and
  [28 supervisor cases](LINUX_CONTINUOUS_TESTS-v10.txt).
- [Protected diagnostic-session closure](PROTECTED_CLOSURE_REVIEW.json): clean
  manual closeout, monitors retained through cooldown and final journal capture.
- [Live blocker passed](LIVE_BLOCKER_PASSED.json): the campaign resumed on v10
  and committed the formerly blocking scene. Its growing monitoring session is
  intentionally left running; this snapshot does not declare it finished.
- [Archive receipt](ARCHIVE_RECEIPT_V10.json) and
  [off-host verification](OFFHOST_VERIFICATION_V10.json): 43,762,522 bytes,
  all 305 member hashes independently checked.

Diagnostic images are excluded from campaign totals. Collection, human review,
four-class export, detector training and endurance remain separate outcomes.
