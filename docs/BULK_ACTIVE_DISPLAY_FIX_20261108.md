# Capture desktop-output guard

November 8, 2026 UTC. The campaign stopped with **4,200 accepted images in 21
completed batches**. The next scene was `scene_4320343` in batch 22; its first
attempt saved no images. This is a dated snapshot, separate from the earlier
[v10 usable-depth repair](BULK_DEPTH_BOUNDARY_FIX_20261108.md).

## Observed stop

The desktop journal records an RDP transport failure and session disconnect at
07:36:45 UTC, followed by removal of virtual monitor `Meta-0`. Scene generation
was still running. Gazebo launched the next scene at about 08:00:26 UTC and
crashed at about 08:00:31 UTC while loading Ogre's OpenGL plugin:
`RenderSystem_GL.so`, `dllStartPlugin`, `Ogre::Root::installPlugin` and
`Ogre::GLRenderSystem::GLRenderSystem`. The process reported a null-address
segmentation fault at `0x4` and exit 139. Missing renderer provenance and ROS
shutdown errors followed the crash.

The timing supports desktop-output removal as a likely trigger, matching the
[November 6 interruption](BULK_RENDERER_CRASH_RECOVERY_20261106.md). It does not
prove the native OpenGL defect. A nearby Mesa file-description warning alone
does not establish causation.

Four automatic monitoring renewals completed before this failure. The fifth
window retained capture and temperatures through cooldown and final journal
capture, with no recorded monitoring gap. Its protected teardown reports
`post_teardown_clean: true` and no matches in its kernel-incident screen. The
supervisor correctly stopped rather than retrying the renderer crash.

## Controller repair

An X11 connection and Xwayland PID are insufficient when the remote desktop has
removed its virtual output. The new controller requires a positive active-monitor
count from RandR 1.5 or later. The query uses the existing X11/RandR libraries in
a separate child with a five-second timeout; it creates no GL context and changes
no display settings. This follows the upstream
[active-monitor query interface](https://sources.debian.org/src/libxrandr/2%3A1.5.1-1/src/XrrMonitor.c/).

Checks run before planning, between layouts, after planning and immediately
before each scene launch. During capture a separate output watcher stays alive
after renderer provenance passes. Output loss sends SIGINT only to the held ROS
launch process; existing ownership-checked cleanup handles its children. The
failed attempt is preserved and the fatal-attempt path prevents three blind
renderer launches. The collector and supervisor report
`DESKTOP_RECONNECT_REQUIRED`, then finish protected monitoring before exiting.

Stop evidence is immutable and bound to the exact invocation. An old stop cannot
misidentify a new invocation. Verified resume keeps the original reservations,
generated scene hashes, failed outputs and accepted batch receipts.

The change retains capture application v10, the qualified image/Ogre plugin,
geometry, camera, depth/label/framing acceptance, native capture duration,
temperature and fan policy, and protected journal checks. It adds no software
rendering fallback or automatic retry after a real renderer failure. A disconnect
can race a launch; these checks do not prove an underlying driver crash impossible.

## Verification and rollout

The controller source suite covers a reachable X11 connection with zero outputs,
old RandR, failed and timed-out queries, malformed evidence, a lost output during
a launch, immutable stop records, one preserved prelaunch attempt, and monitored
closeout without automatic retry. Rendered verification uses a separate copy of
the stopped scene; diagnostic images never enter the campaign.

The installed controller passed 719 local source tests (12 expected platform or
hardware skips), all four browser control programs, and 72 Linux controller and
supervisor tests (one expected native-kit skip). The first Linux check exposed a
test-only import portability issue; that failed check and its release were
retained before correcting the test.

A separate copy of `scene_4320343` produced 20 diagnostic views. All 20 passed
the existing sensor/depth-box checks, qualified hardware-Ogre provenance passed,
and owned-process cleanup reported no survivors. The live probe reported one
active monitor on `:0`, with RandR 1.6. A real RDP disconnect was not injected
during attended rendering; output-loss cases were tested in isolation.

Deployment independently verified all 21 completed batch receipts and hashed
**73,173 preserved campaign files** before and after installation. All 4,200
accepted images, saved scene reservations, generated bundles and failed attempts
were unchanged. The production campaign was not restarted automatically.

The diagnostic session's protected finish completed at 17:14:29 UTC with
`MANUAL_SESSION_CLOSED_CLEANLY`, `post_teardown_clean: true` and no kernel-incident
screen matches. The native timed-hour result remains incomplete; this bounded
capture check establishes no endurance claim.

Installed identities:

| Artifact | SHA-256 |
| --- | --- |
| Approved corrected source archive | `ba0f7775f5fed0166f168b64de75ebf9c1142fcd27744fcbd70189b7514d5233` |
| Unchanged capture-runtime manifest (application v10) | `6f856f0220c7f1cf1056eaaf1e9c139e0324d1a5c43bfb4aabd57a789ffd573f` |
| Controller release manifest | `9470641946817ac7e93d25bc43b41794cda416371be2a1e403cba95480516d65` |
| Continuous release manifest | `172cf5c00b1899b35595a43e67f6aaf41ec11f399c81976d62fbdbe60c5841c0` |

Full private verification outputs remain under
`navigation_sessions/bulk-active-display-fix-20261108/` on Halo. The approved
failed-launch, guard-result, protected-teardown and desktop-journal downloads are
also retained in the local repair directory. Generated diagnostic images and
private logs are omitted from Git. The operator starts the campaign using the
unchanged command in the
[continuous collection guide](BULK_CONTINUOUS_COLLECTION_GUIDE.md), keeping the
graphical desktop connected and the supervisor visible.
