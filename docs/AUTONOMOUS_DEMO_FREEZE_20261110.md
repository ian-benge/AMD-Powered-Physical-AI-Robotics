# Autonomous rehearsal freeze investigation — November 10, 2026

Halo became unresponsive during the first autonomous return. The surviving
records establish an abrupt interruption followed by an unclean shutdown.
**The initiating cause remains undetermined.** No new hardware workload was
started during this investigation.

## What the records establish

| Observation | Evidence and limits |
|---|---|
| Interrupted boot | `9d20c7d3-3bc5-4e0d-90a4-d1e4ce458438`; native case `oem617-7141-rocm-manual-20261110t022009z-e40c-20261110T022010Z-lg8s9fsn` |
| Candidate and mission | `application-v11` / `runtime-v11`; scene session `restart-20261110T033013Z-f79cc0`; mission `7f9db898deea5bf45add96210c85b659` |
| Autonomous behavior | Initial sweep completed, one distinct frontier visit recorded, then automatic `exploration_return` with reason `No feasible budgeted frontier` |
| Last persisted mission status | Elapsed 141.83 s; estimated travel 3.443 m; measured ground 93.97 m² (5.87% of 1,600 m²); no sample record |
| Temperature | Rehearsal supervisor peak 85.125°C, including its final reading at 03:34:05 UTC; separate temperature monitor peak during this scene 84.875°C. Full preceding capture monitor peak was 85.375°C. No recorded 87°C qualification stop or native 90°C trip |
| Memory | Last native snapshot at 03:34:03 UTC: 48.75 GiB available; all 8 GiB swap free |
| Mapping and mission inputs | Worst rolling publication p95 observed during active mission: 177.97 ms; zero unhealthy mission-input snapshots in the saved active observer records |
| Guard | Last saved guard observation: autonomous, ready, no fault; command `[0.0, -0.0487643]`. Final zero-command/shutdown evidence is absent |
| Last saved monitoring | Native capture health 03:34:03 UTC; temperature monitor 03:34:04 UTC; supervisor 03:34:05 UTC. These are persistence times, not a proven exact freeze time |
| Restart | New boot `c43492b6-8b91-4d2f-863b-638e7c5c4d62`, starting November 10 at 21:05:50 UTC (15:05:50 CST) |

Times around the failure are November 10 UTC / November 9 CST. Native capture,
temperature and supervisor files agree that useful records stopped around
03:34 UTC (21:34 CST). The earlier chat observer saw only part of this interval;
recovered on-disk status provides the slightly greater distance/area above.
Measured coverage remains an estimator result, not an independent ground-truth
acceptance measurement.

The reboot journal reports `system.journal corrupted or uncleanly shut down`.
Several application JSONL/control logs have zero-filled tails, and the
synchronized RGB-D review contains incomplete files. Preserve those bytes;
analysis counts malformed records and does not repair the original evidence.
There is no `POST_TEARDOWN.json` or `MANUAL_SESSION_FINAL.json` for this case.
The original monitoring/shutdown gap remains unresolved.

## Cause assessment

The operator exported the failed boot's protected kernel journal, the complete
03:20–03:45 UTC system window, and the recovery boot's kernel journal. Neither
the saved native journal stream nor the protected exports contains a GPU
timeout/reset, kernel panic/lockup, out-of-memory kill, storage error, or thermal
shutdown in the failure window. The failed kernel journal's final message is
an ordinary performance-sampling adjustment at 03:03:12 UTC; the system journal
ends at 03:33:34 UTC with Tailscale control messages.

`/sys/fs/pstore`, `/var/lib/systemd/pstore` and `/var/lib/systemd/coredump` are
empty. The three Apport desktop crash files are dated November 3 and do not
identify this incident.

This evidence does **not** distinguish a hard kernel/GPU/firmware lockup from a
power interruption or another platform failure. Unflushed final messages may
have been lost. Temperature and memory readings provide no recorded explanation
for the freeze; they cannot exclude an unrecorded event after the last sample.
Earlier boot-time DisplayPort errors and known vendor-module taint messages
occurred well before this rehearsal and are not assigned as its cause.

The policy's decision to return when no budgeted frontier remained is recorded
mission behavior. Its appearance before the freeze does not establish causation.
No successful home arrival, sample collection, delivery or accepted autonomous
cycle was demonstrated. The 0.05 m/s rehearsal failed; 0.10 m/s remains unqualified.

## Remote desktop recovery

The separate post-reboot RDP connection problem was identified and corrected:

- GNOME Desktop Sharing was healthy and listening on Halo port **3389**.
- The Windows SSH tunnel forwarded `127.0.0.1:13390` to Halo port **3390**.
- Direct RDP negotiation succeeded on 3389; negotiation through the incorrect
  tunnel was reset.
- Replaced that identified tunnel with a hidden SSH forward to **3389**.
  Negotiation then succeeded through `127.0.0.1:13390`; the operator confirmed
  the desktop opened.

No host desktop/driver settings or credentials were changed. This port mismatch
explains the current viewing failure and does not explain the earlier host freeze.
Use this Windows PowerShell command if the corrected tunnel needs reopening:

```powershell
ssh -N -T -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -L 127.0.0.1:13390:127.0.0.1:3389 halo
```

Connect Windows Remote Desktop to `127.0.0.1:13390`. A tunnel already occupying
that local port should be inspected before starting another copy.

## Preserved evidence and next step

Private remote investigation directory:
`navigation_sessions/autonomous-demo-20261109/freeze-investigation-20261110/`.
The separate analyzer writes `analysis-v1/FINDINGS.json`, a readings CSV and
`FREEZE_EVIDENCE_V1.zip`; it never writes the failed session's original files.
The raw archive includes the incomplete diagnostic captures and both independent
observers, native health/temperature/journal records, mission/guard events,
launch logs, release identities and the operator-exported protected logs.
The RTAB-Map database remains preserved on Halo and is outside this archive.

The 31,107,553-byte archive was downloaded into the corresponding private local
directory. Its SHA-256 and **all 114 archived file hashes** were independently
verified on Windows:

```text
032d349b39e143a83829adc762f933ace30c1555bec50a2eb8e9562b258c050d
```

Source v11 SHA-256 remains:
`ea24f6246dfa7089e519ec644b9015acbe589ab91c75b5e2cc3bfa3a4eb570d7`.
Local tunnel repair identity/negotiation evidence is saved alongside this archive.

Keep this failed run separate from accepted platform/contact/mapping evidence.
Before another autonomous attempt, establish durable off-host failure logging,
review the missing original teardown under the current boot, and isolate the
platform workloads in short attended checks. A new bounded incident review is
needed; the older reboot-recovery helper is pinned to a different predecessor
case and boot chain and must not be reused for this freeze. No driver, firmware,
fan/power policy or hazard threshold change was made as part of this investigation.
