# Halo outage investigation - October 27, 2026

The October 27 outage is more consistent with a system-wide stall than a
Wi-Fi-only disconnection. This is an inference from independent recordings,
not a confirmed kernel panic or an identified driver defect. The operator
confirmed restarting the Halo at approximately 15:27 Central, rather than
leaving it powered off throughout the preceding gap.

## Correlated timeline

All times below are Central daylight time on October 27.

| Time | Evidence | Interpretation |
| --- | --- | --- |
| 15:14:08 | An operator command assigned Ethernet `eno1` a local static address; Tailscale rebound and reported a successful relay check | Network reconfiguration occurred; Wi-Fi remained the default route |
| 15:16:23-24 | NetworkManager changed from `CONNECTED_SITE` to `CONNECTED_GLOBAL` within one second | A brief connectivity-state change, followed by recovery; no sustained Wi-Fi disconnect is recorded here |
| 15:18:14-15 | SSH accepted a connection, then recorded a client-requested disconnect | Remote access was still functioning; the individual SSH logout is not a desktop logout |
| 15:18:42-45 | Keyboard USB reconnection, followed by the last saved system-journal entries | No recorded panic, suspend or Wi-Fi driver failure explains the cutoff |
| About 15:18:47 | Mission log, object-tracking log, simulator log and SLAM database modification times all stop | Several local processes ceased leaving saved evidence in the same interval |
| 15:20 | Expected ten-minute sysstat sample absent | Consistent with a wider stall; the activity collector runs independently of remote access |
| 15:27:47-49 | New boot begins; sysstat records Linux restart | Consistent with the operator's reported reboot time |
| 15:30:06-12 | GNOME Shell SIGSEGV crash report and user-shell service `core-dump` failure | A confirmed desktop crash after reboot, separate from the original outage |

The last valid simulator text precedes the file modification time by a few
seconds; several application logs have incomplete or zero-filled tails. Their
modification times are approximate cutoff evidence, not an exact crash clock.
The files were copied without repairing or truncating them.

## Findings and limits

- A network-only interruption would ordinarily allow local mission, SLAM and
  system activity recording to continue. Their shared cutoff and the gap before
  the reported reboot favor a machine stall over that explanation.
- No kernel panic, relevant Wi-Fi firmware failure, GPU reset, out-of-memory
  kill or suspend transition was saved near the cutoff. Absence of such a
  message does not rule out a kernel, device or storage stall that prevented
  logs from being written. `/sys/fs/pstore` was empty.
- The last pre-outage resource sample, at 15:10:06, showed about 76% aggregate
  CPU idle and about 51 GiB available memory. That sample is several minutes
  older than the outage and cannot rule out a later resource problem.
- Wi-Fi power saving remained disabled (`2`), autoconnect was enabled, and the
  connection was available to all users. This investigation changed none of
  those settings.
- The active Wi-Fi driver is `mt7925e`. The host identifies as a GMKtec
  NucBox EVO-X2, BIOS 1.12 dated December 9, 2025, kernel `7.0.0-38-generic`,
  with early-loaded CPU microcode `0x0b700037`. These are inventory facts,
  not evidence that a particular component caused the stall.
- The October 23 system boot also ends abruptly at 20:56:44. Its saved warnings
  do not establish why that earlier boot stopped recording; its power history
  was not verified. Do not assume every historical outage has the same cause.

## Current diagnostic recording

The initial 30-minute Halo health recording began at 15:35. At inspection,
417 samples through 15:56 reported the same connected Wi-Fi state. The rover
container was stopped at inspection during this observation.

Two new read-only recorders are bounded to one hour from approximately 15:58:

- Halo PID `11609`: `/home/ian/.cache/halo-outage-20261027-live2/health.jsonl`.
  Saves UTC time, boot ID, uptime, memory/load, Wi-Fi state, cached signal/
  frequency, gateway ping and remote-desktop service state. Each sample is
  flushed and synchronized to disk. Cached Wi-Fi information is not a new scan.
- Windows PID `10692`: `C:\Projects\Capstone\.halo-recovery\outage-20261027\windows-connectivity.jsonl`.
  Saves timestamped Tailscale-IP ping and SSH uptime checks every 15 seconds.
  It runs in a hidden process and exits after one hour.

Both produced successful baseline samples. If SSH disappears while the Halo's
local recording continues, that favors a network/access fault. If local
recording stops too, compare the cutoff with the actual power-off time before
inferring another stall. A lack of saved records alone cannot prove its cause.

A further one-hour SSH stream saves live system warnings directly on Windows
in `C:\Projects\Capstone\.halo-recovery\outage-20261027\live-system-warnings.txt`.
Its `journalctl` reader is PID `17601` on Halo. This can preserve an error even
if the Halo subsequently fails to flush its journal. It cannot guarantee a
message will be emitted before a hard lockup, and it stops if SSH is lost.

No drivers, firmware, power settings or network profiles were modified. No
simulation workload was restarted during the focused investigation above.
The user subsequently requested normal-workload restoration; the
[dashboard follow-up](HALO_DASHBOARD_UPDATE_20261027.md) records deployment and
stationary UI checks with the diagnostic recorders running. No further
whole-host outage occurred during those checks.

## Evidence and next isolation step

Raw system journals, sysstat excerpts, boot history and the final application
logs are preserved locally under
`C:\Projects\Capstone\.halo-recovery\outage-20261027\`. Original session
files remain on Halo. Full crash memory and credential-bearing environment
records were not copied into this repository.

The next useful workload comparison is stationary simulation alone, followed
separately by ROCm terrain and the NPU detector, with the paired recorders
running. Keep motion disabled and hold the display/profile constant. Recovery
access is needed before deliberately attempting to reproduce a machine hang.
There is not enough evidence to select a driver or BIOS change yet.

## Later captured incident

At 17:01 Central, the Windows-side warning stream captured an AMD graphics-ring
timeout naming `gz sim gui`, failed recovery attempts, then GNOME Shell and RDP
crashes while SSH stayed reachable. The
[GPU/RDP incident report](HALO_GPU_RDP_OUTAGE_20261027.md) records the evidence
and successful desktop recovery without rebooting. This later incident does
not establish the cause of the earlier 15:18 whole-host recording cutoff.
