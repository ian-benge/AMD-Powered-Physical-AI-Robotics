# Captured GPU timeout and desktop recovery - October 27, 2026

The 17:01 Central outage was a captured AMD graphics timeout followed by
desktop and RDP crashes. The full journal clarifies that the initial ring reset
failed, but a subsequent full GPU reset succeeded and discarded graphics-memory
contents. Desktop applications lost their GPU contexts and aborted even though
the kernel recovered the GPU. SSH remained reachable. Restarting the login manager
restored Ian's existing automatic-login desktop and the RDP listener without
rebooting the host. The operator confirmed that the remote desktop remained
connected for at least 30 seconds afterward.

This incident has stronger evidence than the earlier 15:18 recording cutoff.
It does not establish that the earlier whole-host stall had the same cause.

## Saved timeline

| Central time | Observation |
| --- | --- |
| 17:01:17 | Kernel reports `ring gfx_0.0.0 timeout`; identifies process `ruby`, PID 203543, thread `gz sim gui:cs0`, PID 203756 |
| 17:01:17 | Driver attempts a graphics-ring reset |
| 17:01:19 | MES reset times out; legacy queue/ring reset and queue eviction fail |
| 17:01:19 | Driver cannot suspend process 24829, the previously started ROCm terrain service |
| 17:01:21 | Further queue-removal failure; driver cannot halt graphics command processing |
| 17:01:21.331-380 | Driver performs a MODE2 GPU reset, reports success and lost VRAM contents, then records that the wedged device recovered through reset |
| 17:01:21 | GNOME Shell, Xwayland and Remote Desktop report lost GPU contexts; Shell aborts |
| 17:01:27-32 | GNOME Shell and Remote Desktop services report ABRT/core-dump exits |
| 17:01:32 | Remote Desktop auto-restarts, but Ian's graphical session has ended; no RDP listener is present |
| 17:23:10 | After restarting `gdm3.service`, Ian automatically logs into a new Wayland session and RDP starts on port 3389 |

The warning stream was captured on Windows before the desktop failure.
Subsequent Windows connectivity samples continued to receive Tailscale-IP ping
and SSH uptime replies. The `Connection refused` messages came from forwarding
RDP connections to a port with no server listening, rather than failure to
establish SSH.

This successful full reset appears in the complete kernel journal, rather than
the warning-only Windows stream used during initial recovery. Initial reset
failures do not mean the GPU remained permanently unresponsive. The local health
recording continued across the event, with Wi-Fi connected and substantial
available memory; the inspected incident window contains no out-of-memory kill.
The confirmed sequence is graphics timeout, full GPU recovery, application
context loss and desktop-session failure. It does not establish the original
trigger of the graphics timeout.

The driver's named graphics workload is an important isolation lead. It does
not by itself prove whether Gazebo, Mesa, the kernel driver, firmware or
hardware caused the timeout. Graphics and ROCm share the GPU; seeing both in
the recovery failures does not establish which workload initiated the fault.
The earlier isolated material probe had already completed; its separate
cleanup failure is documented in the
[rendering investigation](HALO_DETECTOR_RENDERING_INVESTIGATION_20261027.md).

## Recovery performed

Read-only checks found no listener on either 3389 or 3390, despite the RDP user
service being active and enabled. Ian had no active graphical-session target;
the local seat showed the GDM sign-in screen. Existing configuration already
contained `AutomaticLoginEnable=True` and `AutomaticLogin=ian`.

Because the operator was away, the login manager was restarted remotely using
`sudo systemctl restart gdm3.service`. A new `gdm-autologin` Wayland session for
Ian appeared. Port 3389 began listening, and the service journal recorded
`RDP server started`. No desktop credentials, network profile, driver, firmware
or automatic-login settings were changed. No host reboot was performed.

The existing Windows tunnel remains appropriate:

```powershell
ssh -N -T -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -L 127.0.0.1:13390:127.0.0.1:3389 halo
```

Keep it open and connect Windows Remote Desktop to `127.0.0.1:13390`.
Recovery restores access; it does not fix the underlying graphics timeout.

## Evidence and next work

Private raw and recovered logs, before/after listener/session state, automatic
login configuration and SHA-256 manifests are preserved under
`C:\Projects\Capstone\.halo-recovery\gpu-rdp-outage-20261027\`.
The original Windows warning file had a short UTF-16 prefix followed by UTF-8
stream bytes. Its original bytes were retained, and the timestamped UTF-8
section was recovered separately. It is not a corrupted Halo journal.

Simulation recovery was not performed during RDP recovery. The dashboard
gateway was reachable afterward but supplied no active ROS session status, so
camera, motion and map readiness must be re-established separately. Previous
session files remain on disk.

Before resuming sustained simulation, isolate visible Gazebo GUI, sensor
rendering, remote desktop and ROCm terrain in bounded comparisons with saved
logs. Hold the scene/display/profile constant and change one workload at a
time. Preserve this failure record before considering any driver or firmware
change. The current evidence does not justify claiming that Wi-Fi changes or
automatic login have fixed the GPU fault.

## Full local log export

A subsequent collection pulled the complete retained October 27 journal,
full current/previous boot journals, kernel/GPU, desktop, network, power, Docker
daemon, retained rover-session logs and paired monitoring evidence. Combined
with the Windows recordings and reports, the package contains **454 files**:

`C:\Projects\Capstone\halo-crash-logs\HALO_LOGS_20261027T223028Z.zip`

The extracted collection and coverage guide are under
`C:\Projects\Capstone\halo-crash-logs\20261027T223028Z\README.md`.
The ZIP is 25,263,916 bytes; SHA-256:
`49d3a711c90c7e9e49bb6d6994e25faa196cdfa1b120d3e59d99e923bfb36ad1`.
The server-archive hash, individual extracted-file hashes and ZIP integrity
were verified. Exact commands, file inventory and collection gaps are included.

The auto-removed navigation container was already absent, so its private
Docker log stream could not be recovered. Host-mounted rover logs remain
included. `coredumpctl` is not installed; stored crash metadata and journal
crash messages were collected. Binary memory dumps and credentials are outside
this log export. Original source files were retained, and the raw collection
is stored outside the published Git repository.

See [startup and future capture commands](HALO_START_AND_CRASH_CAPTURE.md) for
relaunching the current browser simulation, retaining host launch logs and
capturing temporary GPU device dumps during a recurrence.
