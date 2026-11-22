# Autonomous 0.10 m/s qualification — November 17, 2026

Status: testing stopped at the operator's request pending physical access to
Halo. Remote access was lost during the fresh stationary check. Current host
and workload state are unverified; no further remote attempts will be made
until the operator is physically available and requests resumption.
No 0.10 m/s movement occurred or qualification was issued.

The operator requested the next tests after the [GPU repair and bounded
0.05 m/s exploration/return](HALO_GPU_REPAIR_20261117.md). This session uses
the same pinned V26 application and SDMA-off terrain candidate. It begins
with representative 1, a fresh monitoring case, current-case NPU qualification
and CPU/ROCm parity, a connected-desktop identity check, brief exterior
inspection and five minutes of stationary mapping. Only after those checks
pass will the operator start the 0.10 m/s exploration/early-return sequence.
Sampling, other scenes and longer operation require separate reviewed results.

The fresh recovery-v5 helper has the same code as recovery-v4. Its inputs
add the closed prior case and final scoped journal review and pin the current
closed native selection. Six recovery/environment tests and its source checks
passed. The review permits exactly the historical reset already reviewed in
this boot; new incidents remain fatal. Original incident flags remain intact.

| Input | SHA-256 |
|---|---|
| V26 application configuration | `2dd79d3a13871baf745c42587f4da9377d3e8d8e2d25c61837095fffa62505de` |
| V26 runtime release | `476620f66bb5288432a7d6bb1b9d89f891308e6c92805f6a77b83ba02b2bbdbf` |
| Accepted bounded 0.05 m/s receipt | `a86faabafe1b509a0916d5bc0331359fc65f130a0c0dcb7bfc3659cf1b260f20` |
| Recovery-v5 inputs | `e10147513573e619f61f60949249cceea780fbf71cc899a3998303756c8f6614` |

The recovery inputs and all runtime, source and lower-speed evidence pins
passed on Halo at 21:15 UTC. The host was idle, with the same boot and no
simulation containers, NPU/terrain listeners or KFD owners. These are
preparation checks, not hardware-test acceptance.

The 87°C qualification stop, native 90°C protection, one-second operator
heartbeat deadline, freshness, unknown-ground, hazard, explicit arming and Stop
rules remain active. Observers use GET requests and never send operator
heartbeats or commands. Keep monitoring through workload teardown, protected
finish, cooldown and final journal capture before closing and archiving logs.

Private evidence is kept separately under
`navigation_sessions/autonomous-demo-20261109/qualification-010-20261117/` on
Windows and Halo. Prior cases and the verified GPU repair archive are preserved.
The initiating GPU fault, the V24 transient deadline stop and the November 10
freeze remain unresolved. Short successful operation will not establish
endurance or full demo acceptance.

Fresh capture began at 21:20:29 UTC; the durable off-host observer began at
21:20:52 UTC. Both are running. Current-case NPU execution was verified, and
terrain classify, depth and projection parity passed with exact costs/reasons.
The actual terrain process has `HSA_ENABLE_SDMA=0`, float64 ROCm geometry and
HIP scheduling flags 4 before and after warmup. Engine activity is not measured.

Scene `restart-20261117T212413Z-339d8f` reports rehearsal speed 0.10 m/s, angular
speed 0.30 rad/s and the retained one-second heartbeat deadline. Live ROS queries
independently verified controller desired speed 0.10 m/s, PoseProgressChecker,
0.10 m translation / 0.50 rad rotation thresholds and the 20-second timeout.
At the first query it was stopped awaiting clearance; initial sensor/odometry
guard latches are not evidence of a movement failure. No movement was started.

The first ten-second viewer attempt failed because login session 1249 vanished
between `list-sessions` and `show-session`. Its worker exited 0 with no cleanup
errors; the stopped scene and monitoring continued. The failure receipt and
logs are preserved. A separate `qualification-010-20261117/viewer-v2` sibling
retries the complete guard check at most three times, 20 ms apart, only for
that exact disappearing-session error. Each retry is logged; other failures
and persistent races still abort. Nine source tests passed, including Linux
process cleanup, persistent-race rejection and unrelated-guard rejection.
V26 application, runtime, worker code, inspection bound and safeguards are
unchanged. Corrected host viewer SHA-256:
`42bfd37760540f53067dc24d84bbbae67b365e92cd29f9fa76534a72d90831a1`.

The first off-host SSH stream timed out after its 21:29:17.811933 UTC record.
Its replacement began at 21:31:05.963115 UTC, a **108.151182-second off-host
gap**. Native capture retained health records across the interruption, with a
maximum observed interval of 5.351162 seconds in the reviewed window. The same
boot, stopped scene, zero travel and absent incident/control-stop flags were
verified after reconnecting. The obsolete observer was stopped only after
verifying its UID, start time and exact arguments. The network cause is unknown.
No movement or stationary mapping qualification occurred during that gap.

The operator requested a pause for availability, reporting no bad event.
Stationary observation therefore ended before the required five minutes:
47 valid samples from 21:33:23 to 21:37:13 UTC, approximately 230 seconds.
All retained samples passed the stationary checks. Mean sampled Tctl was
82.846°C, peak 84.25°C, maximum rolling mapping publication p95 120.897 ms and
minimum reported RTF 0.99604. No owned exterior GUI processes remained, and
power-saver was retained. These temperatures were higher than the preceding
stationary test; different initial conditions do not establish a cause.

The preserved sampler summary reports `passed=false` and connection refused
after the simulator shut down for the requested pause. It is an incomplete
window, not a completed stationary qualification or an unexpected navigation
failure. Mission events contain configuration only; exploration never started.
The terrain supervisor stopped gracefully with `operator_stop=true` and no
failure, and native workload stop and idle checks passed at 21:38:38 UTC.

Resume only on a new explicit operator request, using a fresh reviewed case,
current-case qualification/parity and the corrected viewer. Repeat the full
stationary prerequisite before the first 0.10 m/s movement attempt. Sampling,
remaining scenes and endurance tests remain pending.

Capture closed at 21:38:42 UTC before protected finish. That monitoring gap is
explicitly preserved; no monitoring-through-cooldown claim is made. All workload
and monitoring processes were verified off at 21:42:41 UTC, and the remaining
off-host observer was stopped by verified process identity. The pending operator
step is `~/bin/halo-rover finish` for the cooldown and protected kernel journal;
capture is left closed and the old case is not reused.

A raw pause checkpoint was downloaded and all **75 manifest members** independently
verified on Windows (**10,407,613 bytes**). Manifest SHA-256:
`56dd02f8e69902026945066d2041151175b99734527b62b05a26ab817b58abc5`.
The receipt is `PAUSED_RAW_OFFHOST_VERIFICATION.json` in the private test folder.
This preserves current evidence; the protected-final review and final evidence
archive remain pending. It does not complete hardware qualification.

On resumption, 1,670 configuration, runtime, terrain, corrected-viewer and
lower-speed evidence pins passed again. The boot is unchanged and native idle
checks passed. A separate `qualification-010-resume-20261117/recovery-v6`
sibling passed the same six recovery/environment tests. It starts no hardware;
its final input pins will bind the closed predecessor and reviewed journal.

Protected finish completed at 18:58:23 CST on November 17 (00:58:23 UTC,
November 18). The full journal matched exactly the 14 reviewed historical
entries; no additional incidents were found. Native idle checks passed and
`INCIDENT_REVIEW_REQUIRED` remains preserved. The final scoped review records
`review_complete=true`, `scoped_no_new_incidents=true`, `monitoring_complete=false`
and `passed=false`: the operator pause interrupted monitoring and qualification.

The closed-case archive `HALO_SPEED_010_20261118T005913Z.zip` contains 1,696
verified members and is 329,362,288 bytes. Its SHA-256 is
`233b4295a692bf91a74e554f8542cb4908af799b1ee6da7a1515447cd07ba683`.
Independent Windows ZIP/member verification passed for every member. The
archive includes the stopped native case/kit, V26 source/install/runtime,
corrected viewer, terrain candidate, closed off-host logs and source/report
snapshot. It preserves the incomplete result and both monitoring gaps.
The external receipt is `verified-archive/HALO_SPEED_010_OFFHOST_VERIFICATION.json`.

Fresh recovery-v6 input SHA-256:
`3684d4092de2a393d9bc408b34b609be4a723c1f19c4963736212b7fe16d2fbd`.
The protected review and fresh capture have been requested; no new workload
has started yet.

The resumed case used native selection `rocm-manual-20261118t010234z-ee1d`.
Its protected review, NPU qualification, retained cooldown and fresh CPU/ROCm
parity passed. Scene `restart-20261118T010732Z-f302d0` passed corrected inspection
and live controller checks. Current Ogre logs confirmed Radeon 8060S/radeonsi.
The complete five-minute stationary prerequisite began at 19:08:57 CST on
November 17 (01:08:57 UTC, November 18). No exploration was requested.

Remote logging ended at 19:12:43 CST (01:12:43 UTC). The last record showed
`ready`, zero travel, healthy status and 78.75°C. SSH and the RDP tunnel became
unreachable, and Tailscale reported Halo offline. Completion of the stationary
window, native monitoring continuity and current process state could not be
verified. Reachability loss alone does not identify a GPU reset or its cause.

The operator confirmed they could not access Strix and requested a stop until
physical access is possible. No reboot, power cycle or host setting change was
made. The fresh case remains unclosed and requires evidence review on recovery.
Local logs and `qualification-010-resume-20261117/STATE.md` preserve the exact
case, process and scene identities for that review. Earlier "requested" and
"not started" statements above describe preparation checkpoints, not current
workload state. The previous case's verified archive remains unchanged.
