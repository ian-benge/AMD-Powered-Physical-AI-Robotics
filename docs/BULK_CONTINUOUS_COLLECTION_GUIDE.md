# Continuous monitored collection

These commands describe the existing pinned Halo deployment. A fresh GitHub
checkout does not include its operator kit, runtime bundles, model artifacts or
saved campaign. Use [Getting started](GETTING_STARTED.md) for portable source
checks and the CPU simulation; deployment paths below are machine-specific.

The separate supervisor resumes the existing 10,000-image campaign and renews
monitoring automatically between batches. One initial local sudo launch allows
the native protected capture and final journal commands to run for each window.
The collector, scene planning, Gazebo containers and temperature logger run as
`ian`. No sudoers entry or persistent service is installed.

Before starting, close any older monitoring session with `~/bin/halo-rover finish`
and stop its old monitors. Leave NPU, terrain, simulation and other collectors
off. The new supervisor owns its capture and temperature processes; do not start
separate `halo-rover capture` or `temps` terminals alongside it.

Use a connected Halo graphical-desktop terminal and leave the supervisor visible:

```bash
sudo /usr/bin/python3 -I /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-continuous-20261107/start_bulk10000_continuous.py --run
```

It verifies the pinned v9 controller/camera candidate, prepares a fresh native
kit and capture case, starts the unchanged 7,200-second recorder, waits for two
health events, verifies native module/fan gates and starts temperature logging.
The supervisor prints temperatures and fan speeds every two seconds and image
progress every thirty seconds. The existing collector verifies prior images,
plans, runtime proofs and receipts before resuming the next unfinished batch.

On `SESSION_RENEWAL_REQUIRED`, the supervisor requires forward progress and a
successful collector exit. It checks idle state, keeps both monitors alive and
fresh through the full 60-second cooldown, runs the native protected final
journal helper, checks clean teardown, then stops its monitors. A new native
window starts automatically. On 10,000 images it finishes the last monitoring
window and independently verifies every batch before reporting completion.

The default permits up to 50 windows, with at least one newly completed batch
required per renewal. Each native capture retains its two-hour duration and
each batch retains its guarded deadline. Thermal, fan, kernel/GPU, desktop,
sensor, renderer, framing, annotation, storage and evidence checks remain in
force. Real failures and Ctrl+C stop collection and do not trigger relaunch.
If final journal capture or workload cleanup fails, monitoring remains open
for local review and the supervisor records its process identities and paths.

Press Ctrl+C once to stop. Allow guarded workload cleanup, the cooldown and
final journal capture to finish. Avoid closing the terminal or stopping its
monitoring children during this sequence. Keep the graphical desktop/RDP
connection active throughout collection. Automatic window renewal does not
establish unattended endurance stability or detector-training approval.

Read live status from another terminal:

```bash
watch -n 5 /usr/bin/python3 -I /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-continuous-20261107/start_bulk10000_continuous.py --status
```

The status combines the supervisor's window/stage with the existing collector's
total images, partial batch and detailed log path. Each supervisor invocation
stores `RUN_IDENTITY.json`, events, command results and a directory per native
window under `navigation_sessions/bulk10000-continuous-<UTC>-<unique>/`.
`WINDOW_RESULT.json` binds protected teardown to the exact kit/case and reports
monitoring gaps instead of treating them as clean renewal.

Before rollout, qualify the actual protected monitor rotation without rendering
or generating training images:

```bash
sudo /usr/bin/python3 -I /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-continuous-20261107/start_bulk10000_continuous.py --qualify
```

This performs two complete native monitoring/cooldown/final-journal cycles.
The source test suite covers completion, clean renewal, bounded iteration,
interruptions, no-progress stops, health freshness, pinned inputs, process
credentials and protected-finish ordering. The existing manual starter remains
available for rollback; use either launcher with a closed prior session.
