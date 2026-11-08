# Environment v2 operator guide

November 5, 2026; closed November 6 UTC. Implementation is delivered. At the
user's explicit request, the separate candidate now defaults to **40 m sparse**
using application-v9. Its status is **experimental**: live latency and remaining
density checks are deferred. The accepted 20 m launcher remains available.
See the [final closeout report](SIMULATION_ENVIRONMENT_V2_CLOSEOUT.md).

The new operating area is 40 x 40 m, with a shared 257 x 257 physical/visual
surface, a joined 20 m surrounding terrain apron, 8192-pixel ground composites
and sparse, standard and cluttered recipes. Navigation memory covers 40 m;
terrain beyond it remains unknown. All scene geometry and route certificates
remain confined to generation/evaluation.

The ordinary hazard limits remain 12 degrees / 3 cm. Adaptive clearing retains
three qualified closer, translated observations and never releases a stop.
The new manual profile starts at 0.05 m/s, caps drive speed at 0.20 m/s and turn
speed at 0.30 rad/s. Restart restores the conservative starting speed.

The native terrain service's 1200-cell bound cannot accept the new 1600-cell
memory. A separately pinned candidate service extends the bound to 1600 while
retaining the exact isolated interpreter, runtime-library hashes, float64
algorithms and verified HIP blocking-sync flag 4. Its worker uses the native
terrain PID slot and protected finish workflow. Neither the baseline source nor
the operator kit is edited. The new mapper rejects an undersized terrain service.

## Default launcher

With a fresh attended native capture, visible temperatures, ready NPU and the
candidate terrain service, use the Halo graphical desktop terminal:

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/environment-v2-20261105T225758Z/start-environment-v2.sh
```

No argument selects sparse 40 m. An explicit `1`, `2` or `3` selects sparse,
standard or cluttered. `baseline` selects the candidate's 20 m comparison scene.
The accepted native `~/bin/halo-rover` launcher remains unchanged for rollback.
The default wrapper verifies and invokes the pinned v9 starter; the receipt is
`DEFAULT_LAUNCHER_IDENTITIES.json`. Older numbered starters/receipts retain their
historical defaults. The v9 mapping scheduler has passed offline checks; its
40 m live latency has not been measured. No workload was left running at closeout.

## Monitoring and optional future qualification

The user ended the current qualification effort. The remaining procedure below
is available for a future explicitly requested session; no further drives are
required to close this implementation. Default selection does not certify the
uncompleted acceptance gates.

Keep a fresh `~/bin/halo-rover capture` and `~/bin/halo-rover temps` visible and
running during the Halo build and checks. Leave GPU workloads off until source
and geometry checks pass. Stop at Tctl >=90 C or a monitoring/fan failure.

Use the checked candidate configuration, which pins its source, installation,
scenes and management tools. Source, grid-capacity and offline parity checks
passed. Fresh runtime/monitoring gates are still required for each live session.

The prepared starters are under
`/home/ian/capstone-training/rover-navigation/navigation_sessions/environment-v2-20261105T225758Z/`.
After NPU readiness, start the candidate service in its own terminal:

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/environment-v2-20261105T225758Z/start-environment-v2-terrain.sh
```

Leave the simulator off until the current-case functional parity and 40 m ROCm
publication parity pass. Later, the attended checked starter accepts `baseline`,
`1`, `2` or `3` as its final argument. The
[closeout report](SIMULATION_ENVIRONMENT_V2_CLOSEOUT.md) lists the deferred gates.

Attended acceptance consists of a matched 20 m baseline and one 40 m run per
density. In each larger-world run, confirm launch clearance, explicitly enable
manual mode and check 0.05, 0.10, then 0.20 m/s on mapped clear ground. Stop
between speed stages. Reach at least 12 m along one axis and return over
observed ground. Do not bypass a guard rejection or restart during recording.
Press browser Stop, wait for `ENVIRONMENT_V2_SYNC_CAPTURE_COMPLETE`, then Ctrl+C
the supervisor. Each scene has a ten-minute bound; synchronized evidence records
for five minutes at 2.5-second intervals.
The 20 m baseline records for two minutes and prints
`PHASE3_SYNC_CAPTURE_COMPLETE`. The default entrypoint now selects sparse 40 m
by explicit user request; use the `baseline` argument for the comparison scene.
The separate historical `start-environment-v2-v9.sh` still defaults to baseline.

Keep capture and temperatures running through `~/bin/halo-rover finish`, its
cooldown and final journal capture. Review `MANUAL_SESSION_FINAL.json`, then
stop the capture/temperature terminals.

## Future capture preparation

**Capture fix verified November 6:** the ten-image repeat saved eight populated
and two background frames with zero validation errors. Ogre capture uses its
supported RGB-D sensor and explicit measured-depth box verification; Ogre2
retains its existing box camera. See the [fix and repeat report](CAPTURE_OGRE_FIX_20261106.md)
for checks, previews and the geometry-only scope beyond usable depth. The three
original crashing attempts remain preserved. The pilot remains uncollected,
and the manual simulation's experimental 40 m default is unchanged.

`mixed_reachable_v2.json` selects the deployed 960 x 540 camera, 0.40 rad tilt,
qualified Ogre renderer, 20 m RGB and 8 m usable depth. The separate
`pilot200_environment_v2.json` contains ten training-only scene seeds, eight
populated scenes and two backgrounds, with 20 views per scene. No pilot images
are authorized or collected by environment qualification.

Four-class primary-target scheduling preserves source labels and uses original
appearance identity for the existing mudstone export conversion. Failed capture
attempts keep separate logs and outputs. Resume verifies scene identity and file
hashes. Capture requires the qualified material plugin and attended hardware
rendering; it cannot silently fall back to a software renderer.

Every eligible geometry target remains labeled through the 20 m RGB range.
Only the portion expected inside usable depth is checked against the 8 m depth
sensor. A visible target beyond that range uses geometry for its annotation;
failed visibility checks reject the attempt and preserve its diagnostics.

The eight-populated/two-background pilot is an explicit project configuration.
Its image variety, complete labeling and split reservations follow the linked
[Ultralytics dataset guidance](https://docs.ultralytics.com/yolov5/tutorials/tips-for-best-training-results/).
The configured 20% background share is a pilot choice, not that guide's general
0–10% recommendation. The pilot has not been captured or accepted as a dataset.
