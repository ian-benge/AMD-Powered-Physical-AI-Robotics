# Phase 1 rendering candidate

Use the [acceptance report](PHASE1_RENDERING_ACCEPTANCE_20261104.md) for measured
results and open items. This guide selects a separate candidate; the retained
`~/bin/halo-rover` launcher continues to select its original baseline.

The candidate fixes opaque Ogre texture depth writing, retains genuine alpha
blending and depth testing, and provides 20 m RGB with 8 m usable depth and a
0.30 m near limit. The default source camera range remains 8 m. Use the explicit
candidate profile only with its qualified image, engine and annotation modes.

## Identities and locations

| Item | Selection |
| --- | --- |
| Baseline image | `sha256:b976b898cc96ad898336310fcd630e2151af527656e549ed24b5f0967a8fc88b` |
| Candidate tag | `rover-navigation-phase1-resume-20261104-v4` |
| Candidate image | `sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa` |
| Baseline Ogre backend | `ab53815ad7857af955710ec62da0a065f6d2ea9712767da9b6fcfa31ae02b360` |
| Candidate Ogre backend | `f6e8842247198a2b34b25cfdb40a8f4bc4f156c2e90a54135a68698dc239a3e6` |
| Halo evidence root | `/home/ian/capstone-training/rover-navigation/navigation_sessions/rendering-phase1-resume-20261104T202728Z` |
| Isolated application build | Evidence root + `/application-v5` |
| Controlled comparisons / stationary evidence | Evidence root + `/qualification-v6` |
| Final motion-fixture / analysis tools | Evidence root + `/qualification-v10` |
| Camera profile | [`config/rendering/phase1_ogre_candidate.json`](../config/rendering/phase1_ogre_candidate.json) |

Keep OEM 6.17.0-1033, inbox amdgpu, the reviewed temporary NPU module and qualified
model/runtime. Terrain uses exactly
`/home/ian/capstone-training/venvs/terrain-rocm7141/bin/python`, isolated mode,
ROCm 7.14.1/Torch 2.12, float64, and HIP flag 4 before initialization and after
warmup. Retain Quiet/Automatic fans and Linux power-saver. These instructions
change no kernel, firmware, driver, power, fan or remote-desktop settings.

## Start the retained baseline

The retained `default-stack/COMMANDS.md` belongs to the private AMD support
package and is not shipped in this checkout. For clean-checkout setup and current
artifact requirements, use the [AMD testing guide](AMD_TESTING_GUIDE.md).
Inspect `~/bin/halo-rover status` first. In separate terminals, start `capture`,
`temps`, `npu`, and `terrain`, waiting for each readiness gate. Inside the Halo
graphical desktop, run `~/bin/halo-rover sim`. After visually inspecting launch
clearance, run `~/bin/halo-rover drive-ready`.

The dashboard is **Halo localhost:8777**. For Windows, check existing tunnels,
then use a separate terminal:

```powershell
ssh -N -T -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -L 127.0.0.1:8877:127.0.0.1:8777 halo
```

Open `http://127.0.0.1:8877/`. Start manual checks at **0.05 m/s** and at most
**0.30 rad/s**, with all localization, obstacle and command guards retained.

## Start the separate candidate

Close the previous session with `stop` and protected `finish`, review its final
report, then end its capture and temperature terminals. Use a **fresh** default
capture and temperature monitor, followed by the default NPU and terrain
commands and readiness gates. Do this with the operator attending. Keep the
temperature display visible throughout.

In a terminal inside the graphical desktop, select the fresh context and run
parity, then the separate candidate supervisor:

A saved copy of this sequence also checks the frozen configuration file's
SHA-256 before running parity. With NPU and terrain ready, run:

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/rendering-phase1-resume-20261104T202728Z/start-candidate-preview.sh
```

That command file's SHA-256 is
`269ed93b35b218c7b9bc8d476d594b853bfd83bcc2ad953ea8923b8058d7ba2c`.
The equivalent manual sequence is:

```bash
PHASE=/home/ian/capstone-training/rover-navigation/navigation_sessions/rendering-phase1-resume-20261104T202728Z
APP="$PHASE/application-v5"
eval "$(python3 -I /home/ian/capstone-training/rover-sim/manage.py context)"
CASE=$(cat "/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103/capture-$SIM_LABEL.path")
bash "$SIM_KIT/trial.sh" "$SIM_LABEL" parity
python3 "$APP/management/phase1_supervise_live.py" "$APP/CANDIDATE_CONFIGURATION.json" \
  --kit "$SIM_KIT" --case "$CASE" --seconds 600 --attended
```

The configuration records the candidate image, source/install hashes and retained
scene hash. The supervisor checks those pins and the original runtime/capture
gates, keeps a separate log, and verifies the server's actual Radeon/Mesa/Ogre
library mappings. It stops the candidate on a guard failure or at the bounded
preview's end. It does not confirm clearance or arm motion. Wait for
`RENDERER_PROVENANCE_READY`, inspect the scene, then use the dashboard's launch
clearance confirmation and manual controls at the speeds above. A startup stop
remains latched until an explicit valid mode selection.

This bounded preview uses no scene-restart supervisor. Keep restart/generation
and autonomous mission controls out of the initial live comparison. The source
restart path preserves image, install, engine and range; separate restart and
new-scene acceptance remains outside these fixture results.

## Stop and rollback

Press **Stop** in the browser, then Ctrl+C in the candidate supervisor terminal.
It stops only the candidate container it identified. Then:

```bash
~/bin/halo-rover stop
~/bin/halo-rover finish
```

Keep capture and temperatures open through protected finish/cooldown. Review
`MANUAL_SESSION_FINAL.json`, the candidate's `supervisor-sessions` result and
incident evidence. Then end capture/temperature terminals. For rollback, start
a fresh session with the retained baseline sequence above. Its source, install,
image tag and expected hashes were preserved; do not edit its pins.

Stop workloads at Tctl **>=90 C**, failed monitoring/capture, fan failure or any
GPU incident. The default temperature display only warns. Preserve the capture
and prompt device dumps before any reboot. An interrupted trial stays incomplete;
do not restart qualification on a boot that has suffered a GPU incident.

## Repeat isolated comparisons

Never teleport the navigation rover. The fixture contains only an evaluator
camera and geometry in a separate `GZ_PARTITION`, with no ROS bridge, detector,
mapper or controller. Its truth is not sent to navigation.

Saved geometry frames require equal measured camera poses before and after the
image timestamp, each within 50 ms. Transient frames spanning a pose update
still receive RGB-D health checks and are counted, but are excluded from saved
geometry evidence. The offline evaluator verifies every saved bracket.

With a fresh capture/temperature monitor and original kit guards ready, stop
the navigation, NPU and terrain workloads before running one fixture; leave
capture and temperatures running. Read `status` and obtain
the current `SIM_KIT`/`CASE` as above. Use a new recording name each time:

```bash
Q="$PHASE/qualification-v10"
python3 "$Q/source/scripts/phase1_run_fixture.py" "$Q" visibility-stationary-20261104-new1 \
  --image sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa \
  --fixture fixture20 --kit "$SIM_KIT" --case "$CASE" \
  --mode stationary --seconds 600 --attended
```

Names must contain lowercase letters, digits and hyphens. Run separately for
`stationary`, `approach_retreat` and `rotation`. Each is a fresh container and
recording; review a run before proceeding. Geometry analysis runs after the
container stops, using the matching `fixture.json` and synchronized recording.
`session-result.json` deliberately says `REVIEW_REQUIRED` until RGB, geometry,
clipping, transparency and kernel evidence have been assessed.

For the retained qualification directory, analyze every saved frame with a CPU
container (no GPU devices) and the calibration saved by that recording:

```bash
docker run --rm --network none --user "$(id -u):$(id -g)" \
  -v "$Q:/evidence" \
  -e PYTHONPATH=/evidence/source/src/mars_training_environment:/evidence/source/src/rover_training \
  sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa \
  python3 /evidence/source/scripts/phase1_analyze_fixture.py \
  /evidence/fixture20/fixture.json /evidence/visibility-stationary-20261104-new1/capture
python3 "$Q/source/scripts/phase1_evaluate_visibility.py" "$Q/visibility-stationary-20261104-new1" \
  --output "$Q/visibility-stationary-20261104-new1/visibility-verdict.json"
```

Keep temperature/capture monitoring active for analysis as well. Reuse no output
path; analyzers and verdict writers refuse to overwrite evidence. The offline
verdict remains pending until protected capture/kernel teardown review.

Compare baseline 8/8 m to candidate 8/8 m first, then candidate 8/8 m to candidate
20/8 m. Keep near 0.30 m, geometry, lighting and poses identical. `--probe` adds
the material contract probe. The baseline is expected to fail that probe.

Ogre clips far distance axially. A nearer-than-0.30 m occluder becomes invalid
depth/background RGB and still hides farther geometry; `near_clip_mode=occlude`
models that observed behavior. Annotation retains historical radial/near-clipping
defaults for old profiles. RGB-only distant objects receive no positioned-map
landmark without valid depth. Existing capture quality checks continue to flag
labels lacking depth support; this work authorizes no large dataset collection.

The older `phase1_fixture_trials.sh` interface is retired because its delayed
log grep and missing live gates were insufficient. Use the retained default
capture/device-dump workflow and the guarded runner. No later improvement phase,
training, model export, commit or push is part of these commands.
