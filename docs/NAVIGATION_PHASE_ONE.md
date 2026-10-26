# Phase One: explain and measure mapping failures

This phase adds read-only diagnostics to the existing guarded manual-mapping
workflow. Navigation thresholds, camera geometry, speed limits, unknown-space
blocking and watchdogs retain their existing settings.

The control page on Halo port 8774 now shows accepted-ground reception age,
localization health, mapped square metres and hazard-cell count. Localization
age compares TF with fresh simulation `/clock` receipts. HTTP control leases continue
to use wall time. Displayed age is reception age, not camera exposure latency.
Fresh localization indicates timely transforms/odometry; it does not measure drift.

The map shows the padded rover footprint about `drive_center`, and the guard's
11 footprint samples over its one-second motion prediction. After a terrain
rejection, dashed orange footprints preserve the rejected command's prediction.
The white outlined cell identifies its first rejected footprint sample. The
explanation gives the cell centre in odom metres, row/column and observation
stamp. Non-spatial faults show their last valid pose when available. New mode
selection clears the guard's current fault; audit evidence remains on disk.

Enable **Show terrain causes** for gray unknown ground, orange excessive slope,
purple excessive height change and pink cells with both causes in their retained
history. Red indicates a hazard whose detailed evidence is unavailable. Pale
ground includes the operator-confirmed launch disk; that disk is not a camera
observation beneath the rover. The slope limit remains 12 degrees and the
within-cell height-span limit remains 3 cm. Unknown includes missing depth and
unsupported measurements; it does not establish a physical drop-off.

Causes are computed from each individual depth frame alongside its existing
classification. Two-frame-confirmed hazards keep their causes through later
free readings. Current, unconfirmed hazards also appear in the local diagnostic
grid. Map resampling uses the same four subcell samples as the navigation map.
Both current and frozen odom overlays use the current map correction, including
rotation. Diagnostic topics cannot update guard sensor health or enable motion.
Missing or mismatched cause evidence is explicitly reported as unavailable.

Topics added:

- `/navigation/local_ground_reasons`: local odom grid, same depth stamp as local ground.
- `/navigation/terrain_reasons`: retained map grid, same stamp as terrain map.
- Existing `/navigation/status` now includes footprint geometry and rejection evidence.
- Existing `/navigation/mapping_status` includes depth receipt age, accepted stamp,
  update duration and unchanged thresholds.

Reason values are `-1` unknown, `0` traversable, bit `2` slope, bit `4` height
change and bit `8` unavailable hazard evidence. These are diagnostic codes,
not occupancy probabilities. The page serves their rendering at `/diagnostics.png`.
The command HTTP handler reads cached guard health directly; visualization and
supervisor requests run through `/status` instead of the command path.

## Capture a synchronized baseline

Inside the running navigation container:

```bash
source /opt/ros/jazzy/setup.bash
source /work/install_navigation/setup.bash
python3 scripts/halo_navigation_terrain_audit.py \
  --session navigation_sessions/SESSION \
  --output navigation_sessions/SESSION/checks/phase1-baseline-001.json \
  --wait 20 --duration 10 --interval 1
```

The observer issues no motion or mode commands. It requires exact matching RGB,
depth, CameraInfo, local terrain, global terrain and global cause timestamps,
matching sensor dimensions and image-time transforms. Buffers and run duration
are bounded; a missing match fails capture instead of substituting newer data.
If a mapped capture fails, a separately labelled `-unmapped` bundle preserves
matching camera/depth/calibration inputs when available. It contains no invented
pose and does not turn that failed audit into a pass.

Each `-frame-NNNN` bundle contains a raw camera JPEG, an NPZ containing BGR,
original depth bytes and terrain arrays, and JSON with encoding/stride/endianness,
calibration, grid origins/resolution, image-time transforms and latest guard
status. The guard status is labelled separately because its receipt is
asynchronous. The published global map uses the processing-time SLAM correction;
image-time transforms are recorded separately. The main report and PNG retain
the existing single-frame classification and independent scene comparison.
The report measures camera and terrain delivery rates and wall-time gaps
separately from simulation timestamp gaps.

`passed` in an audit report means evidence capture succeeded. Terrain accuracy
and fixture acceptance are assessed separately. Scene truth is read only by
this independent evaluator; navigation never consumes it.

For UI checks through the Windows SSH forward on port 8874:

```powershell
node scripts/navigation_browser_check.cjs navigation_sessions/SESSION/checks/ui-001 --diagnostics
```

`--drive` exercises bounded simulation movement, repeated commands, release,
Stop and connection-loss stopping. `--restart` operates the real Restart button
and verifies zero map cells/candidates before launch confirmation. Run each with
a new output prefix; evidence is never intentionally reused. Use inspected
fixtures for movement checks. Source regression tests cover real slope/step
classification, unknown holes, retained causes, map corrections, stale evidence,
diagnostic independence and exact-stamp audit matching.
