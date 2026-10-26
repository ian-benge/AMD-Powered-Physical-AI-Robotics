# Phase Three: confirmed rocks and repeat recognition

Camera boxes are raw detections. They do not immediately become confirmed map
objects. The default browser and RViz object layer contains only confirmed
`rock` and `yellow_crystal_rock` tracks. **Show tentative candidates** exposes
pending and unverified observations as hollow markers. Soil patches and slabs
remain tentative in this phase.

## Confirmation and revisit behavior

Three depth-supported observations are required over at least two simulated
seconds, from camera **and rover drive-center** positions at least 0.20 m from
previously counted views. A rotation alone cannot satisfy this baseline. Each
supporting observation needs detector confidence >=0.50 and a robust surrounding
ground plane: at least 30 inliers, >=50% of surrounding samples within 1.5 cm,
plane normal within 20 degrees of vertical, and >=20% of interior samples at
least 3 cm above it. Missing plane support is ambiguous, not proof of absence.
The plane uses bounded, deterministic RANSAC and fits the current depth frame.

Class labels accumulate weighted evidence from independent views; >=70% support
for one eligible rock class is required. A label change is not a new object.
The existence score starts at 0.20, gains ln(4) log-odds per independent support,
loses ln(3) per eligible contradiction, and is bounded to [0.02,0.98]. Confirmation
also requires a score >=0.90. This is an **engineering evidence score**, not
calibrated detector accuracy or proof that a sample is scientifically interesting.

Associations use odom positions and conservative 3D covariance, gated by squared
Mahalanobis distance <=11.34 and absolute distance <=0.50 m. Matching is mutual,
one-to-one, with a 25% spatial margin over the next candidate. Color chromaticity
histograms and physical extents can reject inconsistent matches; appearance
cannot override ambiguous geometry. Ambiguous observations protect involved
tracks from negative evidence and cannot create a confirmed replacement. They
are retained in the event log. Established confirmed IDs are never auto-merged.
Covariance intersection prevents repeated correlated frames driving uncertainty
to zero; position standard deviations retain a 5 cm floor.

Evidence freezes with stale/invalid odometry, detector or synchronized frames,
missing image-time transforms, localization position standard deviation >0.10 m,
yaw standard deviation >5 degrees, or a detected odometry discontinuity. This
does not change guard readiness or rearm motion. A discontinuity requires a new
session or explicit odometry reset. Unreported drift can still cause wrong
associations: revisit acceptance must independently evaluate pose disagreement.

## Objects are not permanent

Missing detections count only with a healthy synchronized detector frame, a fully
projected in-view object region, sufficient valid depth, no detected occluder,
and an independent viewpoint. Out-of-view, stale or occluded objects receive no
negative evidence. Geometry explicitly showing an insufficiently raised surface
can provide a contradiction; an absent ground fit cannot.

Confirmed tracks demote to `unverified` below score 0.50 or class support 70%.
Reconfirmation requires three new supporting views. At score <=0.20 and at least
three independent contradictions they retire, disappearing from both map layers
but remaining in evidence. Tentative tracks expire after 30 simulated seconds
without an observation. Confirmed tracks do not expire merely because they leave
view. An odometry reset archives the epoch and starts distinct track IDs; clock
rewind requires Restart. Old candidate files are never imported as confirmed.

## Interfaces and evidence

`/navigation/sample_sites` remains a durable JSON String topic, now
`schema_version: 2`. `objects` is confirmed-only, `tentative_objects` contains
pending/unverified tracks, and health/reasons, counts, defaults and timing are
included. Track IDs combine epoch and numeric ID. Position covariance is in
odom; `map_position` and `map_covariance` use the current rigid map correction.
Old version-one payloads are not displayed as confirmed by the new page.

`/navigation/sample_site_markers` is confirmed-only. The optional RViz display
subscribes to `/navigation/tentative_site_markers`. The saved `sample-sites.json`
contains all states under `tracks`, including retired tracks; atomic snapshots
are written every second. `sample-sites-events.jsonl` is append-only and contains
configuration, observations, association decisions, independent support,
contradictions and transitions. No new motion service or Nav2 goal is introduced.
Neither object topic changes terrain costmaps. Existing depth hazards and all
guard thresholds retain their settings. The NPU detector is reused unchanged.

## Bounded simulation acceptance

After inspecting the launch pad, use a fresh scene 50000 session. Inside the
navigation container, with ROS and the navigation install sourced:

```bash
python3 scripts/halo_navigation_tracking_check.py \
  --session navigation_sessions/NEW_SESSION \
  --output navigation_sessions/NEW_SESSION/checks/tracking-001.json \
  --settle 15 --confirm-clear-launch
```

This starts the independent observer before mission operation, stops through
the guard, confirms only the inspected launch area, and selects guarded manual
control. It translates 0.45 m using estimated pose at 0.05 m/s, turns through a
full estimated revolution at <=0.30 rad/s, then stops. Simulator truth is used
only for recording, reference scoring and abort comparisons. Any safety stop,
pose disagreement above 0.10 m / 5 degrees, stage timeout or the total 90 wall
second limit fails the trial; the helper never rearms and continues after a fault.

Three fresh trials must establish at least one confirmed rock before turning,
correct confirmed IDs with fresh associated observations afterward (increased
last-observation timestamps and <=0.6 simulated seconds old), no duplicate or false confirmations in the
independent reference evaluation, and final zero commands. Reference scoring
allows the object's planar radius plus 0.10 m around its scene position because
depth measures a visible surface, not its center. Unconfirmed full-scene objects
include out-of-view objects and are not a visibility-qualified recall statistic.
Save synchronized terrain audits, camera/desktop captures and per-trial logs.
Interrupted trials remain failed and cannot stand in for successful acceptance.

Replay the recorded projected observations against independent scene references:

```bash
python3 scripts/halo_navigation_tracking_replay.py \
  --events navigation_sessions/SESSION/sample-sites-events.jsonl \
  --trial navigation_sessions/SESSION/checks/tracking-001.json \
  --scene /scenes/scenes/scene_050000/scene.json \
  --output navigation_sessions/SESSION/checks/replay-001.json
```

Replay rejects a different recorded threshold configuration, reports positive
and negative observations separately, and requires both a valid confirmed rock
and negative observations rejected for missing raised support. This checks
every recorded frame for false/duplicate confirmations, including tracks later
retired. Raw geometry is checked by the
synchronized audits and geometry regressions. Scene labels never enter tracking.

Windows browser regression through the existing SSH forward:

```powershell
node scripts/navigation_browser_check.cjs navigation_sessions/NEW_SESSION/checks/browser-001 --tracking --coverage
```

This checks hidden-by-default tentative markers, Show/hide behavior and fresh
coverage. `--drive` adds the existing bounded manual-control and Stop checks.
Arms, sampling, autonomous target approach, model retraining and camera promotion
are separate work. Source tests alone do not establish Martian tracking quality.
