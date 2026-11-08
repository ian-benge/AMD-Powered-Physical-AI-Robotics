# Adaptive terrain evidence

The first Phase Four terrain change can reconsider weak red cells when closer
depth observations positively demonstrate safe ground. Raw detector labels do
not participate in this decision. Unknown cells and missing depth stay blocked.

## Clearing requirements

All of these conditions must hold for an individual retained hazard cell:

- Three supporting camera positions, each at least 0.20 m from every previously
  counted position, spanning at least two simulated seconds. Rotation and
  repeated video frames do not add viewpoints. Partial evidence expires after
  30 simulated seconds without clearing the hazard.
- Each view is within 3 m horizontally and at least 0.25 m and 20% closer than
  the closest qualified observation that marked the hazard.
- The cell and the complete surrounding square, extending at least 10 cm in
  each direction, contain measured safe ground. Every cell has at least three
  depth samples, slope at most 8 degrees, and height span at most 1.5 cm.
- The new surface elevation agrees within 1.5 cm with the lowest measured
  elevation from the hazard observations. Seeing only an obstacle's raised top
  must not clear its ground-level evidence.
- Image-time odometry covariance is available and valid, with position
  uncertainty at most 0.10 m and yaw uncertainty at most 5 degrees. Odometry
  receipt and the map correction must also be fresh. An unexplained pose jump
  suspends clearing until the session restarts.

The ordinary hazard thresholds remain **12 degrees and 3 cm**. Clearing uses
stricter positive evidence rather than relaxing those thresholds.

## Evidence that remains blocked

A hazard observed from three independent translated viewpoints becomes
protected in this first implementation. Hazards with unknown provenance or
observed while clearing localization checks fail are also protected. This
policy can retain false red cells; it favors keeping a possible obstacle over
clearing it from ambiguous measurements.

Any new measured hazard immediately invalidates pending clear evidence for
that cell. Current-frame hazards still stop motion before persistence
confirmation. Lost localization or stale accepted depth resets pending clearing
views. Missing observations and elapsed time never make a cell safe.

Once a cell qualifies, its retained cost is replaced by its freshly measured
safe cost, and its old hazard hits and diagnostic cause flags are reset.
Unrelated cells are unchanged. Local and global maps are then derived from
the same revised odometry-frame memory, including the current SLAM correction.

Clearing **does not rearm a safety stop**. The operator must inspect readiness
and explicitly select a mode again. The tracker, mission goals, command
ownership and arm interfaces are unchanged.

## Launch, status and audit

`adaptive_terrain:=true` is the navigation default. Use
`adaptive_terrain:=false` for a comparison with persistent hazard memory. Both
settings retain the immediate depth and footprint checks. Restart creates new
terrain evidence; historical state is not restored as confirmed clearance.

The dashboard shows cleared/pending counts or a suspension reason under
**Terrain reconsideration**. `/navigation/mapping_status` includes the same
`terrain_evidence` object. Each new session saves configuration, qualified view
increments, clearing decisions, cell indices and supporting camera positions
to `terrain-evidence-events.jsonl`. No simulator truth enters clearing.

The source tests cover independent views, rapid views followed by stationary
frames, sparse/unknown/rough/raised ground, closer-view requirements, protected
hazards, changing window origins, map projection, localization failures, and
retention of other obstacles and latched stops. These tests establish policy
behavior, not field traversability acceptance. See the
[camera investigation and deployment report](HALO_CAMERA_ADAPTIVE_REPORT_20261027.md).
