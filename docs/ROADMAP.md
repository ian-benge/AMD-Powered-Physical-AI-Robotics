# Roadmap

## Delivered through November 7, 2026

- Guarded browser manual driving, RGB-D mapping, actual four-class NPU inference, float64 GPU terrain processing and confirmed-rock/revisit tracking have bounded deployment evidence.
- The November rendering phase corrects opaque-material depth behavior and verifies the deployed Ogre camera/material path. [Rendering acceptance](PHASE1_RENDERING_ACCEPTANCE_20261104.md).
- Reachable version 1 terrain uses a shared visual/collision surface, reproducible layouts and evaluator-only route certificates. Fifty seeded layouts and three attended representatives were reviewed. [Terrain acceptance](PHASE2_TERRAIN_ACCEPTANCE_20261104.md).
- Adaptive terrain evidence can clear weak artifacts after repeated independent observations of safe supporting ground. Established hazards stay blocked, lost sensors/localization prevent clearing, and a cleared cell does not release a latched stop. Actual CPU/ROCm ROS replay and a bounded synchronized rendered review are recorded in [mapping acceptance](PHASE3_MAPPING_ACCEPTANCE_20261105.md) and [the policy](ADAPTIVE_TERRAIN.md).
- The 200-image pilot passed automated capture, annotation and owned-process cleanup checks. The full campaign reserves 500 scenes, 20 views each, in 50 resumable batches. Immutable attempts, hash-checked resume and automatic protected monitoring renewal are implemented. [Pilot results](PILOT200_PROCESS_CLEANUP_FIX_20261106.md), [continuous collection](BULK_CONTINUOUS_COLLECTION_GUIDE.md).

The November rendering/terrain/mapping phases are a separate sequence from the older navigation Phase Two/Three reports. Their scoped acceptance does not resolve every historical localization, contact or autonomous-mission failure.

## Current work: collection, review and export

At the November 7 continuous-run handoff, nine batches and **1,800 accepted images** were verified; batch 10 was running. This dated count is not a completion claim. The campaign targets 8,000 training, 1,000 validation and 1,000 test images, including 20% backgrounds with reserved scene/asset splits.

1. Complete the remaining batches with the existing renderer, geometry labels and quality checks. Investigate genuine faults rather than bypassing them.
2. Review images and labels, class/view coverage, backgrounds and rejected attempts. Preserve raw captures and audit reversible edits.
3. Produce the metadata-based four-class export, including `mudstone_slab`, while preserving legacy three-class source IDs. Verify split isolation and export hashes before approving training.
4. Evaluate a detector update on held-out scenes and hard negatives, then separately qualify ONNX/quantized NPU deployment. Collection completion alone does not approve a model.

## Environment v2: implemented, experimental acceptance

The versioned 40 x 40 m environment adds a continuous 20 m apron, larger maps, seeded appearance and three density presets. It became the separate Halo candidate's default by explicit operator request; the accepted 20 m launcher remains available for rollback. [Environment closeout](SIMULATION_ENVIRONMENT_V2_CLOSEOUT.md).

Offline geometry, compatibility, texture allocation and actual CPU/ROCm mapping checks passed. The recorded sparse run reached 13.109 m along an axis and returned within 0.677 m of launch, with reviewed ground error below the retained 15 mm bound. Its p95 local-publication latency was **267.73 ms**, above the **200 ms** target. The subsequent live optimization, remaining standard/cluttered drives and recorded progressive speed stages remain unqualified. Smooth operator observation does not close those gates. Historical cooldown-monitoring gaps remain documented.

## Next navigation milestone: reviewed soil-site approach

The intended workflow is manual mapping, soil-patch detection and evidence checks, operator approval of an outward/return preview, an approach beside the patch, stopped waiting, and explicit Return home.

Required work includes soil-specific repeated observations and identity uncertainty, a full-footprint approach/return through known ground, a stopping pose outside the patch, and retained Stop/takeover/late-response protections. A flat soil patch cannot use a raised-rock geometry test; visual class does not establish material composition. Qualify launch clearance, localization, contact wiring, blocked routes and repeated bounded missions before accepting autonomous approach. Existing waypoint/exploration APIs do not establish that acceptance.

## Later milestones

Long-duration mapping and stability, robust startup across ramp/edge fixtures, physical OAK-D calibration, wheel/IMU fusion, frontier-exploration acceptance, moving-object handling, physical sensor/actuator integration, arms and actual sample collection require separate work and evidence.
