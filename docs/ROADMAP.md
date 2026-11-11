# Roadmap

## Delivered through November 10, 2026

- Guarded browser manual driving, RGB-D mapping, actual four-class NPU inference, float64 GPU terrain processing and confirmed-rock/revisit tracking have bounded deployment evidence.
- The November rendering phase corrects opaque-material depth behavior and verifies the deployed Ogre camera/material path. [Rendering acceptance](PHASE1_RENDERING_ACCEPTANCE_20261104.md).
- Reachable version 1 terrain uses a shared visual/collision surface, reproducible layouts and evaluator-only route certificates. Fifty seeded layouts and three attended representatives were reviewed. [Terrain acceptance](PHASE2_TERRAIN_ACCEPTANCE_20261104.md).
- Adaptive terrain evidence can clear weak artifacts after repeated independent observations of safe supporting ground. Established hazards stay blocked, lost sensors/localization prevent clearing, and a cleared cell does not release a latched stop. Actual CPU/ROCm ROS replay and a bounded synchronized rendered review are recorded in [mapping acceptance](PHASE3_MAPPING_ACCEPTANCE_20261105.md) and [the policy](ADAPTIVE_TERRAIN.md).
- The 200-image pilot passed automated capture, annotation and owned-process cleanup checks. The full campaign reserves 500 scenes, 20 views each, in 50 resumable batches. Immutable attempts, hash-checked resume and automatic protected monitoring renewal are implemented. [Pilot results](PILOT200_PROCESS_CLEANUP_FIX_20261106.md), [continuous collection](BULK_CONTINUOUS_COLLECTION_GUIDE.md).

The November rendering/terrain/mapping phases are a separate sequence from the older navigation Phase Two/Three reports. Their scoped acceptance does not resolve every historical localization, contact or autonomous-mission failure.

## Detector candidate: trained and evaluated, further review pending

The operator stopped further capture on November 8 after **5,000 accepted images in 25 batches**, including 1,000 background frames. Both monitoring windows of the last invocation closed cleanly. All new captures belong to the training split; the original 10,000-image reservation remains incomplete. A separate snapshot now contains **5,300 training, 100 validation and 100 test images**, with the original evaluation labels and source split assignments preserved. Image decoding, export hashes, split isolation, Linux checks and a four-class ROCm training/NMS probe passed. The owner requested experimental candidate training while complete new-image visual review remains outstanding. See the [preparation evidence](CAPTURE5000_TRAINING_READINESS_20261108.md) and [training guide](CAPTURE5000_TRAINING_GUIDE.md).

1. Preserve the completed batches, partial batch 26 and failed attempts; keep further capture stopped for this training candidate.
2. Review images and labels, class/view coverage, backgrounds and rejected attempts. Preserve raw captures and audit reversible edits.
3. The separately pinned YOLOv8s candidate trained for 28 saved epochs before the operator chose to assess its validation-selected epoch-22 checkpoint. Matched GPU and NPU comparisons are complete: NPU test mAP50-95 increased from 76.67% to 78.90%, while mudstone test AP decreased. Export/quantization parity, learned-compute NPU offload, matched validation accuracy gates and a guarded HTTP inference check passed. [Metrics and assessment starter](CAPTURE5000_MODEL_ASSESSMENT.md).
4. Assess the separately pinned candidate through the guarded browser simulator, then review held-out new scenes and hard negatives before global model promotion. Retain the legacy source IDs and unchanged baseline detector. Collection completion alone does not approve a model.

## Environment v2: implemented, experimental acceptance

The versioned 40 x 40 m environment adds a continuous 20 m apron, larger maps, seeded appearance and three density presets. It became the separate Halo candidate's default by explicit operator request; the accepted 20 m launcher remains available for rollback. [Environment closeout](SIMULATION_ENVIRONMENT_V2_CLOSEOUT.md).

Offline geometry, compatibility, texture allocation and actual CPU/ROCm mapping checks passed. The recorded sparse run reached 13.109 m along an axis and returned within 0.677 m of launch, with reviewed ground error below the retained 15 mm bound. Its p95 local-publication latency was **267.73 ms**, above the **200 ms** target. The subsequent live optimization, remaining standard/cluttered drives and recorded progressive speed stages remain unqualified. Smooth operator observation does not close those gates. Historical cooldown-monitoring gaps remain documented.

## Autonomous demonstration: implemented, not accepted

The separate candidate implements fresh-map autonomous exploration, a budgeted
first return, detected-soil or map-click target selection, preview/approval,
travel, ten seconds of simulated collection, and automatic return with a sample
record. It adds route/trail layers, mission status and a typed command service.
Replay, browser and real ROS interface checks support the implementation; they
do not establish successful end-to-end operation. [Mission guide](AUTONOMOUS_DEMO_GUIDE.md).

The first real autonomous rehearsal froze during the first return after 3.443 m
of estimated travel. The protected logs establish an unclean interruption but no
initiating cause. There is no successful home return, sample delivery or accepted
cycle. [Freeze investigation](AUTONOMOUS_DEMO_FREEZE_20261110.md).

The later GUI-off candidate removes continuous exterior-window rendering while
retaining 15 Hz Ogre RGB-D. Its five-minute stationary check recorded mean Tctl
80.82°C, peak 83.125°C, worst reported rolling publication p95 165.65 ms and
reported median RTF 0.996. It did not meet the sustained-below-80°C target or
establish a matched cooling benefit. An initial odometry-fault latch was retained.
[Thermal results and plots](AUTONOMOUS_DEMO_BROWSER_PROFILE.md).

Next steps are platform stability and startup investigation, thermal margin,
unseen-scene detector review, then fresh bounded exploration/return and sampling
cycles. Complete the mission guide's speed, drift, contact, recording and variation
gates before acceptance. Simulated collection remains a timed event; there is no
physical arm or material-transfer claim. The [AMD testing guide](AMD_TESTING_GUIDE.md)
provides the current reproduction and issue matrix.

## Later milestones

Long-duration mapping and stability, robust startup across ramp/edge fixtures, physical OAK-D calibration, wheel/IMU fusion, frontier-exploration acceptance, moving-object handling, physical sensor/actuator integration, arms and actual sample collection require separate work and evidence.
