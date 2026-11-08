# Documentation

Start with [Getting started](GETTING_STARTED.md), [Architecture](ARCHITECTURE.md), and [Verification](VERIFICATION.md). These describe this publication's supported entry points and current acceptance boundaries.

| Topic | Guide |
| --- | --- |
| Current milestone and planned work | [Roadmap](ROADMAP.md) |
| Manual mapping / detector integration | [Operator guide](HALO_MANUAL_MAPPING_DETECTIONS.md) |
| Confirmed rocks and repeat recognition | [Phase Three](NAVIGATION_PHASE_THREE.md) |
| Navigation / ROS interfaces | [Navigation guide](NAVIGATION.md) |
| Implemented waypoint/return workflow | [Demo guide](NAVIGATION_DEMO.md) |
| NPU export, quantization and runtime | [NPU workflow](../scripts/HALO_NPU_TESTING.md) |
| ROCm depth geometry | [GPU terrain](HALO_GPU_TERRAIN.md) |
| Synthetic scenes / capture / labels | [Environment package](../src/mars_training_environment/README.md) |
| Dataset review / training | [Pilot workflow](../scripts/HALO_PILOT50_V2.md), [balanced 500-image workflow](../scripts/HALO_COVERAGE500_BALANCED.md) |
| Rendering/material correction and camera | [Phase 1 guide](PHASE1_RENDERING_GUIDE.md), [acceptance](PHASE1_RENDERING_ACCEPTANCE_20261104.md) |
| Reachable shared-surface terrain | [Phase 2 guide](PHASE2_TERRAIN_GUIDE.md), [acceptance](PHASE2_TERRAIN_ACCEPTANCE_20261104.md) |
| Adaptive mapping/publication checks | [Phase 3 guide](PHASE3_MAPPING_GUIDE.md), [acceptance](PHASE3_MAPPING_ACCEPTANCE_20261105.md) |
| Experimental 40 m environment | [Operator guide](SIMULATION_ENVIRONMENT_V2_GUIDE.md), [open gates and closeout](SIMULATION_ENVIRONMENT_V2_CLOSEOUT.md) |
| New 200-image pilot | [Pilot guide](PILOT200_COLLECTION_GUIDE.md), [cleanup acceptance](PILOT200_PROCESS_CLEANUP_FIX_20261106.md) |
| Full image collection and automatic renewal | [Campaign guide](FULL10000_COLLECTION_GUIDE.md), [continuous starter](BULK_CONTINUOUS_COLLECTION_GUIDE.md), [acceptance](BULK_CONTINUOUS_ACCEPTANCE_20261107.md) |
| Capture recovery at the depth cutoff | [November 8 repair and regression](BULK_DEPTH_BOUNDARY_FIX_20261108.md) |
| November source and deployment evidence | [Development evidence](../evidence/development-20261107/README.md) |
| Original laptop webcam prototype | [Webcam example](../examples/webcam/README.md) |

## Historical deployment notes

Dated `HALO_*REPORT*` files and the NPU/GPU deployment guides record the tested Halo environment. Session names, paths, PIDs and “currently running” statements in those documents describe their recorded dates, not a service running for GitHub readers. Public evidence is curated; omitted local recordings are identified explicitly.

The [integrated development README](archive/INTEGRATED_README_20261025.md), [original dependency notes](archive/DEPENDENCIES_ORIGINAL.md), and [legacy workflows](LEGACY_WORKFLOWS.md) preserve earlier context. Follow Getting started for a new checkout.

Supporting paths under `navigation_sessions/`, full RGB-D archives, generated datasets, trained artifacts and internal handoff notes are local deployment records and are omitted from Git. Selected November summaries are published under `evidence/development-20261107/`; an omitted artifact is not a new acceptance claim.
