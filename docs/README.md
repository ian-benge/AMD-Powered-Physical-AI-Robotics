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
| Original laptop webcam prototype | [Webcam example](../examples/webcam/README.md) |

## Historical deployment notes

Dated `HALO_*REPORT*` files and the NPU/GPU deployment guides record the tested Halo environment. Session names, paths, PIDs and “currently running” statements in those documents describe their recorded dates, not a service running for GitHub readers. Public evidence is curated; omitted local recordings are identified explicitly.

The [integrated development README](archive/INTEGRATED_README_20261025.md), [original dependency notes](archive/DEPENDENCIES_ORIGINAL.md), and [legacy workflows](LEGACY_WORKFLOWS.md) preserve earlier context. Follow Getting started for a new checkout.
