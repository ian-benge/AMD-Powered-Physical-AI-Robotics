# Dependencies

The integrated navigation target is **Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic**. Earlier Humble/Classic workflows are historical and remain documented in [the original dependency notes](docs/archive/DEPENDENCIES_ORIGINAL.md).

| Mode | Requirements |
| --- | --- |
| CPU source tests | Python 3.12 and `requirements-test.txt` |
| Procedural world generation | CPU test dependencies; no ROS or downloaded scans needed for `config/portfolio_demo.json` |
| ROS simulation | Linux Docker; build `docker/Dockerfile.jazzy` then `docker/Dockerfile.navigation` |
| GUI simulation / RViz | Active X11-compatible Ubuntu desktop and display authentication |
| NPU detector | Separately configured AMD Ryzen AI / VitisAI runtime, matched XRT, trained checkpoint and verified quantized manifest |
| GPU terrain | Separately configured ROCm-compatible PyTorch environment and AMD GPU |
| Webcam example | Its own setup and requirements under `examples/webcam` |

The historical Halo runtime used Ryzen AI 1.8, ONNX Runtime VitisAI 1.27.0, XRT 2.25.37 and a Ryzen AI Max+ 395. Runtime/driver/model changes require fresh execution evidence. CPU test dependencies must not be installed into the vendor NPU environment.

The NPU export and runtime environments are deliberately separate. See [the NPU workflow](scripts/HALO_NPU_TESTING.md) and [GPU terrain guide](docs/HALO_GPU_TERRAIN.md) for hardware-specific instructions and measured versions.
