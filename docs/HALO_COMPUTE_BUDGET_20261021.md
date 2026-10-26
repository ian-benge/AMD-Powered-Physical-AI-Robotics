# Halo compute budget — measured October 21, 2026

This is the baseline collected **before** the subsequent [GPU terrain deployment](HALO_GPU_TERRAIN.md). Its CPU terrain measurements describe that earlier implementation.

The current browser rover implementation sustains approximately 30 detector FPS. Object detection executes on the AMD NPU; terrain mapping, RGB-D odometry, SLAM, navigation, and control currently execute on the CPU. Gazebo rendering uses the GPU. GPU terrain segmentation and learned depth estimation are future work, not current deployed components.

## Collection conditions

Read-only sampling on the Ubuntu host, kernel `7.0.0-38-generic`, with 32 logical CPUs. Collected 61 snapshots at approximately one-second intervals over **59.988 seconds**, beginning **2026-10-21 19:23:10 UTC**. Gazebo server and GUI, RViz, browser services, terrain mapping, SLAM, and the existing detector remained running. No launches, workload changes, or movement commands were issued.

The rover was stationary in manual mode, with zero commanded velocity; the autonomous mission remained idle. These results describe this operating state, not peak autonomous navigation or a moving-camera workload.

Raw evidence and calculated statistics are saved in `navigation_sessions/compute-budget-20261021T192310Z/raw.json` and `summary.json` in this checkout. Raw data is also on the Halo under `/home/ian/capstone-training/rover-navigation/navigation_sessions/compute-budget-20261021T192310Z/`.

## Actual placement and CPU support costs

CPU figures below use process CPU-time differences. **1.00 CPU equivalent means one logical CPU continuously occupied**, not 100% of the whole processor. RSS is per-process resident memory and includes shared pages; do not add RSS values to estimate total RAM.

| Component | Actual compute placement | Average CPU equivalents | End-of-sample RSS |
|---|---|---:|---:|
| YOLOv8s A16W8, 576×960 | NPU, with CPU preprocessing/decoding | 0.306 | 593 MiB |
| ROS detector transport/annotation | CPU | 0.327 | 243 MiB |
| Terrain mapping | CPU NumPy | 0.455 | 224 MiB |
| RGB-D odometry | CPU | 0.874 | 447 MiB |
| RTAB-Map SLAM | CPU | 0.489 | 1,333 MiB |
| Motion guard | CPU | 0.267 | 100 MiB |
| Mission manager | CPU | 0.242 | 111 MiB |
| Nav2 planner | CPU; autonomous mission idle | 0.097 | 83 MiB |
| Nav2 controller | CPU; autonomous mission idle | 0.065 | 79 MiB |
| Nav2 behavior-tree navigator | CPU; autonomous mission idle | 0.061 | 82 MiB |
| Gazebo server | GPU rendering plus CPU simulation | 0.935 | 585 MiB |
| Gazebo GUI | GPU rendering plus CPU work | 2.002 | 741 MiB |

The NPU service is PID `4010482`, listening on localhost port `8775`, and has `/dev/accel/accel0` open, with no GPU render device open. Its status identifies the preserved YOLOv8s A16W8 model, verified learned computation offload, and NPU execution proof. XRT samples taken during the collection showed its completions increasing from **307,489 to 307,580**, with zero reported errors. A second, older NPU context (PID `15279`) had unchanged counters; XRT's `Active` context label alone must not be interpreted as ongoing computation.

The active terrain source imports NumPy and processes depth into traversability grids, with a 0.2-second update timer. It does not use a GPU tensor backend. RTAB-Map, odometry, and the Nav2 processes inspected had no GPU device handles. Gazebo server and GUI had `/dev/dri/renderD128` open. Placement conclusions combine source/configuration inspection with live process evidence, rather than attributing all GPU load to navigation.

## Throughput and scheduling budget

| Measurement | Result |
|---|---:|
| Successful detector frames | 1,797 |
| Detector throughput, using counter differences / monotonic elapsed time | **29.956 FPS** |
| Sampled detector-service processing time, mean / p95 | **28.04 / 29.56 ms** |
| 30 FPS detector frame interval | 33.33 ms |
| Nominal remaining interval after mean / p95 service time | **5.29 / 3.77 ms** |
| Sampled terrain-update time, mean / p95 | **48.68 / 54.79 ms** |
| Configured terrain-update interval | 200 ms, or 5 Hz in simulation time |
| Mean terrain-update duration / configured interval | **24.3%** |
| Sampled depth age at the motion guard, mean / p95 | **104.15 / 106.32 ms** |
| Detector/browser/guard readiness | True in every sampled status |
| Detector errors / processing-session resets | None observed |

Detector latency is the service's `last_processing_ms`: preprocessing, model execution, decoding, and row validation. It excludes request upload/body reading, response transmission, ROS publication, drawing/JPEG encoding, and browser delivery. It is **not pure NPU execution time**. The service and ROS detector counters each advanced by 1,797, supporting successful downstream processing during this window.

The latency mean and p95 describe **61 latest-value snapshots**, not a trace of all 1,797 frames. They are preliminary budgeting statistics; they cannot certify worst-case deadlines or per-frame tail latency. The remaining 3–5 ms is a nominal detector scheduling margin, not measured spare NPU capacity or a complete camera-to-browser budget. NPU utilization percentage remains unavailable because the loaded driver does not export busy-time counters.

Mapping and detection operate on separate schedules and hardware paths; do not sum their durations into a single 33.33 ms frame deadline. The motion guard has a configured 50 ms wall-clock tick (20 Hz); Nav2's controller is configured for 20 Hz when active. This observation does not establish their worst-case execution time or deadline compliance while navigating.

## Whole-host resources and power

| Resource | Measured result |
|---|---:|
| Aggregate CPU busy, from `/proc/stat` deltas | **26.70%**, approximately 8.54 logical CPU equivalents |
| GPU busy, sampled mean / p95 | **16.28% / 17%** |
| System RAM used, mean / observed maximum | **10.20 / 10.33 GiB** of 62.43 GiB |
| GPU-accounted VRAM, sampled mean | **2.24 GiB** |
| Shared APU PPT power, sampled mean / p95 | **84.62 / 85.04 W** |

CPU and GPU readings include desktop, browser, GUI, and other host activity. RAM uses `MemTotal - MemAvailable`; GPU-accounted memory is not an independent allocation pool to add to system RAM on this shared-memory APU. GPU busy is an activity metric, not a linear estimate of spare compute throughput.

Power comes from the `amdgpu` hwmon `PPT` sensor (`power1_average`, microwatts converted to watts). This is shared APU/package telemetry, **not isolated NPU power and not wall power**. Dividing 84.62 W by 29.956 frames/s yields approximately 2.82 package joules per delivered detector frame **while all these other workloads are running**. That ratio cannot be attributed to the detector, used to claim NPU efficiency, or compared fairly against a GPU run without a matched controlled comparison.

## Implications

1. The NPU meets the current camera's approximately 30 FPS rate in this stationary observation. Keep inference independent of mapping and controls; the service has modest nominal margin at this rate.
2. Terrain and navigation have not yet been moved to GPU compute. Establish a separate GPU depth/segmentation implementation and benchmark it alongside the NPU detector before allocating that budget.
3. A moving, actively planning trial is required to estimate peak SLAM, mapping, and Nav2 costs. Collect per-frame timing distributions and control deadline/acknowledgment measurements during that trial.
4. Gazebo's GUI alone consumed about two CPU equivalents here. Headless GPU-rendered simulation is a reasonable subsequent comparison when the browser is the primary view. No GUI was stopped for this measurement.
5. Physical OAK-D deployment will remove simulator rendering costs and change depth/transport costs. This simulator budget must be measured again with the real camera.

No application, runtime, driver, firmware, power-mode, or control configuration was changed to collect these results.
