# Architecture and engineering decisions

## Data and compute ownership

Gazebo supplies aligned RGB, depth and calibration. RGB-D odometry estimates `odom → base_link`; RTAB-Map supplies `map → odom`. Neither consumes simulator ground truth for navigation. Independent evaluators use truth to measure drift, travel and return accuracy.

| Component | Compute | Role |
| --- | --- | --- |
| YOLOv8s learned backbone/heads | AMD NPU | Four-class predictions |
| Image preparation, DFL/NMS, drawing/JPEG | CPU | Explicit detector-side processing |
| Depth projection/classification/resampling | AMD GPU / ROCm | Geometric terrain processing with a CPU reference |
| Terrain memory and coverage | CPU | Preserve observations, unknown cells and hazard reasons |
| RGB-D odometry / RTAB-Map | CPU | Motion estimation and map corrections |
| Smac State Lattice / Regulated Pure Pursuit | CPU | Feasible route planning and path following |
| Independent motion guard / wheel watchdog | CPU | Sole command ownership and fail-closed expiry |

The current navigation launch has no EKF/UKF sensor-fusion node. Legacy wheel/IMU fusion requires its own validation.

## Geometry and motion contracts

The simulated vehicle's measured turning center lies 0.13 m ahead of `base_link`. Planning, motion sweeps and mission poses use `drive_center`, with the full chassis/wheel envelope and 0.025 m padding. Testing a single center point would miss rear and corner collisions during rotation.

Only the navigation guard publishes `/cmd_vel`; only the Twist stamper publishes the wheel-controller command. Manual and autonomous sources send requests through separate guard inputs. Browser control uses held input and expiring leases. Stop, stale sensors, clock faults and invalid localization latch a stop; planning never rearms the guard automatically.

Depth geometry marks terrain unsafe above a 12° slope or 3 cm per-cell height span. Unknown terrain is blocked. An operator can confirm the inspected 1.1 m launch disk once per session; this is an explicit clearance assumption, not camera-observed terrain. Measured hazards override launch seeding.

Confirmed terrain hazards currently persist for the session. The proposed reversible-evidence extension is not implemented. A depth ray through empty space does not prove that supporting ground exists at a destination.

## Object confirmation and association

Immediate camera boxes and confirmed map objects have different responsibilities. Phase Three requires three translated views, 0.20 m baselines, a two-second span, class consistency and depth-supported positive relief before confirming eligible rocks. Position covariance, covariance intersection, mutual association and appearance/extent checks reduce correlated evidence and duplicate IDs.

The automatic classes are `rock` and `yellow_crystal_rock`. Soil patches and slabs remain tentative. Healthy, fully visible contradictory observations can demote/retire a track; occlusion or leaving view cannot. An existence score is engineering evidence, not calibrated detection accuracy.

Tracking does not turn semantic labels into traversability or clear terrain hazards. All states and decisions are persisted for audit. [Tracking policy →](NAVIGATION_PHASE_THREE.md)

## Failure isolation and deployment

ROS runs in Jazzy/Harmonic Docker; NPU and ROCm services remain in separate host environments. Both services bind to localhost. The terrain RPC uses bounded numeric arrays rather than pickle. The NPU adapter checks source/model hashes, cache/runtime fingerprints, operator assignment and execution profiles, rejecting learned CPU fallback.

This separation avoids conflicting vendor Python packages and leaves GPU compute available for simulation and terrain work. It is not an energy-efficiency claim. Historical test outcomes and unresolved gates are in [verification](VERIFICATION.md).
