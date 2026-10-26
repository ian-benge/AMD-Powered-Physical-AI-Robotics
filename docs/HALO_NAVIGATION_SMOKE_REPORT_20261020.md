# Halo navigation smoke tests - October 20, 2026

The first bounded manual simulation checks passed on `halo-strix`. The rover is
left stopped, with fresh sensor data, in the running `rover-navigation` container.
Autonomous waypoint following, return-home and physical collision acceptance
have not yet been established.

## Environment and changes

- Source: `navigation/mapping-and-exploration` in `AMD-Capstone-navigation`.
- Runtime: Jazzy/Harmonic Docker image, ROS domain 87, flat fixture,
  RGB-D 960x540, read-only preview at `http://127.0.0.1:8773/frame.png`.
- Halo session: `~/capstone-training/rover-navigation/navigation_sessions/agent-manual-01`.
- Five ROS packages built successfully on the Halo. The navigation and existing
  camera-profile source checks passed: **51 tests on Linux**, and 51 tests plus
  39 subtests in the Windows verification run.
- Fixed a contact-topic integration bug before driving. Harmonic's Contact
  system reads `<contact><topic>`, while the original description set only the
  generic sensor topic. Gazebo was advertising a scoped topic and the ROS bridge
  was subscribing to `/navigation/chassis_contacts`, which had no native publisher.
  The description now sets the contact-specific topic as well. Native publisher
  and bridge subscriber were verified at the same topic after restart.
  The routing follows the [SDF contact-topic schema](https://raw.githubusercontent.com/gazebosim/sdformat/sdf14/sdf/1.10/contact.sdf).
- Added `scripts/halo_navigation_smoke.py` and `scripts/halo_navigation_map_check.py`
  to produce repeatable reports and raw event evidence.

## Recorded results

| Check | Observation | Result |
| --- | --- | --- |
| Preflight | Ground/pose/truth available; stopped and ready; only the motion guard publishes `/cmd_vel`; only Twist stamper publishes the controller command | Passed |
| One-second forward burst, 0.05 m/s | Truth displacement 0.0480 m; estimated displacement 0.0341 m; stop service accepted; final command zero | Passed for bounded manual check |
| Half-second burst followed by command silence | Truth displacement 0.0400 m; estimate 0.0457 m; guard latched `command_timeout` | Passed |
| Command-lease stop timing | First observed zero guard command 0.350 s after publication ended | Passed; this is command timing, not braking distance |
| Stationary Gazebo pause | Guard latched `sensor_or_clock_timeout`, with zero command and a stopped clock | Passed |
| Resume Gazebo | Sensor readiness recovered; motion remained stopped until explicit selection | Passed |
| Nav2 lifecycle | Planner, controller and BT navigator all active | Passed |
| Read-only path request | 0.35 m path, five poses, successful planner action, every sampled footprint on known clear ground | Passed; path was not driven |

The two drive tests together moved the simulated rover about 8.8 cm. Their
relative displacement disagreements were 1.39 cm and 0.57 cm. These are short
smoke-test observations, not a scale, drift or accuracy qualification. There
were no readiness failures in those drive recordings; maximum accepted-ground
age was 0.427 s and 0.650 s respectively. Longer timing acceptance remains open:
the preceding operator rate capture included a 1.196-second reception gap.

## Map preview and scope

The inspected terrain map has 4,242 known 5 cm cells and no encoded hazards in
the flat fixture. Known ground includes the explicitly confirmed 1.1 m launch
clearance; the green circle is not a claim that the camera observed beneath the
rover. Unknown terrain remains blocked. The blue rover, orange proposed goal and
planned path are shown in the [map preview](../evidence/agent-manual-01/checks/map-plan-001.png).

Harmonic contacts are event-only: silence is expected with no chassis contact,
as implemented in the [Contact system](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/src/systems/contact/Contact.cc).
The connected transport and valid configuration do not establish physical
collision detection. A controlled physical-contact test remains required. These
runs do not claim zero wheel/obstacle collisions or hardware readiness.

## Evidence

JSON reports and raw JSONL streams are saved under the session's `checks/` folder:

- [Preflight](../evidence/agent-manual-01/checks/preflight-001.json)
- [Forward movement and explicit stop](../evidence/agent-manual-01/checks/forward-service-001.json)
- [Command-lease stop](../evidence/agent-manual-01/checks/forward-lease-001.json)
- [Clock pause/resume](../evidence/agent-manual-01/checks/clock-pause-001.json)
- [Map and planner](../evidence/agent-manual-01/checks/map-plan-001.json)
- [Final stopped status](../evidence/agent-manual-01/checks/final-status.yaml)
- [Linux source tests](../evidence/agent-manual-01/checks/linux-source-tests.txt)

The evidence is also copied into the local navigation checkout. Session files
are ignored by Git and should be retained with exported test evidence.

## Next acceptance gates

1. Controlled chassis-contact probe, including bridge/guard reaction, without
   claiming that the chassis sensor covers every collision.
2. A bounded waypoint-and-return trial within verified ground, with the existing
   independent evaluator running before mission start.
3. Manual mapping loop, persisted map/relocalization, terrain fixtures, loss of
   depth/localization, clock reset and manual takeover during a mission.
4. Three-scene repeated waypoint/return tests, exploration and sustained timing
   with and without inference. Arms and physical hardware remain separate work.
