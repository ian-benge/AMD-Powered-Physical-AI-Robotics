# Halo dashboard follow-up - October 27, 2026

The generated-scene workflow, expanded camera view and idle manual-selection
fix were deployed and checked on Halo. The final generated scene is running,
stopped, with an empty map and launch clearance awaiting operator confirmation.
No nonzero movement command was sent by these acceptance checks.

## Manual control

Two separate causes of lost selection were corrected:

- A delayed status response captured before Enable could overwrite the newly
  selected browser session. Status responses now carry request ordering and an
  operator-action generation; obsolete responses and errors are discarded.
- A short UI pause after a zero command could expire the browser's input token.
  Idle ownership now tolerates up to ten seconds between packets. Once a
  nonzero command is accepted, the existing 0.25-second input timeout applies,
  including to a late zero packet. The independent guard's 0.35-second command
  timeout remains unchanged.

A scheduling gap over 0.25 seconds clears held directions before the browser
sends another packet. Current sensor failures, stale status, Stop, page hiding
and focus loss still end control. No response or reconnection rearms motion.
The active page continues sending zero commands while selected at rest.

## Operator features

**Expand camera** enlarges the viewing panel; **Restore layout** or Escape
returns to the normal layout. Changing the view releases held directions while
retaining healthy manual selection. Camera capture and detector input resolution
are unchanged. Corrupted direction-button and separator text was also repaired.

**Generate new scene** stops the current simulation and builds a new seeded
Martian layout from the existing asset library and environment constraints.
Generated bundles have separate paths under
`navigation_sessions/generated-scenes/`; training assets and old sessions are
retained. This operation does not capture a training dataset. The new session
starts with no mapped ground, objects, home or inherited launch confirmation.
**Restart scene** subsequently repeats that generated layout with fresh state.

Generation/restart requests are serialized. A generation or launch failure
reports an error and leaves control stopped. Inspect and explicitly confirm the
new launch pad before selecting manual mode.

## Verification

| Check | Recorded result |
| --- | --- |
| Windows navigation source suite | 189 tests run successfully; four actual-ROCm cases skipped |
| Halo Linux/ROS navigation source suite | 189 tests run successfully; same four skips |
| Halo package build | `rover_navigation` rebuilt successfully |
| JavaScript harnesses | Control lifecycle, speed controls and camera recovery passed |
| Isolated generator smoke check | New seeded layout produced in 12.59 wall seconds |
| Browser delayed-status/idle check | Manual selection survived an obsolete pre-Enable response and ten seconds at rest |
| Camera expansion | Expanded to 1,361 CSS pixels; Escape restored layout and retained selection |
| Generate | Changed `scene_244311327` to `scene_85404259`; zero known cells and zero stored objects |
| Restart | Kept `scene_85404259`, created a fresh session, and again reported an empty map |
| Command check | All 168 browser drive requests were zero commands |
| Final read-only browser check | Fresh annotated camera, stopped rover, speed defaults/caps/adjustments and Reset passed |

The browser check used an isolated Chrome profile and refused nonzero drive
requests. Before its zero-command manual checks, the existing 1.1 m launch disk
was reviewed against simulation terrain/object geometry and explicitly
confirmed. This inspection did not supply a truth map to navigation or qualify
collision detection. The final restarted session has no inherited confirmation.

Earlier browser attempts are retained: one encountered an already selected
operator session, one exposed a test-harness promise wait, and another found the
idle-token bug fixed here. After that fix, a generated-scene test reached the
new layout but checked object status before its initial publication; the test
now waits for that message. These attempts are not counted as acceptance passes.

The final capture confirms Ogre with a visible sensor context, adaptive terrain
enabled, the deployed NPU detector ready with `npu_executed` proof, and the ROCm
terrain service ready. Models, accelerator environments and driver settings
were preserved. Source files were copied selectively after baseline hash
checks, with backups retained on Halo.

## Evidence and remaining work

Halo evidence lives in `navigation_sessions/dashboard-features-20261027/` and
`navigation_sessions/dashboard-idle-fix-20261027/`. It includes source manifests
and backups, build/test logs, generation provenance, launch reviews and final
status/configuration. Browser JSON reports and normal/expanded screenshots are
saved locally under `navigation_sessions/dashboard-features-20261027/`;
`browser-features-005.json` is the completed zero-command workflow check.
Session evidence is ignored by Git. Old sessions remain intact.

The Halo evidence archive was copied back and its SHA-256 verified:
`navigation_sessions/dashboard-followup-20261027-evidence.tar.gz`,
`07a9e7831c276253f90033524af95290446fd54ace5c8dabda9949c98e1c2217`.
Its 78 files were extracted under
`navigation_sessions/dashboard-followup-recovered-20261027/`. The final
read-only browser capture after label repair is `final-speed-browser-002.png`.

Final captured session: `restart-20261027T213152Z-1ce4e5`, generated layout
`scene_85404259`. Open `http://127.0.0.1:8774` through the local SSH forward.
Inspect and confirm the launch area, wait for readiness, then explicitly enable
manual drive. Use 0.05 m/s for the next terrain-evidence trial; the operator
default remains 0.20 m/s with the requested 0.50 m/s upper limit.

The [adaptive-terrain policy](ADAPTIVE_TERRAIN.md) remains deployed, but positive
and negative moving-rover clearing acceptance is still open. New layouts,
driving at the speed cap and autonomous navigation are not qualified by these
stationary UI checks. The Ogre camera workaround also changes appearance and
has no new detector-accuracy qualification.

No whole-host hang or network outage occurred during these checks. Restoring
the normal workload does not establish the cause of the
[earlier outage](HALO_OUTAGE_INVESTIGATION_20261027.md); paired diagnostic
recordings are separate evidence, not a crash fix.

Fresh one-hour host-health and Windows connectivity recorders began at about
16:36 Central, and a new Windows-side system-warning stream began at about
16:37. The paired recording window lasts until approximately 17:35 Central.
Host health is saved to `/home/ian/.cache/halo-outage-20261027-live3/health.jsonl`;
Windows recordings and process metadata are under
`C:\Projects\Capstone\.halo-recovery\outage-20261027\resumed-workload\`.
Previous recordings were retained. These are bounded diagnostic processes,
not persistent monitoring services.
