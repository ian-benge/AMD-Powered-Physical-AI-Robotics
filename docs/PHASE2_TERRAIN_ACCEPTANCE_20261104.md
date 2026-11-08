# Phase 2 terrain implementation and acceptance

November 4, 2026; final acceptance November 5, 2026.
**Phase 2 accepted:** 50 seeded layouts and three guarded manual traversals at
0.05 m/s passed. Final protected teardown was clean. Capture and temperature
monitoring ended before cooldown completed, so continuous cooldown logging is
not claimed. See [operator commands](PHASE2_TERRAIN_GUIDE.md).

| Representative | Seed | Sampled travel | Maximum forward / turn command | Peak Tctl |
| --- | --- | --- | --- | --- |
| 1 | 2601136 | 4.05 m | 0.05 m/s / 0.30 rad/s | 73.75 C |
| 2 | 2601113 | 3.38 m | 0.05 m/s / 0.30 rad/s | 67.875 C |
| 3 | 2601104 | 3.63 m | 0.05 m/s / 0.30 rad/s | 68.5 C |

All three included turning and travel beyond the launch disk. The operator
completed the requested gentle-terrain and rock-passage checks. Representative
3 ended with conservative guard stops; the operator confirmed stopping before
contact. These are manual terrain checks, not autonomous navigation or Phase 3
mapping acceptance.

## Implemented behavior

`mixed_reachable` v1 combines seeded, quintic-interpolated noise at three scales,
smooth coordinate distortion, localized hills and shallow depressions. The
20 × 20 m world retains a flat, clear 1.3 m radius launch disk. One 129 × 129
triangle surface supplies rendering, collision, placement and annotation.
Cosmetic grain remains in textures. Historical profiles retain their old
geometry, and existing scene manifests are never replaced by this profile.

After every object is placed, the generator finds connected staging areas near
pale soil, ordinary rock and crystal rock, 3–7 m from launch and outside each
target boundary. Each route has at least a one-metre swept corridor containing
the rover's complete padded turning envelope. Passage triangles stay below 8°;
steeper terrain elsewhere remains available as a challenge. Navigation's 12°
slope and 3 cm hazard limits are unchanged. Truth routes remain generation and
evaluation records; navigation receives no route or traversability shortcut.

Infeasible layouts retry the complete seeded terrain/object placement, up to 24
attempts. Accepted manifests retain rejected attempt seeds/reasons. Exhaustion
reports failure; I/O failures are never disguised as retryable placement errors.
Generate new scene uses this profile; Restart retains the current scene. Scene
generation now preserves the candidate image identity and runs without network
or GPU devices. The direct 8777 page forwards both scene actions to host control.

The independent audit found that some simplified scanned collision hulls extend
beyond their visible vertices. New-profile placement and clearance now enclose
both meshes. It also reads saved meshes in metres, avoiding the historical
asset-import helper's centering/normalization.

## CPU evidence

Local evidence:
`navigation_sessions/terrain-phase2-20261104/qualification-v3/qualification.json`.
Halo candidate root:
`/home/ian/capstone-training/rover-navigation/navigation_sessions/terrain-phase2-20261105T031610Z/application-v5`.

| Check | Result |
| --- | --- |
| Complete seeded layouts | 50 / 50, seeds 2601100–2601149 |
| Full independent replays | 50 / 50 with equal geometry, routes and retry records |
| Post-placement staging routes | 150 / 150 |
| Total rejected intermediate layouts | 97; reasons retained |
| Highest attempt count | 17 / 24 |
| Total relief | 0.3078–0.7440 m |
| Highest audited corridor-triangle slope | 7.9996424°; below 8° |
| Smallest actual obstacle clearance from route center | 0.72153 m; required >=0.5 m |
| Fully materialized scenes matching qualified geometry | 3 / 3 |
| Second-build Linux source suites | navigation 192 (4 skips), environment 161, perception 157; all successful |
| Final third-build Linux source suites | navigation 192 (4 skips), environment 163, perception 157; all successful |
| Fifth-build Linux source suites | navigation 195 (4 skips), environment 164, perception 157; all successful |
| Representative Windows/Linux geometry replay | 3 / 3, with equal geometry hashes and passing independent audits |

The audit derives normals from saved triangles and checks complete continuous
corridors using conservative triangle bounds. It checks physical and visual
object extents, world edges, the full clear launch disk, target boundary gaps
and route endpoints independently of the planner's free-cell mask. Route plots
were generated as evaluation artifacts, without collecting camera images.

Representative seeds, selected across the distribution of maximum surface
slope, are **2601136, 2601113 and 2601104**. Each has three certified staging
areas. The source archive used by the second separate build is
`source-v2.tar.gz`, SHA-256
`040980c9c32299807a1822f863e19348b26e6885594e5d9a87e5a51733989372`.
The third archive adds explicit container inspection and the cancellation gate:
`source-v3.tar.gz`, SHA-256
`159719d4fc952419fe8e7178fb0f7f9c04080f85aaad776844d2421a1b56c3e9`.
The third separate `CANDIDATE_CONFIGURATION.json` contains 982 source/install/
scene-asset pins, SHA-256
`170473c9aa93fe650c84c44fd5976fbeed0a19452d04524146c337cfe7b10bbc`.
Its runtime preflight passed with the fresh capture and retained NPU/float64
terrain identities; actual requested/observed HIP flags were 4. CPU-reference
classify/depth/project parity passed before the desktop attempt. These early
preflights preceded the live traversal acceptance below.
The fourth candidate corrects the host supervisor's collision option to the
launcher's `scene` enum. Its 82 installed runtime Python/UI files are byte-equal
to the successful third build; `INSTALL_DERIVATION.json` records that reuse.
The actual installed ROS launch setup passed for all three representatives in
a container without GPU devices or executed nodes. It confirmed shared terrain
URIs and 20 m RGB / 8 m usable depth. Its final 982-pin configuration SHA-256 is
`ee5ace87d85ec11f824c2323668441fcacec8d73fc93679a8355fe0dbb81ac1e`;
source-v4 SHA-256 is
`56518d27068dcb5b523da686896c58117b145121112e902e1ac516353d522286`.
The immutable layout archive is 178,008,499 bytes, SHA-256
`2c5908f1da3eaf22de41824126b5e757b54432ae40e60fe353c69d236338d5a2`.

The fifth candidate adds a qualification-only manual speed cap of 0.05 m/s
and 0.30 rad/s, enforced on the server and advertised to the page. It preserves
the normal launch defaults. Its desktop parent forwards terminal interrupts
to the supervisor alone, allowing an in-progress retained health query to
finish before candidate cleanup. A real POSIX process-group test passed.
All five packages built, all three Linux source suites passed, and all three
actual installed ROS launch setups passed without executing nodes. The two
JavaScript suites passed on Windows. The original builder's final exit 127 is
preserved: it attempted an unavailable Node executable after the Python suites
passed. `BUILD_REVIEW_FINAL.json` records that tool-availability limitation;
the retained image was unchanged.

Fifth-candidate source archive `source-v5-reviewed.tar.gz` SHA-256:
`108fbbf3208bf4d782aecfdc191e14babedda71d249d8a88f3e54f80193964fd`.
Its 985-pin `CANDIDATE_CONFIGURATION.json` SHA-256:
`d031c2d4451759f350334ab02e7ea915654974b1695d5c8a9a02d5528766001c`.
Its runtime preflight passed against the current capture and retained services.

Earlier candidates/builds are retained. The first source checks exposed a legacy
fixture missing terrain metadata and omitted packaging inputs; both were fixed
before the successful second build. The second candidate's first desktop launch
then found that generic `docker inspect` can resolve the retained image tag
when the container is absent. No navigation container remained running. The
next supervisor uses explicit container inspection, verifies both image and
checkout ownership, and records cleanup failures. A launch/stop gate also
prevents a replacement renderer from starting after supervisor cancellation.
The third desktop attempt stopped cleanly because `terrain_collision=true`
was an invalid launch enum; the fourth candidate uses `scene` and adds the
actual CPU launch-graph review above. The checked starter also preserves and
verifies same-case parity evidence rather than attempting to overwrite it.
Windows also exposed an existing review
page concurrent-file replacement `WinError 5`; that unrelated code was unchanged,
and its Linux regression passed. The final targeted Windows terrain, host
manifest, renderer-profile, scene-control and launch checks passed (27 tests).

## Live preview and final acceptance

The fourth candidate ran scene 2601136 with the expected mapped Ogre backend,
20 m RGB / 8 m usable depth and shared physical/visible terrain. The operator
reported that driving looked good. Restart retained the scene and started a
fresh map; both container renderer proofs passed. The longer session recorded
about 10.21 m sampled truth travel, 4.14 m maximum launch displacement and
substantial turning. Peak Tctl was 76.75 C, and the bounded supervisor ended
without a failure or cleanup error at 04:20:37 UTC November 5.

These are positive rendering/restart/manual-use results. Most recorded forward
commands were 0.20 m/s, so they do not satisfy the three 0.05 m/s acceptance
traversals. The fourth candidate's page reset speed while changing scenes;
the fifth candidate enforces the qualification speed bounds described above.

The preview case closed with `MANUAL_SESSION_CLOSED_CLEANLY`, no kernel incident
matches and clean protected teardown at 14:22:48 UTC November 5. Capture had
stopped at 05:07 UTC, so uninterrupted overnight logging/cooldown is not claimed.
`LIVE_PREVIEW_REVIEW_V4.json`, `PROTECTED_PREVIEW_REVIEW_V4.json` and a closed-case
copy are retained under the Phase 2 root and the reviews were copied off Halo.

Three representative layouts passed attended guarded traversal at 0.05 m/s,
including slopes, turns and passages between obstacles. Review covered
measured motion/guard events, camera readiness, source/image/library identities
and protected kernel teardown. A fresh case started at 14:22:58 UTC November 5,
`oem617-7141-rocm-manual-20261105t142257z-bdc2-20261105T142258Z-eo7fv9o5`,
with capture and visible temperatures checked healthy for those checks.

Representative 1, seed 2601136, completed the strict-speed motion check in
`restart-20261105T142706Z-bae507`: 4.05 m sampled travel, 3.59 m maximum launch
displacement, maximum commanded 0.05 m/s and 0.30 rad/s, and 2.40 rad accumulated
turning. No guard fault occurred after motion began. The operator reported that
everything worked correctly; supervisor shutdown was clean and peak Tctl was
73.75 C. The case's final closure and monitoring limitation are recorded below.

The operator's route sampled a terrain facet up to 9.54 degrees under the
retained 12-degree navigation limit. Manual routes are chosen from the live
camera/map; they are not forced onto evaluator routes. The separately certified
generation passages remain below 8 degrees, as checked over all 150 corridors.
Representative 2 subsequently passed with the fifth candidate in
`restart-20261105T145625Z-50e5c4`: 3.38 m sampled travel, 2.37 m maximum launch
displacement, maximum commands of 0.05 m/s and 0.30 rad/s, and 8.81 rad
accumulated turning. Its sampled terrain facets reached 3.49 degrees. The final
guard state was `stopped/operator_stop`; no guard fault occurred after motion
began. Renderer provenance passed, supervisor shutdown was clean and peak Tctl
was 67.875 C. The operator reported completion with no rejection or defect.
Representative 3 passed in `restart-20261105T150041Z-bfa0e4`: 3.63 m sampled
travel, 2.43 m maximum launch displacement, maximum commands of 0.05 m/s and
0.30 rad/s, and 5.47 rad accumulated turning. Sampled terrain facets reached
8.00 degrees. Renderer provenance passed, peak Tctl was 68.5 C and supervisor
shutdown was clean.

Its guard latched four distinct stops on `unknown_terrain` cells within the
predicted turn sweep. The 121 fault-bearing status records are repeated samples
of those four events. Every latched output command was zero. The operator
clarified that a guard stopped the rover before contact. No `chassis_contact`
fault was logged; no contact messages were recorded, so the logs alone do not
establish a general collision-free guarantee. Conservative turn-sweep coverage
is a recorded Phase 3 mapping follow-up. No guard or threshold was bypassed or
relaxed by this implementation.

The first representative-2 attempt, `restart-20261105T143528Z-350bbe`, recorded
5.74 m travel and positive operator feedback, but its maximum forward command
was 0.20 m/s. It is retained as an incomplete acceptance attempt. Ctrl+C also
interrupted a `powerprofilesctl get` child during that fourth-candidate stop;
the recorded cleanup errors were empty. No GPU or power-policy fault was
established. The fifth candidate addresses both issues before repeating this
representative.

The final case closed at **15:06:14 UTC November 5** with
`MANUAL_SESSION_CLOSED_CLEANLY`, `post_teardown_clean=true` and no kernel
incident matches. The protected kernel journal command succeeded. Its container,
NPU/terrain ports and KFD checks showed no remaining workloads. Capture stopped
at **15:05:07 UTC**, before the operator's cooldown completed; the temperature
display was also stopped early. The protected final journal/teardown passed,
but continuous cooldown telemetry and endurance stability are not claimed.
The generic `POST_TEARDOWN.json` result remains
`INCOMPLETE_OR_FAILED_REVIEW_REQUIRED` because no timed hour was performed;
the separate manual-session verdict above applies to this attended work.

The Phase 2 root retains `PHASE2_ACCEPTANCE_FINAL.json`,
`PHASE2_PROTECTED_REVIEW_FINAL.json`, all individual traversal/pose reviews,
`V4_V5_REPRESENTATIVE_CONTINUITY.json` and copies of both closed cases.
The final 985 candidate pins were rechecked. Representative 1's fourth-candidate
run remains applicable: all three scene/geometry hashes, renderer image,
mapped Ogre plugin and camera profile are identical in the fifth candidate.
Only the manual speed profile, terminal-stop handling and related tests changed.
Earlier immutable evidence retains its historical pending/failed-attempt status;
the final acceptance record supplies the current Phase 2 verdict.

The complete frozen evidence tree was copied off Halo and verified without
extraction: **10,318 content files, the manifest and 8 symlinks**, including
hardlinks resolved to previously verified identical content. Local archive:
`navigation_sessions/terrain-phase2-20261104/terrain-phase2-20261105T031610Z-final-evidence.tar.gz`.
Size: **3,143,990,370 bytes**. SHA-256:
`1a4d34029e6f43a674fd821c5f7a87d773221bde33bdfd83ab55228d1d87982b`.
`final-evidence.json`, `FINAL_EVIDENCE_MANIFEST.json` and
`final-archive-verification.json` sit beside it. The frozen tree retains the
report as it stood before archive creation; this archive receipt and the
completed report are retained separately beside the remote archive as well.

No Phase 3 adaptive-clearing acceptance, dataset expansion, retraining, model
export or autonomous approach was performed. No commit/push or baseline
driver/kernel/runtime/power/fan/RDP change was made.
