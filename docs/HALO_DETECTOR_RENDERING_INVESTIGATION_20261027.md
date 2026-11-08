# Late-visible objects and detector investigation - October 27, 2026

The NPU detector is healthy in the observed window. Late-visible objects have
a concrete renderer concern and a separate camera-range limit. More training
data may help classification, but rendering should be qualified before
collecting a large new dataset.

The operator clarified that objects appear late in the camera/Gazebo view.
This is separate from the intentional multi-view delay before map confirmation.
The simulation was observed without movement commands, reset or model changes.

## Live checks

Scene `scene_85404259`, session `restart-20261027T214314Z-de7450`:

- The SDF contains all 47 physical scene objects plus the terrain model.
  All 240 referenced mesh/texture paths checked exist. The inspected runtime
  log tail had no matching resource-load errors. File presence does not prove
  that every surface is rendered correctly.
- Host detector processing increased at **30.27 FPS** over approximately eight
  wall seconds; last sampled inference-service processing was **26.02 ms**.
  Host and ROS-client readiness stayed healthy in these samples, without
  inference errors. This measures processing, not browser display throughput.
- Model hashes, four-class order, A16W8 static `[1,3,576,960]` input and
  `npu_executed` evidence match the existing deployment. ROS RGB conversion
  retains the required BGR input convention, and saved detections were paired
  with raw images using exact ROS timestamps.

## Original model versus NPU

Seven saved-image comparisons used the original trained checkpoint on CPU in
FP32 and the existing NPU service on identical decoded pixels, confidence 0.25
and a 576x960 input. Both produced **22 detections**, all with corresponding
same-class boxes at IoU at least 0.5. Cases include current visible rocks, the
operator-identified correct pale soil patch, historical camera views and one
image with no detections. Two soil comparisons sample the same stopped view.

This small diagnostic set gives no evidence of a quantization-only failure.
It is not a labeled accuracy evaluation, and the operator's specific incorrect
detection was not visible during these captures. Matching predictions can
still be wrong. Crystal/slab confusion and false-positive rates remain
unmeasured in this set. Do not interpret the earlier synthetic held-out scores
as accuracy on the current renderer/profile.

## Rendering and visibility

The active sensor uses **Ogre** as the earlier blank-RGB/depth workaround;
capture/training worlds use **Ogre2**. The scene camera profile has a 0.10 rad
downward tilt; navigation currently uses 0.40 rad. Those are about 5.7 and 22.9
degrees. Source captures are 1920x1080, whereas live input is 960x540 before
the model's padding. The trained model already uses a 960-wide input, so
this resolution difference alone does not establish a preprocessing bug.

There is also a specific opacity/depth concern:

1. Gazebo's scene-material loader enables texture alpha when loading a PBR
   albedo map. See [the Harmonic scene loader](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/rendering/SceneManager.cc).
2. Ogre's material implementation disables depth writing whenever that flag is
   enabled; its normal-map setter also retains an unimplemented rendering path.
   See [the Ogre implementation](https://github.com/gazebosim/gz-rendering/blob/gz-rendering8/ogre/src/OgreMaterial.cc).
3. A small independent material probe against the installed renderer reproduced
   `depth_write: true -> false -> true` when enabling and disabling texture
   alpha on an opaque captured RGB PNG. It did not modify the running world.

Incorrect depth writing is a strong candidate for rocks being overdrawn by
terrain and changing visibility as the viewpoint changes. This mechanism was
verified at material level; the operator's particular disappearing rock has
not yet been reproduced in an image comparison. A controlled scene comparison
is still required before declaring the cause fixed.

The probe's first compiled run printed the material result but crashed while
unloading the renderer with live local object references. Those references
were released before unloading, and the corrected probe exited successfully.
This separate diagnostic-process failure is not a whole-host or Gazebo-server
crash and must not be mixed with the earlier Halo outage investigation. The
initial pkg-config probe also failed because this ROS vendor installation
provides its build configuration through CMake; the successful probe used that
configuration. Both attempts and the successful rerun are retained.

The current RGB and depth cameras also both clip at **8 m**, with near clip
0.30 m. Objects outside the rendered range can legitimately appear on approach;
this cannot explain a close object disappearing. Initial scene-object centers
extend approximately 10.53 m from launch. Terrain ridges can physically occlude
rocks as well. No distance-based object insertion or custom LOD switch was
found in the generated static-world code.

## Recommended order

1. **Qualify rendering.** Use an isolated copy of a failing viewpoint to compare
   the current material settings with opaque depth-writing settings, retaining
   geometry and camera pose. Check both RGB occlusion and depth. Do not return
   blindly to Ogre2: its earlier intermittent blank-output failure remains
   relevant. A successful visual fix must also pass sensor/localization checks.
2. **Create a reviewed error set.** Save raw, synchronized frames from ordinary
   manual routes with the intended renderer and camera angle. Include bare dirt,
   shadows, ridges and textured ground wrongly detected as rocks, alongside
   actual rocks, pale soil, crystals and slabs. Include near/far, partial views
   and empty backgrounds. Avoid large numbers of adjacent duplicate frames.
3. **Fine-tune with targeted data.** Mix those examples with the existing
   dataset. Background-only examples can reduce false positives; genuine
   objects in a negative example must still be labeled. This is consistent with
   [Ultralytics' data guidance](https://docs.ultralytics.com/yolov5/tutorials/tips-for-best-training-results).
   Choose added volume using measured error reduction rather than assuming a
   fixed number of random new scenes will solve the problem.
4. **Keep evaluation independent.** Split by scene/session and preserve held-out
   mesh groups; the generator already selects assets by split. Do not append
   arbitrary live/test scenes directly to training. Freeze an annotated held-out
   route set before tuning and report per-class precision/recall, confusion and
   false positives per image, including background-only frames.
5. **Revalidate deployment.** Check the new FP32 checkpoint first, then fresh
   export, calibration/quantization, CPU/NPU parity, offload proof and held-out
   accuracy. Keep the current accepted artifacts for comparison and rollback.

Raising display confidence may hide some weak boxes but cannot establish correct
classification. Phase Three's geometry and independent-view checks should
continue protecting the semantic map. Terrain hazards remain depth-derived.

## Evidence and scope

Evidence is saved on Halo under
`navigation_sessions/detector-investigation-20261027T214750Z/`: live status and
timing samples, scene/configuration and resource inventory, paired raw camera
images/detections, CPU/NPU outputs and overlays, probe source and logs.
The archive `navigation_sessions/detector-investigation-20261027-evidence.tar.gz`
was copied to Windows, with SHA-256
`68386277d37b4802888e72a007ec49017b286a23e1c0900246b32a6e9fdea867`.

No source deployment, training, new weights, confidence change, renderer switch
or scene restart was performed. The final observed camera was fresh, the same
session remained active, and the operator's manual selection was preserved
with a zero command. These are investigative observations, not a passed
rendering fix or new model-accuracy qualification.
