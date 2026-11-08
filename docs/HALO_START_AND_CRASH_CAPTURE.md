# Halo browser simulation and future GPU-crash capture

These commands use the existing Halo navigation checkout and Docker image.
They do not rebuild ROS, change drivers or retrain the detector. Run the
graphical launch from a terminal inside Ian's current Halo desktop, so it
inherits the current display and Xauthority cookie. Keep the Ubuntu desktop
connected while using the current Ogre sensor workaround.

## Establish the trigger

The captured failure names stalled Gazebo GUI graphics work. Determining the
underlying trigger needs controlled reproduction and a GPU device dump, not
only another desktop stack trace.

1. Save the GPU device dump immediately after a timeout. The October 27 kernel
   created one, but it had expired before inspection. Linux's default device
   dump retention is five minutes; see the
   [kernel device-coredump documentation](https://www.kernel.org/doc/html/next/process/debugging/driver_development_debugging_guide.html).
   A read-only event watcher that copies new dumps automatically is the next
   monitoring improvement; it has not been installed by this guide.
2. Record kernel messages to disk before starting, plus a Windows-side stream
   so a later host stall cannot erase the only copy. Preserve launch output,
   software/firmware versions, scene, renderer, camera profile and timestamps
   of operator actions. Existing October 27 one-hour recorders are bounded,
   not permanent monitoring services.
3. Repeat comparable stationary trials, changing one workload at a time:
   desktop/RDP alone; simulation with CPU terrain and no detector; add Gazebo
   GUI; add ROCm terrain; then add NPU inference. Hold RViz/display, scene,
   renderer and duration constant when comparing a pair. Keep display
   authentication available for Ogre even when hiding a GUI. Do not change
   renderer, kernel, firmware and workload together. A single successful run
   cannot exclude an intermittent failure.
4. If one configuration reliably reproduces it, reduce that configuration to
   a small scene/test and retain the device dump and version fingerprint for
   driver/Gazebo investigation. More detailed traces or symbolized stacks may
   still be needed; logs cannot guarantee identification of the exact bug.

On Halo, before a test, use a dedicated SSH terminal for kernel recording:

```bash
CAPTURE="$HOME/capstone-training/rover-navigation/navigation_sessions/capture-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$CAPTURE"
printf '%s\n' "$CAPTURE"
sudo timeout 3600 journalctl -k -f -n 0 -o short-iso-precise | tee -a "$CAPTURE/kernel-live.log"
```

This runs for at most one hour and saves on Halo. It is not a Windows-side copy.
For a Windows-side record without a sudo prompt in a pipeline, this separate
PowerShell command records Ian's desktop messages (not the privileged kernel
journal):

```powershell
$Log = "C:\Projects\Capstone\halo-desktop-live-$(Get-Date -Format yyyyMMdd-HHmmss).log"
ssh -T -o ServerAliveInterval=15 -o ServerAliveCountMax=3 halo timeout 3600 journalctl --user -f -n 0 -o short-iso-precise | Tee-Object -FilePath $Log
```

If another fault happens, connect by SSH promptly and run this before a reboot:

```bash
CAPTURE="$HOME/capstone-training/rover-navigation/navigation_sessions/fault-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$CAPTURE"
sudo journalctl -b --since '-15 min' --no-pager -o short-iso-precise > "$CAPTURE/system.log"
journalctl --user -b --since '-15 min' --no-pager -o short-iso-precise > "$CAPTURE/desktop.log"
for dump in /sys/class/devcoredump/devcd*/data; do
  [ -e "$dump" ] || continue
  name=$(basename "$(dirname "$dump")")
  sudo timeout 30 cat "$dump" > "$CAPTURE/$name.bin"
done
sync
printf 'Saved to %s\n' "$CAPTURE"
```

The loop only reads device dumps. An absent dump means none was available at
collection time, not that no GPU hang occurred. After a necessary reboot, also
save `sudo journalctl -b -1 --no-pager -o short-iso-precise` to a new file.

## Start services

On Halo:

```bash
cd /home/ian/capstone-training/rover-navigation
curl -fsS http://127.0.0.1:8775/status
curl -fsS http://127.0.0.1:8782/status
```

Do not start duplicate listeners. A cold start needs the NPU service in its own
terminal; the script warms up and verifies NPU execution before becoming ready:

```bash
cd /home/ian/capstone-training/rover-navigation
mkdir -p navigation_sessions/control
bash scripts/halo_navigation_detector.sh 2>&1 | tee -a "navigation_sessions/control/npu-$(date -u +%Y%m%dT%H%M%SZ).log"
```

The October 27 NPU service remained ready after the graphics failure and can be
retained. A pre-reset ROCm process should be restarted in a fresh process even
if its cached status still reports ready. Stop only its verified owner, then
run in another Halo terminal:

```bash
cd /home/ian/capstone-training/rover-navigation
mkdir -p navigation_sessions/control
bash scripts/halo_terrain_gpu.sh 2>&1 | tee -a "navigation_sessions/control/terrain-$(date -u +%Y%m%dT%H%M%SZ).log"
```

Wait for readiness. Terrain startup performs real GPU computation and readback.
An occupied port is a reason to inspect ownership, not start another instance.

## Start the rover

If a stale scene-control gateway still owns 8774/8776 after a desktop recovery,
stop that verified process first. Starting the new gateway below from the new
desktop refreshes its display authentication. Inspect ownership with
`ss -ltnp`; PID values change between runs. Confirm no navigation container is
already active with `docker ps`.

In a new terminal inside the Halo desktop:

```bash
cd /home/ian/capstone-training/rover-navigation
mkdir -p navigation_sessions/control
NAV_SESSION="manual-$(date -u +%Y%m%dT%H%M%SZ)"
printf '%s\n' "$NAV_SESSION" > navigation_sessions/operator-session.txt
MODEL=/home/ian/capstone-training/mars-pilot50-v2/runs/detect/coverage500_fourclass_yolov8s_npu_960/weights/best.pt
bash scripts/halo_navigation.sh \
  /home/ian/capstone-training/mars-pilot50-v2/dataset/live_eval_scenes \
  scene_050000 "$NAV_SESSION" \
  gui:=true rviz:=true preview:=true mission_controls:=true \
  sensor_render_engine:=ogre sensor_headless:=false \
  terrain_backend:=rocm_http adaptive_terrain:=true \
  controls_port:=8777 mapping_on_confirm:=true \
  detector_model:="$MODEL" \
  2>&1 | tee -a "navigation_sessions/control/$NAV_SESSION-launch.log"
```

This is the current simulation profile, not a claim that the graphics fault is
fixed. It starts a fresh session and preserves earlier session directories.
Keep the terminal open. The explicit `tee` keeps launch output on the host
despite the Docker container being auto-removed on exit.

After the session initializes, use another Halo desktop terminal:

```bash
cd /home/ian/capstone-training/rover-navigation
NAV_SESSION=$(cat navigation_sessions/operator-session.txt)
bash scripts/halo_navigation_scene_control.sh \
  /home/ian/capstone-training/mars-pilot50-v2/dataset/live_eval_scenes \
  "$NAV_SESSION" \
  2>&1 | tee -a "navigation_sessions/control/$NAV_SESSION-gateway.log"
```

The session name must match. Wait for its `session.json` to exist before this
command. The public browser gateway is 8774, its private supervisor is 8776,
and ROS controls are 8777. Use Ctrl+C in the relevant launcher terminal for a
normal shutdown; do not kill unrelated services by broad process-name patterns.

## Open the browser from Windows

Keep the existing RDP tunnel open. In a separate Windows PowerShell terminal:

```powershell
ssh -N -T -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -L 127.0.0.1:8874:127.0.0.1:8774 -L 127.0.0.1:8873:127.0.0.1:8773 halo
```

Open `http://127.0.0.1:8874` for controls or
`http://127.0.0.1:8873/frame.png` for the raw camera. The 887x local ports avoid
the agent's earlier 877x forward. On Halo itself, use `http://127.0.0.1:8774`.

Inspect the new launch area in Gazebo, confirm clearance in the page, wait for
ready, select 0.05 m/s for the next bounded trial, then explicitly enable manual
drive. The normal operator default is 0.20 m/s; the implemented upper limit is
0.50 m/s. Begin stationary when reproducing the graphics fault so movement is
not an additional variable.
