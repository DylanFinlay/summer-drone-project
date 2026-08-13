# DIY Autonomous Drone

ROS 2 Jazzy learning project for a Raspberry Pi 5 companion computer and an
ArduPilot flight controller. ArduPilot owns stabilization and flight safety;
the Raspberry Pi only requests bounded body-frame velocities while the RC
transmitter has selected Guided mode. MAVROS owns the serial/MAVLink link and
translates between ROS and ArduPilot coordinate conventions.

## MAVROS setup

Install the ROS 2 MAVROS package and its required GeographicLib dataset on the
Raspberry Pi:

```bash
sudo apt update
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-msgs
ros2 run mavros install_geographiclib_datasets.sh
```

The GeographicLib dataset is required by MAVROS even though this project only
uses a subset of its plugins. The default connection is
`serial:///dev/ttyAMA0:57600`; it can be replaced at launch with `fcu_url:=...`
for a USB serial device, UDP, or ArduPilot SITL.

## Software-in-the-loop simulation

The SITL launch starts an ArduCopter quad, MAVROS over loopback TCP, and the
complete project stack in safe `hover` mode:

```bash
ros2 launch diy_autonomous_drone drone_sitl.launch.py \
  sim_vehicle_command:=$HOME/ardupilot/Tools/autotest/sim_vehicle.py
```

From another sourced terminal, verify the connected zero-command pipeline:

```bash
ros2 run diy_autonomous_drone sitl_smoke_test
```

See the [SITL setup and test guide](docs/SITL.md) for the one-time ArduPilot
installation, launch options, expected checks, and optional simulated
Guided/armed authority test.

## Vision setup

The active-tracking MVP uses an Ultralytics YOLO11 nano detector. Create the
Python environment with access to apt-installed ROS and Picamera2 packages,
then build the ROS workspace while that environment is active:

```bash
sudo apt install python3-venv
python3 -m venv --system-site-packages ~/drone_venv
source ~/drone_venv/bin/activate
python -m pip install -r requirements-vision.txt
colcon build --symlink-install
```

The default `yolo11n.pt` model may be downloaded by Ultralytics the first time
it is loaded. Do that while internet access is available, before field tests.
Alternatively, put the model on the Pi and set `detector_model` in
`config/params.yaml` to its absolute path.

### Recorded-video perception test

Use a saved video to exercise YOLO, target selection, target loss, filtering,
and command generation without a camera or flight-controller connection:

```bash
ros2 launch diy_autonomous_drone drone_video.launch.py \
  video_file:=/absolute/path/to/test-video.mp4
```

The dedicated launch always disables MAVROS and the FC interface. Add
`loop_video:=true` for repeat testing. Every loop clears target identity and
requires normal multi-frame reacquisition. EOF, a corrupt frame, or a failed
rewind publishes target loss immediately. Add `enable_flight_logging:=true`
to record the resulting low-bandwidth decisions for comparison.

## Demo modes

The launch file defaults to the safest useful configuration: no camera and a
continuous zero-velocity hover request. The MAVROS safety adapter only opens
its command gate when MAVROS is connected, Guided mode is selected, the
vehicle is armed, and the independent safety supervisor is publishing fresh
output.

After building and sourcing the workspace, choose one configuration:

```bash
# RC/manual flight plus a zero-velocity Guided-mode test. Camera stays off.
ros2 launch diy_autonomous_drone drone_autonomous.launch.py

# Active tracking demo. Vision is enabled explicitly.
ros2 launch diy_autonomous_drone drone_autonomous.launch.py \
  autonomy_mode:=active_track enable_vision:=true

# Run perception without connecting the Pi command gateway to the FC.
ros2 launch diy_autonomous_drone drone_autonomous.launch.py \
  autonomy_mode:=active_track enable_vision:=true \
  enable_fc_interface:=false

# Test camera capture without loading or running YOLO.
ros2 launch diy_autonomous_drone drone_autonomous.launch.py \
  enable_vision:=true enable_object_detection:=false \
  enable_fc_interface:=false
```

Gesture control is intentionally disabled by default. To test it later, set
`autonomy_mode:=gesture_control`, `enable_gesture_control:=true`, and
`enable_gesture_recognition:=true`, and keep the same physical RC Guided-mode
gate. A camera gesture never grants flight authority.

Gesture recognition uses `yolo11n-pose.pt`, reusing the installed Ultralytics
runtime instead of adding a second inference framework. The model may download
on first use, so cache it before field testing. Four matching frames are
required before motion; an unclear pose stops immediately. The supported poses
are both arms up, both arms deliberately down-and-out, or one horizontal arm
pointing toward the image's left/right while the other arm rests. Down is not
triggered by an ordinary arms-at-sides stance.

Exercise the complete gesture pipeline against a recording before live camera
or FC testing:

```bash
ros2 launch diy_autonomous_drone drone_video.launch.py \
  video_file:=/absolute/path/to/gestures.mp4 \
  autonomy_mode:=gesture_control enable_gesture_control:=true \
  enable_gesture_recognition:=true
```

The recorded-video launch always disables MAVROS and the FC interface.

The motion mode can also be changed while the stack is running. Every accepted
transition immediately publishes zero, clears observations from the previous
mode, and waits for fresh input before movement can resume:

```bash
# Keep perception available while initially holding zero in hover mode.
ros2 launch diy_autonomous_drone drone_autonomous.launch.py \
  autonomy_mode:=hover enable_vision:=true

# Run these from a second terminal after sourcing the workspace.
ros2 param set /tracking_bridge_node autonomy_mode hover
ros2 param set /tracking_bridge_node autonomy_mode active_track

# Gesture control requires a deliberate two-step unlock.
ros2 param set /tracking_bridge_node enable_gesture_control true
ros2 param set /tracking_bridge_node autonomy_mode gesture_control

# Return to hover before locking the experimental feature again.
ros2 param set /tracking_bridge_node autonomy_mode hover
ros2 param set /tracking_bridge_node enable_gesture_control false
```

For routine demo operation, the one-shot operator tool is simpler and checks
the tracking node's response before reporting success:

```bash
ros2 run diy_autonomous_drone operator_mode_tool status
ros2 run diy_autonomous_drone operator_mode_tool hover
ros2 run diy_autonomous_drone operator_mode_tool track

# These remain deliberate explicit operations for the experimental feature.
ros2 run diy_autonomous_drone operator_mode_tool gesture
ros2 run diy_autonomous_drone operator_mode_tool lock-gesture
```

This tool changes only ROS autonomy parameters. It cannot arm/disarm, change
ArduPilot flight mode, or override the physical RC authority switch.

Invalid modes are rejected without changing the active mode. This ROS setting
only chooses what command generator runs; it cannot arm the vehicle or select
ArduPilot Guided mode. The physical RC mode switch remains the master gate.
Process-level toggles such as `enable_vision` still require a relaunch, so start
the camera in advance when a live switch into active tracking is planned.

Valid tracking and gesture commands pass through a vector acceleration limiter
before reaching the safety supervisor. The conservative defaults permit
`0.5 m/s^2` of linear acceleration and `0.8 rad/s^2` of yaw acceleration.
Normal corrections and direction reversals ramp smoothly. Hover, stale input,
mode changes, and other safety stops bypass ramping and publish zero
immediately. Configure these limits in `config/params.yaml` only after testing
them in SITL.

Active tracking also smooths normalized target position and box height with an
exponential filter (`tracking_filter_alpha: 0.35`). Continuous deadbands ignore
small horizontal (`0.04`) and follow-distance (`0.025`) errors without creating
a command jump at either threshold. Filtering resets after stale input or a
mode change, while the safety supervisor continues using raw observations for
immediate proximity protection. Set the filter alpha to `1.0` or a deadband to
`0.0` to disable that individual behavior during comparison tests.

Launch toggles:

- `autonomy_mode`: `hover`, `active_track`, or `gesture_control`
- `enable_gesture_control`: unlocks the experimental gesture mode
- `enable_gesture_recognition`: runs the optional YOLO pose classifier
- `enable_vision`: starts camera capture and inference hooks
- `enable_object_detection`: loads and runs the YOLO person detector
- `video_file`: replaces the live camera with a local recorded video
- `loop_video`: safely restarts video with cleared target identity
- `enable_tracking`: starts autonomous command generation
- `enable_rc_aux_mode_selection`: uses a calibrated spare RC channel for modes
- `rc_aux_channel`: one-based spare channel; zero means unconfigured
- `enable_fc_interface`: starts MAVROS and its command safety adapter
- `enable_flight_logging`: records selected control and state topics to rosbag
- `flight_log_directory`: overrides the timestamped rosbag output directory
- `fcu_url`: selects the MAVROS serial, UDP, TCP, or SITL connection
- `gcs_url`: optionally forwards MAVLink traffic to a ground station
- `configuration_profile`: `simulation`, `bench`, or `outdoor_demo`
- `parameter_file`: selects the base YAML before the profile overlay

The safety supervisor is deliberately not optional in the launch file.

## Optional RC auxiliary ROS-mode selection

The receiver-to-flight-controller connection remains the manual safety path.
After hardware calibration, a separate unused receiver channel can optionally
select only the ROS command generator. It does not arm, select `GUIDED`, send
RC overrides, or replace the main ArduPilot flight-mode switch.

With propellers removed, record the spare channel and PWM values in the RC
worksheet, leave its ArduPilot `RCx_OPTION` as `0` (Do Nothing), and verify the
values on `/mavros/rc/in`. Then enable the software mapping, for example:

```bash
ros2 launch diy_autonomous_drone drone_autonomous.launch.py \
  enable_rc_aux_mode_selection:=true rc_aux_channel:=6 \
  enable_vision:=true
```

The conservative mapping is low=`hover`, middle=`hover`, and
high=`active_track`. Each position requires three matching samples. Missing,
malformed, transitioning, or stale RC input requests hover immediately. The
current state is reported on `/drone/rc_aux_state` and in `/drone/status`.
While this feature is enabled, RC auxiliary input continuously owns the ROS
mode selection, so operator-tool mode changes are temporary. Keep gesture mode
out of the RC mapping until recorded-video and props-off tests pass.

## Configuration profiles

The main launch defaults to the `bench` profile. Profiles are small overlays
applied after `config/params.yaml`, so shared detector and safety settings stay
in one place:

- `simulation`: moderate limits for ArduPilot SITL.
- `bench`: lowest movement limits for props-off and perception work.
- `outdoor_demo`: conservative starting limits for the eventual field demo.

Select one explicitly with, for example:

```bash
ros2 launch diy_autonomous_drone drone_autonomous.launch.py \
  configuration_profile:=outdoor_demo
```

Every profile keeps the Guided-mode and armed-state FC gates enabled. The
outdoor values are starting points, not validated tuning; keep the `bench`
profile until SITL, props-off, and manual-flight checks are complete.

## Live status and diagnostics

The passive status node starts in every launch configuration and combines the
active autonomy mode, MAVROS flight-controller state, target-lock state, and
the independent safety-gate reasons. It never publishes flight commands or
changes authority. Inspect its compact JSON summary with:

```bash
ros2 topic echo /drone/status
```

The same fields are published as a standard `diagnostic_msgs/DiagnosticArray`
on `/diagnostics` for ROS diagnostic tools. Missing component reports become
`stale` and their values become `unknown`; they are never silently treated as
healthy. An intentionally disabled tracking or FC interface is reported as
`disabled`. `safety_stop_reason` preserves reasons from both the command
safety supervisor and the final MAVROS gate when both are active.

## Flight-data logging

Logging is opt-in to avoid filling the Pi's storage during ordinary testing.
Enable it for SITL, bench, or outdoor test runs with:

```bash
ros2 launch diy_autonomous_drone drone_autonomous.launch.py \
  enable_flight_logging:=true
```

Each default bag is timestamped under
`~/.ros/diy_autonomous_drone/`. It records commands, modes, selected-target
data, target visibility, safety reasons, diagnostics, and MAVROS state. Camera
images are deliberately excluded to keep files small. Stop the launch cleanly
with Ctrl+C so rosbag can finalize its metadata. Inspect or replay a bag with
`ros2 bag info BAG_DIRECTORY` and `ros2 bag play BAG_DIRECTORY`.

## Orderly shutdown

Before intentionally stopping an active stack, prepare it from another
sourced terminal:

```bash
ros2 run diy_autonomous_drone operator_mode_tool prepare-shutdown
```

The command confirms that the requested mode is `hover` and publishes ten
zero commands through the normal safety-supervisor path.
After it reports success, stop the launch with Ctrl+C. During orderly node
teardown, each command-producing stage also attempts five final zero messages.

This is not the crash failsafe: a killed, frozen, or disconnected process may
not execute cleanup code. The independent command/target/MAVROS watchdogs and
ArduPilot's configured Guided timeout remain responsible for those failures.

## Camera

`camera_backend: auto` tries Picamera2 first and then OpenCV. Picamera2 is the
preferred backend for a Raspberry Pi CSI camera when the operating-system
camera stack supports it. OpenCV remains useful for USB cameras and desktop
development. Verify continuous camera capture on the final Pi image before
integrating inference.

The printed camera carrier should use the M3.5 x 13 mm damping balls as its
vibration-isolation elements. VHB can retain or bond parts of the mount, but
use a captive mechanical design or safety tether as well.

## Active-tracking MVP behavior

YOLO is filtered to the configured person class. For safety, the default
selector only acquires when exactly one person is visible and requires three
consecutive, spatially consistent detections. Once acquired, it keeps the
person whose bounding box overlaps the prior locked box. It refuses an
ambiguous association, publishes no target on every missed frame, clears the
identity after repeated misses, and requires a new three-frame confirmation
before motion can resume.

The command generator has three explicit target states, published continuously
on `/drone/tracking_state`:

- `hover`: active tracking has no valid target and commands zero velocity.
- `tracking`: a selected target is visible and fresh.
- `temporarily_lost`: a previously tracked target disappeared; zero velocity
  is published immediately while a bounded reacquisition window runs.

The vision node publishes `/drone/target_visible` for every processed frame.
One missing frame moves `tracking` to `temporarily_lost` without waiting for a
watchdog. A target reacquired within `target_loss_grace_sec` (default `0.75`
seconds) returns to `tracking` from rest. Expiry returns to `hover`. Loss clears
the target filter so an old person's position cannot affect reacquisition. If
camera or visibility messages stop completely, `target_timeout_sec` remains an
independent fallback into the same safe loss path.

This is intentionally conservative and intended for a demonstration with one
participant in a clear area. `require_single_person` can later be disabled,
but a deliberate operator selection mechanism should be implemented before
using that setting around multiple people.

## Minimum test order

1. Configure and prove manual Stabilize, AltHold, Loiter, and RTL flight.
2. Test `/mavros/state` connection and mode reporting with propellers removed.
3. Launch the default hover configuration and confirm that leaving Guided
   immediately closes the command gate.
4. Kill each ROS process in turn and confirm that the FC receives zero or its
   own Guided timeout stops movement.
5. Run perception-only with `enable_fc_interface:=false` and recorded/live
   video.
6. Test active tracking at low altitude and low speed in a clear open area.

Before real autonomous flight, configure ArduPilot's RC, battery, EKF,
geofence, and Guided-command timeout failsafes. Start with the conservative
speed and timeout values in `config/params.yaml`; increase them only after
reviewing flight and vibration logs.

Use the [ArduPilot RC setup checklist](docs/ARDUPILOT_RC_SETUP.md) and its
planning worksheet before hardware testing. The worksheet deliberately leaves
all receiver-specific calibration values blank until they can be measured.
