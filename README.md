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

Gesture control is intentionally disabled by default. To test it later, launch
with `autonomy_mode:=gesture_control enable_gesture_control:=true`, and keep
the same physical RC Guided-mode gate. A camera gesture never grants flight
authority.

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
- `enable_vision`: starts camera capture and inference hooks
- `enable_object_detection`: loads and runs the YOLO person detector
- `enable_tracking`: starts autonomous command generation
- `enable_fc_interface`: starts MAVROS and its command safety adapter
- `fcu_url`: selects the MAVROS serial, UDP, TCP, or SITL connection
- `gcs_url`: optionally forwards MAVLink traffic to a ground station
- `parameter_file`: selects an alternative complete YAML configuration

The safety supervisor is deliberately not optional in the launch file.

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
