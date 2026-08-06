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
```

Gesture control is intentionally disabled by default. To test it later, launch
with `autonomy_mode:=gesture_control enable_gesture_control:=true`, and keep
the same physical RC Guided-mode gate. A camera gesture never grants flight
authority.

Launch toggles:

- `autonomy_mode`: `hover`, `active_track`, or `gesture_control`
- `enable_gesture_control`: unlocks the experimental gesture mode
- `enable_vision`: starts camera capture and inference hooks
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
