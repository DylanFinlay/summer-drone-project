# ArduPilot RC and authority setup

This setup intentionally keeps manual takeover independent of ROS. Connect
the RC receiver directly to the flight controller. The Raspberry Pi connects
separately over MAVLink and must never arm the vehicle or select `GUIDED`.

The planning worksheet is
[`config/ardupilot_rc_setup.template.yaml`](../config/ardupilot_rc_setup.template.yaml).
It is not an ArduPilot parameter file and must not be loaded into the flight
controller.

## What can be decided before hardware

- Pi velocity commands are accepted only while ArduPilot reports connected,
  armed, and `GUIDED`.
- The preferred three-position order is `STABILIZE`, `LOITER`, `GUIDED`.
- `RTL` should have its own deliberate auxiliary switch.
- Changing the ROS autonomy mode never changes the ArduPilot flight mode.
- Switching away from `GUIDED`, losing MAVROS state, or losing fresh commands
  closes the software command gate and publishes zero.

## What must be measured on the aircraft

Do not guess the receiver protocol, channel endpoints, trims, reversals,
flight-mode PWM values, or RC failsafe behavior. They are specific to the
transmitter, receiver, flight-controller wiring, and ArduPilot installation.

With propellers removed, follow the official ArduPilot radio-calibration and
flight-mode procedures. Record the results in the worksheet, then verify each
switch position in the ground station and through `/mavros/state`.

Suggested bench checks, all with propellers removed:

1. Confirm stick directions, endpoints, trims, and throttle failsafe.
2. Confirm every main-switch detent selects its intended flight mode.
3. Confirm the dedicated auxiliary switch selects RTL as intended.
4. Confirm the vehicle never enters `GUIDED` due to a Pi restart or ROS launch.
5. In `GUIDED`, confirm the Pi initially publishes zero and requires a fresh
   command before nonzero output is allowed.
6. Leave `GUIDED` and confirm the MAVROS command gate closes immediately.
7. Turn off the transmitter and validate the configured ArduPilot RC failsafe.

After those checks, export the complete, tested ArduPilot parameter set. Keep
that exported file as the aircraft's known-good configuration and record its
path and firmware version in the worksheet. Battery, EKF, geofence, and other
ArduPilot failsafes remain separate required setup tasks.

Official references:

- [ArduPilot radio calibration](https://ardupilot.org/copter/docs/common-radio-control-calibration.html)
- [ArduPilot flight-mode configuration](https://ardupilot.org/copter/docs/common-rc-transmitter-flight-mode-configuration.html)
