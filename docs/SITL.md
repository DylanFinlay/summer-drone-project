# ArduPilot SITL testing

This workflow runs an ArduCopter quad simulation, MAVROS, and the complete ROS
stack on Ubuntu. It starts in `hover`, leaves the vehicle disarmed, and never
selects `GUIDED` automatically.

## One-time Ubuntu setup

Install ROS 2 Jazzy and MAVROS as described in the main README. Then install
ArduPilot's supported build environment:

```bash
sudo apt update
sudo apt install git
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git \
  ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

Build and source this ROS workspace:

```bash
cd ~/summer-drone-project
colcon build --symlink-install
source install/setup.bash
```

## Start the complete simulated stack

Pass the exact simulator path so the intended ArduPilot checkout is used:

```bash
ros2 launch diy_autonomous_drone drone_sitl.launch.py \
  sim_vehicle_command:=$HOME/ardupilot/Tools/autotest/sim_vehicle.py
```

Add `enable_flight_logging:=true` to capture the complete low-bandwidth
decision and command path for later inspection. Logging is disabled by
default and excludes camera images.

The SITL launch selects `configuration_profile:=simulation` by default. The
main hardware launch instead defaults to `bench`; `outdoor_demo` must always
be selected deliberately.

The first run compiles ArduCopter and can take several minutes. MAVROS may
restart while TCP port 5760 is unavailable and will connect once SITL begins
listening. Simulator state and logs are kept under
`/tmp/diy_autonomous_drone_sitl` by default.

To connect to a SITL process started separately, use:

```bash
ros2 launch diy_autonomous_drone drone_sitl.launch.py \
  start_sitl:=false fcu_url:=tcp://127.0.0.1:5760
```

## Safe end-to-end smoke test

In a second sourced terminal, run:

```bash
ros2 run diy_autonomous_drone sitl_smoke_test
```

The test succeeds only after MAVROS reports a connected flight controller and
at least three zero commands pass through each stage:

```text
/drone/cmd_vel_raw
  -> /drone/cmd_vel_safe
  -> /mavros/setpoint_velocity/cmd_vel
```

It fails on any nonzero command. Run it with the default `hover` launch mode;
it deliberately does not arm, change flight modes, or take off.

Useful inspection commands:

```bash
ros2 topic echo --once /mavros/state
ros2 topic echo /drone/status
ros2 topic echo /diagnostics
ros2 topic echo /drone/tracking_state
ros2 topic echo /drone/target_visible
ros2 topic hz /drone/cmd_vel_raw
ros2 topic hz /drone/cmd_vel_safe
ros2 topic hz /mavros/setpoint_velocity/cmd_vel
```

Automated software tests cover raw-command and target watchdogs, proximity
stops, MAVROS disconnects, incorrect RC flight modes, disarming, command
freshness, target-loss transitions, invalid timing, and unsafe mode requests.
Run them in the sourced Ubuntu workspace with `colcon test`.

Before ending an intentional SITL or bench session, exercise the same orderly
stop sequence intended for the field demo:

```bash
ros2 run diy_autonomous_drone operator_mode_tool prepare-shutdown
```

Confirm the reported mode is `hover`, then stop the launch with Ctrl+C. Also
continue testing abrupt process termination separately; cleanup bursts are
best-effort and do not replace watchdog behavior.

## Optional simulated authority check

Only after the zero-command smoke test passes, the simulated FC can be moved
through the same authority states used on the real aircraft:

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'GUIDED'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}"
```

This does not command takeoff because the ROS mode remains `hover`. Confirm
that `/mavros/state` reports `GUIDED` and armed, then return to a manual mode
and disarm through MAVROS or your ground station before ending the test.

The SITL launch uses loopback TCP and cannot connect to the physical serial
flight controller unless `fcu_url` is deliberately overridden.

Official references:

- [ArduPilot Ubuntu build environment](https://ardupilot.org/dev/docs/building-setup-linux.html)
- [ArduPilot SITL usage](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)
- [MAVROS connection URLs](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md)
