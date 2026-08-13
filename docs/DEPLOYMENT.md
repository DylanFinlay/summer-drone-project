# Raspberry Pi deployment

These scripts target the project hardware image: Ubuntu 24.04 on a 64-bit
Raspberry Pi 5 with ROS 2 Jazzy installed from the official ROS repository.
They intentionally do not install ROS itself, alter ArduPilot, enable a boot
service, or start the flight stack.

## 1. Bootstrap and build

Install ROS 2 Jazzy first, clone this repository, and run from its root:

```bash
scripts/bootstrap_pi.sh --run-tests
```

The script verifies Ubuntu and ROS versions, installs apt/rosdep dependencies,
creates `.venv` with the system Python and system site packages, installs the
optional Ultralytics dependency, installs MAVROS GeographicLib datasets, and
builds the package with colcon. Use `--skip-vision` only for a non-vision SITL
or control-stack installation.

Run the bootstrap again after dependency changes. Rebuild after source-only
changes with:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --symlink-install --packages-select diy_autonomous_drone
```

## 2. Verify interactively

Do not install a boot service first. Follow the README test order and prove the
default launch interactively with propellers removed:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch diy_autonomous_drone drone_autonomous.launch.py \
  enable_fc_interface:=false
```

Then verify MAVROS connection, FC state, the physical RC authority switch, and
zero-command behavior using the ArduPilot RC checklist. Cache required YOLO
models before an offline demo.

## 3. Install the systemd unit

Only after the interactive checks pass:

```bash
scripts/install_systemd_service.sh
```

The installer renders the current absolute repository path and service user,
copies the unit, and runs `systemctl daemon-reload`. It does **not** enable,
start, or restart the service. An existing environment file is preserved.

The initial `/etc/diy-autonomous-drone/environment` configuration is
deliberately inert: `hover`, the `bench` profile, and no FC interface, camera,
gesture recognition, RC auxiliary selection, or logging. Edit it only with
propellers removed. The runtime wrapper validates modes, booleans, feature
dependencies, and RC channel configuration before ROS starts. The service
intentionally uses the repository's tested base parameters and named profile
overlays instead of accepting an arbitrary parameter file.

Test one manual service start before enabling boot startup:

```bash
sudo systemctl start diy-autonomous-drone.service
sudo systemctl status diy-autonomous-drone.service
journalctl -u diy-autonomous-drone.service -f
```

Stop and disable it with:

```bash
sudo systemctl stop diy-autonomous-drone.service
sudo systemctl disable diy-autonomous-drone.service
```

The unit sends `SIGINT` and allows 20 seconds for ROS launch and its nodes to
publish their best-effort shutdown zeros. Unexpected failures restart at most
within systemd's configured start limit; clean operator stops do not restart.

## 4. Explicitly enable boot startup

Only after a successful service-mode props-off test and confirmation that the
environment still starts in `hover`:

```bash
sudo systemctl enable diy-autonomous-drone.service
```

Enabling the service is operational convenience, not a flight failsafe. The
physical RC flight-mode switch, FC gate, watchdogs, ArduPilot failsafes, and
normal pre-flight checks remain required on every run.
