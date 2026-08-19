"""Static safety and syntax tests for Pi deployment assets."""

import os
from pathlib import Path
import re
import subprocess
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = (
    PROJECT_ROOT / 'scripts' / 'bootstrap_pi.sh',
    PROJECT_ROOT / 'scripts' / 'install_systemd_service.sh',
    PROJECT_ROOT / 'scripts' / 'run_drone_service.sh',
)


class TestDeploymentAssets(unittest.TestCase):
    """Ensure deployment defaults cannot silently open flight authority."""

    def test_shell_scripts_parse(self):
        """Every deployment script is valid Bash before reaching the Pi."""
        for script in SCRIPTS:
            with self.subTest(script=script.name):
                result = subprocess.run(
                    ['bash', '-n', str(script)],
                    check=False,
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertTrue(os.access(script, os.X_OK))

    def test_script_help_is_read_only_and_available_off_pi(self):
        """Users can inspect usage without triggering OS checks or writes."""
        for script in SCRIPTS[:2]:
            with self.subTest(script=script.name):
                result = subprocess.run(
                    [str(script), '--help'],
                    check=False,
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_installed_environment_defaults_are_inert(self):
        """A newly installed service cannot connect to the FC or move."""
        contents = (
            PROJECT_ROOT / 'config' / 'systemd.env.example'
        ).read_text()
        self.assertIn('DRONE_CONFIGURATION_PROFILE=bench', contents)
        self.assertIn('DRONE_AUTONOMY_MODE=hover', contents)
        self.assertIn('DRONE_ENABLE_VISION=false', contents)
        self.assertIn('DRONE_ENABLE_GESTURE_CONTROL=false', contents)
        self.assertIn('DRONE_ENABLE_RC_AUX_MODE_SELECTION=false', contents)
        self.assertIn('DRONE_ENABLE_FC_INTERFACE=false', contents)

    def test_service_uses_graceful_stop_and_bounded_restart(self):
        """The unit gives ROS time to stop and avoids restart loops."""
        unit = (
            PROJECT_ROOT / 'systemd' /
            'diy-autonomous-drone.service.in'
        ).read_text()
        self.assertIn('KillSignal=SIGINT', unit)
        self.assertIn('TimeoutStopSec=20', unit)
        self.assertIn('Restart=on-failure', unit)
        self.assertNotIn('Restart=always', unit)
        self.assertIn('StartLimitBurst=3', unit)

    def test_installer_does_not_enable_or_start_service(self):
        """Installing files alone never opts the aircraft into boot launch."""
        installer = (
            PROJECT_ROOT / 'scripts' / 'install_systemd_service.sh'
        ).read_text()
        self.assertIn('systemctl daemon-reload', installer)
        self.assertIn('SERVICE_USER}" != root', installer)
        mutating_command = re.compile(
            r'^\s*(?:sudo\s+)?systemctl\s+(?:enable|start|restart)\b',
            re.MULTILINE,
        )
        self.assertIsNone(mutating_command.search(installer))

    def test_bootstrap_uses_ros_compatible_virtual_environment(self):
        """The setup follows ROS system-Python and colcon isolation needs."""
        bootstrap = (
            PROJECT_ROOT / 'scripts' / 'bootstrap_pi.sh'
        ).read_text()
        self.assertIn('--system-site-packages', bootstrap)
        self.assertIn('COLCON_IGNORE', bootstrap)
        self.assertIn('rosdep install', bootstrap)
        self.assertIn('colcon build --symlink-install', bootstrap)
        self.assertIn('EUID == 0', bootstrap)

    def test_bootstrap_sources_ros_without_nounset(self):
        """ROS-generated setup scripts run outside Bash nounset mode."""
        bootstrap = (
            PROJECT_ROOT / 'scripts' / 'bootstrap_pi.sh'
        ).read_text()
        self.assertIn('source_environment "${ROS_SETUP}"', bootstrap)
        self.assertIn('set +u', bootstrap)
        self.assertIn('set -u', bootstrap)

    def test_bootstrap_runs_mavros_dataset_installer_as_root(self):
        """The current MAVROS GeographicLib installer receives root access."""
        bootstrap = (
            PROJECT_ROOT / 'scripts' / 'bootstrap_pi.sh'
        ).read_text()
        self.assertIn('ros2 pkg prefix mavros', bootstrap)
        self.assertIn('sudo "${MAVROS_DATASET_INSTALLER}"', bootstrap)
        self.assertNotIn(
            'ros2 run mavros install_geographiclib_datasets.sh', bootstrap)

    def test_runtime_wrapper_uses_array_arguments_without_eval(self):
        """Environment values cannot become shell command fragments."""
        runner = (
            PROJECT_ROOT / 'scripts' / 'run_drone_service.sh'
        ).read_text()
        self.assertIn('LAUNCH_ARGUMENTS=(', runner)
        self.assertIn('"${LAUNCH_ARGUMENTS[@]}"', runner)
        self.assertNotIn('eval ', runner)

    def test_runtime_rejects_unsafe_feature_combinations_early(self):
        """Invalid gesture and RC settings fail before ROS is launched."""
        runner = PROJECT_ROOT / 'scripts' / 'run_drone_service.sh'
        cases = (
            {'DRONE_AUTONOMY_MODE': 'gesture_control'},
            {'DRONE_ENABLE_RC_AUX_MODE_SELECTION': 'true'},
            {
                'DRONE_AUTONOMY_MODE': 'active_track',
                'DRONE_ENABLE_VISION': 'false',
            },
            {'DRONE_ROS_DOMAIN_ID': '233'},
        )
        for environment in cases:
            with self.subTest(environment=environment):
                result = subprocess.run(
                    [str(runner)],
                    check=False,
                    capture_output=True,
                    text=True,
                    env={**os.environ, **environment},
                )
                self.assertEqual(result.returncode, 2)
                self.assertIn('Service configuration error', result.stderr)


if __name__ == '__main__':
    unittest.main()
