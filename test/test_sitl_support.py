"""Tests for deterministic ArduPilot SITL command construction."""

from pathlib import Path
import unittest

from diy_autonomous_drone.sitl_support import sim_vehicle_arguments


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class TestSITLSupport(unittest.TestCase):
    """Verify the safe single-vehicle simulator defaults."""

    def test_sim_vehicle_arguments(self):
        """The launch disables MAVProxy and isolates persistent state."""
        arguments = sim_vehicle_arguments(
            vehicle='ArduCopter',
            frame='quad',
            instance='0',
            speedup='1',
            state_directory='/tmp/drone-sitl',
        )
        self.assertEqual(arguments, [
            '-v', 'ArduCopter',
            '-f', 'quad',
            '-I', '0',
            '--speedup', '1',
            '--use-dir', '/tmp/drone-sitl',
            '--no-mavproxy',
        ])

    def test_mavros_container_keeps_internal_node_names(self):
        """A global node-name remap cannot collapse MAVROS plugin services."""
        launch_file = (
            PROJECT_ROOT / 'launch' / 'drone_autonomous.launch.py'
        ).read_text()
        self.assertIn("namespace='mavros'", launch_file)
        self.assertNotIn("name='mavros_node'", launch_file)


if __name__ == '__main__':
    unittest.main()
