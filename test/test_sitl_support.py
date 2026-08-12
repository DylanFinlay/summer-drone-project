"""Tests for deterministic ArduPilot SITL command construction."""

import unittest

from diy_autonomous_drone.sitl_support import sim_vehicle_arguments


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


if __name__ == '__main__':
    unittest.main()
