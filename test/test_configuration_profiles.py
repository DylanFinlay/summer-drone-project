"""Tests for safe, installed configuration-profile selection."""

from pathlib import Path
import unittest

from diy_autonomous_drone.configuration_profiles import (
    CONFIGURATION_PROFILES,
    profile_filename,
)


class TestConfigurationProfiles(unittest.TestCase):
    """Verify profile names and required safety overrides."""

    def test_expected_profiles_are_available(self):
        """The three planned operating contexts have explicit profiles."""
        self.assertEqual(CONFIGURATION_PROFILES, (
            'simulation', 'bench', 'outdoor_demo'))

    def test_filename_rejects_unknown_or_path_like_names(self):
        """Profile selection cannot escape the installed profile directory."""
        self.assertEqual(profile_filename('BENCH'), 'bench.yaml')
        for invalid in ('unknown', '../bench', '', 'outdoor-demo'):
            with self.subTest(profile=invalid):
                with self.assertRaises(ValueError):
                    profile_filename(invalid)

    def test_profile_files_keep_fc_authority_gates_enabled(self):
        """No profile disables the Guided-mode or armed-state gates."""
        profile_directory = (
            Path(__file__).resolve().parents[1] / 'config' / 'profiles')
        for profile in CONFIGURATION_PROFILES:
            with self.subTest(profile=profile):
                contents = (
                    profile_directory / profile_filename(profile)
                ).read_text()
                self.assertIn('require_guided_mode: true', contents)
                self.assertIn('require_armed: true', contents)

    def test_bench_profile_has_lower_motion_limits_than_outdoor(self):
        """The default bench context is deliberately movement-constrained."""
        profile_directory = (
            Path(__file__).resolve().parents[1] / 'config' / 'profiles')
        bench = (profile_directory / 'bench.yaml').read_text()
        outdoor = (profile_directory / 'outdoor_demo.yaml').read_text()
        self.assertIn('max_linear_speed: 0.10', bench)
        self.assertIn('max_linear_speed: 0.25', outdoor)


if __name__ == '__main__':
    unittest.main()
