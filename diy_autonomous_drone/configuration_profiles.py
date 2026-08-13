"""Validated names for installed ROS parameter profile overlays."""


CONFIGURATION_PROFILES = (
    'simulation',
    'bench',
    'outdoor_demo',
)


def profile_filename(profile_name: str) -> str:
    """Return the installed filename for one supported profile name."""
    name = str(profile_name).strip().lower()
    if name not in CONFIGURATION_PROFILES:
        raise ValueError(
            'configuration profile must be one of: %s'
            % ', '.join(CONFIGURATION_PROFILES)
        )
    return '%s.yaml' % name
