"""Helpers shared by the ArduPilot SITL launch configuration."""


def sim_vehicle_arguments(
    vehicle,
    frame,
    instance,
    speedup,
    state_directory,
):
    """Build deterministic ``sim_vehicle.py`` arguments for one vehicle."""
    return [
        '-v',
        vehicle,
        '-f',
        frame,
        '-I',
        instance,
        '--speedup',
        speedup,
        '--use-dir',
        state_directory,
        '--no-mavproxy',
    ]
