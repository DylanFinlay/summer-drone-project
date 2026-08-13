"""Safe launch settings for perception-only recorded-video testing."""


def recorded_video_stack_arguments(
    video_file,
    loop_video,
    enable_object_detection,
    enable_flight_logging,
    autonomy_mode='active_track',
    enable_gesture_control='false',
    enable_gesture_recognition='false',
):
    """Return main-stack arguments with FC connectivity always disabled."""
    if video_file is None:
        raise ValueError('video_file launch value is required')
    return {
        'configuration_profile': 'bench',
        'autonomy_mode': autonomy_mode,
        'enable_gesture_control': enable_gesture_control,
        'enable_gesture_recognition': enable_gesture_recognition,
        'enable_vision': 'true',
        'enable_object_detection': enable_object_detection,
        'video_file': video_file,
        'loop_video': loop_video,
        'enable_tracking': 'true',
        'enable_rc_aux_mode_selection': 'false',
        'enable_fc_interface': 'false',
        'enable_flight_logging': enable_flight_logging,
    }
