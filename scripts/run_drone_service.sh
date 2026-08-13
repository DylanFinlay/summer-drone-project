#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIRECTORY}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
VENV_SETUP="${PROJECT_ROOT}/.venv/bin/activate"
WORKSPACE_SETUP="${PROJECT_ROOT}/install/setup.bash"

DRONE_CONFIGURATION_PROFILE="${DRONE_CONFIGURATION_PROFILE:-bench}"
DRONE_AUTONOMY_MODE="${DRONE_AUTONOMY_MODE:-hover}"
DRONE_ENABLE_VISION="${DRONE_ENABLE_VISION:-false}"
DRONE_ENABLE_OBJECT_DETECTION="${DRONE_ENABLE_OBJECT_DETECTION:-true}"
DRONE_ENABLE_GESTURE_CONTROL="${DRONE_ENABLE_GESTURE_CONTROL:-false}"
DRONE_ENABLE_GESTURE_RECOGNITION="${DRONE_ENABLE_GESTURE_RECOGNITION:-false}"
DRONE_ENABLE_TRACKING="${DRONE_ENABLE_TRACKING:-true}"
DRONE_ENABLE_RC_AUX_MODE_SELECTION="${DRONE_ENABLE_RC_AUX_MODE_SELECTION:-false}"
DRONE_RC_AUX_CHANNEL="${DRONE_RC_AUX_CHANNEL:-0}"
DRONE_ENABLE_FC_INTERFACE="${DRONE_ENABLE_FC_INTERFACE:-false}"
DRONE_ENABLE_FLIGHT_LOGGING="${DRONE_ENABLE_FLIGHT_LOGGING:-false}"
DRONE_FCU_URL="${DRONE_FCU_URL:-serial:///dev/ttyAMA0:57600}"
DRONE_GCS_URL="${DRONE_GCS_URL:-}"
DRONE_ROS_DOMAIN_ID="${DRONE_ROS_DOMAIN_ID:-0}"

fail() {
    printf 'Service configuration error: %s\n' "$1" >&2
    exit 2
}

require_boolean() {
    case "$2" in
        true|false) ;;
        *) fail "$1 must be true or false" ;;
    esac
}

[[ "${ROS_DISTRO_NAME}" == jazzy ]] || \
    fail 'this deployment supports ROS_DISTRO=jazzy only'

for setting in \
    DRONE_ENABLE_VISION \
    DRONE_ENABLE_OBJECT_DETECTION \
    DRONE_ENABLE_GESTURE_CONTROL \
    DRONE_ENABLE_GESTURE_RECOGNITION \
    DRONE_ENABLE_TRACKING \
    DRONE_ENABLE_RC_AUX_MODE_SELECTION \
    DRONE_ENABLE_FC_INTERFACE \
    DRONE_ENABLE_FLIGHT_LOGGING
do
    require_boolean "${setting}" "${!setting}"
done

case "${DRONE_CONFIGURATION_PROFILE}" in
    simulation|bench|outdoor_demo) ;;
    *) fail 'DRONE_CONFIGURATION_PROFILE is invalid' ;;
esac
case "${DRONE_AUTONOMY_MODE}" in
    hover|active_track|gesture_control) ;;
    *) fail 'DRONE_AUTONOMY_MODE is invalid' ;;
esac
[[ "${DRONE_RC_AUX_CHANNEL}" =~ ^[0-9]+$ ]] || \
    fail 'DRONE_RC_AUX_CHANNEL must be a nonnegative integer'
[[ "${DRONE_ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]] || \
    fail 'DRONE_ROS_DOMAIN_ID must be a nonnegative integer'
((10#${DRONE_RC_AUX_CHANNEL} <= 18)) || \
    fail 'DRONE_RC_AUX_CHANNEL cannot exceed MAVROS channel 18'
((10#${DRONE_ROS_DOMAIN_ID} <= 232)) || \
    fail 'DRONE_ROS_DOMAIN_ID must be between 0 and 232'

if [[ "${DRONE_AUTONOMY_MODE}" == gesture_control && \
        "${DRONE_ENABLE_GESTURE_CONTROL}" != true ]]; then
    fail 'gesture mode requires DRONE_ENABLE_GESTURE_CONTROL=true'
fi
if [[ "${DRONE_AUTONOMY_MODE}" == active_track && \
        "${DRONE_ENABLE_VISION}" != true ]]; then
    fail 'active tracking requires DRONE_ENABLE_VISION=true'
fi
if [[ "${DRONE_AUTONOMY_MODE}" == gesture_control && \
        "${DRONE_ENABLE_GESTURE_RECOGNITION}" != true ]]; then
    fail 'gesture mode requires DRONE_ENABLE_GESTURE_RECOGNITION=true'
fi
if [[ "${DRONE_ENABLE_GESTURE_RECOGNITION}" == true && \
        "${DRONE_ENABLE_VISION}" != true ]]; then
    fail 'gesture recognition requires DRONE_ENABLE_VISION=true'
fi
if [[ "${DRONE_ENABLE_GESTURE_RECOGNITION}" == true && \
        "${DRONE_ENABLE_OBJECT_DETECTION}" != true ]]; then
    fail 'gesture recognition requires object detection'
fi
if [[ "${DRONE_ENABLE_RC_AUX_MODE_SELECTION}" == true && \
        "${DRONE_RC_AUX_CHANNEL}" == 0 ]]; then
    fail 'RC auxiliary selection requires a calibrated nonzero channel'
fi
if [[ "${DRONE_ENABLE_RC_AUX_MODE_SELECTION}" == true && \
        "${DRONE_ENABLE_FC_INTERFACE}" != true ]]; then
    fail 'RC auxiliary selection requires the MAVROS/FC interface'
fi
if [[ "${DRONE_ENABLE_RC_AUX_MODE_SELECTION}" == true && \
        "${DRONE_ENABLE_VISION}" != true ]]; then
    fail 'RC auxiliary selection requires vision for its tracking mapping'
fi
if [[ "${DRONE_ENABLE_FC_INTERFACE}" == true && \
        -z "${DRONE_FCU_URL}" ]]; then
    fail 'the FC interface requires DRONE_FCU_URL'
fi
[[ -r "${ROS_SETUP}" ]] || fail "ROS setup is missing: ${ROS_SETUP}"
[[ -r "${VENV_SETUP}" ]] || fail "virtual environment is missing: ${VENV_SETUP}"
[[ -r "${WORKSPACE_SETUP}" ]] || \
    fail "workspace is not built: ${WORKSPACE_SETUP}"

# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${VENV_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"
export ROS_DOMAIN_ID="${DRONE_ROS_DOMAIN_ID}"

LAUNCH_ARGUMENTS=(
    "configuration_profile:=${DRONE_CONFIGURATION_PROFILE}"
    "autonomy_mode:=${DRONE_AUTONOMY_MODE}"
    "enable_vision:=${DRONE_ENABLE_VISION}"
    "enable_object_detection:=${DRONE_ENABLE_OBJECT_DETECTION}"
    "enable_gesture_control:=${DRONE_ENABLE_GESTURE_CONTROL}"
    "enable_gesture_recognition:=${DRONE_ENABLE_GESTURE_RECOGNITION}"
    "enable_tracking:=${DRONE_ENABLE_TRACKING}"
    "enable_rc_aux_mode_selection:=${DRONE_ENABLE_RC_AUX_MODE_SELECTION}"
    "rc_aux_channel:=${DRONE_RC_AUX_CHANNEL}"
    "enable_fc_interface:=${DRONE_ENABLE_FC_INTERFACE}"
    "enable_flight_logging:=${DRONE_ENABLE_FLIGHT_LOGGING}"
    "fcu_url:=${DRONE_FCU_URL}"
    "gcs_url:=${DRONE_GCS_URL}"
)

exec ros2 launch diy_autonomous_drone drone_autonomous.launch.py \
    "${LAUNCH_ARGUMENTS[@]}"
