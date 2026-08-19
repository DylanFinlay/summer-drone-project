#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIRECTORY}/.." && pwd)"
ROS_DISTRO_NAME="jazzy"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
VENV_PATH="${PROJECT_ROOT}/.venv"
INSTALL_VISION=true
RUN_TESTS=false

source_environment() {
    # ROS/ament-generated setup files read some optional variables without
    # default expansions, so they are not compatible with Bash nounset mode.
    local nounset_was_enabled=false
    if [[ $- == *u* ]]; then
        nounset_was_enabled=true
        set +u
    fi

    # shellcheck disable=SC1090
    source "$1"

    if [[ "${nounset_was_enabled}" == true ]]; then
        set -u
    fi
}

usage() {
    printf '%s\n' \
        'Usage: scripts/bootstrap_pi.sh [--skip-vision] [--run-tests]' \
        '' \
        'Installs project dependencies and builds the ROS 2 workspace.' \
        'ROS 2 Jazzy must already be installed from the official repository.'
}

while (($#)); do
    case "$1" in
        --skip-vision)
            INSTALL_VISION=false
            ;;
        --run-tests)
            RUN_TESTS=true
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown option: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

if ((EUID == 0)); then
    printf '%s\n' \
        'Run this script as the normal Pi user; it invokes sudo when needed.' >&2
    exit 1
fi
if [[ ! -r /etc/os-release ]]; then
    printf '%s\n' 'Cannot identify the operating system.' >&2
    exit 1
fi

source_environment /etc/os-release
if [[ "${ID:-}" != 'ubuntu' || "${VERSION_ID:-}" != '24.04' ]]; then
    printf '%s\n' \
        'This bootstrap supports Ubuntu 24.04 only; refusing to guess.' >&2
    exit 1
fi
if [[ ! -r "${ROS_SETUP}" ]]; then
    printf 'ROS 2 Jazzy is missing: %s\n' "${ROS_SETUP}" >&2
    printf '%s\n' \
        'Install it from https://docs.ros.org/en/jazzy/Installation.html' >&2
    exit 1
fi
if [[ ! -f "${PROJECT_ROOT}/package.xml" ]]; then
    printf 'Package metadata missing under %s\n' "${PROJECT_ROOT}" >&2
    exit 1
fi

APT_PACKAGES=(
    python3-colcon-common-extensions
    python3-opencv
    python3-pip
    python3-rosdep
    python3-venv
    ros-jazzy-mavros
    ros-jazzy-mavros-msgs
    ros-jazzy-rosbag2
    ros-jazzy-rosbag2-storage-default-plugins
)

printf '%s\n' 'Installing Ubuntu and ROS package dependencies...'
sudo apt-get update
sudo apt-get install -y "${APT_PACKAGES[@]}"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update

source_environment "${ROS_SETUP}"
rosdep install \
    --from-paths "${PROJECT_ROOT}" \
    --ignore-src \
    --rosdistro "${ROS_DISTRO_NAME}" \
    -y

if [[ ! -d "${VENV_PATH}" ]]; then
    python3 -m venv --system-site-packages "${VENV_PATH}"
fi
touch "${VENV_PATH}/COLCON_IGNORE"
source_environment "${VENV_PATH}/bin/activate"

if [[ "${INSTALL_VISION}" == true ]]; then
    python -m pip install --require-virtualenv \
        -r "${PROJECT_ROOT}/requirements-vision.txt"
else
    printf '%s\n' 'Skipping optional Ultralytics vision dependency.'
fi

printf '%s\n' 'Installing the MAVROS GeographicLib datasets...'
MAVROS_PREFIX="$(ros2 pkg prefix mavros)"
MAVROS_DATASET_INSTALLER="${MAVROS_PREFIX}/lib/mavros/install_geographiclib_datasets.sh"
if [[ ! -x "${MAVROS_DATASET_INSTALLER}" ]]; then
    printf 'MAVROS dataset installer is missing or not executable: %s\n' \
        "${MAVROS_DATASET_INSTALLER}" >&2
    exit 1
fi
sudo "${MAVROS_DATASET_INSTALLER}"

printf '%s\n' 'Building the ROS workspace...'
cd "${PROJECT_ROOT}"
colcon build --symlink-install --packages-select diy_autonomous_drone

if [[ "${RUN_TESTS}" == true ]]; then
    source_environment "${PROJECT_ROOT}/install/setup.bash"
    colcon test --packages-select diy_autonomous_drone
    colcon test-result --verbose
fi

printf '%s\n' \
    'Bootstrap complete.' \
    "Activate with: source ${VENV_PATH}/bin/activate" \
    "Then source:   source ${PROJECT_ROOT}/install/setup.bash"
