#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIRECTORY="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIRECTORY}/.." && pwd)"
SERVICE_TEMPLATE="${PROJECT_ROOT}/systemd/diy-autonomous-drone.service.in"
ENVIRONMENT_TEMPLATE="${PROJECT_ROOT}/config/systemd.env.example"
SERVICE_NAME="diy-autonomous-drone.service"
SERVICE_USER="${SUDO_USER:-${USER:-}}"

usage() {
    printf '%s\n' \
        'Usage: scripts/install_systemd_service.sh [--user USER]' \
        '' \
        'Installs the unit and safe environment file.' \
        'It deliberately does not enable or start the service.'
}

while (($#)); do
    case "$1" in
        --user)
            (($# >= 2)) || {
                printf '%s\n' '--user needs a value' >&2
                exit 2
            }
            SERVICE_USER="$2"
            shift
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

[[ "$(uname -s)" == Linux ]] || {
    printf '%s\n' 'systemd service installation requires Linux.' >&2
    exit 1
}
command -v systemctl >/dev/null || {
    printf '%s\n' 'systemctl is unavailable.' >&2
    exit 1
}
id "${SERVICE_USER}" >/dev/null 2>&1 || {
    printf 'Unknown service user: %s\n' "${SERVICE_USER}" >&2
    exit 1
}
[[ "${SERVICE_USER}" != root ]] || {
    printf '%s\n' 'Refusing to run the drone service as root.' >&2
    exit 1
}
[[ "${SERVICE_USER}" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || {
    printf '%s\n' 'Service user contains unsupported characters.' >&2
    exit 1
}
[[ "${PROJECT_ROOT}" =~ ^/[A-Za-z0-9._/-]+$ ]] || {
    printf '%s\n' 'Project path contains unsupported characters.' >&2
    exit 1
}
[[ -r "${SERVICE_TEMPLATE}" && -r "${ENVIRONMENT_TEMPLATE}" ]] || {
    printf '%s\n' 'Deployment templates are missing.' >&2
    exit 1
}
[[ -r "${PROJECT_ROOT}/install/setup.bash" ]] || {
    printf '%s\n' 'Build the workspace before installing the service.' >&2
    exit 1
}
[[ -r "${PROJECT_ROOT}/.venv/bin/activate" ]] || {
    printf '%s\n' 'Run scripts/bootstrap_pi.sh before installing the service.' >&2
    exit 1
}

SERVICE_GROUP="$(id -gn "${SERVICE_USER}")"
RENDERED_UNIT="$(mktemp)"
trap 'rm -f -- "${RENDERED_UNIT}"' EXIT
sed \
    -e "s|@SERVICE_USER@|${SERVICE_USER}|g" \
    -e "s|@PROJECT_ROOT@|${PROJECT_ROOT}|g" \
    "${SERVICE_TEMPLATE}" >"${RENDERED_UNIT}"

sudo install -d -o root -g "${SERVICE_GROUP}" -m 0750 \
    /etc/diy-autonomous-drone
if [[ ! -e /etc/diy-autonomous-drone/environment ]]; then
    sudo install -o root -g "${SERVICE_GROUP}" -m 0640 \
        "${ENVIRONMENT_TEMPLATE}" \
        /etc/diy-autonomous-drone/environment
else
    printf '%s\n' \
        'Preserving existing /etc/diy-autonomous-drone/environment.'
fi
sudo install -o root -g root -m 0644 \
    "${RENDERED_UNIT}" "/etc/systemd/system/${SERVICE_NAME}"
sudo systemctl daemon-reload

printf '%s\n' \
    "Installed ${SERVICE_NAME} for user ${SERVICE_USER}." \
    'The service was NOT enabled or started.' \
    'Review /etc/diy-autonomous-drone/environment with propellers removed.' \
    "Then inspect with: sudo systemctl status ${SERVICE_NAME}" \
    "Enable later with: sudo systemctl enable ${SERVICE_NAME}"
