#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CAMERA_PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
HAILO_APPS_ROOT_DEFAULT="$HOME/Documents/hailo-apps"
HAILO_APPS_ROOT="${HAILO_APPS_ROOT:-$HAILO_APPS_ROOT_DEFAULT}"
SETUP_ENV_SCRIPT=""

# Headless Pi runs should not depend on a local monitor or X session.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export DISPLAY="${DISPLAY:-}"

if [ -f "$HAILO_APPS_ROOT/setup_env.sh" ]; then
	SETUP_ENV_SCRIPT="$HAILO_APPS_ROOT/setup_env.sh"
elif [ -f "$HAILO_APPS_ROOT/hailo_apps/setup_env.sh" ]; then
	SETUP_ENV_SCRIPT="$HAILO_APPS_ROOT/hailo_apps/setup_env.sh"
else
	echo "Error: could not find Hailo setup_env.sh under $HAILO_APPS_ROOT" >&2
	exit 1
fi

# Source from the script directory so relative paths inside setup_env.sh resolve correctly.
SETUP_ENV_DIR="$(cd -- "$(dirname -- "$SETUP_ENV_SCRIPT")" && pwd)"
SETUP_ENV_NAME="$(basename -- "$SETUP_ENV_SCRIPT")"
pushd "$SETUP_ENV_DIR" >/dev/null

# Hailo's setup script checks shell vars that can be unset under `set -u`.
set +u
# shellcheck disable=SC1090
source "./$SETUP_ENV_NAME"
set -u

popd >/dev/null

exec python3 "$CAMERA_PROJECT_ROOT/camera/first_working_camerasystem.py" "$@"