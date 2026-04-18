#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
HAILO_APPS_ROOT_DEFAULT="$HOME/Documents/hailo-apps"
HAILO_APPS_ROOT="${HAILO_APPS_ROOT:-$HAILO_APPS_ROOT_DEFAULT}"

# Headless Pi runs should not depend on a local monitor or X session.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export DISPLAY="${DISPLAY:-}"

if [ -f "$HAILO_APPS_ROOT/setup_env.sh" ]; then
	# shellcheck disable=SC1090
	source "$HAILO_APPS_ROOT/setup_env.sh"
elif [ -f "$HAILO_APPS_ROOT/hailo_apps/setup_env.sh" ]; then
	# shellcheck disable=SC1090
	source "$HAILO_APPS_ROOT/hailo_apps/setup_env.sh"
else
	echo "Error: could not find Hailo setup_env.sh under $HAILO_APPS_ROOT" >&2
	exit 1
fi

exec python3 "$PROJECT_ROOT/camera/first_working_camerasystem.py" "$@"