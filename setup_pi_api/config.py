from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
API_HOST = "0.0.0.0"
API_PORT = 8000
LOG_DIR = REPO_ROOT / "setup_pi_api" / "logs"

# Adjust these commands on the Pi if your deployment scripts need different flags.
CAMERA_COMMAND = [
    sys.executable,
    str(REPO_ROOT / "camera" / "start_boat_mission.py"),
    "--duration",
    "86400",
]
CAMERA_CWD = REPO_ROOT

CONTROL_COMMAND = [
    sys.executable,
    str(REPO_ROOT / "control" / "boat_main.py"),
]
CONTROL_CWD = REPO_ROOT

LOG_TAIL_LINES = 80
STOP_TIMEOUT_SECONDS = 5.0
