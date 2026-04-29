#!/bin/bash

# Script to start the boat mission on Raspberry Pi
# Activates virtual environment and runs start_boat_mission_wo_hailo.py

# run with
# chmod +x run_mission.sh
# ./run_mission.sh

# Navigate to the current directory (camera folder)
cd "$(dirname "$0")" || exit 1

# Activate the virtual environment
source .venv/bin/activate

# Run the Python script
python start_boat_mission_wo_hailo.py
