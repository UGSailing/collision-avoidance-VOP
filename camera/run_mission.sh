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
python3 start_boat_mission_wo_hailo.py --backend pi --camera-left 0 --camera-right 1 --model yolo_models/duck.pt --camera-depth single --object-height-m 0.175 --calib-yaml calibration_yamls/camera_calibration.yaml --duration 60
