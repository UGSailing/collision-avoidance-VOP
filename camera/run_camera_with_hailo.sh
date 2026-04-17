#!/bin/bash

# To run this script: 
# Documents/collision-avoidance-VOP/camera/run_camera_with_hailo.sh --input rpi --object-height 0.23

# Activate the Hailo environment
source Documents/hailo-apps/setup_env.sh

# Run the camera system with all passed arguments
python3 Documents/collision-avoidance-VOP/camera/first_working_camerasystem.py "$@"