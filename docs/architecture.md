# Architecture & Data Flow

This project is divided into two primary subsystems that communicate over a wired Ethernet connection.

## Subsystems

### 1. Control Pi
The Control Pi acts as the main brain for navigation and path following.
- Receives GPS coordinates over GPIO (Ardusimple RTK).
- Sends commands to the boat's motors via an ESP32 connected over USB.
- Ingests obstacle data to plan paths dynamically.

### 2. Camera Pi (Obstacle/Object Detection)
The Camera Pi detects objects with the onboard cameras.
- Connects to an AI Hat+ processing unit to run YOLO-based object detection locally on a Hailo chip.
- Generates distance and azimuth angles using single-camera estimation or dual-camera stereo depth calculations.
- Sends detected targets and coordinates to the Control Pi over the Ethernet TCP socket.

## Internal Code Structure & Loops

The **Control System** (`control/`) executes three concurrent background threads from `main.py` (run manually) or `boat_main.py` (autorun on ship boot):
1. **Collection Loop (**`data_collection/`**):** Manages connecting to the GNSS RTK, fetching NTRIP corrections, and syncing obstacle data via TCP. It continuously logs location and obstacle data to a shared `points.csv`.
2. **Planning Loop (**`path_planning/`**):** Reads the aggregated coordinate data (from `points.csv`), maps obstacles onto an occupancy grid (`OccupancyMapper`), and computes a collision-free route (`update_path`), saving it to `path.csv`.
3. **Execution Loop (**`path_execution/`**):** The `PathFollower` reads `path.csv` to guide the physical boat along the planned waypoints.

The **Camera System** (`camera/`)... TOD

## Data Exchanged and Stored
Whenever a run is started, a new directory is created locally (`/runs/YYYY-MM-DD_HHMMSS/` or `/recordings/YYYY-MM-DD_HHMMSS/`) to store that run's data.
- `points.csv`: Central truth for coordinates. Rows consist of different "categories" such as GPS (current boat position), Destination, or Camera (detected obstacles).
- `path.csv`: The planned path coordinates.
- `detections.jsonl`: Camera logging detailing detection bounding boxes, confidence, distance offset, and timestamps.

## Other code present in repo
### Control
- **`ardusimpleRTK/`**: Standalone scripts to test and configure the Ardusimple RTK GPS receiver.
- **`connection tests/`**: Independent scripts to test raw TCP connections and the USB serial connection to the ESP32.
- **`visualisation/`**: Houses `live_map.py`, which launches a local dashboard via Dash to visualize the path, boat coordinates, geofence and obstables interactively.

### Camera
- **`calib_good_calibration_imgs/`**: A directory with a subset of images that is particularly better for the camera calibration process.
- **`calibration_npzs/`**: Contains the calibration file for dual camera calibration in npz format, which can be used for rectification.
- **`calibration_yamls/camera_calibration.yaml`**: Contains the intrinsic parameters of the camera, used for rectification.
- **`calibrations/`**: Different calibration files: single vs. dual camera calibration. Also contains rectification files for both single and dual camera.
- **`captures/`**: A directory to store photos taken of the checkboard pattern by the camera of the boat for camera calibration.
- **`depth_calculation/`**: Contains the scripts to calculate depth from the detected bounding boxes. This includes both single camera and dual camera depth calculation.
- **`dual_calib_images/`**: A directory with the images taken for dual camera calibration. These images are taken with both cameras at the same time, and they are used to calculate the extrinsic parameters between the two cameras.
- **`dual_calib_images_rectified/`**: A directory with the rectified images from the dual camera calibration.
- **`images_to_test_models/`**: A directory with images from the Internet and images of the duck to test the different YOLO models on. These images are not all taken from the boat's camera, so they may not be representative of the actual detection performance on the boat. They are only meant to be used for testing the models on a variety of objects and backgrounds.
- **`prototype_Hailo_pipeline/`**: Files to run different versions of Hailo detection on the AI Hat+.
- **`recordings/`**: A directory to store the recordings of the camera feed and the corresponding detections in jsonl format. Each recording is stored in a subdirectory named with the timestamp of when the recording was started.
- **`separate_files/`**: Contains separate scripts for different parts of the camera pipeline, e.g. a file to test the TCP connection to the Control Pi.
- **`yolo_models/`**: Contains the models you can choose between to run object detection. E.g. duck.pt contains the trained weights for the duck detection model.