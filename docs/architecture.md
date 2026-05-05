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

The **Camera System** (`camera/`)... TODO

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
TODO