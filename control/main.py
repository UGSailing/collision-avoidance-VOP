import threading
import subprocess
import pandas as pd
from datetime import datetime
from pathlib import Path
from path_planning.occupancy_mapper import OccupancyMapper

# Mock imports (ensure these exist in your project)
from data_collection import read_CAN
from path_planning.v2 import update_path
from path_execution import follow_path
import sys

PATH_UPDATE_INTERVAL = 3 

def collection_loop(stop_event, run_dir):
    while not stop_event.is_set():
        try:
            read_CAN(run_dir)
            stop_event.wait(10)
        except Exception as e:
            print(f"Data collection error: {e}")

def planning_loop(stop_event, run_dir):
    # Initialize the mapper
    mapper = OccupancyMapper(resolution=0.2, grid_size_m=40)
    
    while not stop_event.is_set():
        try:
            # 1. Generate the grid with a 3.0 meter hitbox
            grid = mapper.create_grid(run_dir / 'points.csv', hitbox_radius_m=3.0)
            
            # 2. Pass BOTH the grid and the mapper to keep origins perfectly synced!
            update_path(run_dir, grid, mapper) 
            
            print(f"Path updated. Obstacles detected: {grid.sum() > 0}")
        except Exception as e:
            print(f"Path planning error: {e}")
        stop_event.wait(PATH_UPDATE_INTERVAL)

def execution_loop(stop_event, run_dir):
    while not stop_event.is_set():
        try:
            follow_path(run_dir)
            stop_event.wait(10)
        except Exception as e:
            print(f"Path execution error: {e}")

if __name__ == "__main__":
    # Setup base directory (root of the project)
    base_dir = Path(__file__).parent.parent
    
    # Create unique run directory
    run_dir = base_dir / "control/runs" / datetime.now().strftime("%Y-%m-%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    # Setup essential files for this run
    points_file = run_dir / 'points.csv'
    points_file.touch()
    (run_dir / 'path.csv').touch()

    # Seed data with initial obstacles and destination
    data = {
        'id': [0, 1, 2, 0],
        'category': ['gps', 'gps', 'gps', 'destination'],
        'latitude': [51.011466, 51.011401, 51.011315, 51.011504],
        'longitude': [3.708731, 3.709055, 3.709353, 3.708728]
    }
    pd.DataFrame(data).to_csv(points_file, index=False)

    stop_event = threading.Event()

    # Start background threads for logic loops
    threading.Thread(target=collection_loop, args=(stop_event, run_dir), daemon=True).start()
    threading.Thread(target=planning_loop, args=(stop_event, run_dir), daemon=True).start()
    threading.Thread(target=execution_loop, args=(stop_event, run_dir), daemon=True).start()

    # Simple interactive CLI
    map_process = None
    try:
        while True:
            resp = input("type 'map' to launch map or 'exit' to exit\n").strip().lower()
            if resp == "exit":
                break
            elif resp == "map":
                if map_process is None or map_process.poll() is not None:
                    # Pathing for Linux environment
                    if sys.platform == "win32":
                        python_exe = base_dir / 'control/.venv/Scripts/python.exe'
                    else:
                        python_exe = base_dir / 'control/.venv/bin/python'
                    script = base_dir / 'control/visualisation/live_map.py'
                    
                    if not python_exe.exists():
                        print(f"Error: Python executable not found at {python_exe}")
                        continue
                    if not script.exists():
                        print(f"Error: Visualization script not found at {script}")
                        continue

                    print("Launching live map...")
                    map_process = subprocess.Popen([str(python_exe), str(script), str(run_dir)])
                else:
                    print("Map is already running.")
            else:
                print("Invalid input.")
    except KeyboardInterrupt:
        print("\nShutdown signal received.")

    # --- CLEANUP LOGIC ---
    print("Shutting down...")
    stop_event.set() # Stop the background threads

    if map_process is not None:
        print("Terminating map process...")
        map_process.terminate() # Gracefully stop the Dash server
        try:
            map_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print("Map process not responding, force killing...")
            map_process.kill() # Force kill if it hangs

    print("Cleanup complete. Program exited.")