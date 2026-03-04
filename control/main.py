import pandas as pd
import threading
import subprocess
import sys
from pathlib import Path
from datetime import datetime
from data_collection import read_CAN
from path_planning import update_path, OccupancyMapper, config
from path_execution import follow_path


def collection_loop(stop_event: threading.Event, run_dir: Path):
    while not stop_event.is_set():
        try:
            read_CAN(run_dir)
            stop_event.wait(10)
        except Exception as e:
            print(f"Data collection error: {e}")

def planning_loop(stop_event: threading.Event, run_dir: Path):
    # initialize the mapper
    mapper = OccupancyMapper(resolution=config.GRID_RESOLUTION, grid_size=config.GRID_SIZE, hitbox_radius=config.HITBOX_RADIUS)
    grid = mapper.create_grid(run_dir / 'points.csv')

    while not stop_event.is_set():
        try:
            # 1. incrementally update the grid (full rebuild only when origin shifts)
            grid = mapper.update_grid(run_dir / 'points.csv')
            
            # 2. pass BOTH the grid and the mapper to keep origins perfectly synced!
            update_path(run_dir, grid, mapper) 
            
            print(f"Path updated. Obstacles detected: {grid.sum() > 0}")
        except Exception as e:
            print(f"Path planning error: {e}")
        stop_event.wait(config.PATH_UPDATE_INTERVAL)

def execution_loop(stop_event: threading.Event, run_dir: Path):
    while not stop_event.is_set():
        try:
            follow_path(run_dir)
            stop_event.wait(10)
        except Exception as e:
            print(f"Path execution error: {e}")

if __name__ == "__main__":
    # setup base directory (root of the project)
    base_dir = Path(__file__).parent.parent
    
    # create unique run directory
    run_dir = base_dir / "control/runs" / datetime.now().strftime("%Y-%m-%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    # setup essential files for this run
    points_file = run_dir / 'points.csv'
    points_file.touch()
    (run_dir / 'path.csv').touch()

    # seed data with initial obstacles and destination
    data = {
        'id': [0, 1, 2, 0, 0, 1, 2, 3, 4],
        'category': ['gps', 'gps', 'gps', 'destination', 'camera', 'camera', 'camera', 'camera', 'camera'],
        'latitude': [51.011466, 51.011401, 51.011315, 51.011504, 51.011453, 51.011412, 51.011340, 51.011485, 51.011394],
        'longitude': [3.708731, 3.709055, 3.709353, 3.708728, 3.708779, 3.708822, 3.708969, 3.708916, 3.709079]
    }
    pd.DataFrame(data).to_csv(points_file, index=False)

    stop_event = threading.Event()

    # start background threads for logic loops
    threading.Thread(target=collection_loop, args=(stop_event, run_dir), daemon=True).start()
    threading.Thread(target=planning_loop, args=(stop_event, run_dir), daemon=True).start()
    threading.Thread(target=execution_loop, args=(stop_event, run_dir), daemon=True).start()

    # simple interactive CLI
    map_process = None
    try:
        while True:
            resp = input("type 'map' to launch map or 'exit' to exit\n").strip().lower()
            if resp == "exit":
                break
            elif resp == "map":
                if map_process is None or map_process.poll() is not None:
                    # pathing for Linux environment
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
    stop_event.set() # stop the background threads

    if map_process is not None:
        print("Terminating map process...")
        map_process.terminate() # gracefully stop the Dash server
        try:
            map_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print("Map process not responding, force killing...")
            map_process.kill() # force kill if it hangs

    print("Cleanup complete. Program exited.")