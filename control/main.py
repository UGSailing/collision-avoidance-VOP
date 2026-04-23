import asyncio
import pandas as pd
import threading
import subprocess
import sys
import time
from pathlib import Path
from datetime import datetime
import config
from data_collection import DataCollector
from path_planning import update_path, OccupancyMapper
from path_execution import PathFollower


def collection_loop(stop_event: threading.Event, run_dir: Path):
    async def _run():
        async_stop = asyncio.Event()

        async def _bridge():
            # Wait for the threading stop_event in a thread-pool so the event loop stays free
            await asyncio.get_running_loop().run_in_executor(None, stop_event.wait)
            async_stop.set()

        asyncio.create_task(_bridge())
        collector = DataCollector(run_dir)
        await collector.run(async_stop)

    asyncio.run(_run())

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
    path_follower = PathFollower(run_dir)
    next_tick = time.monotonic()  # absolute schedule anchor

    while not stop_event.is_set():
        next_tick += 1 / config.PATH_EXECUTION_FREQUENCY_HZ
        try:
            path_follower.follow_path(run_dir)
        except Exception as e:
            print(f"Path execution error: {e}")

        delay = next_tick - time.monotonic()

        if delay > 0:
            stop_event.wait(delay)
        else:
            next_tick = time.monotonic()  # we're late, skip to now to avoid spiraling
        

if __name__ == "__main__":
    # directories
    base_dir = Path(__file__).parent.parent
    run_dir = base_dir / "control/runs" / datetime.now().strftime("%Y-%m-%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    # files
    points_file = run_dir / 'points.csv'
    points_file.touch()
    pd.DataFrame(columns=['id', 'category', 'latitude', 'longitude', 'heading']).to_csv(points_file, index=False)

    path_file = run_dir / 'path.csv'
    path_file.touch()
    pd.DataFrame(columns=['id', 'latitude', 'longitude']).to_csv(path_file, index=False)

    # seed data for testing
    # data = {
    #     'id': [0, 1, 2, 0, 0, 1, 2, 3, 4],
    #     'category': ['gps', 'gps', 'gps', 'destination', 'camera', 'camera', 'camera', 'camera', 'camera'],
    #     'latitude': [51.011466, 51.011401, 51.011335, 51.011504, 51.011453, 51.011412, 51.011340, 51.011485, 51.011394],
    #     'longitude': [3.708731, 3.709055, 3.709215, 3.708728, 3.708779, 3.708822, 3.708969, 3.708916, 3.709079],
    #     'heading': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # }
    # pd.DataFrame(data).to_csv(points_file, index=False)

    # threads
    stop_event = threading.Event()
    threading.Thread(target=collection_loop, args=(stop_event, run_dir), daemon=True).start()
    threading.Thread(target=planning_loop, args=(stop_event, run_dir), daemon=True).start()
    threading.Thread(target=execution_loop, args=(stop_event, run_dir), daemon=True).start()

    # interactive CLI
    map_process = None
    try:
        # Check if we are in a real terminal
        if sys.stdin.isatty():
            while True:
                resp = input("Type 'map', 'exit', or 'destination <lat> <lon>'\n").strip().lower()
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
                elif resp.startswith("destination"):
                    try:
                        # Split the string and extract the numbers
                        parts = resp.split()
                        new_lat = float(parts[1])
                        new_lon = float(parts[2])

                        # Read current points, update the destination row, and save back
                        df = pd.read_csv(points_file)
                        
                        if 'destination' in df['category'].values:
                            df.loc[df['category'] == 'destination', ['latitude', 'longitude']] = [new_lat, new_lon]
                        else:
                            # Fallback if destination row didn't exist yet
                            new_row = pd.DataFrame([{
                                'id': 0, 'category': 'destination', 
                                'latitude': new_lat, 'longitude': new_lon, 'heading': 0.0
                            }])
                            df = pd.concat([df, new_row], ignore_index=True)
                        
                        df.to_csv(points_file, index=False)
                        print(f"Destination updated to: {new_lat}, {new_lon}")
                        
                    except (IndexError, ValueError):
                        print("Invalid format. Use: destination <lat> <lon>")
                    except Exception as e:
                        print(f"Error updating destination: {e}")
                # -----------------------------
                else:
                    print("Invalid input.")
        else:
            # Headless mode detected (no terminal)
            # Interactive CLI disabled 
            # Using default destination and running indefinitely
            df = pd.read_csv(points_file)
            new_row = pd.DataFrame([{
                'id': 0, 'category': 'destination', 
                'latitude': config.DEFAULT_DESTINATION[0], 'longitude': config.DEFAULT_DESTINATION[1], 'heading': 0.0
            }])
            df = pd.concat([df, new_row], ignore_index=True)
            df.to_csv(points_file, index=False)
            while True:
                time.sleep(1) # Keeps the main thread alive so background threads can run
            
    except KeyboardInterrupt:
        print("\nShutdown signal received.")

    # cleanup
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