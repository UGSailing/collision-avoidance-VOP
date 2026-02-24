from datetime import datetime
from pathlib import Path
import pandas as pd
import threading
import subprocess
from data_collection import read_CAN
from path_planning import update_path
from path_execution import follow_path

PATH_UPDATE_INTERVAL = 3  # seconds between path recalculations

def collection_loop(stop_event):
    while not stop_event.is_set():
        try:
            read_CAN(run_dir)
            stop_event.wait(10)
        except Exception as e:
            print(f"Data collection error: {e}")

def planning_loop(stop_event):
    while not stop_event.is_set():
        try:
            update_path(run_dir)
        except Exception as e:
            print(f"Path planning error: {e}")
        stop_event.wait(PATH_UPDATE_INTERVAL)


def execution_loop(stop_event):
    while not stop_event.is_set():
        try:
            follow_path(run_dir)
            stop_event.wait(10)
        except Exception as e:
            print(f"Path execution error: {e}")

if __name__ == "__main__":
    run_dir = Path("control/runs") / datetime.now().strftime("%Y-%m-%d_%H%M%S")
    run_dir.mkdir(parents=True)

    # create files for new run
    (run_dir / 'points.csv').touch()
    (run_dir / 'path.csv').touch()

    # TODO remove
    # start data
    data = {
        'id': [0, 1, 2, 0],
        'category': ['gps', 'gps', 'gps', 'destination'],
        'latitude': [51.011466, 51.011401, 51.011315, 51.011504],
        'longitude': [3.708731, 3.709055, 3.709353, 3.708728]
    }
    pd.DataFrame(data).to_csv(run_dir / 'points.csv', index=False)

    stop_event = threading.Event()

    # start threads
    collection_thread = threading.Thread(target=collection_loop, args=(stop_event,), daemon=True)
    collection_thread.start()
    path_thread = threading.Thread(target=planning_loop, args=(stop_event,), daemon=True)
    path_thread.start()
    execution_thread = threading.Thread(target=execution_loop, args=(stop_event,), daemon=True)
    execution_thread.start()

    # launch simple interactive CLI
    resp = input("type map to launch map or exit to exit\n")
    map_process = None
    while resp != "exit":
        if resp == "map":
            if map_process is None or map_process.poll() is not None:
                control_dir = Path(__file__).parent
                python = control_dir / '.venv/Scripts/python.exe'
                script = control_dir / 'visualisation/live_map.py'
                map_process = subprocess.Popen([str(python), str(script), str(run_dir)])
            else:
                print("map is already running")
        # elif ...
        else:
            print("invalid input")
        resp = input("type map to launch map or exit to exit\n")

    stop_event.set()
