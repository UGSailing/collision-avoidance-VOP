"""
boat_main.py - Autonomous entry point for on boat deployment.

Differences from main.py (debug mode):
  - No interactive CLI or map launcher.
  - On boot, blocks until a GO signal (CAN ID config.CAN_GO_ID) is received.
  - While a run is active, monitors the CAN bus for a STOP signal
    (CAN ID config.CAN_STOP_ID); on receipt, tears down the run and loops
    back to waiting for the next GO.
  - Ctrl-C exits the process entirely.
"""

import asyncio
import can
import threading
import pandas as pd
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
            await asyncio.get_running_loop().run_in_executor(None, stop_event.wait)
            async_stop.set()

        asyncio.create_task(_bridge())
        collector = DataCollector(run_dir)
        await collector.run(async_stop)

    asyncio.run(_run())


def planning_loop(stop_event: threading.Event, run_dir: Path):
    mapper = OccupancyMapper(
        resolution=config.GRID_RESOLUTION,
        grid_size=config.GRID_SIZE,
        hitbox_radius=config.HITBOX_RADIUS,
    )
    grid = mapper.create_grid(run_dir / "points.csv")

    while not stop_event.is_set():
        try:
            grid = mapper.update_grid(run_dir / "points.csv")
            update_path(run_dir, grid, mapper)
            print(f"Path updated. Obstacles detected: {grid.sum() > 0}")
        except Exception as e:
            print(f"Path planning error: {e}")
        stop_event.wait(config.PATH_UPDATE_INTERVAL)


def execution_loop(stop_event: threading.Event, run_dir: Path):
    path_follower = PathFollower(run_dir)
    while not stop_event.is_set():
        try:
            path_follower.follow_path(run_dir)
            stop_event.wait(10)
        except Exception as e:
            print(f"Path execution error: {e}")


def _open_can_bus():
    return can.interface.Bus(
        channel=config.CAN_CHANNEL,
        bustype=config.CAN_BUSTYPE,
        bitrate=config.CAN_BITRATE,
    )


def wait_for_go_signal():
    """Block until a GO signal (config.CAN_GO_ID) is received on the CAN bus."""
    print("Waiting for GO signal on CAN bus…")
    while True:
        try:
            bus = _open_can_bus()
            try:
                while True:
                    msg = bus.recv(timeout=1.0)
                    if msg is not None and msg.arbitration_id == config.CAN_GO_ID:
                        print("GO signal received - starting run.")
                        return
            finally:
                bus.shutdown()
        except (can.CanError, OSError) as e:
            print(f"CAN error while waiting for GO: {e} - retrying in 5 s")
            threading.Event().wait(5)


def can_monitor_loop(stop_event: threading.Event, shutdown_event: threading.Event):
    """Watch the CAN bus for a STOP signal while a run is active."""
    try:
        bus = _open_can_bus()
        try:
            while not stop_event.is_set():
                msg = bus.recv(timeout=0.5)
                if msg is not None and msg.arbitration_id == config.CAN_STOP_ID:
                    print("STOP signal received - shutting down run.")
                    shutdown_event.set()
                    return
        finally:
            bus.shutdown()
    except (can.CanError, OSError) as e:
        print(f"CAN monitor error: {e} - triggering shutdown.")
        shutdown_event.set()


# ── run lifecycle ─────────────────────────────────────────────────────────────

def _init_run_dir(base_dir: Path) -> Path:
    run_dir = base_dir / "control/runs" / datetime.now().strftime("%Y-%m-%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    points_file = run_dir / "points.csv"
    points_file.touch()
    pd.DataFrame(columns=["id", "category", "latitude", "longitude", "heading"]).to_csv(
        points_file, index=False
    )

    path_file = run_dir / "path.csv"
    path_file.touch()
    pd.DataFrame(columns=["id", "latitude", "longitude"]).to_csv(path_file, index=False)

    return run_dir


def run_once(base_dir: Path):
    """Start all worker threads for a single run and block until a STOP arrives."""
    run_dir = _init_run_dir(base_dir)
    print(f"Run directory: {run_dir}")

    stop_event = threading.Event()
    shutdown_event = threading.Event()

    threads = [
        threading.Thread(target=collection_loop, args=(stop_event, run_dir), daemon=True),
        threading.Thread(target=planning_loop,   args=(stop_event, run_dir), daemon=True),
        threading.Thread(target=execution_loop,  args=(stop_event, run_dir), daemon=True),
        threading.Thread(target=can_monitor_loop, args=(stop_event, shutdown_event), daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        shutdown_event.wait()  # blocks until STOP signal or KeyboardInterrupt
    except KeyboardInterrupt:
        raise  # propagate so the outer loop can exit cleanly
    finally:
        stop_event.set()
        print("Run stopped.")

if __name__ == "__main__":
    base_dir = Path(__file__).parent.parent
    print("boat_main.py started - autonomous mode.")

    try:
        while True:
            wait_for_go_signal()
            try:
                run_once(base_dir)
            except KeyboardInterrupt:
                break
            print("Run finished. Ready for next GO signal.\n")
    except KeyboardInterrupt:
        pass

    print("Exiting boat_main.py.")
