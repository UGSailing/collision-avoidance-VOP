"""
    AI generated simulator that listens for rudder/thrust commands on a socket and updates a GPS position in points.csv accordingly.
    This allows testing the path planning and execution without real GPS hardware, e.g. in an indoor setting.
"""

from __future__ import annotations
from pathlib import Path
from urllib.parse import urlparse
import argparse
import math
import socket
import sys
import threading
import time
import pandas as pd

try:
    import config
except ModuleNotFoundError:
    sys.path.append(str(Path(__file__).resolve().parents[1]))
    import config
POINT_COLUMNS = ["id", "category", "latitude", "longitude", "heading"]

# NOTE: Enable fanout to ESP32 serial port(s) in config.ESP_SERIAL_PORTS to feed mock GPS data into the real control loop.

class CommandState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._rudder_deg = 0.0
        self._thrust = 0.0
        self._updated_at = time.monotonic()

    def set(self, rudder_deg: float, thrust: float) -> None:
        with self._lock:
            self._rudder_deg = rudder_deg
            self._thrust = thrust
            self._updated_at = time.monotonic()

    def get(self) -> tuple[float, float, float]:
        with self._lock:
            return self._rudder_deg, self._thrust, self._updated_at


def _normalize_heading(heading_deg: float) -> float:
    return heading_deg % 360.0


def _as_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _safe_points_df(points_path: Path) -> pd.DataFrame:
    if not points_path.exists() or points_path.stat().st_size == 0:
        return pd.DataFrame(columns=POINT_COLUMNS)

    try:
        df = pd.read_csv(points_path)
    except Exception:
        return pd.DataFrame(columns=POINT_COLUMNS)

    for column in POINT_COLUMNS:
        if column not in df.columns:
            df[column] = pd.Series(dtype=float)
    return df


def _initial_pose(run_dir: Path) -> tuple[float, float, float]:
    points_path = run_dir / "points.csv"
    path_path = run_dir / "path.csv"

    df = _safe_points_df(points_path)
    gps_points = df[df["category"] == "gps"] if not df.empty else pd.DataFrame()
    if not gps_points.empty:
        latest = gps_points.loc[gps_points["id"].idxmax()]
        lat = _as_float(latest.get("latitude"))
        lon = _as_float(latest.get("longitude"))
        heading = _as_float(latest.get("heading"))
        if lat is not None and lon is not None and heading is not None:
            return lat, lon, heading

    if path_path.exists() and path_path.stat().st_size > 0:
        try:
            path_df = pd.read_csv(path_path)
            if len(path_df) > 0:
                first = path_df.iloc[0]
                return float(first["latitude"]), float(first["longitude"]), 0.0
        except Exception:
            pass

    return 51.011466, 3.708731, 0.0


def _append_gps(points_path: Path, lat: float, lon: float, heading: float) -> None:
    df = _safe_points_df(points_path)
    if df.empty:
        next_id = 0
    else:
        numeric_ids = pd.to_numeric(df["id"], errors="coerce")
        max_id = numeric_ids.max()
        next_id = 0 if pd.isna(max_id) else int(max_id) + 1

    row = pd.DataFrame(
        [
            {
                "id": next_id,
                "category": "gps",
                "latitude": lat,
                "longitude": lon,
                "heading": heading,
            }
        ]
    )
    out = pd.concat([df, row], ignore_index=True)
    out.to_csv(points_path, index=False)


def _parse_command_line(line: str) -> tuple[float, float] | None:
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 2:
        return None

    try:
        rudder_deg = float(parts[0])
        thrust = float(parts[1])
    except ValueError:
        return None

    rudder_deg = max(-float(config.MAX_RUDDER_ANGLE_DEG), min(float(config.MAX_RUDDER_ANGLE_DEG), rudder_deg))
    thrust = max(-1.0, min(1.0, thrust))
    return rudder_deg, thrust


def _command_listener(state: CommandState, host: str, port: int) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host, port))
        server.listen(1)
        print(f"Simulator listening for autopilot commands on {host}:{port}")

        while True:
            conn, addr = server.accept()
            print(f"Autopilot connected from {addr[0]}:{addr[1]}")
            with conn:
                conn.settimeout(1.0)
                buffer = ""
                while True:
                    try:
                        chunk = conn.recv(1024)
                    except socket.timeout:
                        continue
                    except OSError:
                        break

                    if not chunk:
                        print("Autopilot disconnected")
                        break

                    buffer += chunk.decode("ascii", errors="ignore")
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        parsed = _parse_command_line(line.strip())
                        if parsed is None:
                            continue
                        state.set(*parsed)


def _resolve_socket_endpoint(cli_listen: str | None) -> tuple[str, int]:
    if cli_listen:
        host, port_text = cli_listen.rsplit(":", 1)
        return host, int(port_text)

    endpoints = getattr(config, "ESP_SERIAL_PORTS", None)
    if isinstance(endpoints, list):
        for endpoint in endpoints:
            endpoint_s = str(endpoint)
            if endpoint_s.startswith("socket://"):
                parsed = urlparse(endpoint_s)
                host = parsed.hostname or "127.0.0.1"
                if parsed.port is None:
                    raise ValueError("socket:// endpoint in ESP_SERIAL_PORTS has no port")
                return host, parsed.port

    if config.ESP_SERIAL_PORT.startswith("socket://"):
        parsed = urlparse(config.ESP_SERIAL_PORT)
        host = parsed.hostname or "127.0.0.1"
        if parsed.port is None:
            raise ValueError("config.ESP_SERIAL_PORT uses socket:// but no port is defined")
        return host, parsed.port

    return "127.0.0.1", 8765


def _resolve_run_dir(run_dir_str: str) -> Path:
    candidate = Path(run_dir_str)
    if candidate.is_absolute() or candidate.exists():
        return candidate

    control_root = Path(__file__).resolve().parents[1]
    fallback = control_root / candidate
    if fallback.exists():
        return fallback

    return candidate


def run_command_driven_mock(run_dir_str: str, listen: str | None, update_hz: float) -> None:
    run_dir = _resolve_run_dir(run_dir_str)
    points_path = run_dir / "points.csv"

    print(f"Starting mock GPS for run directory: {run_dir}")
    print(f"Waiting for points file at: {points_path}")
    while not points_path.exists():
        time.sleep(0.2)
    print("Found points.csv, starting command-driven simulation loop")

    lat, lon, heading_deg = _initial_pose(run_dir)
    print(f"Spawned simulator at lat={lat:.6f}, lon={lon:.6f}, heading={heading_deg:.1f} deg")

    host, port = _resolve_socket_endpoint(listen)
    command_state = CommandState()
    listener = threading.Thread(
        target=_command_listener,
        args=(command_state, host, port),
        daemon=True,
    )
    listener.start()

    dt = 1.0 / max(0.1, update_hz)
    command_timeout_s = 2.0

    # Tunable but intentionally conservative indoor dynamics.
    max_speed_mps = 2.4
    max_turn_rate_deg_s = 35.0

    while True:
        rudder_deg, thrust, updated_at = command_state.get()
        if time.monotonic() - updated_at > command_timeout_s:
            thrust = 0.0
            rudder_deg = 0.0

        rudder_fraction = rudder_deg / float(config.MAX_RUDDER_ANGLE_DEG)
        # PathFollower already applies STEERING_DIRECTION when creating rudder commands.
        # Undo that mapping here so simulator yaw follows geometric heading convention.
        turn_fraction = rudder_fraction * float(config.STEERING_DIRECTION)
        speed_mps = thrust * max_speed_mps
        speed_factor = min(1.0, 0.2 + abs(speed_mps) / max_speed_mps)
        heading_deg = _normalize_heading(heading_deg + turn_fraction * max_turn_rate_deg_s * speed_factor * dt)

        distance_m = speed_mps * dt
        heading_rad = math.radians(heading_deg)
        north_m = math.cos(heading_rad) * distance_m
        east_m = math.sin(heading_rad) * distance_m

        lat += north_m / float(config.METERS_PER_DEGREE_LAT)
        meters_per_degree_lon = float(config.METERS_PER_DEGREE_LAT) * max(0.1, math.cos(math.radians(lat)))
        lon += east_m / meters_per_degree_lon

        _append_gps(points_path, lat, lon, heading_deg)
        print(
            f"Cmd rudder={rudder_deg:6.2f} deg thrust={thrust:5.2f} | "
            f"Pose lat={lat:.6f} lon={lon:.6f} heading={heading_deg:6.2f}"
        )
        time.sleep(dt)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Indoor GPS simulator driven by path_execution rudder/thrust commands")
    parser.add_argument("run_dir", help="Run directory, e.g. runs/YYYY-MM-DD_HHMMSS")
    parser.add_argument(
        "--listen",
        default=None,
        help="Command listener endpoint as host:port. Default: from config.ESP_SERIAL_PORT if socket:// else 127.0.0.1:8765",
    )
    parser.add_argument(
        "--update-hz",
        type=float,
        default=float(config.GPS_UPDATE_RATE_HZ),
        help="Simulation update rate in Hz",
    )
    return parser


if __name__ == "__main__":
    args = _build_parser().parse_args()
    try:
        run_command_driven_mock(args.run_dir, args.listen, args.update_hz)
    except KeyboardInterrupt:
        print("Simulator stopped")
    except Exception as exc:
        print(f"Simulator failed to start: {exc}")
        raise