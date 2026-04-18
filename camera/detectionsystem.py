#!/usr/bin/env python3
"""Simple single-camera duck detection system for Hailo.

What this script does:
1. Runs Hailo object detection with duck.hef on one camera.
2. Rectifies each frame with calibration_yamls/camera_calibration.yaml.
3. Computes distance and angle for duck detections.
4. Sends angle,distance payloads over TCP.
5. Stores rectified recordings per minute in control/boatcam_recordings.
"""

# To run: 
#python detectionsystem.py --input rpi --object-height 0.23

from __future__ import annotations

import argparse
import atexit
import importlib
import socket
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from calibrations.rectification import build_undistort_maps, load_calib_yaml
from config import (
    OBSTACLE_OUTPUT_DEBUG,
    OBSTACLE_PAIR_SEPARATOR,
    OBSTACLE_TCP_BACKLOG,
    OBSTACLE_TCP_BIND_HOST,
    OBSTACLE_TCP_ENABLED,
    OBSTACLE_TCP_MAX_CLIENTS,
    OBSTACLE_TCP_PORT,
    OBSTACLE_TCP_SEND_TIMEOUT_S,
    OBSTACLE_TCP_SEND_EMPTY_UPDATES,
    OBSTACLE_VALUE_SEPARATOR,
)
from depth_calculation.single_camera_depth_calculation import (
    distance_and_angle_from_bbox,
)


SCRIPT_PATH = Path(__file__).resolve()
CAMERA_DIR = SCRIPT_PATH.parent
PROJECT_ROOT = CAMERA_DIR.parent
CONTROL_RECORDINGS_DIR = PROJECT_ROOT / "camera" / "boatcam_recordings"

DEFAULT_HAILO_APPS_ROOT = Path.home() / "Documents" / "hailo-apps"
DEFAULT_NETWORK = CAMERA_DIR / "yolo_models" / "duck.hef"
DEFAULT_CALIB = CAMERA_DIR / "calibration_yamls" / "camera_calibration.yaml"
TARGET_LABEL = "duck"


class ObstacleTcpServer:
    """Small non-blocking TCP server that streams one payload line per update."""

    def __init__(
        self,
        host: str,
        port: int,
        backlog: int,
        max_clients: int,
        send_timeout_s: float,
        debug: bool,
    ):
        self.host = host
        self.port = int(port)
        self.backlog = max(1, int(backlog))
        self.max_clients = max(1, int(max_clients))
        self.send_timeout_s = max(0.01, float(send_timeout_s))
        self.debug = bool(debug)
        self._server: socket.socket | None = None
        self._clients: list[socket.socket] = []

    def start(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(self.backlog)
        server.setblocking(False)
        self._server = server
        print(f"[TCP] listening on {self.host}:{self.port}")

    def _accept_new_clients(self) -> None:
        if self._server is None:
            return
        while True:
            try:
                client, addr = self._server.accept()
            except BlockingIOError:
                break
            except OSError:
                break

            client.settimeout(self.send_timeout_s)
            self._clients.append(client)
            if self.debug:
                print(f"[TCP] client connected: {addr[0]}:{addr[1]}")

            if len(self._clients) > self.max_clients:
                old_client = self._clients.pop(0)
                try:
                    old_client.close()
                except OSError:
                    pass

    def send_line(self, line: str) -> None:
        self._accept_new_clients()
        if not self._clients:
            return

        payload = (line + "\n").encode("ascii", errors="ignore")
        alive_clients: list[socket.socket] = []
        for client in self._clients:
            try:
                client.sendall(payload)
                alive_clients.append(client)
            except OSError:
                try:
                    client.close()
                except OSError:
                    pass
        self._clients = alive_clients

    def close(self) -> None:
        for client in self._clients:
            try:
                client.close()
            except OSError:
                pass
        self._clients.clear()

        if self._server is not None:
            try:
                self._server.close()
            except OSError:
                pass
            self._server = None


class MinuteVideoRecorder:
    """Writes one MP4 file per minute into control/boatcam_recordings."""

    def __init__(self, base_dir: Path, fps: float):
        self.base_dir = base_dir
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.fps = float(fps)
        self._writer: cv2.VideoWriter | None = None
        self._minute_key: str | None = None
        self._frame_size: tuple[int, int] | None = None

    def _rotate_writer_if_needed(
        self, now: datetime, frame_size: tuple[int, int]
    ) -> None:
        minute_key = now.strftime("%Y%m%d_%H%M")
        if (
            self._writer is not None
            and self._minute_key == minute_key
            and self._frame_size == frame_size
        ):
            return

        self.close()
        day_dir = self.base_dir / now.strftime("%Y%m%d")
        day_dir.mkdir(parents=True, exist_ok=True)
        out_path = day_dir / f"boatcam_{minute_key}.mp4"

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(out_path), fourcc, self.fps, frame_size)
        if not writer.isOpened():
            raise RuntimeError(f"Could not open video writer: {out_path}")

        self._writer = writer
        self._minute_key = minute_key
        self._frame_size = frame_size

    def write(self, frame: np.ndarray) -> None:
        frame_h, frame_w = frame.shape[:2]
        frame_size = (frame_w, frame_h)
        self._rotate_writer_if_needed(datetime.now(), frame_size)
        assert self._writer is not None
        self._writer.write(frame)

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        self._minute_key = None
        self._frame_size = None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Single-camera duck detection with rectification, depth/angle, TCP and minute recordings."
    )
    parser.add_argument(
        "--input",
        default="rpi",
        help="Input source (rpi, usb, /dev/video0, file path, ...).",
    )
    parser.add_argument(
        "--hailo-apps-root",
        type=Path,
        default=DEFAULT_HAILO_APPS_ROOT,
        help=f"Path to hailo-apps root (default: {DEFAULT_HAILO_APPS_ROOT})",
    )
    parser.add_argument(
        "--network",
        type=Path,
        default=DEFAULT_NETWORK,
        help=f"Path to duck.hef (default: {DEFAULT_NETWORK})",
    )
    parser.add_argument(
        "--calib",
        type=Path,
        default=DEFAULT_CALIB,
        help=f"Calibration YAML path (default: {DEFAULT_CALIB})",
    )
    parser.add_argument(
        "--object-height",
        type=float,
        default=0.23,
        help="Duck height in meters for mono distance estimate (default: 0.23).",
    )
    parser.add_argument(
        "--recordings-dir",
        type=Path,
        default=CONTROL_RECORDINGS_DIR,
        help=f"Per-minute recording folder (default: {CONTROL_RECORDINGS_DIR})",
    )
    parser.add_argument(
        "--recording-fps",
        type=float,
        default=20.0,
        help="FPS for output recording files (default: 20).",
    )
    return parser


def resolve_hailo_paths(hailo_apps_root: Path) -> tuple[Path, Path]:
    root = hailo_apps_root.expanduser().resolve()
    object_detection_dir = (
        root / "hailo_apps" / "python" / "standalone_apps" / "object_detection"
    )
    object_detection_script = object_detection_dir / "object_detection.py"
    return object_detection_dir, object_detection_script


def validate_paths(
    hailo_apps_root: Path,
    object_detection_dir: Path,
    object_detection_script: Path,
    network_path: Path,
    calib_path: Path,
) -> None:
    if not hailo_apps_root.exists() or not hailo_apps_root.is_dir():
        raise FileNotFoundError(f"Invalid hailo-apps root: {hailo_apps_root}")
    if not object_detection_dir.exists():
        raise FileNotFoundError(f"Missing object_detection dir: {object_detection_dir}")
    if not object_detection_script.exists():
        raise FileNotFoundError(
            f"Missing object_detection.py: {object_detection_script}"
        )
    if not network_path.exists():
        raise FileNotFoundError(f"Missing network file: {network_path}")
    if not calib_path.exists():
        raise FileNotFoundError(f"Missing calibration YAML: {calib_path}")


def scale_camera_matrix(
    camera_matrix: np.ndarray,
    from_size: tuple[int, int],
    to_size: tuple[int, int],
) -> np.ndarray:
    from_w, from_h = from_size
    to_w, to_h = to_size
    if (from_w, from_h) == (to_w, to_h):
        return camera_matrix.copy()
    scaled = camera_matrix.copy()
    sx = to_w / from_w
    sy = to_h / from_h
    scaled[0, 0] *= sx
    scaled[0, 2] *= sx
    scaled[1, 1] *= sy
    scaled[1, 2] *= sy
    return scaled


def draw_overlay(
    frame: np.ndarray,
    boxes: list[tuple[float, float, float, float]],
    values: list[tuple[float, float]],
) -> None:
    for (x1, y1, _x2, _y2), (distance_m, angle_deg) in zip(boxes, values):
        text = f"duck | {distance_m:.2f}m | {angle_deg:.2f}deg"
        cv2.putText(
            frame,
            text,
            (int(x1), max(int(y1) - 12, 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )


def install_processing_pipeline(
    hailo_root: Path,
    calib_path: Path,
    object_height_m: float,
    recordings_dir: Path,
    recording_fps: float,
) -> None:
    """Monkey-patch Hailo post-processing to inject our custom logic."""

    sys.path.insert(0, str(hailo_root))
    post_process_module = importlib.import_module(
        "hailo_apps.python.standalone_apps.object_detection.object_detection_post_process"
    )
    original_handler = post_process_module.inference_result_handler
    extract_detections = post_process_module.extract_detections

    calib_size, camera_matrix, distortion = load_calib_yaml(str(calib_path))

    map_cache: dict[str, object] = {
        "frame_size": None,
        "newK": None,
        "map1": None,
        "map2": None,
    }

    tcp_server = None
    if OBSTACLE_TCP_ENABLED:
        tcp_server = ObstacleTcpServer(
            host=OBSTACLE_TCP_BIND_HOST,
            port=OBSTACLE_TCP_PORT,
            backlog=OBSTACLE_TCP_BACKLOG,
            max_clients=OBSTACLE_TCP_MAX_CLIENTS,
            send_timeout_s=OBSTACLE_TCP_SEND_TIMEOUT_S,
            debug=OBSTACLE_OUTPUT_DEBUG,
        )
        tcp_server.start()

    recorder = MinuteVideoRecorder(recordings_dir, fps=recording_fps)

    def close_outputs() -> None:
        recorder.close()
        if tcp_server is not None:
            tcp_server.close()

    atexit.register(close_outputs)

    def patched_handler(
        original_frame,
        infer_results,
        labels,
        config_data,
        tracker=None,
        draw_trail=False,
    ):
        frame_h, frame_w = original_frame.shape[:2]
        frame_size = (frame_w, frame_h)

        # Build undistortion maps once per frame size.
        if map_cache["frame_size"] != frame_size:
            scaled_K = scale_camera_matrix(camera_matrix, calib_size, frame_size)
            newK, _roi, map1, map2 = build_undistort_maps(
                frame_size, scaled_K, distortion, alpha=0.0
            )
            map_cache["frame_size"] = frame_size
            map_cache["newK"] = newK
            map_cache["map1"] = map1
            map_cache["map2"] = map2

        newK = map_cache["newK"]
        map1 = map_cache["map1"]
        map2 = map_cache["map2"]
        assert isinstance(newK, np.ndarray)
        assert isinstance(map1, np.ndarray)
        assert isinstance(map2, np.ndarray)

        # Keep Hailo's own boxes/labels rendering.
        frame_with_detections = original_handler(
            original_frame,
            infer_results,
            labels,
            config_data,
            tracker=tracker,
            draw_trail=draw_trail,
        )

        rectified = cv2.remap(
            frame_with_detections,
            map1,
            map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

        detections = extract_detections(original_frame, infer_results, config_data)
        classes = detections["detection_classes"]
        boxes = detections["detection_boxes"]

        duck_boxes: list[tuple[float, float, float, float]] = []
        distance_angle: list[tuple[float, float]] = []

        fx = float(newK[0, 0])
        fy = float(newK[1, 1])
        cx = float(newK[0, 2])
        cy = float(newK[1, 2])

        for class_id, box in zip(classes, boxes):
            label = str(labels[class_id]).lower()
            if label != TARGET_LABEL:
                continue

            bbox = (float(box[0]), float(box[1]), float(box[2]), float(box[3]))
            try:
                distance_m, angle_deg = distance_and_angle_from_bbox(
                    bbox=bbox,
                    object_height_m=object_height_m,
                    fx=fx,
                    fy=fy,
                    cx=cx,
                    cy=cy,
                )
            except ValueError:
                continue

            duck_boxes.append(bbox)
            distance_angle.append((float(distance_m), float(angle_deg)))

        if distance_angle:
            payload = OBSTACLE_PAIR_SEPARATOR.join(
                f"{angle_deg:.2f}{OBSTACLE_VALUE_SEPARATOR}{distance_m:.2f}"
                for distance_m, angle_deg in distance_angle
            )
        elif OBSTACLE_TCP_SEND_EMPTY_UPDATES:
            payload = ""
        else:
            payload = None

        if tcp_server is not None and payload is not None:
            tcp_server.send_line(payload)

        if OBSTACLE_OUTPUT_DEBUG and payload is not None:
            print(f"[TCP] payload: {payload}")

        if duck_boxes:
            draw_overlay(rectified, duck_boxes, distance_angle)

        recorder.write(rectified)
        return rectified

    setattr(post_process_module, "inference_result_handler", patched_handler)


def run() -> int:
    args = build_parser().parse_args()

    hailo_root = args.hailo_apps_root.expanduser().resolve()
    network_path = args.network.expanduser().resolve()
    calib_path = args.calib.expanduser().resolve()
    recordings_dir = args.recordings_dir.expanduser().resolve()

    object_detection_dir, object_detection_script = resolve_hailo_paths(hailo_root)
    validate_paths(
        hailo_apps_root=hailo_root,
        object_detection_dir=object_detection_dir,
        object_detection_script=object_detection_script,
        network_path=network_path,
        calib_path=calib_path,
    )

    install_processing_pipeline(
        hailo_root=hailo_root,
        calib_path=calib_path,
        object_height_m=float(args.object_height),
        recordings_dir=recordings_dir,
        recording_fps=float(args.recording_fps),
    )

    # We run Hailo's app with HD camera mode as requested (1280x720).
    sys.argv = [
        str(object_detection_script),
        "-n",
        str(network_path),
        "-i",
        args.input,
        "--camera-resolution",
        "hd",
    ]

    print(f"[INFO] Hailo root: {hailo_root}")
    print(f"[INFO] Network: {network_path}")
    print(f"[INFO] Calibration: {calib_path}")
    print(f"[INFO] Recordings: {recordings_dir}")
    print(f"[INFO] Input: {args.input}")

    os_module = importlib.import_module(
        "hailo_apps.python.standalone_apps.object_detection.object_detection"
    )
    os_module.main()
    return 0


if __name__ == "__main__":
    sys.exit(run())
