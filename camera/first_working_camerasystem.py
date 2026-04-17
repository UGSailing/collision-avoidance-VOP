#!/usr/bin/env python3
# VERGEET NIET DE source setup... vanuit hailo als venv te draaien!!!!!
# Linux examples:
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection3_duck.py --input rpi --object-height 0.21367
#
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection3_duck.py --input rpi --object-height 0.23 --save-output
#
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection3_duck.py --input /dev/video0 --object-height 0.23


# python3 run_hailo_detection3_duck.py --input /home/mario/Documents/collision-avoidance-VOP/camera/recordings/Ball_duck_water/duck_in_water.mov --object-height 0.213267
"""Run rectified Hailo duck detection with mono distance and azimuth logging."""

import argparse
import atexit
import csv
import importlib
import json
import math
import os
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import yaml

from config import (
    OBSTACLE_OUTPUT_DEBUG,
    OBSTACLE_PAIR_SEPARATOR,
    OBSTACLE_TCP_BACKLOG,
    OBSTACLE_TCP_BIND_HOST,
    OBSTACLE_TCP_ENABLED,
    OBSTACLE_TCP_MAX_CLIENTS,
    OBSTACLE_TCP_PORT,
    OBSTACLE_TCP_SEND_TIMEOUT_S,
    OBSTACLE_VALUE_SEPARATOR,
    USB_DEVICE,
)


SCRIPT_PATH = Path(__file__).resolve()
CAMERA_DIR = SCRIPT_PATH.parent
DEFAULT_HAILO_APPS_ROOT = Path.home() / "Documents" / "hailo-apps"
DEFAULT_DUCK_HEF = CAMERA_DIR / "yolo_models" / "duck.hef"
DEFAULT_CALIB = CAMERA_DIR / "calibration_yamls" / "camera_calibration.yaml"
SOURCE_LABEL = "person"
TARGET_LABEL = "duck"


@dataclass(frozen=True)
class DetectionEstimate:
    label: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]
    distance_m: float
    angle_deg: float


class ObstacleTcpServer:
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
        print(f"Obstacle TCP server listening on {self.host}:{self.port}")

    def _accept_new_clients(self) -> None:
        if self._server is None:
            return
        while True:
            try:
                client, address = self._server.accept()
            except BlockingIOError:
                break
            except OSError:
                break

            client.settimeout(self.send_timeout_s)
            self._clients.append(client)
            if self.debug:
                print(f"Obstacle TCP client connected: {address[0]}:{address[1]}")

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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run rectified Hailo duck detection with mono distance logging."
    )
    parser.add_argument(
        "--hailo-apps-root",
        type=Path,
        default=DEFAULT_HAILO_APPS_ROOT,
        help=f"Path to the hailo-apps repository root (default: {DEFAULT_HAILO_APPS_ROOT})",
    )
    parser.add_argument(
        "--network",
        type=Path,
        default=DEFAULT_DUCK_HEF,
        help=f"Path to the duck HEF file (default: {DEFAULT_DUCK_HEF})",
    )
    parser.add_argument(
        "--input",
        default="rpi",
        help="Input source, for example rpi, usb, /dev/video0, or a media path.",
    )
    parser.add_argument(
        "--save-output",
        action="store_true",
        help="Save Hailo's rendered output into the run folder.",
    )
    parser.add_argument(
        "--calib",
        type=Path,
        default=DEFAULT_CALIB,
        help="Path to the camera calibration YAML used for realtime rectification.",
    )
    parser.add_argument(
        "--object-height",
        type=float,
        required=True,
        help="Real-world duck height in meters.",
    )
    parser.add_argument(
        "--csv-name",
        default="detections.csv",
        help="CSV filename inside the run directory (default: detections.csv).",
    )
    parser.add_argument(
        "--hide-distance-overlay",
        action="store_true",
        help="Disable the live distance/angle overlay on the Hailo output window.",
    )
    parser.add_argument(
        "--internal-launch-hailo",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--recording-run-dir",
        type=Path,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--camera-name",
        default=None,
        help=argparse.SUPPRESS,
    )
    return parser


def resolve_hailo_paths(hailo_apps_root: Path) -> tuple[Path, Path]:
    resolved_root = hailo_apps_root.expanduser().resolve()
    object_detection_dir = (
        resolved_root / "hailo_apps" / "python" / "standalone_apps" / "object_detection"
    )
    object_detection_script = object_detection_dir / "object_detection.py"
    return object_detection_dir, object_detection_script


def validate_paths(
    hailo_apps_root: Path,
    object_detection_dir: Path,
    script_path: Path,
    network_path: Path,
    calib_path: Path,
) -> None:
    resolved_root = hailo_apps_root.expanduser().resolve()
    if not resolved_root.exists():
        raise FileNotFoundError(
            f"Hailo apps root does not exist: {resolved_root}\n"
            "Pass the correct location with --hailo-apps-root."
        )
    if not resolved_root.is_dir():
        raise NotADirectoryError(
            f"Hailo apps root is not a directory: {resolved_root}\n"
            "Pass the repository root with --hailo-apps-root."
        )
    if not object_detection_dir.exists():
        raise FileNotFoundError(
            f"Expected object_detection directory was not found: {object_detection_dir}\n"
            "Check that --hailo-apps-root points to the hailo-apps repository."
        )
    if not script_path.exists():
        raise FileNotFoundError(
            f"Expected Hailo entrypoint was not found: {script_path}\n"
            "Check that your hailo-apps checkout contains the standalone object detection app."
        )
    if not network_path.exists():
        raise FileNotFoundError(
            f"Expected duck HEF was not found: {network_path}\n"
            "Pass the correct HEF path with --network."
        )
    if not calib_path.exists():
        raise FileNotFoundError(
            f"Calibration YAML was not found: {calib_path}\n"
            "Pass the correct calibration path with --calib."
        )


def detect_camera_name(input_source: str) -> str:
    lowered = input_source.lower()
    if lowered == "rpi":
        return "pi:0"
    if lowered == "usb":
        return "usb:0"
    if lowered.startswith("/dev/video"):
        suffix = input_source[len("/dev/video") :]
        if suffix.isdigit():
            return f"usb:{suffix}"
        return "usb:0"
    return "camera:0"


def load_calib_yaml(path: Path) -> tuple[tuple[int, int], np.ndarray, np.ndarray]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    image_size = tuple(int(value) for value in data["image_size"])
    camera_matrix = np.array(data["K"], dtype=np.float64)
    distortion = np.array(data["dist"], dtype=np.float64).reshape(-1, 1)
    return image_size, camera_matrix, distortion


def scale_camera_matrix(
    camera_matrix: np.ndarray,
    calib_image_size: tuple[int, int],
    frame_size: tuple[int, int],
) -> np.ndarray:
    calib_w, calib_h = calib_image_size
    frame_w, frame_h = frame_size
    if (calib_w, calib_h) == (frame_w, frame_h):
        return camera_matrix.copy()
    scaled = camera_matrix.copy()
    scaled[0, 0] *= frame_w / calib_w
    scaled[0, 2] *= frame_w / calib_w
    scaled[1, 1] *= frame_h / calib_h
    scaled[1, 2] *= frame_h / calib_h
    return scaled


def get_rectification_state(
    frame_size: tuple[int, int],
    calib_image_size: tuple[int, int],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    cache: dict[tuple[int, int], tuple[np.ndarray, np.ndarray, np.ndarray]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if frame_size not in cache:
        scaled_camera_matrix = scale_camera_matrix(
            camera_matrix=camera_matrix,
            calib_image_size=calib_image_size,
            frame_size=frame_size,
        )
        width, height = frame_size
        rectified_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            scaled_camera_matrix,
            distortion,
            (width, height),
            0.0,
            (width, height),
        )
        map1, map2 = cv2.initUndistortRectifyMap(
            scaled_camera_matrix,
            distortion,
            R=None,
            newCameraMatrix=rectified_camera_matrix,
            size=(width, height),
            m1type=cv2.CV_16SC2,
        )
        cache[frame_size] = (rectified_camera_matrix, map1, map2)
    return cache[frame_size]


def rectify_rgb_frame(
    frame_rgb: np.ndarray,
    calib_image_size: tuple[int, int],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    cache: dict[tuple[int, int], tuple[np.ndarray, np.ndarray, np.ndarray]],
) -> np.ndarray:
    frame_h, frame_w = frame_rgb.shape[:2]
    _, map1, map2 = get_rectification_state(
        frame_size=(frame_w, frame_h),
        calib_image_size=calib_image_size,
        camera_matrix=camera_matrix,
        distortion=distortion,
        cache=cache,
    )
    return cv2.remap(
        frame_rgb,
        map1,
        map2,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )


def estimate_from_rectified_bbox(
    bbox_xyxy: tuple[float, float, float, float],
    object_height_m: float,
    rectified_camera_matrix: np.ndarray,
    label: str,
    confidence: float,
) -> DetectionEstimate:
    x1, y1, x2, y2 = (float(value) for value in bbox_xyxy)
    height_px = abs(y2 - y1)
    if height_px < 1.0:
        raise ValueError("Detected object height is too small for a stable estimate.")
    fx = float(rectified_camera_matrix[0, 0])
    fy = float(rectified_camera_matrix[1, 1])
    cx = float(rectified_camera_matrix[0, 2])
    center_u = 0.5 * (x1 + x2)
    distance_m = (fy * object_height_m) / height_px
    angle_deg = math.degrees(math.atan((center_u - cx) / fx))
    return DetectionEstimate(
        label=label,
        confidence=float(confidence),
        bbox_xyxy=(x1, y1, x2, y2),
        distance_m=float(distance_m),
        angle_deg=float(angle_deg),
    )


def create_run_directory(recordings_dir: Path) -> Path:
    run_name = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = recordings_dir / run_name
    counter = 1
    while run_dir.exists():
        run_dir = recordings_dir / f"{run_name}_{counter:02d}"
        counter += 1
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def write_mission_log(
    mission_log_path: Path,
    run_dir: Path,
    camera_output_dir: Path,
    network: Path,
    input_source: str,
    hailo_apps_root: Path,
    calib_path: Path,
    object_height: float,
    csv_path: Path,
) -> None:
    lines = [
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Run folder: {run_dir}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Camera output dir: {camera_output_dir}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Hailo apps root: {hailo_apps_root}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Calibration YAML: {calib_path}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Network: {network}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Input: {input_source}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Target label: {TARGET_LABEL}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Source label override: {SOURCE_LABEL} -> {TARGET_LABEL}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Object height (m): {object_height}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | CSV path: {csv_path}",
    ]
    mission_log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def ensure_csv_header(csv_path: Path) -> None:
    if csv_path.exists() and csv_path.stat().st_size > 0:
        return
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "timestamp_utc",
                "elapsed_s",
                "camera",
                "label",
                "confidence",
                "bbox_x1",
                "bbox_y1",
                "bbox_x2",
                "bbox_y2",
                "distance_m",
                "angle_deg",
            ]
        )


def iso_timestamp_utc() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def append_detection_rows(
    jsonl_path: Path,
    csv_path: Path,
    timestamp_utc: str,
    elapsed_s: float,
    camera_name: str,
    estimates: list[DetectionEstimate],
) -> None:
    payload = {
        "timestamp_utc": timestamp_utc,
        "elapsed_s": round(elapsed_s, 3),
        "detections": [
            {
                "camera": camera_name,
                "label": estimate.label,
                "confidence": round(estimate.confidence, 4),
                "bbox_xyxy": [round(value, 2) for value in estimate.bbox_xyxy],
                "distance_m": round(estimate.distance_m, 3),
                "angle_deg": round(estimate.angle_deg, 3),
            }
            for estimate in estimates
        ],
    }
    with jsonl_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload) + "\n")
    with csv_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        for estimate in estimates:
            x1, y1, x2, y2 = estimate.bbox_xyxy
            writer.writerow(
                [
                    timestamp_utc,
                    f"{elapsed_s:.3f}",
                    camera_name,
                    estimate.label,
                    f"{estimate.confidence:.4f}",
                    f"{x1:.2f}",
                    f"{y1:.2f}",
                    f"{x2:.2f}",
                    f"{y2:.2f}",
                    f"{estimate.distance_m:.3f}",
                    f"{estimate.angle_deg:.3f}",
                ]
            )


def draw_distance_overlay(
    frame: np.ndarray, estimates: list[DetectionEstimate]
) -> None:
    for estimate in estimates:
        x1, y1, _, _ = [int(round(value)) for value in estimate.bbox_xyxy]
        text = f"{estimate.label} | {estimate.distance_m:.2f}m | {estimate.angle_deg:.2f}deg"
        cv2.putText(
            frame,
            text,
            (x1, max(y1 - 28, 45)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )


def rename_label(label: str) -> str:
    return TARGET_LABEL if str(label).lower() == SOURCE_LABEL else str(label)


def install_label_override_hooks(hailo_apps_root: Path) -> None:
    sys.path.insert(0, str(hailo_apps_root))
    toolbox_module = importlib.import_module("hailo_apps.python.core.common.toolbox")
    post_process_module = importlib.import_module(
        "hailo_apps.python.standalone_apps.object_detection.object_detection_post_process"
    )

    original_get_labels = toolbox_module.get_labels
    original_draw_detections = post_process_module.draw_detections

    def patched_get_labels(labels_path):
        labels = original_get_labels(labels_path)
        patched_labels = [rename_label(label) for label in labels]
        if not patched_labels:
            return [TARGET_LABEL]
        if len(patched_labels) == 1 and patched_labels[0].lower() != TARGET_LABEL:
            patched_labels[0] = TARGET_LABEL
        return patched_labels

    def patched_draw_detections(
        detections, img_out, labels, tracker=None, draw_trail=False
    ):
        patched_labels = [rename_label(label) for label in labels]
        return original_draw_detections(
            detections,
            img_out,
            patched_labels,
            tracker=tracker,
            draw_trail=draw_trail,
        )

    toolbox_module.get_labels = patched_get_labels
    post_process_module.draw_detections = patched_draw_detections


def install_rectified_input_hooks(hailo_apps_root: Path, calib_path: Path) -> None:
    sys.path.insert(0, str(hailo_apps_root))
    toolbox_module = importlib.import_module("hailo_apps.python.core.common.toolbox")
    calib_image_size, camera_matrix, distortion = load_calib_yaml(calib_path)
    map_cache: dict[tuple[int, int], tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    original_init_input_source = toolbox_module.init_input_source

    def rectify_for_pipeline(frame_rgb: np.ndarray) -> np.ndarray:
        return rectify_rgb_frame(
            frame_rgb=frame_rgb,
            calib_image_size=calib_image_size,
            camera_matrix=camera_matrix,
            distortion=distortion,
            cache=map_cache,
        )

    def patched_preprocess_from_cap(
        cap,
        batch_size,
        input_queue,
        width,
        height,
        mode,
        preprocess_fn,
        target_fps=None,
        stop_event=None,
    ):
        def should_stop() -> bool:
            return stop_event is not None and stop_event.is_set()

        if mode == toolbox_module.CapProcessingMode.CAMERA_FRAME_DROP:
            if not target_fps or target_fps <= 0:
                raise ValueError("CAMERA_FRAME_DROP requires a positive target_fps")

        next_keep_ts = toolbox_module.time.monotonic()
        keep_period = (
            1.0 / float(target_fps)
            if mode == toolbox_module.CapProcessingMode.CAMERA_FRAME_DROP
            else None
        )
        video_t0_ms = None
        wall_t0 = None
        frames = []
        processed = []

        while not should_stop():
            ret, frame_bgr = cap.read()
            if not ret:
                break

            if mode == toolbox_module.CapProcessingMode.VIDEO_PACE:
                pos_ms = float(cap.get(cv2.CAP_PROP_POS_MSEC) or 0.0)
                if video_t0_ms is None:
                    video_t0_ms = pos_ms
                    wall_t0 = toolbox_module.time.monotonic()
                desired = wall_t0 + (pos_ms - video_t0_ms) / 1000.0
                now = toolbox_module.time.monotonic()
                if now < desired:
                    toolbox_module.time.sleep(desired - now)

            if mode == toolbox_module.CapProcessingMode.CAMERA_FRAME_DROP:
                now = toolbox_module.time.monotonic()
                if now < next_keep_ts:
                    continue
                next_keep_ts += keep_period

            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            rectified_rgb = rectify_for_pipeline(frame_rgb)
            frames.append(rectified_rgb)
            processed.append(preprocess_fn(rectified_rgb, width, height))

            if len(frames) >= batch_size:
                input_queue.put((frames, processed))
                frames, processed = [], []

        if frames and not should_stop():
            input_queue.put((frames, processed))
        input_queue.put(None)

    def patched_preprocess_images(
        images, batch_size, input_queue, width, height, preprocess_fn
    ):
        for batch in toolbox_module.divide_list_to_batches(images, batch_size):
            rectified_batch = [rectify_for_pipeline(image) for image in batch]
            input_queue.put(
                (
                    rectified_batch,
                    [preprocess_fn(image, width, height) for image in rectified_batch],
                )
            )

    def patched_open_rpi_camera():
        picam2 = None
        try:
            from picamera2 import Picamera2
        except Exception as exc:
            toolbox_module.logger.error(f"Picamera2 not available: {exc}")
            return None

        try:
            picam2 = Picamera2()
            main = {"size": calib_image_size, "format": "RGB888"}
            config = picam2.create_video_configuration(
                main=main,
                controls={"FrameRate": 30},
            )
            picam2.configure(config)
            picam2.start()
            return toolbox_module.PiCamera2CaptureAdapter(picam2)
        except Exception as exc:
            toolbox_module.logger.error(f"Failed to open RPi camera: {exc}")
            if picam2 is not None:
                try:
                    picam2.stop()
                except Exception:
                    pass
                try:
                    picam2.close()
                except Exception:
                    pass
            return None

    def patched_init_input_source(input_src: str, batch_size: int, resolution):
        src = input_src.strip()
        if src == "rpi":
            if not toolbox_module.is_raspberry_pi():
                toolbox_module.logger.error(
                    "RPi camera requested, but this is not a Raspberry Pi system."
                )
                sys.exit(1)
            cap = patched_open_rpi_camera()
            if cap is None:
                sys.exit(1)
            toolbox_module.logger.info(
                f"Using Raspberry Pi camera at {calib_image_size[0]}x{calib_image_size[1]} with rectification"
            )
            return cap, None, "rpi"
        return original_init_input_source(input_src, batch_size, resolution)

    toolbox_module.preprocess_from_cap = patched_preprocess_from_cap
    toolbox_module.preprocess_images = patched_preprocess_images
    toolbox_module.open_rpi_camera = patched_open_rpi_camera
    toolbox_module.init_input_source = patched_init_input_source


def install_headless_cv2_hooks() -> None:
    def _noop(*_args, **_kwargs):
        return None

    def _waitkey_noop(*_args, **_kwargs):
        return -1

    # Force headless behavior so no GUI backend is required on monitor-less systems.
    cv2.imshow = _noop
    cv2.namedWindow = _noop
    cv2.startWindowThread = _noop
    cv2.destroyWindow = _noop
    cv2.destroyAllWindows = _noop
    cv2.waitKey = _waitkey_noop
    cv2.pollKey = _waitkey_noop

    if hasattr(cv2, "waitKeyEx"):
        cv2.waitKeyEx = _waitkey_noop

    if hasattr(cv2, "getWindowProperty"):
        cv2.getWindowProperty = _waitkey_noop


def install_distance_logger(
    hailo_apps_root: Path,
    jsonl_path: Path,
    csv_path: Path,
    calib_path: Path,
    camera_name: str,
    object_height: float,
    show_overlay: bool,
    usb_device: str | None = None,
) -> None:
    sys.path.insert(0, str(hailo_apps_root))
    post_process_module = importlib.import_module(
        "hailo_apps.python.standalone_apps.object_detection.object_detection_post_process"
    )
    original_handler = post_process_module.inference_result_handler
    extract_detections = post_process_module.extract_detections
    calib_image_size, camera_matrix, distortion = load_calib_yaml(calib_path)
    rectification_cache: dict[
        tuple[int, int], tuple[np.ndarray, np.ndarray, np.ndarray]
    ] = {}
    start_time = time.monotonic()

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

    ser = None
    if usb_device:
        import serial

        ser = serial.Serial(usb_device, 9600, timeout=1)

    def close_outputs() -> None:
        if tcp_server is not None:
            tcp_server.close()
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    atexit.register(close_outputs)

    def patched_handler(
        original_frame,
        infer_results,
        labels,
        config_data,
        tracker=None,
        draw_trail=False,
    ):
        detections = extract_detections(original_frame, infer_results, config_data)
        classes = detections["detection_classes"]
        boxes = detections["detection_boxes"]
        scores = detections["detection_scores"]
        frame_h, frame_w = original_frame.shape[:2]
        rectified_camera_matrix, _, _ = get_rectification_state(
            frame_size=(frame_w, frame_h),
            calib_image_size=calib_image_size,
            camera_matrix=camera_matrix,
            distortion=distortion,
            cache=rectification_cache,
        )

        estimates: list[DetectionEstimate] = []
        for class_id, bbox_xyxy, score in zip(classes, boxes, scores):
            label = rename_label(str(labels[class_id]))
            if label.lower() != TARGET_LABEL:
                continue
            try:
                estimates.append(
                    estimate_from_rectified_bbox(
                        bbox_xyxy=tuple(float(value) for value in bbox_xyxy),
                        object_height_m=object_height,
                        rectified_camera_matrix=rectified_camera_matrix,
                        label=label,
                        confidence=float(score),
                    )
                )
            except ValueError:
                continue

        patched_labels = [rename_label(label) for label in labels]
        frame_with_detections = original_handler(
            original_frame,
            infer_results,
            patched_labels,
            config_data,
            tracker=tracker,
            draw_trail=draw_trail,
        )

        if estimates:
            timestamp_utc = iso_timestamp_utc()
            elapsed_s = time.monotonic() - start_time
            append_detection_rows(
                jsonl_path=jsonl_path,
                csv_path=csv_path,
                timestamp_utc=timestamp_utc,
                elapsed_s=elapsed_s,
                camera_name=camera_name,
                estimates=estimates,
            )

            obstacle_line = OBSTACLE_PAIR_SEPARATOR.join(
                f"{estimate.angle_deg:.2f}{OBSTACLE_VALUE_SEPARATOR}{estimate.distance_m:.2f}"
                for estimate in estimates
            )

            if tcp_server is not None:
                tcp_server.send_line(obstacle_line)

            if ser:
                ser.write((obstacle_line + "\n").encode("ascii", errors="ignore"))

            if OBSTACLE_OUTPUT_DEBUG:
                print(f"Obstacle payload: {obstacle_line}")

            if show_overlay:
                draw_distance_overlay(frame_with_detections, estimates)

        return frame_with_detections

    def wrapped_handler(
        original_frame,
        infer_results,
        labels,
        config_data,
        tracker=None,
        draw_trail=False,
    ):
        try:
            return patched_handler(
                original_frame,
                infer_results,
                labels,
                config_data,
                tracker=tracker,
                draw_trail=draw_trail,
            )
        except Exception:
            close_outputs()
            raise

    post_process_module.inference_result_handler = wrapped_handler


def build_launcher_command(
    script_path: Path,
    args: argparse.Namespace,
    run_dir: Path,
    camera_name: str,
) -> list[str]:
    command = [
        sys.executable,
        str(script_path),
        "--internal-launch-hailo",
        "--hailo-apps-root",
        str(args.hailo_apps_root.expanduser().resolve()),
        "--network",
        str(args.network.expanduser().resolve()),
        "--input",
        args.input,
        "--calib",
        str(args.calib.expanduser().resolve()),
        "--object-height",
        str(args.object_height),
        "--csv-name",
        args.csv_name,
        "--recording-run-dir",
        str(run_dir),
        "--camera-name",
        camera_name,
    ]
    if args.save_output:
        command.append("--save-output")
    if args.hide_distance_overlay:
        command.append("--hide-distance-overlay")
    return command


def run_internal_hailo(args: argparse.Namespace) -> int:
    if args.recording_run_dir is None:
        print("Error: internal run is missing --recording-run-dir.", file=sys.stderr)
        return 1
    if not args.camera_name:
        print("Error: internal run is missing --camera-name.", file=sys.stderr)
        return 1

    hailo_root = args.hailo_apps_root.expanduser().resolve()
    network_path = args.network.expanduser().resolve()
    calib_path = args.calib.expanduser().resolve()
    object_detection_dir, object_detection_script = resolve_hailo_paths(hailo_root)

    try:
        validate_paths(
            hailo_root,
            object_detection_dir,
            object_detection_script,
            network_path,
            calib_path,
        )
    except (FileNotFoundError, NotADirectoryError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    run_dir = args.recording_run_dir.expanduser().resolve()
    camera_output_dir = run_dir / "camera0_recording"
    jsonl_path = run_dir / "detections.jsonl"
    csv_path = run_dir / args.csv_name

    os.chdir(object_detection_dir)
    install_headless_cv2_hooks()
    install_label_override_hooks(hailo_root)
    install_rectified_input_hooks(hailo_root, calib_path)
    install_distance_logger(
        hailo_apps_root=hailo_root,
        jsonl_path=jsonl_path,
        csv_path=csv_path,
        calib_path=calib_path,
        camera_name=args.camera_name,
        object_height=args.object_height,
        show_overlay=not args.hide_distance_overlay,
        usb_device=USB_DEVICE,
    )

    object_detection_module = importlib.import_module(
        "hailo_apps.python.standalone_apps.object_detection.object_detection"
    )
    sys.argv = [
        str(object_detection_script),
        "-n",
        str(network_path),
        "-i",
        args.input,
        "--output-dir",
        str(camera_output_dir),
        "--camera-resolution",
        "fhd",
    ]
    if args.save_output:
        sys.argv.append("--save-output")

    object_detection_module.main()
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.internal_launch_hailo:
        return run_internal_hailo(args)

    hailo_root = args.hailo_apps_root.expanduser().resolve()
    network_path = args.network.expanduser().resolve()
    calib_path = args.calib.expanduser().resolve()
    object_detection_dir, object_detection_script = resolve_hailo_paths(hailo_root)

    try:
        validate_paths(
            hailo_root,
            object_detection_dir,
            object_detection_script,
            network_path,
            calib_path,
        )
    except (FileNotFoundError, NotADirectoryError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    recordings_dir = CAMERA_DIR / "recordings"
    run_dir = create_run_directory(recordings_dir)
    camera_output_dir = run_dir / "camera0_recording"
    camera_output_dir.mkdir(parents=True, exist_ok=True)
    jsonl_path = run_dir / "detections.jsonl"
    jsonl_path.touch()
    csv_path = run_dir / args.csv_name
    ensure_csv_header(csv_path)

    camera_name = detect_camera_name(args.input)
    write_mission_log(
        mission_log_path=run_dir / "mission.log",
        run_dir=run_dir,
        camera_output_dir=camera_output_dir,
        network=network_path,
        input_source=args.input,
        hailo_apps_root=hailo_root,
        calib_path=calib_path,
        object_height=args.object_height,
        csv_path=csv_path,
    )

    command = build_launcher_command(
        script_path=SCRIPT_PATH,
        args=args,
        run_dir=run_dir,
        camera_name=camera_name,
    )

    print(f"Camera directory: {CAMERA_DIR}")
    print(f"Hailo apps root: {hailo_root}")
    print(f"Duck HEF: {network_path}")
    print(f"Calibration YAML: {calib_path}")
    print(f"Working directory: {object_detection_dir}")
    print(f"Run directory: {run_dir}")
    print(f"Camera output directory: {camera_output_dir}")
    print(f"Detections JSONL: {jsonl_path}")
    print(f"Detections CSV: {csv_path}")
    print(f"Command: {' '.join(command)}")

    completed = subprocess.run(command, cwd=object_detection_dir, check=False)
    return completed.returncode


if __name__ == "__main__":
    sys.exit(main())
