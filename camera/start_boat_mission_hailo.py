#!/usr/bin/env python3
# VERGEET NIET DE source setup... vanuit hailo als venv te draaien!!!!!
# Linux examples:
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 start_boat_mission_hailo.py --input rpi --object-height 0.21367
#
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 start_boat_mission_hailo.py --input rpi --object-height 0.23 --save-output
#
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 start_boat_mission_hailo.py --input rpi --camera-index 1 --object-height 0.23


#python3 start_boat_mission_hailo.py --input /home/mario/Documents/collision-avoidance-VOP/camera/recordings/Ball_duck_water/duck_in_water.mov --object-height 0.213267
"""Run rectified Hailo duck detection with mono distance and TCP obstacle output."""

import argparse
import json
import logging
import math
import socket
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

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
    OBSTACLE_TCP_SEND_EMPTY_UPDATES,
    OBSTACLE_TCP_SEND_TIMEOUT_S,
    OBSTACLE_VALUE_SEPARATOR,
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
        logging.info("[TCP] listening on %s:%s", self.host, self.port)

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
                logging.info("[TCP] client connected: %s:%s", address[0], address[1])

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


def build_obstacle_tcp_payload(detections: list[dict[str, Any]]) -> str | None:
    pairs: list[str] = []
    for detection in detections:
        distance_m = detection.get("distance_m")
        angle_deg = detection.get("angle_deg")
        if distance_m is None or angle_deg is None:
            continue

        pairs.append(
            f"{float(angle_deg):.2f}{OBSTACLE_VALUE_SEPARATOR}{float(distance_m):.2f}{OBSTACLE_PAIR_SEPARATOR}"
        )

    if pairs:
        return "".join(pairs)

    if OBSTACLE_TCP_SEND_EMPTY_UPDATES:
        return ""

    return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run rectified Hailo duck detection with mono distance and TCP obstacle output."
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
        "--camera-index",
        type=int,
        default=0,
        help="Picamera2 camera index to use for --input rpi (default: 0).",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.4,
        help="Detection confidence threshold (default: 0.4).",
    )
    parser.add_argument(
        "--labels",
        type=Path,
        default=None,
        help="Optional labels file. Defaults to a single duck label.",
    )
    parser.add_argument(
        "--save-output",
        action="store_true",
        help="Save rectified annotated video into the run folder.",
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
        help="Compatibility option from run_hailo_detection3_duck.py; TCP output is used instead of CSV.",
    )
    parser.add_argument(
        "--hide-distance-overlay",
        action="store_true",
        help="Disable the live distance/angle overlay on the Hailo output window.",
    )
    return parser


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
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | TCP enabled: {OBSTACLE_TCP_ENABLED}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | TCP bind: {OBSTACLE_TCP_BIND_HOST}:{OBSTACLE_TCP_PORT}",
    ]
    mission_log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def iso_timestamp_utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z"
    )


def append_detection_log_and_send_tcp(
    jsonl_path: Path,
    timestamp_utc: str,
    elapsed_s: float,
    camera_name: str,
    estimates: list[DetectionEstimate],
    tcp_server: ObstacleTcpServer | None,
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

    obstacle_payload = build_obstacle_tcp_payload(payload["detections"])
    if tcp_server is not None and obstacle_payload is not None:
        print(f"Sent to tcp: {obstacle_payload}")
        tcp_server.send_line(obstacle_payload)
        if OBSTACLE_OUTPUT_DEBUG:
            logging.info("[TCP] payload: %s", obstacle_payload)


def draw_distance_overlay(frame: np.ndarray, estimates: list[DetectionEstimate]) -> None:
    for estimate in estimates:
        x1, y1, x2, y2 = [int(round(value)) for value in estimate.bbox_xyxy]
        text = f"{estimate.label} | {estimate.distance_m:.2f}m | {estimate.angle_deg:.2f}deg"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
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


def load_labels(labels_path: Path | None) -> list[str]:
    if labels_path is None:
        return [TARGET_LABEL]
    labels = [
        rename_label(line.strip())
        for line in labels_path.expanduser().resolve().read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not labels:
        return [TARGET_LABEL]
    if len(labels) == 1 and labels[0].lower() != TARGET_LABEL:
        labels[0] = TARGET_LABEL
    return labels


def extract_detections(
    hailo_output: Any,
    frame_w: int,
    frame_h: int,
    labels: list[str],
    threshold: float,
) -> list[tuple[str, float, tuple[float, float, float, float]]]:
    results: list[tuple[str, float, tuple[float, float, float, float]]] = []
    for class_id, detections in enumerate(hailo_output):
        for det in detections:
            score = float(det[4])
            if score < threshold:
                continue
            top_norm, left_norm, bottom_norm, right_norm = (float(value) for value in det[:4])
            left = left_norm * frame_w
            top = top_norm * frame_h
            right = right_norm * frame_w
            bottom = bottom_norm * frame_h
            label = labels[class_id] if class_id < len(labels) else str(class_id)
            results.append((rename_label(label), score, (left, top, right, bottom)))
    return results


def configure_logging(mission_log_path: Path) -> None:
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    for handler in list(root_logger.handlers):
        root_logger.removeHandler(handler)
    formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")
    file_handler = logging.FileHandler(mission_log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    root_logger.addHandler(file_handler)
    root_logger.addHandler(stream_handler)


def run_headless_rpi_hailo(
    args: argparse.Namespace,
    run_dir: Path,
    camera_output_dir: Path,
    jsonl_path: Path,
    camera_name: str,
) -> int:
    try:
        from picamera2 import Picamera2
        from picamera2.devices import Hailo
    except Exception as exc:
        print(f"Error: Picamera2/HailoRT imports failed: {exc}", file=sys.stderr)
        return 1

    network_path = args.network.expanduser().resolve()
    calib_path = args.calib.expanduser().resolve()
    if not network_path.exists():
        print(f"Error: Expected duck HEF was not found: {network_path}", file=sys.stderr)
        return 1
    if not calib_path.exists():
        print(f"Error: Calibration YAML was not found: {calib_path}", file=sys.stderr)
        return 1
    if args.labels is not None and not args.labels.expanduser().resolve().exists():
        print(f"Error: Labels file was not found: {args.labels}", file=sys.stderr)
        return 1

    labels = load_labels(args.labels)
    calib_image_size, camera_matrix, distortion = load_calib_yaml(calib_path)
    rectification_cache: dict[tuple[int, int], tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    main_size = calib_image_size
    main_w, main_h = main_size
    video_writer: cv2.VideoWriter | None = None
    tcp_server: ObstacleTcpServer | None = None

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

    print(f"Running headless Picamera2/Hailo on camera index {args.camera_index}")
    logging.info("Running headless Picamera2/Hailo on camera index %s", args.camera_index)

    start_time = time.monotonic()
    frame_count = 0
    try:
        with Hailo(str(network_path)) as hailo:
            model_h, model_w, _ = hailo.get_input_shape()
            with Picamera2(args.camera_index) as picam2:
                config = picam2.create_video_configuration(
                    main={"size": main_size, "format": "RGB888"},
                    lores={"size": (model_w, model_h), "format": "RGB888"},
                    controls={"FrameRate": 30},
                )
                picam2.configure(config)
                picam2.start()

                rectified_camera_matrix, _, _ = get_rectification_state(
                    frame_size=main_size,
                    calib_image_size=calib_image_size,
                    camera_matrix=camera_matrix,
                    distortion=distortion,
                    cache=rectification_cache,
                )

                while True:
                    lores = picam2.capture_array("lores")
                    output = hailo.run(lores)
                    raw_detections = extract_detections(
                        output,
                        frame_w=main_w,
                        frame_h=main_h,
                        labels=labels,
                        threshold=args.threshold,
                    )

                    estimates: list[DetectionEstimate] = []
                    for label, score, bbox_xyxy in raw_detections:
                        if label.lower() != TARGET_LABEL:
                            continue
                        try:
                            estimates.append(
                                estimate_from_rectified_bbox(
                                    bbox_xyxy=bbox_xyxy,
                                    object_height_m=args.object_height,
                                    rectified_camera_matrix=rectified_camera_matrix,
                                    label=TARGET_LABEL,
                                    confidence=score,
                                )
                            )
                        except ValueError:
                            continue

                    elapsed_s = time.monotonic() - start_time
                    append_detection_log_and_send_tcp(
                        jsonl_path=jsonl_path,
                        timestamp_utc=iso_timestamp_utc(),
                        elapsed_s=elapsed_s,
                        camera_name=camera_name,
                        estimates=estimates,
                        tcp_server=tcp_server,
                    )

                    if OBSTACLE_OUTPUT_DEBUG and estimates:
                        for estimate in estimates:
                            logging.info(
                                "Detection %s %.2f bbox=%s distance=%.2fm angle=%.2fdeg",
                                estimate.label,
                                estimate.confidence,
                                tuple(round(value, 1) for value in estimate.bbox_xyxy),
                                estimate.distance_m,
                                estimate.angle_deg,
                            )

                    if args.save_output:
                        main_frame = picam2.capture_array("main")
                        rectified_frame = rectify_rgb_frame(
                            frame_rgb=main_frame,
                            calib_image_size=calib_image_size,
                            camera_matrix=camera_matrix,
                            distortion=distortion,
                            cache=rectification_cache,
                        )
                        if not args.hide_distance_overlay:
                            draw_distance_overlay(rectified_frame, estimates)
                        if video_writer is None:
                            output_path = camera_output_dir / "headless_hailo_output.mp4"
                            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                            video_writer = cv2.VideoWriter(
                                str(output_path),
                                fourcc,
                                30.0,
                                (rectified_frame.shape[1], rectified_frame.shape[0]),
                            )
                            if not video_writer.isOpened():
                                raise RuntimeError(f"Failed to open video writer: {output_path}")
                            logging.info("Saving video to %s", output_path)
                        video_writer.write(cv2.cvtColor(rectified_frame, cv2.COLOR_RGB2BGR))

                    frame_count += 1
                    if frame_count % 30 == 0:
                        fps = frame_count / max(time.monotonic() - start_time, 0.001)
                        print(f"FPS: {fps:.2f}", flush=True)
                        logging.info("FPS: %.2f", fps)
    except KeyboardInterrupt:
        logging.info("Interrupted by user.")
        return 0
    finally:
        if video_writer is not None:
            video_writer.release()
        if tcp_server is not None:
            tcp_server.close()
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    hailo_root = args.hailo_apps_root.expanduser().resolve()
    network_path = args.network.expanduser().resolve()
    calib_path = args.calib.expanduser().resolve()

    recordings_dir = CAMERA_DIR / "recordings"
    run_dir = create_run_directory(recordings_dir)
    camera_output_dir = run_dir / "camera0_recording"
    camera_output_dir.mkdir(parents=True, exist_ok=True)
    jsonl_path = run_dir / "detections.jsonl"
    jsonl_path.touch()

    camera_name = f"pi:{args.camera_index}" if args.input.lower() == "rpi" else detect_camera_name(args.input)
    write_mission_log(
        mission_log_path=run_dir / "mission.log",
        run_dir=run_dir,
        camera_output_dir=camera_output_dir,
        network=network_path,
        input_source=args.input,
        hailo_apps_root=hailo_root,
        calib_path=calib_path,
        object_height=args.object_height,
    )
    configure_logging(run_dir / "mission.log")

    print(f"Camera directory: {CAMERA_DIR}")
    print(f"Duck HEF: {network_path}")
    print(f"Calibration YAML: {calib_path}")
    print(f"Run directory: {run_dir}")
    print(f"Camera output directory: {camera_output_dir}")
    print(f"Detections JSONL: {jsonl_path}")
    print(f"Obstacle TCP enabled: {OBSTACLE_TCP_ENABLED}")
    print(f"Obstacle TCP bind: {OBSTACLE_TCP_BIND_HOST}:{OBSTACLE_TCP_PORT}")

    if args.input.lower() == "rpi":
        return run_headless_rpi_hailo(
            args=args,
            run_dir=run_dir,
            camera_output_dir=camera_output_dir,
            jsonl_path=jsonl_path,
            camera_name=camera_name,
        )

    raise NotImplementedError(
        "The new headless Hailo runner currently supports --input rpi only. "
        "Use --input rpi with --camera-index 0 or 1."
    )


if __name__ == "__main__":
    sys.exit(main())
