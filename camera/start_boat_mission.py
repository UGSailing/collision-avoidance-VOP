#!/usr/bin/env python3
"""Single-entry runtime for dual-camera recording and periodic object logging.

Expected Raspberry Pi runtime stack:
- Picamera2 for dual camera access and video recording.
- Optional Ultralytics YOLO for object detection.

The script records both camera perspectives and writes a JSONL log entry every
log interval (default 0.1s) with the current detections and estimated distance
metrics.
"""

from __future__ import annotations

import argparse
import importlib
import json
import logging
import math
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

Picamera2: Any = None
H264Encoder: Any = None
FfmpegOutput: Any = None
YOLO: Any = None
SCRIPT_DIR = Path(__file__).resolve().parent


def import_runtime_dependencies(require_cameras: bool = True) -> None:
    global Picamera2, H264Encoder, FfmpegOutput, YOLO

    if require_cameras:
        try:
            picamera2_mod = importlib.import_module("picamera2")
            encoders_mod = importlib.import_module("picamera2.encoders")
            outputs_mod = importlib.import_module("picamera2.outputs")
        except (
            ImportError
        ) as exc:  # pragma: no cover - runtime dependency on Raspberry Pi
            raise SystemExit(
                "Picamera2 is required on the Raspberry Pi. Install it first (python3-picamera2), or run with --mock-no-cameras for testing."
            ) from exc

        Picamera2 = getattr(picamera2_mod, "Picamera2")
        H264Encoder = getattr(encoders_mod, "H264Encoder")
        FfmpegOutput = getattr(outputs_mod, "FfmpegOutput")
    else:
        Picamera2 = None
        H264Encoder = None
        FfmpegOutput = None

    try:
        ultralytics_mod = importlib.import_module("ultralytics")
        YOLO = getattr(ultralytics_mod, "YOLO")
    except ImportError:  # pragma: no cover - optional dependency
        YOLO = None


@dataclass
class CamIntrinsics:
    fx: float
    cx: float


@dataclass
class DetectionRecord:
    label: str
    confidence: float
    distance_m: float | None
    axis_offset_m: float | None
    bbox_xyxy: list[float]


class DetectionEngine:
    def __init__(
        self,
        model_path: str,
        conf_threshold: float,
        class_aliases: dict[str, str],
        object_width_m: dict[str, float],
        intrinsics: CamIntrinsics,
    ) -> None:
        self.conf_threshold = conf_threshold
        self.class_aliases = class_aliases
        self.object_width_m = object_width_m
        self.intrinsics = intrinsics

        self.enabled = YOLO is not None
        self.model = None
        if not self.enabled:
            logging.warning(
                "Ultralytics not installed. Detection will run in no-op mode (empty detections)."
            )
            return

        if YOLO is None:
            return
        self.model = YOLO(model_path)
        logging.info("Loaded YOLO model: %s", model_path)

    def detect(self, frame_bgr: np.ndarray) -> list[DetectionRecord]:
        if not self.model:
            return []

        prediction = self.model.predict(
            source=frame_bgr,
            conf=self.conf_threshold,
            verbose=False,
            device="cpu",
        )
        if not prediction:
            return []

        result = prediction[0]
        if result.boxes is None or result.boxes.xyxy is None:
            return []

        output: list[DetectionRecord] = []
        names = result.names

        xyxy = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)

        for bbox, conf, cls_id in zip(xyxy, confs, classes):
            raw_label = str(names[int(cls_id)])
            label = str(self.class_aliases.get(raw_label, raw_label))

            x1, y1, x2, y2 = [float(v) for v in bbox]
            width_px = max(1.0, x2 - x1)
            center_x = (x1 + x2) / 2.0

            dist = self._estimate_distance_m(label, width_px)
            axis_offset = self._estimate_axis_offset_m(dist, center_x)

            output.append(
                DetectionRecord(
                    label=label,
                    confidence=float(conf),
                    distance_m=dist,
                    axis_offset_m=axis_offset,
                    bbox_xyxy=[x1, y1, x2, y2],
                )
            )

        return output

    def _estimate_distance_m(self, label: str, width_px: float) -> float | None:
        if width_px <= 0:
            return None

        object_width = self.object_width_m.get(label)
        if object_width is None:
            return None

        # Monocular distance approximation: Z = fx * W_real / W_pixels.
        distance_m = (self.intrinsics.fx * object_width) / width_px
        if distance_m <= 0 or not math.isfinite(distance_m):
            return None
        return float(distance_m)

    def _estimate_axis_offset_m(
        self, distance_m: float | None, center_x: float
    ) -> float | None:
        if distance_m is None:
            return None

        # Lateral offset from optical axis at the estimated depth.
        return float(
            ((center_x - self.intrinsics.cx) / self.intrinsics.fx) * distance_m
        )


def load_intrinsics(calib_yaml_path: Path) -> CamIntrinsics:
    with calib_yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if "K" not in data:
        raise ValueError(f"Calibration file {calib_yaml_path} has no 'K' matrix")

    K = np.asarray(data["K"], dtype=float)
    if K.shape != (3, 3):
        raise ValueError(f"Expected K shape (3, 3), got {K.shape}")

    return CamIntrinsics(fx=float(K[0, 0]), cx=float(K[0, 2]))


def parse_json_dict(raw: str, field_name: str) -> dict[str, Any]:
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON for {field_name}: {exc}") from exc

    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be a JSON object")
    return value


def configure_logging(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    handlers = [
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(log_path, encoding="utf-8"),
    ]
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
        handlers=handlers,
    )


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Start dual-camera recording and write 10Hz object log lines."
    )
    parser.add_argument(
        "--duration", type=float, default=60.0, help="Run time in seconds"
    )
    parser.add_argument("--camera-left", type=int, default=0, help="Left camera id")
    parser.add_argument("--camera-right", type=int, default=1, help="Right camera id")
    parser.add_argument("--width", type=int, default=1280, help="Capture width")
    parser.add_argument("--height", type=int, default=720, help="Capture height")
    parser.add_argument("--fps", type=int, default=30, help="Capture FPS")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=SCRIPT_DIR / "recordings",
        help="Output folder",
    )
    parser.add_argument(
        "--calib-yaml",
        type=Path,
        default=SCRIPT_DIR / "calibration_yamls" / "camera_calibration.yaml",
        help="Calibration YAML path for focal length and optical center",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=0.1,
        help="Detection log interval in seconds (default: 0.1)",
    )
    parser.add_argument(
        "--model",
        type=str,
        default="yolov8n.pt",
        help="Ultralytics model path or name",
    )
    parser.add_argument(
        "--conf", type=float, default=0.25, help="Detection confidence threshold"
    )
    parser.add_argument(
        "--class-aliases-json",
        type=str,
        default='{"bird": "duck"}',
        help='JSON map to rename detector labels, e.g. {"bird":"duck"}',
    )
    parser.add_argument(
        "--object-widths-json",
        type=str,
        default='{"duck": 0.25, "buoy": 0.30, "person": 0.45}',
        help="JSON map of real object widths in meters for distance estimation",
    )
    parser.add_argument(
        "--mock-no-cameras",
        action="store_true",
        help="Run full timing/logging loop without camera hardware; writes placeholder camera data.",
    )
    return parser


def build_camera(camera_id: int, width: int, height: int, fps: int) -> Any:
    camera = Picamera2(camera_id)
    video_cfg = camera.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": fps},
    )
    camera.configure(video_cfg)
    return camera


def start_recording(camera: Any, output_path: Path, bitrate: int = 8_000_000) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    encoder = H264Encoder(bitrate=bitrate)
    output = FfmpegOutput(str(output_path))
    camera.start_recording(encoder, output)


def stop_recording(camera: Any) -> None:
    try:
        camera.stop_recording()
    except Exception:
        pass
    try:
        camera.stop()
    except Exception:
        pass
    try:
        camera.close()
    except Exception:
        pass


def main() -> int:
    args = create_parser().parse_args()
    import_runtime_dependencies(require_cameras=not args.mock_no_cameras)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = args.out_dir / stamp
    app_log_path = run_dir / "mission.log"
    detections_log_path = run_dir / "detections.jsonl"

    configure_logging(app_log_path)

    intrinsics = load_intrinsics(args.calib_yaml)
    class_aliases = parse_json_dict(args.class_aliases_json, "class aliases")
    object_widths_raw = parse_json_dict(args.object_widths_json, "object widths")
    object_widths = {str(k): float(v) for k, v in object_widths_raw.items()}

    detector = DetectionEngine(
        model_path=args.model,
        conf_threshold=args.conf,
        class_aliases={str(k): str(v) for k, v in class_aliases.items()},
        object_width_m=object_widths,
        intrinsics=intrinsics,
    )

    left_video = run_dir / f"camera{args.camera_left}.mp4"
    right_video = run_dir / f"camera{args.camera_right}.mp4"

    logging.info("Run folder: %s", run_dir)
    logging.info("Left video: %s", left_video)
    logging.info("Right video: %s", right_video)
    logging.info("Detections log: %s", detections_log_path)

    left_cam = None
    right_cam = None

    if not args.mock_no_cameras:
        left_cam = build_camera(args.camera_left, args.width, args.height, args.fps)
        right_cam = build_camera(args.camera_right, args.width, args.height, args.fps)
    else:
        logging.warning(
            "Running in --mock-no-cameras mode. No real camera/video input; logs will contain '-' placeholders."
        )

    stop_requested = False

    def _signal_handler(signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True
        logging.info("Signal %s received, stopping...", signum)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    if not args.mock_no_cameras and left_cam is not None and right_cam is not None:
        left_cam.start()
        right_cam.start()
        start_recording(left_cam, left_video)
        start_recording(right_cam, right_video)

    start_t = time.monotonic()
    next_log_t = start_t

    run_dir.mkdir(parents=True, exist_ok=True)
    with detections_log_path.open("a", encoding="utf-8") as det_file:
        try:
            while not stop_requested:
                now = time.monotonic()
                elapsed = now - start_t
                if elapsed >= args.duration:
                    break

                if now < next_log_t:
                    time.sleep(min(0.01, next_log_t - now))
                    continue

                ts_utc = datetime.utcnow().isoformat(timespec="milliseconds") + "Z"

                if args.mock_no_cameras:
                    payload = {
                        "timestamp_utc": ts_utc,
                        "elapsed_s": round(elapsed, 3),
                        "mock_mode": True,
                        "camera_data": {
                            str(args.camera_left): "-",
                            str(args.camera_right): "-",
                        },
                        "detections": [
                            {
                                "camera": args.camera_left,
                                "label": "-",
                                "confidence": "-",
                                "distance_m": "-",
                                "axis_offset_m": "-",
                                "bbox_xyxy": "-",
                            },
                            {
                                "camera": args.camera_right,
                                "label": "-",
                                "confidence": "-",
                                "distance_m": "-",
                                "axis_offset_m": "-",
                                "bbox_xyxy": "-",
                            },
                        ],
                    }
                else:
                    if left_cam is None or right_cam is None:
                        raise RuntimeError("Camera objects were not initialized.")

                    left_frame = left_cam.capture_array("main")[:, :, ::-1]
                    right_frame = right_cam.capture_array("main")[:, :, ::-1]

                    left_detections = detector.detect(left_frame)
                    right_detections = detector.detect(right_frame)

                    payload = {
                        "timestamp_utc": ts_utc,
                        "elapsed_s": round(elapsed, 3),
                        "mock_mode": False,
                        "detections": [
                            {
                                "camera": args.camera_left,
                                "label": det.label,
                                "confidence": round(det.confidence, 4),
                                "distance_m": (
                                    None
                                    if det.distance_m is None
                                    else round(det.distance_m, 3)
                                ),
                                "axis_offset_m": (
                                    None
                                    if det.axis_offset_m is None
                                    else round(det.axis_offset_m, 3)
                                ),
                                "bbox_xyxy": [round(v, 2) for v in det.bbox_xyxy],
                            }
                            for det in left_detections
                        ]
                        + [
                            {
                                "camera": args.camera_right,
                                "label": det.label,
                                "confidence": round(det.confidence, 4),
                                "distance_m": (
                                    None
                                    if det.distance_m is None
                                    else round(det.distance_m, 3)
                                ),
                                "axis_offset_m": (
                                    None
                                    if det.axis_offset_m is None
                                    else round(det.axis_offset_m, 3)
                                ),
                                "bbox_xyxy": [round(v, 2) for v in det.bbox_xyxy],
                            }
                            for det in right_detections
                        ],
                    }

                det_file.write(json.dumps(payload) + "\n")
                det_file.flush()

                next_log_t += args.log_interval

        finally:
            if left_cam is not None:
                stop_recording(left_cam)
            if right_cam is not None:
                stop_recording(right_cam)

    logging.info("Finished run.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
