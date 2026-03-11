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
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from camera.depth_calculation.dual_camera_depth_calculation import depth_from_disparity

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
    center_x: float
    center_y: float
    distance_m: float | None
    axis_offset_m: float | None
    bbox_xyxy: list[float]


class DetectionEngine:
    def __init__(
        self,
        model_path: str,
        conf_threshold: float,
        class_aliases: dict[str, str],
    ) -> None:
        self.conf_threshold = conf_threshold
        self.class_aliases = class_aliases

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
            center_y = (y1 + y2) / 2.0

            output.append(
                DetectionRecord(
                    label=label,
                    confidence=float(conf),
                    center_x=center_x,
                    center_y=center_y,
                    distance_m=None,
                    axis_offset_m=None,
                    bbox_xyxy=[x1, y1, x2, y2],
                )
            )

        return output


class DepthEstimator:
    def annotate(
        self,
        left_detections: list[DetectionRecord],
        right_detections: list[DetectionRecord],
    ) -> tuple[list[DetectionRecord], list[DetectionRecord]]:
        return left_detections, right_detections


class CalibratedDepthEstimator(DepthEstimator):
    def __init__(self, intrinsics: CamIntrinsics) -> None:
        self.intrinsics = intrinsics

    def _annotate_detection(
        self, detection: DetectionRecord, distance_m: float | None
    ) -> DetectionRecord:
        axis_offset = self._estimate_axis_offset_m(distance_m, detection.center_x)
        return replace(
            detection,
            distance_m=distance_m,
            axis_offset_m=axis_offset,
        )

    def _estimate_axis_offset_m(
        self, distance_m: float | None, center_x: float
    ) -> float | None:
        if distance_m is None:
            return None

        return float(
            ((center_x - self.intrinsics.cx) / self.intrinsics.fx) * distance_m
        )


class WidthDepthEstimator(CalibratedDepthEstimator):
    def __init__(
        self, intrinsics: CamIntrinsics, object_width_m: dict[str, float]
    ) -> None:
        super().__init__(intrinsics)
        self.object_width_m = object_width_m

    def annotate(
        self,
        left_detections: list[DetectionRecord],
        right_detections: list[DetectionRecord],
    ) -> tuple[list[DetectionRecord], list[DetectionRecord]]:
        return (
            [
                self._annotate_detection(det, self._estimate_distance_m(det))
                for det in left_detections
            ],
            [
                self._annotate_detection(det, self._estimate_distance_m(det))
                for det in right_detections
            ],
        )

    def _estimate_distance_m(self, detection: DetectionRecord) -> float | None:
        x1, _, x2, _ = detection.bbox_xyxy
        width_px = max(1.0, x2 - x1)
        object_width = self.object_width_m.get(detection.label)
        if object_width is None:
            return None

        distance_m = (self.intrinsics.fx * object_width) / width_px
        if distance_m <= 0 or not math.isfinite(distance_m):
            return None
        return float(distance_m)


class DualCameraDepthEstimator(CalibratedDepthEstimator):
    def __init__(
        self,
        intrinsics: CamIntrinsics,
        baseline_m: float,
        min_disparity_px: float,
        max_vertical_gap_px: float,
    ) -> None:
        super().__init__(intrinsics)
        self.baseline_m = baseline_m
        self.min_disparity_px = min_disparity_px
        self.max_vertical_gap_px = max_vertical_gap_px

    def annotate(
        self,
        left_detections: list[DetectionRecord],
        right_detections: list[DetectionRecord],
    ) -> tuple[list[DetectionRecord], list[DetectionRecord]]:
        annotated_left = list(left_detections)
        annotated_right = list(right_detections)
        unmatched_right = set(range(len(right_detections)))

        for left_index, left_detection in enumerate(left_detections):
            right_index = self._find_match(
                left_detection, right_detections, unmatched_right
            )
            if right_index is None:
                continue

            right_detection = right_detections[right_index]
            disparity_px = abs(left_detection.center_x - right_detection.center_x)
            distance_m = self._estimate_distance_m(disparity_px)
            if distance_m is None:
                continue

            annotated_left[left_index] = self._annotate_detection(
                left_detection, distance_m
            )
            annotated_right[right_index] = self._annotate_detection(
                right_detection, distance_m
            )
            unmatched_right.remove(right_index)

        return annotated_left, annotated_right

    def _find_match(
        self,
        left_detection: DetectionRecord,
        right_detections: list[DetectionRecord],
        unmatched_right: set[int],
    ) -> int | None:
        best_index: int | None = None
        best_score: tuple[float, float] | None = None

        for right_index in unmatched_right:
            candidate = right_detections[right_index]
            if candidate.label != left_detection.label:
                continue

            vertical_gap = abs(left_detection.center_y - candidate.center_y)
            if vertical_gap > self.max_vertical_gap_px:
                continue

            disparity_px = abs(left_detection.center_x - candidate.center_x)
            if disparity_px < self.min_disparity_px:
                continue

            score = (
                vertical_gap,
                abs(
                    self._bbox_width_px(left_detection) - self._bbox_width_px(candidate)
                ),
            )
            if best_score is None or score < best_score:
                best_score = score
                best_index = right_index

        return best_index

    def _estimate_distance_m(self, disparity_px: float) -> float | None:
        try:
            distance_m = depth_from_disparity(
                disparity_px, self.intrinsics.fx, self.baseline_m
            )
        except ValueError:
            return None

        if distance_m <= 0 or not math.isfinite(distance_m):
            return None
        return float(distance_m)

    @staticmethod
    def _bbox_width_px(detection: DetectionRecord) -> float:
        x1, _, x2, _ = detection.bbox_xyxy
        return max(1.0, x2 - x1)


def build_depth_estimator(
    depth_calculation: str,
    intrinsics: CamIntrinsics | None,
    object_width_m: dict[str, float],
    stereo_baseline_m: float,
    stereo_min_disparity_px: float,
    stereo_max_vertical_gap_px: float,
) -> DepthEstimator:
    if depth_calculation == "none":
        return DepthEstimator()

    if intrinsics is None:
        raise ValueError(
            "Calibration intrinsics are required for the selected depth calculation"
        )

    if depth_calculation == "bbox-width":
        return WidthDepthEstimator(intrinsics=intrinsics, object_width_m=object_width_m)

    if depth_calculation == "dual-camera":
        return DualCameraDepthEstimator(
            intrinsics=intrinsics,
            baseline_m=stereo_baseline_m,
            min_disparity_px=stereo_min_disparity_px,
            max_vertical_gap_px=stereo_max_vertical_gap_px,
        )

    raise ValueError(f"Unsupported depth calculation: {depth_calculation}")

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
        "--depth-calculation",
        type=str,
        choices=("dual-camera", "bbox-width", "none"),
        default="dual-camera",
        help="Depth estimation strategy. Keep this switchable so a future single-camera estimator can be added cleanly.",
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
        help="JSON map of real object widths in meters for bbox-width distance estimation",
    )
    parser.add_argument(
        "--stereo-baseline-m",
        type=float,
        default=0.06,
        help="Stereo baseline in meters for dual-camera depth estimation",
    )
    parser.add_argument(
        "--stereo-min-disparity-px",
        type=float,
        default=1.0,
        help="Minimum center disparity in pixels before dual-camera depth is considered valid",
    )
    parser.add_argument(
        "--stereo-max-vertical-gap-px",
        type=float,
        default=80.0,
        help="Maximum vertical center gap in pixels when matching left/right detections",
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

    class_aliases = parse_json_dict(args.class_aliases_json, "class aliases")
    object_widths_raw = parse_json_dict(args.object_widths_json, "object widths")
    object_widths = {str(k): float(v) for k, v in object_widths_raw.items()}
    intrinsics = None
    if args.depth_calculation != "none":
        intrinsics = load_intrinsics(args.calib_yaml)

    detector = DetectionEngine(
        model_path=args.model,
        conf_threshold=args.conf,
        class_aliases={str(k): str(v) for k, v in class_aliases.items()},
    )
    depth_estimator = build_depth_estimator(
        depth_calculation=args.depth_calculation,
        intrinsics=intrinsics,
        object_width_m=object_widths,
        stereo_baseline_m=args.stereo_baseline_m,
        stereo_min_disparity_px=args.stereo_min_disparity_px,
        stereo_max_vertical_gap_px=args.stereo_max_vertical_gap_px,
    )

    left_video = run_dir / f"camera{args.camera_left}.mp4"
    right_video = run_dir / f"camera{args.camera_right}.mp4"

    logging.info("Run folder: %s", run_dir)
    logging.info("Left video: %s", left_video)
    logging.info("Right video: %s", right_video)
    logging.info("Detections log: %s", detections_log_path)
    logging.info("Depth calculation: %s", args.depth_calculation)

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
                    left_detections, right_detections = depth_estimator.annotate(
                        left_detections,
                        right_detections,
                    )

                    payload = {
                        "timestamp_utc": ts_utc,
                        "elapsed_s": round(elapsed, 3),
                        "mock_mode": False,  # Set to true to run without cameras connected
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
