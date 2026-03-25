#!/usr/bin/env python3
"""Simple mission runtime: record cameras + YOLO logging (no depth)."""

from __future__ import annotations

import argparse
import json
import logging
import signal
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

import json
import time
import signal
import logging
from pathlib import Path
from datetime import datetime
from typing import Any

import numpy as np
import yaml

from gi.repository import Gst

import hailo
from hailo_apps_infra.hailo_rpi_common import app_callback_class
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

from depth_calculation.single_camera_depth_calculation import (
    distance_and_angle_from_bbox,
)


"""
python camera/start_boat_mission.py --backend webcam --webcam-left 0 --webcam-right -1 --model camera/yolo_models/duck.pt --single-camera-depth --object-height-m 0.175 --calib-yaml camera/calibration_yamls/camera_calibration.yaml
"""

"""
python camera/start_boat_mission.py --backend pi --camera-left 0 --camera-right 1 --model camera/yolo_models/duck.pt --single-camera-depth --object-height-m 0.175 --calib-yaml camera/calibration_yamls/camera_calibration.yaml
"""


SCRIPT_DIR = Path(__file__).resolve().parent




def load_intrinsics_from_yaml(
    calib_yaml_path: Path,
) -> tuple[float, float, float, float]:
    with calib_yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if "K" not in data:
        raise ValueError(f"Calibration file {calib_yaml_path} has no 'K' matrix")

    K = np.asarray(data["K"], dtype=float)
    if K.shape != (3, 3):
        raise ValueError(f"Expected K shape (3, 3), got {K.shape}")

    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])
    return fx, fy, cx, cy


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run mission on Pi cameras or laptop webcam with same script.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  Laptop (selfie cam): python camera/start_boat_mission.py --backend webcam --webcam-left 0 --webcam-right -1 --model camera/yolo_models/duck.pt\n"
            "  Raspberry Pi (2 cams): python start_boat_mission.py --backend pi --camera-left 0 --camera-right 1 --model duck.pt"
        ),
    )
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--log-interval", type=float, default=0.2)
    #parser.add_argument("--hef", type=str, required=True)
    parser.add_argument(
    "--network",
    type=str,
    default="yolov8n",
    help="Hailo model name or local HEF path",
    )
    parser.add_argument("--labels-json", type=str, default=None)
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument(
        "--single-camera-depth",
        action="store_true",
        help="Estimate distance/angle from bbox height using one camera model",
    )
    parser.add_argument(
        "--object-height-m",
        type=float,
        default=0.13,
        help="Real-world object height in meters used for single-camera depth",
    )
    parser.add_argument(
        "--calib-yaml",
        type=Path,
        default=SCRIPT_DIR / "calibration_yamls" / "camera_calibration.yaml",
        help="Calibration YAML path containing K matrix (fx, fy, cx, cy)",
    )

    parser.add_argument("--out-dir", type=Path, default=SCRIPT_DIR / "recordings")
    return parser


def setup_logging(run_dir: Path) -> None:
    log_path = run_dir / "mission.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler(log_path, encoding="utf-8"),
        ],
    )

def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    parsed = []
    for det in detections:
        label = det.get_label()
        confidence = float(det.get_confidence())
        bbox = det.get_bbox()  # normalized bbox

        x1 = float(bbox.xmin())
        y1 = float(bbox.ymin())
        x2 = float(bbox.xmax())
        y2 = float(bbox.ymax())

        parsed.append({
            "label": label,
            "confidence": confidence,
            "bbox_norm": [x1, y1, x2, y2],
        })

    user_data.on_detections(parsed)
    return Gst.PadProbeReturn.OK
class BoatMissionHailoApp(GStreamerDetectionApp):
    def __init__(self, args, user_data):
        super().__init__(args, user_data)

        self.app_callback = app_callback

        # Kies HEF
        # self.hef_path = args.hef
        self.hef_path = args.network

        # Postprocess/NMS thresholds
        self.thresholds_str = (
            f"nms-score-threshold={args.conf} "
            f"nms-iou-threshold=0.45 "
            f"output-format-type=HAILO_FORMAT_TYPE_FLOAT32"
        )

        self.create_pipeline()
class BoatMissionCallback(app_callback_class):
    def __init__(
        self,
        run_dir: Path,
        log_interval: float,
        object_height_m: float | None,
        intrinsics: tuple[float, float, float, float] | None,
        frame_width: int,
        frame_height: int,
        camera_name: str = "hailo_cam",
    ):
        super().__init__()
        self.run_dir = run_dir
        self.log_interval = log_interval
        self.object_height_m = object_height_m
        self.intrinsics = intrinsics
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.camera_name = camera_name

        self.last_emit = 0.0
        self.detections_log = run_dir / "detections.jsonl"
        self._fh = self.detections_log.open("a", encoding="utf-8")

    def close(self):
        try:
            self._fh.close()
        except Exception:
            pass

    def on_detections(self, parsed: list[dict[str, Any]]) -> None:
        now = time.monotonic()
        if now - self.last_emit < self.log_interval:
            return
        self.last_emit = now

        output = []
        for det in parsed:
            x1n, y1n, x2n, y2n = det["bbox_norm"]
            x1 = x1n * self.frame_width
            y1 = y1n * self.frame_height
            x2 = x2n * self.frame_width
            y2 = y2n * self.frame_height

            row = {
                "camera": self.camera_name,
                "label": det["label"],
                "confidence": round(det["confidence"], 4),
                "bbox_xyxy": [
                    round(x1, 2), round(y1, 2),
                    round(x2, 2), round(y2, 2),
                ],
            }

            if self.intrinsics is not None and self.object_height_m is not None:
                fx, fy, cx, cy = self.intrinsics
                try:
                    distance_m, angle_deg = distance_and_angle_from_bbox(
                        bbox=(x1, y1, x2, y2),
                        object_height_m=self.object_height_m,
                        fx=fx,
                        fy=fy,
                        cx=cx,
                        cy=cy,
                    )
                    row["distance_m"] = round(float(distance_m), 3)
                    row["angle_deg"] = round(float(angle_deg), 3)
                except ValueError:
                    row["distance_m"] = None
                    row["angle_deg"] = None
            else:
                row["distance_m"] = None
                row["angle_deg"] = None

            output.append(row)

        payload = {
            "timestamp_utc": datetime.utcnow().isoformat(timespec="milliseconds") + "Z",
            "detections": output,
        }
        self._fh.write(json.dumps(payload) + "\n")
        self._fh.flush()


def main() -> int:
    args = create_parser().parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = args.out_dir / stamp
    run_dir.mkdir(parents=True, exist_ok=True)
    setup_logging(run_dir)

    depth_intrinsics = None
    if args.single_camera_depth:
        depth_intrinsics = load_intrinsics_from_yaml(args.calib_yaml)

    user_data = BoatMissionCallback(
        run_dir=run_dir,
        log_interval=args.log_interval,
        object_height_m=args.object_height_m if args.single_camera_depth else None,
        intrinsics=depth_intrinsics,
        frame_width=args.width,
        frame_height=args.height,
        camera_name="pi_hailo",
    )

    app = BoatMissionHailoApp(args, user_data)

    try:
        app.run()
    finally:
        user_data.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
