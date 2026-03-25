#!/usr/bin/env python3
"""Simple mission runtime: record cameras + YOLO logging (no depth)."""

from __future__ import annotations

import argparse
import importlib
import json
import logging
import signal
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

# import can_comms
from depth_calculation.single_camera_depth_calculation import (
    distance_and_angle_from_bbox,
)


"""
python camera/start_boat_mission.py --backend webcam --webcam-left 0 --webcam-right -1 --model camera/yolo_models/duck.pt --single-camera-depth --object-height-m 0.175 --calib-yaml camera/calibration_yamls/camera_calibration.yaml
"""

"""
python camera/start_boat_mission.py --backend pi --camera-left 0 --camera-right 1 --model camera/yolo_models/duck.pt --single-camera-depth --object-height-m 0.175 --calib-yaml camera/calibration_yamls/camera_calibration.yaml
"""

try:
    import cv2
except ImportError:
    cv2 = None

YOLO: Any = None
Picamera2: Any = None
H264Encoder: Any = None
FfmpegOutput: Any = None

SCRIPT_DIR = Path(__file__).resolve().parent


def import_yolo() -> None:
    global YOLO
    ultralytics_mod = importlib.import_module("ultralytics")
    YOLO = getattr(ultralytics_mod, "YOLO")


def import_picamera_runtime() -> None:
    global Picamera2, H264Encoder, FfmpegOutput
    picamera2_mod = importlib.import_module("picamera2")
    encoders_mod = importlib.import_module("picamera2.encoders")
    outputs_mod = importlib.import_module("picamera2.outputs")
    Picamera2 = getattr(picamera2_mod, "Picamera2")
    H264Encoder = getattr(encoders_mod, "H264Encoder")
    FfmpegOutput = getattr(outputs_mod, "FfmpegOutput")


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
    parser.add_argument("--backend", choices=("pi", "webcam", "mock"), default="pi")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--log-interval", type=float, default=0.2)
    parser.add_argument("--model", type=str, default="duck.pt")
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

    parser.add_argument("--camera-left", type=int, default=0, help="Pi left camera id")
    parser.add_argument(
        "--camera-right", type=int, default=1, help="Pi right camera id"
    )

    parser.add_argument(
        "--webcam-left", type=int, default=0, help="Laptop webcam left index"
    )
    parser.add_argument(
        "--webcam-right",
        type=int,
        default=-1,
        help="Laptop webcam right index; -1 means disabled",
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


def build_pi_camera(camera_id: int, width: int, height: int, fps: int) -> Any:
    camera = Picamera2(camera_id)
    config = camera.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": fps},
    )
    camera.configure(config)
    return camera


def start_pi_recording(
    camera: Any, output_path: Path, bitrate: int = 8_000_000
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    encoder = H264Encoder(bitrate=bitrate)
    output = FfmpegOutput(str(output_path))
    camera.start_recording(encoder, output)


def stop_pi_camera(camera: Any) -> None:
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


def build_webcam(index: int, width: int, height: int, fps: int) -> Any:
    if cv2 is None:
        raise SystemExit(
            "OpenCV is required for --backend webcam. Install opencv-python."
        )

    cam = cv2.VideoCapture(index)
    if not cam.isOpened():
        raise SystemExit(f"Could not open webcam index {index}")

    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cam.set(cv2.CAP_PROP_FPS, fps)
    return cam


def build_webcam_writer(path: Path, width: int, height: int, fps: int) -> Any:
    if cv2 is None:
        raise SystemExit(
            "OpenCV is required for --backend webcam. Install opencv-python."
        )

    path.parent.mkdir(parents=True, exist_ok=True)

    for codec in ("mp4v", "avc1", "XVID"):
        fourcc = cv2.VideoWriter_fourcc(*codec)
        writer = cv2.VideoWriter(str(path), fourcc, fps, (width, height))
        if writer.isOpened():
            logging.info(
                "Opened writer %s with codec %s (%sx%s @ %sfps)",
                path,
                codec,
                width,
                height,
                fps,
            )
            return writer

    raise SystemExit(
        f"Could not open video writer for {path} with codecs mp4v/avc1/XVID"
    )


def detect(frame_bgr: np.ndarray, model: Any, conf: float, camera_name: str):
    prediction = model.predict(source=frame_bgr, conf=conf, verbose=False, device="cpu")
    if not prediction:
        return [], frame_bgr

    result = prediction[0]
    annotated = result.plot()  # draws bbox + label + confidence

    if result.boxes is None or result.boxes.xyxy is None:
        return [], annotated

    names = result.names
    xyxy = result.boxes.xyxy.cpu().numpy()
    confs = result.boxes.conf.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)

    output = []
    for bbox, score, cls_id in zip(xyxy, confs, classes):
        x1, y1, x2, y2 = [float(v) for v in bbox]
        output.append(
            {
                "camera": camera_name,
                "label": str(names[int(cls_id)]),
                "confidence": round(float(score), 4),
                "bbox_xyxy": [round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)],
            }
        )
    return output, annotated


def enrich_detections_with_single_camera_depth(
    detections: list[dict[str, Any]],
    object_height_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> None:
    for detection in detections:
        bbox_xyxy = detection.get("bbox_xyxy")
        if not isinstance(bbox_xyxy, list) or len(bbox_xyxy) != 4:
            detection["distance_m"] = None
            detection["angle_deg"] = None
            continue

        try:
            distance_m, angle_deg = distance_and_angle_from_bbox(
                bbox=tuple(float(v) for v in bbox_xyxy),
                object_height_m=object_height_m,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
            )
            detection["distance_m"] = round(float(distance_m), 3)
            detection["angle_deg"] = round(float(angle_deg), 3)
        except ValueError:
            detection["distance_m"] = None
            detection["angle_deg"] = None


def main() -> int:
    args = create_parser().parse_args()

    import_yolo()
    if args.backend == "pi":
        try:
            import_picamera_runtime()
        except ImportError as exc:
            raise SystemExit(
                "Picamera2 runtime not found. Use --backend webcam on laptop, or install picamera2 on Pi."
            ) from exc

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = args.out_dir / stamp
    run_dir.mkdir(parents=True, exist_ok=True)
    setup_logging(run_dir)

    cam0_dir = run_dir / "camera0_recording"
    cam1_dir = run_dir / "camera1_recording"
    cam0_dir.mkdir(parents=True, exist_ok=True)
    cam1_dir.mkdir(parents=True, exist_ok=True)

    left_video = cam0_dir / "recording.mp4"
    right_video = cam1_dir / "recording.mp4"
    left_annot_video = cam0_dir / "recording_annotated.mp4"
    right_annot_video = cam1_dir / "recording_annotated.mp4"
    detections_log = run_dir / "detections.jsonl"

    logging.info("Run folder: %s", run_dir)
    logging.info("Backend: %s", args.backend)
    logging.info("Model: %s", args.model)
    logging.info("Resolution/FPS: %sx%s @ %s", args.width, args.height, args.fps)
    logging.info("Log interval: %s sec", args.log_interval)

    model = YOLO(args.model)

    depth_intrinsics: tuple[float, float, float, float] | None = None
    if args.single_camera_depth:
        try:
            depth_intrinsics = load_intrinsics_from_yaml(args.calib_yaml)
        except (FileNotFoundError, ValueError) as exc:
            raise SystemExit(
                f"Could not load intrinsics from --calib-yaml={args.calib_yaml}: {exc}"
            ) from exc

        fx, fy, cx, cy = depth_intrinsics
        logging.info(
            "Single-camera depth enabled with intrinsics from %s | fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
            args.calib_yaml,
            fx,
            fy,
            cx,
            cy,
        )

    left_cam = None
    right_cam = None
    left_writer = None
    right_writer = None
    left_annot_writer = None
    right_annot_writer = None

    left_name = "left"
    right_name = "right"

    # can = can_comms.CANComms()

    if args.backend == "pi":
        left_name = f"pi:{args.camera_left}"
        right_name = f"pi:{args.camera_right}"
        left_cam = build_pi_camera(args.camera_left, args.width, args.height, args.fps)
        right_cam = build_pi_camera(
            args.camera_right, args.width, args.height, args.fps
        )

        left_cam.start()
        right_cam.start()
        start_pi_recording(left_cam, left_video)
        start_pi_recording(right_cam, right_video)
        left_annot_writer = build_webcam_writer(
            left_annot_video, args.width, args.height, args.fps
        )
        right_annot_writer = build_webcam_writer(
            right_annot_video, args.width, args.height, args.fps
        )

    elif args.backend == "webcam":
        left_video = cam0_dir / "recording.mp4"
        right_video = cam1_dir / "recording.mp4"

        left_name = f"webcam:{args.webcam_left}"
        left_cam = build_webcam(args.webcam_left, args.width, args.height, args.fps)
        left_writer = None

        logging.info(
            "Left webcam actual mode: %sx%s @ %sfps",
            int(left_cam.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(left_cam.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            left_cam.get(cv2.CAP_PROP_FPS),
        )

        if args.webcam_right >= 0:
            right_name = f"webcam:{args.webcam_right}"
            right_cam = build_webcam(
                args.webcam_right, args.width, args.height, args.fps
            )
            right_writer = None

            logging.info(
                "Right webcam actual mode: %sx%s @ %sfps",
                int(right_cam.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(right_cam.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                right_cam.get(cv2.CAP_PROP_FPS),
            )
        else:
            logging.info(
                "No right webcam configured (use --webcam-right N for two webcams)."
            )

    else:
        logging.warning("Running in mock mode: no real video input.")

    stop_requested = False

    def _signal_handler(signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True
        logging.info("Signal %s received, stopping...", signum)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    start_t = time.monotonic()
    next_log_t = start_t

    with detections_log.open("a", encoding="utf-8") as log_file:
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

                if args.backend == "mock":
                    payload = {
                        "timestamp_utc": ts_utc,
                        "elapsed_s": round(elapsed, 3),
                        "detections": [],
                    }
                else:
                    if left_cam is None:
                        raise RuntimeError("Left camera not initialized.")

                    if args.backend == "pi":
                        left_frame = left_cam.capture_array("main")[:, :, ::-1]
                    else:
                        ok_left, left_frame = left_cam.read()
                        if not ok_left:
                            logging.warning("Could not read left webcam frame")
                            next_log_t += args.log_interval
                            continue

                        if left_writer is None:
                            left_h, left_w = left_frame.shape[:2]
                            left_writer = build_webcam_writer(
                                left_video, left_w, left_h, args.fps
                            )

                        if left_writer is not None:
                            left_writer.write(left_frame)

                    detections, left_annotated = detect(
                        left_frame, model, args.conf, left_name
                    )
                    if left_writer is not None:
                        left_writer.write(left_annotated)
                    if left_annot_writer is not None:
                        left_annot_writer.write(left_annotated)

                    if right_cam is not None:
                        if args.backend == "pi":
                            right_frame = right_cam.capture_array("main")[:, :, ::-1]
                        else:
                            ok_right, right_frame = right_cam.read()
                            if ok_right:
                                if right_writer is None:
                                    right_h, right_w = right_frame.shape[:2]
                                    right_writer = build_webcam_writer(
                                        right_video, right_w, right_h, args.fps
                                    )

                                if right_writer is not None:
                                    right_writer.write(right_frame)
                            else:
                                right_frame = None
                                logging.warning("Could not read right webcam frame")

                        if right_frame is not None:
                            right_detections, right_annotated = detect(
                                right_frame, model, args.conf, right_name
                            )
                            detections += right_detections
                            if right_writer is not None:
                                right_writer.write(right_annotated)
                            if right_annot_writer is not None:
                                right_annot_writer.write(right_annotated)

                    if args.single_camera_depth:
                        if depth_intrinsics is None:
                            raise RuntimeError("Depth intrinsics were not initialized.")

                        fx, fy, cx, cy = depth_intrinsics
                        enrich_detections_with_single_camera_depth(
                            detections=detections,
                            object_height_m=args.object_height_m,
                            fx=fx,
                            fy=fy,
                            cx=cx,
                            cy=cy,
                        )

                    payload = {
                        "timestamp_utc": ts_utc,
                        "elapsed_s": round(elapsed, 3),
                        "detections": detections,
                    }

                log_file.write(json.dumps(payload) + "\n")
                log_file.flush()

                # can.send_objects(payload)

                next_log_t += args.log_interval

        finally:
            # can.close()

            if args.backend == "pi":
                if left_cam is not None:
                    stop_pi_camera(left_cam)
                if right_cam is not None:
                    stop_pi_camera(right_cam)
            elif args.backend == "webcam":
                if left_cam is not None:
                    left_cam.release()
                if right_cam is not None:
                    right_cam.release()
                if left_writer is not None:
                    left_writer.release()
                if right_writer is not None:
                    right_writer.release()

            if left_annot_writer is not None:
                left_annot_writer.release()
            if right_annot_writer is not None:
                right_annot_writer.release()

    logging.info("Finished run.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
