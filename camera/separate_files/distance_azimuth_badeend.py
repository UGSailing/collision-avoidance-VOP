#!/usr/bin/env python3
"""Estimate badeend distance and azimuth from a single camera using duck.pt."""
# rpi camera:
#python .\collision-avoidance-VOP\camera\distance_azimuth_badeend.py --picamera --object-height 0.10
#webcam:
# python .\collision-avoidance-VOP\camera\distance_azimuth_badeend.py --webcam --object-height 0.10
# single image:
# python .\collision-avoidance-VOP\camera\distance_azimuth_badeend.py --image ".\SailToDuck\1.training\Rubber Duck Detection.v1i.yolov8\train\images\image_10_jpg.rf.8e5cd11b4e1ed5c9f685fb4fe9aa65b8.jpg" --object-height 0.10


from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - handled at runtime
    YOLO = None

try:
    from picamera2 import Picamera2
except ImportError:  # pragma: no cover - handled at runtime
    Picamera2 = None


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent
DEFAULT_MODEL_PATH = REPO_ROOT / "SailToDuck" / "3.cpu_detection" / "duck.pt"
DEFAULT_CALIB_PATHS = (
    SCRIPT_DIR / "calib_cam0.yaml",
    SCRIPT_DIR / "calibration_yamls" / "camera_calibration.yaml",
)
DISPLAY_WINDOW = "Duck Distance + Azimuth"
_WARNED_ASPECT_RATIOS: set[tuple[tuple[int, int] | None, tuple[int, int]]] = set()


@dataclass(frozen=True)
class CalibrationData:
    image_size: tuple[int, int] | None
    K: np.ndarray
    dist: np.ndarray


@dataclass(frozen=True)
class DetectionEstimate:
    class_name: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]
    distance_m: float
    azimuth_rad: float
    azimuth_deg: float
    height_px: float
    center_u: float
    global_x: float | None = None
    global_y: float | None = None


def resolve_default_calib_path() -> Path:
    for path in DEFAULT_CALIB_PATHS:
        if path.exists():
            return path
    return DEFAULT_CALIB_PATHS[0]


def load_calib_yaml(path: str | Path) -> CalibrationData:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    image_size_raw = data.get("image_size")
    image_size = None
    if image_size_raw is not None and len(image_size_raw) == 2:
        image_size = (int(image_size_raw[0]), int(image_size_raw[1]))

    K = np.array(data["K"], dtype=np.float64)
    dist = np.array(data["dist"], dtype=np.float64).reshape(-1, 1)
    return CalibrationData(image_size=image_size, K=K, dist=dist)


def scaled_camera_matrix(
    K: np.ndarray,
    calib_image_size: tuple[int, int] | None,
    frame_size: tuple[int, int],
) -> np.ndarray:
    if calib_image_size is None:
        return K.copy()

    calib_w, calib_h = calib_image_size
    frame_w, frame_h = frame_size

    scale_x = frame_w / calib_w
    scale_y = frame_h / calib_h

    scaled = K.copy()
    scaled[0, 0] *= scale_x
    scaled[0, 2] *= scale_x
    scaled[1, 1] *= scale_y
    scaled[1, 2] *= scale_y
    return scaled


def warn_if_aspect_ratio_mismatch(
    calib_image_size: tuple[int, int] | None,
    frame_size: tuple[int, int],
) -> None:
    if calib_image_size is None:
        return

    calib_w, calib_h = calib_image_size
    frame_w, frame_h = frame_size
    calib_ratio = calib_w / calib_h
    frame_ratio = frame_w / frame_h
    warn_key = (calib_image_size, frame_size)
    if abs(calib_ratio - frame_ratio) > 0.02 and warn_key not in _WARNED_ASPECT_RATIOS:
        _WARNED_ASPECT_RATIOS.add(warn_key)
        print(
            "WARNING: calibration aspect ratio "
            f"{calib_w}x{calib_h} differs from frame size {frame_w}x{frame_h}. "
            "Distance and azimuth assume the frame is only resized from the calibrated view."
        )


def undistort_points(pts: np.ndarray, K: np.ndarray, dist: np.ndarray) -> np.ndarray:
    und = cv2.undistortPoints(pts.astype(np.float32), K, dist, P=K)
    return und.reshape(-1, 2)


def estimate_from_bbox(
    bbox_xyxy: tuple[float, float, float, float],
    object_height_m: float,
    K: np.ndarray,
    dist: np.ndarray,
    camera_x: float | None,
    camera_y: float | None,
    camera_heading_deg: float | None,
    class_name: str,
    confidence: float,
) -> DetectionEstimate:
    x1, y1, x2, y2 = (float(v) for v in bbox_xyxy)
    bbox_center_x = 0.5 * (x1 + x2)
    bbox_center_y = 0.5 * (y1 + y2)

    sample_points = np.array(
        [
            [[bbox_center_x, y1]],
            [[bbox_center_x, y2]],
            [[bbox_center_x, bbox_center_y]],
        ],
        dtype=np.float32,
    )
    top_pt, bottom_pt, center_pt = undistort_points(sample_points, K, dist)

    height_px = abs(float(bottom_pt[1] - top_pt[1]))
    if height_px < 1.0:
        raise ValueError("Detected object height is too small for a stable estimate.")

    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])

    distance_m = (fy * object_height_m) / height_px
    azimuth_rad = math.atan((float(center_pt[0]) - cx) / fx)
    azimuth_deg = math.degrees(azimuth_rad)

    global_x = None
    global_y = None
    if (
        camera_x is not None
        and camera_y is not None
        and camera_heading_deg is not None
    ):
        heading_rad = math.radians(camera_heading_deg) + azimuth_rad
        global_x = camera_x + distance_m * math.cos(heading_rad)
        global_y = camera_y + distance_m * math.sin(heading_rad)

    return DetectionEstimate(
        class_name=class_name,
        confidence=confidence,
        bbox_xyxy=(x1, y1, x2, y2),
        distance_m=float(distance_m),
        azimuth_rad=float(azimuth_rad),
        azimuth_deg=float(azimuth_deg),
        height_px=height_px,
        center_u=float(center_pt[0]),
        global_x=global_x,
        global_y=global_y,
    )


def build_model(model_path: Path):
    if YOLO is None:
        raise SystemExit(
            "Ultralytics is not installed. Install it with: pip install ultralytics"
        )
    if not model_path.exists():
        raise SystemExit(f"Model file not found: {model_path}")
    return YOLO(str(model_path), task="detect")


def default_live_size(calib: CalibrationData, frame_width: int) -> tuple[int, int]:
    if calib.image_size is None:
        return frame_width, 480
    calib_w, calib_h = calib.image_size
    frame_height = max(1, int(round(frame_width * calib_h / calib_w)))
    return frame_width, frame_height


def probe_display_available() -> bool:
    try:
        cv2.namedWindow(DISPLAY_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(DISPLAY_WINDOW, 640, 360)
        dummy = np.zeros((100, 100, 3), dtype=np.uint8)
        cv2.imshow(DISPLAY_WINDOW, dummy)
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        return True
    except Exception:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        return False


def resolve_class_name(labels, class_id: int) -> str:
    if isinstance(labels, dict):
        return str(labels.get(class_id, class_id))
    if isinstance(labels, list) and 0 <= class_id < len(labels):
        return str(labels[class_id])
    return str(class_id)


def iter_estimates(
    model,
    frame: np.ndarray,
    object_height_m: float,
    calib: CalibrationData,
    min_conf: float,
    class_name_filter: str | None,
    camera_x: float | None,
    camera_y: float | None,
    camera_heading_deg: float | None,
) -> list[DetectionEstimate]:
    frame_h, frame_w = frame.shape[:2]
    frame_size = (frame_w, frame_h)
    warn_if_aspect_ratio_mismatch(calib.image_size, frame_size)
    scaled_K = scaled_camera_matrix(calib.K, calib.image_size, frame_size)

    results = model(frame, verbose=False)
    boxes = results[0].boxes
    if boxes is None:
        return []

    labels = model.names
    estimates: list[DetectionEstimate] = []
    for box in boxes:
        confidence = float(box.conf[0].item())
        if confidence < min_conf:
            continue

        class_id = int(box.cls[0].item())
        class_name = resolve_class_name(labels, class_id)
        if class_name_filter and class_name != class_name_filter:
            continue

        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(float)
        try:
            estimates.append(
                estimate_from_bbox(
                    bbox_xyxy=(x1, y1, x2, y2),
                    object_height_m=object_height_m,
                    K=scaled_K,
                    dist=calib.dist,
                    camera_x=camera_x,
                    camera_y=camera_y,
                    camera_heading_deg=camera_heading_deg,
                    class_name=class_name,
                    confidence=confidence,
                )
            )
        except ValueError:
            continue

    estimates.sort(key=lambda item: item.confidence, reverse=True)
    return estimates


def draw_estimates(frame: np.ndarray, estimates: list[DetectionEstimate]) -> np.ndarray:
    vis = frame.copy()
    for estimate in estimates:
        x1, y1, x2, y2 = [int(round(v)) for v in estimate.bbox_xyxy]
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = (
            f"{estimate.class_name} {estimate.confidence:.0%} | "
            f"Z={estimate.distance_m:.2f}m | az={estimate.azimuth_deg:.2f}deg"
        )
        cv2.putText(
            vis,
            label,
            (x1, max(y1 - 10, 25)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return vis


def print_estimates(estimates: list[DetectionEstimate]) -> None:
    if not estimates:
        print("No duck detections above the requested confidence threshold.")
        return

    for idx, estimate in enumerate(estimates, start=1):
        print(f"Detection {idx}")
        print(f"  class: {estimate.class_name}")
        print(f"  confidence: {estimate.confidence:.3f}")
        print(f"  bbox_xyxy: {tuple(round(v, 2) for v in estimate.bbox_xyxy)}")
        print(f"  h_px (undistorted): {estimate.height_px:.2f}")
        print(f"  u_center (undistorted): {estimate.center_u:.2f}")
        print(f"  distance_m: {estimate.distance_m:.3f}")
        print(f"  azimuth_deg: {estimate.azimuth_deg:.3f}")
        if estimate.global_x is not None and estimate.global_y is not None:
            print(f"  global_xy: ({estimate.global_x:.3f}, {estimate.global_y:.3f})")


def run_image_mode(args: argparse.Namespace, model, calib: CalibrationData) -> None:
    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit(f"Could not read image: {args.image}")

    estimates = iter_estimates(
        model=model,
        frame=image,
        object_height_m=args.object_height,
        calib=calib,
        min_conf=args.conf,
        class_name_filter=args.class_name,
        camera_x=args.camera_x,
        camera_y=args.camera_y,
        camera_heading_deg=args.camera_heading_deg,
    )
    print_estimates(estimates)

    if args.show:
        vis = draw_estimates(image, estimates)
        cv2.imshow(DISPLAY_WINDOW, vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def open_webcam(frame_size: tuple[int, int]) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise SystemExit("Could not open webcam.")

    frame_w, frame_h = frame_size
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_h)
    return cap


def run_webcam_mode(args: argparse.Namespace, model, calib: CalibrationData) -> None:
    frame_size = (args.frame_width, args.frame_height)
    cap = open_webcam(frame_size)
    display_available = probe_display_available()
    frame_count = 0
    start_time = time.time()

    print(
        f"Starting webcam detection at {frame_size[0]}x{frame_size[1]}. "
        "Press 'q' to quit."
    )

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("WARNING: failed to read webcam frame.")
                break

            frame_count += 1
            estimates = iter_estimates(
                model=model,
                frame=frame,
                object_height_m=args.object_height,
                calib=calib,
                min_conf=args.conf,
                class_name_filter=args.class_name,
                camera_x=args.camera_x,
                camera_y=args.camera_y,
                camera_heading_deg=args.camera_heading_deg,
            )
            elapsed = max(time.time() - start_time, 1e-6)
            fps = frame_count / elapsed

            if display_available:
                vis = draw_estimates(frame, estimates)
                cv2.putText(
                    vis,
                    f"FPS: {fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(DISPLAY_WINDOW, vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            elif frame_count % args.log_every == 0:
                print(f"Frame {frame_count}:")
                print_estimates(estimates[:1])
    finally:
        cap.release()
        cv2.destroyAllWindows()


def run_picamera_mode(args: argparse.Namespace, model, calib: CalibrationData) -> None:
    if Picamera2 is None:
        raise SystemExit(
            "Picamera2 is not installed. Use --image or --webcam, or install picamera2."
        )

    frame_size = (args.frame_width, args.frame_height)
    camera = Picamera2()
    config = camera.create_video_configuration(
        main={"format": "RGB888", "size": frame_size}
    )
    camera.configure(config)
    camera.start()

    display_available = probe_display_available()
    frame_count = 0
    start_time = time.time()

    print(
        f"Starting Pi camera detection at {frame_size[0]}x{frame_size[1]}. "
        "Press 'q' to quit."
    )

    try:
        while True:
            frame = camera.capture_array()
            frame_count += 1
            estimates = iter_estimates(
                model=model,
                frame=frame,
                object_height_m=args.object_height,
                calib=calib,
                min_conf=args.conf,
                class_name_filter=args.class_name,
                camera_x=args.camera_x,
                camera_y=args.camera_y,
                camera_heading_deg=args.camera_heading_deg,
            )
            elapsed = max(time.time() - start_time, 1e-6)
            fps = frame_count / elapsed

            if display_available:
                vis = draw_estimates(frame, estimates)
                cv2.putText(
                    vis,
                    f"FPS: {fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(DISPLAY_WINDOW, cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            elif frame_count % args.log_every == 0:
                print(f"Frame {frame_count}:")
                print_estimates(estimates[:1])
    finally:
        camera.stop()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Detect a badeend with duck.pt and estimate distance/azimuth "
            "from the object's known real-world height."
        )
    )
    source_group = parser.add_mutually_exclusive_group()
    source_group.add_argument("--image", help="Run on a single image and exit")
    source_group.add_argument(
        "--webcam", action="store_true", help="Run live detection from the default webcam"
    )
    source_group.add_argument(
        "--picamera", action="store_true", help="Run live detection from Picamera2"
    )

    parser.add_argument(
        "--object-height",
        type=float,
        required=True,
        help="Real-world badeend height in meters",
    )
    parser.add_argument(
        "--model",
        default=str(DEFAULT_MODEL_PATH),
        help=f"Path to duck.pt model (default: {DEFAULT_MODEL_PATH})",
    )
    parser.add_argument(
        "--calib",
        default=str(resolve_default_calib_path()),
        help="Calibration YAML with K, dist, and preferably image_size",
    )
    parser.add_argument(
        "--frame-width",
        type=int,
        default=640,
        help="Requested live frame width. Height defaults from calibration aspect ratio.",
    )
    parser.add_argument(
        "--frame-height",
        type=int,
        default=None,
        help="Requested live frame height. Leave unset to preserve calibration aspect ratio.",
    )
    parser.add_argument(
        "--conf", type=float, default=0.5, help="Minimum detection confidence"
    )
    parser.add_argument(
        "--class-name",
        default=None,
        help="Optional class name filter, for example 'duck'",
    )
    parser.add_argument(
        "--camera-x",
        type=float,
        default=None,
        help="Optional camera global x-coordinate",
    )
    parser.add_argument(
        "--camera-y",
        type=float,
        default=None,
        help="Optional camera global y-coordinate",
    )
    parser.add_argument(
        "--camera-heading-deg",
        type=float,
        default=None,
        help="Optional camera heading in degrees for global XY output",
    )
    parser.add_argument(
        "--log-every",
        type=int,
        default=10,
        help="Headless live logging interval in frames",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show the annotated image in image mode",
    )
    args = parser.parse_args()

    calib = load_calib_yaml(args.calib)
    if args.frame_height is None:
        _, args.frame_height = default_live_size(calib, args.frame_width)
    args._loaded_calib = calib
    return args


def main() -> None:
    args = parse_args()
    calib: CalibrationData = args._loaded_calib
    model = build_model(Path(args.model))

    if args.image:
        run_image_mode(args, model, calib)
        return

    if args.webcam:
        run_webcam_mode(args, model, calib)
        return

    run_picamera_mode(args, model, calib)


if __name__ == "__main__":
    main()
