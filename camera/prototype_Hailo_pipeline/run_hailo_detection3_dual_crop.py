#!/usr/bin/env python3
# Prompt summary:
# - New script, do not modify existing files
# - Use Hailo HEF object detection on Raspberry Pi camera at 1920x1080
# - Improve small-object detection by alternating between two cropped regions
# - Remove bottom ~25% of frame, use center-left and center-right square-ish crops
# - Alternate crops every frame instead of running inference twice per frame
# - Map detections back to original full-frame coordinates
# - Draw detections on the full frame, optionally show crop rectangles
#
# Linux examples:
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection3_dual_crop.py --input rpi --hef-path yolov8s
#
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection3_dual_crop.py --input rpi --hef-path yolov8m --debug-crops
#
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection3_dual_crop.py --input /home/mario/Documents/collision-avoidance-VOP/camera/recordings/Ball_duck_water/IMG_4478.mov --hef-path yolov11s --save-output

from __future__ import annotations

import argparse
import collections
import os
import queue
import sys
import threading
import time
from functools import partial
from pathlib import Path

import cv2
import numpy as np


def _bootstrap_hailo_imports():
    repo_candidates = []
    current = Path(__file__).resolve()
    repo_candidates.extend(current.parents)
    repo_candidates.append(Path.home() / "Documents" / "hailo-apps")

    repo_root = None
    for candidate in repo_candidates:
        if (candidate / "hailo_apps" / "config" / "config_manager.py").exists():
            repo_root = candidate
            break

    if repo_root is None:
        raise SystemExit(
            "Could not locate hailo-apps. Place it under ~/Documents/hailo-apps "
            "or run this script from a workspace where the repo is discoverable."
        )

    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    from hailo_apps.python.core.common.core import handle_and_resolve_args
    from hailo_apps.python.core.common.defines import (
        IMAGE_EXTENSIONS,
        MAX_ASYNC_INFER_JOBS,
        MAX_INPUT_QUEUE_SIZE,
        MAX_OUTPUT_QUEUE_SIZE,
    )
    from hailo_apps.python.core.common.hailo_inference import HailoInfer
    from hailo_apps.python.core.common.hailo_logger import (
        get_logger,
        init_logging,
        level_from_args,
    )
    from hailo_apps.python.core.common.parser import get_standalone_parser
    from hailo_apps.python.core.common.toolbox import (
        FrameRateTracker,
        default_preprocess,
        get_labels,
        is_stream_url,
        load_json_file,
        resize_frame_for_output,
    )
    from hailo_apps.python.standalone_apps.object_detection.object_detection_post_process import (
        draw_detections,
        extract_detections,
    )

    return {
        "repo_root": repo_root,
        "handle_and_resolve_args": handle_and_resolve_args,
        "IMAGE_EXTENSIONS": IMAGE_EXTENSIONS,
        "MAX_ASYNC_INFER_JOBS": MAX_ASYNC_INFER_JOBS,
        "MAX_INPUT_QUEUE_SIZE": MAX_INPUT_QUEUE_SIZE,
        "MAX_OUTPUT_QUEUE_SIZE": MAX_OUTPUT_QUEUE_SIZE,
        "HailoInfer": HailoInfer,
        "get_logger": get_logger,
        "init_logging": init_logging,
        "level_from_args": level_from_args,
        "get_standalone_parser": get_standalone_parser,
        "FrameRateTracker": FrameRateTracker,
        "default_preprocess": default_preprocess,
        "get_labels": get_labels,
        "is_stream_url": is_stream_url,
        "load_json_file": load_json_file,
        "resize_frame_for_output": resize_frame_for_output,
        "draw_detections": draw_detections,
        "extract_detections": extract_detections,
    }


HAILO = _bootstrap_hailo_imports()
logger = HAILO["get_logger"](__name__)
APP_NAME = "object_detection"
TARGET_LABEL_DEFAULT = "sports ball"
STALE_FRAMES_DEFAULT = 6


class PiCamera2CaptureAdapter:
    """Small OpenCV-like adapter for Picamera2."""

    def __init__(self, picam2):
        self.picam2 = picam2
        self._opened = True
        self._io_lock = threading.Lock()

    def isOpened(self):
        return self._opened

    def read(self):
        if not self._opened:
            return False, None
        with self._io_lock:
            if not self._opened:
                return False, None
            frame = self.picam2.capture_array()
        if frame is None:
            return False, None
        return True, frame

    def get(self, prop_id: int) -> float:
        if prop_id in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT):
            try:
                cfg = self.picam2.camera_configuration()
                size = cfg.get("main", {}).get("size", None)
                if size and len(size) == 2:
                    width, height = int(size[0]), int(size[1])
                    return float(width if prop_id == cv2.CAP_PROP_FRAME_WIDTH else height)
            except Exception:
                pass
            return 0.0
        if prop_id == cv2.CAP_PROP_FPS:
            return 30.0
        return 0.0

    def release(self):
        self._opened = False
        with self._io_lock:
            try:
                self.picam2.stop()
            except Exception:
                pass
            try:
                self.picam2.close()
            except Exception:
                pass


def parse_args():
    parser = HAILO["get_standalone_parser"]()
    parser.description = (
        "Run Hailo object detection on alternating left/right crops to improve "
        "small-object detection for sports ball on water."
    )
    parser.set_defaults(input="rpi", camera_resolution="fhd")
    parser.add_argument(
        "--debug-crops",
        action="store_true",
        help="Draw the two crop rectangles and highlight the active crop.",
    )
    parser.add_argument(
        "--target-label",
        default=TARGET_LABEL_DEFAULT,
        help=f"Only draw and keep detections for this class (default: {TARGET_LABEL_DEFAULT}).",
    )
    parser.add_argument(
        "--stale-frames",
        type=int,
        default=STALE_FRAMES_DEFAULT,
        help="How many frames a crop's previous detections remain visible while alternating.",
    )
    parser.add_argument(
        "--show-fps-overlay",
        action="store_true",
        help="Draw an FPS overlay on the output window.",
    )
    parser.add_argument(
        "--labels",
        "-l",
        type=str,
        default=None,
        help="Optional labels file path.",
    )
    return parser.parse_args()


def normalize_local_paths(args) -> None:
    hef_path = Path(str(args.hef_path)).expanduser()
    if hef_path.exists():
        args.hef_path = str(hef_path.resolve())

    input_path = Path(str(args.input)).expanduser()
    if str(args.input) not in ("rpi", "usb") and input_path.exists():
        args.input = str(input_path.resolve())

    if args.labels:
        labels_path = Path(args.labels).expanduser()
        if labels_path.exists():
            args.labels = str(labels_path.resolve())

    if args.output_dir:
        args.output_dir = str(Path(args.output_dir).expanduser().resolve())


def get_crops(frame: np.ndarray) -> list[dict]:
    """
    Define the two crop windows on the full RGB frame.

    We remove the bottom ~25% of the image first, then create two square-ish
    crops inside the remaining upper region. The crop coordinates are stored in
    full-frame pixel space so detection boxes can later be translated back.
    """
    frame_height, frame_width = frame.shape[:2]
    usable_height = int(round(frame_height * 0.75))
    crop_size = min(usable_height, frame_width)
    crop_y = 0

    left_center_x = int(round(frame_width * 0.33))
    right_center_x = int(round(frame_width * 0.67))

    left_x = max(0, min(left_center_x - crop_size // 2, frame_width - crop_size))
    right_x = max(0, min(right_center_x - crop_size // 2, frame_width - crop_size))

    crops = []
    for name, crop_x in (("left", left_x), ("right", right_x)):
        crop_frame = frame[crop_y : crop_y + crop_size, crop_x : crop_x + crop_size]
        crops.append(
            {
                "name": name,
                "x": crop_x,
                "y": crop_y,
                "width": crop_size,
                "height": crop_size,
                "frame": crop_frame,
            }
        )
    return crops


def select_crop(frame: np.ndarray, frame_index: int) -> dict:
    crops = get_crops(frame)
    return crops[frame_index % 2]


def map_boxes_to_full_frame(
    boxes: list[list[float]],
    crop_info: dict,
    full_frame_shape: tuple[int, int, int],
) -> list[list[float]]:
    """
    Map crop-local [xmin, ymin, xmax, ymax] boxes back to full-frame coordinates.

    Because inference runs on a crop, every detection must be shifted by the
    crop origin. We add the crop's x/y offset and clip to the original frame.
    """
    full_height, full_width = full_frame_shape[:2]
    mapped_boxes = []
    x_offset = float(crop_info["x"])
    y_offset = float(crop_info["y"])

    for xmin, ymin, xmax, ymax in boxes:
        mapped_boxes.append(
            [
                float(np.clip(xmin + x_offset, 0, full_width - 1)),
                float(np.clip(ymin + y_offset, 0, full_height - 1)),
                float(np.clip(xmax + x_offset, 0, full_width - 1)),
                float(np.clip(ymax + y_offset, 0, full_height - 1)),
            ]
        )
    return mapped_boxes


def filter_and_map_detections(
    detections: dict,
    crop_info: dict,
    full_frame_shape: tuple[int, int, int],
    labels: list[str],
    target_label: str,
) -> dict:
    target_label = target_label.lower()
    mapped_boxes = []
    mapped_scores = []
    mapped_classes = []

    boxes = detections["detection_boxes"]
    scores = detections["detection_scores"]
    classes = detections["detection_classes"]

    full_boxes = map_boxes_to_full_frame(boxes, crop_info, full_frame_shape)

    for box, score, class_id in zip(full_boxes, scores, classes):
        if class_id < 0 or class_id >= len(labels):
            continue
        if labels[class_id].lower() != target_label:
            continue
        mapped_boxes.append(box)
        mapped_scores.append(float(score))
        mapped_classes.append(int(class_id))

    return {
        "detection_boxes": mapped_boxes,
        "detection_scores": mapped_scores,
        "detection_classes": mapped_classes,
        "num_detections": len(mapped_boxes),
    }


def merge_recent_detections(
    cache: dict[str, dict],
    current_frame_index: int,
    stale_frames: int,
) -> dict:
    merged_boxes = []
    merged_scores = []
    merged_classes = []

    for item in cache.values():
        age = current_frame_index - item["frame_index"]
        if age > stale_frames:
            continue
        detections = item["detections"]
        merged_boxes.extend(detections["detection_boxes"])
        merged_scores.extend(detections["detection_scores"])
        merged_classes.extend(detections["detection_classes"])

    return {
        "detection_boxes": merged_boxes,
        "detection_scores": merged_scores,
        "detection_classes": merged_classes,
        "num_detections": len(merged_boxes),
    }


def draw_crop_debug(
    frame_rgb: np.ndarray,
    crops: list[dict],
    active_crop_name: str,
) -> None:
    for crop in crops:
        x1 = int(crop["x"])
        y1 = int(crop["y"])
        x2 = x1 + int(crop["width"])
        y2 = y1 + int(crop["height"])
        color = (255, 255, 0) if crop["name"] == active_crop_name else (0, 255, 255)
        cv2.rectangle(frame_rgb, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            frame_rgb,
            crop["name"],
            (x1 + 8, y1 + 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
            cv2.LINE_AA,
        )


def open_rpi_camera():
    try:
        from picamera2 import Picamera2
    except Exception as exc:
        raise SystemExit(f"Picamera2 is not available: {exc}") from exc

    picam2 = Picamera2()
    main = {"size": (1920, 1080), "format": "BGR888"}
    config = picam2.create_video_configuration(main=main, controls={"FrameRate": 30})
    picam2.configure(config)
    picam2.start()
    return PiCamera2CaptureAdapter(picam2)


def init_input_source_dual_crop(input_src: str):
    src = input_src.strip()

    if src == "rpi":
        return open_rpi_camera(), None, "rpi"

    if src == "usb":
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        if not cap.isOpened():
            raise SystemExit("Failed to open USB camera 0.")
        return cap, None, "usb"

    if src.startswith("/dev/video"):
        cap = cv2.VideoCapture(src)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        if not cap.isOpened():
            raise SystemExit(f"Failed to open camera device: {src}")
        return cap, None, "usb"

    if HAILO["is_stream_url"](src):
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            raise SystemExit(f"Failed to open stream URL: {src}")
        return cap, None, "stream"

    path = Path(src)
    if not path.exists():
        raise SystemExit(
            f"Input '{src}' does not exist. Provide a full file path, directory path, "
            "or a camera source: 'usb' / 'rpi'."
        )

    if path.is_file():
        if path.suffix.lower() in HAILO["IMAGE_EXTENSIONS"]:
            image_bgr = cv2.imread(str(path))
            if image_bgr is None:
                raise SystemExit(f"Failed to read image file: {path}")
            return None, [cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)], "image"

        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened():
            raise SystemExit(f"Failed to open video file: {path}")
        return cap, None, "video"

    image_paths = sorted(
        p for p in path.iterdir() if p.is_file() and p.suffix.lower() in HAILO["IMAGE_EXTENSIONS"]
    )
    if not image_paths:
        raise SystemExit(f"No valid images found in the directory: {path}")

    images_rgb = []
    for image_path in image_paths:
        image_bgr = cv2.imread(str(image_path))
        if image_bgr is not None:
            images_rgb.append(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))
    if not images_rgb:
        raise SystemExit(f"Failed to read any valid images from: {path}")
    return None, images_rgb, "images"


def dual_crop_preprocess_loop(
    images,
    cap,
    input_queue: queue.Queue,
    model_width: int,
    model_height: int,
    preprocess_fn,
    stop_event: threading.Event,
):
    frame_index = 0

    def should_stop() -> bool:
        return stop_event.is_set()

    if images is not None:
        for image_rgb in images:
            if should_stop():
                break
            crop_info = select_crop(image_rgb, frame_index)
            payload = {
                "full_frame": image_rgb,
                "crop_info": crop_info,
                "frame_index": frame_index,
            }
            preprocessed = preprocess_fn(crop_info["frame"], model_width, model_height)
            input_queue.put(([payload], [preprocessed]))
            frame_index += 1
        input_queue.put(None)
        return

    while not should_stop():
        ok, frame_bgr = cap.read()
        if not ok:
            break

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        crop_info = select_crop(frame_rgb, frame_index)
        payload = {
            "full_frame": frame_rgb,
            "crop_info": crop_info,
            "frame_index": frame_index,
        }
        preprocessed = preprocess_fn(crop_info["frame"], model_width, model_height)
        input_queue.put(([payload], [preprocessed]))
        frame_index += 1

    input_queue.put(None)


def infer_loop(hailo_inference, input_queue: queue.Queue, output_queue: queue.Queue, stop_event: threading.Event):
    pending_jobs = collections.deque()

    while True:
        next_batch = input_queue.get()
        if not next_batch:
            break

        if stop_event.is_set():
            continue

        input_batch, preprocessed_batch = next_batch
        inference_callback_fn = partial(
            inference_callback,
            input_batch=input_batch,
            output_queue=output_queue,
        )

        while len(pending_jobs) >= HAILO["MAX_ASYNC_INFER_JOBS"]:
            pending_jobs.popleft().wait(10000)

        job = hailo_inference.run(preprocessed_batch, inference_callback_fn)
        pending_jobs.append(job)

    hailo_inference.close()
    output_queue.put(None)


def inference_callback(completion_info, bindings_list: list, input_batch: list, output_queue: queue.Queue) -> None:
    if completion_info.exception:
        logger.error(f"Inference error: {completion_info.exception}")
        return

    for i, bindings in enumerate(bindings_list):
        if len(bindings._output_names) == 1:
            result = bindings.output().get_buffer()
        else:
            result = {
                name: np.expand_dims(bindings.output(name).get_buffer(), axis=0)
                for name in bindings._output_names
            }

        payload = input_batch[i]
        output_queue.put(
            (
                payload["full_frame"],
                result,
                {
                    "crop_info": payload["crop_info"],
                    "frame_index": payload["frame_index"],
                },
            )
        )


def visualize_dual_crop(
    output_queue: queue.Queue,
    cap,
    save_stream_output: bool,
    output_dir: str,
    labels: list[str],
    config_data: dict,
    target_label: str,
    debug_crops: bool,
    show_fps_overlay: bool,
    output_resolution,
    stale_frames: int,
    fps_tracker,
    stop_event: threading.Event,
):
    image_id = 0
    writer = None
    frame_cache: dict[str, dict] = {}
    frame_width = None
    frame_height = None

    if cap is not None:
        cv2.namedWindow("Output", cv2.WINDOW_NORMAL)
        base_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 1920)
        base_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 1080)
        target_size = output_resolution if output_resolution is not None else (base_width, base_height)
        frame_width, frame_height = target_size

        if save_stream_output:
            os.makedirs(output_dir, exist_ok=True)
            out_path = os.path.join(output_dir, "output.avi")
            writer = cv2.VideoWriter(
                out_path,
                cv2.VideoWriter_fourcc(*"XVID"),
                20.0,
                (frame_width, frame_height),
            )

    while True:
        result = output_queue.get()
        if result is None:
            break

        full_frame, infer_result, metadata = result
        crop_info = metadata["crop_info"]
        frame_index = metadata["frame_index"]

        crop_detections = HAILO["extract_detections"](
            crop_info["frame"], infer_result, config_data
        )
        mapped_current = filter_and_map_detections(
            crop_detections,
            crop_info,
            full_frame.shape,
            labels,
            target_label,
        )
        frame_cache[crop_info["name"]] = {
            "frame_index": frame_index,
            "detections": mapped_current,
        }

        merged_detections = merge_recent_detections(
            frame_cache, frame_index, stale_frames
        )

        frame_with_detections = HAILO["draw_detections"](
            merged_detections,
            full_frame.copy(),
            labels,
            tracker=None,
            draw_trail=False,
        )

        if debug_crops:
            draw_crop_debug(
                frame_with_detections,
                get_crops(full_frame),
                crop_info["name"],
            )

        if fps_tracker is not None:
            fps_tracker.increment()
            if show_fps_overlay:
                fps_text = fps_tracker.frame_rate_summary()
                cv2.putText(
                    frame_with_detections,
                    fps_text,
                    (12, 36),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        bgr_frame = cv2.cvtColor(frame_with_detections, cv2.COLOR_RGB2BGR)
        frame_to_show = HAILO["resize_frame_for_output"](bgr_frame, output_resolution)

        if cap is not None:
            cv2.imshow("Output", frame_to_show)
            if writer is not None:
                writer.write(cv2.resize(frame_to_show, (frame_width, frame_height)))
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                stop_event.set()
                break
        else:
            os.makedirs(output_dir, exist_ok=True)
            cv2.imwrite(os.path.join(output_dir, f"output_{image_id}.png"), frame_to_show)

        image_id += 1

    if writer is not None:
        writer.release()
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


def run_inference_pipeline_dual_crop(args) -> None:
    labels = HAILO["get_labels"](args.labels)
    config_path = (
        HAILO["repo_root"]
        / "hailo_apps"
        / "python"
        / "standalone_apps"
        / "object_detection"
        / "config.json"
    )
    config_data = HAILO["load_json_file"](str(config_path))
    cap, images, _ = init_input_source_dual_crop(args.input)

    stop_event = threading.Event()
    input_queue = queue.Queue(HAILO["MAX_INPUT_QUEUE_SIZE"])
    output_queue = queue.Queue(HAILO["MAX_OUTPUT_QUEUE_SIZE"])
    fps_tracker = HAILO["FrameRateTracker"]() if args.show_fps else None

    hailo_inference = HAILO["HailoInfer"](args.hef_path, args.batch_size)
    model_height, model_width, _ = hailo_inference.get_input_shape()

    preprocess_thread = threading.Thread(
        target=dual_crop_preprocess_loop,
        args=(
            images,
            cap,
            input_queue,
            model_width,
            model_height,
            HAILO["default_preprocess"],
            stop_event,
        ),
    )
    infer_thread = threading.Thread(
        target=infer_loop,
        args=(hailo_inference, input_queue, output_queue, stop_event),
    )
    visualize_thread = threading.Thread(
        target=visualize_dual_crop,
        args=(
            output_queue,
            cap,
            args.save_output,
            args.output_dir,
            labels,
            config_data,
            args.target_label,
            args.debug_crops,
            args.show_fps_overlay,
            args.output_resolution,
            args.stale_frames,
            fps_tracker,
            stop_event,
        ),
    )

    preprocess_thread.start()
    infer_thread.start()
    visualize_thread.start()

    if fps_tracker is not None:
        fps_tracker.start()

    preprocess_thread.join()
    infer_thread.join()
    visualize_thread.join()

    if fps_tracker is not None:
        logger.info(fps_tracker.frame_rate_summary())

    logger.success("Dual-crop inference was successful!")
    if args.save_output or str(args.input).lower() not in ("usb", "rpi"):
        logger.info(f"Results have been saved in {args.output_dir}")


def main() -> None:
    args = parse_args()
    normalize_local_paths(args)

    HAILO["init_logging"](level=HAILO["level_from_args"](args))
    HAILO["handle_and_resolve_args"](args, APP_NAME)
    run_inference_pipeline_dual_crop(args)


if __name__ == "__main__":
    main()
