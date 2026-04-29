#!/usr/bin/env python3
# VERGEET NIET DE source setup... vanuit hailo als venv te draaien!!!!!
# Linux examples:
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection2.py --input rpi
#
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection2.py --hailo-apps-root ~/Documents/hailo-apps --network yolov8s --input /dev/video0
#
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection2.py --input rpi --save-output

"""Run Hailo's official object detection app and log sports ball detections.

This wrapper keeps the official Hailo object detection application as the
runtime engine. It creates a new run folder under camera/recordings, prepares a
single-camera output directory, and injects a small runtime hook that logs only
"sports ball" detections to a JSONL file.
"""

import argparse
import importlib
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import yaml


def build_parser() -> argparse.ArgumentParser:
    """Create the CLI parser for the wrapper script."""
    current_file = Path(__file__).resolve()
    camera_dir = current_file.parent
    default_hailo_root = Path.home() / "Documents" / "hailo-apps"

    parser = argparse.ArgumentParser(
        description="Run Hailo's official object detection app from this project."
    )
    parser.add_argument(
        "--hailo-apps-root",
        type=Path,
        default=default_hailo_root,
        help=(
            "Path to the hailo-apps repository root "
            f"(default: {default_hailo_root})"
        ),
    )
    parser.add_argument(
        "--network",
        default="yolov8s",
        help="Hailo network/model name to pass to object_detection.py.",
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
        "--log-label",
        default="sports ball",
        help="Object label to log when detected (default: sports ball).",
    )
    parser.add_argument(
        "--calib",
        type=Path,
        default=camera_dir / "calibration_yamls" / "camera_calibration.yaml",
        help="Path to the camera calibration YAML used for realtime rectification.",
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

    parser.epilog = (
        "Example:\n"
        f"  {camera_dir / 'run_hailo_detection2.py'} --input rpi\n"
        f"  {camera_dir / 'run_hailo_detection2.py'} "
        "--hailo-apps-root ~/Documents/hailo-apps --network yolov8s --input /dev/video0"
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
    hailo_apps_root: Path, object_detection_dir: Path, script_path: Path
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


def build_undistort_maps(
    image_size: tuple[int, int],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    alpha: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    width, height = image_size
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
        camera_matrix,
        distortion,
        (width, height),
        alpha,
        (width, height),
    )
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix,
        distortion,
        R=None,
        newCameraMatrix=new_camera_matrix,
        size=(width, height),
        m1type=cv2.CV_16SC2,
    )
    return map1, map2


def rectify_rgb_frame(
    frame_rgb: np.ndarray,
    calib_image_size: tuple[int, int],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    cache: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]],
) -> np.ndarray:
    frame_h, frame_w = frame_rgb.shape[:2]
    frame_size = (frame_w, frame_h)
    if frame_size not in cache:
        scaled_camera_matrix = scale_camera_matrix(
            camera_matrix=camera_matrix,
            calib_image_size=calib_image_size,
            frame_size=frame_size,
        )
        cache[frame_size] = build_undistort_maps(
            image_size=frame_size,
            camera_matrix=scaled_camera_matrix,
            distortion=distortion,
        )
    map1, map2 = cache[frame_size]
    return cv2.remap(
        frame_rgb,
        map1,
        map2,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
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
    network: str,
    input_source: str,
    hailo_apps_root: Path,
    calib_path: Path,
    log_label: str,
) -> None:
    lines = [
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Run folder: {run_dir}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Camera output dir: {camera_output_dir}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Hailo apps root: {hailo_apps_root}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Calibration YAML: {calib_path}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Network: {network}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Input: {input_source}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Logging label: {log_label}",
    ]
    mission_log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


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
        args.network,
        "--input",
        args.input,
        "--log-label",
        args.log_label,
        "--calib",
        str(args.calib.expanduser().resolve()),
        "--recording-run-dir",
        str(run_dir),
        "--camera-name",
        camera_name,
    ]
    if args.save_output:
        command.append("--save-output")
    return command


def iso_timestamp_utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z"
    )


def append_detection_log(log_path: Path, camera_name: str, label: str) -> None:
    payload = {
        "timestamp_utc": iso_timestamp_utc(),
        "detections": [{"camera": camera_name, "label": label}],
    }
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload) + "\n")


def install_rectified_input_hooks(hailo_apps_root: Path, calib_path: Path) -> None:
    sys.path.insert(0, str(hailo_apps_root))
    toolbox_module = importlib.import_module("hailo_apps.python.core.common.toolbox")
    calib_image_size, camera_matrix, distortion = load_calib_yaml(calib_path)
    map_cache: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]] = {}

    def rectify_for_pipeline(frame_rgb: np.ndarray) -> np.ndarray:
        return rectify_rgb_frame(
            frame_rgb=frame_rgb,
            calib_image_size=calib_image_size,
            camera_matrix=camera_matrix,
            distortion=distortion,
            cache=map_cache,
        )

    original_init_input_source = toolbox_module.init_input_source

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

    def patched_preprocess_images(images, batch_size, input_queue, width, height, preprocess_fn):
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
            # Keep the same camera pixel format as Hailo's original Pi path.
            # The rest of Hailo's pipeline already assumes this behavior.
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


def install_detection_logger(
    hailo_apps_root: Path,
    log_path: Path,
    log_label: str,
    camera_name: str,
):
    sys.path.insert(0, str(hailo_apps_root))
    post_process_module = importlib.import_module(
        "hailo_apps.python.standalone_apps.object_detection.object_detection_post_process"
    )
    original_handler = post_process_module.inference_result_handler
    extract_detections = post_process_module.extract_detections
    target_label = log_label.lower()

    def patched_handler(
        original_frame, infer_results, labels, config_data, tracker=None, draw_trail=False
    ):
        detections = extract_detections(original_frame, infer_results, config_data)
        classes = detections["detection_classes"]
        if any(labels[class_id].lower() == target_label for class_id in classes):
            append_detection_log(log_path, camera_name, log_label)
        return original_handler(
            original_frame,
            infer_results,
            labels,
            config_data,
            tracker=tracker,
            draw_trail=draw_trail,
        )

    post_process_module.inference_result_handler = patched_handler


def run_internal_hailo(args: argparse.Namespace) -> int:
    if args.recording_run_dir is None:
        print("Error: internal run is missing --recording-run-dir.", file=sys.stderr)
        return 1
    if not args.camera_name:
        print("Error: internal run is missing --camera-name.", file=sys.stderr)
        return 1

    object_detection_dir, object_detection_script = resolve_hailo_paths(
        args.hailo_apps_root
    )
    try:
        validate_paths(args.hailo_apps_root, object_detection_dir, object_detection_script)
    except (FileNotFoundError, NotADirectoryError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    run_dir = args.recording_run_dir.expanduser().resolve()
    camera_output_dir = run_dir / "camera0_recording"
    detections_log_path = run_dir / "detections.jsonl"
    calib_path = args.calib.expanduser().resolve()
    if not calib_path.exists():
        print(f"Error: calibration YAML was not found: {calib_path}", file=sys.stderr)
        return 1

    os.chdir(object_detection_dir)
    install_rectified_input_hooks(
        hailo_apps_root=args.hailo_apps_root.expanduser().resolve(),
        calib_path=calib_path,
    )
    install_detection_logger(
        hailo_apps_root=args.hailo_apps_root.expanduser().resolve(),
        log_path=detections_log_path,
        log_label=args.log_label,
        camera_name=args.camera_name,
    )

    object_detection_module = importlib.import_module(
        "hailo_apps.python.standalone_apps.object_detection.object_detection"
    )
    sys.argv = [
        str(object_detection_script),
        "-n",
        args.network,
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

    script_path = Path(__file__).resolve()
    camera_dir = script_path.parent
    recordings_dir = camera_dir / "recordings"
    object_detection_dir, object_detection_script = resolve_hailo_paths(
        args.hailo_apps_root
    )
    try:
        validate_paths(args.hailo_apps_root, object_detection_dir, object_detection_script)
    except (FileNotFoundError, NotADirectoryError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    run_dir = create_run_directory(recordings_dir)
    camera_output_dir = run_dir / "camera0_recording"
    camera_output_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "detections.jsonl").touch()

    calib_path = args.calib.expanduser().resolve()
    if not calib_path.exists():
        print(f"Error: calibration YAML was not found: {calib_path}", file=sys.stderr)
        return 1

    camera_name = detect_camera_name(args.input)
    write_mission_log(
        mission_log_path=run_dir / "mission.log",
        run_dir=run_dir,
        camera_output_dir=camera_output_dir,
        network=args.network,
        input_source=args.input,
        hailo_apps_root=args.hailo_apps_root.expanduser().resolve(),
        calib_path=calib_path,
        log_label=args.log_label,
    )

    command = build_launcher_command(
        script_path=script_path,
        args=args,
        run_dir=run_dir,
        camera_name=camera_name,
    )

    print(f"Camera directory: {camera_dir}")
    print(f"Hailo apps root: {args.hailo_apps_root.expanduser().resolve()}")
    print(f"Calibration YAML: {calib_path}")
    print(f"Working directory: {object_detection_dir}")
    print(f"Run directory: {run_dir}")
    print(f"Camera output directory: {camera_output_dir}")
    print(f"Detections log: {run_dir / 'detections.jsonl'}")
    print(f"Command: {' '.join(command)}")

    completed = subprocess.run(command, cwd=object_detection_dir, check=False)
    return completed.returncode


if __name__ == "__main__":
    sys.exit(main())
