#!/usr/bin/env python3
# VERGEET NIET DE source setup... vanuit hailo als venv te draaien!!!!!
# Linux examples:
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection.py --input rpi
#
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection.py --hailo-apps-root ~/Documents/hailo-apps --network yolov8s --input /dev/video0
#
# cd ~/Documents/collision-avoidance-VOP/camera
# python3 run_hailo_detection.py --input rpi --save-output

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

    # Internal-only flags for the re-executed subprocess that imports Hailo.
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
        f"  {camera_dir / 'run_hailo_detection.py'} --input rpi\n"
        f"  {camera_dir / 'run_hailo_detection.py'} "
        "--hailo-apps-root ~/Documents/hailo-apps --network yolov8s --input /dev/video0"
    )
    return parser


def resolve_hailo_paths(hailo_apps_root: Path) -> tuple[Path, Path]:
    """Return the Hailo object detection directory and script path."""
    resolved_root = hailo_apps_root.expanduser().resolve()
    object_detection_dir = (
        resolved_root / "hailo_apps" / "python" / "standalone_apps" / "object_detection"
    )
    object_detection_script = object_detection_dir / "object_detection.py"
    return object_detection_dir, object_detection_script


def validate_paths(
    hailo_apps_root: Path, object_detection_dir: Path, script_path: Path
) -> None:
    """Raise a clear error when required paths are missing."""
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
    """Map the requested input source to a simple camera identifier."""
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


def create_run_directory(recordings_dir: Path) -> Path:
    """Create a timestamped run folder under camera/recordings."""
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
    log_label: str,
) -> None:
    """Write a small run summary similar to the existing recordings layout."""
    lines = [
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Run folder: {run_dir}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Camera output dir: {camera_output_dir}",
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | INFO | Hailo apps root: {hailo_apps_root}",
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
    """Build the subprocess command that launches the internal Hailo runner."""
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
        "--recording-run-dir",
        str(run_dir),
        "--camera-name",
        camera_name,
    ]
    if args.save_output:
        command.append("--save-output")
    return command


def iso_timestamp_utc() -> str:
    """Return a compact UTC timestamp with millisecond precision."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z"
    )


def append_detection_log(log_path: Path, camera_name: str, label: str) -> None:
    """Append one JSONL record for a matching detection event."""
    payload = {
        "timestamp_utc": iso_timestamp_utc(),
        "detections": [
            {
                "camera": camera_name,
                "label": label,
            }
        ],
    }
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload) + "\n")


def install_detection_logger(
    hailo_apps_root: Path,
    log_path: Path,
    log_label: str,
    camera_name: str,
):
    """Monkeypatch Hailo's post-process handler to log sports ball detections."""
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
    """Import and run the official Hailo object detection app with logging hook."""
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

    os.chdir(object_detection_dir)
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
    ]
    if args.save_output:
        sys.argv.append("--save-output")

    object_detection_module.main()
    return 0


def main() -> int:
    """Parse CLI args, validate paths, and run the Hailo app."""
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

    camera_name = detect_camera_name(args.input)
    write_mission_log(
        mission_log_path=run_dir / "mission.log",
        run_dir=run_dir,
        camera_output_dir=camera_output_dir,
        network=args.network,
        input_source=args.input,
        hailo_apps_root=args.hailo_apps_root.expanduser().resolve(),
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
    print(f"Working directory: {object_detection_dir}")
    print(f"Run directory: {run_dir}")
    print(f"Camera output directory: {camera_output_dir}")
    print(f"Detections log: {run_dir / 'detections.jsonl'}")
    print(f"Command: {' '.join(command)}")

    completed = subprocess.run(command, cwd=object_detection_dir, check=False)
    return completed.returncode


if __name__ == "__main__":
    sys.exit(main())
