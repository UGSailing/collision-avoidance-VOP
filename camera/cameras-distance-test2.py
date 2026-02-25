import subprocess
import os
import sys

def list_cameras():
    """Check how many cameras are detected"""
    print("Listing cameras...")
    result = subprocess.run(
        ["rpicam-hello", "--list-cameras"],
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        print("Error listing cameras:")
        print(result.stderr)
        sys.exit(1)

    print(result.stdout)

    # Count detected cameras
    camera_lines = [
        line for line in result.stdout.splitlines()
        if line.strip().startswith(("0", "1", "2", "3"))
    ]

    return len(camera_lines)


def capture_photo(camera_index, filename):
    """Capture photo from given camera index"""
    print(f"Capturing photo from camera {camera_index}...")

    cmd = [
        "rpicam-still",
        "--camera", str(camera_index),
        "--output", filename,
        "--timeout", "2000"
    ]

    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode == 0 and os.path.exists(filename):
        size = os.path.getsize(filename)
        print(f"Saved {filename} ({size} bytes)")
    else:
        print(f"Failed to capture from camera {camera_index}")
        print(result.stderr)


def main():
    num_cameras = list_cameras()

    if num_cameras < 2:
        print("Less than 2 cameras detected. Aborting.")
        sys.exit(1)

    capture_photo(0, "testfoto_camera0.jpg")
    capture_photo(1, "testfoto_camera1.jpg")

    print("Done.")


if __name__ == "__main__":
    main()
