import cv2
import time
import os

try:
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    Picamera2 = None
    HAS_PICAMERA2 = False

# Settings: 
DURATION = 60  # seconds
OUTPUT_DIR = "./recordings"
FPS = 30
RESOLUTION = (1280, 720)


os.makedirs(OUTPUT_DIR, exist_ok=True)


def _init_cameras():
    if HAS_PICAMERA2:
        cam0 = Picamera2(0)
        cam1 = Picamera2(1)

        config0 = cam0.create_video_configuration(main={"size": RESOLUTION})
        config1 = cam1.create_video_configuration(main={"size": RESOLUTION})

        cam0.configure(config0)
        cam1.configure(config1)

        cam0.start()
        cam1.start()

        return cam0, cam1, "picamera2"

    print("picamera2 not found. Falling back to OpenCV VideoCapture.")

    cam0 = cv2.VideoCapture(0)
    cam1 = cv2.VideoCapture(1)

    for cam in (cam0, cam1):
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
        cam.set(cv2.CAP_PROP_FPS, FPS)

    if not cam0.isOpened() or not cam1.isOpened():
        raise RuntimeError(
            "Unable to open both cameras with OpenCV. "
            "Use Raspberry Pi OS + python3-picamera2 for CSI cameras, "
            "or check that camera indices 0 and 1 exist for USB cameras."
        )

    return cam0, cam1, "opencv"


cam0, cam1, backend = _init_cameras()

# Video writers
fourcc = cv2.VideoWriter_fourcc(*"mp4v")

video0_path = os.path.join(OUTPUT_DIR, "camera0.mp4")
video1_path = os.path.join(OUTPUT_DIR, "camera1.mp4")

writer0 = cv2.VideoWriter(video0_path, fourcc, FPS, RESOLUTION)
writer1 = cv2.VideoWriter(video1_path, fourcc, FPS, RESOLUTION)

start_time = time.time()

print("Recording started...")

while time.time() - start_time < DURATION:

    if backend == "picamera2":
        frame0 = cam0.capture_array()
        frame1 = cam1.capture_array()

        frame0 = cv2.cvtColor(frame0, cv2.COLOR_RGB2BGR)
        frame1 = cv2.cvtColor(frame1, cv2.COLOR_RGB2BGR)
    else:
        ok0, frame0 = cam0.read()
        ok1, frame1 = cam1.read()
        if not ok0 or not ok1:
            print("Frame capture failed, stopping early.")
            break

    writer0.write(frame0)
    writer1.write(frame1)

# Cleanup
writer0.release()
writer1.release()

if backend == "picamera2":
    cam0.stop()
    cam1.stop()
else:
    cam0.release()
    cam1.release()

print("Recording finished.")
print("Saved to:", OUTPUT_DIR)