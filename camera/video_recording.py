from picamera2 import Picamera2
import cv2
import time
import os

# Settings: 
DURATION = 60  # seconds
OUTPUT_DIR = "./recordings"
FPS = 30
RESOLUTION = (1280, 720)


os.makedirs(OUTPUT_DIR, exist_ok=True)

# Initialize cameras
cam0 = Picamera2(0)
cam1 = Picamera2(1)

config0 = cam0.create_video_configuration(main={"size": RESOLUTION})
config1 = cam1.create_video_configuration(main={"size": RESOLUTION})

cam0.configure(config0)
cam1.configure(config1)

cam0.start()
cam1.start()

# Video writers
fourcc = cv2.VideoWriter_fourcc(*"mp4v")

video0_path = os.path.join(OUTPUT_DIR, "camera0.mp4")
video1_path = os.path.join(OUTPUT_DIR, "camera1.mp4")

writer0 = cv2.VideoWriter(video0_path, fourcc, FPS, RESOLUTION)
writer1 = cv2.VideoWriter(video1_path, fourcc, FPS, RESOLUTION)

start_time = time.time()

print("Recording started...")

while time.time() - start_time < DURATION:

    frame0 = cam0.capture_array()
    frame1 = cam1.capture_array()

    frame0 = cv2.cvtColor(frame0, cv2.COLOR_RGB2BGR)
    frame1 = cv2.cvtColor(frame1, cv2.COLOR_RGB2BGR)

    writer0.write(frame0)
    writer1.write(frame1)

# Cleanup
writer0.release()
writer1.release()

cam0.stop()
cam1.stop()

print("Recording finished.")
print("Saved to:", OUTPUT_DIR)