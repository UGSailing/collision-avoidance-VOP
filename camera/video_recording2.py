#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
from datetime import datetime


def start_recording(camera_id, output_path, width, height, fps, duration_sec, shutter_us, gain):
	timeout_ms = int(duration_sec * 1000)
	cmd = [
		"rpicam-vid",
		"--camera",
		str(camera_id),
		"--output",
		output_path,
		"--timeout",
		str(timeout_ms),
		"--width",
		str(width),
		"--height",
		str(height),
		"--framerate",
		str(fps),
		"--codec",
		"libav",
		"--libav-format",
		"mp4",
		"--shutter",
		str(shutter_us),
		"--gain",
		str(gain),
		"--flush",
		"--nopreview",
	]

	print("[START]", " ".join(cmd))
	return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def get_frame_count(video_path):
	cmd = [
		"ffprobe",
		"-v",
		"error",
		"-count_frames",
		"-select_streams",
		"v:0",
		"-show_entries",
		"stream=nb_read_frames",
		"-of",
		"default=nokey=1:noprint_wrappers=1",
		video_path,
	]
	r = subprocess.run(cmd, capture_output=True, text=True)
	if r.returncode != 0:
		return None
	value = r.stdout.strip()
	if not value.isdigit():
		return None
	return int(value)


def main():
	parser = argparse.ArgumentParser(
		description="Record 500s videos from Pi cameras 0 and 1 using rpicam-vid."
	)
	parser.add_argument("--duration", type=int, default=60, help="Duration in seconds (default: 10)")
	parser.add_argument("--w", type=int, default=1280, help="Frame width (default: 1280)")
	parser.add_argument("--h", type=int, default=720, help="Frame height (default: 720)")
	parser.add_argument("--fps", type=int, default=30, help="Frame rate (default: 30)")
	parser.add_argument(
		"--shutter-us",
		type=int,
		default=10000,
		help="Exposure time in microseconds (default: 10000)",
	)
	parser.add_argument("--gain", type=float, default=2.0, help="Analog gain (default: 2.0)")
	parser.add_argument(
		"--out-dir",
		default="recordings",
		help="Directory to store output videos (default: recordings)",
	)
	args = parser.parse_args()

	os.makedirs(args.out_dir, exist_ok=True)

	stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
	cam0_file = os.path.join(args.out_dir, f"camera0_{stamp}.mp4")
	cam1_file = os.path.join(args.out_dir, f"camera1_{stamp}.mp4")

	proc0 = start_recording(0, cam0_file, args.w, args.h, args.fps, args.duration, args.shutter_us, args.gain)
	proc1 = start_recording(1, cam1_file, args.w, args.h, args.fps, args.duration, args.shutter_us, args.gain)

	out0, err0 = proc0.communicate()
	out1, err1 = proc1.communicate()

	ok0 = proc0.returncode == 0 and os.path.exists(cam0_file)
	ok1 = proc1.returncode == 0 and os.path.exists(cam1_file)

	if ok0 and ok1:
		frames0 = get_frame_count(cam0_file)
		frames1 = get_frame_count(cam1_file)

		print("[DONE] Recording complete.")
		print(f"[FILE] {cam0_file}")
		print(f"[FILE] {cam1_file}")
		if frames0 is not None:
			print(f"[CAM0 FRAMES] {frames0}")
		if frames1 is not None:
			print(f"[CAM1 FRAMES] {frames1}")
		if (frames0 is not None and frames0 <= 1) or (frames1 is not None and frames1 <= 1):
			print("[WARN] At least one output still has <= 1 frame. Try lower --w/--h or --fps.")
		return

	print("[ERROR] One or both recordings failed.")

	if not ok0:
		print("[CAM0] Return code:", proc0.returncode)
		if out0.strip():
			print("[CAM0 STDOUT]\n" + out0.strip())
		if err0.strip():
			print("[CAM0 STDERR]\n" + err0.strip())

	if not ok1:
		print("[CAM1] Return code:", proc1.returncode)
		if out1.strip():
			print("[CAM1 STDOUT]\n" + out1.strip())
		if err1.strip():
			print("[CAM1 STDERR]\n" + err1.strip())

	sys.exit(1)


if __name__ == "__main__":
	main()
