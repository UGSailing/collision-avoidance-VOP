#!/usr/bin/env python3
import argparse
import os
import time
from pathlib import Path

import cv2
from picamera2 import Picamera2

# python3 stereo_live_capture.py --out calib --width 1920 --height 1080 --max-pairs 20 --detect-board


def draw_text(img, text, y):
    cv2.putText(
        img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA
    )


def main():
    parser = argparse.ArgumentParser(
        description="Live stereo capture with preview for chessboard calibration"
    )
    parser.add_argument("--out", default="calib", help="Output folder")
    parser.add_argument("--width", type=int, default=1280, help="Capture width")
    parser.add_argument("--height", type=int, default=720, help="Capture height")
    parser.add_argument("--left-id", type=int, default=0, help="Left camera id")
    parser.add_argument("--right-id", type=int, default=1, help="Right camera id")
    parser.add_argument(
        "--max-pairs",
        type=int,
        default=20,
        help="How many pairs to capture before stopping",
    )
    parser.add_argument(
        "--detect-board",
        action="store_true",
        help="Try to detect and draw the chessboard live",
    )
    parser.add_argument(
        "--cols", type=int, default=9, help="Checkerboard inner corners in width"
    )
    parser.add_argument(
        "--rows", type=int, default=6, help="Checkerboard inner corners in height"
    )
    parser.add_argument(
        "--flip-left",
        action="store_true",
        help="Flip left image horizontally for preview/save",
    )
    parser.add_argument(
        "--flip-right",
        action="store_true",
        help="Flip right image horizontally for preview/save",
    )
    args = parser.parse_args()

    left_dir = "../dual_calib_images/left"
    right_dir = "../dual_calib_images/right"
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    picam_left = Picamera2(args.left_id)
    picam_right = Picamera2(args.right_id)

    cfg_left = picam_left.create_preview_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"}
    )
    cfg_right = picam_right.create_preview_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"}
    )

    picam_left.configure(cfg_left)
    picam_right.configure(cfg_right)

    picam_left.start()
    picam_right.start()

    time.sleep(2.0)  # let auto exposure / white balance settle

    pair_idx = 0
    board_size = (args.cols, args.rows)

    print("Controls:")
    print("  c or SPACE  -> capture current pair")
    print("  q or ESC    -> quit")
    print("Make sure the chessboard is visible in BOTH views before capturing.")

    try:
        while True:
            frame_left = picam_left.capture_array()
            frame_right = picam_right.capture_array()

            # Picamera2 gives RGB888 here; OpenCV expects BGR for normal display/save colors.
            frame_left = cv2.cvtColor(frame_left, cv2.COLOR_RGB2BGR)
            frame_right = cv2.cvtColor(frame_right, cv2.COLOR_RGB2BGR)

            if args.flip_left:
                frame_left = cv2.flip(frame_left, 1)
            if args.flip_right:
                frame_right = cv2.flip(frame_right, 1)

            preview_left = frame_left.copy()
            preview_right = frame_right.copy()

            ok_left = ok_right = False

            if args.detect_board:
                gray_left = cv2.cvtColor(preview_left, cv2.COLOR_BGR2GRAY)
                gray_right = cv2.cvtColor(preview_right, cv2.COLOR_BGR2GRAY)

                flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                ok_left, corners_left = cv2.findChessboardCorners(
                    gray_left, board_size, flags
                )
                ok_right, corners_right = cv2.findChessboardCorners(
                    gray_right, board_size, flags
                )

                if ok_left:
                    cv2.drawChessboardCorners(
                        preview_left, board_size, corners_left, ok_left
                    )
                if ok_right:
                    cv2.drawChessboardCorners(
                        preview_right, board_size, corners_right, ok_right
                    )

            draw_text(preview_left, f"LEFT cam {args.left_id}", 30)
            draw_text(preview_right, f"RIGHT cam {args.right_id}", 30)
            draw_text(preview_left, f"pair {pair_idx}/{args.max_pairs}", 60)
            draw_text(preview_right, f"pair {pair_idx}/{args.max_pairs}", 60)

            if args.detect_board:
                draw_text(preview_left, f"board: {'YES' if ok_left else 'NO'}", 90)
                draw_text(preview_right, f"board: {'YES' if ok_right else 'NO'}", 90)

            combined = cv2.hconcat([preview_left, preview_right])
            draw_text(
                combined, "c/SPACE = capture    q/ESC = quit", combined.shape[0] - 20
            )

            cv2.imshow("Stereo preview", combined)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):  # q or ESC
                break

            if key in (ord("c"), 32):  # c or SPACE
                left_path = left_dir / f"left_{pair_idx:02d}.jpg"
                right_path = right_dir / f"right_{pair_idx:02d}.jpg"

                cv2.imwrite(str(left_path), frame_left)
                cv2.imwrite(str(right_path), frame_right)

                print(f"Saved pair {pair_idx:02d}:")
                print(f"  {left_path}")
                print(f"  {right_path}")

                pair_idx += 1

                if pair_idx >= args.max_pairs:
                    print("Reached max number of pairs.")
                    break

    finally:
        picam_left.stop()
        picam_right.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
