#!/usr/bin/env python3
import argparse
import os
import time
from pathlib import Path

import cv2
from picamera2 import Picamera2

# python3 live_capture_for_dual_calib.py --width 1920 --height 1080 --max-pairs 20 --detect-board


def draw_text(img, text, y, color=(0, 255, 0)):
    """Helper function to draw text on an image."""
    cv2.putText(
        img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA
    )


def main():
    parser = argparse.ArgumentParser(
        description="Live stereo capture with preview for chessboard calibration"
    )
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

    # --- Setup directories ---
    current_file = Path(__file__).resolve()
    camera_dir = current_file.parent.parent
    left_dir = camera_dir / "dual_calib_images" / "left"
    right_dir = camera_dir / "dual_calib_images" / "right"
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    picam_left = Picamera2(args.left_id)
    picam_right = Picamera2(args.right_id)

    print("Configuring cameras...")
    cfg_left = picam_left.create_preview_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"}
    )
    cfg_right = picam_right.create_preview_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"}
    )
    picam_left.configure(cfg_left)
    picam_right.configure(cfg_right)

    pair_idx = 0
    board_size = (args.cols, args.rows)

    try:
        print("Starting camera streams for preview...")
        picam_left.start()
        picam_right.start()
        time.sleep(2.0)  # let auto exposure / white balance settle

        print("\nControls:")
        print("  c or SPACE  -> capture current pair")
        print("  q or ESC    -> quit")
        print("Make sure the chessboard is visible in BOTH views before capturing.\n")

        while True:
            # --- Main preview loop ---
            frame_left = picam_left.capture_array()
            frame_right = picam_right.capture_array()

            # This check is important in case a camera stream fails
            if frame_left is None or frame_right is None:
                print("ERROR: Failed to get frame from a camera during preview.")
                break

            preview_left = cv2.cvtColor(frame_left, cv2.COLOR_RGB2BGR)
            preview_right = cv2.cvtColor(frame_right, cv2.COLOR_RGB2BGR)

            # (The rest of the preview drawing logic is the same)
            draw_text(preview_left, f"LEFT cam {args.left_id}", 30)
            draw_text(preview_right, f"RIGHT cam {args.right_id}", 30)
            draw_text(preview_left, f"pair {pair_idx}/{args.max_pairs}", 60)
            draw_text(preview_right, f"pair {pair_idx}/{args.max_pairs}", 60)

            combined = cv2.hconcat([preview_left, preview_right])
            draw_text(
                combined, "c/SPACE = capture    q/ESC = quit", combined.shape[0] - 20
            )

            cv2.imshow("Stereo preview", combined)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            # --- FIX: The Capture Process ---
            if key in (ord("c"), 32):
                print(f"\n[CAPTURE] User pressed capture for pair {pair_idx:02d}.")

                # 1. Stop the continuous streams to free the cameras
                print("  -> Stopping preview streams...")
                picam_left.stop()
                picam_right.stop()
                time.sleep(0.5)  # Give time for cameras to release

                # 2. Capture high-quality still images
                print("  -> Capturing still images...")
                # We re-use the same configuration, but capture a single frame
                capture_left = picam_left.capture_array()
                capture_right = picam_right.capture_array()

                if capture_left is None or capture_right is None:
                    print(
                        "  -> ERROR: Failed to capture still image from one or both cameras!"
                    )
                    # Try to restart streams to not leave user blind
                    picam_left.start()
                    picam_right.start()
                    continue  # Go back to preview loop

                # Convert to BGR for saving with OpenCV
                save_left = cv2.cvtColor(capture_left, cv2.COLOR_RGB2BGR)
                save_right = cv2.cvtColor(capture_right, cv2.COLOR_RGB2BGR)

                # Apply optional flip
                if args.flip_left:
                    save_left = cv2.flip(save_left, 1)
                if args.flip_right:
                    save_right = cv2.flip(save_right, 1)

                # 3. Save the images
                left_path = left_dir / f"left_{pair_idx:02d}.jpg"
                right_path = right_dir / f"right_{pair_idx:02d}.jpg"
                ok_l = cv2.imwrite(str(left_path), save_left)
                ok_r = cv2.imwrite(str(right_path), save_right)

                if ok_l and ok_r:
                    print(f"  -> Successfully saved pair {pair_idx:02d}.")
                    pair_idx += 1
                else:
                    print(
                        "  -> ERROR: Failed to save images. Check permissions/disk space."
                    )

                # 4. Restart the streams for the live preview
                print("  -> Restarting preview streams...")
                picam_left.start()
                picam_right.start()
                time.sleep(1.0)  # Let cameras settle again

                if pair_idx >= args.max_pairs:
                    print("\nReached max number of pairs.")
                    break

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")

    finally:
        print("\nStopping cameras...")
        # Check if running before stopping
        if picam_left and picam_left.is_open:
            picam_left.stop()
        if picam_right and picam_right.is_open:
            picam_right.stop()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
