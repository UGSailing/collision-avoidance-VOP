import cv2
import numpy as np
import argparse
import os

# To run this file:
# python3 rectify_stereo.py --calib stereo_calib.npz --left left.jpg --right right.jpg --out-left rect_left.jpg --out-right rect_right.jpg --show


def rectify_pair(calib_file, left_path, right_path, out_left, out_right, show=False):
    # loads the dual calib file (.npz)
    data = np.load(calib_file)

    map1x = data["map1x"]
    map1y = data["map1y"]
    map2x = data["map2x"]
    map2y = data["map2y"]

    img_left = cv2.imread(left_path)
    img_right = cv2.imread(right_path)

    if img_left is None:
        raise FileNotFoundError(f"Left image not found: {left_path}")
    if img_right is None:
        raise FileNotFoundError(f"Right image not found: {right_path}")

    # Actual rectification step: undistortion + rectification
    rect_left = cv2.remap(img_left, map1x, map1y, cv2.INTER_LINEAR)
    rect_right = cv2.remap(img_right, map2x, map2y, cv2.INTER_LINEAR)

    cv2.imwrite(out_left, rect_left)
    cv2.imwrite(out_right, rect_right)

    print(f"Saved rectified left image to:  {out_left}")
    print(f"Saved rectified right image to: {out_right}")

    if show:  # then draw green horizontal lines
        vis_left = rect_left.copy()
        vis_right = rect_right.copy()

        for y in range(0, vis_left.shape[0], 40):
            cv2.line(vis_left, (0, y), (vis_left.shape[1], y), (0, 255, 0), 1)
            cv2.line(vis_right, (0, y), (vis_right.shape[1], y), (0, 255, 0), 1)

        cv2.imshow("Rectified Left", vis_left)
        cv2.imshow("Rectified Right", vis_right)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--calib", required=True, help="Path to stereo_calib.npz")
    parser.add_argument("--left", required=True, help="Path to left image")
    parser.add_argument("--right", required=True, help="Path to right image")
    parser.add_argument(
        "--out-left", default="rect_left.png", help="Output rectified left image"
    )
    parser.add_argument(
        "--out-right", default="rect_right.png", help="Output rectified right image"
    )
    parser.add_argument(
        "--show", action="store_true", help="Show rectified images with guide lines"
    )
    args = parser.parse_args()

    rectify_pair(
        args.calib, args.left, args.right, args.out_left, args.out_right, args.show
    )
