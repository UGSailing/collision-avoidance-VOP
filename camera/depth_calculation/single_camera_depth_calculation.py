import argparse
import math
import cv2
import numpy as np
import yaml

def load_calib_yaml(path: str):
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    K = np.array(data["K"], dtype=np.float64)
    dist = np.array(data["dist"], dtype=np.float64).reshape(-1, 1)
    # dist kan (1,5) of (5,1) etc zijn; dit is ok voor OpenCV
    return K, dist

def undistort_points(pts, K, dist):
    # pts: Nx1x2 (float32)
    # output: Nx2 in pixel coords (met dezelfde K als projectie)
    und = cv2.undistortPoints(pts, K, dist, P=K)
    return und.reshape(-1, 2)

def main():
    ap = argparse.ArgumentParser(description="Distance + azimuth from a single image of a chessboard (pinhole formulas).")
    ap.add_argument("--image", required=True, help="Path to image (jpg/png)")
    ap.add_argument("--calib", required=True, help="Calibration YAML with K and dist (from your calibration script)")
    ap.add_argument("--cols", type=int, required=True, help="Inner corners (cols)")
    ap.add_argument("--rows", type=int, required=True, help="Inner corners (rows)")
    ap.add_argument("--square", type=float, required=True, help="Square size in meters (e.g. 0.0285)")
    ap.add_argument("--show", action="store_true", help="Show debug visualization")
    args = ap.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f"Could not read image: {args.image}")

    K, dist = load_calib_yaml(args.calib)
    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])

    pattern_size = (args.cols, args.rows)  # (cols, rows)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
    if not found:
        raise SystemExit("No chessboard found. Try better lighting / correct rows/cols / bigger board in frame.")

    # subpixel refine
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    # undistort corner coordinates before measuring pixel size
    corners_ud = undistort_points(corners, K, dist)  # Nx2

    # Real board dimensions from INNER corners:
    # distance between top and bottom inner-corner rows spans (rows-1) squares
    H = (args.rows - 1) * args.square  # meters (vertical)
    W = (args.cols - 1) * args.square  # meters (horizontal) (not strictly needed)

    # Pixel height: use mean Y of top row vs mean Y of bottom row (more stable than max-min)
    top_row = corners_ud[0:args.cols, :]
    bot_row = corners_ud[(args.rows - 1) * args.cols : args.rows * args.cols, :]

    y_top = float(np.mean(top_row[:, 1]))
    y_bot = float(np.mean(bot_row[:, 1]))
    h_px = abs(y_bot - y_top)

    if h_px < 1.0:
        raise SystemExit("Measured pixel height too small; board likely too far / detection unstable.")

    # Center u: mean x of all corners
    u_center = float(np.mean(corners_ud[:, 0]))

    # Formulas from your doc
    Z = fy * H / h_px  # meters
    azimuth_rad = math.atan((u_center - cx) / fx)
    azimuth_deg = azimuth_rad * 180.0 / math.pi

    print("=== INPUTS ===")
    print(f"Inner corners (cols x rows): {args.cols} x {args.rows}")
    print(f"Square size: {args.square} m")
    print(f"Board size (W x H): {W:.4f} m x {H:.4f} m")
    print("\n=== MEASURED IN IMAGE ===")
    print(f"h_px (board pixel height): {h_px:.2f} px")
    print(f"u_center: {u_center:.2f} px")
    print("\n=== RESULT ===")
    print(f"Distance Z: {Z:.3f} m")
    print(f"Azimuth: {azimuth_deg:.3f} deg (relative to camera optical axis)")

    if args.show:
        vis = img.copy()
        # draw corners
        for (x, y) in corners_ud.astype(int):
            cv2.circle(vis, (int(x), int(y)), 4, (0, 255, 0), -1)
        # draw top/bottom reference lines
        cv2.line(vis, (0, int(y_top)), (vis.shape[1]-1, int(y_top)), (255, 0, 0), 2)
        cv2.line(vis, (0, int(y_bot)), (vis.shape[1]-1, int(y_bot)), (255, 0, 0), 2)
        cv2.putText(vis, f"Z={Z:.2f}m, az={azimuth_deg:.2f}deg",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("result", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()