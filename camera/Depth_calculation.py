#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_calib_yaml(path: str) -> np.ndarray:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if "K" not in data:
        raise ValueError(f"Calibration file '{path}' does not contain key 'K'.")

    K = np.array(data["K"], dtype=np.float64)
    if K.shape != (3, 3):
        raise ValueError(f"Expected K to have shape (3,3), got {K.shape}.")
    return K


def depth_from_disparity(
    disparity_px: float, focal_px: float, baseline_m: float
) -> float:
    if focal_px <= 0:
        raise ValueError("focal_px must be > 0")
    if baseline_m <= 0:
        raise ValueError("baseline_m must be > 0")
    if disparity_px <= 0:
        raise ValueError("disparity_px must be > 0")

    return (focal_px * baseline_m) / float(disparity_px)


def depth_map_from_disparity(
    disparity_px: np.ndarray, focal_px: float, baseline_m: float
) -> np.ndarray:
    if focal_px <= 0:
        raise ValueError("focal_px must be > 0")
    if baseline_m <= 0:
        raise ValueError("baseline_m must be > 0")

    disparity_px = np.asarray(disparity_px, dtype=np.float64)
    depth_m = np.full_like(disparity_px, np.inf, dtype=np.float64)

    # Avoid division by zero and negative disparities.
    valid = disparity_px > 1e-9
    depth_m[valid] = (focal_px * baseline_m) / disparity_px[valid]
    return depth_m


def resolve_focal_px(calib_path: str | None, focal_px: float | None) -> float:
    if focal_px is not None:
        return float(focal_px)
    if not calib_path:
        raise ValueError("Provide either --focal-px or --calib.")

    K = load_calib_yaml(calib_path)
    return float(K[0, 0])


def save_depth_visualization(
    depth_m: np.ndarray, out_path: str, max_depth_m: float
) -> None:
    finite = np.isfinite(depth_m)
    vis = np.zeros(depth_m.shape, dtype=np.uint8)

    if np.any(finite):
        clipped = np.clip(depth_m[finite], 0.0, max_depth_m)
        normalized = np.uint8((clipped / max_depth_m) * 255.0)
        vis[finite] = normalized

    color = cv2.applyColorMap(255 - vis, cv2.COLORMAP_TURBO)
    cv2.imwrite(out_path, color)


def main() -> None:
    ap = argparse.ArgumentParser(description="Diepte z wordt berekend m.b.v. z = f*b/d")
    ap.add_argument(
        "--baseline-m", type=float, default=0.06, help="Stereo baseline in meter"
    )
    ap.add_argument(
        "--calib", type=str, default=None, help="Calibration YAML with camera matrix K"
    )
    ap.add_argument(
        "--focal-px",
        type=float,
        default=None,
        help="Focal length in pixels (overrides --calib)",
    )

    mode = ap.add_subparsers(dest="mode", required=True)

    single = mode.add_parser("single", help="Compute depth for one disparity value")
    single.add_argument(
        "--disparity-px", type=float, required=True, help="Disparity in pixels"
    )

    map_mode = mode.add_parser(
        "map", help="Compute depth map from a .npy disparity array"
    )
    map_mode.add_argument(
        "--disparity-npy", type=str, required=True, help="Path to disparity .npy"
    )
    map_mode.add_argument(
        "--out-depth-npy",
        type=str,
        default="depth_map.npy",
        help="Output depth .npy path",
    )
    map_mode.add_argument(
        "--out-depth-png",
        type=str,
        default="depth_map.png",
        help="Output colored depth image",
    )
    map_mode.add_argument(
        "--max-depth-m",
        type=float,
        default=20.0,
        help="Visualization clipping max depth",
    )

    args = ap.parse_args()

    focal_px = resolve_focal_px(args.calib, args.focal_px)

    if args.mode == "single":
        d = float(args.disparity_px)
        z_m = depth_from_disparity(d, focal_px, args.baseline_m)
        print(
            f"focal_px={focal_px:.6f}, baseline_m={args.baseline_m:.6f}, disparity_px={d:.6f}"
        )
        print(f"depth_m={z_m:.6f}")
        return

    disparity_path = Path(args.disparity_npy)
    if not disparity_path.exists():
        raise FileNotFoundError(f"Disparity file not found: {disparity_path}")

    disparity = np.load(disparity_path)
    depth_m = depth_map_from_disparity(disparity, focal_px, args.baseline_m)

    np.save(args.out_depth_npy, depth_m)
    save_depth_visualization(depth_m, args.out_depth_png, args.max_depth_m)

    finite = np.isfinite(depth_m)
    if np.any(finite):
        mn = float(np.min(depth_m[finite]))
        mx = float(np.max(depth_m[finite]))
        print(f"Saved depth map to {args.out_depth_npy}")
        print(f"Saved depth visualization to {args.out_depth_png}")
        print(f"Depth range (finite): min={mn:.4f} m, max={mx:.4f} m")
    else:
        print("No valid disparity values > 0 found, depth map contains only infinity.")


if __name__ == "__main__":
    main()
