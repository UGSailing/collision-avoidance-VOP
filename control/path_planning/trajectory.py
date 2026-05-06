"""Trajectory optimizer for smooth path + speed profile.

Input:  path.csv from v2 A* (lat/lon waypoints).
Output: trajectory.csv with lat/lon, heading, speed, and thrust.

Runs only when the path changes.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd

import config
from .Speed_profile import Params, compute_path_geometry, solve_speed_profile, bspline_curve_np


_LAST_PATH_SIGNATURE: str | None = None


@dataclass
class TrajectoryConfig:
    n_control_points: int = int(getattr(config, "TRAJ_CONTROL_POINTS", 16))
    n_samples: int = int(getattr(config, "TRAJ_POINTS", 300))
    v_min: float = float(getattr(config, "TRAJ_MIN_SPEED", 0.05))
    v_max: float = float(getattr(config, "TRAJ_MAX_SPEED", 5.0))
    use_speed_profile: bool = bool(getattr(config, "TRAJ_USE_SPEED_PROFILE", True))
    fallback_speed: float = float(getattr(config, "TRAJ_FALLBACK_SPEED", 0.4))


def _path_signature(df: pd.DataFrame) -> str:
    rounded = df[["latitude", "longitude"]].round(7).to_csv(index=False).encode("ascii")
    return hashlib.sha1(rounded).hexdigest()


def _latlon_to_local(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    y = (lat - origin_lat) * config.METERS_PER_DEGREE_LAT
    x = (lon - origin_lon) * (config.METERS_PER_DEGREE_LAT * np.cos(np.radians(origin_lat)))
    return x, y


def _local_to_latlon(x: float, y: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    lat = (y / config.METERS_PER_DEGREE_LAT) + origin_lat
    lon = (x / (config.METERS_PER_DEGREE_LAT * np.cos(np.radians(origin_lat)))) + origin_lon
    return lat, lon


def _fit_control_points(points_xy: np.ndarray, n_cp: int) -> tuple[np.ndarray, np.ndarray]:
    if len(points_xy) < 2:
        raise ValueError("Need at least two points to fit a spline")

    diffs = np.diff(points_xy, axis=0)
    seg_lens = np.sqrt((diffs**2).sum(axis=1))
    s = np.concatenate([[0.0], np.cumsum(seg_lens)])
    if s[-1] <= 1e-6:
        s = np.linspace(0.0, 1.0, len(points_xy))
    else:
        s = s / s[-1]

    s_cp = np.linspace(0.0, 1.0, n_cp)
    px = np.interp(s_cp, s, points_xy[:, 0])
    py = np.interp(s_cp, s, points_xy[:, 1])
    return px, py


def _dedupe_points(points_xy: np.ndarray, min_dist: float) -> np.ndarray:
    if len(points_xy) < 2:
        return points_xy
    keep = [points_xy[0]]
    for pt in points_xy[1:]:
        if np.linalg.norm(pt - keep[-1]) >= min_dist:
            keep.append(pt)
    if len(keep) == 1:
        keep.append(points_xy[-1])
    return np.array(keep)


def _fallback_speed_profile(s_path: np.ndarray, speed_mps: float) -> tuple[np.ndarray, np.ndarray]:
    v = np.full_like(s_path, max(speed_mps, 0.01), dtype=float)
    thrust = np.full_like(s_path, float(config.FIXED_THRUST), dtype=float)
    return v, thrust


def _eval_bspline(Px: np.ndarray, Py: np.ndarray, t_vals: Iterable[float]) -> tuple[np.ndarray, np.ndarray]:
    xs, ys = [], []
    for t in t_vals:
        xi, yi = bspline_curve_np(Px, Py, float(t))
        xs.append(xi)
        ys.append(yi)
    return np.array(xs), np.array(ys)


def update_trajectory(run_dir: Path, mapper) -> None:
    """Generate trajectory.csv from path.csv if the path changed."""
    global _LAST_PATH_SIGNATURE

    path_file = run_dir / "path.csv"
    if not path_file.exists() or path_file.stat().st_size == 0:
        return

    try:
        path_df = pd.read_csv(path_file)
    except Exception:
        return

    if len(path_df) < 2:
        return

    sig = _path_signature(path_df)
    if sig == _LAST_PATH_SIGNATURE:
        return

    if mapper.origin_lat is None or mapper.origin_lon is None:
        return

    origin_lat = float(mapper.origin_lat)
    origin_lon = float(mapper.origin_lon)

    points_xy = np.array([
        _latlon_to_local(float(row.latitude), float(row.longitude), origin_lat, origin_lon)
        for row in path_df.itertuples()
    ])

    points_xy = _dedupe_points(points_xy, min_dist=max(0.01, float(config.GRID_RESOLUTION) * 0.25))
    if len(points_xy) < 2:
        return

    cfg = TrajectoryConfig()
    px, py = _fit_control_points(points_xy, cfg.n_control_points)

    ship = Params()
    result = None
    if cfg.use_speed_profile:
        try:
            geo = compute_path_geometry(px, py, N=max(200, cfg.n_samples))
            result = solve_speed_profile(geo, ship, N_opt=cfg.n_samples, v_min=cfg.v_min, v_max=cfg.v_max)
        except Exception as exc:
            print(f"Trajectory speed profile failed: {exc}")

    t_vals = np.linspace(0.0, 1.0, cfg.n_samples)
    xs, ys = _eval_bspline(px, py, t_vals)

    loop_tol = max(0.25, float(config.GRID_RESOLUTION))
    if len(xs) > 2:
        loop_dist = float(np.hypot(xs[-1] - xs[0], ys[-1] - ys[0]))
        if loop_dist <= loop_tol:
            xs = xs[:-1]
            ys = ys[:-1]
            t_vals = t_vals[:-1]

    dx = np.gradient(xs)
    dy = np.gradient(ys)
    headings = (np.degrees(np.arctan2(dy, dx)) + 360.0) % 360.0

    s_path = np.concatenate([[0.0], np.cumsum(np.sqrt(np.diff(xs)**2 + np.diff(ys)**2))])
    if result is None or not np.isfinite(result["v"]).all():
        v_path, thrust = _fallback_speed_profile(s_path, cfg.fallback_speed)
    else:
        v_path = np.interp(s_path, result["s"], result["v"])
        T_req = np.interp(s_path, result["s"], result["T_req"])
        thrust = np.clip(T_req / max(ship.T_max, 1e-6), 0.0, 1.0)

    latlon = [_local_to_latlon(x, y, origin_lat, origin_lon) for x, y in zip(xs, ys)]

    traj_df = pd.DataFrame(
        {
            "latitude": [ll[0] for ll in latlon],
            "longitude": [ll[1] for ll in latlon],
            "heading_deg": headings,
            "speed_mps": v_path,
            "thrust": thrust,
            "s_m": s_path,
        }
    )

    traj_df.to_csv(run_dir / "trajectory.csv", index=False)
    _LAST_PATH_SIGNATURE = sig

