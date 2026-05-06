import numpy as np
import pandas as pd
from pandas.errors import EmptyDataError

import config
from . import path_execution_path


class PathFollowerV2(path_execution_path.PathFollower):
    """Follow a time-parameterized trajectory with heading + thrust."""

    def _nearest_trajectory_index(self, curr_lat: float, curr_lon: float, traj_df: pd.DataFrame) -> int:
        lats = traj_df["latitude"].to_numpy()
        lons = traj_df["longitude"].to_numpy()
        dlat = lats - curr_lat
        dlon = lons - curr_lon
        return int(np.argmin(dlat * dlat + dlon * dlon))

    def _trajectory_target(self, curr_lat: float, curr_lon: float, traj_df: pd.DataFrame) -> pd.Series:
        idx = self._nearest_trajectory_index(curr_lat, curr_lon, traj_df)
        lookahead = int(getattr(config, "TRAJ_LOOKAHEAD_POINTS", 6))
        idx = min(idx + max(0, lookahead), len(traj_df) - 1)
        return traj_df.iloc[idx]

    def follow_path(self, run_dir):
        """Reads trajectory.csv, calculates steering error, and sends ESP32 command."""
        try:
            curr_lat, curr_lon, current_heading = self._get_current_heading_and_location(run_dir)
            if not (self._is_numeric(curr_lat) and self._is_numeric(curr_lon) and self._is_numeric(current_heading)):
                self._send_autopilot_command(0, 0)
                return

            curr_lat_f = float(curr_lat)
            curr_lon_f = float(curr_lon)
            current_heading_f = float(current_heading)

            traj_file = run_dir / "trajectory.csv"
            if not traj_file.exists():
                self._send_autopilot_command(0, 0)
                return

            try:
                traj_df = pd.read_csv(traj_file)
            except EmptyDataError:
                self._send_autopilot_command(0, 0)
                return

            if len(traj_df) < 2:
                self._send_autopilot_command(0, 0)
                return

            target = self._trajectory_target(curr_lat_f, curr_lon_f, traj_df)
            target_heading = float(target.get("heading_deg", current_heading_f))
            target_thrust = float(target.get("thrust", config.FIXED_THRUST))

            relative_target_heading_deg = self._normalize_angle_deg(target_heading - current_heading_f)
            rudder_angle_deg = self._heading_error_to_rudder_angle(relative_target_heading_deg)

            self._send_autopilot_command(rudder_angle_deg, target_thrust)

            print(
                f"Target Heading: {target_heading:.1f} deg | "
                f"Heading Error: {relative_target_heading_deg:.1f} deg | "
                f"Rudder Cmd: {rudder_angle_deg:.1f} deg | "
                f"Thrust: {target_thrust:.2f}"
            )
        except Exception as e:
            print(f"Execution Error: {e}")
