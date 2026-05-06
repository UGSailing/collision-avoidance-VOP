"""
    This module implements the PathFollower class, which is responsible for reading the planned path from path.csv, calculating the appropriate rudder angle based on the boat's current GPS position and heading, and sending commands to the ESP32 autopilot. 
    It uses a lookahead mechanism to blend multiple upcoming waypoints for smoother steering and includes a turn feed-forward term to anticipate bends in the path.
"""

import pandas as pd
import math
import time
import config
import serial
from collections.abc import Sequence
from typing import TypeGuard, Union
from pandas.errors import EmptyDataError

class PathFollower:
    def __init__(self, run_dir):
        self.run_dir = run_dir
        self._autopilot_endpoints = self._resolve_autopilot_endpoints()
        self._serial_by_endpoint = {}
        self._last_connect_log_by_endpoint = {}
        self._connect_autopilot_transports()

    def _resolve_autopilot_endpoints(self) -> list[str]:
        raw = getattr(config, "ESP_SERIAL_PORTS", None)
        if isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)):
            endpoints = [str(item).strip() for item in raw if str(item).strip()]
        else:
            fallback = str(getattr(config, "ESP_SERIAL_PORT", "")).strip()
            endpoints = [fallback] if fallback else []

        # Preserve order, remove duplicates.
        unique_endpoints = list(dict.fromkeys(endpoints))
        if not unique_endpoints:
            raise ValueError("No autopilot transport configured. Set ESP_SERIAL_PORT or ESP_SERIAL_PORTS in config.py")
        return unique_endpoints

    def _connect_autopilot_transport(self, endpoint: str) -> None:
        if endpoint in self._serial_by_endpoint and self._serial_by_endpoint[endpoint] is not None:
            return

        try:
            # serial_for_url supports regular serial devices and URL backends like socket://
            serial_conn = serial.serial_for_url(
                endpoint,
                baudrate=config.ESP_BAUDRATE,
                timeout=config.ESP_TIMEOUT,
            )
            self._serial_by_endpoint[endpoint] = serial_conn
            print(f"Connected autopilot transport on {endpoint}")
        except Exception as exc:
            self._serial_by_endpoint[endpoint] = None
            now = time.monotonic()
            last_log = self._last_connect_log_by_endpoint.get(endpoint, 0.0)
            if now - last_log > 2.0:
                print(f"Autopilot transport unavailable ({endpoint}): {exc}")
                self._last_connect_log_by_endpoint[endpoint] = now

    def _connect_autopilot_transports(self) -> None:
        for endpoint in self._autopilot_endpoints:
            self._connect_autopilot_transport(endpoint)

    def _is_numeric(self, value: object) -> TypeGuard[float | int]:
        return isinstance(value, (int, float))

    def _normalize_angle_deg(self, angle_deg: float) -> float:
        """Normalizes an angle to [-180, 180)."""
        return (angle_deg + 180) % 360 - 180

    def _compute_bearing(self, lat1, lon1, lat2, lon2):
        """Calculates the true bearing (in degrees) from point 1 to point 2."""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)

        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)

        initial_bearing = math.atan2(y, x)
        return (math.degrees(initial_bearing) + 360) % 360

    def _compute_blended_target_bearing(self, curr_lat: float, curr_lon: float, path_df: pd.DataFrame) -> float:
        """Computes a weighted target bearing from a few upcoming waypoints."""
        future_waypoints = path_df.iloc[1:1 + config.LOOKAHEAD_WAYPOINT_COUNT]

        sin_sum = 0.0
        cos_sum = 0.0
        weight_sum = 0.0

        for i, (_, waypoint) in enumerate(future_waypoints.iterrows(), start=1):
            waypoint_lat = float(waypoint['latitude'])
            waypoint_lon = float(waypoint['longitude'])
            bearing = self._compute_bearing(curr_lat, curr_lon, waypoint_lat, waypoint_lon)

            weight = 1.0 / i
            rad = math.radians(bearing)
            sin_sum += weight * math.sin(rad)
            cos_sum += weight * math.cos(rad)
            weight_sum += weight

        if weight_sum == 0.0:
            return self._compute_bearing(
                curr_lat,
                curr_lon,
                float(path_df.iloc[1]['latitude']),
                float(path_df.iloc[1]['longitude'])
            )

        return (math.degrees(math.atan2(sin_sum, cos_sum)) + 360) % 360

    def _compute_turn_feedforward(self, path_df: pd.DataFrame) -> float:
        """Estimates near-future path curvature as a heading feed-forward term. This compensates for the boat's steering lag and helps it start turning earlier into bends."""
        preview_segments = max(1, int(config.TURN_PREVIEW_SEGMENTS))
        preview_points = path_df.iloc[:preview_segments + 2]
        if len(preview_points) < 3:
            return 0.0

        segment_bearings = []
        for i in range(len(preview_points) - 1):
            p0 = preview_points.iloc[i]
            p1 = preview_points.iloc[i + 1]
            segment_bearings.append(
                self._compute_bearing(
                    float(p0['latitude']),
                    float(p0['longitude']),
                    float(p1['latitude']),
                    float(p1['longitude'])
                )
            )

        if len(segment_bearings) < 2:
            return 0.0

        weighted_turn_sum = 0.0
        weight_sum = 0.0
        for i in range(len(segment_bearings) - 1):
            turn_delta = self._normalize_angle_deg(segment_bearings[i + 1] - segment_bearings[i])
            weight = 1.0 / (i + 1)
            weighted_turn_sum += turn_delta * weight
            weight_sum += weight

        if weight_sum == 0.0:
            return 0.0

        avg_turn_delta = weighted_turn_sum / weight_sum
        return float(config.TURN_FEEDFORWARD_GAIN) * avg_turn_delta

    def _heading_error_to_rudder_angle(self, heading_error_deg: float) -> float:
        """Maps heading error to a limited rudder angle with configurable aggressiveness."""
        aggressiveness = max(0.1, float(config.STEERING_AGGRESSIVENESS))
        max_error_for_full_rudder = max(1e-6, float(config.HEADING_ERROR_FOR_MAX_RUDDER_DEG))

        # Ignore tiny heading errors to reduce rudder chatter
        if abs(heading_error_deg) <= config.HEADING_DEADBAND_DEG:
            return 0.0

        effective_error = math.copysign(abs(heading_error_deg) - config.HEADING_DEADBAND_DEG, heading_error_deg)
        normalized_error = max(-1.0, min(1.0, effective_error / max_error_for_full_rudder))

        # Non-linear scaling for rudder: small adjustments for minor errors, more aggressive for larger errors
        shaped_error = math.tanh(aggressiveness * normalized_error) / math.tanh(aggressiveness)

        return float(config.STEERING_DIRECTION) * shaped_error * config.MAX_RUDDER_ANGLE_DEG
    
    def _get_current_heading_and_location(self, run_dir) -> tuple[float | None, float | None, float | None]:
        """Reads the latest GPS point from points.csv and returns (lat, lon, heading)."""
        try:
            df = pd.read_csv(run_dir / 'points.csv')
            gps_points = df[df['category'] == 'gps']
            
            if len(gps_points) < 1:
                return None, None, None 
                
            current_loc = gps_points.loc[gps_points['id'].idxmax()]
            current_heading = current_loc.get('heading', None)
            current_lat = current_loc.get('latitude', None)
            current_lon = current_loc.get('longitude', None)
            
            if pd.isna(current_heading) or pd.isna(current_lat) or pd.isna(current_lon): # type: ignore
                return None, None, None

            if not (
                self._is_numeric(current_heading)
                and self._is_numeric(current_lat)
                and self._is_numeric(current_lon)
            ):
                return None, None, None
                
            return float(current_lat), float(current_lon), float(current_heading)
        except (TypeError, ValueError, KeyError, FileNotFoundError):
            return None, None, None

    def follow_path(self, run_dir):
        """Reads the path, calculates steering error, and sends ESP32 command."""
        try:
            # 1. Get current state
            curr_lat, curr_lon, current_heading = self._get_current_heading_and_location(run_dir)
            if not (self._is_numeric(curr_lat) and self._is_numeric(curr_lon) and self._is_numeric(current_heading)):
                # Waiting for GPS data
                self._send_autopilot_command(0, 0)
                return
            curr_lat_f = float(curr_lat)
            curr_lon_f = float(curr_lon)
            current_heading_f = float(current_heading)

            # 2. Read the planned path
            path_file = run_dir / 'path.csv'
            if not path_file.exists():
                self._send_autopilot_command(0, 0)
                return
            try:
                path_df = pd.read_csv(path_file)
            except EmptyDataError:
                self._send_autopilot_command(0, 0)
                return
            if len(path_df) < 2:
                # We are at the destination (or no path exists)
                self._send_autopilot_command(0, 0)
                return

            # 3. Build a lookahead target heading from multiple upcoming waypoints.
            target_bearing = self._compute_blended_target_bearing(curr_lat_f, curr_lon_f, path_df)

            # 4. Add turn preview feed-forward so the boat starts steering into bends earlier.
            target_bearing = (target_bearing + self._compute_turn_feedforward(path_df)) % 360

            # 5. Convert relative heading target to a rudder angle.
            relative_target_heading_deg = self._normalize_angle_deg(target_bearing - current_heading_f)
            rudder_angle_deg = self._heading_error_to_rudder_angle(relative_target_heading_deg)

            # 6. Send rudder + fixed thrust to ESP32.
            self._send_autopilot_command(rudder_angle_deg, config.FIXED_THRUST)

            print(
                f"Target Bearing: {target_bearing:.1f}° | "
                f"Heading Error: {relative_target_heading_deg:.1f}° | "
                f"Rudder Cmd: {rudder_angle_deg:.1f}°"
            )
        except Exception as e:
            print(f"Execution Error: {e}")

    def _build_autopilot_message(self, target_angle: Union[int, float], target_thrust: Union[int, float]) -> str:
        """Builds the command message to send to the ESP32, ensuring values are within configured limits. Only needs to be used in _send_autopilot_command."""
        angle = max(-config.MAX_RUDDER_ANGLE_DEG, min(config.MAX_RUDDER_ANGLE_DEG, float(target_angle)))
        thrust = max(-1.0, min(1.0, float(target_thrust)))

        # Firmware expects: "angle,thrust\n"
        return f"{angle:.2f},{thrust:.3f}\n"

    def _send_autopilot_command(self, target_angle: Union[int, float], target_thrust: Union[int, float]) -> None:
        """Sends a command to the ESP32 controller."""
        msg = self._build_autopilot_message(target_angle, target_thrust)
        wire = msg.encode("ascii")

        for endpoint in self._autopilot_endpoints:
            if self._serial_by_endpoint.get(endpoint) is None:
                self._connect_autopilot_transport(endpoint)

            serial_conn = self._serial_by_endpoint.get(endpoint)
            if serial_conn is None:
                continue

            try:
                serial_conn.write(wire)
            except Exception:
                self._serial_by_endpoint[endpoint] = None