import pandas as pd
import math
import config
import can

def is_boat_in_geofence(boat_lat, boat_lon):
    """Checks raw GPS against the geofence."""
    if not hasattr(config, 'GEOFENCE_GPS') or not config.GEOFENCE_GPS:
        return True # Default to safe if no geofence defined
        
    is_inside = False
    j = len(config.GEOFENCE_GPS) - 1
    for i in range(len(config.GEOFENCE_GPS)):
        lati, loni = config.GEOFENCE_GPS[i]
        latj, lonj = config.GEOFENCE_GPS[j]
        
        # Note: Y is Lat, X is Lon
        y_between = (lati > boat_lat) != (latj > boat_lat)
        if y_between:
            lon_intersect = loni + (boat_lat - lati) * (lonj - loni) / (latj - lati)
            if boat_lon < lon_intersect:
                is_inside = not is_inside
        j = i
    return is_inside


class PathFollower:
    def __init__(self, run_dir):
        self.run_dir = run_dir
        
        # Safe initialization for bench testing without hardware
        try:
            self.can_bus = can.interface.Bus(
                channel=config.CAN_CHANNEL,
                bustype=config.CAN_BUSTYPE,
                bitrate=config.CAN_BITRATE,
            )
        except (can.CanError, OSError):
            self.can_bus = None

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculates the true bearing (in degrees) from point 1 to point 2."""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)

        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)

        initial_bearing = math.atan2(y, x)
        return (math.degrees(initial_bearing) + 360) % 360

    def get_current_heading_and_location(self, run_dir):
        try:
            df = pd.read_csv(run_dir / 'points.csv')
            gps_points = df[df['category'] == 'gps']
            
            if len(gps_points) < 1:
                return None, None, None 
                
            current_loc = gps_points.loc[gps_points['id'].idxmax()]
            current_heading = current_loc.get('heading', None)
            
            if pd.isna(current_heading):
                return None, None, None
                
            return current_loc['latitude'], current_loc['longitude'], current_heading
        except (ValueError, KeyError, FileNotFoundError):
            return None, None, None

    def follow_path(self, run_dir):
        """Reads the path, calculates steering error, and sends CAN command."""
        try:
            # 1. Get current state
            curr_lat, curr_lon, current_heading = self.get_current_heading_and_location(run_dir)
            if curr_lat is None:
                return # Waiting for GPS data

            # --- GEOFENCE EMERGENCY CHECK ---
            if not is_boat_in_geofence(curr_lat, curr_lon):
                print("🚨 GEOFENCE BREACH DETECTED! 🚨 Centering Rudder.")
                if self.can_bus:
                    self.can_bus.send_rudder_command(config.CENTER_RUDDER_RAW)
                # TODO: Kill motor throttle here when implemented!
                return

            # 2. Read the planned path
            path_file = run_dir / 'path.csv'
            if not path_file.exists():
                return
                
            path_df = pd.read_csv(path_file)
            if len(path_df) < 2:
                # We are at the destination (or no path exists)
                if self.can_bus:
                    self.can_bus.send_rudder_command(config.CENTER_RUDDER_RAW)
                return

            # 3. Get the next waypoint (index 1)
            next_lat = path_df.iloc[1]['latitude']
            next_lon = path_df.iloc[1]['longitude']

            # 4. Calculate desired bearing
            target_bearing = self.calculate_bearing(curr_lat, curr_lon, next_lat, next_lon)

            # 5. Calculate heading error
            error = target_bearing - current_heading
            error = (error + 180) % 360 - 180 

            # 6. P-Controller math
            rudder_adjustment = int(error * config.P_GAIN)
            rudder_adjustment = max(-config.MAX_RUDDER_TURN, min(config.MAX_RUDDER_TURN, rudder_adjustment))
            target_raw_angle = config.CENTER_RUDDER_RAW + rudder_adjustment

            # 7. Send command
            if self.can_bus:
                success = self.can_bus.send_rudder_command(target_raw_angle)
                if success:
                    print(f"Target Bearing: {target_bearing:.1f}° | Error: {error:.1f}° | Sending Command: {target_raw_angle}")
            else:
                # Mock Mode Print
                print(f"[MOCK] Target Bearing: {target_bearing:.1f}° | Error: {error:.1f}° | Command: {target_raw_angle}")

        except Exception as e:
            print(f"Execution Error: {e}")