import pandas as pd
import math
import config
import can

class PathFollower:
    def __init__(self, run_dir):
        self.run_dir = run_dir
        self.can_bus = can.interface.Bus(
            channel=config.CAN_CHANNEL,
            bustype=config.CAN_BUSTYPE,
            bitrate=config.CAN_BITRATE,
        )

    def calculate_bearing(self,lat1, lon1, lat2, lon2):
        """
        Calculates the true bearing (in degrees) from point 1 to point 2.
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)

        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)

        initial_bearing = math.atan2(y, x)
        # Convert to 0-360 degrees
        bearing = (math.degrees(initial_bearing) + 360) % 360
        return bearing

    def get_current_heading_and_location(self, run_dir):
        try:
            df = pd.read_csv(run_dir / 'points.csv')
            gps_points = df[df['category'] == 'gps']
            
            if len(gps_points) < 1:
                return None, None, None 
                
            # Grab the absolute latest GPS row
            current_loc = gps_points.loc[gps_points['id'].idxmax()]
            
            # Now we extract the actual true heading from the simpleRTK3B!
            current_heading = current_loc.get('heading', None)
            
            # Safety check: if the GPS hasn't locked a heading yet
            if pd.isna(current_heading):
                return None, None, None
                
            return current_loc['latitude'], current_loc['longitude'], current_heading
        except (ValueError, KeyError, FileNotFoundError):
            return None, None, None

    def follow_path(self, run_dir):
        """
        Reads the path, calculates steering error, and sends CAN command.
        """
        try:
            # 1. Get current state
            curr_lat, curr_lon, current_heading = self.get_current_heading_and_location(run_dir)
            if curr_lat is None:
                return # Waiting for GPS data

            # 2. Read the planned path
            path_file = run_dir / 'path.csv'
            if not path_file.exists():
                return
                
            path_df = pd.read_csv(path_file)
            if len(path_df) < 2:
                # We are at the destination (or no path exists)
                self.can_bus.send_rudder_command(config.CENTER_RUDDER_RAW)
                return

            # 3. Get the next waypoint (index 1, since index 0 is usually current location)
            next_lat = path_df.iloc[1]['latitude']
            next_lon = path_df.iloc[1]['longitude']

            # 4. Calculate desired bearing
            target_bearing = self.calculate_bearing(curr_lat, curr_lon, next_lat, next_lon)

            # 5. Calculate heading error (-180 to +180 degrees)
            error = target_bearing - current_heading
            # Normalize error to find the shortest turn direction
            error = (error + 180) % 360 - 180 

            # 6. Map error to rudder command using P-Controller
            # Negative error means turn left, positive means turn right (or vice versa depending on your mechanics)
            rudder_adjustment = int(error * config.P_GAIN)
            
            # Clamp the adjustment to your physical hardware limits
            rudder_adjustment = max(-config.MAX_RUDDER_TURN, min(config.MAX_RUDDER_TURN, rudder_adjustment))
            
            # Calculate final raw AS5600 target value
            target_raw_angle = config.CENTER_RUDDER_RAW + rudder_adjustment

            # 7. Send command to ESP32
            success = self.can_bus.send_rudder_command(target_raw_angle)
            
            if success:
                print(f"Target Bearing: {target_bearing:.1f}° | Error: {error:.1f}° | Sending Rudder Command: {target_raw_angle}")

        except Exception as e:
            print(f"Execution Error: {e}")