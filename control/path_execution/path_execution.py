import pandas as pd
import math
import time
from can_communication import BoatCANInterface


"""
Dit is volledig gevibecode en gebaseerd op een commit in de github, controleer later
"""


# Initialize CAN bus for the execution thread
can_bus = BoatCANInterface(channel='can0', bustype='socketcan', bitrate=1000000)

# Hardware Constants (You will need to tune these!)
CENTER_RUDDER_RAW = 2048  # Assuming 2048 is straight ahead on the 0-4095 scale
MAX_RUDDER_TURN = 500     # Maximum raw units the rudder is allowed to turn left/right
P_GAIN = 5.0              # Proportional gain: How aggressively the boat steers towards the path

def calculate_bearing(lat1, lon1, lat2, lon2):
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

def get_current_heading_and_location(run_dir):
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

def follow_path(run_dir):
    """
    Reads the path, calculates steering error, and sends CAN command.
    """
    try:
        # 1. Get current state
        curr_lat, curr_lon, current_heading = get_current_heading_and_location(run_dir)
        if curr_lat is None:
            return # Waiting for GPS data

        # 2. Read the planned path
        path_file = run_dir / 'path.csv'
        if not path_file.exists():
            return
            
        path_df = pd.read_csv(path_file)
        if len(path_df) < 2:
            # We are at the destination (or no path exists)
            can_bus.send_rudder_command(CENTER_RUDDER_RAW)
            return

        # 3. Get the next waypoint (index 1, since index 0 is usually current location)
        next_lat = path_df.iloc[1]['latitude']
        next_lon = path_df.iloc[1]['longitude']

        # 4. Calculate desired bearing
        target_bearing = calculate_bearing(curr_lat, curr_lon, next_lat, next_lon)

        # 5. Calculate heading error (-180 to +180 degrees)
        error = target_bearing - current_heading
        # Normalize error to find the shortest turn direction
        error = (error + 180) % 360 - 180 

        # 6. Map error to rudder command using P-Controller
        # Negative error means turn left, positive means turn right (or vice versa depending on your mechanics)
        rudder_adjustment = int(error * P_GAIN)
        
        # Clamp the adjustment to your physical hardware limits
        rudder_adjustment = max(-MAX_RUDDER_TURN, min(MAX_RUDDER_TURN, rudder_adjustment))
        
        # Calculate final raw AS5600 target value
        target_raw_angle = CENTER_RUDDER_RAW + rudder_adjustment

        # 7. Send command to ESP32
        success = can_bus.send_rudder_command(target_raw_angle)
        
        if success:
            print(f"Target Bearing: {target_bearing:.1f}° | Error: {error:.1f}° | Sending Rudder Command: {target_raw_angle}")

    except Exception as e:
        print(f"Execution Error: {e}")




# def follow_path(run_dir):
#     global previous_error, last_time
    
#     try:
#         # 1. Get current state & planned path
#         curr_lat, curr_lon, current_heading = get_current_heading_and_location(run_dir)
#         if curr_lat is None:
#             return

#         path_file = run_dir / 'path.csv'
#         if not path_file.exists():
#             return
            
#         path_df = pd.read_csv(path_file)
#         if len(path_df) < 2:
#             can_bus.send_rudder_command(CENTER_RUDDER_RAW)
#             return

#         next_lat = path_df.iloc[1]['latitude']
#         next_lon = path_df.iloc[1]['longitude']

#         # 2. Calculate error
#         target_bearing = calculate_bearing(curr_lat, curr_lon, next_lat, next_lon)
#         error = target_bearing - current_heading
#         error = (error + 180) % 360 - 180 

#         # --- 3. PD CONTROLLER MATH ---
#         current_time = time.time()
#         dt = current_time - last_time
        
#         # Prevent division by zero if the loop runs insanely fast
#         if dt <= 0.0: 
#             dt = 0.001 

#         # Calculate Derivative: (Current Error - Previous Error) / Time Elapsed
#         derivative = (error - previous_error) / dt
        
#         # Calculate final adjustment
#         rudder_adjustment = int((P_GAIN * error) + (D_GAIN * derivative))
        
#         # Save current state for the next loop
#         previous_error = error
#         last_time = current_time
#         # ------------------------------

#         # 4. Clamp and Send
#         rudder_adjustment = max(-MAX_RUDDER_TURN, min(MAX_RUDDER_TURN, rudder_adjustment))
#         target_raw_angle = CENTER_RUDDER_RAW + rudder_adjustment

#         success = can_bus.send_rudder_command(target_raw_angle)
        
#         if success:
#             print(f"Error: {error:.1f}° | Deriv: {derivative:.1f} | Command: {target_raw_angle}")

#     except Exception as e:
#         print(f"Execution Error: {e}")
# alternatieve code voor aansturing, zou vloeiender moeten zijn, check later