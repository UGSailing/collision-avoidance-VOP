import serial
import pynmea2
import pandas as pd
import time
from pathlib import Path
from path_execution import BoatCANInterface

# 1. Initialize CAN Bus
can_bus = BoatCANInterface(channel='can0', bustype='socketcan', bitrate=1000000)

# 2. Initialize GPS Serial Connection
# NOTE: Change '/dev/ttyACM0' to your actual simpleRTK3B port (e.g., 'COM3' on Windows)
# 115200 is the standard baud rate for Ardusimple boards
try:
    gps_serial = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.1)
    print("GPS Serial connection established.")
except serial.SerialException as e:
    print(f"Warning: GPS not connected. {e}")
    gps_serial = None

# We store the latest known values here to bundle them together
boat_state = {
    'latitude': None,
    'longitude': None,
    'heading': None,
    'rudder_raw': None
}

def read_CAN_and_GPS(run_dir):
    """
    Reads hardware sensors and updates the state.
    Called continuously by the collection thread in main.py.
    """
    global boat_state
    updated_gps = False

    # --- READ RUDDER ANGLE OVER CAN ---
    raw_angle = can_bus.read_angle_message(timeout=0.01)
    if raw_angle is not None:
        boat_state['rudder_raw'] = raw_angle

    # --- READ LOCATION & HEADING FROM simpleRTK3B ---
    if gps_serial and gps_serial.in_waiting > 0:
        # Read all available lines in the buffer
        while gps_serial.in_waiting > 0:
            try:
                line = gps_serial.readline().decode('ascii', errors='replace').strip()
                if line.startswith('$'):
                    msg = pynmea2.parse(line)
                    
                    # Extract Location
                    if msg.sentence_type in ['GGA', 'RMC']:
                        boat_state['latitude'] = msg.latitude
                        boat_state['longitude'] = msg.longitude
                        updated_gps = True
                        
                    # Extract True Heading
                    elif msg.sentence_type == 'HDT':
                        boat_state['heading'] = float(msg.heading)
                        updated_gps = True
                        
            except pynmea2.ParseError:
                pass # Skip partial or corrupted NMEA lines

    # --- SAVE TO CSV IF WE HAVE DATA ---
    # We only write to the CSV if we actually have valid GPS coordinates
    if updated_gps and boat_state['latitude'] is not None:
        update_points_csv(run_dir)

def update_points_csv(run_dir):
    """
    Appends the latest boat state to points.csv so the planner and execution
    threads can access the current position and heading.
    """
    points_path = run_dir / 'points.csv'
    
    try:
        # Read the existing CSV
        df = pd.read_csv(points_path)
        
        # Figure out the next ID
        next_id = df['id'].max() + 1 if not df.empty else 0
        
        # Create the new GPS row
        new_row = pd.DataFrame([{
            'id': next_id,
            'category': 'gps',
            'latitude': boat_state['latitude'],
            'longitude': boat_state['longitude'],
            'heading': boat_state['heading']  # Add the heading to the CSV!
        }])
        
        # Append and save
        df = pd.concat([df, new_row], ignore_index=True)
        df.to_csv(points_path, index=False)
        
    except Exception as e:
        print(f"Error updating points.csv: {e}")

# In main.py, your collection_loop will just call:
def read_CAN(run_dir):
    read_CAN_and_GPS(run_dir)