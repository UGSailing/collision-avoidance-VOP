"""
    A simple path planner that reads the current location and destination from points.csv, and writes a path to path.csv. 
    The path is currently just a straight line between the latest GPS point and the latest destination point, so it ignores objects for now.
"""

import pandas as pd

def update_path(run_dir, grid, mapper):
    """Recomputes the path from points.csv and write it to path.csv."""
    path = get_path_points(pd.read_csv(run_dir / 'points.csv'))
    pd.DataFrame(path, columns=['latitude', 'longitude']).to_csv(run_dir / 'path.csv', index=False)

def get_path_points(df):
    gps_points = df[df['category'] == 'gps']
    current_location = gps_points.loc[gps_points['id'].idxmax()]
    destination_points = df[df['category'] == 'destination']
    current_destination = destination_points.loc[destination_points['id'].idxmax()]
    
    res = [(current_location['latitude'], current_location['longitude']), (current_destination['latitude'], current_destination['longitude'])]
    return res