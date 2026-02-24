def get_path_points(df):
    gps_points = df[df['category'] == 'gps']
    current_location = gps_points.loc[gps_points['id'].idxmax()]
    destination_points = df[df['category'] == 'destination']
    current_destination = destination_points.loc[destination_points['id'].idxmax()]
    
    res = [(current_location['latitude'], current_location['longitude']), (current_destination['latitude'], current_destination['longitude'])]
    return res