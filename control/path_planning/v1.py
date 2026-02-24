import pandas as pd

def get_path_points(df):
    gps_points = df[df['category'] == 'gps']
    current_location = gps_points.loc[gps_points['id'].idxmax()]
    destination_points = df[df['category'] == 'destination']
    current_destination = destination_points.loc[destination_points['id'].idxmax()]
    

    res = [(current_location['latitude'], current_location['longitude']), (current_destination['latitude'], current_destination['longitude'])]
    pd.DataFrame(res, columns=['latitude', 'longitude']).to_csv('mapping/path.csv', index=False)
    return res

if __name__ == "__main__":
    df = pd.read_csv('mapping/points.csv')
    get_path_points_v1(df)
