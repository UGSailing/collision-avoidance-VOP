import numpy as np
import pandas as pd
from scipy.ndimage import binary_dilation

class OccupancyMapper:
    def __init__(self, resolution=0.2, grid_size_m=50):
        """
        resolution: meters per pixel (e.g., 0.2 = 20cm per cell).
        grid_size_m: total width and height of the map square in meters.
        """
        self.res = resolution
        self.size_cells = int(grid_size_m / resolution)
        self.grid = np.zeros((self.size_cells, self.size_cells))
        
        # Reference point (Origin) to convert large GPS coordinates into small local meters
        self.origin_lat = None
        self.origin_lon = None

    def _gps_to_local(self, lat, lon):
        """
        Converts GPS latitude/longitude to local XY meters relative to the first 
        point the mapper ever saw.
        """
        if self.origin_lat is None:
            self.origin_lat, self.origin_lon = lat, lon
            
        # Standard equirectangular projection
        # 1 degree latitude is ~111,320 meters
        y = (lat - self.origin_lat) * 111320
        # Longitude length varies based on latitude
        x = (lon - self.origin_lon) * (111320 * np.cos(np.radians(self.origin_lat)))
        return x, y

    def get_grid_indices(self, lat, lon):
        """
        Converts a GPS coordinate to specific (row, col) indices on the current grid matrix.
        Useful for checking if a specific GPS goal is 'blocked'.
        """
        x_m, y_m = self._gps_to_local(lat, lon)
        center = self.size_cells // 2
        
        gx = int(x_m / self.res) + center
        gy = int(y_m / self.res) + center
        
        return gy, gx  # Returns Row (Y), Column (X)

    def create_grid(self, csv_path, hitbox_radius_m=0.6):
        """
        Reads the points.csv, projects GPS points to the grid, and applies hitboxes.
        """
        try:
            df = pd.read_csv(csv_path)
            if df.empty:
                return self.grid
        except Exception:
            return self.grid

        # 1. Reset the grid to all zeros (empty)
        self.grid.fill(0)
        center = self.size_cells // 2

        # 2. Place Obstacles
        for _, row in df.iterrows():
            # In your project, 'gps' category represents obstacles/boundaries
            if row['category'] == 'gps':
                x_m, y_m = self._gps_to_local(row['latitude'], row['longitude'])
                
                gx = int(x_m / self.res) + center
                gy = int(y_m / self.res) + center
                
                # Only plot if the point is within our defined grid bounds
                if 0 <= gx < self.size_cells and 0 <= gy < self.size_cells:
                    self.grid[gy, gx] = 1

        # 3. Apply Hitbox (Dilation)
        # This expands every '1' into a circle of radius hitbox_radius_m
        hitbox_cells = int(hitbox_radius_m / self.res)
        if hitbox_cells > 0:
            y, x = np.ogrid[-hitbox_cells:hitbox_cells+1, -hitbox_cells:hitbox_cells+1]
            # Create a circular mask for the hitbox
            struct = x**2 + y**2 <= hitbox_cells**2
            self.grid = binary_dilation(self.grid, structure=struct).astype(float)
            
        return self.grid

def get_path_points(df):
    """
    Helper function to extract current robot position and destination from the dataframe.
    """
    try:
        gps_points = df[df['category'] == 'gps']
        current_location = gps_points.loc[gps_points['id'].idxmax()]
        
        destination_points = df[df['category'] == 'destination']
        current_destination = destination_points.loc[destination_points['id'].idxmax()]
        
        return [
            (current_location['latitude'], current_location['longitude']), 
            (current_destination['latitude'], current_destination['longitude'])
        ]
    except (ValueError, KeyError):
        # Return None or empty if data isn't ready yet
        return None