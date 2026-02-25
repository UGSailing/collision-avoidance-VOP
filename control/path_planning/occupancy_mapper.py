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
        The grid is always centred on the latest GPS position so that the current
        boat location sits at the middle cell and the path planner uses it as start.
        """
        try:
            df = pd.read_csv(csv_path)
            if df.empty:
                return self.grid
        except Exception:
            return self.grid

        # 1. Pin the origin to the latest GPS point so the grid is centred there.
        #    This ensures get_grid_indices(latest_gps) == (center, center).
        gps_rows = df[df['category'] == 'gps']
        if not gps_rows.empty:
            latest_gps = gps_rows.loc[gps_rows['id'].idxmax()]
            self.origin_lat = latest_gps['latitude']
            self.origin_lon = latest_gps['longitude']

        # 2. Reset the grid to all zeros (empty)
        self.grid.fill(0)
        center = self.size_cells // 2

        # 3. Place Obstacles
        for _, row in df.iterrows():
            # In your project, 'camera' category represents obstacles/boundaries
            if row['category'] == 'camera':
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