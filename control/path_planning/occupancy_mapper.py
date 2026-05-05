"""
    The OccupancyMapper class reads GPS coordinates of obstacles from a CSV file and maintains an occupancy grid that represents the environment around the boat. 
    It converts GPS coordinates to local XY coordinates, rasterizes them onto a grid, and applies dilation to create hitboxes around obstacles. 
    The mapper also handles incremental updates efficiently by only processing new CSV rows and re-centering the grid when the boat's GPS origin shifts significantly. 
    Additionally, it applies geofencing to mark areas outside the pond or within exclusion zones as blocked.
"""

import numpy as np
import pandas as pd
from scipy.ndimage import binary_dilation
import config
from matplotlib.path import Path

class OccupancyMapper:
    def __init__(self, resolution=0.2, grid_size=50, hitbox_radius=0.3) -> None:
        """
        resolution: meters per pixel (e.g., 0.2 = 20cm per cell).
        grid_size_m: total width and height of the map square in meters.
        """
        self.res = resolution
        self.size_cells = int(grid_size / resolution)
        self.grid = np.zeros((self.size_cells, self.size_cells))
        self.hitbox_radius = hitbox_radius
        
        # reference point (Origin) to convert large GPS coordinates into small local meters
        self.origin_lat = None
        self.origin_lon = None

        # --- incremental update state ---
        # all obstacle GPS coords ever seen (lat, lon); used to re-rasterize on origin shift
        self._known_obstacle_gps: set[tuple] = set()
        # how many CSV rows have already been processed
        self._rows_processed: int = 0
        # threshold: only trigger a full rebuild when the origin moves more than this many cells
        self._origin_shift_threshold_cells: float = 0.5

    def _gps_to_local(self, lat, lon) -> tuple[float, float]:
        """
        Converts GPS latitude/longitude to local XY meters relative to the first 
        point the mapper ever saw.
        """
        if self.origin_lat is None:
            self.origin_lat, self.origin_lon = lat, lon
            
        # standard equirectangular projection
        y = (lat - self.origin_lat) * config.METERS_PER_DEGREE_LAT
        # longitude length varies based on latitude
        x = (lon - self.origin_lon) * (config.METERS_PER_DEGREE_LAT * np.cos(np.radians(self.origin_lat)))
        return x, y

    def _get_grid_indices(self, lat, lon) -> tuple[int, int]:
        """
        Converts a GPS coordinate to specific (row, col) indices on the current grid matrix.
        Useful for checking if a specific GPS goal is 'blocked'.
        """
        x_m, y_m = self._gps_to_local(lat, lon)
        center = self.size_cells // 2
        
        gx = int(x_m / self.res) + center
        gy = int(y_m / self.res) + center
        
        return gy, gx  # returns Row (Y), Column (X)

    def _latest_gps_origin(self, df: pd.DataFrame) -> tuple:
        """Returns (lat, lon) of the latest GPS row in df, or (None, None) if absent."""
        gps_rows = df[df['category'] == 'gps']
        if gps_rows.empty:
            return np.nan, np.nan
        latest_gps = gps_rows.loc[gps_rows['id'].idxmax()]
        return latest_gps['latitude'], latest_gps['longitude']

    def _build_dilation_struct(self) -> np.ndarray:
        """
        Creates a circular hitbox struct a.k.a. dilation struct to put on the obstacle points
        """
        hitbox_cells = int(self.hitbox_radius / self.res)
        if hitbox_cells == 0:
            return np.ones((1, 1), dtype=bool)  # no dilation if hitbox smaller than cell size
        y, x = np.ogrid[-hitbox_cells:hitbox_cells+1, -hitbox_cells:hitbox_cells+1]
        return x**2 + y**2 <= hitbox_cells**2

    def _rasterize_obstacles(self, obstacle_gps_iter) -> np.ndarray:
        """Rasterize an iterable of (lat, lon) obstacle pairs into a fresh grid layer."""
        layer = np.zeros((self.size_cells, self.size_cells), dtype=bool)
        center = self.size_cells // 2
        for lat, lon in obstacle_gps_iter:
            x_m, y_m = self._gps_to_local(lat, lon)
            gx = int(x_m / self.res) + center
            gy = int(y_m / self.res) + center
            if 0 <= gx < self.size_cells and 0 <= gy < self.size_cells:
                layer[gy, gx] = True
        struct = self._build_dilation_struct() 
        layer = binary_dilation(layer, structure=struct) # transforms object points into hitboxes
        return layer.astype(float)

    def _is_boat_in_geofence(self,boat_lat, boat_lon):
        """Checks raw GPS against the geofence."""
        if not hasattr(config, 'GEOFENCE_GPS') or not config.GEOFENCE_POND_ZWIJNAARDE:
            return True # Default to safe if no geofence defined
            
        is_inside = False
        j = len(config.GEOFENCE_POND_ZWIJNAARDE) - 1
        for i in range(len(config.GEOFENCE_POND_ZWIJNAARDE)):
            lati, loni = config.GEOFENCE_POND_ZWIJNAARDE[i]
            latj, lonj = config.GEOFENCE_POND_ZWIJNAARDE[j]
            
            # Note: Y is Lat, X is Lon
            y_between = (lati > boat_lat) != (latj > boat_lat)
            if y_between:
                lon_intersect = loni + (boat_lat - lati) * (lonj - loni) / (latj - lati)
                if boat_lon < lon_intersect:
                    is_inside = not is_inside
            j = i
        return is_inside

    # def _apply_geofence(self):
    #     """Sets all grid cells outside the geofence to 1 (obstacle)."""
    #     if not hasattr(config, 'GEOFENCE_GPS') or not config.GEOFENCE_POND_ZWIJNAARDE:
    #         return

    #     # 1. Convert GPS geofence corners to local grid indices
    #     poly_grid = []
    #     center = self.size_cells // 2
    #     for lat, lon in config.GEOFENCE_POND_ZWIJNAARDE:
    #         x_m, y_m = self._gps_to_local(lat, lon)
    #         gx = int(x_m / self.res) + center
    #         gy = int(y_m / self.res) + center
    #         poly_grid.append((gx, gy))

    #     # 2. Iterate through the grid and mask out the boundary
    #     # Note: If this nested loop is too slow (e.g., Grid is 1000x1000), 
    #     # we can optimize this later using skimage.draw.polygon or matplotlib.path
    #     # 2. Create a coordinate grid (vectorized)
    #     # This creates a fast mathematical mesh of every X/Y coordinate in your array
    #     x, y = np.meshgrid(np.arange(self.size_cells), np.arange(self.size_cells))
    #     x, y = x.flatten(), y.flatten()
    #     points = np.vstack((x,y)).T

    #     # 3. Fast C-level ray-casting using matplotlib
    #     geofence_path = Path(poly_grid)
        
    #     # Returns a boolean array of what is INSIDE the geofence
    #     inside_mask = geofence_path.contains_points(points).reshape(self.size_cells, self.size_cells)

    #     # 4. Mask the grid: Any cell NOT inside (~), set to 1.0 (obstacle)
    #     self.grid[~inside_mask] = 1.0

    def create_grid(self, run_dir) -> np.ndarray:
        """
        Full rebuild: reads all of points.csv from scratch.
        Resets the incremental cursor so subsequent update_grid calls start fresh.
        """
        try:
            df = pd.read_csv(run_dir)
            if df.empty:
                return self.grid
        except Exception as e:
            print(f"Error reading CSV for grid creation: {e}")
            return self.grid

        # pin origin to latest GPS point
        lat, lon = self._latest_gps_origin(df)
        if not (np.isnan(lat) or np.isnan(lon)):
            self.origin_lat, self.origin_lon = lat, lon

        # rebuild known-obstacle set and reset cursor
        self._known_obstacle_gps = {
            (r['latitude'], r['longitude'])
            for _, r in df.iterrows() if r['category'] == 'camera'
        }
        self._rows_processed = len(df)

        self.grid = self._rasterize_obstacles(self._known_obstacle_gps)
        self._apply_geofence() #GEOFENCING TOEVOEGING
        return self.grid

    def _apply_geofence(self):
        """
        Sets all grid cells outside the pond OR inside exclusion zones 
        to 1.0 (blocked).
        """
        x, y = np.meshgrid(np.arange(self.size_cells), np.arange(self.size_cells))
        points = np.vstack((x.flatten(), y.flatten())).T
        center = self.size_cells // 2

        # --- 1. OUTER BOUNDARY ---
        fence_coords = getattr(config, 'GEOFENCE_POND_ZWIJNAARDE', None)
        if fence_coords:
            poly_grid = []
            for lat, lon in fence_coords:
                x_m, y_m = self._gps_to_local(lat, lon)
                gx = int(x_m / self.res) + center
                gy = int(y_m / self.res) + center
                poly_grid.append((gx, gy))

            geofence_path = Path(poly_grid)
            inside_main_mask = geofence_path.contains_points(points).reshape(self.size_cells, self.size_cells)
            self.grid[~inside_main_mask] = 1.0

        # --- 2. EXCLUSION ZONES ---
        exclusion_zones = getattr(config, 'EXCLUSION_ZONES', None)
        if exclusion_zones:
            for zone in exclusion_zones:
                zone_poly_grid = []
                for lat, lon in zone:
                    x_m, y_m = self._gps_to_local(lat, lon)
                    gx = int(x_m / self.res) + center
                    gy = int(y_m / self.res) + center
                    zone_poly_grid.append((gx, gy))
                
                exclusion_path = Path(zone_poly_grid)
                inside_exclusion_mask = exclusion_path.contains_points(points).reshape(self.size_cells, self.size_cells)
                self.grid[inside_exclusion_mask] = 1.0


#TEST GEOFENCING EINDE  

    def update_grid(self, run_dir) -> np.ndarray:
        """
        Incremental update: only processes CSV rows that are new since the last call.

        - If the boat's GPS origin has shifted more than `_origin_shift_threshold_cells`
          cells, a full re-rasterize of all known obstacles is done.
        - Otherwise only the genuinely new obstacle points are rasterized and OR-ed
          into the existing grid, leaving untouched cells as-is.
        """
        try:
            df = pd.read_csv(run_dir)
            if df.empty:
                return self.grid
        except Exception as e:
            print(f"Error reading CSV for grid update: {e}")
            return self.grid

        # determine new origin from latest GPS row
        old_origin_lat, old_origin_lon = self.origin_lat, self.origin_lon
        lat, lon = self._latest_gps_origin(df)
        if not (np.isnan(lat) or np.isnan(lon)):
            self.origin_lat, self.origin_lon = lat, lon

        # check whether the origin has shifted enough to matter
        origin_shifted = False
        if self.origin_lat is None or self.origin_lon is None:
            origin_shifted = True
        else:
            dy = (self.origin_lat - old_origin_lat) * config.METERS_PER_DEGREE_LAT / self.res
            dx = (self.origin_lon - old_origin_lon) * (config.METERS_PER_DEGREE_LAT * np.cos(np.radians(self.origin_lat))) / self.res
            if (dx**2 + dy**2) ** 0.5 > self._origin_shift_threshold_cells:
                origin_shifted = True

        # read only new CSV rows
        new_df = df.iloc[self._rows_processed:]
        self._rows_processed = len(df)

        new_obstacles = [
            (r['latitude'], r['longitude'])
            for _, r in new_df.iterrows() if r['category'] == 'camera'
        ]
        for obs in new_obstacles:
            self._known_obstacle_gps.add(obs)

        # rebuild or incrementally update
        if origin_shifted:
            # origin moved: re-center and re-rasterize everything from stored GPS coords
            self.grid = self._rasterize_obstacles(self._known_obstacle_gps)
        else:
            # revert to old origin
            self.origin_lat, self.origin_lon = old_origin_lat, old_origin_lon  
        
        if new_obstacles:
            # origin stable: just OR in the dilation of the new points only
            new_layer = self._rasterize_obstacles(new_obstacles)
            self.grid = np.maximum(self.grid, new_layer)
        # else: nothing new, grid is already up to date

        self._apply_geofence() #GEOFENCING TOEVOEGING
        return self.grid