# configurable parameters
PATH_UPDATE_INTERVAL = 3 # seconds
GRID_RESOLUTION = 0.2 # meters per grid cell
GRID_SIZE = 100 # meters
HITBOX_RADIUS = 3 # meters around each obstacle point
SMOOTHING_TOLERANCE = 0.1  # for path waypoint number reduction
GEOFENCE_GPS = [      # basic geofencing test boundaries
    (51.0116, 3.7085), # Top Left
    (51.0116, 3.7095), # Top Right
    (51.0112, 3.7095), # Bottom Right
    (51.0112, 3.7085),  # Bottom Left
    (51.0114, 3.7080)  # testpoint
]

# constants
METERS_PER_DEGREE_LAT = 111320  # approximate meters per degree latitude
