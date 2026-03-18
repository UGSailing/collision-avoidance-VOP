# path planning parameters
PATH_UPDATE_INTERVAL = 0.5 # seconds
GRID_RESOLUTION = 0.2 # meters per grid cell
GRID_SIZE = 100 # meters
HITBOX_RADIUS = 3 # meters around each obstacle point
SMOOTHING_TOLERANCE = 0.1  # for path waypoint number reduction
METERS_PER_DEGREE_LAT = 111320  # approximate meters per degree latitude

# path execution parameters
CENTER_RUDDER_RAW = 2048  # Assuming 2048 is straight ahead on the 0-4095 scale
MAX_RUDDER_TURN = 500     # Maximum raw units the rudder is allowed to turn left/right
P_GAIN = 5.0              # Proportional gain: How aggressively the boat steers towards the path

# devices
GPS_PORT = '/dev/ttyACM0'
GPS_BAUDRATE = 115200
READ_TIMEOUT = 0.1
CAN_CHANNEL = 'can0'
CAN_BUSTYPE = 'socketcan'
CAN_BITRATE = 1000000

# geofencing
GEOFENCE_GPS = [      # basic geofencing test boundaries
    (51.0116, 3.7085), # Top Left
    (51.011379, 3.709218), # Top Right
    (51.0112, 3.7095), # Bottom Right
    (51.0112, 3.7085),  # Bottom Left
    (51.0114, 3.7080)  # testpoint
]

EXCLUSION_ZONES = [
    # Zone 1: Het vierkant, nu precies in het midden tussen de vorige twee locaties
    [
        (51.01145, 3.70890), # Top Left
        (51.01145, 3.70900), # Top Right
        (51.01140, 3.70900), # Bottom Right
        (51.01140, 3.70890)  # Bottom Left
    ], # <--- Belangrijk: De komma hier scheidt Zone 1 van Zone 2

    # Zone 2: De driehoek (onaangepast)
    [
        (51.0115, 3.7091), (51.0114, 3.7092), (51.0115, 3.7093)
    ]
]
