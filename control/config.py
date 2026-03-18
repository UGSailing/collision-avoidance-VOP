# path planning parameters
PATH_UPDATE_INTERVAL = 0.5 # seconds
GRID_RESOLUTION = 0.2 # meters per grid cell
GRID_SIZE = 200 # meters
HITBOX_RADIUS = 1 # meters around each obstacle point
SMOOTHING_TOLERANCE = 0.1  # for path waypoint number reduction
METERS_PER_DEGREE_LAT = 111320  # approximate meters per degree latitude

# path execution parameters
TARGET_RPM = 60  # TODO TEMP
TURNS_PER_SEC = TARGET_RPM / 60.0 # TODO TEMP
CENTER_RUDDER_RAW = 2048  # Assuming 2048 is straight ahead on the 0-4095 scale
MAX_RUDDER_TURN = 500     # Maximum raw units the rudder is allowed to turn left/right
P_GAIN = 5.0              # Proportional gain: How aggressively the boat steers towards the path

# devices
GPS_PORT = '/dev/ttyAMA0'
GPS_BAUD = 115200
GPS_UPDATE_RATE_HZ = 2 # Hz
READ_TIMEOUT = 0.1
CAN_CHANNEL = 'can0'
CAN_BUSTYPE = 'socketcan'
CAN_BITRATE = 1000000

# NTRIP Configuration (Unicore / RTK)
# username & password in env.py
NTRIP_ENABLED = True
NTRIP_HOST = "flepos.vlaanderen.be"
NTRIP_PORT = 2101
NTRIP_MOUNT = "FLEPOSVRS32GREC"
NTRIP_USE_SSL = False
NTRIP_SEND_GGA_EVERY = 10.0 # seconds
USER_HEADING_OFFSET_DEG = 0.0

# CAN control signals (boat_main.py autonomous mode)
# TODO fill in with actual values
CAN_GO_ID = 0x10    # arbitration ID that triggers run start
CAN_STOP_ID = 0x11  # arbitration ID that triggers run stop / restart

# geofencing
GEOFENCE_POND_ZWIJNAARDE = [
    (51.011556, 3.708620),
    (51.011556, 3.708620),
    (51.011323, 3.708454),
    (51.011343, 3.708491),
    (51.011435, 3.708564),
    (51.011423, 3.708698),
    (51.011423, 3.708698),
    (51.011293, 3.709374),
    (51.011293, 3.709374),
    (51.011293, 3.709374)
]

EXCLUSION_ZONES = [
    # Zone 1 vierkant
    [
        (51.01145, 3.70890), # Top Left
        (51.01145, 3.70900), # Top Right
        (51.01140, 3.70900), # Bottom Right
        (51.01140, 3.70890)  # Bottom Left
    ],

    # Zone 2: De driehoek
    [
        (51.0115, 3.7091), (51.0114, 3.7092), (51.0115, 3.7093)
    ]
]