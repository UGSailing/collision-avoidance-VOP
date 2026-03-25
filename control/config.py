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

# CAN
CAN_CHANNEL = 'can0'
CAN_BUSTYPE = 'socketcan'
CAN_BITRATE = 1000000
CAN_OBSTACLE_ID = 0x120
CAN_OBSTACLE_IS_EXTENDED_ID = False
CAN_OBSTACLE_DLC = 4
CAN_OBSTACLE_ANGLE_SIGNED = True
CAN_OBSTACLE_DISTANCE_SIGNED = False
CAN_OBSTACLE_BYTEORDER = 'big'
CAN_OBSTACLE_ANGLE_SCALE_DEG_PER_LSB = 0.1
CAN_OBSTACLE_DISTANCE_SCALE_M_PER_LSB = 0.01
CAN_STEERING_FEEDBACK_ID = 0x01
CAN_STEERING_COMMAND_ID = 0x02
CAN_STEERING_FRAME_DLC = 2

# GPS
GPS_PORT = '/dev/ttyAMA0'
GPS_BAUD = 115200
GPS_UPDATE_RATE_HZ = 2 # Hz

# NTRIP Configuration (Unicore / RTK)
# username & password in env.py
NTRIP_ENABLED = True
NTRIP_HOST = "flepos.vlaanderen.be"
NTRIP_PORT = 2101
NTRIP_MOUNT = "FLEPOSVRS32GREC"
NTRIP_USE_SSL = False
NTRIP_SEND_GGA_EVERY = 10.0 # seconds
USER_HEADING_OFFSET_DEG = 0.0

# geofencing
GEOFENCE_POND_ZWIJNAARDE = [
    (51.011512, 3.708449),
    (51.011328, 3.708441),
    (51.011325, 3.708473),
    (51.011439, 3.708548),
    (51.011423, 3.708703),
    (51.011369, 3.708770),
    (51.011305, 3.708891),
    (51.011295, 3.709387),
    (51.011450, 3.709382),
    (51.011554, 3.708612)
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