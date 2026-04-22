# path planning parameters
PATH_UPDATE_INTERVAL = 0.5 # seconds
GRID_RESOLUTION = 0.2 # meters per grid cell
GRID_SIZE = 200 # meters
HITBOX_RADIUS = 1 # meters around each obstacle point
SMOOTHING_TOLERANCE = 0.1  # for path waypoint number reduction
METERS_PER_DEGREE_LAT = 111320  # approximate meters per degree latitude

# path execution parameters
PATH_EXECUTION_FREQUENCY_HZ = 2.0 # control loop frequency in Hz
MAX_RUDDER_ANGLE_DEG = 30.0 # Maximum rudder angle in degrees for safety limits
FIXED_THRUST = 0.1        # Fixed thrust level (0 to 1) for path following
LOOKAHEAD_WAYPOINT_COUNT = 4       # Number of future waypoints used for blended heading
TURN_PREVIEW_SEGMENTS = 3          # Number of path segments used for turn feed-forward
TURN_FEEDFORWARD_GAIN = 0.25       # How strongly upcoming turn curvature biases heading
HEADING_DEADBAND_DEG = 2.0         # Ignore tiny heading errors to reduce rudder chatter
HEADING_ERROR_FOR_MAX_RUDDER_DEG = 45.0  # Heading error (deg) that maps to full rudder
STEERING_AGGRESSIVENESS = 1.0      # >1.0 more aggressive, <1.0 more gentle steering response
STEERING_DIRECTION = 1.0          # Flip if rudder sign is reversed on hardware

# ESP32 Serial Configuration
ESP_SERIAL_PORT = '/dev/ttyUSB0'  # Adjust as needed for your system
ESP_BAUDRATE = 115200
ESP_TIMEOUT = 0.1  # seconds

# Optional autopilot output fanout. Path execution will send the same "angle,thrust\n" command to each configured endpoint.
# Keep only ESP_SERIAL_PORT for normal operation, or add a socket endpoint to feed the indoor simulator simultaneously (mock_gps.py).
ESP_SERIAL_PORTS = [
    ESP_SERIAL_PORT,
    # 'socket://127.0.0.1:8765',
]

# Obstacle input over direct Ethernet connection (TCP)
# The control Pi connects as a TCP client to the server object-detection Pi.
#
# Message framing:
# - One UTF-8/ASCII line per update, newline-terminated ("\n").
# - Empty lines are ignored.
#
# Payload format for each line:
# - One object: "<angle_deg>,<distance_m>"
# - Multiple objects: "<angle_deg>,<distance_m>;<angle_deg>,<distance_m>;..."
OBSTACLE_TCP_HOST = '192.168.50.2'
OBSTACLE_TCP_PORT = 9000
OBSTACLE_TCP_READ_TIMEOUT_S = 1.0
OBSTACLE_RECONNECT_DELAY_S = 2.0
OBSTACLE_VALUE_SEPARATOR = ','
OBSTACLE_PAIR_SEPARATOR = ';'
OBSTACLE_INPUT_DEBUG = False

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
# EXCLUSION_ZONES = [
#     # Zone 1 vierkant
#     [
#         (51.01145, 3.70890), # Top Left
#         (51.01145, 3.70900), # Top Right
#         (51.01140, 3.70900), # Bottom Right
#         (51.01140, 3.70890)  # Bottom Left
#     ],

#     # Zone 2: De driehoek
#     [
#         (51.0115, 3.7091), (51.0114, 3.7092), (51.0115, 3.7093)
#     ]
# ]