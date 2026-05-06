# Path planning
PATH_UPDATE_INTERVAL = 0.5  # seconds
GRID_RESOLUTION = 0.2  # meters per grid cell
GRID_SIZE = 200  # meters
HITBOX_RADIUS = 1  # obstacle inflation radius in meters
SMOOTHING_TOLERANCE = 0.1  # waypoint reduction tolerance
METERS_PER_DEGREE_LAT = 111320  # approximate meters per degree latitude

# Path execution
PATH_EXECUTION_FREQUENCY_HZ = 2.0  # control loop frequency
MAX_RUDDER_ANGLE_DEG = 30.0  # safety limit
FIXED_THRUST = 0.1  # fixed thrust (0..1)
LOOKAHEAD_WAYPOINT_COUNT = 4  # future waypoints for blended heading
TURN_PREVIEW_SEGMENTS = 3  # path segments for turn feed-forward
TURN_FEEDFORWARD_GAIN = 0.25  # turn anticipation strength
HEADING_DEADBAND_DEG = 2.0  # ignore tiny heading errors
HEADING_ERROR_FOR_MAX_RUDDER_DEG = 45.0  # full rudder at this heading error
STEERING_AGGRESSIVENESS = 1.0  # >1.0 stronger response, <1.0 gentler
STEERING_DIRECTION = 1.0  # set to -1.0 if rudder sign is reversed

# Trajectory optimization (v3)
TRAJ_CONTROL_POINTS = 16
TRAJ_POINTS = 300
TRAJ_MIN_SPEED = 0.05
TRAJ_MAX_SPEED = 5.0
TRAJ_LOOKAHEAD_POINTS = 6
TRAJ_USE_SPEED_PROFILE = True
TRAJ_FALLBACK_SPEED = 0.4
TRAJ_HEADING_LEAD_METERS = 2.0
TRAJ_HEADING_ALIGN_DEG = 25.0
TRAJ_VERBOSE = False
TRAJ_EXECUTION_VERBOSE = False

# ESP32 serial
ESP_SERIAL_PORT = '/dev/ttyUSB0'
ESP_BAUDRATE = 115200
ESP_TIMEOUT = 0.1  # seconds

# Optional output fanout for mock_gps.py
ESP_SERIAL_PORTS = [
    # ESP_SERIAL_PORT,
    'socket://127.0.0.1:8765',
]

# Obstacle input over Ethernet TCP.
# Payload per line: "<angle_deg>,<distance_m>" pairs separated by ';'.
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
GPS_UPDATE_RATE_HZ = 2  # Hz

# NTRIP (Unicore / RTK). Credentials are in env.py.
NTRIP_ENABLED = True
NTRIP_HOST = "flepos.vlaanderen.be"
NTRIP_PORT = 2101
NTRIP_MOUNT = "FLEPOSVRS32GREC"
NTRIP_USE_SSL = False
NTRIP_SEND_GGA_EVERY = 10.0  # seconds
USER_HEADING_OFFSET_DEG = 0.0

# Geofencing
<<<<<<< Updated upstream
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
    (51.011554, 3.708612),
]

# GEOFENCE_POND_ZWIJNAARDE = [(51.045357, 3.682699),
#                             (51.045121, 3.682321),
#                             (51.044627, 3.681828),
#                             (51.044565, 3.682018),
#                             (51.044848, 3.682423),
                            # (51.045091, 3.683042)] #voor demo in blaarmeersen

=======
# GEOFENCE_POND_ZWIJNAARDE = [
#     (51.011512, 3.708449),
#     (51.011328, 3.708441),
#     (51.011325, 3.708473),
#     (51.011439, 3.708548),
#     (51.011423, 3.708703),
#     (51.011369, 3.708770),
#     (51.011305, 3.708891),
#     (51.011295, 3.709387),
#     (51.011450, 3.709382),
#     (51.011554, 3.708612),
# ]
>>>>>>> Stashed changes
EXCLUSION_ZONES = [
    # # Vierkant
    # [
    #     (51.01145, 3.70890), # Top Left
    #     (51.01145, 3.70900), # Top Right
    #     (51.01140, 3.70900), # Bottom Right
    #     (51.01140, 3.70890)  # Bottom Left
    # ],
    # # Driehoek
    # [
    #     (51.0115, 3.7091), (51.0114, 3.7092), (51.0115, 3.7093)
    # ],
    # Fountain
    [
        (51.011434, 3.708956),
        (51.011419, 3.709034),
        (51.011379, 3.709018),
        (51.011404, 3.708922),
    ]
]

# Default destination for autostart
DEFAULT_DESTINATION = (51.011387, 3.709251)