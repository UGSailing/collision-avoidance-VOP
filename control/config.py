# path planning parameters
PATH_UPDATE_INTERVAL = 3 # seconds
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

# constants
