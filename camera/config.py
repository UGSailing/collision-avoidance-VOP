
# CAN settings for camera -> control obstacle transmission.
# Keep these values aligned with control/config.py.
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

# Grouped obstacle CAN payloads; each object uses CAN_OBSTACLE_DLC bytes.
CAN_OBSTACLE_MAX_DLC = 8

# Ethernet obstacle output settings (camera Pi acts as TCP server).
# The control Pi connects as TCP client and expects newline-delimited payloads:
#   "<angle_deg>,<distance_m>" for one object
#   "<angle_deg>,<distance_m>;<angle_deg>,<distance_m>;..." for multiple
OBSTACLE_TCP_ENABLED = True
OBSTACLE_TCP_BIND_HOST = '0.0.0.0'
OBSTACLE_TCP_PORT = 9000
OBSTACLE_TCP_BACKLOG = 1
OBSTACLE_TCP_MAX_CLIENTS = 1
OBSTACLE_TCP_SEND_TIMEOUT_S = 0.2

OBSTACLE_VALUE_SEPARATOR = ','
OBSTACLE_PAIR_SEPARATOR = ';'
OBSTACLE_OUTPUT_DEBUG = False

# Optional USB serial obstacle output (legacy).
USB_DEVICE = None  # Example: '/dev/ttyUSB1'