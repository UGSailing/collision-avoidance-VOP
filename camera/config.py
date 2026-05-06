
# Ethernet obstacle output settings (camera Pi acts as TCP server).
# The control Pi connects as TCP client and expects newline-delimited payloads:
#   "<angle_deg>,<distance_m>" for one object
#   "<angle_deg>,<distance_m>;<angle_deg>,<distance_m>;..." for multiple
OBSTACLE_TCP_ENABLED = True
OBSTACLE_TCP_BIND_HOST = "0.0.0.0"
OBSTACLE_TCP_PORT = 9000
OBSTACLE_TCP_BACKLOG = 1
OBSTACLE_TCP_MAX_CLIENTS = 1
OBSTACLE_TCP_SEND_TIMEOUT_S = 0.2
OBSTACLE_TCP_SEND_EMPTY_UPDATES = True

OBSTACLE_VALUE_SEPARATOR = ","
OBSTACLE_PAIR_SEPARATOR = ";"
OBSTACLE_OUTPUT_DEBUG = False
