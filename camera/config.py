
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