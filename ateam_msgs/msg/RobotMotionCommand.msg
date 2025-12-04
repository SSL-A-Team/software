# Global velocity commands
# XY in translation (m/s)
# Z in rotation (rad/s)
geometry_msgs/Twist twist
geometry_msgs/Pose pose
uint8 twist_frame 0
uint8 kick_request
float32 kick_speed # (m/s)
float32 dribbler_speed # (rpm)

# kick options
uint8 KR_ARM = 0
uint8 KR_DISABLE = 1
uint8 KR_KICK_NOW = 2
uint8 KR_KICK_TOUCH = 3
uint8 KR_KICK_CAPTURED = 4
uint8 KR_CHIP_NOW = 5
uint8 KR_CHIP_TOUCH = 6
uint8 KR_CHIP_CAPTURED = 7

# frame options
uint8 FRAME_WORLD = 0
uint8 FRAME_BODY = 1
