# Global velocity commands
# XY in translation (m/s)
# Z in rotation (rad/s)
geometry_msgs/Twist twist
uint8 kick
float32 kick_speed # (m/s)
float32 dribbler_speed # (rpm)

# kick options
uint8 KICK_ARM = 0
uint8 KICK_DISABLE = 1
uint8 KICK_NOW = 2
uint8 KICK_ON_TOUCH = 3
uint8 KICK_ON_CAPTURE = 4
