# Global velocity commands
# XY in translation (m/s)
# Z in rotation (rad/s)
geometry_msgs/Twist twist
int8 kick # 0 = arm, 1 = kick on breakbeam, 2 = kick now
float32 kick_speed # (m/s)
float32 dribbler_speed # (rpm)
