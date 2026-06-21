uint8 body_control_mode
Twist2D pose  # (m, rad)
Twist2D velocity  # (m/s, rad/s)
Twist2D acceleration  # (m/s/s, rad/s/s)
float64 limit_vel_linear  # (m/s)
float64 limit_vel_angular  # (rad/s)
float64 limit_acc_linear  # (m/s/s)
float64 limit_acc_angular  # (rad/s/s)

# pivot parameters
float64 pivot_orbit_radius  # (m)
float64 pivot_inset_angle  # (rad)

uint8 kick_request
float32 kick_speed  # (m/s)
float32 dribbler_speed  # (rpm)

# kick options
uint8 KR_ARM = 0
uint8 KR_DISABLE = 1
uint8 KR_KICK_NOW = 2
uint8 KR_KICK_TOUCH = 3
uint8 KR_KICK_CAPTURED = 4
uint8 KR_CHIP_NOW = 5
uint8 KR_CHIP_TOUCH = 6
uint8 KR_CHIP_CAPTURED = 7

# body control modes
uint8 BCM_OFF = 0
uint8 BCM_GLOBAL_POSITION = 1
uint8 BCM_GLOBAL_VELOCITY = 2
uint8 BCM_LOCAL_VELOCITY = 3
uint8 BCM_GLOBAL_ACCEL = 4
uint8 BCM_LOCAL_ACCEL = 5
uint8 BCM_PIVOT = 6
