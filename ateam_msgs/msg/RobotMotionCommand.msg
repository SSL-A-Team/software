uint8 body_control_mode
Twist2D pose  # (m, rad)
Twist2D velocity  # (m/s, rad/s)
Twist2D acceleration  # (m/s/s, rad/s/s)
float64 limit_vel_linear  # (m/s)
float64 limit_vel_angular  # (rad/s)
float64 limit_acc_linear  # (m/s/s)
float64 limit_acc_angular  # (rad/s/s)

# pivot parameters
float64 pivot_target_x # (m)
float64 pivot_target_y # (m)
float64 pivot_global_theta # (rad)
float64 pivot_orbit_radius  # (m)
float64 pivot_inset_angle  # (rad)
uint8 pivot_direction  # orbit drive direction: PD_FORWARD=0, PD_BACKWARD=1
bool pivot_compute_inset_angle  # if true, firmware derives inset_angle from peak angular vel

# line parameters (BCM_HEADING_LINE / BCM_POINT_LINE)
float64 line_start_x  # (m) a point the line passes through
float64 line_start_y  # (m)
float64 line_dir_x  # line direction (normalized by firmware)
float64 line_dir_y
float64 line_velocity  # target velocity along the line (m/s)
float64 line_target_x  # (m) global point to face (BCM_POINT_LINE)
float64 line_target_y  # (m)
float64 line_colinear_start_thresh  # perp distance to begin colinear accel (m)

uint8 kick_request
float32 kick_speed  # (m/s)
float32 dribbler_setpoint  # factor of full effort 0.0 - 1.0
uint8 dribbler_mode  # DC_DISABLE=0, DC_HARD_RECEIVE=1, DC_SOFT_RECEIVE=2, DC_DRIBBLE=3, DC_VELOCITY=4, DC_CURRENT=5

# kick options
uint8 KR_ARM = 0
uint8 KR_DISABLE = 1
uint8 KR_KICK_NOW = 2
uint8 KR_KICK_TOUCH = 3
uint8 KR_KICK_CAPTURED = 4
uint8 KR_CHIP_NOW = 5
uint8 KR_CHIP_TOUCH = 6
uint8 KR_CHIP_CAPTURED = 7

# pivot direction (orbit drive direction)
uint8 PD_FORWARD = 0
uint8 PD_BACKWARD = 1

# dribbler modes
uint8 DC_DISABLE = 0
uint8 DC_HARD_RECEIVE = 1
uint8 DC_SOFT_RECEIVE = 2
uint8 DC_DRIBBLE = 3
uint8 DC_VELOCITY = 4
uint8 DC_CURRENT = 5

# body control modes
uint8 BCM_OFF = 0
uint8 BCM_ESTOP_BRAKE = 1
uint8 BCM_GLOBAL_POSITION = 10
uint8 BCM_GLOBAL_VELOCITY = 11
uint8 BCM_LOCAL_VELOCITY = 12
uint8 BCM_GLOBAL_ACCEL = 13
uint8 BCM_LOCAL_ACCEL = 14
uint8 BCM_HEADING_PIVOT = 20
uint8 BCM_POINT_PIVOT = 21
uint8 BCM_HEADING_LINE = 30
uint8 BCM_POINT_LINE = 31
