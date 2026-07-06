uint8 body_control_mode
Twist2D pose  # (m, rad)
Twist2D velocity  # (m/s, rad/s)
Twist2D acceleration  # (m/s/s, rad/s/s)
float32 limit_vel_linear  # (m/s)
float32 limit_vel_angular  # (rad/s)
float32 limit_acc_linear  # (m/s/s)
float32 limit_acc_angular  # (rad/s/s)

# (BCM_HEADING_PIVOT)
float32 pivot_global_theta  # (rad)
# (BCM_POINT_PIVOT)
float32 pivot_target_x  # (m)
float32 pivot_target_y  # (m)
# (BCM_HEADING_PIVOT / BCM_POINT_PIVOT)
float32 pivot_max_angular_vel  # (rad/s)
float32 pivot_max_angular_acc  # (rad/s/s)
float32 pivot_orbit_radius  # (m)
float32 pivot_inset_angle  # (rad)
uint8 pivot_direction  # orbit drive direction: PD_FORWARD=0, PD_BACKWARD=1
bool pivot_compute_inset_angle  # if true, firmware derives inset_angle from peak angular vel

# (BCM_HEADING_LINE)
float32 line_global_theta  # (rad) maintained global heading
# (BCM_HEADING_POINT)
float32 line_target_x  # (m) face this point while dynamically moving along the line
float32 line_target_y  # (m)
# (BCM_HEADING_LINE / BCM_POINT_LINE)
float32 line_start_x  # (m) a point the line passes through
float32 line_start_y  # (m)
float32 line_dir_x  # line direction (normalized by firmware)
float32 line_dir_y
float32 line_velocity  # target velocity along the line (m/s)
float32 line_max_vel_colinear  # maximum velocity in the colinear direction (m/s) (NOTE: this should just be set to the requested line_velocity)
float32 line_max_vel_perp  # maximum velocity in the perpendicular direction (m/s)
float32 line_max_vel_angular  # (m/s)
float32 line_max_accel_colinear  # maximum velocity in the colinear direction (m/s)
float32 line_max_accel_perp  # maximum velocity in the perpendicular direction (m/s)
float32 line_max_accel_angular  # (m/s)
float32 line_colinear_start_thresh  # perp distance to begin colinear accel (m)

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
