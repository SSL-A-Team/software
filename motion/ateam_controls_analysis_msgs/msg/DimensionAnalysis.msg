# Controls analysis signals for a single dimension (x, y, or theta).
#
# All fields are float32; NaN means "no data at this instant" (a gap in
# PlotJuggler). Fields that are not physically meaningful for a given dimension
# stay NaN (e.g. vel_gyro is populated only for theta; accel_imu only for x/y).
#
# The three stacked plots for a dimension are:
#   position     (1st derivative): *_pos_*
#   velocity     (2nd derivative): *_vel_*
#   acceleration (3rd derivative): *_accel_*

# --- position plot (1st derivative) ---
float32 pos_estimate        # KF body position estimate
float32 pos_traj            # reference trajectory position setpoint
float32 pos_vision          # vision position measurement
float32 pos_cmd             # software command (BCM_GLOBAL_POSITION)

# --- velocity plot (2nd derivative) ---
float32 vel_estimate        # KF body velocity estimate
float32 vel_traj            # reference trajectory velocity setpoint
float32 vel_cmd             # software command (BCM_GLOBAL_VELOCITY / BCM_LOCAL_VELOCITY)
float32 vel_gyro            # gyro angular-rate measurement (theta only)

# --- acceleration plot (3rd derivative) ---
float32 accel_u             # firmware body accel control output
float32 accel_u_fric_comp   # firmware body accel output with friction compensation
float32 accel_imu           # IMU acceleration measurement (x/y only)
float32 accel_cmd           # software command (BCM_GLOBAL_ACCEL / BCM_LOCAL_ACCEL)

# --- position-plot reference trajectory, split per active mode (color-by-mode) ---
float32 pos_traj_global_pos
float32 pos_traj_global_vel
float32 pos_traj_local_vel
float32 pos_traj_heading_pivot
float32 pos_traj_point_pivot
float32 pos_traj_heading_line
float32 pos_traj_point_line

# --- velocity-plot reference trajectory, split per active mode (color-by-mode) ---
float32 vel_traj_global_pos
float32 vel_traj_global_vel
float32 vel_traj_local_vel
float32 vel_traj_heading_pivot
float32 vel_traj_point_pivot
float32 vel_traj_heading_line
float32 vel_traj_point_line

# --- software command, split per mode, on its mapped derivative ---
float32 pos_cmd_global_pos
float32 vel_cmd_global_vel
float32 vel_cmd_local_vel
float32 accel_cmd_global_acc
float32 accel_cmd_local_acc
