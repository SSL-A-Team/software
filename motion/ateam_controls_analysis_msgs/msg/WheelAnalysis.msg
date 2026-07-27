# Controls analysis signals for a single wheel motor.
#
# Derived by ateam_controls_analysis from one CcmTelemetry motor inside
# ateam_radio_msgs/ExtendedTelemetry (front_left / back_left / back_right /
# front_right). All fields are float32; NaN means "no data at this instant"
# (a gap in PlotJuggler). Wheel telemetry is always present (it is not gapped
# by body control mode), so these fields are only NaN when the underlying
# telemetry is missing (e.g. an empty current-sample buffer).
#
# The two stacked wheel views in the bundled PlotJuggler layout are:
#   Velocity : wheel_vel_rads vs. vel_setpoint_rads  (vel / vel_setpoint)
#   Current  : mean measured current vs. setpoint    (current / current_setpoint)

# --- Velocity view (rad/s) ---
float32 vel                 # measured wheel velocity (CcmVelocityTelemetry.wheel_vel_rads)
float32 vel_setpoint        # commanded wheel velocity setpoint (CcmVelocityTelemetry.vel_setpoint_rads)

# --- Current view (mA) ---
float32 current             # mean measured current (mean of CcmCurrentTelemetry.current_samples_ma; those samples are unsigned magnitudes, so the sign of current_setpoint is applied to recover direction)
float32 current_setpoint    # commanded current setpoint (CcmCurrentTelemetry.current_setpoint_ma)
