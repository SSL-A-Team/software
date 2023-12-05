uint8 FRONT_LEFT_MOTOR = 0
uint8 FRONT_RIGHT_MOTOR = 1
uint8 BACK_RIGHT_MOTOR = 2
uint8 BACK_LEFT_MOTOR = 3

ateam_msgs/RobotMotorFeedback[4] motors

geometry_msgs/Twist body_velocity_setpoint
geometry_msgs/Twist clamped_body_velocity_setpoint
geometry_msgs/Twist body_velocity_state_estimate
geometry_msgs/Twist body_velocity_control_variable

float32[4] wheel_velocity_control_variable
float32[4] clamped_wheel_velocity_control_variable