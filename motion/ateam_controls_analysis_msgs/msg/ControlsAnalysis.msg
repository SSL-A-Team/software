# Flat, PlotJuggler-friendly controls analysis signals for a single robot.
#
# Derived by ateam_controls_analysis from ateam_radio_msgs/ExtendedTelemetry.
# Grouped into one DimensionAnalysis per dimension (x, y, theta); each dimension
# stacks position / velocity / acceleration plots that share a time axis in the
# bundled PlotJuggler layout. All value fields are float32 (matching the float32
# source telemetry); NaN marks a gap.
#
# Body-control-mode -> derivative mapping for the software command (cmd):
#   BCM_GLOBAL_POSITION (10)                          -> position
#   BCM_GLOBAL_VELOCITY (11), BCM_LOCAL_VELOCITY (12) -> velocity
#   BCM_GLOBAL_ACCEL (13),    BCM_LOCAL_ACCEL (14)    -> acceleration
#   BCM_OFF (0), BCM_ESTOP_BRAKE (1), and pivot/line modes -> gap (no cmd)
# Local modes are rotated into the global frame using theta_estimate.

std_msgs/Header header

# Numeric active body control mode (see enum below); plottable as a step signal.
uint8 body_control_mode
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

# Robot heading estimate (rad); used for local->global rotation, also plottable.
float32 theta_estimate

# Reboot indicators (reconstructed robot-time axis). reboot_event pulses to 1.0
# on the first sample after a detected robot reboot (else 0.0); reboot_count is
# a monotonically increasing staircase of reboots seen so far.
float32 reboot_event
uint32 reboot_count

DimensionAnalysis x
DimensionAnalysis y
DimensionAnalysis theta
