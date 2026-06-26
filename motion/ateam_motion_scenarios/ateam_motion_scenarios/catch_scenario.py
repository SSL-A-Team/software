"""
Catch scenario node for profiling robot motion performance.

Hardcoded to robot 2 on the blue team. A ball is rolled in from the +x
edge of the field. The robot continuously predicts where the ball is
headed and drives to intercept along the predicted line.

Three intercept strategies (`mode` parameter):

  * ``simple``  - The robot holds its starting x coordinate and only
                  translates along y until its dribbler lands on the
                  ball's predicted line, facing directly back down that
                  line. End of trajectory has zero velocity.
  * ``medium``  - The intercept point is the end of the robot's trajectory
                  (zero terminal velocity), but its location along the
                  ball's path is optimized to minimize the interception
                  time. Solved by 1D bisection of the bang-bang trajectory
                  end time vs. the ball arrival time along the ball line.
  * ``complex`` - The intercept point sits in the *middle* of the robot's
                  trajectory, with non-zero velocity at the moment of
                  contact. The end pose sent to firmware is placed at
                  ``P_end = 2*P - S`` so that the bang-bang trajectory
                  (starting from rest ``S``, ending at rest ``P_end``)
                  passes exactly through ``P`` at half its end time.
                  The optimization searches along the ball line for the
                  earliest time the robot can be at ``P`` simultaneously
                  with the ball. Requires the ``ateam_controls`` Python
                  bindings (from the SSL-A-Team/controls repo) so the
                  trajectory computation matches the firmware exactly.

Robot orientation at the intercept always points back along the ball's
velocity vector (i.e. the dribbler faces the incoming ball). The robot
center is offset behind the intercept point by ``robot_front_offset``
so the dribbler tip lands on the ball trajectory line.

Any solution that would place the robot center (or, for complex mode,
the trajectory end pose) outside the playable field minus
``robot_radius`` is logged and rejected, falling back to the previously
latched intercept (or no command, if none is latched yet).
"""

from collections import deque
from enum import auto, Enum
import json
import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
from ateam_motion_scenarios.common.capture import (
    Capture,
    CaptureConfig,
    load_capture_defaults,
)
from ateam_motion_scenarios.common.overlays import (
    make_array,
    make_circle,
    make_line,
    make_point,
    make_pose_marker,
    make_text,
)
from ateam_motion_scenarios.common.pivot import make_pivot_cmd, PivotConfig
from ateam_msgs.msg import (
    FieldInfo,
    OverlayArray,
    RobotMotionCommand,
    Twist2D,
    VisionStateBall,
    VisionStateRobot,
)
from ateam_radio_msgs.msg import BasicTelemetry, ConnectionStatus
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


OVERLAY_NS = 'catch_scenario'
DEFAULT_PARAM_FILE = os.path.join(
    get_package_share_directory('ateam_motion_scenarios'),
    'config', 'catch_params.json')


# --------------------------------------------------------------------------
# Optional ateam_controls import. Required for `mode=complex`. Tries the
# system PYTHONPATH first; falls back to a couple of standard sibling
# locations next to the ROS workspace.
# --------------------------------------------------------------------------
_ATEAM_CONTROLS = None
_ATEAM_CONTROLS_IMPORT_ERR = None


def _try_import_ateam_controls():
    global _ATEAM_CONTROLS, _ATEAM_CONTROLS_IMPORT_ERR
    if _ATEAM_CONTROLS is not None:
        return _ATEAM_CONTROLS
    candidate_paths = []
    env_path = os.environ.get('ATEAM_CONTROLS_PY_PATH')
    if env_path:
        candidate_paths.append(env_path)
    home = os.path.expanduser('~')
    candidate_paths.extend([
        os.path.join(home, 'workspace', 'controls', 'ateam-controls-py'),
        os.path.join(home, 'controls', 'ateam-controls-py'),
    ])
    for p in candidate_paths:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)
    try:
        import ateam_controls as ac  # noqa: F401
        _ATEAM_CONTROLS = ac
    except Exception as e:  # ImportError or FileNotFoundError for the .so
        _ATEAM_CONTROLS_IMPORT_ERR = e
        _ATEAM_CONTROLS = None
    return _ATEAM_CONTROLS


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def ang_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class State(Enum):
    WAIT_FOR_CONNECTION = auto()
    WAIT_FOR_BALL = auto()
    WAIT_FOR_INCOMING = auto()
    INTERCEPT = auto()
    GIVE = auto()
    CATCH_DWELL = auto()
    KICK_BACK = auto()
    COLLECT_APPROACH = auto()
    COLLECT_CAPTURE = auto()
    CAPTURE_SKILL = auto()
    RESET = auto()
    DONE = auto()


class CatchScenario(Node):

    def __init__(self):
        super().__init__('catch_scenario')

        self.defaults = self._load_defaults()
        self.robot_id = int(self._p('robot_id', 2))
        self.team_color = str(self._p('team_color', 'blue'))

        # Mode selection
        self._declare('mode', 'simple')

        # Trajectory limits. These are the
        # caps used both by the in-process bang-bang simulation and by
        # the limit fields sent to the firmware so the two solvers
        # produce the same trajectory.
        self._declare('max_vel_linear', 1.0)
        self._declare('max_vel_angular', 3.0 * math.pi)
        self._declare('max_accel_linear', 2.0)
        self._declare('max_accel_angular', 6.0 * math.pi)

        # Geometry / robot
        self._declare('robot_radius', 0.09)
        self._declare('ball_radius', 0.0215)
        # Sourced from the shared capture config (config/skill_capture_params.json)
        # so the dribbler offset stays consistent with the Capture skill.
        self._declare(
            'robot_front_offset',
            load_capture_defaults().get('robot_front_offset', 0.09))
        # Safety margin added to robot_radius when checking field bounds.
        self._declare('field_safety_margin', 0.05)

        # Ball gating
        self._declare('min_ball_speed', 0.01)
        # Lower speed floor used while actively tracking an incoming pass
        # (the INTERCEPT state) and by the realized-intercept detector.
        # `min_ball_speed` gates when a pass is first considered "incoming",
        # but once we're chasing it we keep recomputing the intercept and
        # stay responsive until the ball stops moving forward (its forward
        # velocity drops below this floor or reverses). Must be > 0 so the
        # ball-direction unit vector stays well defined; should be small.
        self._declare('min_ball_speed_tracking', 0.01)
        # Lower bound used only by the collision detector's rolling
        # history. Samples below this speed are still kept in the
        # window so the "post" average can see a captured / stopped
        # ball. Should be <= min_ball_speed.
        self._declare('min_ball_speed_detector', 0.0)
        # Only act on balls moving toward -x (rolled in from +x edge).
        self._declare('require_negative_vx', True)
        # End-to-end latency to compensate for: vision pipeline +
        # command publish + radio + firmware actuation. The observed
        # ball position/velocity is propagated forward by this amount
        # (constant-velocity model) so the intercept solver targets
        # where the ball will be when the command actually lands.
        self._declare('latency_compensation_s', 0.0)
        # Intercept solver scheduling:
        # - `single_intercept_calculation`: if true, the solver runs at
        #   most once per ball cycle (no replanning while the robot
        #   drives). Useful for evaluating an open-loop plan.
        # - `intercept_calc_delay`: wait this many seconds after the
        #   first valid ball observation in INTERCEPT before solving.
        #   Gives the vision filter time to converge on the ball's
        #   velocity before committing to a plan.
        self._declare('single_intercept_calculation', False)
        self._declare('intercept_calc_delay', 0.0)

        # Interception search
        self._declare('search_d_min', 0.05)
        self._declare('search_d_max', 8.0)
        self._declare('search_tolerance', 0.01)
        self._declare('search_max_iter', 40)

        # Catch hold and reset
        self._declare('dribbler_speed', 300.0)
        self._declare('pos_tol', 0.04)
        self._declare('yaw_tol', 0.08)
        self._declare('catch_dwell', 1.5)

        # "Give" - just before the ball arrives, shift the position
        # target back along the ball travel direction so the robot is
        # already moving away at the moment of contact, softening the
        # catch. Stays in BCM_GLOBAL_POSITION mode the entire time.
        self._declare('give_enabled', False)
        # Distance (m) between robot center and ball at which to switch
        # the position target into the retreated pose.
        self._declare('give_trigger_distance', 0.6)
        # Distance (m) to retreat backward along the ball travel line.
        self._declare('give_retreat_distance', 0.15)
        # How long to hold the retreated target before transitioning to
        # CATCH_DWELL (which then dwells wherever the robot ended up).
        self._declare('give_duration', 1.0)

        # Touch pass: hold KR_KICK_TOUCH armed throughout the intercept
        # (and during GIVE) so the firmware fires the kicker the
        # instant the breakbeam detects ball contact - bouncing the
        # ball back out as a one-touch pass. Mutually useful with
        # `give_*` for a soft redirect. Overrides any post-catch
        # behavior (CATCH_DWELL still runs to keep the robot in place,
        # but the ball will already be gone).
        self._declare('touch_pass_enabled', True)
        self._declare('touch_pass_speed', 1.5)  # m/s
        self._declare('kick_back_enabled', True)
        self._declare('kick_back_speed', 1.5)   # m/s
        self._declare('kick_back_aim_dwell', 0.3)
        self._declare('kick_back_post_dwell', 0.2)
        # KICK_BACK turns the robot back toward the +x goal with the firmware
        # BCM_PIVOT maneuver (orbit radius / inset / angular limits come from
        # the shared config/skill_pivot_params.json via kick_back_pivot_* params).
        # 0 = use the shared pivot angular-velocity limit; >0 overrides it.
        self._declare('kick_back_max_angular_vel', 0.0)

        # After a missed pass (ball passed the robot or stopped without
        # being caught), drive over and collect the ball from whichever
        # side approaches it from the robot's starting position. The
        # approach point sits on the segment between the robot start
        # pose `S` and the ball, offset back from the ball.
        self._declare('collect_missed_enabled', False)
        self._declare('collect_approach_offset', 0.15)  # m
        self._declare('collect_capture_overdrive', 0.04)
        self._declare('collect_capture_vel_limit', 0.2)
        self._declare('collect_capture_dwell', 0.5)
        self._declare('collect_stationary_threshold', 0.1)  # m/s
        # After a successful catch (CATCH_DWELL), optionally drive over and
        # capture the ball if it ended up within capture_after_catch_radius
        # of the robot. This uses the shared Capture skill (capture.py),
        # honoring its parameters (approach_radius, capture_speed,
        # capture_accel, capture_filter_count, use_breakbeam, ...).
        self._declare('capture_after_catch_enabled', True)
        self._declare('capture_after_catch_radius', 0.5)  # m
        # The shared Capture skill's parameters (approach_radius,
        # capture_speed, capture_accel, capture_filter_count, use_breakbeam,
        # ...) are declared on demand by CaptureConfig.from_params with their
        # defaults sourced from config/skill_capture_params.json, so they are not
        # re-declared here. breakbeam_topic is read directly by this node, so
        # default it from the shared capture config.
        self._declare(
            'breakbeam_topic',
            load_capture_defaults().get('breakbeam_topic', ''))
        # After collision is detected, the robot drives to the latched
        # intercept target and dwells. If it can't reach that target
        # within this many seconds, give up and reset anyway. Counted
        # from the moment of collision.
        self._declare('post_collision_arrival_timeout', 3.0)
        # If the ball isn't visible / valid for this long while in
        # INTERCEPT, give up and reset to the back edge.
        self._declare('ball_lost_timeout', 0.5)
        # How long WAIT_FOR_INCOMING will wait for a valid (visible,
        # fast-enough, negative-vx) ball before giving up and going
        # back to WAIT_FOR_BALL. Prevents getting stuck when the ball
        # is visible but stationary or moving the wrong way.
        self._declare('wait_for_incoming_timeout', 3.0)
        # Realized-intercept event detection. Compares the averaged
        # ball velocity in two non-overlapping rolling windows:
        #   * "pre"  window: from `dir_window_total` ago up to
        #                    `dir_window_total - dir_window_pre` ago.
        #   * "post" window: the most recent `dir_window_post` seconds.
        # Latches a collision when the velocity-retention metric
        #     r = (pre . post) / (pre . pre)
        # drops below `intercept_velocity_retention_threshold`. r = 1.0
        # means the ball is unchanged, 0.0 means stopped or perpendicular,
        # negative values mean reversal. The default 0.4 catches both
        # a sharp deflection and a big speed drop while still heading
        # in the same direction.
        # When `collision_detection_enabled` is false the robot keeps
        # tracking the ball through INTERCEPT without ever latching a
        # realized-intercept / collision event.
        self._declare('collision_detection_enabled', False)
        self._declare('intercept_velocity_retention_threshold', 0.4)
        self._declare('dir_window_pre', 0.08)
        self._declare('dir_window_post', 0.04)
        self._declare('dir_window_total', 0.15)
        self._declare('dir_min_samples', 3)
        self._declare('reset_margin', 0.3)
        # Status-text placement on the /overlays layer. By default the
        # state name + timing is rendered just below the negative-y
        # touchline of the field, off the playing surface. Provide
        # finite `status_text_x` / `status_text_y` (meters in field
        # frame) to override.
        self._declare('status_text_x', float('nan'))
        self._declare('status_text_y', float('nan'))
        self._declare('status_text_margin', 0.2)
        # Overlay visibility toggles.
        self._declare('show_detected_collision_overlay', True)
        self._declare('publish_rate_hz', 100.0)
        self._declare('loop', True)
        # One-shot mode: run the state machine for a single complete
        # cycle (RESET -> WAIT_FOR_BALL -> INTERCEPT -> CATCH_DWELL ->
        # RESET -> DONE), then call rclpy.shutdown(). Implies loop=False.
        self._declare('one_shot', False)

        # Radio connection monitoring. When ``require_connection`` is true the
        # scenario waits for the robot's radio link before driving and
        # restarts the state machine if the link drops mid-run.
        self._declare('require_connection', True)
        self._declare('connection_topic', '')

        # ------------------------------------------------------------------
        self.mode = str(self.get_parameter('mode').value).lower()
        if self.mode not in ('simple', 'medium', 'complex'):
            self.get_logger().warn(
                f"Unknown mode '{self.mode}', defaulting to 'simple'")
            self.mode = 'simple'

        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.ball_radius = float(self.get_parameter('ball_radius').value)
        self.robot_front_offset = float(self.get_parameter(
            'robot_front_offset').value)
        self.field_safety_margin = float(self.get_parameter(
            'field_safety_margin').value)

        self.min_ball_speed = float(self.get_parameter('min_ball_speed').value)
        self.min_ball_speed_tracking = float(self.get_parameter(
            'min_ball_speed_tracking').value)
        self.min_ball_speed_detector = float(self.get_parameter(
            'min_ball_speed_detector').value)
        self.require_negative_vx = bool(self.get_parameter(
            'require_negative_vx').value)
        self.latency_compensation_s = float(self.get_parameter(
            'latency_compensation_s').value)
        self.single_intercept_calculation = bool(self.get_parameter(
            'single_intercept_calculation').value)
        self.intercept_calc_delay = float(self.get_parameter(
            'intercept_calc_delay').value)

        self.max_vel_linear = float(self.get_parameter('max_vel_linear').value)
        self.max_vel_angular = float(self.get_parameter(
            'max_vel_angular').value)
        self.max_accel_linear = float(self.get_parameter(
            'max_accel_linear').value)
        self.max_accel_angular = float(self.get_parameter(
            'max_accel_angular').value)

        self.search_d_min = float(self.get_parameter('search_d_min').value)
        self.search_d_max = float(self.get_parameter('search_d_max').value)
        self.search_tolerance = float(self.get_parameter(
            'search_tolerance').value)
        self.search_max_iter = int(self.get_parameter('search_max_iter').value)

        self.dribbler_speed = float(self.get_parameter('dribbler_speed').value)
        self.pos_tol = float(self.get_parameter('pos_tol').value)
        self.yaw_tol = float(self.get_parameter('yaw_tol').value)
        self.catch_dwell = float(self.get_parameter('catch_dwell').value)

        self.give_enabled = bool(self.get_parameter('give_enabled').value)
        self.give_trigger_distance = float(self.get_parameter(
            'give_trigger_distance').value)
        self.give_retreat_distance = float(self.get_parameter(
            'give_retreat_distance').value)
        self.give_duration = float(self.get_parameter('give_duration').value)

        self.touch_pass_enabled = bool(self.get_parameter(
            'touch_pass_enabled').value)
        self.touch_pass_speed = float(self.get_parameter(
            'touch_pass_speed').value)

        self.kick_back_enabled = bool(self.get_parameter(
            'kick_back_enabled').value)
        self.kick_back_speed = float(self.get_parameter(
            'kick_back_speed').value)
        self.kick_back_aim_dwell = float(self.get_parameter(
            'kick_back_aim_dwell').value)
        self.kick_back_post_dwell = float(self.get_parameter(
            'kick_back_post_dwell').value)
        self.kick_back_max_angular_vel = float(self.get_parameter(
            'kick_back_max_angular_vel').value)

        self.collect_missed_enabled = bool(self.get_parameter(
            'collect_missed_enabled').value)
        self.collect_approach_offset = float(self.get_parameter(
            'collect_approach_offset').value)
        self.collect_capture_overdrive = float(self.get_parameter(
            'collect_capture_overdrive').value)
        self.collect_capture_vel_limit = float(self.get_parameter(
            'collect_capture_vel_limit').value)
        self.collect_capture_dwell = float(self.get_parameter(
            'collect_capture_dwell').value)
        self.collect_stationary_threshold = float(self.get_parameter(
            'collect_stationary_threshold').value)
        self.capture_after_catch_enabled = bool(self.get_parameter(
            'capture_after_catch_enabled').value)
        self.capture_after_catch_radius = float(self.get_parameter(
            'capture_after_catch_radius').value)
        # Shared two-phase capture skill (see capture.py). Reads its own
        # parameters via the get-parameter adapter below.
        self.breakbeam_topic = str(self.get_parameter(
            'breakbeam_topic').value) \
            or f'/robot_feedback/basic/robot{self.robot_id}'
        self.capture = Capture(CaptureConfig.from_params(self._capture_param))
        # Firmware-pivot tuning for the kick-back turn (orbit radius, inset,
        # angular limits, dribbler). Defaults come from the shared
        # config/skill_pivot_params.json; override per scenario via
        # kick_back_pivot_* ROS params.
        self.kick_back_pivot = PivotConfig.from_params(
            self._capture_param, prefix='kick_back_pivot_')
        self.breakbeam_detected = None
        self.post_collision_arrival_timeout = float(self.get_parameter(
            'post_collision_arrival_timeout').value)
        self.ball_lost_timeout = float(self.get_parameter(
            'ball_lost_timeout').value)
        self.wait_for_incoming_timeout = float(self.get_parameter(
            'wait_for_incoming_timeout').value)
        self.intercept_velocity_retention_threshold = float(
            self.get_parameter(
                'intercept_velocity_retention_threshold').value)
        self.collision_detection_enabled = bool(self.get_parameter(
            'collision_detection_enabled').value)
        self.dir_window_pre = float(self.get_parameter(
            'dir_window_pre').value)
        self.dir_window_post = float(self.get_parameter(
            'dir_window_post').value)
        self.dir_window_total = float(self.get_parameter(
            'dir_window_total').value)
        self.dir_min_samples = int(self.get_parameter(
            'dir_min_samples').value)
        self.reset_margin = float(self.get_parameter('reset_margin').value)
        sx = float(self.get_parameter('status_text_x').value)
        sy = float(self.get_parameter('status_text_y').value)
        self._status_text_x = None if math.isnan(sx) else sx
        self._status_text_y = None if math.isnan(sy) else sy
        self.status_text_margin = float(self.get_parameter(
            'status_text_margin').value)
        self.show_detected_collision_overlay = bool(self.get_parameter(
            'show_detected_collision_overlay').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.one_shot = bool(self.get_parameter('one_shot').value)
        if self.one_shot:
            self.loop = False
        self.require_connection = bool(self.get_parameter(
            'require_connection').value)
        self.connection_topic = str(self.get_parameter(
            'connection_topic').value) \
            or f'/robot_feedback/connection/robot{self.robot_id}'
        rate = float(self.get_parameter('publish_rate_hz').value)

        # ateam_controls bindings (only required for complex mode).
        self.ac = _try_import_ateam_controls()
        if self.mode == 'complex' and self.ac is None:
            self.get_logger().error(
                'mode=complex requires the ateam_controls Python bindings '
                f'(import failed: {_ATEAM_CONTROLS_IMPORT_ERR}). '
                'Either set ATEAM_CONTROLS_PY_PATH to ateam-controls-py, '
                'or run with mode=medium / mode=simple.')

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.ball = None
        self.robot = None
        self.field = None
        self.radio_connected = None  # None until first connection message

        self.ball_sub = self.create_subscription(
            VisionStateBall, '/ball', self.ball_cb, sensor_qos)
        self.robot_sub = self.create_subscription(
            VisionStateRobot, f'/{self.team_color}_team/robot{self.robot_id}',
            self.robot_cb, sensor_qos)
        self.field_sub = self.create_subscription(
            FieldInfo, '/field', self.field_cb, 10)
        # Basic telemetry carries the breakbeam used by the capture skill
        # (RELIABLE + VOLATILE, matching the radio bridge's SystemDefaultsQoS).
        feedback_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.telemetry_sub = self.create_subscription(
            BasicTelemetry, self.breakbeam_topic,
            self.telemetry_cb, feedback_qos)
        self.connection_sub = self.create_subscription(
            ConnectionStatus, self.connection_topic,
            self.connection_cb, feedback_qos)

        self.cmd_pub = self.create_publisher(
            RobotMotionCommand,
            f'/robot_motion_commands/robot{self.robot_id}', 10)
        self.overlay_pub = self.create_publisher(
            OverlayArray, '/overlays', 10)

        self.state = (State.WAIT_FOR_CONNECTION if self.require_connection
                      else State.RESET)
        self.state_entered = self.get_clock().now()
        self.catch_arrived_at = None
        # After RESET arrival, transition to this state. Set to DONE
        # only after a successful catch when loop=False.
        self.post_reset_state = State.WAIT_FOR_BALL

        # Latched data populated when entering INTERCEPT.
        self.S = None              # (x, y) initial robot rest position
        self.intercept_P = None    # (x, y, theta) dribbler intercept pose
        self.intercept_target = None  # (x, y, theta) pose to send to firmware
        self.last_solution_d = None
        self.last_solution_T = None
        self.last_ball_seen = None  # rclpy.Time of last valid ball
        # Solver scheduling state.
        self.first_valid_ball_time = None  # earliest valid ball this cycle
        self.intercept_solved_once = False
        # Wall-clock time at which the solver predicts the ball will
        # reach the planned intercept (updated each tick the solver
        # runs).
        self.predicted_impact_time = None
        # Latched position target for GIVE: the retreated dribbler
        # tip pose backward along the ball line. (x, y, theta) of the
        # robot center.
        self.give_pose = None
        # Pose latched on entry to CATCH_DWELL (or after a give) so the
        # dwell holds wherever the robot actually ended up.
        self.catch_dwell_pose = None
        # Latched aim heading + position for KICK_BACK.
        self.kick_back_pose = None
        self.kick_back_aim_arrived_at = None
        # Latched ball position when entering the COLLECT flow.
        self.collect_ball_xy = None

        # Realized intercept tracking: when the ball reverses direction
        # we treat that as the realized interception event and pin its
        # last observed position on the overlay until the next cycle.
        self.realized_intercept = None  # (x, y) or None
        # Captured states at the exact moment of collision detection.
        self.collision_ball_xy = None       # (x, y) ball pos at trigger
        self.collision_robot_pose = None    # (x, y, theta) robot pose at trigger
        # Once a collision is detected, freeze the plan overlays and
        # stop chasing the ball for the remainder of the cycle.
        self.collision_detected = False
        self.collision_time = None  # rclpy.Time when collision was latched
        # Last raw ball observation (independent of gating) used to
        # detect the direction-change event.
        self._last_ball_pos = None
        self._last_ball_vel = None
        self._last_ball_visible = False
        self._last_ball_speed = 0.0
        # Rolling history of visible ball samples: deque of
        # (t_sec, (x, y), (vx, vy), speed). Pruned to dir_window_total.
        self._ball_history = deque()

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'catch_scenario started for robot {self.robot_id} '
            f'({self.team_color}), mode={self.mode}, '
            f'one_shot={self.one_shot}. State: {self.state.name}')

    # ------------------------------------------------------------------ subs
    def ball_cb(self, msg: VisionStateBall):
        self.ball = msg

    def robot_cb(self, msg: VisionStateRobot):
        self.robot = msg

    def field_cb(self, msg: FieldInfo):
        self.field = msg

    def telemetry_cb(self, msg: BasicTelemetry):
        self.breakbeam_detected = msg.breakbeam_ball_detected

    def connection_cb(self, msg: ConnectionStatus):
        self.radio_connected = msg.radio_connected

    # --------------------------------------------------------------- helpers
    def _load_defaults(self) -> dict:
        self.declare_parameter('param_file', '')
        pf = str(self.get_parameter('param_file').value) or DEFAULT_PARAM_FILE
        try:
            with open(pf) as f:
                data = json.load(f)
            self.get_logger().info(f'Loaded params from {pf}')
            return data
        except (OSError, ValueError) as exc:
            self.get_logger().warn(
                f'Could not load param file {pf} ({exc}); using built-in '
                'defaults')
            return {}

    def _p(self, name: str, default):
        """Declare a ROS param defaulting to the JSON value, return value."""
        if not self.has_parameter(name):
            self.declare_parameter(name, self.defaults.get(name, default))
        return self.get_parameter(name).value

    def _declare(self, name, default):
        """Declare a ROS param, sourcing its default from the JSON file.

        The hardcoded ``default`` is only used as a fallback when the
        parameter is absent from ``config/catch_params.json``.
        """
        self.declare_parameter(name, self.defaults.get(name, default))

    def _capture_param(self, name, default):
        """Adapter exposing get_parameter as a p(name, default) getter.

        Like :meth:`_declare`, the default is taken from the JSON file when
        present so the shared Capture/Pivot tuning can also be overridden
        per scenario via ``config/catch_params.json``; otherwise it falls
        back to the shared-config default passed in by the caller.
        """
        if not self.has_parameter(name):
            self.declare_parameter(name, self.defaults.get(name, default))
        return self.get_parameter(name).value

    def transition(self, new_state: State):
        self.get_logger().info(
            f'State {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_entered = self.get_clock().now()
        self.catch_arrived_at = None
        self.last_ball_seen = None

    def time_in_state(self) -> float:
        return (self.get_clock().now() - self.state_entered).nanoseconds * 1e-9

    def robot_xy_yaw(self):
        if self.robot is None or not self.robot.visible:
            return None
        p = self.robot.pose.position
        return (p.x, p.y, yaw_from_quat(self.robot.pose.orientation))

    def robot_speed(self) -> float:
        if self.robot is None or not self.robot.visible:
            return float('inf')
        v = self.robot.twist.linear
        return math.hypot(v.x, v.y)

    def make_pos_cmd(self, x: float, y: float, theta: float = 0.0,
                     vel_limit: float = 0.0,
                     ang_vel_limit: float = 0.0,
                     accel_limit_linear: float = 0.0,
                     accel_limit_angular: float = 0.0,
                     kick_request: int = RobotMotionCommand.KR_DISABLE,
                     kick_speed: float = 0.0,
                     dribbler_speed: float = 0.0) -> RobotMotionCommand:
        cmd = RobotMotionCommand()
        cmd.body_control_mode = RobotMotionCommand.BCM_GLOBAL_POSITION
        cmd.pose = Twist2D(x=float(x), y=float(y), theta=float(theta))
        cmd.velocity = Twist2D()
        cmd.acceleration = Twist2D()
        cmd.limit_vel_linear = float(vel_limit)
        cmd.limit_vel_angular = float(ang_vel_limit)
        cmd.limit_acc_linear = float(accel_limit_linear)
        cmd.limit_acc_angular = float(accel_limit_angular)
        cmd.kick_request = kick_request
        cmd.kick_speed = float(kick_speed)
        cmd.dribbler_speed = float(dribbler_speed)
        return cmd

    def make_off_cmd(self) -> RobotMotionCommand:
        cmd = RobotMotionCommand()
        cmd.body_control_mode = RobotMotionCommand.BCM_OFF
        cmd.pose = Twist2D()
        cmd.velocity = Twist2D()
        cmd.acceleration = Twist2D()
        cmd.kick_request = RobotMotionCommand.KR_DISABLE
        return cmd

    def make_hold_cmd(self) -> RobotMotionCommand:
        """Hold the reset/back-edge pose so the firmware control loop
        stays warm (motors energized, KF active) instead of decaying
        to BCM_OFF while we wait for a ball."""
        if self.field is not None and self.field.field_length > 0.0:
            tx = -self.field.field_length / 2.0 + self.reset_margin
        else:
            rs = self.robot_xy_yaw()
            if rs is None:
                return self.make_off_cmd()
            tx = rs[0]
        return self.make_pos_cmd(tx, 0.0, 0.0)

    def make_global_vel_cmd(self, vx: float, vy: float, vtheta: float = 0.0,
                            kick_request: int = RobotMotionCommand.KR_DISABLE,
                            dribbler_speed: float = 0.0
                            ) -> RobotMotionCommand:
        cmd = RobotMotionCommand()
        cmd.body_control_mode = RobotMotionCommand.BCM_GLOBAL_VELOCITY
        cmd.pose = Twist2D()
        cmd.velocity = Twist2D(x=float(vx), y=float(vy), theta=float(vtheta))
        cmd.acceleration = Twist2D()
        cmd.limit_acc_linear = float(self.max_accel_linear)
        cmd.limit_acc_angular = float(self.max_accel_angular)
        cmd.kick_request = kick_request
        cmd.kick_speed = 0.0
        cmd.dribbler_speed = float(dribbler_speed)
        return cmd

    # --------------------------------------------------------- field bounds
    def in_bounds(self, x: float, y: float) -> bool:
        """Return True if (x, y) is a legal robot-center position."""
        if self.field is None or self.field.field_length <= 0.0 \
                or self.field.field_width <= 0.0:
            return True  # Field unknown: don't reject solutions.
        margin = self.robot_radius + self.field_safety_margin
        half_l = self.field.field_length / 2.0 - margin
        half_w = self.field.field_width / 2.0 - margin
        return abs(x) <= half_l and abs(y) <= half_w

    # -------------------------------------------------------- bang-bang sim
    def _bang_bang_time(self, S, target_pose, init_vel=(0.0, 0.0, 0.0),
                        init_yaw=0.0):
        """Compute end_time of a bang-bang trajectory using the controls
        library (matches firmware exactly). Returns None if bindings not
        available or solver fails.
        """
        if self.ac is None:
            return None
        try:
            init_state = self.ac.Vector6C()
            init_state.data[0] = float(S[0])
            init_state.data[1] = float(S[1])
            init_state.data[2] = float(init_yaw)
            init_state.data[3] = float(init_vel[0])
            init_state.data[4] = float(init_vel[1])
            init_state.data[5] = float(init_vel[2])
            tgt = self.ac.Vector3C(
                x=float(target_pose[0]),
                y=float(target_pose[1]),
                z=float(target_pose[2]),
            )
            params = self.ac.TrajectoryParams(
                max_vel_linear=self.max_vel_linear,
                max_vel_angular=self.max_vel_angular,
                max_accel_linear=self.max_accel_linear,
                max_accel_angular=self.max_accel_angular,
            )
            traj = self.ac.traj_from_target_pose(init_state, tgt, params)
            return float(self.ac.traj_end_time(traj))
        except Exception as e:
            self.get_logger().warn(
                f'bang_bang_time failed: {e}', throttle_duration_sec=2.0)
            return None

    def _bang_bang_time_2d_fallback(self, S, P_end):
        """Closed-form bang-bang end-time for a 2D pose from rest to rest,
        ignoring theta. Used when ateam_controls bindings aren't available
        (simple/medium modes). Matches the firmware's alpha-search
        formulation: synchronize x and y end-times by splitting the
        max linear acceleration between them. The longer-axis dominates
        the total time.

        For starting and ending at rest, each axis's time is
        triangular T = 2*sqrt(D/a)  if vpeak = sqrt(D*a) < vmax,
        trapezoidal T = D/vmax + vmax/a otherwise.

        With acceleration scaled by alpha (cos/sin), the optimum
        synchronization sets a_x = a*Dx/|D|, a_y = a*Dy/|D| (which is
        what the planner converges to). Under this allocation both axes
        share the same end-time:

          T = max( triangular_or_trapezoidal(|D|, a, vmax),
                   trapezoidal_or_triangular along the resultant ) .

        That collapses to the 1D bang-bang time for distance |D| under
        the full a_max and vmax, which is the exact firmware result.
        """
        D = math.hypot(P_end[0] - S[0], P_end[1] - S[1])
        a = self.max_accel_linear
        v = self.max_vel_linear
        if a <= 0.0 or D <= 1e-9:
            return 0.0
        # Triangular: vpeak = sqrt(D*a).
        vpeak = math.sqrt(D * a)
        if vpeak <= v:
            return 2.0 * math.sqrt(D / a)
        # Trapezoidal.
        return D / v + v / a

    def trajectory_time(self, S, P_end):
        """Pick whichever solver is available."""
        if self.ac is not None:
            t = self._bang_bang_time(S, (P_end[0], P_end[1], 0.0))
            if t is not None:
                return t
        return self._bang_bang_time_2d_fallback(S, P_end)

    # ----------------------------------------------------- ball prediction
    def ball_state(self, min_speed=None):
        """Return ((bx, by), (vx, vy), speed) or None.

        Position is propagated forward by ``latency_compensation_s``
        using a constant-velocity model so the solver targets where
        the ball will be by the time the command is actuated.

        ``min_speed`` overrides the speed floor below which the ball is
        treated as no longer in play (defaults to ``min_ball_speed``).
        Callers that keep tracking a decelerating pass pass the lower
        ``min_ball_speed_tracking`` floor instead.
        """
        if min_speed is None:
            min_speed = self.min_ball_speed
        if self.ball is None or not self.ball.visible:
            return None
        bp = self.ball.pose.position
        bv = self.ball.twist.linear
        speed = math.hypot(bv.x, bv.y)
        if speed < min_speed:
            return None
        if self.require_negative_vx and bv.x >= 0.0:
            return None
        dt = self.latency_compensation_s
        bx = bp.x + bv.x * dt
        by = bp.y + bv.y * dt
        return ((bx, by), (bv.x, bv.y), speed)

    def ball_dribbler_target(self, d, ball):
        """For an intercept at distance `d` along the ball line, return
        (P_dribbler, P_center, theta).
        """
        (bx, by), (vx, vy), speed = ball
        ux, uy = vx / speed, vy / speed
        dx = bx + ux * d
        dy = by + uy * d
        theta = math.atan2(-vy, -vx)
        cx = dx - self.robot_front_offset * math.cos(theta)
        cy = dy - self.robot_front_offset * math.sin(theta)
        return (dx, dy), (cx, cy), theta

    # --------------------------------------------------- solver per mode
    def solve_simple(self, ball, S):
        """Robot holds its starting x; pick target y such that the
        dribbler tip lands on the ball's predicted line, facing back
        down the line. Returns (P_dribbler, P_center, theta, T_ball)
        or None.
        """
        (bx, by), (vx, vy), speed = ball
        theta = math.atan2(-vy, -vx)
        # Dribbler tip lies at (S_x, ?). Solve t for ball.x: bx + t*vx = S_x + off*cos(theta).
        off_x = self.robot_front_offset * math.cos(theta)
        off_y = self.robot_front_offset * math.sin(theta)
        drib_x = S[0] + off_x
        if abs(vx) < 1e-6:
            return None
        t_ball = (drib_x - bx) / vx
        if t_ball <= 0.0:
            return None
        drib_y = by + vy * t_ball
        center_y = drib_y - off_y
        P_drib = (drib_x, drib_y)
        P_center = (S[0], center_y)
        if not self.in_bounds(P_center[0], P_center[1]):
            self.get_logger().warn(
                f'Rejected simple intercept: P_center=({P_center[0]:.2f},'
                f'{P_center[1]:.2f}) out of bounds',
                throttle_duration_sec=1.0)
            return None
        return P_drib, P_center, theta, t_ball

    def _feasibility_fn(self, d, ball, S, mode):
        """For a candidate intercept distance `d`, return (slack, info)
        where slack = T_ball - T_robot. Positive slack means the robot
        can reach the intercept early.
        info = (P_drib, P_center, theta, T_traj, T_ball, P_end_or_None)
        """
        speed = ball[2]
        t_ball = d / speed
        P_drib, P_center, theta = self.ball_dribbler_target(d, ball)
        if mode == 'medium':
            t_traj = self.trajectory_time(S, P_center)
            return t_ball - t_traj, (P_drib, P_center, theta, t_traj,
                                     t_ball, None)
        # complex
        P_end = (2.0 * P_center[0] - S[0], 2.0 * P_center[1] - S[1])
        t_traj = self.trajectory_time(S, P_end)
        # Intercept happens at half the end-time (symmetric rest-to-rest).
        return t_ball - 0.5 * t_traj, (P_drib, P_center, theta, t_traj,
                                       t_ball, P_end)

    def solve_search(self, ball, S, mode):
        """Bisect for the minimum d where the robot can just reach in
        time. Returns (P_dribbler, P_center, theta, target_pose, T_int)
        or None.  target_pose is what gets sent to firmware as the
        position command.
        """
        d_lo = max(self.search_d_min, 0.0)
        d_hi = self.search_d_max

        slack_lo, info_lo = self._feasibility_fn(d_lo, ball, S, mode)
        slack_hi, info_hi = self._feasibility_fn(d_hi, ball, S, mode)

        # If even at d_hi the robot can't make it, search will diverge.
        # Just take d_hi as best-effort and let in_bounds reject if needed.
        if slack_hi < 0.0:
            self.get_logger().warn(
                'No feasible intercept within search range '
                f'(d_hi={d_hi:.2f}, slack={slack_hi:.2f}s)',
                throttle_duration_sec=1.0)
            return None
        # If d_lo is already feasible, the minimum-time intercept is
        # somewhere between 0 and d_lo (or exactly at d_lo if d_lo
        # happens to be the answer). Take d_lo directly.
        if slack_lo >= 0.0:
            return self._materialize(info_lo, mode)

        # Bisection.
        for _ in range(self.search_max_iter):
            d_mid = 0.5 * (d_lo + d_hi)
            if (d_hi - d_lo) < self.search_tolerance:
                break
            slack_mid, info_mid = self._feasibility_fn(d_mid, ball, S, mode)
            if slack_mid >= 0.0:
                d_hi = d_mid
                info_hi = info_mid
            else:
                d_lo = d_mid

        return self._materialize(info_hi, mode)

    def _materialize(self, info, mode):
        P_drib, P_center, theta, t_traj, t_ball, P_end = info
        if not self.in_bounds(P_center[0], P_center[1]):
            self.get_logger().warn(
                f'Rejected intercept: P_center=({P_center[0]:.2f},'
                f'{P_center[1]:.2f}) out of bounds',
                throttle_duration_sec=1.0)
            return None
        if mode == 'medium':
            target = (P_center[0], P_center[1], theta)
            t_int = t_traj
        else:  # complex
            if not self.in_bounds(P_end[0], P_end[1]):
                self.get_logger().warn(
                    f'Rejected complex intercept: P_end=({P_end[0]:.2f},'
                    f'{P_end[1]:.2f}) out of bounds',
                    throttle_duration_sec=1.0)
                return None
            target = (P_end[0], P_end[1], theta)
            t_int = 0.5 * t_traj
        return P_drib, P_center, theta, target, t_int

    # ------------------------------------------------------------------ tick
    def tick(self):
        self._check_connection()
        self.detect_realized_intercept()
        cmd = self.step_state_machine()
        if cmd is not None:
            self.cmd_pub.publish(cmd)
        self.publish_overlays()
        if self.one_shot and self.state == State.DONE:
            self.get_logger().info(
                'one_shot cycle complete, shutting down.')
            try:
                self.cmd_pub.publish(self.make_off_cmd())
            except Exception:
                pass
            self.timer.cancel()
            rclpy.shutdown()

    def _check_connection(self):
        """Restart the state machine if the robot's radio link drops."""
        if not self.require_connection:
            return
        if self.state == State.WAIT_FOR_CONNECTION:
            return
        if self.radio_connected is False:
            self.get_logger().warn(
                'Robot disconnected; restarting state machine')
            self._reset_run_state()
            self.transition(State.WAIT_FOR_CONNECTION)

    def _reset_run_state(self):
        """Clear all latched per-run state."""
        self.catch_arrived_at = None
        self.post_reset_state = State.WAIT_FOR_BALL
        self.S = None
        self.intercept_P = None
        self.intercept_target = None
        self.last_solution_d = None
        self.last_solution_T = None
        self.last_ball_seen = None
        self.first_valid_ball_time = None
        self.intercept_solved_once = False
        self.predicted_impact_time = None
        self.give_pose = None
        self.catch_dwell_pose = None
        self.kick_back_pose = None
        self.kick_back_aim_arrived_at = None
        self.collect_ball_xy = None
        self.realized_intercept = None
        self.collision_ball_xy = None
        self.collision_robot_pose = None
        self.collision_detected = False
        self.collision_time = None
        self._last_ball_pos = None
        self._last_ball_vel = None
        self._last_ball_visible = False
        self._last_ball_speed = 0.0
        self._ball_history.clear()
        self.capture.reset()

    def detect_realized_intercept(self):
        """Watch ball observations for a sustained direction-trend
        change (or a disappearance) while in INTERCEPT / CATCH_DWELL.
        Uses two sliding windows of past ball velocities to compare
        the longer-term trajectory direction (pre-window) against the
        recent direction (post-window) and latches when the angle
        between the averaged velocities exceeds the configured
        threshold.
        """
        active = self.state in (State.INTERCEPT, State.CATCH_DWELL)
        if self.ball is None:
            return
        visible = bool(self.ball.visible)
        bp = self.ball.pose.position
        bv = self.ball.twist.linear
        speed = math.hypot(bv.x, bv.y)
        cur_pos = (bp.x, bp.y)
        cur_vel = (bv.x, bv.y)
        t = self.get_clock().now().nanoseconds * 1e-9

        # Maintain rolling history of every visible sample at or above
        # the detector floor (kept loose so the "post" window can see
        # a captured / stopped ball).
        if visible and speed >= self.min_ball_speed_detector:
            self._ball_history.append((t, cur_pos, cur_vel, speed))
        # Prune older than dir_window_total.
        cutoff = t - self.dir_window_total
        while self._ball_history and self._ball_history[0][0] < cutoff:
            self._ball_history.popleft()

        if (self.collision_detection_enabled and active
                and not self.collision_detected):
            # Disappearance trigger.
            if (not visible and self._last_ball_visible
                    and self._last_ball_speed >= self.min_ball_speed_tracking):
                self.realized_intercept = self._last_ball_pos
                self._latch_collision(self._last_ball_pos)
                self.get_logger().info(
                    f'Realized intercept (disappear) at '
                    f'({self._last_ball_pos[0]:.3f}, '
                    f'{self._last_ball_pos[1]:.3f})')
            else:
                # Trend-direction-change trigger using sliding windows.
                self._check_dir_trend(t)

        if visible:
            self._last_ball_pos = cur_pos
            self._last_ball_vel = cur_vel
            self._last_ball_speed = speed
        self._last_ball_visible = visible

    def _check_dir_trend(self, t_now):
        """Compare averaged velocity in the pre window (older portion
        of history) vs the post window (most recent portion). Latch
        collision when the velocity-retention metric
            r = (pre . post) / (pre . pre)
        drops below the configured threshold. This catches both sharp
        deflections (negative dot) and big speed drops in the same
        direction (small but positive r).
        """
        pre_hi = t_now - self.dir_window_total + self.dir_window_pre
        pre_lo = t_now - self.dir_window_total
        post_lo = t_now - self.dir_window_post

        pre_vx = pre_vy = 0.0
        pre_n = 0
        post_vx = post_vy = 0.0
        post_n = 0
        post_pos = None
        for ts, pos, vel, _sp in self._ball_history:
            if pre_lo <= ts <= pre_hi:
                pre_vx += vel[0]
                pre_vy += vel[1]
                pre_n += 1
            if ts >= post_lo:
                post_vx += vel[0]
                post_vy += vel[1]
                post_n += 1
                post_pos = pos
        if pre_n < self.dir_min_samples or post_n < self.dir_min_samples:
            return
        pre_vx /= pre_n
        pre_vy /= pre_n
        post_vx /= post_n
        post_vy /= post_n
        pre_mag_sq = pre_vx * pre_vx + pre_vy * pre_vy
        track_floor = self.min_ball_speed_tracking
        if pre_mag_sq < track_floor * track_floor:
            return
        dot = pre_vx * post_vx + pre_vy * post_vy
        retention = dot / pre_mag_sq
        if retention < self.intercept_velocity_retention_threshold:
            self.realized_intercept = post_pos
            self._latch_collision(post_pos)
            self.get_logger().info(
                f'Realized intercept (retention={retention:.2f}, '
                f'|pre|={math.sqrt(pre_mag_sq):.2f}, '
                f'|post|={math.hypot(post_vx, post_vy):.2f}, '
                f'pre_n={pre_n}, post_n={post_n}) at '
                f'({post_pos[0]:.3f}, {post_pos[1]:.3f})')

    def _latch_collision(self, ball_pos):
        """Common bookkeeping for collision detection: latch flag,
        timestamp, ball position at trigger and current robot pose."""
        self.collision_detected = True
        self.collision_time = self.get_clock().now()
        self.collision_ball_xy = ball_pos
        self.collision_robot_pose = self.robot_xy_yaw()

    def _touch_pass_kick(self):
        """Return (kick_request, kick_speed) honoring `touch_pass_enabled`.
        When enabled, holds KR_KICK_TOUCH armed at `touch_pass_speed` so
        the firmware fires immediately on breakbeam detection.
        """
        if self.touch_pass_enabled:
            return (RobotMotionCommand.KR_KICK_TOUCH, self.touch_pass_speed)
        return (RobotMotionCommand.KR_DISABLE, 0.0)

    def _enter_collect_if_possible(self) -> bool:
        """If collect_missed is enabled and we know where the ball is
        (currently visible, or last seen and roughly stationary), latch
        the ball position and transition into COLLECT_APPROACH. Returns
        True if a transition happened."""
        if not self.collect_missed_enabled:
            return False
        # Prefer current visible ball; fall back to last seen.
        bxy = None
        if self.ball is not None and self.ball.visible:
            bp = self.ball.pose.position
            bv = self.ball.twist.linear
            speed = math.hypot(bv.x, bv.y)
            # Either it's stationary, or it's still moving but we missed
            # the catch and want to chase. Either case is OK.
            bxy = (bp.x, bp.y)
            del speed
        elif self._last_ball_pos is not None:
            bxy = self._last_ball_pos
        if bxy is None:
            return False
        if not self.in_bounds(bxy[0], bxy[1]):
            return False
        self.collect_ball_xy = bxy
        self.get_logger().info(
            f'Entering COLLECT_APPROACH for missed ball at '
            f'({bxy[0]:.2f}, {bxy[1]:.2f})')
        self.transition(State.COLLECT_APPROACH)
        return True

    def _enter_capture_after_catch_if_possible(self) -> bool:
        """If enabled and the ball ended up within capture_after_catch_radius
        of the robot after a catch, run the shared Capture skill to drive
        over and pick it up. Returns True if a transition happened."""
        if not self.capture_after_catch_enabled:
            return False
        rs = self.robot_xy_yaw()
        if rs is None:
            return False
        bxy = None
        if self.ball is not None and self.ball.visible:
            bp = self.ball.pose.position
            bxy = (bp.x, bp.y)
        elif self._last_ball_pos is not None:
            bxy = self._last_ball_pos
        if bxy is None:
            return False
        dist = math.hypot(bxy[0] - rs[0], bxy[1] - rs[1])
        if dist > self.capture_after_catch_radius:
            return False
        if not self.in_bounds(bxy[0], bxy[1]):
            return False
        self.collect_ball_xy = bxy
        self.capture.reset()
        self.get_logger().info(
            f'Ball within {dist:.2f} m after catch; running capture skill '
            f'(ball at ({bxy[0]:.2f}, {bxy[1]:.2f}))')
        self.transition(State.CAPTURE_SKILL)
        return True

    def step_state_machine(self):

        s = self.state

        if s == State.WAIT_FOR_CONNECTION:
            if self.radio_connected:
                self.get_logger().info('Robot connected')
                self.transition(State.RESET)
            return self.make_off_cmd()

        if s == State.WAIT_FOR_BALL:
            if self.ball is not None and self.ball.visible:
                self.transition(State.WAIT_FOR_INCOMING)
            return self.make_hold_cmd()

        if s == State.WAIT_FOR_INCOMING:
            ball = self.ball_state()
            if ball is None:
                if (self.wait_for_incoming_timeout > 0.0
                        and self.time_in_state()
                        >= self.wait_for_incoming_timeout):
                    # If the ball is still visible but stationary /
                    # heading wrong, try to go fetch it instead of
                    # giving up.
                    if self.collect_missed_enabled:
                        # Latch S as current robot pose so the COLLECT
                        # geometry can use the "start -> ball" line.
                        rs = self.robot_xy_yaw()
                        if rs is not None:
                            self.S = (rs[0], rs[1])
                            if self._enter_collect_if_possible():
                                return self.make_hold_cmd()
                    self.get_logger().info(
                        f'WAIT_FOR_INCOMING timeout '
                        f'({self.wait_for_incoming_timeout:.2f}s); going '
                        'back to WAIT_FOR_BALL.')
                    self.transition(State.WAIT_FOR_BALL)
                return self.make_hold_cmd()
            rs = self.robot_xy_yaw()
            if rs is None:
                return self.make_hold_cmd()
            # Latch initial rest position.
            self.S = (rs[0], rs[1])
            # New cycle starts: clear prior realized intercept and
            # all plan overlays.
            self.realized_intercept = None
            self.collision_detected = False
            self.collision_time = None
            self.collision_ball_xy = None
            self.collision_robot_pose = None
            self.intercept_P = None
            self.intercept_target = None
            self.last_solution_d = None
            self.last_solution_T = None
            self._ball_history.clear()
            self.first_valid_ball_time = None
            self.intercept_solved_once = False
            self.predicted_impact_time = None
            self.give_pose = None
            self.catch_dwell_pose = None
            self.kick_back_pose = None
            self.kick_back_aim_arrived_at = None
            self.collect_ball_xy = None
            self.transition(State.INTERCEPT)
            return self.make_hold_cmd()

        if s == State.INTERCEPT:
            # Keep solving the intercept from the latest ball observation
            # using the lower tracking floor so a decelerating pass stays
            # tracked (and the robot responsive) until the ball stops
            # moving forward, rather than cutting out at min_ball_speed.
            ball = self.ball_state(self.min_ball_speed_tracking)
            rs = self.robot_xy_yaw()
            if rs is None:
                return self.make_off_cmd()
            now = self.get_clock().now()
            # While no collision is detected, refresh the plan from the
            # latest ball observation. After collision, the plan stays
            # frozen and the robot drives to the latched target, dwells,
            # then resets.
            if not self.collision_detected:
                if ball is not None:
                    self.last_ball_seen = now
                    if self.first_valid_ball_time is None:
                        self.first_valid_ball_time = now
                    elapsed_since_first = (
                        now - self.first_valid_ball_time).nanoseconds * 1e-9
                    may_solve = (
                        elapsed_since_first >= self.intercept_calc_delay
                        and not (self.single_intercept_calculation
                                 and self.intercept_solved_once))
                    sol = None
                    if may_solve:
                        if self.mode == 'simple':
                            res = self.solve_simple(ball, self.S)
                            if res is not None:
                                P_drib, P_center, theta, t_int = res
                                sol = (P_drib, P_center, theta,
                                       (P_center[0], P_center[1], theta),
                                       t_int)
                        else:
                            sol = self.solve_search(ball, self.S, self.mode)
                    if sol is not None:
                        P_drib, P_center, theta, target, t_int = sol
                        self.intercept_P = (P_drib[0], P_drib[1], theta)
                        self.intercept_target = target
                        self.last_solution_d = math.hypot(
                            P_drib[0] - ball[0][0], P_drib[1] - ball[0][1])
                        self.last_solution_T = t_int
                        self.intercept_solved_once = True
                        # Latch the earliest predicted impact time.
                        # Subsequent solves can pull it earlier (sooner
                        # impact) but never push it later — otherwise
                        # `time_to_impact` stays equal to `t_int` and
                        # never counts down to the give window.
                        candidate = now + Duration(seconds=float(t_int))
                        if (self.predicted_impact_time is None
                                or candidate < self.predicted_impact_time):
                            self.predicted_impact_time = candidate
                else:
                    # Ball lost without collision detection (no plan yet
                    # or ball never reached us). If collect-missed is
                    # enabled and the ball is still somewhere on the
                    # field, try to go pick it up. Otherwise reset.
                    if self.last_ball_seen is None:
                        self.last_ball_seen = now
                    elapsed = (now - self.last_ball_seen).nanoseconds * 1e-9
                    if elapsed >= self.ball_lost_timeout:
                        if self._enter_collect_if_possible():
                            return self.make_off_cmd()
                        self.get_logger().info(
                            f'Ball lost for {elapsed:.2f}s, resetting.')
                        self.transition(State.RESET)
                        return self.make_off_cmd()
            # Post-collision behavior: either we have a frozen target to
            # arrive at (then CATCH_DWELL), or we never had a plan -
            # either way bail to RESET after a bounded time.
            if self.collision_detected:
                elapsed = 0.0
                if self.collision_time is not None:
                    elapsed = (now - self.collision_time).nanoseconds * 1e-9
                if (self.intercept_target is None
                        or elapsed >= self.post_collision_arrival_timeout):
                    self.get_logger().info(
                        f'Post-collision timeout ({elapsed:.2f}s) or no '
                        'latched plan; resetting.')
                    self.transition(State.RESET)
                    return self.make_off_cmd()
            if self.intercept_target is None:
                return self.make_off_cmd()
            # GIVE trigger: distance-based. Once the ball is within
            # `give_trigger_distance` of the robot center and we're
            # still actively chasing, latch a retreated position
            # target backward along the ball travel direction. Uses
            # the raw ball pose (not ball_state) so a slowing ball
            # still triggers; uses raw ball velocity for direction
            # but falls back to the intercept heading otherwise.
            if (self.give_enabled
                    and not self.collision_detected
                    and self.intercept_P is not None
                    and self.ball is not None
                    and self.ball.visible):
                bp = self.ball.pose.position
                bv = self.ball.twist.linear
                dist_to_ball = math.hypot(rs[0] - bp.x, rs[1] - bp.y)
                if dist_to_ball <= self.give_trigger_distance:
                    sp = math.hypot(bv.x, bv.y)
                    if sp > 1e-6:
                        ux, uy = bv.x / sp, bv.y / sp
                    else:
                        ttheta = self.intercept_P[2]
                        ux, uy = -math.cos(ttheta), -math.sin(ttheta)
                    base_cx = (self.intercept_P[0]
                               - self.robot_front_offset
                               * math.cos(self.intercept_P[2]))
                    base_cy = (self.intercept_P[1]
                               - self.robot_front_offset
                               * math.sin(self.intercept_P[2]))
                    gx = base_cx + ux * self.give_retreat_distance
                    gy = base_cy + uy * self.give_retreat_distance
                    self.give_pose = (gx, gy, self.intercept_P[2])
                    self.get_logger().info(
                        f'GIVE triggered (ball_dist={dist_to_ball:.2f}m); '
                        f'retreat to ({gx:.2f}, {gy:.2f})')
                    self.transition(State.GIVE)
                    return self.make_off_cmd()
            tx, ty, ttheta = self.intercept_target
            kreq, kspd = self._touch_pass_kick()
            cmd = self.make_pos_cmd(
                tx, ty, ttheta,
                accel_limit_linear=self.max_accel_linear,
                accel_limit_angular=self.max_accel_angular,
                kick_request=kreq,
                kick_speed=kspd,
                dribbler_speed=self.dribbler_speed,
            )
            # Arrival check uses the dribbler intercept pose (where we
            # want the robot to actually be), not the firmware target
            # (which sits past the intercept in complex mode).
            if self.intercept_P is not None:
                px = self.intercept_P[0] - self.robot_front_offset * math.cos(
                    self.intercept_P[2])
                py = self.intercept_P[1] - self.robot_front_offset * math.sin(
                    self.intercept_P[2])
                if (math.hypot(rs[0] - px, rs[1] - py) <= self.pos_tol
                        and abs(ang_diff(rs[2], self.intercept_P[2]))
                        <= self.yaw_tol):
                    self.catch_dwell_pose = (rs[0], rs[1], rs[2])
                    self.transition(State.CATCH_DWELL)
            return cmd

        if s == State.GIVE:
            if self.give_pose is None:
                self.catch_dwell_pose = self.robot_xy_yaw()
                self.transition(State.CATCH_DWELL)
                return self.make_off_cmd()
            gx, gy, gtheta = self.give_pose
            kreq, kspd = self._touch_pass_kick()
            cmd = self.make_pos_cmd(
                gx, gy, gtheta,
                accel_limit_linear=self.max_accel_linear,
                accel_limit_angular=self.max_accel_angular,
                kick_request=kreq,
                kick_speed=kspd,
                dribbler_speed=self.dribbler_speed,
            )
            if self.time_in_state() >= self.give_duration:
                self.catch_dwell_pose = self.robot_xy_yaw()
                self.transition(State.CATCH_DWELL)
            return cmd

        if s == State.CATCH_DWELL:
            # Hold wherever we ended up (post-give or post-arrival).
            if self.catch_dwell_pose is None:
                rs = self.robot_xy_yaw()
                if rs is not None:
                    self.catch_dwell_pose = (rs[0], rs[1], rs[2])
            if self.catch_dwell_pose is None:
                self.transition(State.RESET)
                return self.make_off_cmd()
            hx, hy, htheta = self.catch_dwell_pose
            cmd = self.make_pos_cmd(
                hx, hy, htheta,
                kick_request=RobotMotionCommand.KR_DISABLE,
                dribbler_speed=self.dribbler_speed,
            )
            if self.time_in_state() >= self.catch_dwell:
                self.post_reset_state = (
                    State.WAIT_FOR_BALL if self.loop else State.DONE)
                # Optionally capture the ball if it ended up close by.
                if self._enter_capture_after_catch_if_possible():
                    return cmd
                if self.kick_back_enabled:
                    self.transition(State.KICK_BACK)
                else:
                    self.transition(State.RESET)
            return cmd

        if s == State.KICK_BACK:
            rs = self.robot_xy_yaw()
            if rs is None:
                return self.make_off_cmd()
            half_l = (self.field.field_length / 2.0
                      if self.field is not None
                      and self.field.field_length > 0.0
                      else 4.5)
            aim_theta = math.atan2(0.0 - rs[1], half_l - rs[0])
            self.kick_back_pose = (rs[0], rs[1], aim_theta)
            pc = self.kick_back_pivot
            # `kick_back_max_angular_vel` (>0) overrides the shared pivot
            # angular-velocity limit; otherwise use the pivot config value.
            ang_vel = (self.kick_back_max_angular_vel
                       if self.kick_back_max_angular_vel > 0.0
                       else pc.max_angular_vel)
            yaw_ok = abs(ang_diff(rs[2], aim_theta)) <= self.yaw_tol
            now = self.get_clock().now()
            if yaw_ok:
                if self.kick_back_aim_arrived_at is None:
                    self.kick_back_aim_arrived_at = now
                aim_elapsed = (
                    now - self.kick_back_aim_arrived_at).nanoseconds * 1e-9
                if aim_elapsed >= self.kick_back_aim_dwell:
                    # Fire kick (still pivoting to hold the heading).
                    cmd = make_pivot_cmd(
                        aim_theta, pc.orbit_radius, pc.inset_angle,
                        ang_vel_limit=ang_vel,
                        ang_acc_limit=pc.max_angular_acc,
                        kick_request=RobotMotionCommand.KR_KICK_NOW,
                        kick_speed=self.kick_back_speed,
                        dribbler_speed=self.dribbler_speed,
                    )
                    if aim_elapsed >= (self.kick_back_aim_dwell
                                       + self.kick_back_post_dwell):
                        self.transition(State.RESET)
                    return cmd
            else:
                self.kick_back_aim_arrived_at = None
            # Pivot toward the kick-back heading, holding the ball on the
            # dribbler via the firmware BCM_PIVOT maneuver.
            return make_pivot_cmd(
                aim_theta, pc.orbit_radius, pc.inset_angle,
                ang_vel_limit=ang_vel,
                ang_acc_limit=pc.max_angular_acc,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )

        if s == State.COLLECT_APPROACH:
            if self.collect_ball_xy is None:
                self.transition(State.RESET)
                return self.make_off_cmd()
            rs = self.robot_xy_yaw()
            if rs is None:
                return self.make_off_cmd()
            bx, by = self.collect_ball_xy
            sx, sy = self.S if self.S is not None else (rs[0], rs[1])
            dx = bx - sx
            dy = by - sy
            d = math.hypot(dx, dy)
            if d < 1e-6:
                # Robot start coincides with ball — just capture.
                self.transition(State.COLLECT_CAPTURE)
                return self.make_off_cmd()
            ux, uy = dx / d, dy / d
            theta = math.atan2(uy, ux)
            back = self.collect_approach_offset + self.robot_front_offset
            tx = bx - back * ux
            ty = by - back * uy
            if not self.in_bounds(tx, ty):
                self.get_logger().warn(
                    'COLLECT approach target out of bounds; aborting.',
                    throttle_duration_sec=1.0)
                self.transition(State.RESET)
                return self.make_off_cmd()
            cmd = self.make_pos_cmd(
                tx, ty, theta,
                kick_request=RobotMotionCommand.KR_DISABLE,
                dribbler_speed=self.dribbler_speed,
            )
            if (math.hypot(rs[0] - tx, rs[1] - ty) <= self.pos_tol
                    and abs(ang_diff(rs[2], theta)) <= self.yaw_tol):
                self.transition(State.COLLECT_CAPTURE)
            return cmd

        if s == State.COLLECT_CAPTURE:
            if self.collect_ball_xy is None:
                self.transition(State.RESET)
                return self.make_off_cmd()
            rs = self.robot_xy_yaw()
            if rs is None:
                return self.make_off_cmd()
            bx, by = self.collect_ball_xy
            sx, sy = self.S if self.S is not None else (rs[0], rs[1])
            dx = bx - sx
            dy = by - sy
            d = math.hypot(dx, dy)
            if d < 1e-6:
                self.transition(State.RESET)
                return self.make_off_cmd()
            ux, uy = dx / d, dy / d
            theta = math.atan2(uy, ux)
            forward = -self.robot_front_offset + self.collect_capture_overdrive
            tx = bx + forward * ux
            ty = by + forward * uy
            cmd = self.make_pos_cmd(
                tx, ty, theta,
                vel_limit=self.collect_capture_vel_limit,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            if (math.hypot(rs[0] - tx, rs[1] - ty) <= self.pos_tol
                    and abs(ang_diff(rs[2], theta)) <= self.yaw_tol):
                if self.time_in_state() >= self.collect_capture_dwell:
                    self.catch_dwell_pose = (rs[0], rs[1], rs[2])
                    self.post_reset_state = (
                        State.WAIT_FOR_BALL if self.loop else State.DONE)
                    if self.kick_back_enabled:
                        self.transition(State.KICK_BACK)
                    else:
                        self.transition(State.RESET)
            return cmd

        if s == State.CAPTURE_SKILL:
            # Drive over and pick up the nearby ball using the shared
            # Capture skill (honors approach_radius / capture_speed /
            # capture_accel / capture_filter_count / use_breakbeam, ...).
            rs = self.robot_xy_yaw()
            ball_xy = None
            ball_visible = self.ball is not None and self.ball.visible
            if ball_visible:
                bp = self.ball.pose.position
                ball_xy = (bp.x, bp.y)
            elif self.collect_ball_xy is not None:
                ball_xy = self.collect_ball_xy
            cmd = self.capture.run_frame(
                rs, self.robot_speed(), ball_xy,
                ball_visible, self.breakbeam_detected)
            if self.capture.is_done():
                if rs is not None:
                    self.catch_dwell_pose = (rs[0], rs[1], rs[2])
                self.post_reset_state = (
                    State.WAIT_FOR_BALL if self.loop else State.DONE)
                if self.kick_back_enabled:
                    self.transition(State.KICK_BACK)
                else:
                    self.transition(State.RESET)
                return self.make_off_cmd()
            return cmd if cmd is not None else self.make_off_cmd()

        if s == State.RESET:
            if self.field is None or self.field.field_length <= 0.0:
                self.get_logger().warn(
                    'Waiting for /field before reset...',
                    throttle_duration_sec=2.0)
                return self.make_off_cmd()
            tx = -self.field.field_length / 2.0 + self.reset_margin
            ty = 0.0
            rs = self.robot_xy_yaw()
            if rs is not None and math.hypot(
                    rs[0] - tx, rs[1] - ty) <= self.pos_tol:
                # Keep latched overlays visible across RESET arrival;
                # they are cleared when the next ball cycle starts in
                # WAIT_FOR_INCOMING.
                self.transition(self.post_reset_state)
            return self.make_pos_cmd(tx, ty, 0.0)

        if s == State.DONE:
            return self.make_off_cmd()

        return self.make_off_cmd()

    # ------------------------------------------------------------- overlays
    def publish_overlays(self):
        items = []

        # Use the same low tracking floor as the INTERCEPT solver so the
        # ball-trajectory line stays visible (and matches the robot's
        # adjustment) as long as the ball is still moving forward.
        ball = self.ball_state(self.min_ball_speed_tracking)
        if ball is not None:
            (bx, by), (vx, vy), speed = ball
            # Ball line extended by ~2x current solution distance (or
            # 3m default) along velocity direction.
            line_len = 3.0
            if self.last_solution_d is not None:
                line_len = max(2.0 * self.last_solution_d, 1.0)
            ex = bx + (vx / speed) * line_len
            ey = by + (vy / speed) * line_len
            items.append(make_line(
                OVERLAY_NS, 'ball_line',
                [(bx, by), (ex, ey)],
                color='#FFA500FF', stroke_width=2))

        if self.intercept_P is not None:
            # Ball at the intercept point: ball-sized filled circle.
            items.append(make_point(
                OVERLAY_NS, 'intercept_dribbler',
                (self.intercept_P[0], self.intercept_P[1]),
                color='#00FF7FFF', radius=self.ball_radius))
            theta = self.intercept_P[2]
            cx = self.intercept_P[0] - self.robot_front_offset * math.cos(
                theta)
            cy = self.intercept_P[1] - self.robot_front_offset * math.sin(
                theta)
            # Robot at the intercept: robot-sized outline + heading.
            items.append(make_circle(
                OVERLAY_NS, 'intercept_center', (cx, cy),
                self.robot_radius,
                stroke_color='#00FF7FFF', fill_color='#00000000',
                stroke_width=2))
            items.append(make_pose_marker(
                OVERLAY_NS, 'intercept_center_pose', (cx, cy, theta),
                self.pos_tol, color='#00FF7FFF',
                heading_length=self.robot_radius + 0.05))

        if (self.mode == 'complex' and self.intercept_target is not None
                and self.intercept_P is not None):
            tx, ty, ttheta = self.intercept_target
            # Robot at firmware target (end of bang-bang traj): robot-sized.
            items.append(make_circle(
                OVERLAY_NS, 'firmware_target', (tx, ty),
                self.robot_radius,
                stroke_color='#00BFFFFF', fill_color='#00000000',
                stroke_width=2))
            items.append(make_pose_marker(
                OVERLAY_NS, 'firmware_target_pose', (tx, ty, ttheta),
                self.pos_tol, color='#00BFFFFF',
                heading_length=self.robot_radius + 0.05))
            # Segment from intercept center -> firmware target (the
            # second half of the symmetric trajectory).
            theta = self.intercept_P[2]
            cx = self.intercept_P[0] - self.robot_front_offset * math.cos(
                theta)
            cy = self.intercept_P[1] - self.robot_front_offset * math.sin(
                theta)
            items.append(make_line(
                OVERLAY_NS, 'trajectory_second_half',
                [(cx, cy), (tx, ty)],
                color='#00BFFFFF', stroke_width=2))
            if self.S is not None:
                items.append(make_line(
                    OVERLAY_NS, 'trajectory_first_half',
                    [(self.S[0], self.S[1]), (cx, cy)],
                    color='#00BFFFFF', stroke_width=2))

        if self.show_detected_collision_overlay:
            if self.collision_ball_xy is not None:
                cbx, cby = self.collision_ball_xy
                items.append(make_point(
                    OVERLAY_NS, 'collision_ball', (cbx, cby),
                    color='#FF0000FF', radius=self.ball_radius))
            if self.collision_robot_pose is not None:
                crx, cry, crt = self.collision_robot_pose
                items.append(make_circle(
                    OVERLAY_NS, 'collision_robot', (crx, cry),
                    self.robot_radius,
                    stroke_color='#FF0000FF', fill_color='#00000000',
                    stroke_width=2))
                items.append(make_pose_marker(
                    OVERLAY_NS, 'collision_robot_pose', (crx, cry, crt),
                    self.pos_tol, color='#FF0000FF',
                    heading_length=self.robot_radius + 0.05))

        # (The realized intercept point is rendered in red via the
        # collision_ball / collision_robot captures above.)

        # State/time status text - placed outside the playable field
        # (below the negative-y touchline by `status_text_margin`).
        # Configurable via status_text_x / status_text_y; if both are
        # finite, that absolute position overrides the auto placement.
        sx = self._status_text_x
        sy = self._status_text_y
        if (sx is None or sy is None) and self.field is not None \
                and self.field.field_width > 0.0:
            half_w = self.field.field_width / 2.0
            boundary = getattr(self.field, 'boundary_width', 0.0) or 0.0
            sx = 0.0 if sx is None else sx
            sy = -(half_w + boundary + self.status_text_margin) \
                if sy is None else sy
        if sx is not None and sy is not None:
            label = f'{self.state.name}  mode={self.mode}'
            if self.last_solution_T is not None:
                label += f'  T={self.last_solution_T:.2f}s'
            items.append(make_text(
                OVERLAY_NS, 'state', label, (sx, sy),
                color='#FFFFFFFF', font_size=20))

        if items:
            self.overlay_pub.publish(make_array(*items))


def main(args=None):
    rclpy.init(args=args)
    node = CatchScenario()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
