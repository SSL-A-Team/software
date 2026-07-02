"""
Juke shot scenario node for profiling robot motion performance.

A "juke shot" is a quick juking movement followed by a shot on goal. The
goal is at +y, so the scenario ends with a final in-place pivot that turns
the robot to face +y before kicking the captured ball into the net.

The juking movement itself ("juke type": none, linear, pivot, double
linear, double pivot) is parameterized. ``none`` skips the juke entirely
and goes straight from capture to the ending pivot toward the goal; the
other types are reserved for upcoming increments.

Setup
-----
When ``drive_to_start`` is true, the scenario first drives the robot to
``(start_pose_x, start_pose_y, start_pose_theta_deg)`` using a global
position command, so each run starts from a known place. This works on
physical hardware (no simulator teleport required).

Geometry conventions
--------------------
The ball is dribbled directly in front of the robot, along the robot's
local +x axis (``robot_front_offset`` meters ahead of the body center).
The robot picks the ball up from the *nearest side*: it stages on the side
of the ball closest to the robot's position (latched when the ball goes
stationary), facing the ball, then drives forward to capture. After capture
(and any future juke) it pivots to shoot: the goal heading is computed as
the direction from the robot to the goal point ``(goal_x, goal_y)`` (a
point on the +y goalline at x = 0), and the robot rotates from its current
heading to that goal heading along an arc of radius ``final_pivot_radius``
(0 = pivot in place) before kicking.

Parameters are loaded from a JSON file (``config/juke_shot_params.json``
by default, override with the ``param_file`` ROS parameter). Every key in
the JSON becomes the default for a ROS parameter of the same name, so any
value can still be overridden on the command line with
``--ros-args -p name:=value``.
"""

from enum import auto, Enum
import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from ateam_motion_scenarios.common.capture import (
    Capture,
    CaptureConfig,
    CapturePhase,
    load_capture_defaults,
)
from ateam_motion_scenarios.common.overlays import (
    make_array,
    make_point,
    make_pose_marker,
    make_text,
)
from ateam_motion_scenarios.common.pivot import (
    load_pivot_defaults,
    make_heading_pivot_cmd,
    make_point_pivot_cmd,
)
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
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


OVERLAY_NS = 'juke_shot_scenario'
DEFAULT_PARAM_FILE = os.path.join(
    get_package_share_directory('ateam_motion_scenarios'),
    'config', 'juke_shot_params.json')


class State(Enum):
    WAIT_FOR_CONNECTION = auto()
    PLACE_ROBOT = auto()
    WAIT_FOR_BALL = auto()
    WAIT_FOR_STATIONARY = auto()
    CAPTURE = auto()
    PRE_JUKE_PIVOT = auto()
    LINEAR_JUKE = auto()
    FINAL_PIVOT = auto()
    KICK = auto()
    DONE = auto()


def yaw_from_quat(q) -> float:
    """Extract yaw from a planar quaternion (assumes roll=pitch=0)."""
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


class JukeShotScenario(Node):

    def __init__(self):
        super().__init__('juke_shot_scenario')

        self.defaults = self._load_defaults()

        # Robot / team selection.
        self.robot_id = int(self._p('robot_id', 2))
        self.team_color = str(self._p('team_color', 'blue'))

        # Ball-stationary gate before the capture skill begins.
        self.stationary_ball_threshold = float(
            self._p('stationary_ball_threshold', 0.005))
        self.stationary_dwell = float(self._p('stationary_dwell', 0.5))
        # Shared capture defaults (config/skill_capture_params.json) source the
        # robot front offset and breakbeam topic so they stay consistent with
        # the capture skill; either can still be overridden per scenario.
        cap_def = load_capture_defaults()
        # Dribbler distance ahead of the robot center (used by the final-pivot
        # aim solver; the capture skill reads its own copy via CaptureConfig).
        self.robot_front_offset = float(
            self._p('robot_front_offset', cap_def['robot_front_offset']))
        # Basic-telemetry topic carrying the breakbeam (empty -> default for
        # this robot). The capture skill consumes the breakbeam value.
        self.breakbeam_topic = str(
            self._p('breakbeam_topic', cap_def['breakbeam_topic'])) \
            or f'/robot_feedback/basic/robot{self.robot_id}'

        self.pos_tol = float(self._p('pos_tol', 0.03))
        self.yaw_tol = float(self._p('yaw_tol', 0.05))

        self.dribbler_speed = float(self._p('dribbler_speed', 300.0))

        # Motion limits (0.0 means "use firmware default").
        self.limit_vel_linear = float(self._p('limit_vel_linear', 1.0))
        self.limit_vel_angular = float(
            self._p('limit_vel_angular', 2.0 * math.pi))
        self.limit_acc_linear = float(self._p('limit_acc_linear', 1.0))
        self.limit_acc_angular = float(
            self._p('limit_acc_angular', 2.0 * math.pi))

        # Shared two-phase ball-capture skill (see capture.py). It declares
        # its own parameters via the same JSON-backed getter.
        self.capture = Capture(CaptureConfig.from_params(self._p))

        # Juke params. ``juke_type`` selects the post-capture move before the
        # ending pivot:
        #   'none'   -> straight to the final pivot.
        #   'linear' -> a sideways linear juke (emulating shooting around a
        #               defender): the robot moves along +x (toward the field
        #               +x edge) while holding a heading inset toward the goal,
        #               then enters the final pivot once juke_distance is
        #               travelled.
        self.juke_type = str(self._p('juke_type', 'none')).lower()
        # Sideways distance (m, along +x) travelled before the final pivot.
        self.juke_distance = float(self._p('juke_distance', 0.5))
        # Velocity / acceleration limits for the linear juke move.
        self.juke_speed = float(self._p('juke_speed', 1.0))
        self.juke_accel = float(self._p('juke_accel', 1.0))
        # Heading held during the linear juke, as the angle from the
        # horizontal (+x) movement line toward the goal. A sentinel < 0
        # means "use the firmware pivot inset" (resolved below).
        self.juke_inset_angle = float(self._p('juke_inset_angle', -1.0))
        # If true, command a global position straight to the field +x edge and
        # switch to the final pivot once juke_distance has been travelled --
        # entering the pivot at non-zero velocity (before the firmware
        # trajectory completes). If false, target exactly juke_distance away so
        # the robot arrives (near zero velocity) before pivoting.
        self.juke_target_field_edge = bool(
            self._p('juke_target_field_edge', True))
        # Fallback +x field-edge x-coordinate when field geometry is absent.
        self.juke_field_edge_x = float(self._p('juke_field_edge_x', 4.5))
        # If true, pivot (around the ball) to the juke inset heading before
        # starting the linear juke.
        self.juke_pre_pivot = bool(self._p('juke_pre_pivot', True))

        if self.juke_type not in ('none', 'linear'):
            self.get_logger().warn(
                f"juke_type '{self.juke_type}' is not implemented; "
                "treating it as 'none' (capture then pivot to goal)")
            self.juke_type = 'none'

        # Ending (final) pivot toward the goal, executed with the firmware's
        # built-in BCM_POINT_PIVOT maneuver. The firmware derives the orbit
        # center from the robot's current pose, ``final_pivot_radius`` (orbit
        # radius) and ``final_pivot_inset_angle``, then orbits until the robot
        # faces the goal point ``(goal_x, goal_y)`` -- accounting for the body
        # translation during the orbit, so the firmware (not this scenario)
        # solves the aim. The firmware-pivot tuning defaults (orbit radius,
        # inset, angular limits) come from the shared
        # ``config/skill_pivot_params.json`` so they stay consistent with every
        # other pivoting scenario; either can still be overridden per scenario.
        # These params are independent of the juke pivot / double pivot above.
        # When ``goal_y_dynamic`` is true the +y goalline is taken from the
        # live field geometry (top edge); ``goal_y`` is only the fallback.
        piv_def = load_pivot_defaults()
        self.goal_x = float(self._p('goal_x', 0.0))
        self.goal_y_dynamic = bool(self._p('goal_y_dynamic', True))
        self.goal_y = float(self._p('goal_y', 4.5))
        self.final_pivot_radius = float(
            self._p('final_pivot_radius', piv_def['orbit_radius']))
        self.final_pivot_inset_angle = float(
            self._p('final_pivot_inset_angle', piv_def['inset_angle']))
        # Resolve the linear-juke inset default to the firmware pivot inset.
        if self.juke_inset_angle < 0.0:
            self.juke_inset_angle = self.final_pivot_inset_angle
        self.final_pivot_angular_vel = float(
            self._p('final_pivot_angular_vel', piv_def['max_angular_vel']))
        self.final_pivot_angular_acc = float(
            self._p('final_pivot_angular_acc', piv_def['max_angular_acc']))
        self.final_pivot_dwell = float(self._p('final_pivot_dwell', 0.2))
        self.final_pivot_yaw_tol = float(
            self._p('final_pivot_yaw_tol', 0.05))

        # Kick.
        self.kick_speed = float(self._p('kick_speed', 5.0))
        self.kick_dwell = float(self._p('kick_dwell', 0.3))

        self.loop = bool(self._p('loop', True))
        self.loop_dwell = float(self._p('loop_dwell', 1.0))

        # Startup placement. Drives the robot to a known start pose with a
        # BCM_GLOBAL_POSITION command before the scenario begins so each run
        # is repeatable. Works on physical hardware (no simulator teleport).
        self.drive_to_start = bool(self._p('drive_to_start', True))
        self.start_pose_x = float(self._p('start_pose_x', -0.5))
        self.start_pose_y = float(self._p('start_pose_y', -0.5))
        self.start_pose_theta_rad = math.radians(
            float(self._p('start_pose_theta_deg', 0.0)))
        self.start_dwell = float(self._p('start_dwell', 0.3))

        # Radio connection monitoring. When ``require_connection`` is true the
        # scenario waits for the robot's radio link before driving and
        # restarts the state machine if the link drops mid-run.
        self.require_connection = bool(self._p('require_connection', True))
        self.connection_topic = str(self._p('connection_topic', '')) \
            or f'/robot_feedback/connection/robot{self.robot_id}'

        rate = float(self._p('publish_rate_hz', 60.0))

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Match the joystick node's QoS for robot topics:
        # SystemDefaultsQoS().keep_last(1) == RELIABLE + VOLATILE, depth 1.
        connection_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.ball = None
        self.robot = None
        self.field = None
        self.radio_connected = None  # None until first connection message
        self.breakbeam_detected = None  # None until first basic telemetry

        self.ball_sub = self.create_subscription(
            VisionStateBall, '/ball', self.ball_cb, sensor_qos)
        self.robot_sub = self.create_subscription(
            VisionStateRobot, f'/{self.team_color}_team/robot{self.robot_id}',
            self.robot_cb, sensor_qos)
        self.field_sub = self.create_subscription(
            FieldInfo, '/field', self.field_cb, 10)
        self.connection_sub = self.create_subscription(
            ConnectionStatus, self.connection_topic,
            self.connection_cb, connection_qos)
        # Basic telemetry carries breakbeam state (RELIABLE + VOLATILE, like
        # the radio bridge's SystemDefaultsQoS publisher).
        self.telemetry_sub = self.create_subscription(
            BasicTelemetry, self.breakbeam_topic,
            self.telemetry_cb, connection_qos)

        self.cmd_pub = self.create_publisher(
            RobotMotionCommand,
            f'/robot_motion_commands/robot{self.robot_id}', 10)
        self.overlay_pub = self.create_publisher(
            OverlayArray, '/overlays', 10)

        # State machine. Start by waiting for the radio link when connection
        # monitoring is enabled; otherwise jump straight to placement.
        self.state = (State.WAIT_FOR_CONNECTION if self.require_connection
                      else State.PLACE_ROBOT)
        self.state_entered = self.get_clock().now()
        self.stationary_since = None
        self.ball_xy = None       # latched ball (x, y) at WAIT_FOR_STATIONARY
        self.aim_since = None      # time yaw first within tolerance in pivot
        self.kick_latched = False  # once aimed, latch the kick command on
        self.start_arrived_at = None  # time robot first reached start pose
        self.juke_start_xy = None  # latched robot (x, y) at juke start
        self.juke_heading = 0.0    # heading held during the linear juke

        # Final-pivot geometry: the goal point is sent to the firmware as a
        # BCM_POINT_PIVOT command; these latched values are the closed-form
        # *prediction* of the resulting orbit, used only for the overlay.
        self.final_goal_heading = None  # predicted heading (faces ->goal)
        self.final_center = None        # predicted pivot orbit center (x, y)
        self.final_radius = 0.0         # pivot orbit radius
        self.final_pose = None          # predicted final robot (x, y, theta)

        self.timer = self.create_timer(1.0 / rate, self.tick)

        goal_y_desc = ('field top edge' if self.goal_y_dynamic
                       else f'{self.goal_y:.2f}')
        self.get_logger().info(
            f'juke_shot_scenario started for robot {self.robot_id} '
            f'({self.team_color}). juke_type={self.juke_type}, '
            f'goal=(x={self.goal_x:.2f}, y={goal_y_desc}), '
            f'final_pivot_radius={self.final_pivot_radius:.2f}. '
            f'State: {self.state.name}')

    # --------------------------------------------------------------- params
    def _load_defaults(self) -> dict:
        self.declare_parameter('param_file', '')
        pf = str(self.get_parameter('param_file').value) or DEFAULT_PARAM_FILE
        try:
            with open(pf) as f:
                data = json.load(f)
            self.get_logger().info(f'Loaded juke params from {pf}')
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

    # ------------------------------------------------------------------ subs
    def ball_cb(self, msg: VisionStateBall):
        self.ball = msg

    def robot_cb(self, msg: VisionStateRobot):
        self.robot = msg

    def field_cb(self, msg: FieldInfo):
        self.field = msg

    def connection_cb(self, msg: ConnectionStatus):
        self.radio_connected = msg.radio_connected

    def telemetry_cb(self, msg: BasicTelemetry):
        self.breakbeam_detected = msg.breakbeam_ball_detected

    # --------------------------------------------------------------- helpers
    def transition(self, new_state: State):
        self.get_logger().info(
            f'State {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_entered = self.get_clock().now()
        self.stationary_since = None
        self.aim_since = None

    def time_in_state(self) -> float:
        return (self.get_clock().now() - self.state_entered).nanoseconds * 1e-9

    def robot_xy_yaw(self):
        if self.robot is None or not self.robot.visible:
            return None
        p = self.robot.pose.position
        return (p.x, p.y, yaw_from_quat(self.robot.pose.orientation))

    def ball_speed(self) -> float:
        if self.ball is None:
            return float('inf')
        v = self.ball.twist.linear
        return math.hypot(v.x, v.y)

    def robot_speed(self) -> float:
        if self.robot is None or not self.robot.visible:
            return float('inf')
        v = self.robot.twist.linear
        return math.hypot(v.x, v.y)

    def at_pose(self, target_xy, pos_tol: float = None,
                yaw_target: float = 0.0,
                yaw_tol: float = None) -> bool:
        if pos_tol is None:
            pos_tol = self.pos_tol
        if yaw_tol is None:
            yaw_tol = self.yaw_tol
        rs = self.robot_xy_yaw()
        if rs is None:
            return False
        rx, ry, ryaw = rs
        if math.hypot(rx - target_xy[0], ry - target_xy[1]) > pos_tol:
            return False
        if abs(ang_diff(ryaw, yaw_target)) > yaw_tol:
            return False
        return True

    def make_pos_cmd(self, x: float, y: float, theta: float = 0.0,
                     vel_limit: float = 0.0,
                     acc_limit: float = 0.0,
                     ang_vel_limit: float = 0.0,
                     ang_acc_limit: float = 0.0,
                     kick_request: int = RobotMotionCommand.KR_DISABLE,
                     dribbler_speed: float = 0.0,
                     kick_speed: float = 0.0) -> RobotMotionCommand:
        cmd = RobotMotionCommand()
        cmd.body_control_mode = RobotMotionCommand.BCM_GLOBAL_POSITION
        cmd.pose = Twist2D(x=float(x), y=float(y), theta=float(theta))
        cmd.velocity = Twist2D()
        cmd.acceleration = Twist2D()
        cmd.limit_vel_linear = (
            float(vel_limit) if vel_limit > 0.0 else self.limit_vel_linear)
        cmd.limit_vel_angular = (
            float(ang_vel_limit) if ang_vel_limit > 0.0
            else self.limit_vel_angular)
        cmd.limit_acc_linear = (
            float(acc_limit) if acc_limit > 0.0 else self.limit_acc_linear)
        cmd.limit_acc_angular = (
            float(ang_acc_limit) if ang_acc_limit > 0.0
            else self.limit_acc_angular)
        cmd.kick_request = kick_request
        cmd.kick_speed = float(kick_speed)
        cmd.dribbler_speed = float(dribbler_speed)
        return cmd

    def make_pivot_cmd(self, target_heading: float, orbit_radius: float,
                       inset_angle: float = 0.0,
                       ang_vel_limit: float = 0.0,
                       ang_acc_limit: float = 0.0,
                       kick_request: int = RobotMotionCommand.KR_DISABLE,
                       dribbler_speed: float = 0.0,
                       kick_speed: float = 0.0) -> RobotMotionCommand:
        """
        Build a firmware BCM_HEADING_PIVOT command (delegates to the shared
        builder).

        The firmware derives the orbit center from the robot's current pose,
        ``orbit_radius`` and ``inset_angle``; only the target heading is
        commanded. Unset angular limits fall back to the scenario defaults.
        """
        return make_heading_pivot_cmd(
            target_heading, orbit_radius, inset_angle,
            ang_vel_limit if ang_vel_limit > 0.0 else self.limit_vel_angular,
            ang_acc_limit if ang_acc_limit > 0.0 else self.limit_acc_angular,
            kick_request=kick_request,
            dribbler_speed=dribbler_speed,
            kick_speed=kick_speed,
        )

    def make_off_cmd(self) -> RobotMotionCommand:
        cmd = RobotMotionCommand()
        cmd.body_control_mode = RobotMotionCommand.BCM_OFF
        cmd.pose = Twist2D()
        cmd.velocity = Twist2D()
        cmd.acceleration = Twist2D()
        cmd.kick_request = RobotMotionCommand.KR_DISABLE
        return cmd

    # ------------------------------------------------------------------ tick
    def tick(self):
        self._check_connection()
        cmd = self.step_state_machine()
        if cmd is not None:
            self.cmd_pub.publish(cmd)
        self.publish_overlays()

    def _check_connection(self):
        """Restart the state machine if the robot's radio link drops."""
        if not self.require_connection:
            return
        # Only react once we are actually running (past the connection wait)
        # and the link is known to be down.
        if self.state == State.WAIT_FOR_CONNECTION:
            return
        if self.radio_connected is False:
            self.get_logger().warn(
                'Robot disconnected; restarting state machine')
            self._reset_run_state()
            self.transition(State.WAIT_FOR_CONNECTION)

    def _reset_run_state(self):
        """Clear all latched per-run state."""
        self.ball_xy = None
        self.final_center = None
        self.final_pose = None
        self.start_arrived_at = None
        self.stationary_since = None
        self.aim_since = None
        self.kick_latched = False
        self.juke_start_xy = None
        self.capture.reset()

    # --------------------------------------------------------------- pickup
    def ball_pos(self):
        """Return the live ball (x, y) if visible, else the latched ball."""
        if self.ball is not None and self.ball.visible:
            p = self.ball.pose.position
            return (p.x, p.y)
        return self.ball_xy

    # ----------------------------------------------------------- final pivot
    def _goal_y(self) -> float:
        """
        Return the +y goalline coordinate.

        Uses the live field geometry's top edge (``field_width / 2``) when
        ``goal_y_dynamic`` is enabled and a field is available; otherwise
        falls back to the ``goal_y`` parameter.
        """
        if self.goal_y_dynamic and self.field is not None:
            half = self.field.field_width / 2.0
            if half > 1e-6:
                return half
        return self.goal_y

    # ------------------------------------------------------------ linear juke
    def _field_edge_x(self) -> float:
        """Return the +x field-edge x (live geometry or fallback)."""
        if self.field is not None and self.field.field_length > 1e-6:
            return self.field.field_length / 2.0
        return self.juke_field_edge_x

    def _enter_linear_juke(self):
        """Latch the linear-juke start position and the held heading."""
        rs = self.robot_xy_yaw()
        if rs is None:
            self.juke_start_xy = (0.0, 0.0)
            self.juke_heading = self.juke_inset_angle
            return
        rx, ry, _ = rs
        self.juke_start_xy = (rx, ry)
        # Move along +x; hold a heading inset from that line toward the goal.
        sign = 1.0 if self._goal_y() >= ry else -1.0
        self.juke_heading = sign * self.juke_inset_angle
        tgt = 'field +x edge' if self.juke_target_field_edge else 'distance'
        self.get_logger().info(
            f'Linear juke: start=({rx:.2f}, {ry:.2f}), '
            f'heading={math.degrees(self.juke_heading):.1f} deg, '
            f'distance={self.juke_distance:.2f} m, target={tgt}')

    def _begin_juke_or_pivot(self):
        """Branch out of CAPTURE into the juke or straight to the pivot."""
        if self.juke_type == 'linear':
            self._enter_linear_juke()
            if self.juke_pre_pivot:
                self.transition(State.PRE_JUKE_PIVOT)
            else:
                self.transition(State.LINEAR_JUKE)
        else:
            self._enter_final_pivot()
            self.transition(State.FINAL_PIVOT)

    def _pivot_geometry(self, rx, ry, ryaw, theta_target):
        """
        Reproduce the firmware pivot geometry for a candidate target heading.

        Returns ``(center_x, center_y, final_x, final_y, heading_offset)``
        where ``(final_x, final_y)`` is the robot-center position at the end
        of the orbit (heading == ``theta_target``).
        """
        r = max(self.final_pivot_radius, 1e-6)
        d = 1.0 if ang_diff(theta_target, ryaw) >= 0.0 else -1.0
        heading_offset = math.pi - d * self.final_pivot_inset_angle
        orbit_start = ryaw - heading_offset
        cx = rx - r * math.cos(orbit_start)
        cy = ry - r * math.sin(orbit_start)
        phi_target = theta_target - heading_offset
        fx = cx + r * math.cos(phi_target)
        fy = cy + r * math.sin(phi_target)
        return cx, cy, fx, fy, heading_offset

    def _solve_aim_heading(self, rx, ry, ryaw, goal_x, goal_y):
        """
        Closed-form target heading that aims the ball at the goal post-pivot.

        The orbit center is fixed by the robot's current pose, the orbit
        radius ``r`` and the inset angle (it does not move during the pivot).
        At orbit angle phi the robot is at ``center + r*(cos phi, sin phi)``
        with heading ``theta = phi + ho`` where ``ho = pi - d*inset``.
        Requiring the heading ray to pass through the goal reduces to

            L * sin(theta - alpha) = r * sin(ho)

        where ``L`` and ``alpha`` are the distance and bearing from the orbit
        center to the goal. (The dribbler front-offset cancels: sliding the
        ball along the aim ray does not change the ray's direction.) Each
        turn direction ``d`` gives a fixed center and up to two roots; the
        smallest self-consistent turn whose heading points the ball *toward*
        the goal is chosen. Falls back to the least-error front-facing
        heading when the goal lies inside the orbit's tangent envelope (no
        exact aim exists, e.g. capturing very close to the goal line).
        """
        r = max(self.final_pivot_radius, 1e-6)
        best = None       # (abs_turn, theta): self-consistent and front-facing
        fallback = None   # (residual, theta): front-facing only
        for d in (1.0, -1.0):
            ho = math.pi - d * self.final_pivot_inset_angle
            orbit_start = ryaw - ho
            cx = rx - r * math.cos(orbit_start)
            cy = ry - r * math.sin(orbit_start)
            dist = math.hypot(goal_x - cx, goal_y - cy)
            if dist < 1e-9:
                continue
            s = r * math.sin(ho) / dist
            if abs(s) > 1.0:
                continue
            alpha = math.atan2(goal_y - cy, goal_x - cx)
            asin_s = math.asin(s)
            for theta in (alpha + asin_s, alpha + math.pi - asin_s):
                theta = math.atan2(math.sin(theta), math.cos(theta))
                phi = theta - ho
                bx = cx + r * math.cos(phi) \
                    + self.robot_front_offset * math.cos(theta)
                by = cy + r * math.sin(phi) \
                    + self.robot_front_offset * math.sin(theta)
                # Reject headings that point away from the goal.
                if (goal_x - bx) * math.cos(theta) \
                        + (goal_y - by) * math.sin(theta) <= 0.0:
                    continue
                turn = ang_diff(theta, ryaw)
                consistent = (1.0 if turn >= 0.0 else -1.0) == d
                residual = abs(ang_diff(
                    math.atan2(goal_y - by, goal_x - bx), theta))
                if consistent and (best is None or abs(turn) < best[0]):
                    best = (abs(turn), theta)
                if fallback is None or residual < fallback[0]:
                    fallback = (residual, theta)
        if best is not None:
            return best[1]
        if fallback is not None:
            self.get_logger().warn(
                'No exact pivot aim solution (goal inside orbit envelope); '
                'using closest heading')
            return fallback[1]
        return math.atan2(goal_y - ry, goal_x - rx)

    def _enter_final_pivot(self):
        """
        Latch the goal and predict the final-pivot pose for visualization.

        The actual aim is now performed by the firmware ``BCM_POINT_PIVOT``
        maneuver (see :meth:`_final_pivot_cmd`), which orbits until the robot
        faces the goal point, accounting for the body translation. The
        closed-form solver below is retained only to predict the orbit center
        and final pose for the overlay; it does not drive the command.
        """
        goal_y = self._goal_y()
        rs = self.robot_xy_yaw()
        if rs is None:
            # No pose available; fall back to facing +y.
            self.final_goal_heading = math.pi / 2.0
            self.final_center = None
            self.final_radius = self.final_pivot_radius
            self.final_pose = (0.0, 0.0, math.pi / 2.0)
            return
        rx, ry, ryaw = rs

        r = max(self.final_pivot_radius, 1e-6)
        self.final_radius = r

        # Target heading: aim the ball at the goal from the robot's FINAL
        # (post-orbit) position, not its current position.
        theta_goal = self._solve_aim_heading(rx, ry, ryaw, self.goal_x, goal_y)
        self.final_goal_heading = theta_goal

        cx, cy, fx, fy, _ = self._pivot_geometry(rx, ry, ryaw, theta_goal)
        self.final_center = (cx, cy)
        self.final_pose = (fx, fy, theta_goal)

        dtheta = ang_diff(theta_goal, ryaw)
        self.kick_latched = False
        self.get_logger().info(
            f'Final pivot (BCM_POINT_PIVOT): goal=({self.goal_x:.2f}, '
            f'{goal_y:.2f}), predicted_heading={math.degrees(theta_goal):.1f} '
            f'deg, predicted_turn={math.degrees(dtheta):.1f} deg, R={r:.2f}, '
            f'inset={self.final_pivot_inset_angle:.2f} rad')

    def _final_pivot_cmd(self, kick_request, kick_speed=0.0):
        """Build the BCM_POINT_PIVOT command aiming the ball at the goal.

        The firmware orbits (center derived from the robot pose,
        ``final_pivot_radius`` and ``final_pivot_inset_angle``) until the
        robot faces the goal point, accounting for the body translation during
        the orbit -- so the scenario just streams the goal point and lets the
        firmware solve the aim.
        """
        return make_point_pivot_cmd(
            self.goal_x, self._goal_y(), self.final_pivot_radius,
            inset_angle=self.final_pivot_inset_angle,
            ang_vel_limit=self.final_pivot_angular_vel,
            ang_acc_limit=self.final_pivot_angular_acc,
            kick_request=kick_request,
            kick_speed=kick_speed,
            dribbler_speed=self.dribbler_speed,
        )

    # --------------------------------------------------------- visualization
    def publish_overlays(self):
        items = []

        if self.ball_xy is not None:
            items.append(make_point(
                OVERLAY_NS, 'latched_ball', self.ball_xy,
                color='#FFA500FF', radius=0.025))

        s = self.state
        target_color = '#00BFFFFF'
        if s == State.PLACE_ROBOT:
            items.append(make_pose_marker(
                OVERLAY_NS, 'target',
                (self.start_pose_x, self.start_pose_y,
                 self.start_pose_theta_rad),
                self.pos_tol, color=target_color, heading_length=0.3))
        elif s == State.CAPTURE:
            rs = self.robot_xy_yaw()
            bp = self.ball_pos()
            if self.capture.phase == CapturePhase.APPROACH:
                staging = self.capture.staging_target(rs, bp)
                if staging is not None:
                    items.append(make_pose_marker(
                        OVERLAY_NS, 'target', staging,
                        self.capture.config.approach_pos_tol,
                        color=target_color, heading_length=0.3))
            elif bp is not None:
                items.append(make_point(
                    OVERLAY_NS, 'target', bp,
                    color='#00FF7FFF', radius=0.03))
        elif s in (State.PRE_JUKE_PIVOT, State.LINEAR_JUKE):
            rs = self.robot_xy_yaw()
            if rs is not None and self.juke_start_xy is not None:
                if s == State.LINEAR_JUKE:
                    tx = self._field_edge_x() if self.juke_target_field_edge \
                        else self.juke_start_xy[0] + self.juke_distance
                    ty = self.juke_start_xy[1]
                else:
                    tx, ty = rs[0], rs[1]
                items.append(make_pose_marker(
                    OVERLAY_NS, 'target', (tx, ty, self.juke_heading),
                    self.pos_tol, color='#FF8C00FF', heading_length=0.3))
                items.append(make_point(
                    OVERLAY_NS, 'juke_start', self.juke_start_xy,
                    color='#FF8C0080', radius=0.02))
        elif s in (State.FINAL_PIVOT, State.KICK) \
                and self.final_pose is not None:
            items.append(make_pose_marker(
                OVERLAY_NS, 'target', self.final_pose,
                self.pos_tol, color='#FFFF00FF', heading_length=0.3))
            if self.final_center is not None and self.final_radius > 1e-6:
                items.append(make_point(
                    OVERLAY_NS, 'pivot_center', self.final_center,
                    color='#FFFF0080', radius=0.02))
            items.append(make_point(
                OVERLAY_NS, 'goal_point', (self.goal_x, self._goal_y()),
                color='#FF00FFFF', radius=0.05))

        rs = self.robot_xy_yaw()
        if rs is not None:
            items.append(make_text(
                OVERLAY_NS, 'state', self.state.name,
                (rs[0], rs[1] + 0.25),
                color='#FFFFFFFF', font_size=20))

        if items:
            self.overlay_pub.publish(make_array(*items))

    # --------------------------------------------------------- state machine
    def step_state_machine(self):
        s = self.state

        if s == State.WAIT_FOR_CONNECTION:
            if self.radio_connected:
                self.get_logger().info('Robot connected; starting scenario')
                self.transition(State.PLACE_ROBOT)
            return self.make_off_cmd()

        if s == State.PLACE_ROBOT:
            if not self.drive_to_start:
                self.transition(State.WAIT_FOR_BALL)
                return self.make_off_cmd()
            # Drive to the start pose with a global position command and
            # hold there briefly before the scenario proper begins.
            tx, ty = self.start_pose_x, self.start_pose_y
            cmd = self.make_pos_cmd(tx, ty, self.start_pose_theta_rad)
            now = self.get_clock().now()
            if self.at_pose((tx, ty), self.pos_tol,
                            yaw_target=self.start_pose_theta_rad,
                            yaw_tol=self.yaw_tol):
                if self.start_arrived_at is None:
                    self.start_arrived_at = now
                    self.get_logger().info(
                        f'Reached start pose ({tx:.2f}, {ty:.2f})')
                elif (now - self.start_arrived_at).nanoseconds * 1e-9 \
                        >= self.start_dwell:
                    self.transition(State.WAIT_FOR_BALL)
            else:
                self.start_arrived_at = None
            return cmd

        if s == State.WAIT_FOR_BALL:
            if self.ball is not None and self.ball.visible:
                self.transition(State.WAIT_FOR_STATIONARY)
            return self.make_off_cmd()

        if s == State.WAIT_FOR_STATIONARY:
            if self.ball is None or not self.ball.visible:
                self.transition(State.WAIT_FOR_BALL)
                return self.make_off_cmd()
            now = self.get_clock().now()
            if self.ball_speed() < self.stationary_ball_threshold:
                if self.stationary_since is None:
                    self.stationary_since = now
                elif (now - self.stationary_since).nanoseconds * 1e-9 \
                        >= self.stationary_dwell:
                    bp = self.ball.pose.position
                    self.ball_xy = (bp.x, bp.y)
                    self.capture.reset()
                    self.get_logger().info(
                        f'Ball stationary at ({bp.x:.3f}, {bp.y:.3f}); '
                        'capturing')
                    self.transition(State.CAPTURE)
                    return self.make_off_cmd()
            else:
                self.stationary_since = None
            return self.make_off_cmd()

        if s == State.CAPTURE:
            # Delegate to the shared two-phase capture skill (see capture.py).
            rs = self.robot_xy_yaw()
            ball_visible = self.ball is not None and self.ball.visible
            cmd = self.capture.run_frame(
                rs, self.robot_speed(), self.ball_pos(),
                ball_visible, self.breakbeam_detected)
            if self.capture.is_done():
                self._begin_juke_or_pivot()
                return self.make_off_cmd()
            return cmd if cmd is not None else self.make_off_cmd()

        if s == State.PRE_JUKE_PIVOT:
            # Optional pivot (around the ball) to the juke inset heading
            # before the linear juke, using the firmware BCM_HEADING_PIVOT
            # maneuver.
            cmd = self.make_pivot_cmd(
                self.juke_heading, self.final_pivot_radius,
                inset_angle=self.final_pivot_inset_angle,
                ang_vel_limit=self.final_pivot_angular_vel,
                ang_acc_limit=self.final_pivot_angular_acc,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            rs = self.robot_xy_yaw()
            if rs is not None and abs(ang_diff(
                    rs[2], self.juke_heading)) <= self.final_pivot_yaw_tol:
                self.transition(State.LINEAR_JUKE)
            return cmd

        if s == State.LINEAR_JUKE:
            # Sideways global-position move along +x while holding the inset
            # heading. Target the field +x edge (so the robot is still moving
            # when it switches) or a point exactly juke_distance away. Switch
            # to the final pivot once juke_distance has been travelled --
            # entering the pivot at non-zero velocity in the field-edge case.
            rs = self.robot_xy_yaw()
            if rs is None or self.juke_start_xy is None:
                return self.make_off_cmd()
            rx, ry, _ = rs
            sx, sy = self.juke_start_xy
            if self.juke_target_field_edge:
                target_x = self._field_edge_x()
            else:
                target_x = sx + self.juke_distance
            if (rx - sx) >= self.juke_distance:
                self._enter_final_pivot()
                self.transition(State.FINAL_PIVOT)
                return self.make_off_cmd()
            return self.make_pos_cmd(
                target_x, sy, self.juke_heading,
                vel_limit=self.juke_speed,
                acc_limit=self.juke_accel,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )

        if s == State.FINAL_PIVOT:
            # Built-in pivot maneuver (BCM_POINT_PIVOT): the firmware orbits
            # while rotating to face the goal point, accounting for the body
            # translation during the orbit. The robot is "aimed" once its
            # heading points at the goal from its current pose (recomputed each
            # tick, since the firmware -- not us -- decides the final pose).
            # Once aimed, latch the kick and keep streaming it (KR_KICK_NOW)
            # through final_pivot_dwell without reverting to KR_ARM.
            now = self.get_clock().now()
            rs = self.robot_xy_yaw()
            aimed = False
            if rs is not None:
                aim_heading = math.atan2(
                    self._goal_y() - rs[1], self.goal_x - rs[0])
                aimed = abs(ang_diff(rs[2], aim_heading)) \
                    <= self.final_pivot_yaw_tol
            if aimed and self.aim_since is None:
                self.aim_since = now
                self.kick_latched = True
            if self.kick_latched:
                cmd = self._final_pivot_cmd(
                    kick_request=RobotMotionCommand.KR_KICK_NOW,
                    kick_speed=self.kick_speed)
            else:
                cmd = self._final_pivot_cmd(
                    kick_request=RobotMotionCommand.KR_ARM)
            if self.aim_since is not None and \
                    (now - self.aim_since).nanoseconds * 1e-9 \
                    >= self.final_pivot_dwell:
                self.transition(State.KICK)
            return cmd

        if s == State.KICK:
            cmd = self._final_pivot_cmd(
                kick_request=RobotMotionCommand.KR_KICK_NOW,
                kick_speed=self.kick_speed)
            if self.time_in_state() >= self.kick_dwell:
                self.transition(State.DONE)
            return cmd

        if s == State.DONE:
            if self.loop and self.time_in_state() >= self.loop_dwell:
                self.ball_xy = None
                self.final_center = None
                self.final_pose = None
                self.juke_start_xy = None
                self.capture.reset()
                # Drive back to the start pose before the next trial.
                self.start_arrived_at = None
                self.transition(State.PLACE_ROBOT)
            return self.make_off_cmd()

        return self.make_off_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = JukeShotScenario()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
