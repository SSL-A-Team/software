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
from ateam_motion_scenarios.overlays import (
    make_array,
    make_point,
    make_pose_marker,
    make_text,
)
from ateam_msgs.msg import (
    FieldInfo,
    OverlayArray,
    RobotMotionCommand,
    Twist2D,
    VisionStateBall,
    VisionStateRobot,
)
from ateam_radio_msgs.msg import ConnectionStatus
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
    APPROACH_STAGING = auto()
    CAPTURE = auto()
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

        # Capture / staging params (mirrors ball_capture_scenario).
        self.stationary_ball_threshold = float(
            self._p('stationary_ball_threshold', 0.005))
        self.stationary_dwell = float(self._p('stationary_dwell', 0.5))
        self.approach_offset = float(self._p('approach_offset', 0.10))
        self.robot_front_offset = float(self._p('robot_front_offset', 0.09))
        self.capture_overdrive = float(self._p('capture_overdrive', 0.05))
        self.capture_vel_limit = float(self._p('capture_vel_limit', 0.15))
        self.capture_dwell = float(self._p('capture_dwell', 0.5))

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

        # Juke params. ``juke_type`` selects the quick-movement style:
        # none, linear, pivot, double_linear, double_pivot. ``none`` skips
        # the juke and goes straight to the ending pivot toward the goal.
        # The non-none types are reserved for upcoming increments. These
        # parameters are intentionally kept SEPARATE from the ending
        # (final) pivot below so the two can be tuned independently.
        self.juke_type = str(self._p('juke_type', 'none')).lower()

        # Linear-juke params (linear / double_linear).
        self.juke_speed = float(self._p('juke_speed', 1.0))
        self.juke_acc_linear = float(self._p('juke_acc_linear', 1.0))
        self.juke_distance = float(self._p('juke_distance', 0.30))
        self.juke_angle_deg = float(self._p('juke_angle_deg', 30.0))
        self.juke_second_angle_deg = float(
            self._p('juke_second_angle_deg', -30.0))
        self.juke_dwell = float(self._p('juke_dwell', 0.10))

        # Juke-pivot params (pivot juke).
        self.juke_pivot_radius = float(self._p('juke_pivot_radius', 0.10))
        self.juke_pivot_angle_deg = float(
            self._p('juke_pivot_angle_deg', 45.0))
        self.juke_pivot_angular_vel = float(
            self._p('juke_pivot_angular_vel', 2.0 * math.pi))
        self.juke_pivot_angular_acc = float(
            self._p('juke_pivot_angular_acc', 2.0 * math.pi))

        # Juke-double-pivot params (double_pivot juke).
        self.juke_double_pivot_radius = float(
            self._p('juke_double_pivot_radius', 0.10))
        self.juke_double_pivot_angle_deg = float(
            self._p('juke_double_pivot_angle_deg', 45.0))
        self.juke_double_pivot_second_angle_deg = float(
            self._p('juke_double_pivot_second_angle_deg', -45.0))
        self.juke_double_pivot_angular_vel = float(
            self._p('juke_double_pivot_angular_vel', 2.0 * math.pi))
        self.juke_double_pivot_angular_acc = float(
            self._p('juke_double_pivot_angular_acc', 2.0 * math.pi))

        if self.juke_type != 'none':
            self.get_logger().warn(
                f"juke_type '{self.juke_type}' is not implemented yet; "
                "treating it as 'none' (capture then pivot to goal)")

        # Ending (final) pivot toward the goal, executed with the firmware's
        # built-in BCM_PIVOT maneuver. The firmware derives the orbit center
        # from the robot's current pose, ``final_pivot_radius`` (orbit radius)
        # and ``final_pivot_inset_angle`` (heading offset from the radius
        # line; 0 = face the center); only the target heading is commanded.
        # The robot orbits until its heading reaches the goal heading
        # (direction toward the goal point on the +y goalline at x = 0).
        # Defaults mirror the firmware PivotParams::default(): orbit_radius
        # 0.2 m, inset_angle 0.5 rad, max angular vel pi rad/s, max angular
        # acc 2*pi rad/s^2. These params are independent of the juke pivot /
        # double pivot above. When ``goal_y_dynamic`` is true the +y goalline
        # is taken from the live field geometry (top edge); ``goal_y`` is only
        # the fallback.
        self.goal_x = float(self._p('goal_x', 0.0))
        self.goal_y_dynamic = bool(self._p('goal_y_dynamic', True))
        self.goal_y = float(self._p('goal_y', 4.5))
        self.final_pivot_radius = float(self._p('final_pivot_radius', 0.2))
        self.final_pivot_inset_angle = float(
            self._p('final_pivot_inset_angle', 0.5))
        self.final_pivot_angular_vel = float(
            self._p('final_pivot_angular_vel', 1.0 * math.pi))
        self.final_pivot_angular_acc = float(
            self._p('final_pivot_angular_acc', 2.0 * math.pi))
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
        self.capture_arrived_at = None
        self.ball_xy = None       # latched ball (x, y) at WAIT_FOR_STATIONARY
        self.approach_dir = None  # unit vec ball->robot (nearest-side pickup)
        self.approach_theta = 0.0  # robot yaw facing the ball during pickup
        self.aim_since = None     # time yaw first within tolerance in pivot
        self.start_arrived_at = None  # time robot first reached start pose

        # Final-pivot geometry, latched on entry to FINAL_PIVOT and sent to
        # the firmware as a BCM_PIVOT command.
        self.final_goal_heading = None  # target heading (faces center->goal)
        self.final_center = None        # pivot orbit center (x, y)
        self.final_radius = 0.0         # pivot orbit radius
        self.final_pose = None          # final robot (x, y, theta)

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

    # --------------------------------------------------------------- helpers
    def transition(self, new_state: State):
        self.get_logger().info(
            f'State {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_entered = self.get_clock().now()
        self.stationary_since = None
        self.capture_arrived_at = None
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
        cmd.limit_acc_linear = self.limit_acc_linear
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
        Build a firmware BCM_PIVOT command.

        The firmware derives the orbit center from the robot's current pose,
        ``orbit_radius`` and ``inset_angle``; only the target heading is
        commanded (via ``pose.theta``). The robot orbits at ``orbit_radius``
        holding ``inset_angle`` off the radius line (0 = face the center)
        until its heading reaches ``target_heading``.
        """
        cmd = RobotMotionCommand()
        cmd.body_control_mode = RobotMotionCommand.BCM_PIVOT
        cmd.pose = Twist2D(x=0.0, y=0.0, theta=float(target_heading))
        cmd.velocity = Twist2D()
        cmd.acceleration = Twist2D()
        cmd.limit_vel_angular = (
            float(ang_vel_limit) if ang_vel_limit > 0.0
            else self.limit_vel_angular)
        cmd.limit_acc_angular = (
            float(ang_acc_limit) if ang_acc_limit > 0.0
            else self.limit_acc_angular)
        cmd.pivot_orbit_radius = float(orbit_radius)
        cmd.pivot_inset_angle = float(inset_angle)
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
        self.approach_dir = None
        self.final_center = None
        self.final_pose = None
        self.start_arrived_at = None
        self.stationary_since = None
        self.capture_arrived_at = None
        self.aim_since = None

    # --------------------------------------------------------------- pickup
    def _compute_approach(self, robot_xy):
        """
        Latch the nearest-side pickup geometry for the current ball.

        ``approach_dir`` is the unit vector pointing from the ball toward
        the robot's position, so the robot stages on the side of the ball
        nearest to it. ``approach_theta`` faces the robot at the ball.
        """
        bx, by = self.ball_xy
        dx = robot_xy[0] - bx
        dy = robot_xy[1] - by
        d = math.hypot(dx, dy)
        if d < 1e-6:
            # Degenerate: robot on top of ball. Fall back to -x side.
            self.approach_dir = (-1.0, 0.0)
        else:
            self.approach_dir = (dx / d, dy / d)
        # Face the ball (opposite the approach direction).
        self.approach_theta = math.atan2(-self.approach_dir[1],
                                         -self.approach_dir[0])

    def _staging_target(self):
        if self.ball_xy is None or self.approach_dir is None:
            return None
        dist = self.approach_offset + self.robot_front_offset
        return (self.ball_xy[0] + self.approach_dir[0] * dist,
                self.ball_xy[1] + self.approach_dir[1] * dist)

    def _capture_target(self):
        if self.ball_xy is None or self.approach_dir is None:
            return None
        dist = self.robot_front_offset - self.capture_overdrive
        return (self.ball_xy[0] + self.approach_dir[0] * dist,
                self.ball_xy[1] + self.approach_dir[1] * dist)

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
        Latch the built-in (BCM_PIVOT) final-pivot target.

        Only the target heading is commanded; the firmware derives the orbit
        center from the robot's pose, ``final_pivot_radius`` and
        ``final_pivot_inset_angle``. Because the robot orbits to a new
        position during the pivot, the target heading is solved so the ball
        (held ``robot_front_offset`` ahead of the robot) is aimed at the goal
        point ``(goal_x, goal_y)`` at the *end* of the orbit -- not from the
        robot's pre-pivot position. The orbit center and final pose are
        reconstructed here only for visualization.
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
        self.get_logger().info(
            f'Final pivot (BCM_PIVOT): goal=({self.goal_x:.2f}, '
            f'{goal_y:.2f}), target_heading={math.degrees(theta_goal):.1f} '
            f'deg, turn={math.degrees(dtheta):.1f} deg, R={r:.2f}, '
            f'inset={self.final_pivot_inset_angle:.2f} rad')

    def _final_pivot_cmd(self, kick_request, kick_speed=0.0):
        """Build the BCM_PIVOT command for the latched final-pivot target."""
        return self.make_pivot_cmd(
            self.final_goal_heading, self.final_radius,
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
        elif s == State.APPROACH_STAGING:
            tgt = self._staging_target()
            if tgt is not None:
                items.append(make_pose_marker(
                    OVERLAY_NS, 'target',
                    (tgt[0], tgt[1], self.approach_theta),
                    self.pos_tol, color=target_color, heading_length=0.3))
        elif s == State.CAPTURE:
            tgt = self._capture_target()
            if tgt is not None:
                items.append(make_pose_marker(
                    OVERLAY_NS, 'target',
                    (tgt[0], tgt[1], self.approach_theta),
                    self.pos_tol, color='#00FF7FFF', heading_length=0.3))
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
                    rs = self.robot_xy_yaw()
                    robot_xy = (rs[0], rs[1]) if rs is not None \
                        else (bp.x - 1.0, bp.y)
                    self._compute_approach(robot_xy)
                    self.get_logger().info(
                        f'Ball stationary at ({bp.x:.3f}, {bp.y:.3f}); '
                        'picking up from nearest side '
                        f'(dir={self.approach_dir[0]:.2f}, '
                        f'{self.approach_dir[1]:.2f})')
                    self.transition(State.APPROACH_STAGING)
                    return self.make_off_cmd()
            else:
                self.stationary_since = None
            return self.make_off_cmd()

        if s == State.APPROACH_STAGING:
            tgt = self._staging_target()
            tx, ty = tgt
            if self.at_pose((tx, ty), self.pos_tol,
                            yaw_target=self.approach_theta,
                            yaw_tol=self.yaw_tol):
                self.transition(State.CAPTURE)
            return self.make_pos_cmd(tx, ty, self.approach_theta)

        if s == State.CAPTURE:
            tgt = self._capture_target()
            tx, ty = tgt
            cmd = self.make_pos_cmd(
                tx, ty, self.approach_theta,
                vel_limit=self.capture_vel_limit,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            if self.at_pose((tx, ty), self.pos_tol,
                            yaw_target=self.approach_theta,
                            yaw_tol=self.yaw_tol):
                now = self.get_clock().now()
                if self.capture_arrived_at is None:
                    self.capture_arrived_at = now
                elif (now - self.capture_arrived_at).nanoseconds * 1e-9 \
                        >= self.capture_dwell:
                    # (Juke would go here in a future increment.) Compute
                    # the final-pivot geometry from the current pose.
                    self._enter_final_pivot()
                    self.transition(State.FINAL_PIVOT)
            else:
                self.capture_arrived_at = None
            return cmd

        if s == State.FINAL_PIVOT:
            # Use the firmware's built-in pivot maneuver (BCM_PIVOT): the
            # robot orbits the latched center while facing it, rotating from
            # its current heading to the goal heading. We hold the kicker
            # armed and dribbler on, and watch the robot's yaw to detect
            # completion.
            cmd = self._final_pivot_cmd(
                kick_request=RobotMotionCommand.KR_ARM)
            now = self.get_clock().now()
            rs = self.robot_xy_yaw()
            aimed = (rs is not None and abs(ang_diff(
                rs[2], self.final_goal_heading)) <= self.final_pivot_yaw_tol)
            if aimed:
                if self.aim_since is None:
                    self.aim_since = now
                elif (now - self.aim_since).nanoseconds * 1e-9 \
                        >= self.final_pivot_dwell:
                    self.transition(State.KICK)
            else:
                self.aim_since = None
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
                self.approach_dir = None
                self.final_center = None
                self.final_pose = None
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
