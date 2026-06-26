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

from enum import auto, Enum
import math
import os
import sys

from ateam_motion_scenarios.overlays import (
    make_array,
    make_line,
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
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


ROBOT_ID = 2
TEAM_COLOR = 'blue'
OVERLAY_NS = 'catch_scenario'


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
    WAIT_FOR_BALL = auto()
    WAIT_FOR_INCOMING = auto()
    INTERCEPT = auto()
    CATCH_DWELL = auto()
    RESET = auto()
    DONE = auto()


class CatchScenario(Node):

    def __init__(self):
        super().__init__('catch_scenario')

        # Mode selection
        self.declare_parameter('mode', 'simple')

        # Geometry / robot
        self.declare_parameter('robot_radius', 0.09)
        self.declare_parameter('robot_front_offset', 0.09)
        # Safety margin added to robot_radius when checking field bounds.
        self.declare_parameter('field_safety_margin', 0.05)

        # Ball gating
        self.declare_parameter('min_ball_speed', 0.3)
        # Only act on balls moving toward -x (rolled in from +x edge).
        self.declare_parameter('require_negative_vx', True)

        # Trajectory limits (matching firmware defaults). These are the
        # caps used both by the in-process bang-bang simulation and by
        # the limit fields sent to the firmware so the two solvers
        # produce the same trajectory.
        self.declare_parameter('max_vel_linear', 3.0)
        self.declare_parameter('max_vel_angular', 3.0 * math.pi)
        self.declare_parameter('max_accel_linear', 2.0)
        self.declare_parameter('max_accel_angular', 2.0 * math.pi)

        # Interception search
        self.declare_parameter('search_d_min', 0.05)
        self.declare_parameter('search_d_max', 8.0)
        self.declare_parameter('search_tolerance', 0.01)
        self.declare_parameter('search_max_iter', 40)

        # Catch hold and reset
        self.declare_parameter('dribbler_speed', 300.0)
        self.declare_parameter('pos_tol', 0.04)
        self.declare_parameter('yaw_tol', 0.08)
        self.declare_parameter('catch_dwell', 1.5)
        # If the ball isn't visible / valid for this long while in
        # INTERCEPT, give up and reset to the back edge.
        self.declare_parameter('ball_lost_timeout', 0.75)
        self.declare_parameter('reset_margin', 0.3)
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('loop', True)

        # ------------------------------------------------------------------
        self.mode = str(self.get_parameter('mode').value).lower()
        if self.mode not in ('simple', 'medium', 'complex'):
            self.get_logger().warn(
                f"Unknown mode '{self.mode}', defaulting to 'simple'")
            self.mode = 'simple'

        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.robot_front_offset = float(self.get_parameter(
            'robot_front_offset').value)
        self.field_safety_margin = float(self.get_parameter(
            'field_safety_margin').value)

        self.min_ball_speed = float(self.get_parameter('min_ball_speed').value)
        self.require_negative_vx = bool(self.get_parameter(
            'require_negative_vx').value)

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
        self.ball_lost_timeout = float(self.get_parameter(
            'ball_lost_timeout').value)
        self.reset_margin = float(self.get_parameter('reset_margin').value)
        self.loop = bool(self.get_parameter('loop').value)
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

        self.ball_sub = self.create_subscription(
            VisionStateBall, '/ball', self.ball_cb, sensor_qos)
        self.robot_sub = self.create_subscription(
            VisionStateRobot, f'/{TEAM_COLOR}_team/robot{ROBOT_ID}',
            self.robot_cb, sensor_qos)
        self.field_sub = self.create_subscription(
            FieldInfo, '/field', self.field_cb, 10)

        self.cmd_pub = self.create_publisher(
            RobotMotionCommand, f'/robot_motion_commands/robot{ROBOT_ID}', 10)
        self.overlay_pub = self.create_publisher(
            OverlayArray, '/overlays', 10)

        self.state = State.RESET
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

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'catch_scenario started for robot {ROBOT_ID} ({TEAM_COLOR}), '
            f'mode={self.mode}. State: RESET (driving to back edge)')

    # ------------------------------------------------------------------ subs
    def ball_cb(self, msg: VisionStateBall):
        self.ball = msg

    def robot_cb(self, msg: VisionStateRobot):
        self.robot = msg

    def field_cb(self, msg: FieldInfo):
        self.field = msg

    # --------------------------------------------------------------- helpers
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

    def make_pos_cmd(self, x: float, y: float, theta: float = 0.0,
                     vel_limit: float = 0.0,
                     ang_vel_limit: float = 0.0,
                     accel_limit_linear: float = 0.0,
                     accel_limit_angular: float = 0.0,
                     kick_request: int = RobotMotionCommand.KR_DISABLE,
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
        cmd.kick_speed = 0.0
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
    def ball_state(self):
        """Return ((bx, by), (vx, vy), speed) or None."""
        if self.ball is None or not self.ball.visible:
            return None
        bp = self.ball.pose.position
        bv = self.ball.twist.linear
        speed = math.hypot(bv.x, bv.y)
        if speed < self.min_ball_speed:
            return None
        if self.require_negative_vx and bv.x >= 0.0:
            return None
        return ((bp.x, bp.y), (bv.x, bv.y), speed)

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
        cmd = self.step_state_machine()
        if cmd is not None:
            self.cmd_pub.publish(cmd)
        self.publish_overlays()

    def step_state_machine(self):
        s = self.state

        if s == State.WAIT_FOR_BALL:
            if self.ball is not None and self.ball.visible:
                self.transition(State.WAIT_FOR_INCOMING)
            return self.make_off_cmd()

        if s == State.WAIT_FOR_INCOMING:
            ball = self.ball_state()
            if ball is None:
                return self.make_off_cmd()
            rs = self.robot_xy_yaw()
            if rs is None:
                return self.make_off_cmd()
            # Latch initial rest position.
            self.S = (rs[0], rs[1])
            self.transition(State.INTERCEPT)
            return self.make_off_cmd()

        if s == State.INTERCEPT:
            ball = self.ball_state()
            rs = self.robot_xy_yaw()
            if rs is None:
                return self.make_off_cmd()
            now = self.get_clock().now()
            if ball is not None:
                self.last_ball_seen = now
                sol = None
                if self.mode == 'simple':
                    res = self.solve_simple(ball, self.S)
                    if res is not None:
                        P_drib, P_center, theta, t_int = res
                        sol = (P_drib, P_center, theta,
                               (P_center[0], P_center[1], theta), t_int)
                else:
                    sol = self.solve_search(ball, self.S, self.mode)
                if sol is not None:
                    P_drib, P_center, theta, target, t_int = sol
                    self.intercept_P = (P_drib[0], P_drib[1], theta)
                    self.intercept_target = target
                    self.last_solution_d = math.hypot(
                        P_drib[0] - ball[0][0], P_drib[1] - ball[0][1])
                    self.last_solution_T = t_int
            else:
                # Ball lost (off field, occluded, or below min_ball_speed).
                # If it stays gone for ball_lost_timeout, reset to back edge.
                if self.last_ball_seen is None:
                    self.last_ball_seen = now
                elapsed = (now - self.last_ball_seen).nanoseconds * 1e-9
                if elapsed >= self.ball_lost_timeout:
                    self.get_logger().info(
                        f'Ball lost for {elapsed:.2f}s, resetting.')
                    self.transition(State.RESET)
                    return self.make_off_cmd()
            if self.intercept_target is None:
                return self.make_off_cmd()
            tx, ty, ttheta = self.intercept_target
            cmd = self.make_pos_cmd(
                tx, ty, ttheta,
                accel_limit_linear=self.max_accel_linear,
                accel_limit_angular=self.max_accel_angular,
                kick_request=RobotMotionCommand.KR_DISABLE,
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
                    self.transition(State.CATCH_DWELL)
            return cmd

        if s == State.CATCH_DWELL:
            if self.intercept_P is None:
                self.transition(State.RESET)
                return self.make_off_cmd()
            theta = self.intercept_P[2]
            px = self.intercept_P[0] - self.robot_front_offset * math.cos(
                theta)
            py = self.intercept_P[1] - self.robot_front_offset * math.sin(
                theta)
            cmd = self.make_pos_cmd(
                px, py, theta,
                kick_request=RobotMotionCommand.KR_DISABLE,
                dribbler_speed=self.dribbler_speed,
            )
            if self.time_in_state() >= self.catch_dwell:
                # If non-looping, this is the last cycle: go to DONE
                # after the post-catch reset. Looping always re-arms.
                self.post_reset_state = (
                    State.WAIT_FOR_BALL if self.loop else State.DONE)
                self.transition(State.RESET)
            return cmd

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
                # Clear latched state before re-arming for the next ball.
                self.S = None
                self.intercept_P = None
                self.intercept_target = None
                self.last_solution_d = None
                self.last_solution_T = None
                self.transition(self.post_reset_state)
            return self.make_pos_cmd(tx, ty, 0.0)

        if s == State.DONE:
            return self.make_off_cmd()

        return self.make_off_cmd()

    # ------------------------------------------------------------- overlays
    def publish_overlays(self):
        items = []

        ball = self.ball_state()
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
            items.append(make_point(
                OVERLAY_NS, 'intercept_dribbler',
                (self.intercept_P[0], self.intercept_P[1]),
                color='#00FF7FFF', radius=0.04))
            theta = self.intercept_P[2]
            cx = self.intercept_P[0] - self.robot_front_offset * math.cos(
                theta)
            cy = self.intercept_P[1] - self.robot_front_offset * math.sin(
                theta)
            items.append(make_pose_marker(
                OVERLAY_NS, 'intercept_center', (cx, cy, theta),
                self.pos_tol, color='#00FF7FFF',
                heading_length=0.25))

        if (self.mode == 'complex' and self.intercept_target is not None
                and self.intercept_P is not None):
            tx, ty, ttheta = self.intercept_target
            items.append(make_pose_marker(
                OVERLAY_NS, 'firmware_target', (tx, ty, ttheta),
                self.pos_tol, color='#00BFFFFF',
                heading_length=0.3))
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

        rs = self.robot_xy_yaw()
        if rs is not None:
            label = f'{self.state.name}  mode={self.mode}'
            if self.last_solution_T is not None:
                label += f'  T={self.last_solution_T:.2f}s'
            items.append(make_text(
                OVERLAY_NS, 'state', label,
                (rs[0], rs[1] + 0.25),
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
