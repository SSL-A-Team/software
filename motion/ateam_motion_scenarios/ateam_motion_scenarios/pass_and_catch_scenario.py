"""
Pass-and-catch scenario node for profiling robot motion performance.

Hardcoded to robot 2 on the blue team. Drives the robot through a scripted
sequence: capture a stationary ball, aim at a configurable point on a
field-edge "wall", kick the ball, then drive to intercept the ricochet
along the geometrically reflected trajectory.

Reflection geometry assumes a perfectly elastic bounce off the chosen wall
(no friction, no spin). The wall is one of the four field edges:
  - 'pos_y' / 'neg_y' : touchlines (constant y), reflect y component
  - 'pos_x' / 'neg_x' : goallines  (constant x), reflect x component

The hit point on the wall is parameterized by `wall_hit_offset` (signed,
meters along the wall measured from the field center along the parallel
axis). `wall_inset` shifts the reflection point inward from the outer
edge if desired (e.g. to account for ball radius).

The intercept pose places the dribbler at a point `intercept_distance`
meters along the reflected ray from the bounce point, with the robot
facing the bounce point so the ball runs into the dribbler.
"""

from enum import auto, Enum
import math

from ateam_msgs.msg import (
    FieldInfo,
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


class State(Enum):
    WAIT_FOR_BALL = auto()
    WAIT_FOR_STATIONARY = auto()
    APPROACH_STAGING = auto()
    CAPTURE = auto()
    AIM_AT_WALL = auto()
    KICK = auto()
    DRIVE_TO_INTERCEPT = auto()
    CATCH_DWELL = auto()
    RESET = auto()
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


class PassAndCatchScenario(Node):

    def __init__(self):
        super().__init__('pass_and_catch_scenario')

        # Ball / capture
        self.declare_parameter('stationary_ball_threshold', 0.005)
        self.declare_parameter('stationary_dwell', 0.5)
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('robot_front_offset', 0.09)
        self.declare_parameter('capture_overdrive', 0.05)
        self.declare_parameter('capture_vel_limit', 0.15)
        self.declare_parameter('capture_dwell', 0.5)

        # Wall / pass geometry
        # Which field edge to bounce off. One of:
        #   'pos_y' (default), 'neg_y', 'pos_x', 'neg_x'
        self.declare_parameter('wall_axis', 'pos_y')
        # Signed distance along the wall from field center, in meters.
        # For pos_y/neg_y walls this is the x coordinate of the hit point.
        # For pos_x/neg_x walls this is the y coordinate of the hit point.
        self.declare_parameter('wall_hit_offset', 0.0)
        # Inward offset from the outer field edge to the reflection point.
        self.declare_parameter('wall_inset', 0.0)

        # Aim / kick
        self.declare_parameter('aim_dwell', 0.3)
        self.declare_parameter('aim_max_angular_vel', 0.5)
        self.declare_parameter('pre_kick_dwell', 0.0)
        self.declare_parameter('kick_speed', 3.0)
        self.declare_parameter('dribbler_speed', 250.0)

        # Catch / intercept
        # Distance along the reflected ray from the bounce point at which
        # to place the dribbler. Smaller values catch closer to the wall.
        self.declare_parameter('intercept_distance', 1.0)
        # Time to dwell at the intercept pose with dribbler on, capturing
        # the ricochet. Set high enough that the ball arrives.
        self.declare_parameter('catch_dwell', 3.0)
        # Optional cap on linear velocity while driving to intercept.
        # 0.0 leaves the firmware default.
        self.declare_parameter('intercept_vel_limit', 0.0)
        # Live trajectory tracking. When True, DRIVE_TO_INTERCEPT
        # recomputes the intercept point every tick from the ball's
        # current position/velocity (projecting through one wall
        # reflection if needed) and re-targets the robot.
        self.declare_parameter('tracking_enabled', True)
        # Below this speed (m/s) the ball motion is too noisy to
        # extrapolate; fall back to the latched geometric prediction.
        self.declare_parameter('min_ball_speed_for_tracking', 0.3)

        # Tolerances / motion limits
        self.declare_parameter('pos_tol', 0.03)
        self.declare_parameter('yaw_tol', 0.08)
        self.declare_parameter('limit_acc_linear', 1.5)
        self.declare_parameter('limit_acc_angular', 15.0)

        # Control flow
        self.declare_parameter('reset_margin', 0.3)
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('loop_dwell', 0.0)

        # ------------------------------------------------------------------
        self.stationary_ball_threshold = float(self.get_parameter(
            'stationary_ball_threshold').value)
        self.stationary_dwell = float(
            self.get_parameter('stationary_dwell').value)
        self.approach_offset = float(
            self.get_parameter('approach_offset').value)
        self.robot_front_offset = float(self.get_parameter(
            'robot_front_offset').value)
        self.capture_overdrive = float(
            self.get_parameter('capture_overdrive').value)
        self.capture_vel_limit = float(
            self.get_parameter('capture_vel_limit').value)
        self.capture_dwell = float(self.get_parameter('capture_dwell').value)

        self.wall_axis = str(self.get_parameter('wall_axis').value).lower()
        if self.wall_axis not in ('pos_y', 'neg_y', 'pos_x', 'neg_x'):
            self.get_logger().warn(
                f"Unknown wall_axis '{self.wall_axis}', defaulting to pos_y")
            self.wall_axis = 'pos_y'
        self.wall_hit_offset = float(
            self.get_parameter('wall_hit_offset').value)
        self.wall_inset = float(self.get_parameter('wall_inset').value)

        self.aim_dwell = float(self.get_parameter('aim_dwell').value)
        self.aim_max_angular_vel = float(
            self.get_parameter('aim_max_angular_vel').value)
        self.pre_kick_dwell = float(
            self.get_parameter('pre_kick_dwell').value)
        self.kick_speed = float(self.get_parameter('kick_speed').value)
        self.dribbler_speed = float(
            self.get_parameter('dribbler_speed').value)

        self.intercept_distance = float(
            self.get_parameter('intercept_distance').value)
        self.catch_dwell = float(self.get_parameter('catch_dwell').value)
        self.intercept_vel_limit = float(
            self.get_parameter('intercept_vel_limit').value)
        self.tracking_enabled = bool(
            self.get_parameter('tracking_enabled').value)
        self.min_ball_speed_for_tracking = float(self.get_parameter(
            'min_ball_speed_for_tracking').value)

        self.pos_tol = float(self.get_parameter('pos_tol').value)
        self.yaw_tol = float(self.get_parameter('yaw_tol').value)
        self.limit_acc_linear = float(
            self.get_parameter('limit_acc_linear').value)
        self.limit_acc_angular = float(
            self.get_parameter('limit_acc_angular').value)

        self.reset_margin = float(self.get_parameter('reset_margin').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.loop_dwell = float(self.get_parameter('loop_dwell').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

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

        self.state = State.WAIT_FOR_BALL
        self.state_entered = self.get_clock().now()
        self.stationary_since = None
        self.capture_arrived_at = None
        self.aim_arrived_at = None
        self.ball_xy = None        # latched ball (x, y) at capture time
        self.aim_hold = None       # latched (x, y, theta) for KICK
        self.intercept_pose = None  # latched (x, y, theta) for catch

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'pass_and_catch_scenario started for robot {ROBOT_ID} '
            f'({TEAM_COLOR}). State: WAIT_FOR_BALL')

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
        self.stationary_since = None
        self.capture_arrived_at = None
        self.aim_arrived_at = None

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
                     kick_request: int = RobotMotionCommand.KR_DISABLE,
                     dribbler_speed: float = 0.0,
                     kick_speed: float = 0.0) -> RobotMotionCommand:
        cmd = RobotMotionCommand()
        cmd.body_control_mode = RobotMotionCommand.BCM_GLOBAL_POSITION
        cmd.pose = Twist2D(x=float(x), y=float(y), theta=float(theta))
        cmd.velocity = Twist2D()
        cmd.acceleration = Twist2D()
        cmd.limit_vel_linear = float(vel_limit)
        cmd.limit_vel_angular = float(ang_vel_limit)
        cmd.limit_acc_linear = self.limit_acc_linear
        cmd.limit_acc_angular = self.limit_acc_angular
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

    # ------------------------------------------------------ pass-and-catch
    def wall_hit_point(self):
        """Return (wx, wy) of the configured reflection point on the wall.

        Returns None if the field is not yet known.
        """
        if self.field is None or self.field.field_length <= 0.0 \
                or self.field.field_width <= 0.0:
            return None
        half_l = self.field.field_length / 2.0
        half_w = self.field.field_width / 2.0
        if self.wall_axis == 'pos_y':
            return (self.wall_hit_offset, half_w - self.wall_inset)
        if self.wall_axis == 'neg_y':
            return (self.wall_hit_offset, -half_w + self.wall_inset)
        if self.wall_axis == 'pos_x':
            return (half_l - self.wall_inset, self.wall_hit_offset)
        # neg_x
        return (-half_l + self.wall_inset, self.wall_hit_offset)

    def reflect_dir(self, ux: float, uy: float):
        """Reflect an incoming direction off the configured wall."""
        if self.wall_axis in ('pos_y', 'neg_y'):
            return (ux, -uy)
        return (-ux, uy)

    def compute_intercept_pose(self):
        """Return (x, y, theta) intercept pose, or None if not computable.

        Places the robot's dribbler tip at a point `intercept_distance`
        meters along the reflected ray from the bounce point. Robot
        faces back toward the bounce point so the ball runs into the
        dribbler.
        """
        if self.ball_xy is None:
            return None
        wp = self.wall_hit_point()
        if wp is None:
            return None
        wx, wy = wp
        bx, by = self.ball_xy
        # Incoming direction (ball -> wall)
        ix = wx - bx
        iy = wy - by
        n = math.hypot(ix, iy)
        if n < 1e-6:
            return None
        ix /= n
        iy /= n
        rx_dir, ry_dir = self.reflect_dir(ix, iy)
        # Dribbler target along reflected ray
        dx = wx + rx_dir * self.intercept_distance
        dy = wy + ry_dir * self.intercept_distance
        # Robot faces back along reflected ray (toward bounce point)
        theta = math.atan2(-ry_dir, -rx_dir)
        # Pull robot center back from dribbler tip by robot_front_offset
        cx = dx - self.robot_front_offset * math.cos(theta)
        cy = dy - self.robot_front_offset * math.sin(theta)
        return (cx, cy, theta)

    def aim_theta(self):
        """Heading from the latched ball position toward the wall hit."""
        wp = self.wall_hit_point()
        if wp is None or self.ball_xy is None:
            return None
        wx, wy = wp
        bx, by = self.ball_xy
        return math.atan2(wy - by, wx - bx)

    def _wall_normal(self):
        """Outward unit normal toward the configured wall."""
        if self.wall_axis == 'pos_y':
            return (0.0, 1.0)
        if self.wall_axis == 'neg_y':
            return (0.0, -1.0)
        if self.wall_axis == 'pos_x':
            return (1.0, 0.0)
        return (-1.0, 0.0)  # neg_x

    def _wall_signed_distance(self):
        """Signed scalar W such that wall is the line ((p . n) == W)."""
        if self.field is None:
            return None
        if self.wall_axis in ('pos_y', 'neg_y'):
            half = self.field.field_width / 2.0
        else:
            half = self.field.field_length / 2.0
        if half <= 0.0:
            return None
        return half - self.wall_inset

    def predict_intercept_pose_live(self):
        """Recompute the intercept pose from the live ball state.

        Projects the ball's current trajectory forward through (at most)
        one elastic reflection off the configured wall and returns the
        pose at which the dribbler should sit on the intercept line.

        Returns None if the ball is not visible, the trajectory cannot
        be extrapolated reliably, or the ball has already passed the
        intercept line on the way back from the wall.
        """
        if self.ball is None or not self.ball.visible:
            return None
        W = self._wall_signed_distance()
        if W is None:
            return None
        nx, ny = self._wall_normal()

        bp = self.ball.pose.position
        bv = self.ball.twist.linear
        bx, by = bp.x, bp.y
        vx, vy = bv.x, bv.y
        speed = math.hypot(vx, vy)
        if speed < self.min_ball_speed_for_tracking:
            return None

        # Decompose along/perpendicular to wall normal.
        # tangent = perpendicular to n (rotate +90deg)
        tx, ty = -ny, nx
        bn = bx * nx + by * ny
        bt = bx * tx + by * ty
        vn = vx * nx + vy * ny
        vt = vx * tx + vy * ty

        intercept_n = W - self.intercept_distance

        if vn > 0.0:
            # Pre-bounce: extrapolate to wall, reflect, then to intercept
            if bn >= W:
                # Already past the wall (vision glitch); bail.
                return None
            t_to_wall = (W - bn) / vn
            t_to_intercept = self.intercept_distance / vn
            n_at = intercept_n
            t_at = bt + vt * (t_to_wall + t_to_intercept)
            # Velocity at intercept (post reflection).
            vn_at = -vn
            vt_at = vt
        else:
            # Already moving away from wall (post bounce or never headed
            # there). Catch ball on its current straight trajectory.
            if bn <= intercept_n:
                # Ball has already passed the intercept line.
                return None
            if vn >= 0.0:
                # vn == 0: ball drifting parallel to wall; never crosses.
                return None
            t_at_param = (intercept_n - bn) / vn  # vn < 0 -> positive
            n_at = intercept_n
            t_at = bt + vt * t_at_param
            vn_at = vn
            vt_at = vt

        # Convert (n, t) back to (x, y).
        dx = n_at * nx + t_at * tx
        dy = n_at * ny + t_at * ty
        # Velocity at intercept in world frame.
        vx_at = vn_at * nx + vt_at * tx
        vy_at = vn_at * ny + vt_at * ty
        # Robot faces opposite the ball's incoming velocity.
        theta = math.atan2(-vy_at, -vx_at)
        # Pull robot center back from dribbler tip by robot_front_offset.
        cx = dx - self.robot_front_offset * math.cos(theta)
        cy = dy - self.robot_front_offset * math.sin(theta)
        return (cx, cy, theta)

    # ------------------------------------------------------------------ tick
    def tick(self):
        cmd = self.step_state_machine()
        if cmd is not None:
            self.cmd_pub.publish(cmd)

    def step_state_machine(self):
        s = self.state

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
                    self.get_logger().info(
                        f'Ball stationary at ({bp.x:.3f}, {bp.y:.3f})')
                    self.transition(State.APPROACH_STAGING)
                    return self.make_off_cmd()
            else:
                self.stationary_since = None
            return self.make_off_cmd()

        if s == State.APPROACH_STAGING:
            # Stage behind the ball along the (ball -> wall) line so
            # capture also leaves us roughly pre-aimed.
            theta = self.aim_theta()
            if theta is None:
                self.get_logger().warn(
                    'APPROACH_STAGING: waiting for /field...',
                    throttle_duration_sec=2.0)
                return self.make_off_cmd()
            back = self.approach_offset + self.robot_front_offset
            tx = self.ball_xy[0] - back * math.cos(theta)
            ty = self.ball_xy[1] - back * math.sin(theta)
            if self.at_pose((tx, ty), self.pos_tol, theta, self.yaw_tol):
                self.transition(State.CAPTURE)
            return self.make_pos_cmd(tx, ty, theta)

        if s == State.CAPTURE:
            theta = self.aim_theta()
            if theta is None:
                return self.make_off_cmd()
            # Drive forward into the ball along the aim line.
            forward = -self.robot_front_offset + self.capture_overdrive
            tx = self.ball_xy[0] + forward * math.cos(theta)
            ty = self.ball_xy[1] + forward * math.sin(theta)
            cmd = self.make_pos_cmd(
                tx, ty, theta,
                vel_limit=self.capture_vel_limit,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            if self.at_pose((tx, ty), self.pos_tol, theta, self.yaw_tol):
                now = self.get_clock().now()
                if self.capture_arrived_at is None:
                    self.capture_arrived_at = now
                elif (now - self.capture_arrived_at).nanoseconds * 1e-9 \
                        >= self.capture_dwell:
                    self.transition(State.AIM_AT_WALL)
            else:
                self.capture_arrived_at = None
            return cmd

        if s == State.AIM_AT_WALL:
            rs = self.robot_xy_yaw()
            theta = self.aim_theta()
            if rs is None or theta is None:
                return self.make_off_cmd()
            rx, ry, _ = rs
            cmd = self.make_pos_cmd(
                rx, ry, theta,
                ang_vel_limit=self.aim_max_angular_vel,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            yaw_ok = abs(ang_diff(rs[2], theta)) <= self.yaw_tol
            if yaw_ok:
                now = self.get_clock().now()
                if self.aim_arrived_at is None:
                    self.aim_arrived_at = now
                elif (now - self.aim_arrived_at).nanoseconds * 1e-9 \
                        >= self.aim_dwell:
                    self.aim_hold = (rx, ry, theta)
                    self.transition(State.KICK)
            else:
                self.aim_arrived_at = None
            return cmd

        if s == State.KICK:
            hx, hy, htheta = self.aim_hold
            cmd = self.make_pos_cmd(
                hx, hy, htheta,
                ang_vel_limit=self.aim_max_angular_vel,
                kick_request=RobotMotionCommand.KR_KICK_NOW,
                kick_speed=self.kick_speed,
                dribbler_speed=self.dribbler_speed,
            )
            # Latch the intercept pose using the ball position at the
            # moment of the kick (it has not moved yet from ball_xy).
            if self.intercept_pose is None:
                self.intercept_pose = self.compute_intercept_pose()
                if self.intercept_pose is not None:
                    cx, cy, ct = self.intercept_pose
                    self.get_logger().info(
                        f'Intercept pose: ({cx:.3f}, {cy:.3f}, '
                        f'{math.degrees(ct):.1f} deg)')
            if self.time_in_state() >= max(self.pre_kick_dwell, 0.2):
                if self.intercept_pose is None:
                    # Fall back if geometry could not be computed.
                    self.transition(State.RESET)
                else:
                    self.transition(State.DRIVE_TO_INTERCEPT)
            return cmd

        if s == State.DRIVE_TO_INTERCEPT:
            # Live-track: each tick, recompute the intercept from the
            # ball's current position and velocity. Fall back to the
            # geometric estimate latched at kick time if tracking is
            # disabled or the ball motion is too noisy / unavailable.
            if self.tracking_enabled:
                live = self.predict_intercept_pose_live()
                if live is not None:
                    self.intercept_pose = live
            cx, cy, ct = self.intercept_pose
            cmd = self.make_pos_cmd(
                cx, cy, ct,
                vel_limit=self.intercept_vel_limit,
                kick_request=RobotMotionCommand.KR_DISABLE,
                dribbler_speed=self.dribbler_speed,
            )
            if self.at_pose((cx, cy), self.pos_tol, ct, self.yaw_tol):
                self.transition(State.CATCH_DWELL)
            return cmd

        if s == State.CATCH_DWELL:
            cx, cy, ct = self.intercept_pose
            cmd = self.make_pos_cmd(
                cx, cy, ct,
                kick_request=RobotMotionCommand.KR_DISABLE,
                dribbler_speed=self.dribbler_speed,
            )
            if self.time_in_state() >= self.catch_dwell:
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
            if self.at_pose((tx, ty), self.pos_tol):
                self.transition(State.DONE)
            return self.make_pos_cmd(tx, ty)

        if s == State.DONE:
            if self.loop and self.time_in_state() >= self.loop_dwell:
                self.ball_xy = None
                self.aim_hold = None
                self.intercept_pose = None
                self.transition(State.WAIT_FOR_BALL)
            return self.make_off_cmd()

        return self.make_off_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = PassAndCatchScenario()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
