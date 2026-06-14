"""
Ball intercept scenario node.

Hardcoded to robot 2 on the blue team. Holds the robot at the origin while
waiting for a human to roll the ball in from the +x edge of the field. Once
the ball is moving inward (negative x-velocity), continuously sweeps
candidate intercept times forward along the ball's predicted (linear) path,
and accepts the earliest candidate where the robot can both translate to
the intercept point and rotate to face it within the candidate ball time
under closed-form accel-then-(optional)-coast bang-bang motion (no
terminal deceleration before the catch).

After acceptance, the scenario builds the *full* trajectory that ends at
rest by extending each axis past the intercept with a symmetric
deceleration whose magnitude matches that axis's accel from the first
half. The resulting at-rest pose is the BCM_GLOBAL_POSITION setpoint sent
to the robot — the firmware's own controller plans through it, so the
robot naturally passes through the intercept point while still moving
(catching the ball), then decelerates to rest. If the at-rest pose lies
outside the field, the trial is logged and skipped rather than clamped
onto the boundary. The dribbler roller is enabled while intercepting.
Path overlays are sampled from the same synthetic profile and published
to `/overlays`.

Run:

    ros2 run ateam_motion_scenarios ball_intercept_scenario

Useful parameters (all overridable via `--ros-args -p name:=value`):

* publish_rate_hz        — control loop / planner rate (default 60)
* max_vel_linear         — trajectory linear velocity limit, m/s (default 2.0)
* max_vel_angular        — trajectory angular velocity limit, rad/s (default 6.0)
* max_accel_linear       — trajectory linear accel limit, m/s² (default 2.0)
* max_accel_angular      — trajectory angular accel limit, rad/s² (default 12.0)
* search_t_max           — max intercept time to search, seconds (default 4.0)
* search_dt              — intercept time step, seconds (default 0.05)
* feasible_slack         — slack added to t when checking accel-only arrival (default 0.05 s)
* field_margin           — keep targets at least this far from field edges, m (default 0.1)
* incoming_vx_threshold  — ball x-velocity must be below this to start (default -0.05 m/s)
* dribbler_speed         — dribbler RPM during intercept (default 250)
* loop                   — return to WAIT_FOR_BALL after DONE (default True)
"""

from enum import auto, Enum
import math

from ateam_msgs.msg import (
    FieldInfo,
    Overlay,
    OverlayArray,
    RobotMotionCommand,
    Twist2D,
    VisionStateBall,
    VisionStateRobot,
)
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


ROBOT_ID = 2
TEAM_COLOR = 'blue'

# Overlay type / command codes (mirrors ateam_msgs/Overlay constants).
OVERLAY_POINT = 0
OVERLAY_LINE = 1
OVERLAY_CMD_REPLACE = 0
OVERLAY_CMD_REMOVE = 2

OVERLAY_TRAJECTORY_NAME = 'intercept_trajectory'
OVERLAY_INTERCEPT_NAME = 'intercept_point'
OVERLAY_FINAL_NAME = 'final_rest_pose'


def yaw_from_quat(q) -> float:
    """Extract yaw from a planar quaternion (assumes roll=pitch=0)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class State(Enum):
    WAIT_FOR_BALL = auto()
    INTERCEPT = auto()
    DONE = auto()


class BallInterceptScenario(Node):

    def __init__(self):
        super().__init__('ball_intercept_scenario')

        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('max_vel_linear', 2.0)
        self.declare_parameter('max_vel_angular', 6.0)
        self.declare_parameter('max_accel_linear', 2.0)
        self.declare_parameter('max_accel_angular', 12.0)
        self.declare_parameter('search_t_max', 4.0)
        self.declare_parameter('search_dt', 0.05)
        self.declare_parameter('feasible_slack', 0.05)
        self.declare_parameter('field_margin', 0.1)
        self.declare_parameter('incoming_vx_threshold', -0.05)
        self.declare_parameter('dribbler_speed', 250.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('trajectory_sample_points', 24)
        self.declare_parameter('done_dwell', 0.5)
        self.declare_parameter('home_pose_x', 0.0)
        self.declare_parameter('home_pose_y', 0.0)
        self.declare_parameter('home_pose_theta', 0.0)

        self.max_vel_linear = float(self.get_parameter('max_vel_linear').value)
        self.max_vel_angular = float(
            self.get_parameter('max_vel_angular').value)
        self.max_accel_linear = float(
            self.get_parameter('max_accel_linear').value)
        self.max_accel_angular = float(
            self.get_parameter('max_accel_angular').value)
        self.search_t_max = float(self.get_parameter('search_t_max').value)
        self.search_dt = float(self.get_parameter('search_dt').value)
        self.feasible_slack = float(
            self.get_parameter('feasible_slack').value)
        self.field_margin = float(self.get_parameter('field_margin').value)
        self.incoming_vx_threshold = float(
            self.get_parameter('incoming_vx_threshold').value)
        self.dribbler_speed = float(self.get_parameter('dribbler_speed').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.trajectory_sample_points = max(
            2, int(self.get_parameter('trajectory_sample_points').value))
        self.done_dwell = float(self.get_parameter('done_dwell').value)
        self.home_xy_theta = (
            float(self.get_parameter('home_pose_x').value),
            float(self.get_parameter('home_pose_y').value),
            float(self.get_parameter('home_pose_theta').value),
        )
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
        self.overlay_pub = self.create_publisher(
            OverlayArray, '/overlays', 10)

        self.state = State.WAIT_FOR_BALL
        self.state_entered = self.get_clock().now()
        self.overlays_published = False

        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'ball_intercept_scenario started for robot {ROBOT_ID} '
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

    def time_in_state(self) -> float:
        return (self.get_clock().now() - self.state_entered).nanoseconds * 1e-9

    def robot_state(self):
        """Return [x, y, theta, vx, vy, vtheta] in global frame, or None."""
        if self.robot is None or not self.robot.visible:
            return None
        p = self.robot.pose.position
        theta = yaw_from_quat(self.robot.pose.orientation)
        # VisionStateRobot.twist is in the global frame for our pipeline.
        v = self.robot.twist.linear
        w = self.robot.twist.angular
        return [p.x, p.y, theta, v.x, v.y, w.z]

    def ball_state(self):
        if self.ball is None or not self.ball.visible:
            return None
        bp = self.ball.pose.position
        bv = self.ball.twist.linear
        return (bp.x, bp.y, bv.x, bv.y)

    def field_bounds(self):
        """Return (xmin, ymin, xmax, ymax) usable play area, or None."""
        if self.field is None or self.field.field_length <= 0.0 \
                or self.field.field_width <= 0.0:
            return None
        m = self.field_margin
        hx = self.field.field_length / 2.0 - m
        hy = self.field.field_width / 2.0 - m
        return (-hx, -hy, hx, hy)

    @staticmethod
    def _point_in_bounds(x, y, bounds) -> bool:
        xmin, ymin, xmax, ymax = bounds
        return xmin <= x <= xmax and ymin <= y <= ymax

    # ----------------------------------------------------------- intercept
    def _t_lin_to_distance(self, d: float) -> float:
        """Closed-form time to traverse distance ``d`` from rest under a
        bang-bang accel-then-(optionally)-coast linear profile. No terminal
        deceleration — for an interceptor we want to be moving on arrival.
        """
        if d <= 0.0:
            return 0.0
        vmax = self.max_vel_linear
        amax = self.max_accel_linear
        if amax <= 0.0:
            return float('inf')
        d_accel = 0.5 * vmax * vmax / amax
        if d <= d_accel:
            return math.sqrt(2.0 * d / amax)
        return vmax / amax + (d - d_accel) / vmax

    def _t_theta_to_angle(self, dtheta: float) -> float:
        """Closed-form time to rotate by ``|dtheta|`` from rest under a
        bang-bang accel-then-(optionally)-coast angular profile. No
        terminal deceleration — yaw matches the linear profile so both
        axes share the same first-half-only structure.
        """
        a = abs(dtheta)
        if a <= 0.0:
            return 0.0
        wmax = self.max_vel_angular
        amax = self.max_accel_angular
        if amax <= 0.0:
            return float('inf')
        a_accel = 0.5 * wmax * wmax / amax
        if a <= a_accel:
            return math.sqrt(2.0 * a / amax)
        return wmax / amax + (a - a_accel) / wmax

    def _arrive_speed_lin(self, d: float) -> float:
        """Speed reached at the intercept under accel-then-coast over ``d``."""
        if d <= 0.0 or self.max_accel_linear <= 0.0:
            return 0.0
        return min(self.max_vel_linear,
                   math.sqrt(2.0 * self.max_accel_linear * d))

    def _arrive_speed_ang(self, dtheta_abs: float) -> float:
        """Angular speed reached at the intercept under accel-then-coast."""
        if dtheta_abs <= 0.0 or self.max_accel_angular <= 0.0:
            return 0.0
        return min(self.max_vel_angular,
                   math.sqrt(2.0 * self.max_accel_angular * dtheta_abs))

    def _final_rest_pose(self, robot6, intercept_xy, intercept_theta):
        """Build the full at-rest pose past the intercept.

        Per axis: the first half accelerates (then optionally coasts) to
        reach the intercept point with some velocity. The second half
        decelerates symmetrically (decel magnitude == that axis's accel
        magnitude during the first half) from that velocity back to zero.
        Returns ``(fx, fy, ftheta)``.

        The linear axes share a common scalar speed ``v_arrive`` along the
        unit vector ``û`` from the robot to the intercept; per-axis accel
        magnitude is ``amax * |û_i|`` so the per-axis decel distance is
        ``v_arrive * |û_i| / (2 * amax) * v_arrive`` — combined this is
        ``û * v_arrive²/(2*amax)`` past the intercept.
        """
        rx, ry, rtheta = robot6[0], robot6[1], robot6[2]
        ix, iy = intercept_xy
        dx, dy = ix - rx, iy - ry
        d = math.hypot(dx, dy)
        v_arr = self._arrive_speed_lin(d)
        a_lin = self.max_accel_linear
        extra_lin = (v_arr * v_arr) / (2.0 * a_lin) if a_lin > 0.0 else 0.0
        if d > 1e-9:
            ux, uy = dx / d, dy / d
        else:
            ux, uy = 0.0, 0.0
        fx = ix + ux * extra_lin
        fy = iy + uy * extra_lin

        dtheta = math.atan2(math.sin(intercept_theta - rtheta),
                            math.cos(intercept_theta - rtheta))
        w_arr = self._arrive_speed_ang(abs(dtheta))
        a_ang = self.max_accel_angular
        extra_ang = (w_arr * w_arr) / (2.0 * a_ang) if a_ang > 0.0 else 0.0
        sign_t = 1.0 if dtheta >= 0.0 else -1.0
        ftheta = intercept_theta + sign_t * extra_ang
        return (fx, fy, ftheta)

    def compute_intercept(self, robot6, ball, bounds):
        """
        Search for the earliest reachable intercept along the ball path.

        For each candidate ball-time ``t``, the intercept point is
        ``b0 + v_ball * t`` (linear ball model). Feasibility uses
        closed-form accel-then-(optional)-coast bang-bang times; the
        candidate is reachable when ``max(t_lin, t_theta) <= t +
        feasible_slack``. After acceptance, the full at-rest pose past
        the intercept is built by ``_final_rest_pose``.

        Returns ``{t, t_required, intercept, theta, final_pose}`` on
        success, else ``None``.
        """
        bx, by, bvx, bvy = ball
        rx, ry, rtheta = robot6[0], robot6[1], robot6[2]

        n_steps = max(1, int(round(self.search_t_max / self.search_dt)))
        for k in range(1, n_steps + 1):
            t = k * self.search_dt
            ix = bx + bvx * t
            iy = by + bvy * t
            if bounds is not None and not self._point_in_bounds(
                    ix, iy, bounds):
                # ball has left the playable area; stop searching forward.
                break
            d = math.hypot(ix - rx, iy - ry)
            theta = math.atan2(iy - ry, ix - rx)
            dtheta = math.atan2(math.sin(theta - rtheta),
                                math.cos(theta - rtheta))
            t_lin = self._t_lin_to_distance(d)
            t_ang = self._t_theta_to_angle(dtheta)
            t_required = max(t_lin, t_ang)
            if t_required <= t + self.feasible_slack:
                final_pose = self._final_rest_pose(
                    robot6, (ix, iy), theta)
                return {'t': t, 't_required': t_required,
                        'intercept': (ix, iy), 'theta': theta,
                        'final_pose': final_pose}

        return None

    # ----------------------------------------------------------- overlays
    def _ns(self) -> str:
        return self.get_name()

    def _sample_path_polyline(self, robot6, intercept_xy, final_xy):
        """Sample positions along the synthetic accel-coast-decel profile.

        The robot moves along a straight line from its start to the
        ``final_xy`` rest pose. Speed: 0 → v_arrive over the first phase
        (accel ± optional coast at vmax), then v_arrive → 0 over the
        symmetric decel phase. We sample ``trajectory_sample_points``
        evenly in time and convert each to a position on the line.
        """
        rx, ry = robot6[0], robot6[1]
        fx, fy = final_xy
        ix, iy = intercept_xy
        dx_total = fx - rx
        dy_total = fy - ry
        d_total = math.hypot(dx_total, dy_total)
        if d_total < 1e-9:
            # Degenerate: just a single point.
            return [Point(x=float(rx), y=float(ry), z=0.0)]
        ux, uy = dx_total / d_total, dy_total / d_total

        # Reconstruct profile parameters along the direction of travel.
        d_to_intercept = math.hypot(ix - rx, iy - ry)
        v_arr = self._arrive_speed_lin(d_to_intercept)
        amax = self.max_accel_linear
        vmax = self.max_vel_linear
        if amax <= 0.0 or v_arr <= 0.0:
            return [Point(x=float(rx), y=float(ry), z=0.0),
                    Point(x=float(fx), y=float(fy), z=0.0)]

        t_acc = v_arr / amax  # time to reach v_arr from rest
        d_acc = 0.5 * amax * t_acc * t_acc  # distance covered while accel
        # Coast phase fills the gap (only present when v_arr == vmax).
        d_coast = max(0.0, d_to_intercept - d_acc)
        t_coast = (d_coast / vmax) if vmax > 0.0 else 0.0
        t_dec = t_acc  # symmetric decel: same magnitude, ends at rest
        t_total = t_acc + t_coast + t_dec

        n = max(2, int(self.trajectory_sample_points))
        points = []
        for i in range(n):
            tau = (t_total * i) / (n - 1)
            if tau <= t_acc:
                s = 0.5 * amax * tau * tau
            elif tau <= t_acc + t_coast:
                s = d_acc + vmax * (tau - t_acc)
            else:
                td = tau - (t_acc + t_coast)
                s = d_acc + d_coast + v_arr * td - 0.5 * amax * td * td
            points.append(
                Point(x=float(rx + ux * s), y=float(ry + uy * s), z=0.0))
        return points

    def publish_overlays(self, robot6, intercept_xy, final_xy):
        """Publish line + point overlays for the planned intercept path."""
        points = self._sample_path_polyline(robot6, intercept_xy, final_xy)
        if len(points) < 2:
            return

        ns = self._ns()
        msg = OverlayArray()
        msg.overlays.append(Overlay(
            ns=ns, name=OVERLAY_TRAJECTORY_NAME, visible=True,
            type=OVERLAY_LINE, command=OVERLAY_CMD_REPLACE,
            position=Point(x=0.0, y=0.0, z=0.0),
            scale=Point(x=1.0, y=1.0, z=0.0),
            stroke_color='#00FF88FF', fill_color='#00FF8800',
            stroke_width=3, lifetime=0, points=points, depth=2,
        ))
        msg.overlays.append(Overlay(
            ns=ns, name=OVERLAY_INTERCEPT_NAME, visible=True,
            type=OVERLAY_POINT, command=OVERLAY_CMD_REPLACE,
            position=Point(
                x=float(intercept_xy[0]), y=float(intercept_xy[1]), z=0.0),
            scale=Point(x=0.08, y=0.08, z=0.0),
            stroke_color='#FFFF00FF', fill_color='#FFFF00FF',
            stroke_width=2, lifetime=0, depth=3,
        ))
        msg.overlays.append(Overlay(
            ns=ns, name=OVERLAY_FINAL_NAME, visible=True,
            type=OVERLAY_POINT, command=OVERLAY_CMD_REPLACE,
            position=Point(
                x=float(final_xy[0]), y=float(final_xy[1]), z=0.0),
            scale=Point(x=0.08, y=0.08, z=0.0),
            stroke_color='#FF44FFFF', fill_color='#FF44FFFF',
            stroke_width=2, lifetime=0, depth=3,
        ))
        self.overlay_pub.publish(msg)
        self.overlays_published = True

    def clear_overlays(self):
        if not self.overlays_published:
            return
        ns = self._ns()
        msg = OverlayArray()
        for name in (OVERLAY_TRAJECTORY_NAME, OVERLAY_INTERCEPT_NAME,
                     OVERLAY_FINAL_NAME):
            msg.overlays.append(Overlay(
                ns=ns, name=name, visible=False,
                type=OVERLAY_LINE, command=OVERLAY_CMD_REMOVE,
            ))
        self.overlay_pub.publish(msg)
        self.overlays_published = False

    # ------------------------------------------------------- command builders
    def make_pos_cmd(self, x, y, theta,
                     dribbler_speed: float = 0.0) -> RobotMotionCommand:
        cmd = RobotMotionCommand()
        cmd.body_control_mode = RobotMotionCommand.BCM_GLOBAL_POSITION
        cmd.pose = Twist2D(x=float(x), y=float(y), theta=float(theta))
        cmd.velocity = Twist2D()
        cmd.acceleration = Twist2D()
        cmd.limit_vel_linear = float(self.max_vel_linear)
        cmd.limit_vel_angular = float(self.max_vel_angular)
        cmd.limit_acc_linear = float(self.max_accel_linear)
        cmd.limit_acc_angular = float(self.max_accel_angular)
        cmd.kick_request = RobotMotionCommand.KR_DISABLE
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

    # ------------------------------------------------------------------ tick
    def tick(self):
        cmd = self.step_state_machine()
        if cmd is not None:
            self.cmd_pub.publish(cmd)

    def _ball_is_incoming(self, ball, bounds) -> bool:
        bx, by, bvx, bvy = ball
        if bvx >= self.incoming_vx_threshold:
            return False
        if bounds is not None and not self._point_in_bounds(bx, by, bounds):
            return False
        return True

    def step_state_machine(self):
        s = self.state
        bounds = self.field_bounds()

        if s == State.WAIT_FOR_BALL:
            ball = self.ball_state()
            if ball is not None and self._ball_is_incoming(ball, bounds):
                self.transition(State.INTERCEPT)
            hx, hy, htheta = self.home_xy_theta
            return self.make_pos_cmd(hx, hy, htheta)

        if s == State.INTERCEPT:
            robot6 = self.robot_state()
            ball = self.ball_state()
            if robot6 is None or ball is None:
                return self.make_off_cmd()
            # Ball stopped or reversed: intercept attempt is over.
            if ball[2] >= self.incoming_vx_threshold and \
                    math.hypot(ball[2], ball[3]) < 0.05:
                self.clear_overlays()
                self.transition(State.DONE)
                return self.make_off_cmd()
            if bounds is not None and not self._point_in_bounds(
                    ball[0], ball[1], bounds):
                self.clear_overlays()
                self.transition(State.DONE)
                return self.make_off_cmd()

            intercept = self.compute_intercept(robot6, ball, bounds)
            if intercept is None:
                # Couldn't plan: hold position, try again next tick.
                return self.make_pos_cmd(
                    robot6[0], robot6[1], robot6[2],
                    dribbler_speed=self.dribbler_speed)
            ix, iy = intercept['intercept']
            fx, fy, ftheta = intercept['final_pose']
            if bounds is not None and not self._point_in_bounds(
                    fx, fy, bounds):
                self.get_logger().warn(
                    'final at-rest pose '
                    f'({fx:.2f}, {fy:.2f}) is outside field bounds; '
                    'skipping this trial.')
                self.clear_overlays()
                self.transition(State.DONE)
                return self.make_off_cmd()
            self.publish_overlays(robot6, (ix, iy), (fx, fy))
            return self.make_pos_cmd(
                fx, fy, ftheta,
                dribbler_speed=self.dribbler_speed)

        if s == State.DONE:
            self.clear_overlays()
            if self.loop and self.time_in_state() >= self.done_dwell:
                self.transition(State.WAIT_FOR_BALL)
            return self.make_off_cmd()

        return self.make_off_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = BallInterceptScenario()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
