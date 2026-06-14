"""
Ball intercept scenario node.

Hardcoded to robot 2 on the blue team. Holds the robot at the origin while
waiting for a human to roll the ball in from the +x edge of the field. Once
the ball is moving inward (negative x-velocity), continuously plans a
bang-bang trajectory (using the Rust-backed `ateam_controls` Python bindings)
to the earliest reachable point on the ball's predicted path, and commands
the robot toward an "overshoot" point past that intercept so it is still
moving when it crosses the ball. The dribbler roller is enabled while
intercepting and the robot is kept facing the ball at the intercept point.
The selected trajectory and intercept marker are published to `/overlays`
for visualization in the UI.

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
* feasible_slack         — slack added to t when checking trajectory end time (default 0.05 s)
* overshoot_distance     — distance past the intercept along the ball path, m (default 0.5)
* field_margin           — keep targets at least this far from field edges, m (default 0.1)
* incoming_vx_threshold  — ball x-velocity must be below this to start (default -0.05 m/s)
* dribbler_speed         — dribbler RPM during intercept (default 250)
* loop                   — return to WAIT_FOR_BALL after DONE (default True)
* require_controls       — fail to start if `ateam_controls` is unavailable (default True)

Note: requires the `ateam_controls` Python package from the SSL-A-Team/controls
repo (`uv sync` in that repo, then activate its venv before launching this
node). If the package can't be imported, the node logs a clear error and
shuts down (unless `require_controls` is False, in which case it stays in
WAIT_FOR_BALL forever).
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
OVERLAY_OVERSHOOT_NAME = 'overshoot_target'


def yaw_from_quat(q) -> float:
    """Extract yaw from a planar quaternion (assumes roll=pitch=0)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _try_import_controls(logger):
    """Import ateam_controls lazily so the rest of the package keeps working."""
    try:
        import ateam_controls  # noqa: F401
        return ateam_controls
    except Exception as e:  # ImportError, FileNotFoundError, etc.
        logger.error(
            'Failed to import ateam_controls: %s. Install it from the '
            'SSL-A-Team/controls repository (run `uv sync` there and source '
            'its .venv) before launching this scenario.' % (e,))
        return None


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
        self.declare_parameter('overshoot_distance', 0.5)
        self.declare_parameter('field_margin', 0.1)
        self.declare_parameter('incoming_vx_threshold', -0.05)
        self.declare_parameter('dribbler_speed', 250.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('require_controls', True)
        self.declare_parameter('trajectory_sample_dt', 0.05)
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
        self.overshoot_distance = float(
            self.get_parameter('overshoot_distance').value)
        self.field_margin = float(self.get_parameter('field_margin').value)
        self.incoming_vx_threshold = float(
            self.get_parameter('incoming_vx_threshold').value)
        self.dribbler_speed = float(self.get_parameter('dribbler_speed').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.require_controls = bool(
            self.get_parameter('require_controls').value)
        self.trajectory_sample_dt = float(
            self.get_parameter('trajectory_sample_dt').value)
        self.done_dwell = float(self.get_parameter('done_dwell').value)
        self.home_xy_theta = (
            float(self.get_parameter('home_pose_x').value),
            float(self.get_parameter('home_pose_y').value),
            float(self.get_parameter('home_pose_theta').value),
        )
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.controls = _try_import_controls(self.get_logger())
        if self.controls is None and self.require_controls:
            raise RuntimeError(
                'ateam_controls is required but could not be imported.')

        self.traj_params = None
        if self.controls is not None:
            self.traj_params = self.controls.TrajectoryParams(
                max_vel_linear=self.max_vel_linear,
                max_vel_angular=self.max_vel_angular,
                max_accel_linear=self.max_accel_linear,
                max_accel_angular=self.max_accel_angular,
            )

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

    @staticmethod
    def _clamp_segment_to_bounds(x0, y0, x1, y1, bounds):
        """Clip endpoint (x1,y1) of segment from (x0,y0) so it stays in bounds.

        Returns the (cx, cy) on the segment closest to (x1,y1) that's in
        bounds (inclusive of axes). Assumes (x0,y0) is in bounds. If the
        whole segment is in bounds, returns (x1,y1).
        """
        xmin, ymin, xmax, ymax = bounds
        dx = x1 - x0
        dy = y1 - y0
        s = 1.0
        if dx > 0.0:
            s = min(s, (xmax - x0) / dx)
        elif dx < 0.0:
            s = min(s, (xmin - x0) / dx)
        if dy > 0.0:
            s = min(s, (ymax - y0) / dy)
        elif dy < 0.0:
            s = min(s, (ymin - y0) / dy)
        s = max(0.0, min(1.0, s))
        return (x0 + s * dx, y0 + s * dy)

    # ----------------------------------------------------------- intercept
    def _solve_traj(self, init_state6, target_xy, target_theta):
        """Solve a bang-bang trajectory; return (traj, t_end) or (None,None)."""
        c = self.controls
        if c is None:
            return None, None
        try:
            init = c.Vector6C()
            init.data[0] = float(init_state6[0])
            init.data[1] = float(init_state6[1])
            init.data[2] = float(init_state6[2])
            init.data[3] = float(init_state6[3])
            init.data[4] = float(init_state6[4])
            init.data[5] = float(init_state6[5])
            target = c.Vector3C(
                x=float(target_xy[0]),
                y=float(target_xy[1]),
                z=float(target_theta),
            )
            traj = c.traj_from_target_pose(init, target, self.traj_params)
            t_end = c.traj_end_time(traj)
            return traj, float(t_end)
        except Exception as e:
            self.get_logger().warn(
                f'traj_from_target_pose failed: {e}',
                throttle_duration_sec=2.0)
            return None, None

    def compute_intercept(self, robot6, ball, bounds):
        """
        Search for the earliest reachable intercept along the ball path.

        Returns dict {t, intercept, traj} on success, else None.
        """
        if self.controls is None:
            return None
        bx, by, bvx, bvy = ball
        rx, ry = robot6[0], robot6[1]

        n_steps = max(1, int(round(self.search_t_max / self.search_dt)))
        last_intercept_xy = None
        for k in range(1, n_steps + 1):
            t = k * self.search_dt
            ix = bx + bvx * t
            iy = by + bvy * t
            if bounds is not None and not self._point_in_bounds(
                    ix, iy, bounds):
                # ball has left the playable area; stop searching forward.
                break
            theta = math.atan2(iy - ry, ix - rx)
            traj, t_end = self._solve_traj(robot6, (ix, iy), theta)
            if traj is None or t_end is None:
                continue
            if t_end <= t + self.feasible_slack:
                return {'t': t, 'intercept': (ix, iy), 'traj': traj,
                        'theta': theta}
            last_intercept_xy = (ix, iy)

        # Fallback: nearest projection of robot onto the ball's line.
        bv_mag2 = bvx * bvx + bvy * bvy
        if bv_mag2 < 1e-6:
            target_xy = (bx, by)
        else:
            s = ((rx - bx) * bvx + (ry - by) * bvy) / bv_mag2
            s = max(0.0, s)  # only forward along ball path
            target_xy = (bx + s * bvx, by + s * bvy)
        if bounds is not None:
            if not self._point_in_bounds(*target_xy, bounds):
                if last_intercept_xy is not None:
                    target_xy = last_intercept_xy
                else:
                    target_xy = self._clamp_segment_to_bounds(
                        bx, by, target_xy[0], target_xy[1], bounds)
        theta = math.atan2(target_xy[1] - ry, target_xy[0] - rx)
        traj, t_end = self._solve_traj(robot6, target_xy, theta)
        if traj is None:
            return None
        return {'t': float(t_end), 'intercept': target_xy, 'traj': traj,
                'theta': theta, 'fallback': True}

    def overshoot_target(self, intercept_xy, ball, bounds):
        bx, by, bvx, bvy = ball
        bv_mag = math.hypot(bvx, bvy)
        ix, iy = intercept_xy
        if bv_mag < 1e-6:
            ox, oy = ix, iy
        else:
            ux, uy = bvx / bv_mag, bvy / bv_mag
            ox = ix + self.overshoot_distance * ux
            oy = iy + self.overshoot_distance * uy
        if bounds is not None:
            ox, oy = self._clamp_segment_to_bounds(ix, iy, ox, oy, bounds)
            if not self._point_in_bounds(ox, oy, bounds):
                ox, oy = self._clamp_segment_to_bounds(
                    bx, by, ox, oy, bounds)
        return (ox, oy)

    # ----------------------------------------------------------- overlays
    def _ns(self) -> str:
        return self.get_name()

    def publish_overlays(self, traj, t_intercept, robot6, intercept_xy,
                         overshoot_xy):
        """Sample the trajectory and publish line+point overlays."""
        if self.controls is None or traj is None:
            return
        c = self.controls
        cur = c.Vector6C()
        cur.data[0] = float(robot6[0])
        cur.data[1] = float(robot6[1])
        cur.data[2] = float(robot6[2])
        cur.data[3] = float(robot6[3])
        cur.data[4] = float(robot6[4])
        cur.data[5] = float(robot6[5])

        t_end = max(t_intercept, 0.05)
        dt = max(self.trajectory_sample_dt, 1e-3)
        n = max(2, int(math.ceil(t_end / dt)) + 1)
        points = []
        for i in range(n):
            t = min(i * dt, t_end)
            try:
                s = c.traj_state_at(traj, cur, 0.0, t)
                points.append(Point(x=float(s.data[0]),
                                    y=float(s.data[1]), z=0.0))
            except Exception:
                break
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
            ns=ns, name=OVERLAY_OVERSHOOT_NAME, visible=True,
            type=OVERLAY_POINT, command=OVERLAY_CMD_REPLACE,
            position=Point(
                x=float(overshoot_xy[0]), y=float(overshoot_xy[1]), z=0.0),
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
                     OVERLAY_OVERSHOOT_NAME):
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
            overshoot = self.overshoot_target((ix, iy), ball, bounds)
            # Face the ball (current ball pose), not the intercept point.
            theta_face = math.atan2(ball[1] - robot6[1],
                                    ball[0] - robot6[0])
            self.publish_overlays(
                intercept['traj'], intercept['t'], robot6,
                (ix, iy), overshoot)
            return self.make_pos_cmd(
                overshoot[0], overshoot[1], theta_face,
                dribbler_speed=self.dribbler_speed)

        if s == State.DONE:
            self.clear_overlays()
            if self.loop and self.time_in_state() >= self.done_dwell:
                self.transition(State.WAIT_FOR_BALL)
            return self.make_off_cmd()

        return self.make_off_cmd()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BallInterceptScenario()
    except RuntimeError as e:
        # ateam_controls missing and require_controls=True.
        print(f'ball_intercept_scenario: {e}')
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
