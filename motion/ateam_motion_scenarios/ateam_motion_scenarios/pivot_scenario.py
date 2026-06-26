"""
Pivot scenario node for profiling robot motion performance.

Hardcoded to robot 2 on the blue team. Drives the robot through a scripted
sequence: capture the ball, back up so the ball is at the configured circle
radius from the field origin, then pivot around the origin while always
pointing the robot at the center of the circle (i.e. the field origin).

Optionally pauses ("holds") at fixed angular intervals to simulate turning
to pass or shoot.

Geometry conventions
--------------------
The ball is dribbled directly in front of the robot. With the robot facing
the field origin, the ball lies between the robot and the origin. The
``circle_radius`` parameter is the distance of the *ball* center from the
field origin. The robot's body therefore sits at ``circle_radius +
robot_front_offset`` from the origin. For an angle phi on the circle:

    ball_x  = R * cos(phi)
    ball_y  = R * sin(phi)
    robot_x = (R + robot_front_offset) * cos(phi)
    robot_y = (R + robot_front_offset) * sin(phi)
    robot_theta = phi + pi   (robot +x points toward the origin)
"""

from enum import auto, Enum
import math

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
OVERLAY_NS = 'pivot_scenario'


class State(Enum):
    WAIT_FOR_BALL = auto()
    WAIT_FOR_STATIONARY = auto()
    APPROACH_STAGING = auto()
    CAPTURE = auto()
    BACKUP_TO_RADIUS = auto()
    PIVOT = auto()
    HOLD = auto()
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


class PivotScenario(Node):

    def __init__(self):
        super().__init__('pivot_scenario')

        # Capture / staging params (mirrors ball_capture_scenario).
        self.declare_parameter('stationary_ball_threshold', 0.005)
        self.declare_parameter('stationary_dwell', 0.5)
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('robot_front_offset', 0.09)
        self.declare_parameter('capture_overdrive', 0.05)
        self.declare_parameter('capture_vel_limit', 0.15)
        self.declare_parameter('capture_dwell', 0.5)
        self.declare_parameter('pos_tol', 0.03)
        self.declare_parameter('yaw_tol', 0.08)
        self.declare_parameter('dribbler_speed', 250.0)
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('limit_acc_linear', 1.5)
        self.declare_parameter('limit_acc_angular', 15.0)

        # Pivot params.
        self.declare_parameter('circle_radius', 1.0)
        self.declare_parameter('circle_linear_speed', 0.5)
        # +1 = counter-clockwise (math positive), -1 = clockwise.
        self.declare_parameter('circle_direction', 1)
        self.declare_parameter('circle_max_angular_vel', 0.0)
        # Number of full revolutions before stopping. 0 = run forever.
        self.declare_parameter('revolutions', 0.0)
        # Optional starting angle override (degrees). NaN-style default of
        # NaN is awkward in ROS params, so use a sentinel string param.
        self.declare_parameter('start_angle_deg', float('nan'))

        # Hold params: pause at every multiple of hold_interval_deg
        # (measured from the start angle, advancing in circle_direction).
        # 0.0 disables holds.
        self.declare_parameter('hold_interval_deg', 0.0)
        self.declare_parameter('hold_duration', 1.0)

        self.declare_parameter('loop', True)
        self.declare_parameter('loop_dwell', 0.0)

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
        self.pos_tol = float(self.get_parameter('pos_tol').value)
        self.yaw_tol = float(self.get_parameter('yaw_tol').value)
        self.dribbler_speed = float(
            self.get_parameter('dribbler_speed').value)
        self.limit_acc_linear = float(
            self.get_parameter('limit_acc_linear').value)
        self.limit_acc_angular = float(
            self.get_parameter('limit_acc_angular').value)

        self.circle_radius = float(
            self.get_parameter('circle_radius').value)
        self.circle_linear_speed = float(
            self.get_parameter('circle_linear_speed').value)
        cdir = int(self.get_parameter('circle_direction').value)
        self.circle_direction = 1 if cdir >= 0 else -1
        self.circle_max_angular_vel = float(
            self.get_parameter('circle_max_angular_vel').value)
        self.revolutions = float(self.get_parameter('revolutions').value)
        sa = float(self.get_parameter('start_angle_deg').value)
        self.start_angle_rad = math.radians(sa) if not math.isnan(sa) \
            else None

        self.hold_interval_rad = math.radians(
            float(self.get_parameter('hold_interval_deg').value))
        self.hold_duration = float(
            self.get_parameter('hold_duration').value)

        self.loop = bool(self.get_parameter('loop').value)
        self.loop_dwell = float(self.get_parameter('loop_dwell').value)

        if self.circle_radius <= 0.0:
            self.get_logger().error(
                f'circle_radius must be > 0 (got {self.circle_radius}); '
                'pivot will not run')

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

        # State machine.
        self.state = State.WAIT_FOR_BALL
        self.state_entered = self.get_clock().now()
        self.stationary_since = None
        self.capture_arrived_at = None
        self.ball_xy = None  # latched ball (x, y) at WAIT_FOR_STATIONARY

        # Pivot state.
        self.phi = None         # current angular position on circle (rad)
        self.phi_start = None   # angle at start of pivot
        self.phi_traveled = 0.0  # accumulated absolute angular travel (rad)
        # Next hold trigger: total absolute angular travel at which to HOLD.
        self.next_hold_at = None
        self.last_tick_time = None
        # Pose latched when entering HOLD so the robot stays put.
        self.hold_pose = None

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'pivot_scenario started for robot {ROBOT_ID} ({TEAM_COLOR}). '
            f'R={self.circle_radius:.3f} m, '
            f'v={self.circle_linear_speed:.3f} m/s, '
            f'dir={self.circle_direction:+d}. '
            'State: WAIT_FOR_BALL')

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

    def pose_for_phi(self, phi: float):
        """Return (robot_x, robot_y, robot_theta) for a given circle angle."""
        r_robot = self.circle_radius + self.robot_front_offset
        return (r_robot * math.cos(phi),
                r_robot * math.sin(phi),
                phi + math.pi)

    # ------------------------------------------------------------------ tick
    def tick(self):
        cmd = self.step_state_machine()
        if cmd is not None:
            self.cmd_pub.publish(cmd)
        self.publish_overlays()

    # --------------------------------------------------------- visualization
    def _approach_target(self):
        if self.ball_xy is None:
            return None
        return (self.ball_xy[0] - self.approach_offset
                - self.robot_front_offset,
                self.ball_xy[1])

    def _capture_target(self):
        if self.ball_xy is None:
            return None
        return (self.ball_xy[0] - self.robot_front_offset
                + self.capture_overdrive,
                self.ball_xy[1])

    def _circle_polyline(self, n: int = 64):
        pts = []
        for i in range(n + 1):
            t = (2.0 * math.pi * i) / n
            pts.append((self.circle_radius * math.cos(t),
                        self.circle_radius * math.sin(t)))
        return pts

    def publish_overlays(self):
        items = []

        # The reference circle (ball path) and origin marker.
        if self.circle_radius > 0.0:
            items.append(make_line(
                OVERLAY_NS, 'circle',
                self._circle_polyline(),
                color='#FFFFFF60', stroke_width=1))
        items.append(make_point(
            OVERLAY_NS, 'origin', (0.0, 0.0),
            color='#FFFFFFFF', radius=0.02))

        # Latched ball.
        if self.ball_xy is not None:
            items.append(make_point(
                OVERLAY_NS, 'latched_ball', self.ball_xy,
                color='#FFA500FF', radius=0.025))

        s = self.state
        target_color = '#00BFFFFF'
        if s == State.APPROACH_STAGING:
            tgt = self._approach_target()
            if tgt is not None:
                items.append(make_pose_marker(
                    OVERLAY_NS, 'target',
                    (tgt[0], tgt[1], 0.0),
                    self.pos_tol, color=target_color))
        elif s == State.CAPTURE:
            tgt = self._capture_target()
            if tgt is not None:
                items.append(make_pose_marker(
                    OVERLAY_NS, 'target',
                    (tgt[0], tgt[1], 0.0),
                    self.pos_tol, color='#00FF7FFF'))
        elif s == State.BACKUP_TO_RADIUS:
            if self.phi_start is not None:
                pose = self.pose_for_phi(self.phi_start)
                items.append(make_pose_marker(
                    OVERLAY_NS, 'target', pose,
                    self.pos_tol, color=target_color,
                    heading_length=0.3))
        elif s == State.PIVOT:
            if self.phi is not None:
                pose = self.pose_for_phi(self.phi)
                items.append(make_pose_marker(
                    OVERLAY_NS, 'target', pose,
                    self.pos_tol, color='#00FF7FFF',
                    heading_length=0.3))
        elif s == State.HOLD:
            if self.hold_pose is not None:
                items.append(make_pose_marker(
                    OVERLAY_NS, 'target', self.hold_pose,
                    self.pos_tol, color='#FFFF00FF',
                    heading_length=0.3))

        rs = self.robot_xy_yaw()
        if rs is not None:
            items.append(make_text(
                OVERLAY_NS, 'state',
                self.state.name,
                (rs[0], rs[1] + 0.25),
                color='#FFFFFFFF', font_size=20))

        if items:
            self.overlay_pub.publish(make_array(*items))

    # ----------------------------------------------------------- transitions
    def _enter_pivot(self):
        """Initialize pivot state, computing the starting angle."""
        if self.start_angle_rad is not None:
            self.phi_start = self.start_angle_rad
        elif self.ball_xy is not None and \
                math.hypot(self.ball_xy[0], self.ball_xy[1]) > 1e-3:
            self.phi_start = math.atan2(self.ball_xy[1], self.ball_xy[0])
        else:
            # Fall back to current robot heading along radial direction.
            rs = self.robot_xy_yaw()
            if rs is not None and math.hypot(rs[0], rs[1]) > 1e-3:
                self.phi_start = math.atan2(rs[1], rs[0])
            else:
                self.phi_start = 0.0
        self.phi = self.phi_start
        self.phi_traveled = 0.0
        self.last_tick_time = None
        if self.hold_interval_rad > 0.0:
            self.next_hold_at = self.hold_interval_rad
        else:
            self.next_hold_at = None

    # --------------------------------------------------------- state machine
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
            tx = self.ball_xy[0] - self.approach_offset \
                - self.robot_front_offset
            ty = self.ball_xy[1]
            if self.at_pose((tx, ty), self.pos_tol):
                self.transition(State.CAPTURE)
            return self.make_pos_cmd(tx, ty)

        if s == State.CAPTURE:
            tx = self.ball_xy[0] - self.robot_front_offset \
                + self.capture_overdrive
            ty = self.ball_xy[1]
            cmd = self.make_pos_cmd(
                tx, ty,
                vel_limit=self.capture_vel_limit,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            if self.at_pose((tx, ty), self.pos_tol):
                now = self.get_clock().now()
                if self.capture_arrived_at is None:
                    self.capture_arrived_at = now
                elif (now - self.capture_arrived_at).nanoseconds * 1e-9 \
                        >= self.capture_dwell:
                    if self.circle_radius <= 0.0:
                        self.transition(State.DONE)
                    else:
                        self._enter_pivot()
                        self.transition(State.BACKUP_TO_RADIUS)
            else:
                self.capture_arrived_at = None
            return cmd

        if s == State.BACKUP_TO_RADIUS:
            tx, ty, ttheta = self.pose_for_phi(self.phi_start)
            cmd = self.make_pos_cmd(
                tx, ty, ttheta,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            if self.at_pose((tx, ty), self.pos_tol,
                            yaw_target=ttheta, yaw_tol=self.yaw_tol):
                # Reset tick timer so PIVOT advances from this moment.
                self.last_tick_time = None
                self.transition(State.PIVOT)
            return cmd

        if s == State.PIVOT:
            now = self.get_clock().now()
            if self.last_tick_time is None:
                dt = 0.0
            else:
                dt = (now - self.last_tick_time).nanoseconds * 1e-9
            self.last_tick_time = now

            omega = self.circle_linear_speed / self.circle_radius
            if self.circle_max_angular_vel > 0.0:
                omega = min(omega, self.circle_max_angular_vel)
            d_phi = self.circle_direction * omega * dt

            # Check whether this step crosses the next hold threshold.
            if self.next_hold_at is not None and \
                    self.phi_traveled + abs(d_phi) >= self.next_hold_at:
                # Snap to the hold angle exactly.
                remaining = self.next_hold_at - self.phi_traveled
                self.phi += self.circle_direction * remaining
                self.phi_traveled = self.next_hold_at
                self.hold_pose = self.pose_for_phi(self.phi)
                self.transition(State.HOLD)
                # Issue the snap pose this tick.
                hx, hy, htheta = self.hold_pose
                return self.make_pos_cmd(
                    hx, hy, htheta,
                    kick_request=RobotMotionCommand.KR_ARM,
                    dribbler_speed=self.dribbler_speed,
                )

            self.phi += d_phi
            self.phi_traveled += abs(d_phi)

            # Check revolutions limit.
            if self.revolutions > 0.0 and \
                    self.phi_traveled >= self.revolutions * 2.0 * math.pi:
                self.transition(State.DONE)
                return self.make_off_cmd()

            tx, ty, ttheta = self.pose_for_phi(self.phi)
            return self.make_pos_cmd(
                tx, ty, ttheta,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )

        if s == State.HOLD:
            hx, hy, htheta = self.hold_pose
            cmd = self.make_pos_cmd(
                hx, hy, htheta,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            if self.time_in_state() >= self.hold_duration:
                # Schedule next hold and resume pivot.
                if self.hold_interval_rad > 0.0:
                    self.next_hold_at = \
                        self.phi_traveled + self.hold_interval_rad
                else:
                    self.next_hold_at = None
                self.last_tick_time = None
                self.transition(State.PIVOT)
            return cmd

        if s == State.DONE:
            if self.loop and self.time_in_state() >= self.loop_dwell:
                self.ball_xy = None
                self.phi = None
                self.phi_start = None
                self.phi_traveled = 0.0
                self.next_hold_at = None
                self.hold_pose = None
                self.transition(State.WAIT_FOR_BALL)
            return self.make_off_cmd()

        return self.make_off_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = PivotScenario()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
