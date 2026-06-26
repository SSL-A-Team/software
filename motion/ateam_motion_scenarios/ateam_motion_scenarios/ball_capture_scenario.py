"""
Ball capture scenario node for profiling robot motion performance.

Hardcoded to robot 2 on the blue team. Drives the robot through a scripted
sequence: approach a stationary ball, gently capture it, retreat, kick it
forward, then reset toward the back of the field.
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
    RETREAT_WITH_BALL = auto()
    STRAFE = auto()
    AIM = auto()
    KICK_AIMED = auto()
    WAIT_BEFORE_KICK = auto()
    KICK = auto()
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


class BallCaptureScenario(Node):

    def __init__(self):
        super().__init__('ball_capture_scenario')

        self.declare_parameter('stationary_ball_threshold', 0.005)
        self.declare_parameter('stationary_dwell', 0.5)
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('robot_front_offset', 0.09)
        self.declare_parameter('capture_overdrive', 0.05)
        self.declare_parameter('strafe_enabled', False)
        self.declare_parameter('strafe_speed', 0.4)
        self.declare_parameter('strafe_angle_deg', -10.0)
        self.declare_parameter('strafe_duration', 2.0)
        self.declare_parameter('post_kick_strafe', 0.0)
        self.declare_parameter('aim_enabled', True)
        self.declare_parameter('aim_dwell', 0.3)
        self.declare_parameter('aim_max_angular_vel', 0.5)
        self.declare_parameter('pos_tol', 0.03)
        self.declare_parameter('yaw_tol', 0.08)
        self.declare_parameter('capture_vel_limit', 0.15)
        self.declare_parameter('capture_dwell', 0.5)
        self.declare_parameter('pre_kick_dwell', 0.0)
        self.declare_parameter('kick_speed', 2.0)
        self.declare_parameter('dribbler_speed', 250.0)
        self.declare_parameter('reset_margin', 0.3)
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('loop_dwell', 0.0)
        self.declare_parameter('limit_acc_linear', 1.5)
        self.declare_parameter('limit_acc_angular', 15.0)

        self.stationary_ball_threshold = self.get_parameter(
            'stationary_ball_threshold').value
        self.stationary_dwell = self.get_parameter('stationary_dwell').value
        self.approach_offset = self.get_parameter('approach_offset').value
        self.robot_front_offset = self.get_parameter(
            'robot_front_offset').value
        self.capture_overdrive = self.get_parameter('capture_overdrive').value
        self.strafe_enabled = bool(
            self.get_parameter('strafe_enabled').value)
        self.strafe_speed = float(self.get_parameter('strafe_speed').value)
        self.strafe_angle_rad = math.radians(
            float(self.get_parameter('strafe_angle_deg').value))
        self.strafe_duration = float(
            self.get_parameter('strafe_duration').value)
        self.post_kick_strafe = float(
            self.get_parameter('post_kick_strafe').value)
        self.aim_enabled = bool(self.get_parameter('aim_enabled').value)
        self.aim_dwell = float(self.get_parameter('aim_dwell').value)
        self.aim_max_angular_vel = float(
            self.get_parameter('aim_max_angular_vel').value)
        self.pos_tol = float(self.get_parameter('pos_tol').value)
        self.yaw_tol = float(self.get_parameter('yaw_tol').value)
        self.capture_vel_limit = self.get_parameter('capture_vel_limit').value
        self.capture_dwell = self.get_parameter('capture_dwell').value
        self.pre_kick_dwell = self.get_parameter('pre_kick_dwell').value
        self.kick_speed = self.get_parameter('kick_speed').value
        self.dribbler_speed = self.get_parameter('dribbler_speed').value
        self.reset_margin = self.get_parameter('reset_margin').value
        self.loop = bool(self.get_parameter('loop').value)
        self.loop_dwell = float(self.get_parameter('loop_dwell').value)
        self.limit_acc_linear = float(
            self.get_parameter('limit_acc_linear').value)
        self.limit_acc_angular = float(
            self.get_parameter('limit_acc_angular').value)
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
        self.ball_xy = None  # latched ball (x, y)
        self.aim_hold = None  # latched (x, y, theta) for AIM/KICK_AIMED
        self.aim_arrived_at = None

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'ball_capture_scenario started for robot {ROBOT_ID} '
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

    def make_global_vel_cmd(self, vx: float, vy: float, vtheta: float = 0.0,
                            kick_request: int = RobotMotionCommand.KR_DISABLE,
                            dribbler_speed: float = 0.0,
                            kick_speed: float = 0.0) -> RobotMotionCommand:
        cmd = RobotMotionCommand()
        cmd.body_control_mode = RobotMotionCommand.BCM_GLOBAL_VELOCITY
        cmd.pose = Twist2D()
        cmd.velocity = Twist2D(
            x=float(vx), y=float(vy), theta=float(vtheta))
        cmd.acceleration = Twist2D()
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
                    if self.strafe_enabled and self.strafe_duration > 0.0:
                        self.transition(State.RETREAT_WITH_BALL)
                    elif self.aim_enabled:
                        self.aim_hold = None
                        self.transition(State.AIM)
                    else:
                        self.transition(State.RETREAT_WITH_BALL)
            else:
                self.capture_arrived_at = None
            return cmd

        if s == State.RETREAT_WITH_BALL:
            tx = self.ball_xy[0] - self.approach_offset \
                - self.robot_front_offset
            ty = self.ball_xy[1]
            cmd = self.make_pos_cmd(
                tx, ty,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            if self.at_pose((tx, ty), self.pos_tol):
                if self.strafe_enabled and self.strafe_duration > 0.0:
                    self.transition(State.STRAFE)
                else:
                    self.transition(State.WAIT_BEFORE_KICK)
            return cmd

        if s == State.STRAFE:
            # angle measured from -y axis, positive rotates toward +x
            vx = self.strafe_speed * math.sin(self.strafe_angle_rad)
            vy = -self.strafe_speed * math.cos(self.strafe_angle_rad)
            cmd = self.make_global_vel_cmd(
                vx, vy,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            if self.time_in_state() >= self.strafe_duration:
                if self.aim_enabled:
                    self.transition(State.AIM)
                else:
                    self.transition(State.WAIT_BEFORE_KICK)
            return cmd

        if s == State.AIM:
            rs = self.robot_xy_yaw()
            if rs is None:
                return self.make_off_cmd()
            if self.field is None or self.field.field_length <= 0.0:
                self.get_logger().warn(
                    'AIM: waiting for /field...',
                    throttle_duration_sec=2.0)
                return self.make_off_cmd()
            rx, ry, _ = rs
            target_x = self.field.field_length / 2.0
            target_y = 0.0
            theta = math.atan2(target_y - ry, target_x - rx)
            cmd = self.make_pos_cmd(
                rx, ry, theta,
                ang_vel_limit=self.aim_max_angular_vel,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            rs2 = self.robot_xy_yaw()
            yaw_ok = rs2 is not None and \
                abs(ang_diff(rs2[2], theta)) <= self.yaw_tol
            if yaw_ok:
                now = self.get_clock().now()
                if self.aim_arrived_at is None:
                    self.aim_arrived_at = now
                elif (now - self.aim_arrived_at).nanoseconds * 1e-9 \
                        >= self.aim_dwell:
                    self.aim_hold = (rx, ry, theta)
                    self.transition(State.KICK_AIMED)
            else:
                self.aim_arrived_at = None
            return cmd

        if s == State.KICK_AIMED:
            hx, hy, htheta = self.aim_hold
            cmd = self.make_pos_cmd(
                hx, hy, htheta,
                ang_vel_limit=self.aim_max_angular_vel,
                kick_request=RobotMotionCommand.KR_KICK_NOW,
                kick_speed=self.kick_speed,
                dribbler_speed=self.dribbler_speed,
            )
            if self.time_in_state() >= 0.2:
                self.transition(State.RESET)
            return cmd

        if s == State.WAIT_BEFORE_KICK:
            tx = self.ball_xy[0] - self.approach_offset \
                - self.robot_front_offset
            ty = self.ball_xy[1]
            cmd = self.make_pos_cmd(
                tx, ty,
                kick_request=RobotMotionCommand.KR_ARM,
                dribbler_speed=self.dribbler_speed,
            )
            if self.time_in_state() >= self.pre_kick_dwell:
                self.transition(State.KICK)
            return cmd

        if s == State.KICK:
            tx = self.ball_xy[0] - self.approach_offset \
                - self.robot_front_offset
            ty = self.ball_xy[1]
            cmd = self.make_pos_cmd(
                tx, ty,
                kick_request=RobotMotionCommand.KR_KICK_NOW,
                kick_speed=self.kick_speed,
                dribbler_speed=self.dribbler_speed,
            )
            if self.time_in_state() >= 0.2:
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
                self.transition(State.WAIT_FOR_BALL)
            return self.make_off_cmd()

        return self.make_off_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = BallCaptureScenario()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
