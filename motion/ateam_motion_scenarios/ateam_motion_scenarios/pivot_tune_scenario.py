"""
Pivot-tuning scenario node.

Waits until a ball is *manually* placed into the robot's dribbler (detected
via the firmware breakbeam on the basic feedback topic), then sweeps the robot
around a full circle in steps of a parameterized angular interval using the
firmware ``BCM_PIVOT`` maneuver -- analogous to the firmware ``hwtest-pivot``
binary (control-board/src/bin/hwtest-pivot/main.rs), but driven over ROS so
the pivot can be tuned on the real stack.

State machine
-------------
``WAIT_FOR_CONNECTION -> WAIT_FOR_BALL -> PIVOT_SWEEP`` (loops forever).

  * WAIT_FOR_BALL: holds the motors off until the breakbeam reports the ball
    settled for ``ball_settle_count`` consecutive frames.
  * PIVOT_SWEEP: latches the start heading and steps the target heading by
    ``pivot_interval_deg`` each leg, holding each leg for
    ``pivot_exec_duration + pivot_hold_duration`` seconds, wrapping after a
    full circle and repeating. If the ball is lost (breakbeam clears for
    ``ball_lost_count`` frames) and ``require_ball`` is set, it returns to
    WAIT_FOR_BALL.

All pivot commands are built by the shared ``pivot`` module.

Parameters are loaded from ``config/pivot_tune_params.json`` by default
(override with the ``param_file`` ROS parameter); every JSON key becomes the
default for a ROS parameter of the same name.
"""

from enum import auto, Enum
import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from ateam_motion_scenarios.overlays import (
    make_array,
    make_circle,
    make_point,
    make_pose_marker,
    make_text,
)
from ateam_motion_scenarios.pivot import (
    circle_leg_count,
    leg_target,
    PivotConfig,
)
from ateam_msgs.msg import (
    OverlayArray,
    RobotMotionCommand,
    Twist2D,
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


OVERLAY_NS = 'pivot_tune_scenario'
DEFAULT_PARAM_FILE = os.path.join(
    get_package_share_directory('ateam_motion_scenarios'),
    'config', 'pivot_tune_params.json')


class State(Enum):
    WAIT_FOR_CONNECTION = auto()
    WAIT_FOR_BALL = auto()
    PIVOT_SWEEP = auto()


def yaw_from_quat(q) -> float:
    """Extract yaw from a planar quaternion (assumes roll=pitch=0)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PivotTuneScenario(Node):

    def __init__(self):
        super().__init__('pivot_tune_scenario')

        self.defaults = self._load_defaults()

        self.robot_id = int(self._p('robot_id', 2))
        self.team_color = str(self._p('team_color', 'blue'))

        # Angular interval (per leg) of the circular sweep.
        self.pivot_interval = math.radians(
            float(self._p('pivot_interval_deg', 45.0)))
        # Time spent pivoting toward / holding each leg target (seconds).
        self.pivot_exec_duration = float(
            self._p('pivot_exec_duration', 3.0))
        self.pivot_hold_duration = float(
            self._p('pivot_hold_duration', 1.0))

        # Shared firmware-pivot config (orbit radius, inset, limits, dribbler).
        self.pivot = PivotConfig.from_params(self._p)

        # Ball detection via the firmware breakbeam (basic telemetry).
        self.ball_settle_count = int(self._p('ball_settle_count', 10))
        self.ball_lost_count = int(self._p('ball_lost_count', 30))
        self.require_ball = bool(self._p('require_ball', True))
        self.breakbeam_topic = str(self._p('breakbeam_topic', '')) \
            or f'/robot_feedback/basic/robot{self.robot_id}'

        # Radio connection monitoring.
        self.require_connection = bool(self._p('require_connection', True))
        self.connection_topic = str(self._p('connection_topic', '')) \
            or f'/robot_feedback/connection/robot{self.robot_id}'

        rate = float(self._p('publish_rate_hz', 60.0))

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Match the radio bridge's SystemDefaultsQoS for robot feedback.
        feedback_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.robot = None
        self.radio_connected = None
        self.breakbeam_detected = None

        self.robot_sub = self.create_subscription(
            VisionStateRobot, f'/{self.team_color}_team/robot{self.robot_id}',
            self.robot_cb, sensor_qos)
        self.connection_sub = self.create_subscription(
            ConnectionStatus, self.connection_topic,
            self.connection_cb, feedback_qos)
        self.telemetry_sub = self.create_subscription(
            BasicTelemetry, self.breakbeam_topic,
            self.telemetry_cb, feedback_qos)

        self.cmd_pub = self.create_publisher(
            RobotMotionCommand,
            f'/robot_motion_commands/robot{self.robot_id}', 10)
        self.overlay_pub = self.create_publisher(
            OverlayArray, '/overlays', 10)

        self.state = (State.WAIT_FOR_CONNECTION if self.require_connection
                      else State.WAIT_FOR_BALL)
        self.state_entered = self.get_clock().now()
        self.detect_filter = 0       # breakbeam settle/lost debounce
        self.sweep_start_heading = None
        self.leg_index = 0
        self.leg_started = None
        self.num_legs = circle_leg_count(self.pivot_interval)

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'pivot_tune_scenario started for robot {self.robot_id} '
            f'({self.team_color}). interval='
            f'{math.degrees(self.pivot_interval):.1f} deg, '
            f'{self.num_legs} legs/circle, orbit_radius='
            f'{self.pivot.orbit_radius:.3f} m, inset='
            f'{self.pivot.inset_angle:.2f} rad. State: {self.state.name}')

    # --------------------------------------------------------------- params
    def _load_defaults(self) -> dict:
        self.declare_parameter('param_file', '')
        pf = str(self.get_parameter('param_file').value) or DEFAULT_PARAM_FILE
        try:
            with open(pf) as f:
                data = json.load(f)
            self.get_logger().info(f'Loaded pivot params from {pf}')
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
    def robot_cb(self, msg: VisionStateRobot):
        self.robot = msg

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

    def time_in_state(self) -> float:
        return (self.get_clock().now() - self.state_entered).nanoseconds * 1e-9

    def robot_yaw(self):
        if self.robot is None or not self.robot.visible:
            return None
        return yaw_from_quat(self.robot.pose.orientation)

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
        self.publish_overlays()

    def _current_target(self):
        if self.sweep_start_heading is None:
            return None
        return leg_target(
            self.sweep_start_heading, self.pivot_interval, self.leg_index)

    # --------------------------------------------------------- state machine
    def step_state_machine(self):
        s = self.state

        if s == State.WAIT_FOR_CONNECTION:
            if self.radio_connected:
                self.get_logger().info('Robot connected')
                self.transition(State.WAIT_FOR_BALL)
            return self.make_off_cmd()

        if s == State.WAIT_FOR_BALL:
            if self.breakbeam_detected:
                self.detect_filter += 1
            else:
                self.detect_filter = 0
            if self.detect_filter >= self.ball_settle_count:
                start = self.robot_yaw()
                self.sweep_start_heading = start if start is not None else 0.0
                self.leg_index = 0
                self.leg_started = self.get_clock().now()
                self.detect_filter = 0
                self.get_logger().info(
                    'Ball detected; starting pivot sweep at '
                    f'{math.degrees(self.sweep_start_heading):.1f} deg')
                self.transition(State.PIVOT_SWEEP)
            return self.make_off_cmd()

        if s == State.PIVOT_SWEEP:
            # Lost-ball guard.
            if self.require_connection and self.radio_connected is False:
                self.transition(State.WAIT_FOR_CONNECTION)
                return self.make_off_cmd()
            if self.require_ball:
                if self.breakbeam_detected is False:
                    self.detect_filter += 1
                else:
                    self.detect_filter = 0
                if self.detect_filter >= self.ball_lost_count:
                    self.get_logger().warn(
                        'Ball lost; waiting for it to be replaced')
                    self.detect_filter = 0
                    self.transition(State.WAIT_FOR_BALL)
                    return self.make_off_cmd()

            # Advance to the next leg after exec + hold time.
            now = self.get_clock().now()
            leg_dur = self.pivot_exec_duration + self.pivot_hold_duration
            if self.leg_started is None:
                self.leg_started = now
            elif (now - self.leg_started).nanoseconds * 1e-9 >= leg_dur:
                self.leg_index = (self.leg_index + 1) % self.num_legs
                self.leg_started = now
                self.get_logger().info(
                    f'Pivot leg {self.leg_index}/{self.num_legs} -> '
                    f'{math.degrees(self._current_target()):.1f} deg')

            return self.pivot.command(
                self._current_target(),
                kick_request=RobotMotionCommand.KR_DISABLE)

        return self.make_off_cmd()

    # --------------------------------------------------------- visualization
    def publish_overlays(self):
        items = []
        rs = self.robot
        if rs is not None and rs.visible:
            p = rs.pose.position
            items.append(make_text(
                OVERLAY_NS, 'state', self.state.name,
                (p.x, p.y + 0.25), color='#FFFFFFFF', font_size=20))
            if self.state == State.PIVOT_SWEEP:
                items.append(make_circle(
                    OVERLAY_NS, 'orbit', (p.x, p.y),
                    self.pivot.orbit_radius, stroke_color='#FFFF0060'))
                tgt = self._current_target()
                if tgt is not None:
                    items.append(make_pose_marker(
                        OVERLAY_NS, 'target', (p.x, p.y, tgt),
                        0.05, color='#FFFF00FF', heading_length=0.3))
            else:
                items.append(make_point(
                    OVERLAY_NS, 'robot', (p.x, p.y),
                    color='#00BFFFFF', radius=0.03))
        if items:
            self.overlay_pub.publish(make_array(*items))


def main(args=None):
    rclpy.init(args=args)
    node = PivotTuneScenario()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
