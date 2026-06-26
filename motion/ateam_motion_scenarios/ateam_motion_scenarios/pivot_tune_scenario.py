"""
Pivot-tuning scenario node.

Waits until a ball is *manually* placed into the robot's dribbler (detected
via the firmware breakbeam on the basic feedback topic), then sweeps the robot
using a firmware pivot maneuver -- analogous to the firmware ``hwtest-pivot``
binary (control-board/src/bin/hwtest-pivot/main.rs), but driven over ROS so
the pivot can be tuned on the real stack.

Two sweep modes (``pivot_mode``):
  * ``heading`` (default): steps the target heading around a full circle in
    ``pivot_interval_deg`` increments using ``BCM_HEADING_PIVOT``.
  * ``point``: cycles the robot to face each of the four field-edge centers
    (the goalline/touchline midpoints from ``/field``) using
    ``BCM_POINT_PIVOT``.

Set ``pivot_slide_enabled`` to continuously recompute the leg target while
the pivot executes (to observe firmware tracking of a moving goal): in point
mode the target point slides ``pivot_slide_distance`` m along the field edge;
in heading mode the target heading slides ``pivot_slide_heading_frac`` of the
interval toward the next leg.

State machine
-------------
``WAIT_FOR_CONNECTION -> WAIT_FOR_BALL -> PIVOT_SWEEP`` (loops forever).

  * WAIT_FOR_BALL: holds the motors off until the breakbeam reports the ball
    settled for ``ball_settle_count`` consecutive frames.
  * PIVOT_SWEEP: advances the leg target every
    ``pivot_exec_duration + pivot_hold_duration`` seconds, wrapping after a
    full cycle and repeating. If the ball is lost (breakbeam clears for
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
from ateam_motion_scenarios.common.overlays import (
    make_array,
    make_circle,
    make_line,
    make_point,
    make_pose_marker,
    make_text,
)
from ateam_motion_scenarios.common.pivot import (
    circle_leg_count,
    leg_target,
    PivotConfig,
    wrap_angle,
)
from ateam_msgs.msg import (
    FieldInfo,
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

        # Sweep maneuver: 'heading' steps the target heading around a full
        # circle (BCM_HEADING_PIVOT); 'point' pivots to face each of the four
        # field-edge centers in turn (BCM_POINT_PIVOT).
        self.pivot_mode = str(self._p('pivot_mode', 'heading')).lower()
        if self.pivot_mode not in ('heading', 'point'):
            self.get_logger().warn(
                f"Unknown pivot_mode '{self.pivot_mode}', using 'heading'")
            self.pivot_mode = 'heading'

        # Angular interval (per leg) of the circular (heading-mode) sweep.
        self.pivot_interval = math.radians(
            float(self._p('pivot_interval_deg', 45.0)))
        # Time spent pivoting toward / holding each leg target (seconds).
        self.pivot_exec_duration = float(
            self._p('pivot_exec_duration', 3.0))
        self.pivot_hold_duration = float(
            self._p('pivot_hold_duration', 1.0))

        # Continuously-sliding target. When ``pivot_slide_enabled`` is true the
        # leg's target is recomputed every tick during the exec phase, sliding
        # from the base target toward the next one so the firmware sees a
        # moving target (useful for observing how it tracks a recomputed goal).
        #   * point mode: slide the target point ``pivot_slide_distance`` (m)
        #     along the field edge.
        #   * heading mode: slide the target heading by
        #     ``pivot_slide_heading_frac`` of the leg interval (default 0.5,
        #     i.e. halfway toward the next interval).
        self.pivot_slide_enabled = bool(self._p('pivot_slide_enabled', False))
        self.pivot_slide_distance = float(
            self._p('pivot_slide_distance', 0.5))
        self.pivot_slide_heading_frac = float(
            self._p('pivot_slide_heading_frac', 0.5))

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
        self.field = None
        self.radio_connected = None
        self.breakbeam_detected = None

        self.robot_sub = self.create_subscription(
            VisionStateRobot, f'/{self.team_color}_team/robot{self.robot_id}',
            self.robot_cb, sensor_qos)
        self.field_sub = self.create_subscription(
            FieldInfo, '/field', self.field_cb, 10)
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
        # Heading mode steps around a full circle; point mode cycles the four
        # field-edge centers.
        self.num_legs = (4 if self.pivot_mode == 'point'
                         else circle_leg_count(self.pivot_interval))

        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'pivot_tune_scenario started for robot {self.robot_id} '
            f'({self.team_color}). mode={self.pivot_mode}, '
            f'slide={self.pivot_slide_enabled}, '
            f'{self.num_legs} legs/cycle, orbit_radius='
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

    def make_dribbler_hold_cmd(self) -> RobotMotionCommand:
        """Motors off but the dribbler running (to hold a placed ball)."""
        cmd = self.make_off_cmd()
        cmd.dribbler_speed = float(self.pivot.dribbler_speed)
        return cmd

    # ------------------------------------------------------------------ tick
    def tick(self):
        cmd = self.step_state_machine()
        if cmd is not None:
            self.cmd_pub.publish(cmd)
        self.publish_overlays()

    def _slide_progress(self) -> float:
        """Fraction (0..1) through the current leg's exec phase.

        Returns 0 when sliding is disabled or a leg hasn't started yet. Used
        to continuously interpolate the target while the pivot executes.
        """
        if not self.pivot_slide_enabled or self.leg_started is None \
                or self.pivot_exec_duration <= 0.0:
            return 0.0
        t = (self.get_clock().now() - self.leg_started).nanoseconds * 1e-9
        frac = t / self.pivot_exec_duration
        return max(0.0, min(1.0, frac))

    def _current_target(self):
        if self.sweep_start_heading is None:
            return None
        base = leg_target(
            self.sweep_start_heading, self.pivot_interval, self.leg_index)
        # Slide further CCW (+theta) from the base, up to `frac` of the
        # interval, while executing. Always CCW regardless of sweep sign.
        slide = (self._slide_progress() * self.pivot_slide_heading_frac
                 * abs(self.pivot_interval))
        return wrap_angle(base + slide)

    def _edge_centers(self):
        """Return the four field-edge center points, or None if no field.

        Order: +x, +y, -x, -y goalline/touchline midpoints.
        """
        if self.field is None or self.field.field_length <= 0.0 \
                or self.field.field_width <= 0.0:
            return None
        half_l = self.field.field_length / 2.0
        half_w = self.field.field_width / 2.0
        return [
            (half_l, 0.0),    # +x goalline center
            (0.0, half_w),    # +y touchline center
            (-half_l, 0.0),   # -x goalline center
            (0.0, -half_w),   # -y touchline center
        ]

    def _current_point_target(self):
        """Return the (x, y) edge target for the current leg, or None.

        With sliding enabled the point starts at the edge center and slides
        ``pivot_slide_distance`` meters along the edge toward the CCW field
        corner (CCW around the field perimeter) over the exec phase.
        """
        centers = self._edge_centers()
        if centers is None:
            return None
        idx = self.leg_index % len(centers)
        cx, cy = centers[idx]
        # Outward normal of this edge.
        if idx in (0, 2):           # goallines (constant x)
            nx, ny = (1.0 if cx > 0.0 else -1.0), 0.0
        else:                       # touchlines (constant y)
            nx, ny = 0.0, (1.0 if cy > 0.0 else -1.0)
        # CCW along-edge tangent = outward normal rotated +90 deg.
        tx, ty = -ny, nx
        off = self._slide_progress() * self.pivot_slide_distance
        return (cx + tx * off, cy + ty * off)

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
                self.leg_started = None
                self.detect_filter = 0
                self.get_logger().info(
                    'Ball detected; settling for '
                    f'{self.pivot_exec_duration:.1f}s before pivoting (start '
                    f'heading {math.degrees(self.sweep_start_heading):.1f} '
                    'deg)')
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

            # Point mode needs the field geometry to locate the edge centers.
            if self.pivot_mode == 'point' and self._edge_centers() is None:
                self.get_logger().warn(
                    'PIVOT_SWEEP (point mode): waiting for /field...',
                    throttle_duration_sec=2.0)
                return self.make_dribbler_hold_cmd()

            # Hold for one full pivot-exec duration after the ball is placed
            # so it settles before pivoting -- but run the dribbler the whole
            # time to keep the ball captured.
            if self.time_in_state() < self.pivot_exec_duration:
                return self.make_dribbler_hold_cmd()

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
                    f'{self._leg_target_desc()}')

            if self.pivot_mode == 'point':
                tx, ty = self._current_point_target()
                return self.pivot.point_command(
                    tx, ty, kick_request=RobotMotionCommand.KR_DISABLE)
            return self.pivot.heading_command(
                self._current_target(),
                kick_request=RobotMotionCommand.KR_DISABLE)

        return self.make_off_cmd()

    def _leg_target_desc(self) -> str:
        """Human-readable description of the current leg's target."""
        if self.pivot_mode == 'point':
            pt = self._current_point_target()
            if pt is None:
                return 'point ?'
            return f'point ({pt[0]:.2f}, {pt[1]:.2f})'
        tgt = self._current_target()
        if tgt is None:
            return 'heading ?'
        return f'{math.degrees(tgt):.1f} deg'

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
                if self.pivot_mode == 'point':
                    pt = self._current_point_target()
                    if pt is not None:
                        items.append(make_point(
                            OVERLAY_NS, 'target_point', pt,
                            color='#FFFF00FF', radius=0.05))
                        items.append(make_line(
                            OVERLAY_NS, 'face_line',
                            [(p.x, p.y), pt],
                            color='#FFFF00FF', stroke_width=2))
                else:
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
