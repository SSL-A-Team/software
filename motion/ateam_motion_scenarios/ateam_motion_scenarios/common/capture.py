"""
Shared ball-capture skill for motion scenarios.

Mirrors the kenobi ``Capture`` skill (ateam_kenobi/src/skills/capture.cpp)
as a small, reusable, ROS-agnostic state machine that any scenario node in
``ateam_motion_scenarios`` can drive. It emits ``RobotMotionCommand``
messages using global commands only:

  1. APPROACH -- a ``BCM_GLOBAL_POSITION`` move to a staging point
     ``approach_radius`` from the ball, on the robot->ball line, facing the
     ball. Completes once the robot is within ``approach_pos_tol`` /
     ``approach_yaw_tol`` of that pose AND its speed is below
     ``capture_speed``.
  2. CAPTURE -- a constantly-updating ``BCM_GLOBAL_POSITION`` straight at
     the ball, oriented along the robot-center -> ball-center line, with the
     velocity limited to ``capture_speed`` and the acceleration to
     ``capture_accel``. Completes when the ball is settled on the dribbler.

Completion uses the firmware breakbeam (``BasicTelemetry.breakbeam_ball_
detected``, passed in by the node) when available, mirroring the kenobi
30-frame filter (+1 detected, -2 not). When no breakbeam value is available
it falls back to a vision proxy: the ball within ``robot_front_offset +
dribbler_tol``, or occluded by the robot.

Usage from a scenario node::

    from ateam_motion_scenarios.common.capture import Capture, CaptureConfig

    self.capture = Capture(CaptureConfig.from_params(self._p))
    # each tick, while capturing:
    cmd = self.capture.run_frame(
        robot_pose, robot_speed, ball_xy, ball_visible, breakbeam)
    if self.capture.is_done():
        ...  # capture complete
    else:
        publish(cmd)

The node owns all ROS plumbing (subscriptions, the breakbeam topic); this
module is pure logic, like ``overlays.py``.
"""

from dataclasses import dataclass
from enum import auto, Enum
import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from ateam_msgs.msg import RobotMotionCommand, Twist2D


# Shared capture parameters used by every scenario. Individual scenarios (or
# the command line) can still override any value via their own ROS params.
CAPTURE_PARAM_FILE = os.path.join(
    get_package_share_directory('ateam_motion_scenarios'),
    'config', 'skill_capture_params.json')

# Built-in fallback defaults (kenobi Capture skill), used when the shared
# JSON file is missing or a key is absent from it.
_BUILTIN_DEFAULTS = {
    'robot_front_offset': 0.09,
    'approach_radius': 0.25,
    'approach_pos_tol': 0.05,
    'approach_yaw_tol_deg': 5.0,
    'capture_speed': 0.3,
    'capture_accel': 1.0,
    'capture_filter_count': 30,
    'use_breakbeam': True,
    'breakbeam_topic': '',
    'capture_dribbler_tol': 0.03,
}


def load_capture_defaults(param_file: str = None) -> dict:
    """
    Load the shared capture parameter defaults from the JSON file.

    Returns a dict merged over the built-in defaults so missing keys always
    resolve. Falls back to the built-ins if the file cannot be read.
    """
    defaults = dict(_BUILTIN_DEFAULTS)
    path = param_file or CAPTURE_PARAM_FILE
    try:
        with open(path) as f:
            defaults.update(json.load(f))
    except (OSError, ValueError):
        pass
    return defaults


def _ang_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


@dataclass
class CaptureConfig:
    """Parameters for the :class:`Capture` skill (kenobi defaults)."""

    approach_radius: float = 0.25       # m, staging distance from the ball
    approach_pos_tol: float = 0.05      # m, staging arrival tolerance
    approach_yaw_tol: float = math.radians(5.0)  # rad, facing tolerance
    capture_speed: float = 0.3          # m/s, capture creep + arrival gate
    capture_accel: float = 1.0          # m/s^2, capture accel limit
    robot_front_offset: float = 0.09    # m, dribbler distance ahead of center
    filter_count: int = 30              # frames settled before "done"
    dribbler_tol: float = 0.03          # m, vision-proxy dribbler tolerance
    use_breakbeam: bool = True          # prefer firmware breakbeam when given
    dribbler_speed: float = 300.0       # rpm
    kick_request: int = RobotMotionCommand.KR_ARM  # held during capture
    # Approach-phase motion limits (0.0 => firmware default).
    approach_vel_limit: float = 0.0     # m/s
    approach_acc_limit: float = 0.0     # m/s^2
    # Angular limits applied to every capture command (0.0 => firmware).
    limit_vel_angular: float = 0.0      # rad/s
    limit_acc_angular: float = 0.0      # rad/s^2

    @staticmethod
    def from_params(p) -> 'CaptureConfig':
        """
        Build a config from a ``p(name, default)`` parameter getter.

        ``p`` is any callable that returns the resolved value for a named
        scenario parameter (e.g. a node's JSON-backed ``_p`` helper). The
        defaults come from the shared ``config/skill_capture_params.json`` so every
        scenario uses the same capture tuning unless it explicitly overrides a
        value via its own ROS params / command line.
        """
        d = load_capture_defaults()
        return CaptureConfig(
            approach_radius=float(
                p('approach_radius', d['approach_radius'])),
            approach_pos_tol=float(
                p('approach_pos_tol', d['approach_pos_tol'])),
            approach_yaw_tol=math.radians(
                float(p('approach_yaw_tol_deg', d['approach_yaw_tol_deg']))),
            capture_speed=float(p('capture_speed', d['capture_speed'])),
            capture_accel=float(p('capture_accel', d['capture_accel'])),
            robot_front_offset=float(
                p('robot_front_offset', d['robot_front_offset'])),
            filter_count=int(
                p('capture_filter_count', d['capture_filter_count'])),
            dribbler_tol=float(
                p('capture_dribbler_tol', d['capture_dribbler_tol'])),
            use_breakbeam=bool(p('use_breakbeam', d['use_breakbeam'])),
            dribbler_speed=float(p('dribbler_speed', 300.0)),
            approach_vel_limit=float(p('limit_vel_linear', 0.0)),
            approach_acc_limit=float(p('limit_acc_linear', 0.0)),
            limit_vel_angular=float(p('limit_vel_angular', 0.0)),
            limit_acc_angular=float(p('limit_acc_angular', 0.0)),
        )


class CapturePhase(Enum):
    APPROACH = auto()
    CAPTURE = auto()
    DONE = auto()


class Capture:
    """
    Two-phase ball-capture skill emitting global-position commands.

    Stateless with respect to ROS: the driving node feeds the robot pose,
    speed, ball position, ball visibility and (optional) breakbeam state
    each frame, and receives a ``RobotMotionCommand`` back.
    """

    def __init__(self, config: CaptureConfig = None):
        self.config = config or CaptureConfig()
        self._phase = CapturePhase.APPROACH
        self._filter = 0

    def reset(self):
        """Restart the skill at the APPROACH phase."""
        self._phase = CapturePhase.APPROACH
        self._filter = 0

    @property
    def phase(self) -> CapturePhase:
        return self._phase

    def is_done(self) -> bool:
        return self._phase == CapturePhase.DONE

    def staging_target(self, robot_pose, ball_xy):
        """
        Return the approach staging pose ``(x, y, theta)`` for overlays.

        ``robot_pose`` is ``(x, y, yaw)`` and ``ball_xy`` is ``(x, y)``;
        returns ``None`` if either is missing.
        """
        if robot_pose is None or ball_xy is None:
            return None
        return self._staging(robot_pose, ball_xy)

    def run_frame(self, robot_pose, robot_speed, ball_xy,
                  ball_visible=True, breakbeam_detected=None):
        """
        Advance the skill one tick and return a ``RobotMotionCommand``.

        ``robot_pose`` is ``(x, y, yaw)`` (or ``None``); ``robot_speed`` is
        the robot linear speed in m/s; ``ball_xy`` is ``(x, y)`` of the ball
        (live, or latched if occluded, or ``None``); ``ball_visible`` is
        whether the ball is currently visible; ``breakbeam_detected`` is the
        firmware breakbeam state (bool) or ``None`` when unavailable.

        Returns ``None`` when no command should be sent (missing inputs, or
        the skill has just completed -- check :meth:`is_done`).
        """
        c = self.config
        if robot_pose is None or ball_xy is None:
            return None
        rx, ry, ryaw = robot_pose

        if self._phase == CapturePhase.APPROACH:
            tx, ty, theta = self._staging(robot_pose, ball_xy)
            pos_ok = math.hypot(rx - tx, ry - ty) <= c.approach_pos_tol
            yaw_ok = abs(_ang_diff(ryaw, theta)) <= c.approach_yaw_tol
            if pos_ok and yaw_ok and robot_speed < c.capture_speed:
                self._phase = CapturePhase.CAPTURE
                self._filter = 0
            return self._make_pos_cmd(
                tx, ty, theta, c.approach_vel_limit, c.approach_acc_limit)

        if self._phase == CapturePhase.CAPTURE:
            if c.use_breakbeam and breakbeam_detected is not None:
                detected = breakbeam_detected
            else:
                on_dribbler = math.hypot(ball_xy[0] - rx, ball_xy[1] - ry) \
                    <= c.robot_front_offset + c.dribbler_tol
                detected = on_dribbler or not ball_visible
            if detected:
                self._filter += 1
            else:
                self._filter = max(0, self._filter - 2)
            if self._filter >= c.filter_count:
                self._phase = CapturePhase.DONE
                return None
            theta = math.atan2(ball_xy[1] - ry, ball_xy[0] - rx)
            return self._make_pos_cmd(
                ball_xy[0], ball_xy[1], theta,
                c.capture_speed, c.capture_accel)

        return None

    # ------------------------------------------------------------- internal
    def _staging(self, robot_pose, ball_xy):
        rx, ry, _ = robot_pose
        dx = rx - ball_xy[0]
        dy = ry - ball_xy[1]
        d = math.hypot(dx, dy)
        if d < 1e-6:
            ux, uy = -1.0, 0.0
        else:
            ux, uy = dx / d, dy / d
        tx = ball_xy[0] + ux * self.config.approach_radius
        ty = ball_xy[1] + uy * self.config.approach_radius
        theta = math.atan2(ball_xy[1] - ry, ball_xy[0] - rx)
        return tx, ty, theta

    def _make_pos_cmd(self, x, y, theta, vel_limit, acc_limit):
        c = self.config
        cmd = RobotMotionCommand()
        cmd.body_control_mode = RobotMotionCommand.BCM_GLOBAL_POSITION
        cmd.pose = Twist2D(x=float(x), y=float(y), theta=float(theta))
        cmd.velocity = Twist2D()
        cmd.acceleration = Twist2D()
        if vel_limit > 0.0:
            cmd.limit_vel_linear = float(vel_limit)
        if acc_limit > 0.0:
            cmd.limit_acc_linear = float(acc_limit)
        if c.limit_vel_angular > 0.0:
            cmd.limit_vel_angular = float(c.limit_vel_angular)
        if c.limit_acc_angular > 0.0:
            cmd.limit_acc_angular = float(c.limit_acc_angular)
        cmd.kick_request = c.kick_request
        cmd.dribbler_speed = float(c.dribbler_speed)
        return cmd
