"""
Shared firmware-pivot (BCM_PIVOT) helpers for motion scenarios.

The firmware pivot maneuver orbits the robot around a center it derives from
the robot pose, ``orbit_radius`` and ``inset_angle``, rotating the robot's
heading to a commanded global ``target_heading``. This module provides a pure
command builder plus a small config/sequencing helper so scenarios don't have
to duplicate the BCM_PIVOT wiring (mirrors how ``capture.py`` is shared).
"""

from dataclasses import dataclass
import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from ateam_msgs.msg import RobotMotionCommand, Twist2D


# Shared firmware-pivot tuning used by every scenario that pivots. Individual
# scenarios (or the command line) can still override any value via their own
# ROS params.
PIVOT_PARAM_FILE = os.path.join(
    get_package_share_directory('ateam_motion_scenarios'),
    'config', 'skill_pivot_params.json')

# Built-in fallback defaults (firmware PivotParams::default()), used when the
# shared JSON file is missing or a key is absent from it.
_BUILTIN_DEFAULTS = {
    'orbit_radius': 0.2,
    'inset_angle': 0.5,
    'max_angular_vel': math.pi,
    'max_angular_acc': 2.0 * math.pi,
    'dribbler_speed': 300.0,
}


def load_pivot_defaults(param_file: str = None) -> dict:
    """
    Load the shared firmware-pivot parameter defaults from the JSON file.

    Returns a dict merged over the built-in defaults so missing keys always
    resolve. Falls back to the built-ins if the file cannot be read.
    """
    defaults = dict(_BUILTIN_DEFAULTS)
    path = param_file or PIVOT_PARAM_FILE
    try:
        with open(path) as f:
            defaults.update(json.load(f))
    except (OSError, ValueError):
        pass
    return defaults


def make_pivot_cmd(target_heading, orbit_radius, inset_angle=0.0,
                   ang_vel_limit=0.0, ang_acc_limit=0.0,
                   kick_request=RobotMotionCommand.KR_DISABLE,
                   dribbler_speed=0.0, kick_speed=0.0) -> RobotMotionCommand:
    """
    Build a firmware ``BCM_PIVOT`` command.

    Only the target heading is commanded (via ``pose.theta``); the firmware
    derives the orbit center from the robot pose, ``orbit_radius`` and
    ``inset_angle``. Angular limits of ``0.0`` mean "use the firmware
    default".
    """
    cmd = RobotMotionCommand()
    cmd.body_control_mode = RobotMotionCommand.BCM_PIVOT
    cmd.pose = Twist2D(x=0.0, y=0.0, theta=float(target_heading))
    cmd.velocity = Twist2D()
    cmd.acceleration = Twist2D()
    if ang_vel_limit > 0.0:
        cmd.limit_vel_angular = float(ang_vel_limit)
    if ang_acc_limit > 0.0:
        cmd.limit_acc_angular = float(ang_acc_limit)
    cmd.pivot_orbit_radius = float(orbit_radius)
    cmd.pivot_inset_angle = float(inset_angle)
    cmd.kick_request = kick_request
    cmd.kick_speed = float(kick_speed)
    cmd.dribbler_speed = float(dribbler_speed)
    return cmd


def wrap_angle(a: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    a = math.fmod(a, 2.0 * math.pi)
    if a > math.pi:
        a -= 2.0 * math.pi
    elif a <= -math.pi:
        a += 2.0 * math.pi
    return a


def circle_leg_count(interval: float) -> int:
    """Number of legs to sweep a full circle at ``interval`` radians/leg."""
    if interval <= 1e-6:
        return 1
    return max(1, int(round(2.0 * math.pi / interval)))


def leg_target(start_heading: float, interval: float, leg_index: int) -> float:
    """
    Absolute target heading for ``leg_index`` of a circular sweep.

    Mirrors the hwtest-pivot sequencer: leg ``i`` targets
    ``start_heading + interval * (i + 1)``, wrapped to ``(-pi, pi]``.
    """
    return wrap_angle(start_heading + interval * (leg_index + 1))


@dataclass
class PivotConfig:
    """Firmware-pivot tuning parameters (firmware PivotParams defaults)."""

    orbit_radius: float = 0.2
    inset_angle: float = 0.5
    max_angular_vel: float = math.pi
    max_angular_acc: float = 2.0 * math.pi
    dribbler_speed: float = 300.0

    @staticmethod
    def from_params(p, prefix: str = 'pivot_') -> 'PivotConfig':
        """
        Build a config from a ``p(name, default)`` parameter getter.

        The defaults come from the shared ``config/skill_pivot_params.json`` so every
        scenario uses the same firmware-pivot tuning unless it explicitly
        overrides a value via its own ROS params / command line.
        """
        d = load_pivot_defaults()
        return PivotConfig(
            orbit_radius=float(p(prefix + 'orbit_radius', d['orbit_radius'])),
            inset_angle=float(p(prefix + 'inset_angle', d['inset_angle'])),
            max_angular_vel=float(
                p(prefix + 'max_angular_vel', d['max_angular_vel'])),
            max_angular_acc=float(
                p(prefix + 'max_angular_acc', d['max_angular_acc'])),
            dribbler_speed=float(
                p(prefix + 'dribbler_speed', d['dribbler_speed'])),
        )

    def command(self, target_heading,
                kick_request=RobotMotionCommand.KR_DISABLE,
                kick_speed=0.0) -> RobotMotionCommand:
        """Build a BCM_PIVOT command for ``target_heading`` using this config."""
        return make_pivot_cmd(
            target_heading, self.orbit_radius, self.inset_angle,
            self.max_angular_vel, self.max_angular_acc,
            kick_request=kick_request, dribbler_speed=self.dribbler_speed,
            kick_speed=kick_speed)
