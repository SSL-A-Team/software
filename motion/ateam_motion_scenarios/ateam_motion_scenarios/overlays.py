"""
Overlay helpers for motion scenarios.

Builds ``ateam_msgs/Overlay`` messages for publishing on ``/overlays``
so scenario state shows up in the UI alongside Kenobi visualizations.

Each scenario should pick a unique namespace (the node name) so its
overlays don't collide with Kenobi's. Use REPLACE for live-updating
overlays so the same name simply re-renders each tick. Overlays
default to a short lifetime so they auto-clear when the scenario
stops publishing.
"""

import math

from ateam_msgs.msg import Overlay, OverlayArray
from geometry_msgs.msg import Point, Vector3


# Default lifetime in milliseconds. Long enough to survive a few missed
# ticks at 60 Hz, short enough that overlays disappear quickly when the
# scenario stops publishing them.
DEFAULT_LIFETIME_MS = 200


def _base(ns: str, name: str,
          lifetime_ms: int = DEFAULT_LIFETIME_MS,
          depth: int = 1) -> Overlay:
    msg = Overlay()
    msg.ns = ns
    msg.name = name
    msg.visible = True
    msg.command = Overlay.REPLACE
    msg.lifetime = int(lifetime_ms)
    msg.depth = int(depth)
    return msg


def make_line(ns: str, name: str, points,
              color: str = '#FFFFFFFF',
              stroke_width: int = 3,
              lifetime_ms: int = DEFAULT_LIFETIME_MS) -> Overlay:
    """Build a LINE overlay through the given (x, y) points."""
    msg = _base(ns, name, lifetime_ms)
    msg.type = Overlay.LINE
    msg.position = Point()
    msg.stroke_color = color
    msg.stroke_width = int(stroke_width)
    for x, y in points:
        msg.points.append(Point(x=float(x), y=float(y), z=0.0))
    return msg


def make_circle(ns: str, name: str, center, radius: float,
                stroke_color: str = '#FFFFFFFF',
                fill_color: str = '#00000000',
                stroke_width: int = 2,
                lifetime_ms: int = DEFAULT_LIFETIME_MS) -> Overlay:
    """Build an ELLIPSE overlay rendered as a circle."""
    cx, cy = center
    msg = _base(ns, name, lifetime_ms)
    msg.type = Overlay.ELLIPSE
    msg.position = Point(x=float(cx), y=float(cy), z=0.0)
    d = 2.0 * float(radius)
    msg.scale = Point(x=d, y=d, z=0.0)
    msg.stroke_color = stroke_color
    msg.fill_color = fill_color
    msg.stroke_width = int(stroke_width)
    return msg


def make_point(ns: str, name: str, position,
               color: str = '#FFFFFFFF',
               radius: float = 0.04,
               lifetime_ms: int = DEFAULT_LIFETIME_MS) -> Overlay:
    """Build a small filled circle for marking a point of interest."""
    fill = color
    return make_circle(
        ns, name, position, radius,
        stroke_color=color,
        fill_color=fill,
        stroke_width=1,
        lifetime_ms=lifetime_ms,
    )


def make_text(ns: str, name: str, text: str, position,
              color: str = '#FFFFFFFF',
              font_size: int = 24,
              lifetime_ms: int = DEFAULT_LIFETIME_MS) -> Overlay:
    """Build a TEXT overlay anchored at ``position``."""
    px, py = position
    msg = _base(ns, name, lifetime_ms)
    msg.type = Overlay.TEXT
    msg.position = Point(x=float(px), y=float(py), z=0.0)
    msg.fill_color = color
    msg.stroke_width = int(font_size)
    msg.text = str(text)
    return msg


def make_arrows(ns: str, name: str, arrows,
                color: str = '#FFFFFFFF',
                stroke_width: int = 3,
                lifetime_ms: int = DEFAULT_LIFETIME_MS) -> Overlay:
    """
    Build an ARROWS overlay.

    ``arrows`` is an iterable of ((origin_x, origin_y), (vec_x, vec_y))
    pairs. The arrow renders from origin in the direction/length of vec.
    """
    msg = _base(ns, name, lifetime_ms)
    msg.type = Overlay.ARROWS
    msg.position = Point()
    msg.stroke_color = color
    msg.stroke_width = int(stroke_width)
    for (ox, oy), (vx, vy) in arrows:
        msg.points.append(Point(x=float(ox), y=float(oy), z=0.0))
        msg.scales.append(Vector3(x=float(vx), y=float(vy), z=0.0))
    return msg


def make_pose_marker(ns: str, name: str, pose,
                     radius: float,
                     color: str = '#FFFFFFFF',
                     stroke_width: int = 2,
                     heading_length: float = None,
                     lifetime_ms: int = DEFAULT_LIFETIME_MS):
    """
    Build a (circle, heading-arrow) pair for visualizing a target pose.

    ``pose`` is ``(x, y, theta)``. The circle has the given radius
    (typically the position tolerance). The heading arrow points along
    theta with length ``heading_length`` (default = 2 * radius).

    Returns a list of Overlay messages.
    """
    x, y, theta = pose
    if heading_length is None:
        heading_length = max(2.0 * radius, 0.1)
    arrow_vec = (heading_length * math.cos(theta),
                 heading_length * math.sin(theta))
    return [
        make_circle(ns, name, (x, y), radius,
                    stroke_color=color, fill_color='#00000000',
                    stroke_width=stroke_width, lifetime_ms=lifetime_ms),
        make_arrows(ns, name + '_heading', [((x, y), arrow_vec)],
                    color=color, stroke_width=stroke_width,
                    lifetime_ms=lifetime_ms),
    ]


def make_array(*overlays) -> OverlayArray:
    """
    Pack a flat sequence of overlays into a single ``OverlayArray``.

    Nested lists (e.g. from ``make_pose_marker``) are flattened.
    ``None`` values are skipped so callers can conditionally include
    overlays without scattering ``if`` statements at the call site.
    """
    msg = OverlayArray()
    _extend(msg.overlays, overlays)
    return msg


def _extend(out, items):
    for item in items:
        if item is None:
            continue
        if isinstance(item, Overlay):
            out.append(item)
        else:
            _extend(out, item)
