"""Pure diff-drive math for the sim cart. No Viam imports, so it can be tested anywhere.

Pose: x_mm, y_mm in the world; yaw_deg counter-clockwise from +X. The cart drives
along its own +X (thesis convention). Viam's base API speaks in "forward" (mm/s)
and "turn left" (deg/s), so nothing here depends on which axis Viam calls forward.
"""

import math
from dataclasses import dataclass


@dataclass
class Pose:
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_deg: float = 0.0


def wrap_deg(a):
    """Angle into (-180, 180]."""
    a = math.fmod(a + 180.0, 360.0)
    if a <= 0:
        a += 360.0
    return a - 180.0


def step(p: Pose, v_mm_s: float, w_deg_s: float, dt_s: float) -> Pose:
    """Exact unicycle motion for one time step (straight line or circular arc)."""
    th = math.radians(p.yaw_deg)
    w = math.radians(w_deg_s)
    if abs(w) < 1e-12:
        x = p.x_mm + v_mm_s * dt_s * math.cos(th)
        y = p.y_mm + v_mm_s * dt_s * math.sin(th)
    else:
        r = v_mm_s / w
        th2 = th + w * dt_s
        x = p.x_mm + r * (math.sin(th2) - math.sin(th))
        y = p.y_mm - r * (math.cos(th2) - math.cos(th))
    return Pose(x, y, wrap_deg(p.yaw_deg + w_deg_s * dt_s))


def odom_step(p: Pose, d_left_mm: float, d_right_mm: float, track_mm: float) -> Pose:
    """Diff-drive odometry: how far each wheel rolled (mm) since the last step -> new pose.
    Forward = the mean of the two wheels; turn (left +) = (right - left) / track.
    Treats the step as one arc, so it is exact while both wheels keep a steady speed."""
    turn_deg = math.degrees((d_right_mm - d_left_mm) / track_mm)
    return step(p, (d_left_mm + d_right_mm) / 2.0, turn_deg, 1.0)


def wheel_speeds(v_mm_s, w_deg_s, track_mm):
    """Left / right wheel ground speeds (mm/s) for a body speed and turn rate."""
    half = track_mm / 2.0 * math.radians(w_deg_s)
    return v_mm_s - half, v_mm_s + half


def clamp(v, limit):
    return max(-limit, min(limit, v))
