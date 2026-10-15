"""Tests for the sim cart math. Run: python3 20_sim/modules/capbot_sim/tests/test_drive_math.py"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from drive_math import Pose, odom_step, step, wheel_speeds, wrap_deg  # noqa: E402


def drive(p, v, w, seconds, dt=0.05):
    t = 0.0
    while t < seconds - 1e-12:
        h = min(dt, seconds - t)
        p = step(p, v, w, h)
        t += h
    return p


def test_straight():
    p = drive(Pose(), 200, 0, 5)
    assert abs(p.x_mm - 1000) < 1e-9 and abs(p.y_mm) < 1e-9


def test_square_closes():
    p = Pose()
    for _ in range(4):
        p = drive(p, 250, 0, 4)       # 1 m
        p = drive(p, 0, 45, 2)        # 90 deg left
    assert math.hypot(p.x_mm, p.y_mm) < 1e-6, p
    assert abs(wrap_deg(p.yaw_deg)) < 1e-6, p


def test_arc_quarter_circle():
    # v / w = radius 1000 mm, quarter turn ends at (1000, 1000) facing +Y
    p = drive(Pose(), 1000 * math.radians(30), 30, 3)
    assert abs(p.x_mm - 1000) < 1e-6 and abs(p.y_mm - 1000) < 1e-6 and abs(p.yaw_deg - 90) < 1e-9


def test_wrap():
    assert wrap_deg(190) == -170 and wrap_deg(-180) == 180 and wrap_deg(540) == 180


def test_odom_arc_matches_unicycle():
    # wheels rolled for a 1000 mm-radius quarter circle, in one step and in 50 steps
    track, r, quarter = 478.0, 1000.0, math.pi / 2
    dl, dr = (r - track / 2) * quarter, (r + track / 2) * quarter
    one = odom_step(Pose(), dl, dr, track)
    many = Pose()
    for _ in range(50):
        many = odom_step(many, dl / 50, dr / 50, track)
    for p in (one, many):
        assert abs(p.x_mm - 1000) < 1e-6 and abs(p.y_mm - 1000) < 1e-6 and abs(p.yaw_deg - 90) < 1e-9, p


def test_odom_spin_in_place():
    p = odom_step(Pose(5, 6, 0), -100.0, 100.0, 200.0)   # right wheel forward = turn left
    assert abs(p.x_mm - 5) < 1e-9 and abs(p.y_mm - 6) < 1e-9 and abs(p.yaw_deg - math.degrees(1.0)) < 1e-9


def test_wheels():
    left, right = wheel_speeds(0, 90, 478)
    assert abs(left + right) < 1e-9 and right > 0


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
