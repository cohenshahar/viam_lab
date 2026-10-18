"""Tests for the cap verbs' pure math (no viam-server). Run: python3 20_sim/modules/capbot_sim/tests/test_cap_math.py
(also works with pytest)."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cap_math import along, arc_plan, cart_to_room, drive_to_plan, facing_face, first_hit, room_to_cart, sweep, wrap_deg  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from drive_math import Pose, step  # noqa: E402


def run_plan(start, steps):
    p = Pose(start["x_mm"], start["y_mm"], start["yaw_deg"])
    for kind, v in steps:
        p = step(p, v, 0, 1) if kind == "straight" else step(p, 0, v, 1)
    return p


def test_room_cart_round_trip():
    cart = {"x_mm": 1234.0, "y_mm": -567.0, "yaw_deg": 137.0}
    p = (100.0, 2000.0, 55.0)
    back = cart_to_room(cart, room_to_cart(cart, p))
    assert math.dist(back, p) < 1e-9


def test_room_to_cart_facing_table():
    # cart facing +Y: a point 1 m further along +Y is 1 m straight ahead (cart +X)
    c = room_to_cart({"x_mm": 2000, "y_mm": -50, "yaw_deg": 90}, (2000, 950, 0))
    assert abs(c[0] - 1000) < 1e-9 and abs(c[1]) < 1e-9


def test_drive_to_reaches_goal():
    for start, goal in [({"x_mm": 0, "y_mm": 0, "yaw_deg": 0}, {"x_mm": 2000, "y_mm": -50, "yaw_deg": 90}),
                        ({"x_mm": 2000, "y_mm": -50, "yaw_deg": 90}, {"x_mm": 3300, "y_mm": -50, "yaw_deg": 90}),
                        ({"x_mm": 3300, "y_mm": -50, "yaw_deg": 90}, {"x_mm": 0, "y_mm": 0, "yaw_deg": 0})]:
        steps = drive_to_plan(start, goal)
        p = run_plan(start, steps)
        assert math.hypot(p.x_mm - goal["x_mm"], p.y_mm - goal["y_mm"]) < 1e-6, (steps, p)
        assert abs(wrap_deg(p.yaw_deg - goal["yaw_deg"])) < 1e-6, (steps, p)
        assert all(abs(v) <= 180 for k, v in steps if k == "spin")


def test_drive_to_with_docking_reaches_goal():
    start, goal = {"x_mm": 1900, "y_mm": 90, "yaw_deg": 90}, {"x_mm": -1500, "y_mm": 1480, "yaw_deg": 90}
    steps = drive_to_plan(start, goal, dock_mm=250)
    assert steps[0] == ("straight", -250) and steps[-1] == ("straight", 250), steps
    p = run_plan(start, steps)
    assert math.hypot(p.x_mm - goal["x_mm"], p.y_mm - goal["y_mm"]) < 1e-6 and abs(wrap_deg(p.yaw_deg - 90)) < 1e-6


def test_sweep_and_hit():
    table = {"table": [(1770, 670), (3430, 670), (3430, 1530), (1770, 1530)]}
    fp = (-570, 518, -350, 350)
    start = {"x_mm": 1900, "y_mm": 90, "yaw_deg": 90}
    # spinning in place right at the table: the cart's corners sweep into it
    assert first_hit(sweep(start, [("spin", 90)]), fp, table) is not None
    # backing out 250 mm first: the spin is clear
    assert first_hit(sweep(start, [("straight", -250), ("spin", 90)]), fp, table) is None
    poses = sweep(start, [("straight", 100)])
    assert len(poses) == 5 and abs(poses[-1][1] - 190) < 1e-9


def test_drive_to_nothing_to_do():
    assert drive_to_plan({"x_mm": 5, "y_mm": 5, "yaw_deg": 10}, {"x_mm": 5, "y_mm": 5, "yaw_deg": 10}) == []


def test_arc():
    v, w, t = arc_plan(1000, 90, 250)
    assert v == 250 and abs(w - math.degrees(0.25)) < 1e-9 and abs(t - math.pi * 1000 / 2 / 250) < 1e-9
    p = step(Pose(), v, w, t)
    assert abs(p.x_mm - 1000) < 1e-6 and abs(p.y_mm - 1000) < 1e-6 and abs(p.yaw_deg - 90) < 1e-6
    try:
        arc_plan(0, 90, 250)
        raise AssertionError("accepted radius 0")
    except ValueError:
        pass


def test_facing_face():
    # box 1 m ahead of the cart, 120 mm cube: the face toward the cart is at x = 940, tool points +X
    face, d = facing_face((1000, 0, 1060), 0, (120, 120, 120))
    assert math.dist(face, (940, 0, 1060)) < 1e-9 and math.dist(d, (1, 0, 0)) < 1e-9
    # box turned 90 deg: still the face toward the cart
    face, d = facing_face((1000, 0, 1060), 90, (120, 120, 120))
    assert math.dist(face, (940, 0, 1060)) < 1e-9
    assert math.dist(along(face, d, 60), (1000, 0, 1060)) < 1e-9


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
    print("test_cap_math PASS")
