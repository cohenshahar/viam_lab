"""Test capbot_common.room_in_cart (no viam-server needed).

Run: python3 20_sim/scripts/test_room_frame.py
"""

import math

from capbot_common import numbers, room_in_cart, room_world_state

OBST = numbers()["world"]["obstacles"]


def box(boxes, label):
    return next(b for b in boxes if b["label"] == label)


def test_at_origin_nothing_moves():
    b = box(room_in_cart(OBST, {"x_mm": 0, "y_mm": 0, "yaw_deg": 0}), "lab-table")
    pb = OBST["lab-table"]["planning_box_mm"]
    want = [(pb["min"][i] + pb["max"][i]) / 2 for i in range(3)]
    assert all(abs(b["centre_mm"][i] - want[i]) < 1e-9 for i in range(3)), b
    assert b["yaw_deg"] == 0


def test_at_the_table_facing_it():
    # cart 650 mm in front of the table edge, facing +Y (as world_check does): the table is straight ahead
    pb = OBST["lab-table"]["planning_box_mm"]
    tx = (pb["min"][0] + pb["max"][0]) / 2
    ty = (pb["min"][1] + pb["max"][1]) / 2
    cart = {"x_mm": tx, "y_mm": pb["min"][1] - 650, "yaw_deg": 90}
    b = box(room_in_cart(OBST, cart), "lab-table")
    assert abs(b["centre_mm"][0] - (ty - cart["y_mm"])) < 1e-9 and abs(b["centre_mm"][1]) < 1e-9, b
    assert b["yaw_deg"] == -90


def test_distances_kept():
    cart = {"x_mm": -1234, "y_mm": 567, "yaw_deg": -137}
    a, b = box(room_in_cart(OBST, cart), "lab-table"), box(room_in_cart(OBST, cart), "lab-cabinet")
    room = [((o["planning_box_mm"]["min"][0] + o["planning_box_mm"]["max"][0]) / 2,
             (o["planning_box_mm"]["min"][1] + o["planning_box_mm"]["max"][1]) / 2)
            for o in (OBST["lab-table"], OBST["lab-cabinet"])]
    assert abs(math.dist(a["centre_mm"][:2], b["centre_mm"][:2]) - math.dist(*room)) < 1e-9


def test_world_state_builds():
    ws = room_world_state(OBST, {"x_mm": 100, "y_mm": 0, "yaw_deg": 30})
    geoms = ws.obstacles[0].geometries
    assert ws.obstacles[0].reference_frame == "world" and len(geoms) == len(OBST)
    assert {g.label for g in geoms} == set(OBST)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
    print("test_room_frame PASS")
