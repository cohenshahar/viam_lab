"""Pure math for the `cap` verbs. No Viam imports, so it can be tested anywhere (tests/test_cap_math.py).

Frames: "room" = the lab (x, y mm, yaw deg; thesis world). "cart" = Viam's world frame, the cart centre on the
floor (decision #29). Yaw counter-clockwise from +X; the cart drives along its +X.
"""

import math
from typing import Dict, List, Sequence, Tuple


def wrap_deg(a: float) -> float:
    """Angle into (-180, 180]."""
    a = math.fmod(a + 180.0, 360.0)
    if a <= 0:
        a += 360.0
    return a - 180.0


def room_to_cart(cart: Dict[str, float], p: Sequence[float]) -> Tuple[float, float, float]:
    """Room point (x, y, z) -> cart frame."""
    yaw = math.radians(cart["yaw_deg"])
    dx, dy = p[0] - cart["x_mm"], p[1] - cart["y_mm"]
    return (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy, p[2])


def cart_to_room(cart: Dict[str, float], p: Sequence[float]) -> Tuple[float, float, float]:
    """Cart-frame point (x, y, z) -> room."""
    yaw = math.radians(cart["yaw_deg"])
    return (cart["x_mm"] + math.cos(yaw) * p[0] - math.sin(yaw) * p[1],
            cart["y_mm"] + math.sin(yaw) * p[0] + math.cos(yaw) * p[1], p[2])


def drive_to_plan(start: Dict[str, float], goal: Dict[str, float], dock_mm: float = 0.0,
                  min_move_mm: float = 5.0, dock_in_mm: float = None) -> List[Tuple[str, float]]:
    """Drive from start to goal (room poses) with turns and straight lines only.
    dock_mm = 0: turn, straight, turn (the thesis cart_utils recipe).
    dock_mm > 0: back out dock_mm along the start heading, turn, drive to a point dock_in_mm (default dock_mm) in
    front of the goal, turn to the goal heading, drive the last dock_in_mm straight in. Keeps the spins away from
    tables and stands.
    Returns [("spin", deg) | ("straight", mm), ...] without steps that are too small."""
    if math.hypot(goal["x_mm"] - start["x_mm"], goal["y_mm"] - start["y_mm"]) < min_move_mm:
        last = wrap_deg(goal["yaw_deg"] - start["yaw_deg"])
        return [("spin", last)] if abs(last) >= 0.05 else []
    steps = []
    x, y, heading = start["x_mm"], start["y_mm"], start["yaw_deg"]
    if dock_mm > 0:
        steps.append(("straight", -dock_mm))
        x -= dock_mm * math.cos(math.radians(heading))
        y -= dock_mm * math.sin(math.radians(heading))
    dock_in = dock_mm if dock_in_mm is None else dock_in_mm
    gy = math.radians(goal["yaw_deg"])
    px, py = goal["x_mm"] - dock_in * math.cos(gy), goal["y_mm"] - dock_in * math.sin(gy)
    dx, dy = px - x, py - y
    dist = math.hypot(dx, dy)
    if dist >= min_move_mm:
        turn = wrap_deg(math.degrees(math.atan2(dy, dx)) - heading)
        if abs(turn) >= 0.05:
            steps.append(("spin", turn))
        heading += turn
        steps.append(("straight", dist))
    last = wrap_deg(goal["yaw_deg"] - heading)
    if abs(last) >= 0.05:
        steps.append(("spin", last))
    if dock_in > 0:
        steps.append(("straight", dock_in))
    return steps


def sweep(start: Dict[str, float], steps: List[Tuple[str, float]], ds_mm: float = 25.0, dyaw_deg: float = 3.0):
    """Every cart pose (x, y, yaw_deg) along the steps, at most ds_mm / dyaw_deg apart."""
    x, y, h = start["x_mm"], start["y_mm"], start["yaw_deg"]
    out = [(x, y, h)]
    for kind, v in steps:
        n = max(1, int(math.ceil(abs(v) / (dyaw_deg if kind == "spin" else ds_mm))))
        for _ in range(n):
            if kind == "spin":
                h += v / n
            else:
                x += v / n * math.cos(math.radians(h))
                y += v / n * math.sin(math.radians(h))
            out.append((x, y, h))
    return out


def _rect(x, y, yaw_deg, xmin, xmax, ymin, ymax):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax))]


def _overlap(a, b):
    """Two convex polygons (lists of (x, y)) overlap? Separating axis test."""
    for poly in (a, b):
        for i in range(len(poly)):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
            nx, ny = y2 - y1, x1 - x2
            pa = [nx * px + ny * py for px, py in a]
            pb = [nx * px + ny * py for px, py in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def first_hit(poses, footprint, obstacles):
    """First obstacle the cart outline touches along poses, or None.
    footprint: (xmin, xmax, ymin, ymax) in the cart frame; obstacles: {name: 2D polygon in the room}."""
    for x, y, h in poses:
        cart = _rect(x, y, h, *footprint)
        for name, poly in obstacles.items():
            if _overlap(cart, poly):
                return name, (round(x), round(y), round(h, 1))
    return None


def arc_plan(radius_mm: float, angle_deg: float, speed_mm_s: float) -> Tuple[float, float, float]:
    """Drive an arc: radius (mm, > 0 = centre on the left), angle (deg, > 0 = forward), speed (mm/s, > 0).
    Returns (forward mm/s, turn deg/s, seconds) for set_velocity."""
    if radius_mm == 0 or speed_mm_s <= 0 or angle_deg == 0:
        raise ValueError("drive_arc needs a non-zero radius, a positive speed and a non-zero angle")
    v = math.copysign(speed_mm_s, angle_deg)
    w = math.degrees(v / radius_mm)
    length = abs(radius_mm) * math.radians(abs(angle_deg))
    return v, w, length / speed_mm_s


def unit(v: Sequence[float]) -> Tuple[float, float, float]:
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-12:
        raise ValueError("zero-length direction")
    return tuple(c / n for c in v)


def along(p: Sequence[float], d: Sequence[float], dist: float) -> Tuple[float, float, float]:
    """p + dist * d."""
    return tuple(p[i] + dist * d[i] for i in range(3))


def facing_face(box_centre_cart: Sequence[float], box_yaw_cart_deg: float, size_mm: Sequence[float]):
    """The side face of a box that faces the cart (whose outward normal points most toward the cart origin).
    Returns (face centre, tool direction = into the box), both in the cart frame."""
    yaw = math.radians(box_yaw_cart_deg)
    ux, uy = (math.cos(yaw), math.sin(yaw), 0.0), (-math.sin(yaw), math.cos(yaw), 0.0)
    to_cart = unit((-box_centre_cart[0], -box_centre_cart[1], 0.0))
    best = None
    for n, half in ((ux, size_mm[0] / 2), ((-ux[0], -ux[1], 0.0), size_mm[0] / 2),
                    (uy, size_mm[1] / 2), ((-uy[0], -uy[1], 0.0), size_mm[1] / 2)):
        score = n[0] * to_cart[0] + n[1] * to_cart[1]
        if best is None or score > best[0]:
            best = (score, n, half)
    _, n, half = best
    face = along(box_centre_cart, n, half)
    return face, (-n[0], -n[1], 0.0)
