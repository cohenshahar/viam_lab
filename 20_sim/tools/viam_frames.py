"""Rotation helpers that follow Viam's conventions.

Viam writes an orientation as an "orientation vector" (OV): the direction the
frame's +Z axis points (ox, oy, oz) plus a turn theta about it. Viam turns an OV
into a rotation as Rz(lon) * Ry(lat) * Rz(theta), with lat = acos(oz) and
lon = atan2(oy, ox) (rdk spatialmath/orientationVector.go). At the poles
(+Z straight up or down) lon is set to 0 here, so the numbers we write are exact.

All lengths are millimetres, all angles in the JSON are degrees.
"""

import math

import numpy as np

POLE = 1e-4  # same pole radius as rdk (orientationVectorPoleRadius)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def ov_to_matrix(ox, oy, oz, theta_deg):
    n = math.sqrt(ox * ox + oy * oy + oz * oz)
    ox, oy, oz = ox / n, oy / n, oz / n
    lat = math.acos(max(-1.0, min(1.0, oz)))
    lon = 0.0 if 1 - abs(oz) <= POLE else math.atan2(oy, ox)
    return rot_z(lon) @ rot_y(lat) @ rot_z(math.radians(theta_deg))


def matrix_to_ov(r, snap=True):
    """Rotation matrix -> (ox, oy, oz, theta_deg), exact at the poles.
    snap=True: within the pole radius, report the vector as exactly (0, 0, +-1) (clean numbers for configs).
    snap=False: keep the small x / y part (as RDK reports poses); only an exact pole uses the pole branch."""
    r = np.asarray(r, dtype=float)
    ox, oy, oz = r[:, 2]
    if (1 - abs(oz) <= POLE) if snap else (math.hypot(ox, oy) < 1e-12):
        ox, oy, oz = 0.0, 0.0, (1.0 if oz > 0 else -1.0)
        lon = 0.0
    else:
        lon = math.atan2(oy, ox)
    lat = math.acos(max(-1.0, min(1.0, oz)))
    rest = (rot_z(lon) @ rot_y(lat)).T @ r  # = Rz(theta)
    theta = math.degrees(math.atan2(rest[1, 0], rest[0, 0]))
    clean = lambda v: 0.0 if abs(v) < 1e-12 else round(float(v), 9)
    return clean(ox), clean(oy), clean(oz), clean(theta)


def ov_json(r):
    ox, oy, oz, th = matrix_to_ov(r)
    return {"type": "ov_degrees", "value": {"x": ox, "y": oy, "z": oz, "th": th}}


def quat_to_matrix(q):
    """MuJoCo quaternion (w, x, y, z) -> rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def axis_angle(axis, angle):
    """Rotation about a unit axis by angle (radians)."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def pose(r=None, t=None):
    m = np.eye(4)
    if r is not None:
        m[:3, :3] = r
    if t is not None:
        m[:3, 3] = t
    return m


def sva_fk(model, joints_deg_or_mm, end=None):
    """Forward kinematics of a Viam SVA JSON model.

    Walks the chain from 'world' following parent links. Static links apply
    translation then orientation. Revolute joints turn about their axis,
    prismatic joints slide along it. Returns {frame_id: 4x4 pose in world}.
    """
    links = {l["id"]: l for l in model["links"]}
    joints = {j["id"]: j for j in model["joints"]}
    order = [j["id"] for j in model["joints"]]
    value = dict(zip(order, joints_deg_or_mm))
    poses = {"world": np.eye(4)}

    def solve(fid):
        if fid in poses:
            return poses[fid]
        if fid in links:
            l = links[fid]
            parent = solve(l["parent"])
            t = l.get("translation", {})
            tr = np.array([t.get("x", 0.0), t.get("y", 0.0), t.get("z", 0.0)])
            r = np.eye(3)
            if "orientation" in l:
                v = l["orientation"]["value"]
                r = ov_to_matrix(v["x"], v["y"], v["z"], v["th"])
            poses[fid] = parent @ pose(r, tr)
        else:
            j = joints[fid]
            parent = solve(j["parent"])
            ax = np.array([j["axis"]["x"], j["axis"]["y"], j["axis"]["z"]], dtype=float)
            q = value[fid]
            if j["type"] == "revolute":
                poses[fid] = parent @ pose(axis_angle(ax, math.radians(q)))
            else:
                poses[fid] = parent @ pose(t=ax / np.linalg.norm(ax) * q)
        return poses[fid]

    for fid in list(links) + list(joints):
        solve(fid)
    return poses
