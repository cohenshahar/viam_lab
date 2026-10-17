"""Build every Viam file that holds a robot or room number, from the thesis MuJoCo model.

One source of truth: the numbers live in the CaP_Thesis MuJoCo files. This script
compiles that model with MuJoCo and writes the Viam versions. Never edit the
outputs by hand; change the thesis model (or 30_robot/measured_*.json) and run
this again.

Run:
    export CAP_THESIS_DIR=/path/to/CaP_Thesis
    python3 20_sim/tools/build_viam_files.py

Needs: mujoco (same version as the thesis, 3.8.1), numpy.

Writes (paths relative to the Viam_Lab repo):
    20_sim/generated/robot_numbers.json   every number, mm / degrees, with its source
    20_sim/kinematics/kr6.json            KR6 kinematics for Viam (SVA format)
    20_sim/generated/visuals/             CAD look for the 3D scene (Phase V, decision #30): KR6 link GLBs,
                                          tool / cart / room PLYs, all in metres, + visuals.json

The room (world.obstacles) stays in robot_numbers.json, in the room frame. Viam's world frame is
the cart (decision #29), so 20_sim/scripts/capbot_common.py places the room around the cart from
the odometry before each arm move.
"""

import json
import math
import os
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import viam_frames as vf  # noqa: E402
import visual_meshes as vm  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
WORLD_REL = "sim/cart_kr6/worlds/world_lab.xml"
MM = 1000.0
MARGIN_MM = 30.0           # obstacle clearance, the planner adds none of its own
CART_BOX_FLOOR_GAP_MM = 50.0  # cart body box starts this far above the floor
ARM_SEAT_GAP_MM = 2.0      # cart body box stops this far under the arm base plate
ARM_BASE_BOX_LIFT_MM = 1.0  # arm base box starts this far above the plate underside
TOOL_BOX_SHORT_MM = 5.0    # tool box ends this far before the cup face, so the cup can touch a box

ARM_CHAIN = [  # (body, joint that moves it)
    ("kuka_base", None),
    ("link_1", "joint_a1"),
    ("link_2", "joint_a2"),
    ("link_3", "joint_a3"),
    ("link_4", "joint_a4"),
    ("link_5", "joint_a5"),
    ("link_6", "joint_a6"),
]
TOOL_ROOT = "vac_mount"
TCP_SITE = "suction_tip_site"
CART = "cart_base"

OBSTACLES = {  # Viam name: (MuJoCo geom names or a body subtree, keep top exact?)
    "pillar-front-left": (["lab_column_front_left"], False),
    "pillar-front-right": (["lab_column_front_right"], False),
    "pillar-rear-left": (["lab_column_rear_left"], False),
    "pillar-rear-right": (["lab_column_rear_right"], False),
    "lab-table": ("lab_table", True),
    "lab-cabinet": ("lab_cabinet", False),
    "floor": (["lab_floor"], True),
}


def thesis_dir():
    d = os.environ.get("CAP_THESIS_DIR")
    if not d:
        sys.exit("Set CAP_THESIS_DIR to the CaP_Thesis repo folder first.")
    p = Path(d).expanduser().resolve()
    if not (p / WORLD_REL).exists():
        sys.exit(f"Not a CaP_Thesis folder (no {WORLD_REL}): {p}")
    return p


def r3(v):
    return [round(float(x), 3) for x in v]


def xyz(v):
    return {"x": round(float(v[0]), 3), "y": round(float(v[1]), 3), "z": round(float(v[2]), 3)}


class Model:
    def __init__(self, path):
        self.m = mujoco.MjModel.from_xml_path(str(path))
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)

    def id(self, kind, name):
        i = mujoco.mj_name2id(self.m, kind, name)
        if i < 0:
            raise KeyError(f"{name} not found in the model")
        return i

    def body(self, name):
        return self.id(mujoco.mjtObj.mjOBJ_BODY, name)

    def body_local(self, name):
        b = self.body(name)
        return vf.pose(vf.quat_to_matrix(self.m.body_quat[b]), self.m.body_pos[b] * MM)

    def body_in(self, name, ancestor):
        """Pose of body `name` in the frame of body `ancestor` (from the model tree, not the state)."""
        b, a = self.body(name), self.body(ancestor)
        t = np.eye(4)
        while b != a:
            if b == 0:
                raise ValueError(f"{ancestor} is not above {name}")
            t = vf.pose(vf.quat_to_matrix(self.m.body_quat[b]), self.m.body_pos[b] * MM) @ t
            b = self.m.body_parentid[b]
        return t

    def subtree(self, name):
        root = self.body(name)
        out = []
        for b in range(self.m.nbody):
            x = b
            while x not in (0, root):
                x = self.m.body_parentid[x]
            if x == root:
                out.append(b)
        return out

    def geom_corners(self, g, frame_body):
        """8 corners of geom g's bounding box, in the frame of frame_body (mm)."""
        m = self.m
        c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
        geom_pose = vf.pose(vf.quat_to_matrix(m.geom_quat[g]), m.geom_pos[g] * MM)
        owner = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "world"
        to_frame = np.eye(4) if owner == frame_body else self.body_in(owner, frame_body)
        pts = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    p = (c + h * np.array([sx, sy, sz])) * MM
                    pts.append((to_frame @ geom_pose @ np.append(p, 1))[:3])
        return np.array(pts)

    def collision_geoms(self, bodies):
        m = self.m
        return [g for g in range(m.ngeom)
                if m.geom_bodyid[g] in bodies and (m.geom_contype[g] or m.geom_conaffinity[g])]

    def box_of(self, geoms, frame_body):
        pts = np.vstack([self.geom_corners(g, frame_body) for g in geoms])
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        return lo, hi

    def world_box(self, geoms):
        """Axis-aligned box in world coordinates from the current state (mm)."""
        d, m = self.d, self.m
        pts = []
        for g in geoms:
            c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        p = c + h * np.array([sx, sy, sz])
                        pts.append((d.geom_xmat[g].reshape(3, 3) @ p + d.geom_xpos[g]) * MM)
        pts = np.array(pts)
        return pts.min(axis=0), pts.max(axis=0)


def box_geometry(lo, hi, label):
    size, centre = hi - lo, (hi + lo) / 2
    return {"type": "box", "x": round(float(size[0]), 1), "y": round(float(size[1]), 1),
            "z": round(float(size[2]), 1), "translation": xyz(centre), "label": label}


def build_kr6(mdl, numbers):
    """KR6 as Viam SVA. Each moving link gets a zero-length link that carries its collision box
    (so the box sits exactly in the MuJoCo body frame), then an offset link to the next joint."""
    m = mdl.m
    links, joints, record = [], [], []
    prev = "world"
    for i, (body, joint) in enumerate(ARM_CHAIN):
        if joint:
            j = mdl.id(mujoco.mjtObj.mjOBJ_JOINT, joint)
            lo, hi = np.degrees(m.jnt_range[j])
            joints.append({"id": joint, "type": "revolute", "parent": prev,
                           "axis": xyz(m.jnt_axis[j]), "min": round(float(lo), 4), "max": round(float(hi), 4)})
            if np.linalg.norm(m.jnt_pos[j]) > 1e-9:
                raise ValueError(f"{joint} is not at its body origin; the generator assumes it is")
            record.append({"joint": joint, "axis": r3(m.jnt_axis[j]), "min_deg": round(float(lo), 3),
                           "max_deg": round(float(hi), 3)})
            prev = joint
        geoms = mdl.collision_geoms({mdl.body(body)})
        lo_b, hi_b = mdl.box_of(geoms, body)
        if joint is None:
            # The mesh bounding box dips under the base plate; the plate sits on the cart top,
            # so start the box just above it or the arm "collides" with its own cart.
            lo_b[2] = max(lo_b[2], ARM_BASE_BOX_LIFT_MM)
        link = {"id": body, "parent": prev, "translation": xyz([0, 0, 0]),
                "geometry": box_geometry(lo_b, hi_b, body)}
        links.append(link)
        prev = body
        if i + 1 < len(ARM_CHAIN):
            nxt = mdl.body_local(ARM_CHAIN[i + 1][0])
            if not np.allclose(nxt[:3, :3], np.eye(3), atol=1e-9):
                raise ValueError(f"{ARM_CHAIN[i + 1][0]} is rotated; add orientation support first")
            links.append({"id": f"{body}_to_next", "parent": prev, "translation": xyz(nxt[:3, 3])})
            prev = f"{body}_to_next"
    # Flange: MuJoCo link_6 points its tool along +X. Viam expects +Z out of the flange,
    # so turn the last frame about Y by +90 deg (new +Z = old +X).
    flange_r = vf.rot_y(math.pi / 2)
    links.append({"id": "flange", "parent": prev, "translation": xyz([0, 0, 0]),
                  "orientation": vf.ov_json(flange_r)})
    for lk in links:  # drop helper labels Viam does not know
        if "geometry" in lk:
            lk["geometry"].pop("label", None)
    model = {"name": "kuka_kr6_capbot", "kinematic_param_type": "SVA", "links": links, "joints": joints}
    numbers["arm"] = {
        "source": f"{WORLD_REL} -> arm_kr6_body.xml (link tree), compiled with MuJoCo {mujoco.__version__}",
        "joints": record,
        "flange_rotation": "Ry(+90 deg): Viam +Z of the flange = MuJoCo +X of link_6",
    }
    return model, flange_r


def build_tool(mdl, numbers, flange_r):
    """Vacuum tool: TCP offset and one box, both in the Viam flange frame (+Z out of the flange)."""
    s = mdl.id(mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    site_body = mujoco.mj_id2name(mdl.m, mujoco.mjtObj.mjOBJ_BODY, mdl.m.site_bodyid[s])
    tcp_l6 = (mdl.body_in(site_body, "link_6") @ np.append(mdl.m.site_pos[s] * MM, 1))[:3]
    to_flange = flange_r.T  # link_6 coordinates -> flange coordinates
    tcp_f = to_flange @ tcp_l6
    lo, hi = mdl.box_of(mdl.collision_geoms(set(mdl.subtree(TOOL_ROOT))), "link_6")
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    cf = corners @ to_flange.T
    lo_f, hi_f = cf.min(axis=0), cf.max(axis=0)
    hi_f[2] = min(hi_f[2], tcp_f[2] - TOOL_BOX_SHORT_MM)
    box = box_geometry(lo_f, hi_f, "vacuum tool")
    box.pop("label")
    numbers["tool"] = {
        "source": "sim/tools/vacuum/vacuum_body.xml (site suction_tip_site, collision geoms)",
        "tcp_in_flange_mm": r3(tcp_f),
        "tool_box_in_flange_mm": {"min": r3(lo_f), "max": r3(hi_f)},
        "note": "The telescope joint (0..-60 mm spring) is left out: the TCP is the uncompressed cup face.",
    }
    # Gripper frame = the TCP. Its box is given relative to the TCP.
    box["translation"] = xyz(np.array([box["translation"][k] for k in "xyz"]) - tcp_f)
    return tcp_f, box


def build_cart(mdl, numbers):
    m = mdl.m
    cart = mdl.body(CART)
    lo, hi = mdl.box_of(mdl.collision_geoms({cart}), CART)
    mount = mdl.body_in("kuka_base", CART)
    wl = mdl.body_in("wheel_left", CART)[:3, 3]
    wr = mdl.body_in("wheel_right", CART)[:3, 3]
    wgeom = mdl.id(mujoco.mjtObj.mjOBJ_GEOM, "wheel_left_geom")
    wheel_d = 2 * m.geom_size[wgeom][0] * MM
    body_lo = np.array([lo[0], lo[1], CART_BOX_FLOOR_GAP_MM])
    body_hi = np.array([hi[0], hi[1], mount[2, 3] - ARM_SEAT_GAP_MM])
    numbers["cart"] = {
        "source": "sim/cart_kr6/cart.xml (cart_base frame: centre of the drive axle on the floor)",
        "frame_outline_mm": {"min": r3(lo), "max": r3(hi)},
        "body_box_for_viam_mm": {"min": r3(body_lo), "max": r3(body_hi)},
        "drive_wheel_diameter_mm": round(float(wheel_d), 1),
        "drive_wheel_circumference_mm": round(float(math.pi * wheel_d), 1),
        "track_width_mm": round(float(wl[1] - wr[1]), 1),
        "arm_mount_in_cart_mm": r3(mount[:3, 3]),
        "arm_mount_rotation": "none" if np.allclose(mount[:3, :3], np.eye(3)) else vf.ov_json(mount[:3, :3]),
    }
    return body_lo, body_hi, mount


def camera_viam_rotation(r_body_cam):
    """MuJoCo camera looks along its -Z with +Y up. Viam/OpenCV camera: +Z forward, +X right, +Y down."""
    return r_body_cam @ np.diag([1.0, -1.0, -1.0])


def build_sensors(mdl, numbers):
    m = mdl.m
    out = {}
    for cam, parent in (("rgbd_front", CART), ("rgbd_rear", CART), ("rgbd_wrist", "link_5")):
        c = mdl.id(mujoco.mjtObj.mjOBJ_CAMERA, cam)
        body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.cam_bodyid[c])
        t = mdl.body_in(body, parent) @ vf.pose(vf.quat_to_matrix(m.cam_quat[c]), m.cam_pos[c] * MM)
        out[cam] = {"parent_in_mujoco": parent, "translation_mm": r3(t[:3, 3]),
                    "orientation": vf.ov_json(camera_viam_rotation(t[:3, :3])),
                    "fovy_deg": round(float(m.cam_fovy[c]), 2)}
    lidar = mdl.body_in("lidar_base", CART)
    out["lidar"] = {"parent_in_mujoco": CART, "translation_mm": r3(lidar[:3, 3]),
                    "orientation": vf.ov_json(lidar[:3, :3])}
    s = mdl.id(mujoco.mjtObj.mjOBJ_SITE, "imu_base")
    imu_body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.site_bodyid[s])
    imu = mdl.body_in(imu_body, CART) @ vf.pose(vf.quat_to_matrix(m.site_quat[s]), m.site_pos[s] * MM)
    out["imu"] = {"parent_in_mujoco": CART, "translation_mm": r3(imu[:3, 3]),
                  "orientation": vf.ov_json(imu[:3, :3])}
    # Wrist camera for Viam: hang it on the arm's end (the flange frame, all joints at 0).
    link5_to_flange = mdl.body_in("link_6", "link_5") @ vf.pose(vf.rot_y(math.pi / 2))
    w = out["rgbd_wrist"]
    cam_l5 = vf.pose(vf.ov_to_matrix(*[w["orientation"]["value"][k] for k in ("x", "y", "z", "th")]),
                     w["translation_mm"])
    cam_f = np.linalg.inv(link5_to_flange) @ cam_l5
    out["rgbd_wrist_on_flange"] = {"parent_in_viam": "kr6", "translation_mm": r3(cam_f[:3, 3]),
                                   "orientation": vf.ov_json(cam_f[:3, :3])}
    numbers["sensors"] = {"source": "sim/sensors/*/mount_*.xml", "frames": out,
                          "note": "Camera orientations are converted to Viam's camera habit (+Z forward, +Y down). "
                                  "The wrist camera rides on link_5 in the thesis; Viam can only hang it on the "
                                  "arm's end, so in Viam it turns with joint a6 (known difference)."}
    return out


NAMED_POSES_REL = "ros2_ws/src/cap_cart_teleop/config/arm_named_poses.json"


def build_named_poses(cap, numbers):
    """The thesis named pose table (the one go_to_named_pose uses), in degrees."""
    table = json.loads((cap / NAMED_POSES_REL).read_text())
    order = [j for _, j in ARM_CHAIN[1:]]
    if table["joints"] != order or table.get("unit") != "rad":
        raise ValueError(f"{NAMED_POSES_REL}: expected joints {order} in rad")
    poses = {k: [round(math.degrees(v), 3) for v in q] for k, q in table["poses"].items()}
    numbers["named_arm_poses_deg"] = {"source": NAMED_POSES_REL, **poses}
    return poses


def build_world(mdl, numbers):
    rec = {}
    for name, (what, exact_top) in OBSTACLES.items():
        if isinstance(what, str):
            geoms = mdl.collision_geoms(set(mdl.subtree(what)))
        else:
            geoms = [mdl.id(mujoco.mjtObj.mjOBJ_GEOM, g) for g in what]
        lo, hi = mdl.world_box(geoms)
        if name == "floor":
            glo, ghi = lo.copy(), hi.copy()
        else:
            glo, ghi = lo - MARGIN_MM, hi + MARGIN_MM
            glo[2] = max(glo[2], 0.0)  # never below the floor
            if exact_top:
                ghi[2] = hi[2]
        rec[name] = {"min_mm": r3(lo), "max_mm": r3(hi), "margin_mm": 0 if name == "floor" else MARGIN_MM,
                     "top_kept_exact": bool(exact_top),
                     "planning_box_mm": {"min": r3(glo), "max": r3(ghi)}}
    numbers["world"] = {"source": "sim/worlds/lab/*.xml via " + WORLD_REL,
                        "status": "SKETCH, not measured (see sim/worlds/lab/README.md)",
                        "origin": "centre of the lab floor, +X along the 9 m side",
                        "obstacles": rec,
                        "note": "Room frame. planning_box_mm = the box the arm planner avoids (margin added)."}


def geom_part(mdl, g, frame_body):
    """One visible MuJoCo geom as (verts mm in frame_body, faces, rgb), or None if not drawable."""
    m = mdl.m
    typ, s = m.geom_type[g], m.geom_size[g] * MM
    G = mujoco.mjtGeom
    if typ == G.mjGEOM_MESH:
        mid = m.geom_dataid[g]
        va, vn, fa, fn = m.mesh_vertadr[mid], m.mesh_vertnum[mid], m.mesh_faceadr[mid], m.mesh_facenum[mid]
        v, f = m.mesh_vert[va:va + vn] * MM, m.mesh_face[fa:fa + fn]
    elif typ == G.mjGEOM_BOX:
        v, f = vm.box(s)
    elif typ == G.mjGEOM_CYLINDER:
        v, f = vm.cylinder(s[0], s[1])
    elif typ == G.mjGEOM_SPHERE:
        v, f = vm.sphere(s[0])
    elif typ == G.mjGEOM_CAPSULE:
        v, f = vm.capsule(s[0], s[1])
    else:
        return None
    owner = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "world"
    to_frame = np.eye(4) if owner == frame_body else mdl.body_in(owner, frame_body)
    geom_pose = vf.pose(vf.quat_to_matrix(m.geom_quat[g]), m.geom_pos[g] * MM)
    rgba = m.geom_rgba[g] if m.geom_matid[g] < 0 else m.mat_rgba[m.geom_matid[g]]
    return vm.transform(v, to_frame @ geom_pose), np.asarray(f), rgba[:3]


def visible_geoms(mdl, bodies):
    """Geoms MuJoCo draws by default (groups 0-2), mostly opaque, not the ground plane."""
    m = mdl.m
    out = []
    for g in range(m.ngeom):
        rgba = m.geom_rgba[g] if m.geom_matid[g] < 0 else m.mat_rgba[m.geom_matid[g]]
        if (m.geom_bodyid[g] in bodies and m.geom_group[g] <= 2 and rgba[3] >= 0.5
                and m.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE):
            out.append(g)
    return out


def chunks(parts, max_faces):
    """Split parts into groups of at most max_faces faces (one scene shape each, each well under 1 MB)."""
    out, cur, n = [], [], 0
    for p in parts:
        if cur and n + len(p[1]) > max_faces:
            out.append(cur)
            cur, n = [], 0
        cur.append(p)
        n += len(p[1])
    return out + ([cur] if cur else [])


def build_visuals(mdl, numbers, kr6, flange_r, tcp_f):
    """CAD look for Viam's 3D scene (decision #30). Arm: one GLB per link, placed at that link's box centre
    (that is how the scene draws get_3d_models). Tool / cart / room: PLYs for the scene service."""
    out = REPO / "20_sim/generated/visuals"
    (out / "kr6").mkdir(parents=True, exist_ok=True)
    manifest = {"_about": "GENERATED by build_viam_files.py. Metres. See 20_sim/README.md (CAD look).",
                "kr6": {}, "shapes": []}
    tool_bodies = set(mdl.subtree(TOOL_ROOT))
    # each arm link carries everything mounted on it (e.g. the wrist camera on link_5), except the next link
    # and the tool (the tool is drawn on the vacuum frame)
    names = [b for b, _ in ARM_CHAIN]
    carried = {}
    for i, name in enumerate(names):
        below = set(mdl.subtree(names[i + 1])) if i + 1 < len(names) else set()
        carried[name] = set(mdl.subtree(name)) - below - tool_bodies
    for link in kr6["links"]:
        if "geometry" not in link:
            continue
        c = np.array([link["geometry"]["translation"][k] for k in "xyz"])
        parts = [p for p in (geom_part(mdl, g, link["id"]) for g in visible_geoms(mdl, carried[link["id"]])) if p]
        parts = [(v - c, f, rgb) for v, f, rgb in parts]
        size = vm.write_glb(parts, out / "kr6" / f"{link['id']}.glb")
        manifest["kr6"][link["id"]] = {"file": f"kr6/{link['id']}.glb", "bytes": size,
                                       "faces": int(sum(len(p[1]) for p in parts))}
    # tool, in the Viam gripper frame (= the TCP; +Z out of the flange)
    to_flange = vf.pose(flange_r.T, -flange_r.T @ np.zeros(3))
    parts = []
    for g in visible_geoms(mdl, tool_bodies):
        p = geom_part(mdl, g, "link_6")
        if p:
            parts.append((vm.transform(p[0], to_flange) - tcp_f, p[1], p[2]))
    shapes = [("tool", "vacuum", parts)]
    # cart, in the cart frame (= Viam's world frame)
    cart_bodies = set(mdl.subtree(CART)) - set(mdl.subtree(ARM_CHAIN[0][0]))
    parts = [p for p in (geom_part(mdl, g, CART) for g in visible_geoms(mdl, cart_bodies)) if p]
    for i, group in enumerate(chunks(parts, 30000)):
        shapes.append((f"cart-{i + 1}", "cart", group))
    # room, in the room frame; the scene service places it around the cart from odom
    robot = set(mdl.subtree(CART))
    room = [b for b in range(mdl.m.nbody) if b not in robot]
    parts = [p for p in (geom_part(mdl, g, "world") for g in visible_geoms(mdl, room)) if p]
    shapes.append(("room", "room", parts))
    for name, parent, group in shapes:
        size = vm.write_ply(group, out / f"{name}.ply")
        manifest["shapes"].append({"name": name, "file": f"{name}.ply", "parent": parent, "bytes": size,
                                   "faces": int(sum(len(p[1]) for p in group))})
    (out / "visuals.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    numbers["visuals"] = {"dir": "20_sim/generated/visuals", "note": "CAD look only; the planner uses the boxes."}
    print("wrote", out.relative_to(REPO), f"({len(manifest['kr6'])} arm links, {len(manifest['shapes'])} shapes)")


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    print("wrote", path.relative_to(REPO))


def main():
    cap = thesis_dir()
    mdl = Model(cap / WORLD_REL)
    numbers = {"_about": "GENERATED by 20_sim/tools/build_viam_files.py from the CaP_Thesis MuJoCo model. "
                         "Do not edit. Units: mm and degrees.",
               "mujoco_version": mujoco.__version__}
    kr6, flange_r = build_kr6(mdl, numbers)
    tcp, tool_box = build_tool(mdl, numbers, flange_r)
    body_lo, body_hi, mount = build_cart(mdl, numbers)
    build_sensors(mdl, numbers)
    build_named_poses(cap, numbers)
    build_world(mdl, numbers)
    build_visuals(mdl, numbers, kr6, flange_r, tcp)
    numbers["viam_frames"] = {
        "cart_parent": "world",
        "cart_geometry": {k: v for k, v in box_geometry(body_lo, body_hi, "").items() if k != "label"},
        "kr6_parent": "cart", "kr6_translation_mm": r3(mount[:3, 3]),
        "vacuum_parent": "kr6", "vacuum_translation_mm": r3(tcp), "vacuum_geometry": tool_box,
    }
    write(REPO / "20_sim/kinematics/kr6.json", kr6)
    write(REPO / "20_sim/generated/robot_numbers.json", numbers)


if __name__ == "__main__":
    main()
