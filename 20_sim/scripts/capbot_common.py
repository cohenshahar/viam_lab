"""Shared helpers for the capbot scripts: connect, load the generated numbers, build poses.

Machine addresses and API keys come from 20_sim/.env (never committed):
    SIM_ADDRESS=capbot-sim-main.xxxx.viam.cloud
    SIM_API_KEY_ID=...
    SIM_API_KEY=...
    REAL_ADDRESS=capbot-main.xxxx.viam.cloud
    REAL_API_KEY_ID=...
    REAL_API_KEY=...
Get them in the app: the machine -> CONNECT -> API keys.
"""

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "20_sim/tools"))

ENV_FILE = REPO / "20_sim/.env"


def load_env():
    if not ENV_FILE.exists():
        sys.exit(f"Missing {ENV_FILE.relative_to(REPO)}. Copy 20_sim/.env.example to 20_sim/.env and fill it in.")
    env = {}
    for line in ENV_FILE.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            env[k.strip()] = v.strip().strip('"').strip("'")
    return env


async def connect(machine="sim"):
    # The SDK only builds clients for resource types whose code was imported before connecting,
    # so import every type the helpers below use (arm_move needs generic + motion) here.
    import viam.components.arm  # noqa: F401  (collision_audit)
    import viam.components.generic  # noqa: F401
    import viam.services.generic  # noqa: F401  (the cap service)
    import viam.services.motion  # noqa: F401
    from viam.robot.client import RobotClient

    import os
    local = os.environ.get("CAPBOT_LOCAL")
    if local:
        # Offline test: a viam-server started by hand with a local config (no cloud, no keys),
        # e.g. CAPBOT_LOCAL=localhost:8081 python3 arm_check.py
        from viam.rpc.dial import DialOptions
        opts = RobotClient.Options(dial_options=DialOptions(insecure=True, disable_webrtc=True))
        return await RobotClient.at_address(local, opts)

    prefix = {"sim": "SIM", "real": "REAL"}[machine]
    env = load_env()
    try:
        address, key, key_id = env[f"{prefix}_ADDRESS"], env[f"{prefix}_API_KEY"], env[f"{prefix}_API_KEY_ID"]
    except KeyError as e:
        sys.exit(f"20_sim/.env is missing {e}")
    opts = RobotClient.Options.with_api_key(api_key=key, api_key_id=key_id)
    return await RobotClient.at_address(address, opts)


def numbers():
    return json.loads((REPO / "20_sim/generated/robot_numbers.json").read_text())


def kinematics(name):
    return json.loads((REPO / f"20_sim/kinematics/{name}.json").read_text())


def pose(x, y, z, ox=0.0, oy=0.0, oz=1.0, th=0.0):
    from viam.proto.common import Pose
    return Pose(x=x, y=y, z=z, o_x=ox, o_y=oy, o_z=oz, theta=th)


def pose_in(frame, p):
    from viam.proto.common import PoseInFrame
    return PoseInFrame(reference_frame=frame, pose=p)


def arm_constraints():
    """Constraints for every motion.move of the KR6.
    The wrist camera is bolted to the wrist. Viam can only hang it from the arm's end, and the
    fake camera reports a 40 x 40 x 10 mm box, which then sits inside link 4's collision box at
    every pose. So the planner may let the camera touch the wrist links it is mounted on.
    Also link_2 / link_4 and link_4 / link_6: their mesh-bound boxes overlap whenever the elbow or wrist folds;
    the real KR6 cannot hit itself there inside its joint limits. Same list as cap.ALLOWED_PAIRS in the module."""
    from viam.proto.service.motion import CollisionSpecification, Constraints
    allow = CollisionSpecification.AllowedFrameCollisions
    pairs = [allow(frame1="cam-wrist", frame2=f"kr6:{link}") for link in ("link_4", "link_5", "link_6")]
    pairs += [allow(frame1="kr6:link_2", frame2="kr6:link_4"), allow(frame1="kr6:link_4", frame2="kr6:link_6")]
    return Constraints(collision_specification=[CollisionSpecification(allows=pairs)])


def box_obstacle(label, frame, centre, size):
    """A box for a WorldState: centre (x, y, z) mm in `frame`, size (x, y, z) mm."""
    from viam.proto.common import Geometry, GeometriesInFrame, RectangularPrism, Vector3
    g = Geometry(center=pose(*centre), box=RectangularPrism(dims_mm=Vector3(x=size[0], y=size[1], z=size[2])),
                 label=label)
    return GeometriesInFrame(reference_frame=frame, geometries=[g])


def room_in_cart(obstacles, cart_pose):
    """The room's obstacle boxes, moved into the cart frame (= Viam's world frame, decision #29).

    obstacles: numbers()["world"]["obstacles"] (room frame, mm); each box's planning_box_mm is used.
    cart_pose: odom's pose of the cart in the room, {"x_mm", "y_mm", "yaw_deg"}.
    Returns [{"label", "centre_mm": [x, y, z], "yaw_deg", "size_mm": [x, y, z]}], all in the cart frame:
    p_cart = R(-yaw) * (p_room - cart_xy). Heights do not change (both frames sit on the floor)."""
    import math
    yaw = math.radians(cart_pose["yaw_deg"])
    c, s = math.cos(-yaw), math.sin(-yaw)
    out = []
    for label, o in obstacles.items():
        lo, hi = o["planning_box_mm"]["min"], o["planning_box_mm"]["max"]
        size = [hi[i] - lo[i] for i in range(3)]
        dx = (lo[0] + hi[0]) / 2 - cart_pose["x_mm"]
        dy = (lo[1] + hi[1]) / 2 - cart_pose["y_mm"]
        out.append({"label": label, "centre_mm": [dx * c - dy * s, dx * s + dy * c, (lo[2] + hi[2]) / 2],
                    "yaw_deg": -cart_pose["yaw_deg"], "size_mm": size})
    return out


def room_point_in_cart(xyz, cart_pose):
    """A point given in the room frame (mm), in the cart frame (= Viam's world frame)."""
    import math
    yaw = math.radians(cart_pose["yaw_deg"])
    dx, dy = xyz[0] - cart_pose["x_mm"], xyz[1] - cart_pose["y_mm"]
    return (dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw), xyz[2])


def room_world_state(obstacles, cart_pose):
    """room_in_cart() as a Viam WorldState, for motion.move."""
    from viam.proto.common import Geometry, GeometriesInFrame, RectangularPrism, Vector3, WorldState
    geoms = [Geometry(center=pose(*b["centre_mm"], oz=1.0, th=b["yaw_deg"]),
                      box=RectangularPrism(dims_mm=Vector3(x=b["size_mm"][0], y=b["size_mm"][1], z=b["size_mm"][2])),
                      label=b["label"])
             for b in room_in_cart(obstacles, cart_pose)]
    return WorldState(obstacles=[GeometriesInFrame(reference_frame="world", geometries=geoms)])


async def arm_move(robot, destination, extra_obstacles=()):
    """Every KR6 motion goes through here: vacuum to `destination` (a PoseInFrame).

    Reads the cart's place in the room from odom, puts the room's obstacles around the cart,
    and asks the motion service to plan for the arm only. Viam's world frame is the cart, so
    the planner cannot move the cart (decision #29). Returns what motion.move returns."""
    from viam.components.generic import Generic
    from viam.services.motion import MotionClient

    cart_pose = await Generic.from_robot(robot, "odom").do_command({"command": "pose"})
    ws = room_world_state(numbers()["world"]["obstacles"], cart_pose)
    ws.obstacles.extend(extra_obstacles)
    motion = MotionClient.from_robot(robot, "builtin")
    return await motion.move("vacuum", destination, world_state=ws, constraints=arm_constraints())


def _obb_overlap(a, b):
    """Two boxes given as 8 corners each (same frame) overlap? Separating axis test, 15 axes."""
    import numpy as np

    def axes(c):
        return [c[4] - c[0], c[2] - c[0], c[1] - c[0]]
    ax = axes(a) + axes(b) + [np.cross(u, v) for u in axes(a) for v in axes(b)]
    for v in ax:
        n = np.linalg.norm(v)
        if n < 1e-9:
            continue
        v = v / n
        pa, pb = a @ v, b @ v
        if pa.max() < pb.min() or pb.max() < pa.min():
            return False
    return True


def _box_corners(T, size, centre):
    """8 corners (cart frame) of a box of size (x, y, z) centred at `centre` in frame T (4x4)."""
    import itertools
    import numpy as np
    h, c = np.array(size, float) / 2, np.array(centre, float)
    return np.array([(T @ np.append(c + h * s, 1))[:3] for s in itertools.product((-1, 1), repeat=3)])


async def collision_audit(robot, shrink_mm=2.0):
    """Independent check, from our own kinematics (not Viam's planner): do the KR6 link boxes, the tool box, the held
    crate or the cart body overlap the room's obstacles (no margin) or a resting crate? Returns a list of
    "<part> x <thing>" strings, empty when clear. Boxes are shrunk by shrink_mm so resting contact (cup on a crate,
    crate on the table) does not count."""
    import numpy as np
    import viam_frames as vf
    from viam.components.arm import Arm

    n, kr6 = numbers(), kinematics("kr6")
    st = await cap(robot, "status")
    cart = st["cart"]
    q = list((await Arm.from_robot(robot, "kr6").get_joint_positions()).values)
    fk = vf.sva_fk(kr6, q)
    mount = vf.pose(t=np.array(n["viam_frames"]["kr6_translation_mm"], float))
    tcp = mount @ fk["flange"] @ vf.pose(t=np.array(n["viam_frames"]["vacuum_translation_mm"], float))
    s = shrink_mm
    parts = {}
    for l in kr6["links"]:
        g = l.get("geometry")
        if g:
            parts[f"kr6:{l['id']}"] = _box_corners(mount @ fk[l["id"]], [g["x"] - s, g["y"] - s, g["z"] - s],
                                                   [g["translation"][k] for k in "xyz"])
    tg = n["viam_frames"]["vacuum_geometry"]
    parts["tool"] = _box_corners(tcp, [tg["x"] - s, tg["y"] - s, tg["z"] - s], [tg["translation"][k] for k in "xyz"])
    cg = n["viam_frames"]["cart_geometry"]
    parts["cart body"] = _box_corners(np.eye(4), [cg["x"] - s, cg["y"] - s, cg["z"] - s], [cg["translation"][k] for k in "xyz"])
    held = next((k for k, v in st["boxes"].items() if v == "held"), None)
    size = 120.0
    if held:
        parts[f"held {held}"] = _box_corners(tcp, [size - s] * 3, [0, 0, size / 2])
    yaw = np.radians(cart["yaw_deg"])
    R = np.array([[np.cos(yaw), np.sin(yaw), 0], [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])   # room -> cart rotation
    room_to_cart = np.eye(4)
    room_to_cart[:3, :3] = R
    room_to_cart[:3, 3] = -R @ np.array([cart["x_mm"], cart["y_mm"], 0.0])
    things = {}
    for name, o in n["world"]["obstacles"].items():
        lo, hi = np.array(o["min_mm"], float), np.array(o["max_mm"], float)
        if name == "floor":
            hi[2] -= s   # the floor's top face is z = 0; parts resting on it are fine
        things[name] = _box_corners(room_to_cart, hi - lo - s, (hi + lo) / 2)
    for name, v in st["boxes"].items():
        if isinstance(v, dict):
            T = room_to_cart @ vf.pose(vf.rot_z(np.radians(v["yaw_deg"])), np.array([v["x_mm"], v["y_mm"], v["z_mm"]]))
            things[f"crate {name}"] = _box_corners(T, [size - s] * 3, [0, 0, 0])
    hits = []
    for pname, pc in parts.items():
        for tname, tc in things.items():
            if pname == "cart body" and tname == "floor":
                continue
            if _obb_overlap(pc, tc):
                hits.append(f"{pname} x {tname}")
    return hits


async def cap(robot, verb, **args):
    """Call one `cap` verb (20_sim/primitive_map.md). Returns its answer: {"ok": True, ...} or {"error": ...}."""
    from viam.services.generic import Generic as GenericService
    return dict(await GenericService.from_robot(robot, "cap").do_command({"command": verb, **args}, timeout=300))


def fmt_pose(p):
    return (f"x={p.x:9.2f} y={p.y:9.2f} z={p.z:9.2f}  "
            f"o=({p.o_x:+.3f},{p.o_y:+.3f},{p.o_z:+.3f}) th={p.theta:+8.3f}")
