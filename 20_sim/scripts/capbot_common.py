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
    from viam.robot.client import RobotClient

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
    every pose. So the planner may let the camera touch the wrist links it is mounted on."""
    from viam.proto.service.motion import CollisionSpecification, Constraints
    allow = CollisionSpecification.AllowedFrameCollisions
    pairs = [allow(frame1="cam-wrist", frame2=f"kr6:{link}") for link in ("link_4", "link_5", "link_6")]
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


def fmt_pose(p):
    return (f"x={p.x:9.2f} y={p.y:9.2f} z={p.z:9.2f}  "
            f"o=({p.o_x:+.3f},{p.o_y:+.3f},{p.o_z:+.3f}) th={p.theta:+8.3f}")
