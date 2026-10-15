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


def box_obstacle(label, frame, centre, size):
    """A box for a WorldState: centre (x, y, z) mm in `frame`, size (x, y, z) mm."""
    from viam.proto.common import Geometry, GeometriesInFrame, RectangularPrism, Vector3
    g = Geometry(center=pose(*centre), box=RectangularPrism(dims_mm=Vector3(x=size[0], y=size[1], z=size[2])),
                 label=label)
    return GeometriesInFrame(reference_frame=frame, geometries=[g])


def fmt_pose(p):
    return (f"x={p.x:9.2f} y={p.y:9.2f} z={p.z:9.2f}  "
            f"o=({p.o_x:+.3f},{p.o_y:+.3f},{p.o_z:+.3f}) th={p.theta:+8.3f}")
