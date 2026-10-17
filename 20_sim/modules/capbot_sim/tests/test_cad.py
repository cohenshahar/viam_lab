"""Test the CAD-look parts (no viam-server needed): capbot:sim:kr6 and capbot:sim:scene.

Run: python3 20_sim/modules/capbot_sim/tests/test_cad.py
Needs the generated files (20_sim/kinematics/kr6.json, 20_sim/generated/visuals/).
"""

import asyncio
import json
import math
import sys
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(SIM / "modules/capbot_sim/src"))
sys.path.insert(0, str(SIM / "tools"))

from google.protobuf.struct_pb2 import Struct  # noqa: E402
from viam.components.arm import JointPositions  # noqa: E402
from viam.components.generic import Generic  # noqa: E402
from viam.proto.app.robot import ComponentConfig, ServiceConfig  # noqa: E402
from viam.proto.service.worldstatestore import TransformChangeType  # noqa: E402

import viam_frames as vf  # noqa: E402
from kr6_arm import Kr6Arm, plan_steps  # noqa: E402
from scene import Scene, room_pose  # noqa: E402

KIN = str(SIM / "kinematics/kr6.json")
VIS = str(SIM / "generated/visuals")


def cfg(kind, name, attrs):
    s = Struct()
    s.update(attrs)
    return kind(name=name, attributes=s)


class StandInOdom:
    def __init__(self):
        self.pose = {"x_mm": 0.0, "y_mm": 0.0, "yaw_deg": 0.0}

    async def do_command(self, command, **kw):
        return dict(self.pose)


def test_plan_steps():
    steps = plan_steps([0, 0], [10, -5], speed_deg_s=100, dt=0.01)
    assert len(steps) == 10 and steps[-1] == [10, -5], steps[-1]
    assert abs(steps[0][0] - 1) < 1e-9 and abs(steps[0][1] + 0.5) < 1e-9
    print("ok   plan_steps: all joints arrive together at the fastest joint's speed limit")


async def test_arm():
    attrs = {"kinematics_path": KIN, "visuals_dir": VIS, "speed_deg_per_sec": 600, "tick_hz": 200}
    assert Kr6Arm.validate_config(cfg(ComponentConfig, "kr6", attrs)) == ([], [])
    for bad in ({**attrs, "kinematics_path": "/nope.json"}, {**attrs, "speed_deg_per_sec": 0}):
        try:
            Kr6Arm.validate_config(cfg(ComponentConfig, "kr6", bad))
            raise AssertionError(f"accepted {bad}")
        except ValueError as e:
            print("ok   rejects bad config:", e)
    arm = Kr6Arm.new(cfg(ComponentConfig, "kr6", attrs), {})
    model = json.loads(Path(KIN).read_text())
    q = [10, -80, 100, 20, 45, -30]
    await arm.move_to_joint_positions(JointPositions(values=q))
    got = await arm.get_joint_positions()
    assert np.allclose(got.values, q), got
    p = await arm.get_end_position()
    want = vf.sva_fk(model, q)["flange"]
    assert math.dist((p.x, p.y, p.z), want[:3, 3]) < 1e-6
    r = vf.ov_to_matrix(p.o_x, p.o_y, p.o_z, p.theta)
    assert np.allclose(r, want[:3, :3], atol=1e-6)
    print(f"ok   move + end position = our FK ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})")
    try:
        await arm.move_to_joint_positions(JointPositions(values=[400, 0, 0, 0, 0, 0]))
        raise AssertionError("accepted a joint outside its limits")
    except ValueError as e:
        print("ok   refuses:", e)
    await arm.move_through_joint_positions([JointPositions(values=[0] * 6), JointPositions(values=q)])
    assert np.allclose((await arm.get_joint_positions()).values, q)
    links = {l["id"] for l in model["links"] if "geometry" in l}
    models = await arm.get_3d_models()
    assert set(models) == links, (set(models), links)
    assert all(m.mesh[:4] == b"glTF" for m in models.values())
    fmt, data = await arm.get_kinematics()
    assert json.loads(data)["name"] == model["name"]
    print(f"ok   3D models for every link with a box: {sorted(models)}")
    await arm.close()


async def test_scene():
    attrs = {"visuals_dir": VIS, "poll_hz": 100}
    assert Scene.validate_config(cfg(ServiceConfig, "scene", attrs)) == (["odom"], [])
    odom = StandInOdom()
    sc = Scene.new(cfg(ServiceConfig, "scene", attrs), {Generic.get_resource_name("odom"): odom})
    ids = await sc.list_uuids()
    names = {(await sc.get_transform(u)).reference_frame for u in ids}
    assert {"cad-tool", "cad-room", "cad-cart-1"} <= names, names
    parents = {(await sc.get_transform(u)).reference_frame: (await sc.get_transform(u)).pose_in_observer_frame.reference_frame
               for u in ids}
    assert parents["cad-tool"] == "vacuum" and parents["cad-cart-1"] == "cart" and parents["cad-room"] == "world"
    print(f"ok   {len(ids)} shapes: tool on vacuum, cart on cart, room on world")

    stream = sc.stream_transform_changes()
    opener = await stream.__anext__()   # Viam's stream protocol opens with an empty message
    assert not opener.transform.reference_frame and not opener.change_type
    first = asyncio.ensure_future(stream.__anext__())
    await asyncio.sleep(0.05)
    odom.pose = {"x_mm": 1000.0, "y_mm": 500.0, "yaw_deg": 90.0}
    change = await asyncio.wait_for(first, 2)
    for _ in range(5):   # the room's starting pose may come first; wait for the one after the move
        if abs(change.transform.pose_in_observer_frame.pose.theta + 90) < 1e-9:
            break
        change = await asyncio.wait_for(stream.__anext__(), 2)
    assert change.change_type == TransformChangeType.TRANSFORM_CHANGE_TYPE_UPDATED
    assert list(change.updated_fields.paths) == ["pose_in_observer_frame.pose"]
    rp = change.transform.pose_in_observer_frame.pose
    # room origin seen from the cart, then back: cart pose (x, y, yaw) applied to it must give (0, 0)
    yaw = math.radians(90)
    back = (1000 + math.cos(yaw) * rp.x - math.sin(yaw) * rp.y, 500 + math.sin(yaw) * rp.x + math.cos(yaw) * rp.y)
    assert math.dist(back, (0, 0)) < 1e-6 and abs(rp.theta + 90) < 1e-9, (rp, back)
    print(f"ok   cart moved -> room update streamed: room origin at ({rp.x:.0f}, {rp.y:.0f}) mm, {rp.theta:.0f} deg")
    await stream.aclose()
    await sc.close()


def test_room_pose_identity():
    p = room_pose({"x_mm": 0, "y_mm": 0, "yaw_deg": 0})
    assert (p.x, p.y, p.theta) == (0, 0, 0)


if __name__ == "__main__":
    test_plan_steps()
    test_room_pose_identity()
    asyncio.run(test_arm())
    asyncio.run(test_scene())
    print("test_cad PASS")
