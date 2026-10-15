"""Run the sim base against a stand-in arm (no viam-server needed).

Run: python3 20_sim/modules/capbot_sim/tests/test_sim_base.py
"""

import asyncio
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from google.protobuf.struct_pb2 import Struct  # noqa: E402
from viam.components.arm import Arm  # noqa: E402
from viam.proto.app.robot import ComponentConfig  # noqa: E402
from viam.proto.common import Vector3  # noqa: E402

from sim_base import SimBase  # noqa: E402
from vacuum import Vacuum  # noqa: E402


class StandInArm:
    def __init__(self):
        self.joints = None

    async def move_to_joint_positions(self, jp, **kw):
        self.joints = list(jp.values)


def config(name, attrs):
    s = Struct()
    s.update(attrs)
    return ComponentConfig(name=name, attributes=s)


async def run():
    attrs = {"pose_arm": "cart-pose", "width_mm": 478, "wheel_circumference_mm": 1124.7, "tick_hz": 100}
    assert SimBase.validate_config(config("cart", attrs)) == (["cart-pose"], [])
    for bad in ({}, {"pose_arm": "x"}, {"pose_arm": "x", "width_mm": -1, "wheel_circumference_mm": 1}):
        try:
            SimBase.validate_config(config("cart", bad))
            raise AssertionError(f"accepted bad config {bad}")
        except ValueError as e:
            print("ok   rejects bad config:", e)
    arm = StandInArm()
    base = SimBase.new(config("cart", attrs), {Arm.get_resource_name("cart-pose"): arm})
    await base.move_straight(300, 300)
    await base.spin(90, 90)
    await base.move_straight(-200, 400)
    p = await base.do_command({"command": "pose"})
    assert abs(p["x_mm"] - 300) < 0.5 and abs(p["y_mm"] + 200) < 0.5 and abs(p["yaw_deg"] - 90) < 0.01, p
    assert arm.joints is not None and abs(arm.joints[0] - p["x_mm"]) < 1e-6, arm.joints
    print("ok   straight / spin / backward ->", p, "arm joints", [round(j, 3) for j in arm.joints])
    await base.set_velocity(Vector3(y=200), Vector3(z=0))
    await asyncio.sleep(0.2)
    assert await base.is_moving()
    await base.stop()
    assert not await base.is_moving()
    print("ok   set_velocity then stop")
    print("ok   unknown verb ->", await base.do_command({"command": "fly"}))
    await base.close()

    vac = Vacuum.new(config("vacuum", {"tool_box_mm": {"x": 70, "y": 70, "z": 476,
                                                        "translation": {"x": 0, "y": 0, "z": -243}}}), {})
    assert (await vac.do_command({"command": "seal"}))["seal"] == 0.0
    assert await vac.grab() and (await vac.is_holding_something()).is_holding_something
    await vac.open()
    assert not (await vac.is_holding_something()).is_holding_something
    fmt, data = await vac.get_kinematics()
    assert b"vacuum_tool" in data
    print("ok   vacuum grab / open / seal / kinematics")


if __name__ == "__main__":
    asyncio.run(run())
    print("test_sim_base PASS")
