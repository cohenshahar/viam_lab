"""Run the module's parts against stand-ins (no viam-server needed).

wheel-odom: stand-in motors are turned the way Viam's wheeled base turns them (the same
formulas as rdk components/base/wheeled/wheeled_base.go: MoveStraight and spinMath), in
small steps, and the odometry must follow a 1 m square and close it.

Run: python3 20_sim/modules/capbot_sim/tests/test_module.py
"""

import asyncio
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from google.protobuf.struct_pb2 import Struct  # noqa: E402
from viam.components.motor import Motor  # noqa: E402
from viam.proto.app.robot import ComponentConfig  # noqa: E402

from vacuum import Vacuum  # noqa: E402
from wheel_odom import WheelOdom  # noqa: E402

WIDTH_MM, CIRC_MM = 478, 1125


class StandInMotor:
    def __init__(self):
        self.rev = 0.0

    async def get_position(self, **kw):
        return self.rev


def config(name, attrs):
    s = Struct()
    s.update(attrs)
    return ComponentConfig(name=name, attributes=s)


# Viam's wheeled base: wheel revolutions for each move (left, right)
def straight_rev(distance_mm):
    r = distance_mm / CIRC_MM
    return r, r


def spin_rev(angle_deg):
    r = abs(WIDTH_MM * math.pi * (angle_deg / 360.0) / CIRC_MM)
    s = 1.0 if angle_deg > 0 else -1.0
    return -s * r, s * r          # runAllGoFor(-rpm, rev, rpm, rev): + angle = turn left


async def roll(odom, left, right, d_rev, steps=37):
    for i in range(steps):
        left.rev += d_rev[0] / steps
        right.rev += d_rev[1] / steps
        await odom._tick()


async def run():
    attrs = {"left_motor": "left", "right_motor": "right",
             "width_mm": WIDTH_MM, "wheel_circumference_mm": CIRC_MM}
    assert WheelOdom.validate_config(config("odom", attrs)) == (["left", "right"], [])
    for bad in ({}, {**attrs, "left_motor": ""}, {**attrs, "width_mm": -1}, {**attrs, "tick_hz": "fast"}):
        try:
            WheelOdom.validate_config(config("odom", bad))
            raise AssertionError(f"accepted bad config {bad}")
        except ValueError as e:
            print("ok   rejects bad config:", e)

    left, right = StandInMotor(), StandInMotor()
    deps = {Motor.get_resource_name("left"): left, Motor.get_resource_name("right"): right}
    odom = WheelOdom.new(config("odom", {**attrs, "tick_hz": 1000}), deps)
    odom.task.cancel()  # the test ticks by hand
    await odom._tick()

    await roll(odom, left, right, straight_rev(1000))
    p = await odom.do_command({"command": "pose"})
    assert abs(p["x_mm"] - 1000) < 1e-6 and abs(p["y_mm"]) < 1e-6, p
    await roll(odom, left, right, spin_rev(90))
    p = await odom.do_command({"command": "pose"})
    assert abs(p["yaw_deg"] - 90) < 1e-6 and abs(p["x_mm"] - 1000) < 1e-6, p
    print("ok   1 m forward, then spin +90 = left ->", p)
    for _ in range(3):
        await roll(odom, left, right, straight_rev(1000))
        await roll(odom, left, right, spin_rev(90))
    p = await odom.do_command({"command": "pose"})
    assert math.hypot(p["x_mm"], p["y_mm"]) < 1e-6 and abs((p["yaw_deg"] + 180) % 360 - 180) < 1e-6, p
    print("ok   1 m square closes ->", p)

    await roll(odom, left, right, straight_rev(-500))
    p = await odom.do_command({"command": "pose"})
    assert abs(p["x_mm"] + 500) < 1e-6, p
    print("ok   backward 500 mm ->", p)

    assert (await odom.do_command({"command": "set_pose", "x_mm": 10, "y_mm": 20, "yaw_deg": 30}))["ok"]
    await roll(odom, left, right, (0.0, 0.0), steps=3)
    p = await odom.do_command({"command": "pose"})
    assert (p["x_mm"], p["y_mm"], p["yaw_deg"]) == (10, 20, 30), p
    print("ok   set_pose teleports, wheels untouched")
    print("ok   unknown verb ->", await odom.do_command({"command": "fly"}))
    await odom.close()

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
    print("test_module PASS")
