"""Test capbot:sim:motor (no viam-server needed).

Run: python3 20_sim/modules/capbot_sim/tests/test_sim_motor.py
"""

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from google.protobuf.struct_pb2 import Struct  # noqa: E402
from viam.proto.app.robot import ComponentConfig  # noqa: E402

from sim_motor import SimMotor, shape_power  # noqa: E402


def config(attrs):
    s = Struct()
    s.update(attrs)
    return ComponentConfig(name="left", attributes=s)


def test_power_rule():
    assert shape_power(0.1, 0.27, 0.32) == 0.27 and shape_power(-0.5, 0.27, 0.32) == -0.32
    assert shape_power(0.0, 0.27, 0.32) == 0.0 and shape_power(0.5, None, None) == 0.5
    print("ok   power rule like the real GPIO motor (raise to min, clip to max)")


async def test_motor():
    assert SimMotor.validate_config(config({"max_rpm": 600})) == ([], [])
    for bad in ({}, {"max_rpm": 0}, {"max_rpm": 60, "min_power_pct": 0.5, "max_power_pct": 0.3}):
        try:
            SimMotor.validate_config(config(bad))
            raise AssertionError(f"accepted {bad}")
        except ValueError as e:
            print("ok   rejects bad config:", e)
    m = SimMotor.new(config({"max_rpm": 600, "tick_hz": 200}), {})
    run = asyncio.ensure_future(m.go_for(300, 2.0))     # 2 rev at 5 rev/s = 0.4 s
    await asyncio.sleep(0.2)
    mid = await m.get_position()
    assert 0.5 < mid < 1.5 and await m.is_moving(), mid
    await run
    assert await m.get_position() == 2.0 and not await m.is_moving()
    print(f"ok   go_for counts while running (mid-way {mid:.2f} rev) and lands exactly on 2.0 rev")
    await m.go_for(-300, 1.0)
    assert await m.get_position() == 1.0
    await m.go_to(300, 1.5)
    assert await m.get_position() == 1.5
    print("ok   backwards (negative rpm) and go_to")
    await m.set_rpm(60)
    await asyncio.sleep(0.1)
    await m.stop()
    p = await m.get_position()
    assert 1.55 < p < 1.65 and not await m.is_moving(), p  # 0.1 s at 1 rev/s, plus sleep jitter
    print(f"ok   set_rpm then stop -> {p:.3f} rev")
    await m.close()


if __name__ == "__main__":
    test_power_rule()
    asyncio.run(test_motor())
    print("test_sim_motor PASS")
