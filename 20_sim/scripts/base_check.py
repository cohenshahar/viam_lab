"""GATE 4: drive the sim cart around a 1 m square and check it ends where it started.

The cart is Viam's wheeled base on motors left / right; odom works out the pose from the wheel turns.
Also checks the wheel turns against the wheeled-base math: each wheel rolls 4 x 1000 mm forward, and
each 90 deg spin adds +/- width * pi / 4 (left backward, right forward).

Run:  python3 20_sim/scripts/base_check.py
Pass: "base_check PASS" (back at start within 1 mm and 0.1 deg; wheel turns within 0.01 rev).
"""

import asyncio
import math

from viam.components.base import Base
from viam.components.generic import Generic

from capbot_common import connect, numbers

SIDE_MM, SPEED_MM_S, TURN_DEG, TURN_DEG_S = 1000, 250, 90, 45


async def main():
    robot = await connect("sim")
    try:
        cart = Base.from_robot(robot, "cart")
        odom = Generic.from_robot(robot, "odom")  # sim odometry: pose / set_pose
        await odom.do_command({"command": "set_pose", "x_mm": 0, "y_mm": 0, "yaw_deg": 0})
        start = await odom.do_command({"command": "pose"})
        for i in range(4):
            await cart.move_straight(SIDE_MM, SPEED_MM_S)
            await cart.spin(TURN_DEG, TURN_DEG_S)
            p = await odom.do_command({"command": "pose"})
            print(f"side {i + 1}: {p}")
        p = await odom.do_command({"command": "pose"})
        d = math.hypot(p["x_mm"], p["y_mm"])
        yaw = abs((p["yaw_deg"] + 180) % 360 - 180)
        c = numbers()["cart"]
        width, circ = round(c["track_width_mm"]), round(c["drive_wheel_circumference_mm"])
        spin = width * math.pi * (TURN_DEG / 360.0) / circ
        want = (4 * SIDE_MM / circ - 4 * spin, 4 * SIDE_MM / circ + 4 * spin)
        got = (p["left_rev"] - start["left_rev"], p["right_rev"] - start["right_rev"])
        wheels_ok = all(abs(g - w) < 0.01 for g, w in zip(got, want))
        print(f"end error {d:.3f} mm, {yaw:.4f} deg; wheel turns left {got[0]:.4f} / right {got[1]:.4f} rev "
              f"(expected {want[0]:.4f} / {want[1]:.4f})")
        ok = d < 1.0 and yaw < 0.1 and wheels_ok
        print("base_check PASS" if ok else "base_check FAIL")
    finally:
        await robot.close()


if __name__ == "__main__":
    asyncio.run(main())
