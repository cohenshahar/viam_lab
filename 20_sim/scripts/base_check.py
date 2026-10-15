"""GATE 4: drive the sim cart around a 1 m square and check it ends where it started.

Run:  python3 20_sim/scripts/base_check.py
Pass: "base_check PASS" (back at start within 1 mm and 0.1 deg; the frame system agrees).
"""

import asyncio
import math

from viam.components.base import Base

from capbot_common import connect

SIDE_MM, SPEED_MM_S, TURN_DEG, TURN_DEG_S = 1000, 250, 90, 45


async def main():
    robot = await connect("sim")
    try:
        cart = Base.from_robot(robot, "cart")
        await cart.do_command({"command": "set_pose", "x_mm": 0, "y_mm": 0, "yaw_deg": 0})
        for i in range(4):
            await cart.move_straight(SIDE_MM, SPEED_MM_S)
            await cart.spin(TURN_DEG, TURN_DEG_S)
            p = await cart.do_command({"command": "pose"})
            print(f"side {i + 1}: {p}")
        p = await cart.do_command({"command": "pose"})
        fs = (await robot.get_pose("cart", "world")).pose
        d = math.hypot(p["x_mm"], p["y_mm"])
        yaw = abs((p["yaw_deg"] + 180) % 360 - 180)
        agree = math.hypot(fs.x - p["x_mm"], fs.y - p["y_mm"]) < 0.5
        print(f"end error {d:.3f} mm, {yaw:.4f} deg; frame system says x={fs.x:.2f} y={fs.y:.2f}")
        ok = d < 1.0 and yaw < 0.1 and agree
        print("base_check PASS" if ok else "base_check FAIL")
    finally:
        await robot.close()


if __name__ == "__main__":
    asyncio.run(main())
