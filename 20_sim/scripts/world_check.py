"""GATE 7: the planner respects the lab-world obstacles.

Puts the cart in front of the lab table, then
  1. asks for a cup pose just above the table top   -> must succeed
  2. asks for a cup pose inside the table           -> must be refused
  3. asks for a cup pose inside the cabinet's box   -> must be refused
Needs the lab-world fragment on capbot-sim.

Run:  python3 20_sim/scripts/world_check.py
Pass: "world_check PASS".
"""

import asyncio

from viam.components.arm import Arm, JointPositions
from viam.components.base import Base
from viam.services.motion import MotionClient

from capbot_common import connect, numbers, pose, pose_in

DOWN = dict(ox=0, oy=0, oz=-1, th=0)


async def try_move(motion, frame, xyz):
    try:
        return bool(await motion.move("vacuum", pose_in(frame, pose(*xyz, **DOWN))))
    except Exception as e:
        print(f"     planner said: {str(e).splitlines()[0][:160]}")
        return False


async def main():
    n = numbers()
    table = n["world"]["obstacles"]["lab-table"]
    cab = n["world"]["obstacles"]["lab-cabinet"]
    tx = (table["min_mm"][0] + table["max_mm"][0]) / 2
    top = table["max_mm"][2]
    front_y = table["min_mm"][1]
    robot = await connect("sim")
    ok = True
    try:
        cart = Base.from_robot(robot, "cart")
        arm = Arm.from_robot(robot, "kr6")
        motion = MotionClient.from_robot(robot, "builtin")
        names = {r.name for r in robot.resource_names}
        if "lab-table" not in names:
            print("FAIL the lab-world fragment is not on this machine (no 'lab-table')")
            return
        # cart 650 mm in front of the table edge, facing it (+Y)
        await cart.do_command({"command": "set_pose", "x_mm": tx, "y_mm": front_y - 650, "yaw_deg": 90})
        await arm.move_to_joint_positions(JointPositions(values=n["named_arm_poses_deg"]["home_folded"]))
        above = (tx, front_y + 250, top + 150)
        inside = (tx, front_y + 250, top - 200)
        in_cab = ((cab["min_mm"][0] + cab["max_mm"][0]) / 2, (cab["min_mm"][1] + cab["max_mm"][1]) / 2, 700)
        r1 = await try_move(motion, "world", above)
        print(f"{'OK  ' if r1 else 'FAIL'} 150 mm above the table: planned")
        r2 = await try_move(motion, "world", inside)
        print(f"{'OK  ' if not r2 else 'FAIL'} inside the table: refused")
        r3 = await try_move(motion, "world", in_cab)
        print(f"{'OK  ' if not r3 else 'FAIL'} inside the cabinet: refused")
        ok = r1 and not r2 and not r3
    finally:
        await robot.close()
    print("world_check PASS" if ok else "world_check FAIL")


if __name__ == "__main__":
    asyncio.run(main())
