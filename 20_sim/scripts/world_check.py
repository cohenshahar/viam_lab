"""GATE 7: the planner respects the room's obstacles.

Puts the cart (odom teleport) in front of the lab table, then, through capbot_common.arm_move
(which places the room around the cart from odom, decision #29):
  1. asks for a cup pose just above the table top   -> must succeed
  2. asks for a cup pose inside the table           -> must be refused
  3. asks for a cup pose inside the cabinet's box   -> must be refused
Targets are written in the room frame and turned into the cart frame (= Viam's world frame).

Run:  python3 20_sim/scripts/world_check.py
Pass: "world_check PASS".
"""

import asyncio

from viam.components.arm import Arm, JointPositions
from viam.components.generic import Generic

from capbot_common import arm_move, connect, numbers, pose, pose_in, room_point_in_cart

DOWN = dict(ox=0, oy=0, oz=-1, th=0)


async def try_move(robot, cart_pose, room_xyz):
    try:
        xyz = room_point_in_cart(room_xyz, cart_pose)
        return bool(await arm_move(robot, pose_in("world", pose(*xyz, **DOWN))))
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
        odom = Generic.from_robot(robot, "odom")  # sim odometry: pose / set_pose
        arm = Arm.from_robot(robot, "kr6")
        # cart 650 mm in front of the table edge, facing it (+Y)
        cart_pose = {"x_mm": tx, "y_mm": front_y - 650, "yaw_deg": 90}
        await odom.do_command({"command": "set_pose", **cart_pose})
        await arm.move_to_joint_positions(JointPositions(values=n["named_arm_poses_deg"]["home_folded"]))
        above = (tx, front_y + 250, top + 150)
        inside = (tx, front_y + 250, top - 200)
        in_cab = ((cab["min_mm"][0] + cab["max_mm"][0]) / 2, (cab["min_mm"][1] + cab["max_mm"][1]) / 2, 700)
        r1 = await try_move(robot, cart_pose, above)
        print(f"{'OK  ' if r1 else 'FAIL'} 150 mm above the table: planned")
        r2 = await try_move(robot, cart_pose, inside)
        print(f"{'OK  ' if not r2 else 'FAIL'} inside the table: refused")
        r3 = await try_move(robot, cart_pose, in_cab)
        print(f"{'OK  ' if not r3 else 'FAIL'} inside the cabinet: refused")
        ok = r1 and not r2 and not r3
        await odom.do_command({"command": "set_pose", "x_mm": 0, "y_mm": 0, "yaw_deg": 0})
    finally:
        await robot.close()
    print("world_check PASS" if ok else "world_check FAIL")


if __name__ == "__main__":
    asyncio.run(main())
