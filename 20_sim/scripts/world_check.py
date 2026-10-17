"""GATE 7: the planner respects the room's obstacles.

The cart is teleported (odom set_pose) in front of an obstacle, facing it. Then, through
capbot_common.arm_move (which places the room around the cart from odom, decision #29), the cup is
sent with the tool pointing forward from the cart (a side approach; with the 491 mm tool pointing
down, the top of the 1 m table is out of the KR6's reach):
  table:   cup 150 mm above the table top        -> must be planned
           cup 200 mm inside the table           -> must be refused
  cabinet: cup 100 mm in front of its face        -> must be planned
           cup 150 mm inside the cabinet          -> must be refused
Targets are written in the room frame and turned into the cart frame (= Viam's world frame).
Each refusal is checked to be a collision, not "out of reach".

Run:  python3 20_sim/scripts/world_check.py
Pass: "world_check PASS".
"""

import asyncio

from viam.components.arm import Arm, JointPositions
from viam.components.generic import Generic

from capbot_common import arm_move, connect, numbers, pose, pose_in, room_point_in_cart

FORWARD = dict(ox=1, oy=0, oz=0, th=0)   # tool +Z along the cart's +X (in the cart frame)
STANDOFF_MM = 1000                        # cart centre to the obstacle face (cart front 500 mm from it): room for the wrist


async def try_move(robot, cart_pose, room_xyz):
    """True if planned and done; False with the planner's reason otherwise."""
    try:
        xyz = room_point_in_cart(room_xyz, cart_pose)
        return bool(await arm_move(robot, pose_in("world", pose(*xyz, **FORWARD)))), ""
    except Exception as e:
        return False, str(e).splitlines()[0][:200]


async def case(robot, odom, arm, home, cart_pose, label, room_xyz, want_planned):
    await odom.do_command({"command": "set_pose", **cart_pose})
    await arm.move_to_joint_positions(JointPositions(values=home))
    planned, why = await try_move(robot, cart_pose, room_xyz)
    reach = "too far" in why or "unreachable" in why
    good = planned if want_planned else (not planned and not reach)
    print(f"{'OK  ' if good else 'FAIL'} {label}: {'planned' if planned else 'refused'}")
    if why:
        print(f"     planner said: {why}")
    if not want_planned and reach:
        print("     -> refused for reach, not for the obstacle: pick a closer target")
    return good


async def main():
    n = numbers()
    obst = n["world"]["obstacles"]
    home = n["named_arm_poses_deg"]["home_folded"]
    t = obst["lab-table"]["planning_box_mm"]
    c = obst["lab-cabinet"]["planning_box_mm"]
    tx = (t["min"][0] + t["max"][0]) / 2
    cy = (c["min"][1] + c["max"][1]) / 2
    robot = await connect("sim")
    ok = True
    try:
        odom = Generic.from_robot(robot, "odom")  # sim odometry: pose / set_pose
        arm = Arm.from_robot(robot, "kr6")
        # table: approach from its -Y edge, cart facing +Y
        at_table = {"x_mm": tx, "y_mm": t["min"][1] - STANDOFF_MM, "yaw_deg": 90}
        ok &= await case(robot, odom, arm, home, at_table, "table, 150 mm above the top",
                         (tx, t["min"][1] + 100, t["max"][2] + 150), True)
        ok &= await case(robot, odom, arm, home, at_table, "table, 200 mm inside",
                         (tx, t["min"][1] + 100, t["max"][2] - 200), False)
        # cabinet: approach from its -X face, cart facing +X
        at_cab = {"x_mm": c["min"][0] - STANDOFF_MM, "y_mm": cy, "yaw_deg": 0}
        ok &= await case(robot, odom, arm, home, at_cab, "cabinet, 100 mm in front of it",
                         (c["min"][0] - 100, cy, 900), True)
        ok &= await case(robot, odom, arm, home, at_cab, "cabinet, 150 mm inside",
                         (c["min"][0] + 150, cy, 900), False)
        await odom.do_command({"command": "set_pose", "x_mm": 0, "y_mm": 0, "yaw_deg": 0})
        await arm.move_to_joint_positions(JointPositions(values=home))
    finally:
        await robot.close()
    print("world_check PASS" if ok else "world_check FAIL")


if __name__ == "__main__":
    asyncio.run(main())
