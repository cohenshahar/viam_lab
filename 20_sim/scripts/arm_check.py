"""GATE 3 (+ spikes S2, S3): the KR6 in capbot-sim.

1. Joint moves: zero, home_arm_up, home_folded, one random pose. After each, Viam's
   get_end_position must match our own forward kinematics of kr6.json (< 0.1 mm, < 0.01 deg).
   This proves viam-server reads kr6.json the way we wrote it.
2. The cart frame is Viam's world frame (decision #29; the old cart trick S2 is archived).
3. Planner (S3): arm_move the vacuum to 3 poses in the cart frame, with the cart at two places in
   the room (room obstacles placed around it from odom). Each pose reached; odom unchanged.

Run:  python3 20_sim/scripts/arm_check.py
Pass: every line says OK and the last line says "arm_check PASS".
"""

import asyncio
import math
import random

import numpy as np
from viam.components.arm import Arm, JointPositions
from viam.components.generic import Generic

from capbot_common import arm_move, connect, fmt_pose, kinematics, numbers, pose, pose_in
import viam_frames as vf  # in 20_sim/tools; capbot_common puts that folder on the path

POS_TOL, ANG_TOL = 0.1, 0.01


def ov_angle(p, r_expected):
    r = vf.ov_to_matrix(p.o_x, p.o_y, p.o_z, p.theta)
    c = (np.trace(r.T @ r_expected) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


async def main():
    n, kr6_model = numbers(), kinematics("kr6")
    robot = await connect("sim")
    ok = True
    try:
        arm = Arm.from_robot(robot, "kr6")
        odom = Generic.from_robot(robot, "odom")  # sim odometry: pose / set_pose

        # 1. joints vs our kinematics
        poses = dict(zero=[0.0] * 6, **{k: v for k, v in n["named_arm_poses_deg"].items() if k != "source"})
        lims = [(j["min"], j["max"]) for j in kr6_model["joints"]]
        rnd = random.Random(15)
        poses["random"] = [round(rnd.uniform(lo * 0.5, hi * 0.5), 2) for lo, hi in lims]
        for name, q in poses.items():
            await arm.move_to_joint_positions(JointPositions(values=q))
            got = await arm.get_end_position()
            want = vf.sva_fk(kr6_model, q)["flange"]
            dp = float(np.linalg.norm(np.array([got.x, got.y, got.z]) - want[:3, 3]))
            da = ov_angle(got, want[:3, :3])
            good = dp < POS_TOL and da < ANG_TOL
            ok &= good
            print(f"{'OK  ' if good else 'FAIL'} joints {name:12s} pos err {dp:.4f} mm, angle err {da:.4f} deg | {fmt_pose(got)}")

        # 2. Viam's world frame is the cart (decision #29): the cart frame sits at the world origin
        await arm.move_to_joint_positions(JointPositions(values=n["named_arm_poses_deg"]["home_folded"]))
        cp = (await robot.get_pose("cart", "world")).pose
        good = math.dist((cp.x, cp.y, cp.z), (0, 0, 0)) < 1e-6 and abs(cp.theta) < 1e-6
        ok &= good
        print(f"{'OK  ' if good else 'FAIL'} cart frame = world frame | {fmt_pose(cp)}")

        # 3. planner (S3), through arm_move: suction cup pointing down, targets in the cart frame.
        #    Same targets with the cart at two places in the room: the arm result must not depend on it,
        #    and odom (the cart's place in the room) must not change.
        targets = [(700, 0, 700), (600, 300, 600), (550, -300, 650)]
        for place in ({"x_mm": 0, "y_mm": 0, "yaw_deg": 0}, {"x_mm": -1500, "y_mm": -800, "yaw_deg": 135}):
            await odom.do_command({"command": "set_pose", **place})
            before = await odom.do_command({"command": "pose"})
            for t in targets:
                try:
                    moved = await arm_move(robot, pose_in("world", pose(*t, ox=0, oy=0, oz=-1, th=0)))
                except Exception as e:
                    moved = False
                    print(f"     planner said: {str(e).splitlines()[0][:200]}")
                got = (await robot.get_pose("vacuum", "world")).pose
                dp = math.dist((got.x, got.y, got.z), t)
                good = bool(moved) and dp < 1.0
                ok &= good
                print(f"{'OK  ' if good else 'FAIL'} cart at room ({place['x_mm']}, {place['y_mm']}, {place['yaw_deg']} deg): "
                      f"vacuum to {t}: error {dp:.3f} mm")
            after = await odom.do_command({"command": "pose"})
            still = all(abs(before[k] - after[k]) < 1e-6 for k in ("x_mm", "y_mm", "yaw_deg"))
            ok &= still
            print(f"{'OK  ' if still else 'FAIL'} the arm moves left the cart where it was ({after['x_mm']}, {after['y_mm']}, {after['yaw_deg']})")
        await odom.do_command({"command": "set_pose", "x_mm": 0, "y_mm": 0, "yaw_deg": 0})
    finally:
        await robot.close()
    print("arm_check PASS" if ok else "arm_check FAIL")


if __name__ == "__main__":
    asyncio.run(main())
