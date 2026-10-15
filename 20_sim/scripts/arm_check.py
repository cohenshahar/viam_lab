"""GATE 3 (+ spikes S2, S3): the KR6 in capbot-sim.

1. Joint moves: zero, home_arm_up, home_folded, one random pose. After each, Viam's
   get_end_position must match our own forward kinematics of kr6.json (< 0.1 mm, < 0.01 deg).
   This proves viam-server reads kr6.json the way we wrote it.
2. Cart trick (S2): teleport the cart; the vacuum must move in the world by the same amount.
3. Planner (S3): motion.move the vacuum to 3 poses given in frame "cart"; the cart must not move.

Run:  python3 20_sim/scripts/arm_check.py
Pass: every line says OK and the last line says "arm_check PASS".
"""

import asyncio
import math
import random

import numpy as np
import viam_frames as vf
from viam.components.arm import Arm, JointPositions
from viam.components.base import Base
from viam.services.motion import MotionClient

from capbot_common import connect, fmt_pose, kinematics, numbers, pose, pose_in

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
        cart = Base.from_robot(robot, "cart")
        motion = MotionClient.from_robot(robot, "builtin")

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

        # 2. cart trick
        await arm.move_to_joint_positions(JointPositions(values=n["named_arm_poses_deg"]["home_folded"]))
        await cart.do_command({"command": "set_pose", "x_mm": 0, "y_mm": 0, "yaw_deg": 0})
        a = (await robot.get_pose("vacuum", "world")).pose
        await cart.do_command({"command": "set_pose", "x_mm": 1000, "y_mm": 500, "yaw_deg": 90})
        await asyncio.sleep(0.3)
        b = (await robot.get_pose("vacuum", "world")).pose
        # vacuum position relative to the cart must be the same, rotated by 90 deg about z
        rel_a = np.array([a.x, a.y])
        rel_b = np.array([b.x - 1000, b.y - 500])
        expect = np.array([-rel_a[1], rel_a[0]])
        err = float(np.linalg.norm(rel_b - expect)) + abs(b.z - a.z)
        good = err < 0.5
        ok &= good
        print(f"{'OK  ' if good else 'FAIL'} cart trick: vacuum follows the cart (error {err:.3f} mm)")
        await cart.do_command({"command": "set_pose", "x_mm": 0, "y_mm": 0, "yaw_deg": 0})

        # 3. planner, targets in the cart frame: suction cup pointing down (+Z of the tool = -Z of the cart)
        before = await cart.do_command({"command": "pose"})
        targets = [(700, 0, 700), (600, 300, 600), (650, -250, 800)]
        for t in targets:
            moved = await motion.move("vacuum", pose_in("cart", pose(*t, ox=0, oy=0, oz=-1, th=0)))
            got = (await robot.get_pose("vacuum", "cart")).pose
            dp = math.dist((got.x, got.y, got.z), t)
            good = bool(moved) and dp < 1.0
            ok &= good
            print(f"{'OK  ' if good else 'FAIL'} motion.move vacuum to {t} in 'cart': error {dp:.3f} mm")
        after = await cart.do_command({"command": "pose"})
        still = all(abs(before[k] - after[k]) < 1e-6 for k in before)
        ok &= still
        print(f"{'OK  ' if still else 'FAIL'} the planner left the cart where it was ({after})")
        if not still:
            print("     -> S3 failed: the planner moved cart-pose. Write it in the log; see checklist Phase 1.")
    finally:
        await robot.close()
    print("arm_check PASS" if ok else "arm_check FAIL")


if __name__ == "__main__":
    asyncio.run(main())
