"""Check that 20_sim/kinematics/kr6.json moves exactly like the thesis MuJoCo arm.

For random joint angles inside the limits, compare the flange and the suction cup
(TCP) pose from our Viam kinematics file with MuJoCo's own forward kinematics.

Run:
    export CAP_THESIS_DIR=/path/to/CaP_Thesis
    python3 20_sim/tools/check_fk.py            # 50 random poses
Pass: worst position error < 0.1 mm and worst angle error < 0.01 deg.
"""

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_viam_files as bvf  # noqa: E402
import viam_frames as vf  # noqa: E402

N_POSES = 50
POS_TOL_MM = 0.1
ANG_TOL_DEG = 0.01


def angle_between(r1, r2):
    c = (np.trace(r1.T @ r2) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def mujoco_pose(mdl, body):
    b = mdl.body(body)
    return vf.pose(mdl.d.xmat[b].reshape(3, 3), mdl.d.xpos[b] * bvf.MM)


def main():
    cap = bvf.thesis_dir()
    mdl = bvf.Model(cap / bvf.WORLD_REL)
    kr6 = json.loads((bvf.REPO / "20_sim/kinematics/kr6.json").read_text())
    numbers = json.loads((bvf.REPO / "20_sim/generated/robot_numbers.json").read_text())
    tcp = np.array(numbers["viam_frames"]["vacuum_translation_mm"])
    flange_fix = vf.pose(vf.rot_y(math.pi / 2))
    s = mdl.id(mujoco.mjtObj.mjOBJ_SITE, bvf.TCP_SITE)

    rng = np.random.default_rng(15102026)
    joints = [mdl.id(mujoco.mjtObj.mjOBJ_JOINT, j) for _, j in bvf.ARM_CHAIN[1:]]
    worst_p = worst_a = worst_tcp = 0.0
    for i in range(N_POSES):
        q = np.array([rng.uniform(*mdl.m.jnt_range[j]) for j in joints])
        if i == 0:
            q[:] = 0.0  # the zero pose is always checked
        for j, v in zip(joints, q):
            mdl.d.qpos[mdl.m.jnt_qposadr[j]] = v
        mujoco.mj_forward(mdl.m, mdl.d)
        base = mujoco_pose(mdl, "kuka_base")
        flange_mj = np.linalg.inv(base) @ mujoco_pose(mdl, "link_6") @ flange_fix
        tcp_mj = (np.linalg.inv(base) @ np.append(mdl.d.site_xpos[s] * bvf.MM, 1))[:3]

        flange_v = vf.sva_fk(kr6, list(np.degrees(q)))["flange"]
        tcp_v = (flange_v @ np.append(tcp, 1))[:3]

        worst_p = max(worst_p, np.linalg.norm(flange_v[:3, 3] - flange_mj[:3, 3]))
        worst_a = max(worst_a, angle_between(flange_v[:3, :3], flange_mj[:3, :3]))
        worst_tcp = max(worst_tcp, np.linalg.norm(tcp_v - tcp_mj))
        if i == 0:
            print("zero pose flange (mm):", np.round(flange_v[:3, 3], 3), " TCP (mm):", np.round(tcp_v, 3))

    ok = worst_p < POS_TOL_MM and worst_tcp < POS_TOL_MM and worst_a < ANG_TOL_DEG
    print(f"{N_POSES} poses | worst flange position {worst_p:.6f} mm | worst TCP position {worst_tcp:.6f} mm"
          f" | worst angle {worst_a:.6f} deg")
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
