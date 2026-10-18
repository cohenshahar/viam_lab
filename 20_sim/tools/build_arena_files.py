"""Build the Viam files for the arm arena (the arm-table rig) from the thesis MuJoCo model.

The arena: one KR6 with the vacuum tool on the 2 x 2 m table inside the steel cage, and one box
(CaP_Thesis sim/rig/scene/world.xml). Fixed arm: Viam's world frame = the rig's world (centre of the
table on the floor), so the room's obstacles can stay in the machine config.

Run:
    CAP_THESIS_DIR=/path/to/CaP_Thesis python3 20_sim/tools/build_arena_files.py

Checks first that the rig's KR6 is the same arm as 20_sim/kinematics/kr6.json (forward kinematics,
50 random joint sets, < 0.1 mm / 0.01 deg); stops if not.

Writes (generated, never edited by hand):
    20_sim/generated/arena/arena_numbers.json   arm mount, obstacles (with margin), the box, the place spot
    20_sim/generated/arena/visuals/             room.ply (table, fence, cage, work area) + tool.ply + visuals.json
                                                (the KR6 link meshes are shared with 20_sim/generated/visuals/kr6)
"""

import json
import math
import shutil
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_viam_files as bvf  # noqa: E402
import viam_frames as vf  # noqa: E402
import visual_meshes as vm  # noqa: E402

REPO = bvf.REPO
RIG_REL = "sim/rig/scene/world.xml"
OUT = REPO / "20_sim/generated/arena"
MM = bvf.MM
MARGIN_MM = bvf.MARGIN_MM
BOX_BODY = "sealed_box"
# Demo choice (not a rig number): where the box is put down, on the table inside the work area
PLACE_SPOT_XY_MM = (-100.0, -250.0)

# Shahar (18 Oct): keep three rig cameras - the roof, the front and the right one. The rig file calls the arm's corner
# (-x, -y) the back left, so front = +y, right = +x. MuJoCo camera name: Viam camera name.
CAMERAS = {"cam_top": "cam-roof", "cam_side_py": "cam-front", "cam_side_px": "cam-right"}

OBSTACLES = {  # Viam name: (geom names, keep the top exact?)
    "tabletop": (["tabletop"], True),
    "fence-front": (["fence_ny"], False), "fence-back": (["fence_py"], False),
    "fence-left": (["fence_nx"], False), "fence-right": (["fence_px"], False),
    "post-fl": (["post_high_fl"], False), "post-fr": (["post_high_fr"], False),
    "post-bl": (["post_high_bl"], False), "post-br": (["post_high_br"], False),
}


def check_same_arm(mdl):
    """The rig's KR6 vs our kr6.json: link_6 pose in the arm base frame at 50 random joint sets."""
    m, d = mdl.m, mdl.d
    kr6 = json.loads((REPO / "20_sim/kinematics/kr6.json").read_text())
    names = [j["id"] for j in kr6["joints"]]
    adr = [m.jnt_qposadr[mdl.id(mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
    rng = np.random.default_rng(18)
    worst_p = worst_a = 0.0
    base, l6 = mdl.body("kuka_base"), mdl.body("link_6")
    for _ in range(50):
        q = [rng.uniform(j["min"], j["max"]) * 0.8 for j in kr6["joints"]]
        d.qpos[:] = m.qpos0
        for a, v in zip(adr, q):
            d.qpos[a] = math.radians(v)
        mujoco.mj_kinematics(m, d)
        tb = vf.pose(d.xmat[base].reshape(3, 3), d.xpos[base] * MM)
        tl = vf.pose(d.xmat[l6].reshape(3, 3), d.xpos[l6] * MM)
        mj = np.linalg.inv(tb) @ tl
        ours = vf.sva_fk(kr6, q)["link_6"]
        worst_p = max(worst_p, float(np.linalg.norm(mj[:3, 3] - ours[:3, 3])))
        c = (np.trace(mj[:3, :3].T @ ours[:3, :3]) - 1) / 2
        worst_a = max(worst_a, math.degrees(math.acos(max(-1.0, min(1.0, c)))))
    d.qpos[:] = m.qpos0
    mujoco.mj_forward(m, d)
    ok = worst_p < 0.1 and worst_a < 0.01
    print(f"rig KR6 vs kr6.json: worst {worst_p:.4f} mm, {worst_a:.5f} deg -> {'same arm' if ok else 'DIFFERENT'}")
    if not ok:
        sys.exit("The rig's KR6 is not the arm in kr6.json; the arena needs its own kinematics file.")


def main():
    cap = bvf.thesis_dir()
    mdl = bvf.Model(cap / RIG_REL)
    m, d = mdl.m, mdl.d
    check_same_arm(mdl)

    base = mdl.body("kuka_base")
    if not np.allclose(d.xquat[base], [1, 0, 0, 0], atol=1e-9):
        raise ValueError("the rig's KR6 base is rotated; add orientation support first")
    mount = d.xpos[base] * MM

    obstacles = {}
    for name, (geoms, exact_top) in OBSTACLES.items():
        lo, hi = mdl.world_box([mdl.id(mujoco.mjtObj.mjOBJ_GEOM, g) for g in geoms])
        glo, ghi = lo - MARGIN_MM, hi + MARGIN_MM
        if exact_top:
            ghi[2] = hi[2]
        obstacles[name] = {"min_mm": bvf.r3(lo), "max_mm": bvf.r3(hi), "margin_mm": MARGIN_MM,
                           "planning_box_mm": {"min": bvf.r3(glo), "max": bvf.r3(ghi)}}
    table_top = obstacles["tabletop"]["max_mm"][2]

    # the box: its size from its geom, resting on the table at its default x, y
    bb = mdl.body(BOX_BODY)
    g = [i for i in range(m.ngeom) if m.geom_bodyid[i] == bb][0]
    size = [round(float(v), 1) for v in m.geom_size[g] * 2 * MM]
    yaw = math.degrees(2 * math.atan2(d.xquat[bb][3], d.xquat[bb][0]))
    start = {"x_mm": round(float(d.xpos[bb][0] * MM), 1), "y_mm": round(float(d.xpos[bb][1] * MM), 1),
             "z_mm": round(table_top + size[2] / 2, 1), "yaw_deg": round(yaw, 2)}
    spot = {"type": "surface", "x_mm": PLACE_SPOT_XY_MM[0], "y_mm": PLACE_SPOT_XY_MM[1],
            "z_mm": round(table_top + size[2] / 2, 1), "yaw_deg": 0.0}

    cameras = {}
    for mj_name, viam_name in CAMERAS.items():
        c = mdl.id(mujoco.mjtObj.mjOBJ_CAMERA, mj_name)
        r = bvf.camera_viam_rotation(d.cam_xmat[c].reshape(3, 3))
        cameras[viam_name] = {"translation_mm": bvf.r3(d.cam_xpos[c] * MM), "orientation": vf.ov_json(r),
                              "fovy_deg": round(float(m.cam_fovy[c]), 1), "source": f"rig camera {mj_name}"}

    numbers = {
        "_about": "GENERATED by 20_sim/tools/build_arena_files.py from the thesis rig model. Do not edit. mm and degrees. "
                  "Viam world frame = the rig world: centre of the table, on the floor.",
        "source": f"{RIG_REL} (copied from vla-sim-playground scene_v2), compiled with MuJoCo {mujoco.__version__}",
        "kr6_mount_mm": bvf.r3(mount),
        "cameras": cameras,
        "table_top_mm": table_top,
        "obstacles": obstacles,
        "boxes": {"box1": {"size_mm": size, "start": start, "source": f"rig body {BOX_BODY}"}},
        "places": {},   # no cart: no named cart places
        "box_spots": {"place_spot": spot, "start_spot": {"type": "surface", **start}},
        "pick_station": {k: start[k] for k in ("x_mm", "y_mm", "z_mm")},
        "note": "place_spot is a demo choice (PLACE_SPOT_XY_MM in build_arena_files.py), not a rig number.",
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "arena_numbers.json").write_text(json.dumps(numbers, indent=2) + "\n", encoding="utf-8")

    # CAD look: the static rig (table, fence, cage, work area) in the world frame; the tool and KR6 are shared
    vis = OUT / "visuals"
    vis.mkdir(parents=True, exist_ok=True)
    skip = set(mdl.subtree("kuka_base")) | {bb} | {mdl.body(n) for n in ("crate", "ball") if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) >= 0}
    # camera mounts: only the three kept cameras are drawn
    kept = {m.cam_bodyid[mdl.id(mujoco.mjtObj.mjOBJ_CAMERA, n)] for n in CAMERAS}
    for c in range(m.ncam):
        b = m.cam_bodyid[c]
        if b != 0 and b not in kept:
            skip |= set(mdl.subtree(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)))
    room_bodies = [b for b in range(m.nbody) if b not in skip]
    parts = [p for p in (bvf.geom_part(mdl, g, "world") for g in bvf.visible_geoms(mdl, room_bodies)) if p]
    size_room = vm.write_ply(parts, vis / "room.ply")
    main_vis = REPO / "20_sim/generated/visuals"
    shutil.copyfile(main_vis / "tool.ply", vis / "tool.ply")
    main_manifest = json.loads((main_vis / "visuals.json").read_text())
    manifest = {"_about": "GENERATED by build_arena_files.py. Metres. KR6 meshes shared with 20_sim/generated/visuals.",
                "kr6": {k: {**v, "file": "../../visuals/" + v["file"]} for k, v in main_manifest["kr6"].items()},
                "shapes": [{"name": "room", "file": "room.ply", "parent": "world", "bytes": size_room,
                            "faces": int(sum(len(p[1]) for p in parts))},
                           {"name": "tool", "file": "tool.ply", "parent": "vacuum",
                            "bytes": (vis / "tool.ply").stat().st_size}]}
    (vis / "visuals.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {OUT.relative_to(REPO)}: mount {bvf.r3(mount)}, table top {table_top} mm, box {size} at "
          f"({start['x_mm']}, {start['y_mm']}), {len(obstacles)} obstacles, room {len(parts)} parts")


if __name__ == "__main__":
    main()
