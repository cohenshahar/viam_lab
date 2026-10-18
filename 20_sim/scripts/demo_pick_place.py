"""GATE 9 / Sunday's sim demo, like the Viam course palletizer: crates come one at a time to the pick station on
the lab table (0.55 m); the cart parks side-on, the arm picks the crate from above, the cart drives to the crate
pallet, the arm sets the crate in the next slot from above (2 x 2 grid, 2 layers). Then everything goes home.
After every step an independent collision audit (our own kinematics, capbot_common.collision_audit) checks the arm,
tool, crate and cart against the room and the crates; any overlap stops the run.
Built only from the cap verbs (20_sim/primitive_map.md).

Watch it in the app: capbot-sim -> 3D SCENE.

Run:  python3 20_sim/scripts/demo_pick_place.py                 all 8 crates (one run)
      python3 20_sim/scripts/demo_pick_place.py --crates 2      a short run
      python3 20_sim/scripts/demo_pick_place.py --runs 5        GATE 9 (5 runs in a row; crates reset between runs)
      python3 20_sim/scripts/demo_pick_place.py --pause 1       wait 1 s between steps (easier to watch)
On any error: e-stop (arm + cart stop), say which step failed, release the e-stop, end.
"""

import argparse
import asyncio
import time

from capbot_common import cap, collision_audit, connect


class StepFailed(Exception):
    pass


async def step(robot, what, verb, pause, **args):
    t0 = time.time()
    r = await cap(robot, verb, **args)
    if not r.get("ok"):
        raise StepFailed(f"{what}: FAILED after {time.time() - t0:.1f} s -> {r.get('error')}")
    extra = ""
    if "cart" in r:
        c = r["cart"]
        extra = f" (cart at x={c['x_mm']:.0f} y={c['y_mm']:.0f} yaw={c['yaw_deg']:.0f})"
    if verb == "place":
        b = r["at"]
        extra = f" (slot {int(r.get('pallet_slot', 0))}, crate at x={b['x_mm']:.0f} y={b['y_mm']:.0f} z={b['z_mm']:.0f})"
    if verb == "pick":
        extra = f" ({r['box']})"
    hits = await collision_audit(robot)
    if hits:
        raise StepFailed(f"{what}: COLLISION found by the audit: {', '.join(hits)}")
    print(f"  {what}: ok in {time.time() - t0:.1f} s{extra} | audit: no collisions")
    if pause:
        await asyncio.sleep(pause)
    return r


async def one_run(robot, n, crates, pause):
    print(f"--- run {n}: {crates} crate(s)")
    await cap(robot, "reset_boxes")
    t_run = time.time()
    try:
        await step(robot, "fold the arm for driving", "go_to_named_pose", pause, name="home_folded")
        for i in range(1, crates + 1):
            print(f" crate {i}")
            await step(robot, "drive to the pick station", "go_to_location", pause, name="table_pick")
            await step(robot, "pick the crate from above", "pick", pause)
            await step(robot, "drive to the pallet", "go_to_location", pause, name="pallet")
            await step(robot, "set it in the next slot from above", "place", pause, spot="pallet")
        await step(robot, "fold the arm", "go_to_named_pose", pause, name="home_folded")
        await step(robot, "drive home", "go_to_location", pause, name="home")
    except StepFailed as e:
        print(f"  {e}")
        await cap(robot, "estop")
        await cap(robot, "release_estop")
        return False
    print(f"  run {n} done in {time.time() - t_run:.0f} s")
    return True


async def main(a):
    robot = await connect("sim")
    try:
        await cap(robot, "release_estop")
        good = 0
        for n in range(1, a.runs + 1):
            good += await one_run(robot, n, a.crates, a.pause)
        print(f"{good}/{a.runs} runs OK")
        print("demo PASS" if good == a.runs else "demo FAIL")
    finally:
        await robot.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--crates", type=int, default=8, help="how many crates to stack (1..8)")
    ap.add_argument("--pause", type=float, default=0.0, help="seconds to wait between steps")
    asyncio.run(main(ap.parse_args()))
