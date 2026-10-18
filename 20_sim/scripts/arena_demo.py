"""The arm arena demo: the KR6 on the rig table picks the box from above, puts it on the place spot, picks it
again and puts it back where it started. Built from the cap verbs (20_sim/primitive_map.md); after every step
the independent collision audit (capbot_common.collision_audit) checks arm, tool and box against the rig.

Machine: armtable-sim (20_sim/configs/armtable-sim.json). Watch it in the app: armtable-sim -> 3D SCENE.

Run:  python3 20_sim/scripts/arena_demo.py              one cycle
      python3 20_sim/scripts/arena_demo.py --cycles 3   three cycles (the box goes there and back each time)
Connects to the machine in 20_sim/.env as ARENA_* (or CAPBOT_LOCAL=localhost:<port> for an offline test).
"""

import argparse
import asyncio
import time

from capbot_common import cap, collision_audit, connect


class StepFailed(Exception):
    pass


async def step(robot, what, verb, **args):
    t0 = time.time()
    r = await cap(robot, verb, **args)
    if not r.get("ok"):
        raise StepFailed(f"{what}: FAILED after {time.time() - t0:.1f} s -> {r.get('error')}")
    hits = await collision_audit(robot)
    if hits:
        raise StepFailed(f"{what}: COLLISION found by the audit: {', '.join(hits)}")
    extra = ""
    if verb == "place":
        b = r["at"]
        extra = f" (box at x={b['x_mm']:.0f} y={b['y_mm']:.0f} z={b['z_mm']:.0f})"
    print(f"  {what}: ok in {time.time() - t0:.1f} s{extra} | audit: no collisions")


async def main(a):
    robot = await connect("arena")
    good = 0
    try:
        await cap(robot, "release_estop")
        await cap(robot, "reset_boxes")
        for c in range(1, a.cycles + 1):
            print(f"--- cycle {c}")
            t0 = time.time()
            try:
                await step(robot, "fold the arm", "go_to_named_pose", name="home_folded")
                await step(robot, "pick the box from above", "pick", box="box1")
                await step(robot, "put it on the place spot", "place", spot="place_spot")
                await step(robot, "pick it again", "pick", box="box1")
                await step(robot, "put it back where it started", "place", spot="start_spot")
                await step(robot, "fold the arm", "go_to_named_pose", name="home_folded")
                good += 1
                print(f"  cycle {c} done in {time.time() - t0:.0f} s")
            except StepFailed as e:
                print(f"  {e}")
                await cap(robot, "estop")
                await cap(robot, "release_estop")
                break
    finally:
        await robot.close()
    print(f"{good}/{a.cycles} cycles OK")
    print("arena_demo PASS" if good == a.cycles else "arena_demo FAIL")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=1)
    asyncio.run(main(ap.parse_args()))
