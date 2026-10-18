"""GATE 5: pick the demo box, carry it, place it; the gripper's is_holding must be right at every step.

Steps (through the cap verbs): reset the crates -> fold the arm -> drive to table_pick -> pick the crate on the
pick station -> drive to the pallet -> place it (first pallet slot) -> fold -> drive home. After each step:
is_holding (gripper API) and where box1 is. Also checks: suction on with nothing under the cup does not hold;
the crate lands on pallet slot 1 (< 5 mm); the next crate (box2) appears on the pick station.

Run:  python3 20_sim/scripts/pick_place_check.py
Pass: every line OK, last line "pick_place_check PASS".
"""

import asyncio
import math

from viam.components.gripper import Gripper

from capbot_common import cap, connect


async def main():
    robot = await connect("sim")
    ok = True

    def check(good, text):
        nonlocal ok
        ok &= bool(good)
        print(f"{'OK  ' if good else 'FAIL'} {text}")

    try:
        vac = Gripper.from_robot(robot, "vacuum")
        holding = lambda: vac.is_holding_something()   # noqa: E731
        spots = (await cap(robot, "locations"))["box_spots"]
        await cap(robot, "reset_boxes")
        await vac.open()
        check(await vac.grab() is False and not (await holding()).is_holding_something,
              "suction on with nothing under the cup: not holding")
        await vac.open()
        steps = [("go_to_named_pose", {"name": "home_folded"}, False),
                 ("go_to_location", {"name": "table_pick"}, False),
                 ("pick", {"box": "box1"}, True),
                 ("go_to_location", {"name": "pallet"}, True),
                 ("place", {"spot": "pallet"}, False),
                 ("go_to_named_pose", {"name": "home_folded"}, False),
                 ("go_to_location", {"name": "home"}, False)]
        for verb, args, want_holding in steps:
            r = await cap(robot, verb, **args)
            h = (await holding()).is_holding_something
            box = (await cap(robot, "status"))["boxes"]["box1"]
            check(r.get("ok") and h == want_holding,
                  f"{verb} {args}: {'ok' if r.get('ok') else r.get('error')} | holding={h} (want {want_holding}) | box: "
                  + (box if isinstance(box, str) else f"({box['x_mm']:.1f}, {box['y_mm']:.1f}, {box['z_mm']:.1f})"))
            if not r.get("ok"):
                break
        boxes = (await cap(robot, "status"))["boxes"]
        box, s = boxes["box1"], spots["pallet"]["slots"][0]
        if isinstance(box, dict):
            err = math.dist((box["x_mm"], box["y_mm"], box["z_mm"]), (s["x_mm"], s["y_mm"], s["z_mm"]))
            check(err < 5.0, f"crate rests on pallet slot 1: {err:.2f} mm off")
        else:
            check(False, f"crate should rest on the pallet, but is {box}")
        check(isinstance(boxes["box2"], dict), f"next crate box2 is on the pick station: {boxes['box2']}")
    finally:
        await robot.close()
    print("pick_place_check PASS" if ok else "pick_place_check FAIL")


if __name__ == "__main__":
    asyncio.run(main())
