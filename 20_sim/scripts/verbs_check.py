"""GATE 8: every cap verb called once; bad input gives a clear error, never a crash.

Each line: the call, what came back, and whether that is what we expect:
  "ok"      -> {"ok": true}
  "unclear" -> {"error": ..., "needs_clarification": true}   (bad / unknown argument)
  "refused" -> {"error": ..., "needs_clarification": false}  (clear request that cannot be done, e.g. e-stop)

Run:  python3 20_sim/scripts/verbs_check.py
Pass: every line OK, last line "verbs_check PASS".
"""

import asyncio
import json

from capbot_common import cap, connect

DOWN = {"ox": 0, "oy": 0, "oz": -1}

CASES = [
    ("status", {}, "ok"),
    ("locations", {}, "ok"),
    ("reset_boxes", {}, "ok"),
    ("go_to_named_pose", {"name": "home_arm_up"}, "ok"),
    ("go_to_named_pose", {"name": "nowhere"}, "unclear"),
    ("approach", {"x": 700, "y": 0, "z": 500, **DOWN, "standoff_mm": 100}, "ok"),   # standoff point 100 mm higher must be reachable too
    ("approach", {"y": 0, "z": 700, **DOWN}, "unclear"),
    ("move_relative", {"dz": 50}, "ok"),
    ("move_relative", {"dx": 30, "frame": "tool"}, "ok"),
    ("move_relative", {"dz": 50, "frame": "moon"}, "unclear"),
    ("go_to_named_pose", {"name": "home_folded"}, "ok"),
    ("drive_to", {"x_mm": 500, "y_mm": 300, "yaw_deg": 45}, "ok"),
    ("drive_to", {"x_mm": 500}, "unclear"),
    ("drive_arc", {"radius_mm": 1000, "angle_deg": 30, "speed_mm_per_sec": 200}, "ok"),
    ("drive_arc", {"radius_mm": 0, "angle_deg": 30}, "unclear"),
    ("go_to_location", {"name": "home"}, "ok"),
    ("go_to_location", {"name": "mars"}, "unclear"),
    ("pick", {"box": "box99"}, "unclear"),
    ("place", {"spot": "pallet"}, "refused"),            # nothing held
    ("place", {"spot": "nowhere"}, "unclear"),
    ("estop", {}, "ok"),
    ("go_to_named_pose", {"name": "home_arm_up"}, "refused"),   # e-stop latched
    ("drive_to", {"x_mm": 0, "y_mm": 0, "yaw_deg": 0}, "refused"),
    ("release_estop", {}, "ok"),
    ("go_to_named_pose", {"name": "home_folded"}, "ok"),
    ("fly", {}, "unclear"),
]


def kind(r):
    if r.get("ok"):
        return "ok"
    if "error" in r:
        return "unclear" if r.get("needs_clarification") else "refused"
    return "???"


async def main():
    robot = await connect("sim")
    ok = True
    try:
        for verb, args, want in CASES:
            try:
                r = await cap(robot, verb, **args)
            except Exception as e:   # a crash is a fail, not a stop
                r = {"crash": str(e).splitlines()[0][:150]}
            got = kind(r)
            good = got == want
            ok &= good
            short = r.get("error", "") if got != "ok" else ""
            print(f"{'OK  ' if good else 'FAIL'} {verb} {json.dumps(args)} -> {got} (want {want}) {short[:110]}")
    finally:
        await robot.close()
    print("verbs_check PASS" if ok else "verbs_check FAIL")


if __name__ == "__main__":
    asyncio.run(main())
