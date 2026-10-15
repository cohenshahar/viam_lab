"""Sunday's script: drive the cart through Viam. The same file runs on the sim and the real cart.

    python3 30_robot/scripts/cart_sunday.py --machine sim --drive
    python3 30_robot/scripts/cart_sunday.py --machine real --motor-test          # WHEELS IN THE AIR
    python3 30_robot/scripts/cart_sunday.py --machine real --drive               # on the floor

--motor-test (real only): each motor forward 2 s, stop, backward 2 s, stop. Check direction.
--drive: forward 500 mm, back 500 mm, spin 90 deg left, spin 90 deg right. Measure with a tape.

Safety:
  * asks before every step (skip with --yes), Ctrl+C stops everything
  * every step has a time limit; on any error or timeout: stop the motors
  * speeds come from 30_robot/configs/capbot_build_info.json (the generated real config),
    so the requested power is the day-one operating power and the motor config caps it
  * on the real cart, both gear pins are set to LOW gear first. The level that means "low"
    must be known (measured file or --gear-low-level); the script refuses to guess.
"""

import argparse
import asyncio
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "20_sim/scripts"))
from capbot_common import connect  # noqa: E402

DISTANCE_MM = 500
SPIN_DEG = 90
MOTOR_TEST_S = 2.0
SIM_SPEED_MM_S = 250
SIM_TURN_DEG_S = 45


def load_info():
    p = REPO / "30_robot/configs/capbot_build_info.json"
    if not p.exists():
        sys.exit("Run 20_sim/tools/make_configs.py first (it writes capbot_build_info.json).")
    return json.loads(p.read_text())


def real_speeds(info):
    cfg = json.loads((REPO / "30_robot/configs/capbot.json").read_text())
    base = next(c for c in cfg["components"] if c["name"] == "cart")["attributes"]
    v = info["speed_at_operating_power_mm_s_free_wheel"]
    w = math.degrees(v / (base["width_mm"] / 2.0))
    return v, w


def ask(msg, yes):
    if yes:
        print(msg)
        return
    if input(f"{msg}  [Enter = go, q = quit] ").strip().lower() == "q":
        raise KeyboardInterrupt


async def guarded(name, coro, seconds, stop):
    print(f"  -> {name} (limit {seconds:.1f} s)")
    try:
        await asyncio.wait_for(coro, timeout=seconds)
    except asyncio.TimeoutError:
        print(f"  !! {name} took too long, stopping")
        await stop()
        raise
    finally:
        await stop()


async def main(a):
    from viam.components.base import Base
    from viam.components.board import Board
    from viam.components.motor import Motor

    real = a.machine == "real"
    info = load_info() if real else None
    level = None
    if real:
        level = a.gear_low_level or info["gear_pins"]["low_gear_level"]
        if level not in ("high", "low"):
            if a.drive:
                sys.exit("Which pin level is LOW gear? Give --gear-low-level high|low (find it with --motor-test first).")
            level = None
            print("gear level unknown: the motor test leaves the gear pins alone (try both levels by hand)")
    robot = await connect(a.machine)
    motors = []
    try:
        cart = Base.from_robot(robot, "cart")
        names = {r.name for r in robot.resource_names}

        async def stop_all():
            for m in motors:
                await m.stop()
            await cart.stop()

        if real:
            if level:
                board = Board.from_robot(robot, "board")
                for side in ("left", "right"):
                    pin = await board.gpio_pin_by_name(str(info["gear_pins"][side]))
                    await pin.set(level == "high")
                print(f"gear pins {info['gear_pins']['left']}/{info['gear_pins']['right']} set to {level} (= low gear)")
            motors = [Motor.from_robot(robot, n) for n in ("left", "right") if n in names]
            v, w = real_speeds(info)
        else:
            v, w = SIM_SPEED_MM_S, SIM_TURN_DEG_S
        print(f"speed {v:.0f} mm/s, turn {w:.1f} deg/s ({a.machine})")

        if a.motor_test:
            if not real:
                print("motor test is for the real cart only (the sim has no motors)")
            else:
                ask("MOTOR TEST: are the drive wheels OFF the floor and someone at the power switch?", a.yes)
                for m in motors:
                    for sign, word in ((1, "forward"), (-1, "backward")):
                        ask(f"{m.name} {word} at power {a.power}", a.yes)
                        await m.set_power(sign * a.power)
                        await asyncio.sleep(MOTOR_TEST_S)
                        await m.stop()
                        print(f"  {m.name} {word}: did it turn, and which way? (write it down)")

        if a.drive:
            if real:
                ask("DRIVE: cart on the floor, area clear, someone at the power switch?", a.yes)
            steps = [
                (f"forward {DISTANCE_MM} mm", cart.move_straight(DISTANCE_MM, v), DISTANCE_MM / v),
                (f"back {DISTANCE_MM} mm", cart.move_straight(-DISTANCE_MM, v), DISTANCE_MM / v),
                (f"spin {SPIN_DEG} deg left", cart.spin(SPIN_DEG, w), SPIN_DEG / w),
                (f"spin {SPIN_DEG} deg right", cart.spin(-SPIN_DEG, w), SPIN_DEG / w),
            ]
            for name, coro, expect_s in steps:
                try:
                    ask(name, a.yes)
                except KeyboardInterrupt:
                    coro.close()
                    raise
                await guarded(name, coro, expect_s * 1.5 + 2.0, stop_all)
                if not real:
                    print(f"  pose now: {await cart.do_command({'command': 'pose'})}")
                else:
                    print("  measure it now (distance / angle) and write it in the log")
        print("done")
    except KeyboardInterrupt:
        print("stopped by you")
    finally:
        try:
            for m in motors:
                await m.stop()
            await Base.from_robot(robot, "cart").stop()
        finally:
            await robot.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--machine", choices=["sim", "real"], required=True)
    ap.add_argument("--motor-test", action="store_true")
    ap.add_argument("--drive", action="store_true")
    ap.add_argument("--power", type=float, default=0.30, help="motor test power, 0..1 (the motor config caps it)")
    ap.add_argument("--gear-low-level", choices=["high", "low"], help="pin level that selects LOW gear")
    ap.add_argument("--yes", action="store_true", help="do not ask before each step")
    args = ap.parse_args()
    if not (args.motor_test or args.drive):
        ap.error("choose --motor-test and/or --drive")
    if not 0 < args.power <= 0.5:
        ap.error("--power must be between 0 and 0.5")
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("stopped")
