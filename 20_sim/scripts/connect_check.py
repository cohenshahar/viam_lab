"""GATE 0: connect to a capbot machine and list what it has.

Run:  python3 20_sim/scripts/connect_check.py --machine sim      (or real)
Pass: it prints the resource names and "connected OK".
"""

import argparse
import asyncio

from capbot_common import connect


async def main(machine):
    robot = await connect(machine)
    try:
        names = sorted(f"{r.subtype:16s} {r.name}" for r in robot.resource_names)
        print("\n".join(names) or "(no resources yet)")
        print(f"connected OK to {machine}: {len(names)} resources")
    finally:
        await robot.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--machine", choices=["sim", "real", "arena"], default="sim")
    asyncio.run(main(ap.parse_args().machine))
