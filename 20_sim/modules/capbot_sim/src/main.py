"""Entry point of the capbot-sim local module: registers kr6, motor, wheel-odom, vacuum and scene."""

import asyncio

from viam.module.module import Module

import kr6_arm  # noqa: F401  (registers capbot:sim:kr6)
import scene  # noqa: F401  (registers capbot:sim:scene)
import sim_motor  # noqa: F401  (registers capbot:sim:motor)
import vacuum  # noqa: F401  (registers capbot:sim:vacuum)
import wheel_odom  # noqa: F401  (registers capbot:sim:wheel-odom)

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
