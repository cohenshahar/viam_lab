"""Entry point of the capbot-sim local module: registers wheel-odom and vacuum."""

import asyncio

from viam.module.module import Module

import vacuum  # noqa: F401  (registers capbot:sim:vacuum)
import wheel_odom  # noqa: F401  (registers capbot:sim:wheel-odom)

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
