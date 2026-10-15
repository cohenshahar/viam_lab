"""Entry point of the capbot-sim local module: registers sim-base and vacuum."""

import asyncio

from viam.module.module import Module

import sim_base  # noqa: F401  (registers capbot:sim:sim-base)
import vacuum  # noqa: F401  (registers capbot:sim:vacuum)

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
