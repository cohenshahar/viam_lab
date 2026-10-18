"""Entry point of the capbot-sim local module: registers kr6, motor, wheel-odom, vacuum, scene, cap and camera."""

import asyncio

from viam.module.module import Module

import cap  # noqa: F401  (registers capbot:sim:cap)
import kr6_arm  # noqa: F401  (registers capbot:sim:kr6)
import scene  # noqa: F401  (registers capbot:sim:scene)
import sim_camera  # noqa: F401  (registers capbot:sim:camera)
import sim_motor  # noqa: F401  (registers capbot:sim:motor)
import vacuum  # noqa: F401  (registers capbot:sim:vacuum)
import wheel_odom  # noqa: F401  (registers capbot:sim:wheel-odom)

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
