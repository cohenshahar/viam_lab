"""capbot:sim:wheel-odom - wheel odometry for the sim cart: where the cart is in the room.

The sim cart is built like the real one: Viam's wheeled base `cart` drives two motors
`left` and `right` (fake motors with fake encoders in the sim). The wheeled base does
the real math: distance / angle -> revolutions and rpm per wheel, from width_mm and
wheel_circumference_mm. This part watches how far each wheel turned and works out the
cart pose in the room from that (diff-drive odometry).

Viam's world frame is the cart (decision #29, as in Viam's "mobile base with arm" docs), so
this pose is not in the frame system. The arm helper (capbot_common.arm_move) reads it to
place the room's obstacles around the cart before each arm move.

Attributes:
    left_motor, right_motor (str, required)   motor names
    width_mm, wheel_circumference_mm (number, required)  same values as the wheeled base
    tick_hz (number)                          default 20
    start_x_mm, start_y_mm, start_yaw_deg (number)   default 0

DoCommand:
    {"command": "pose"}                                -> {"x_mm", "y_mm", "yaw_deg", "left_rev", "right_rev"}
    {"command": "set_pose", "x_mm", "y_mm", "yaw_deg"} -> teleport (sim only; the wheels are not moved)
"""

import asyncio
from typing import ClassVar, Mapping, Sequence, Tuple

from typing_extensions import Self
from viam.components.generic import Generic
from viam.components.motor import Motor
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

from drive_math import Pose, odom_step

LOGGER = getLogger(__name__)


def _number(attrs, key, default=None, positive=False):
    v = attrs.get(key, default)
    if v is None:
        raise ValueError(f"{key} is required")
    if not isinstance(v, (int, float)):
        raise ValueError(f"{key} must be a number")
    if positive and v <= 0:
        raise ValueError(f"{key} must be greater than zero")
    return float(v)


def _name(attrs, key, example):
    v = attrs.get(key)
    if not isinstance(v, str) or not v:
        raise ValueError(f'{key} must name a component, e.g. "{example}"')
    return v


class WheelOdom(Generic, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("capbot", "sim"), "wheel-odom")

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        deps = [_name(attrs, "left_motor", "left"), _name(attrs, "right_motor", "right")]
        for key in ("width_mm", "wheel_circumference_mm"):
            _number(attrs, key, positive=True)
        if "tick_hz" in attrs:
            _number(attrs, "tick_hz", positive=True)
        return deps, []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        self = super().new(config, dependencies)
        attrs = struct_to_dict(config.attributes)
        self.left = dependencies[Motor.get_resource_name(attrs["left_motor"])]
        self.right = dependencies[Motor.get_resource_name(attrs["right_motor"])]
        self.width_mm = _number(attrs, "width_mm")
        self.wheel_circ_mm = _number(attrs, "wheel_circumference_mm")
        self.dt = 1.0 / _number(attrs, "tick_hz", 20.0, positive=True)
        self.pose = Pose(_number(attrs, "start_x_mm", 0.0), _number(attrs, "start_y_mm", 0.0),
                         _number(attrs, "start_yaw_deg", 0.0))
        self.last_rev = None   # (left, right) wheel revolutions at the last tick
        self.lock = asyncio.Lock()
        self.task = asyncio.create_task(self._loop())
        return self

    async def _read_rev(self):
        return await self.left.get_position(), await self.right.get_position()

    async def _tick(self):
        async with self.lock:
            rev = await self._read_rev()
            if self.last_rev is not None:
                dl = (rev[0] - self.last_rev[0]) * self.wheel_circ_mm
                dr = (rev[1] - self.last_rev[1]) * self.wheel_circ_mm
                if dl or dr:
                    self.pose = odom_step(self.pose, dl, dr, self.width_mm)
            self.last_rev = rev

    async def _loop(self):
        while True:
            try:
                await self._tick()
            except Exception as e:  # keep the clock alive if one read or write fails
                LOGGER.warning(f"wheel-odom tick failed: {e}")
            await asyncio.sleep(self.dt)

    async def do_command(self, command: Mapping[str, ValueTypes], *, timeout=None, **kwargs) -> Mapping[str, ValueTypes]:
        verb = command.get("command")
        if verb == "pose":
            await self._tick()  # fresh, not up to one tick old
            p, rev = self.pose, self.last_rev or (0.0, 0.0)
            return {"x_mm": round(p.x_mm, 3), "y_mm": round(p.y_mm, 3), "yaw_deg": round(p.yaw_deg, 4),
                    "left_rev": round(rev[0], 4), "right_rev": round(rev[1], 4)}
        if verb == "set_pose":
            try:
                new = Pose(float(command["x_mm"]), float(command["y_mm"]), float(command["yaw_deg"]))
            except (KeyError, TypeError, ValueError):
                return {"error": "set_pose needs numbers x_mm, y_mm, yaw_deg"}
            async with self.lock:
                self.pose, self.last_rev = new, await self._read_rev()
            return {"ok": True}
        return {"error": f"unknown command {verb!r}; known: pose, set_pose"}

    async def close(self):
        self.task.cancel()
