"""capbot:sim:sim-base - a simulated diff-drive cart for Viam.

Viam's fake base does nothing and its frame never moves. This base keeps the cart
pose itself and writes it, as three joints (x mm, y mm, yaw deg), into a fake arm
called cart-pose whose kinematics are 20_sim/kinematics/cart_planar.json. Every
frame hung under the cart (the KR6, the sensors) then moves with it.

Attributes:
    pose_arm (str, required)          name of the cart-pose arm
    width_mm (float, required)        drive wheel track
    wheel_circumference_mm (float, required)
    max_speed_mm_per_sec (float)      default 500
    max_turn_deg_per_sec (float)      default 90
    tick_hz (float)                   default 20
    start_x_mm, start_y_mm, start_yaw_deg (float)  default 0

DoCommand:
    {"command": "pose"}                                -> {"x_mm", "y_mm", "yaw_deg"}
    {"command": "set_pose", "x_mm", "y_mm", "yaw_deg"} -> teleport (sim only)
"""

import asyncio
import math
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.arm import Arm, JointPositions
from viam.components.base import Base
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName, Vector3
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

from drive_math import Pose, clamp, step

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


class SimBase(Base, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("capbot", "sim"), "sim-base")

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        arm = attrs.get("pose_arm")
        if not isinstance(arm, str) or not arm:
            raise ValueError("pose_arm must name the cart-pose arm, e.g. \"cart-pose\"")
        for key in ("width_mm", "wheel_circumference_mm"):
            _number(attrs, key, positive=True)
        for key in ("max_speed_mm_per_sec", "max_turn_deg_per_sec", "tick_hz"):
            if key in attrs:
                _number(attrs, key, positive=True)
        return [arm], []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        self = super().new(config, dependencies)
        attrs = struct_to_dict(config.attributes)
        self.pose_arm = dependencies[Arm.get_resource_name(attrs["pose_arm"])]
        self.width_mm = _number(attrs, "width_mm")
        self.wheel_circ_mm = _number(attrs, "wheel_circumference_mm")
        self.max_v = _number(attrs, "max_speed_mm_per_sec", 500.0, positive=True)
        self.max_w = _number(attrs, "max_turn_deg_per_sec", 90.0, positive=True)
        self.dt = 1.0 / _number(attrs, "tick_hz", 20.0, positive=True)
        self.pose = Pose(_number(attrs, "start_x_mm", 0.0), _number(attrs, "start_y_mm", 0.0),
                         _number(attrs, "start_yaw_deg", 0.0))
        self.v = 0.0      # commanded forward speed, mm/s
        self.w = 0.0      # commanded turn rate, deg/s (+ = left)
        self.remaining_s = None  # None = run until told otherwise
        self.done = asyncio.Event()
        self.done.set()
        self.lock = asyncio.Lock()
        self.task = asyncio.create_task(self._loop())
        return self

    # ---- the clock -------------------------------------------------------------------

    async def _loop(self):
        await self._publish()
        while True:
            await asyncio.sleep(self.dt)
            async with self.lock:
                if self.v == 0.0 and self.w == 0.0:
                    continue
                dt = self.dt
                if self.remaining_s is not None:
                    dt = min(dt, self.remaining_s)
                    self.remaining_s -= dt
                self.pose = step(self.pose, self.v, self.w, dt)
                if self.remaining_s is not None and self.remaining_s <= 1e-9:
                    self._halt()
            try:
                await self._publish()
            except Exception as e:  # keep the clock alive even if the arm call fails once
                LOGGER.warning(f"could not move cart-pose: {e}")

    async def _publish(self):
        p = self.pose
        await self.pose_arm.move_to_joint_positions(JointPositions(values=[p.x_mm, p.y_mm, p.yaw_deg]))

    def _halt(self):
        self.v = self.w = 0.0
        self.remaining_s = None
        self.done.set()

    async def _run(self, v, w, seconds):
        async with self.lock:
            self.v, self.w = v, w
            self.remaining_s = seconds
            self.done.clear()
        await self.done.wait()

    # ---- Base API --------------------------------------------------------------------

    async def move_straight(self, distance: int, velocity: float, *, extra=None, timeout=None, **kwargs):
        if distance == 0 or velocity == 0:
            return
        speed = min(abs(velocity), self.max_v)
        direction = 1.0 if (distance > 0) == (velocity > 0) else -1.0
        await self._run(direction * speed, 0.0, abs(distance) / speed)

    async def spin(self, angle: float, velocity: float, *, extra=None, timeout=None, **kwargs):
        if angle == 0 or velocity == 0:
            return
        rate = min(abs(velocity), self.max_w)
        direction = 1.0 if (angle > 0) == (velocity > 0) else -1.0
        await self._run(0.0, direction * rate, abs(angle) / rate)

    async def set_velocity(self, linear: Vector3, angular: Vector3, *, extra=None, timeout=None, **kwargs):
        async with self.lock:
            self.v = clamp(linear.y, self.max_v)
            self.w = clamp(angular.z, self.max_w)
            self.remaining_s = None
            if self.v == 0.0 and self.w == 0.0:
                self.done.set()
            else:
                self.done.clear()

    async def set_power(self, linear: Vector3, angular: Vector3, *, extra=None, timeout=None, **kwargs):
        await self.set_velocity(Vector3(y=clamp(linear.y, 1.0) * self.max_v),
                                Vector3(z=clamp(angular.z, 1.0) * self.max_w))

    async def stop(self, *, extra=None, timeout=None, **kwargs):
        async with self.lock:
            self._halt()

    async def is_moving(self) -> bool:
        return self.v != 0.0 or self.w != 0.0

    async def get_properties(self, *, timeout=None, **kwargs) -> Base.Properties:
        return Base.Properties(width_meters=self.width_mm / 1000.0, turning_radius_meters=0.0,
                               wheel_circumference_meters=self.wheel_circ_mm / 1000.0)

    async def get_geometries(self, *, extra=None, timeout=None) -> List[Geometry]:
        return []  # the cart body box lives on cart-pose

    async def do_command(self, command: Mapping[str, ValueTypes], *, timeout=None, **kwargs) -> Mapping[str, ValueTypes]:
        verb = command.get("command")
        if verb == "pose":
            p = self.pose
            return {"x_mm": round(p.x_mm, 3), "y_mm": round(p.y_mm, 3), "yaw_deg": round(p.yaw_deg, 4)}
        if verb == "set_pose":
            try:
                new = Pose(float(command["x_mm"]), float(command["y_mm"]), float(command["yaw_deg"]))
            except (KeyError, TypeError, ValueError):
                return {"error": "set_pose needs numbers x_mm, y_mm, yaw_deg"}
            async with self.lock:
                self._halt()
                self.pose = new
            await self._publish()
            return {"ok": True}
        return {"error": f"unknown command {verb!r}; known: pose, set_pose"}

    async def close(self):
        self.task.cancel()
