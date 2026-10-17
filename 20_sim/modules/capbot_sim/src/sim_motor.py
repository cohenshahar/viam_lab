"""capbot:sim:motor - a simulated drive motor whose turn count moves smoothly while it runs.

Why not rdk:builtin:fake + a fake encoder: the fake encoder's counting loop is started with the setup
context, which Viam cancels right after setup, so it never counts (rdk components/encoder/fake/encoder.go).
The fake motor then only jumps to the end position when go_for finishes, one wheel a few ms after the
other, and odom sees a big false turn in between (found 17 Oct running base_check). Decision #31.

Behaves like the real cart's GPIO motor in Viam (rdk components/motor/gpio): power = rpm / max_rpm,
go_for runs for |revolutions / rpm| minutes then stops. Unlike the real one it knows its position exactly
(the real cart has no encoders). Optional min_power_pct / max_power_pct copy the real motor's limits
(a small non-zero power is raised to min, a big one clipped to max); off by default so the sim is exact.

Attributes:
    max_rpm (number, required)      rpm at full power
    min_power_pct, max_power_pct    0..1, optional (see above)
    tick_hz (number)                how often to check for the end of a go_for, default 100
"""

import asyncio
import math
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.motor import Motor
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import struct_to_dict


def _fraction(attrs, key):
    v = attrs.get(key)
    if v is None:
        return None
    if not isinstance(v, (int, float)) or not 0 <= v <= 1:
        raise ValueError(f"{key} must be a number from 0 to 1")
    return float(v)


def shape_power(p, lo, hi):
    """The real GPIO motor's power rule: |p| clipped to max, a small non-zero |p| raised to min."""
    if p == 0:
        return 0.0
    mag = min(abs(p), hi if hi is not None else 1.0)
    if lo is not None:
        mag = max(mag, lo)
    return math.copysign(mag, p)


class SimMotor(Motor, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("capbot", "sim"), "motor")

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        rpm = attrs.get("max_rpm")
        if not isinstance(rpm, (int, float)) or rpm <= 0:
            raise ValueError("max_rpm must be a number greater than zero")
        lo, hi = _fraction(attrs, "min_power_pct"), _fraction(attrs, "max_power_pct")
        if lo is not None and hi is not None and lo > hi:
            raise ValueError("min_power_pct must not be above max_power_pct")
        hz = attrs.get("tick_hz", 100)
        if not isinstance(hz, (int, float)) or hz <= 0:
            raise ValueError("tick_hz must be a number greater than zero")
        return [], []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        self = super().new(config, dependencies)
        attrs = struct_to_dict(config.attributes)
        self.max_rpm = float(attrs["max_rpm"])
        self.lo, self.hi = _fraction(attrs, "min_power_pct"), _fraction(attrs, "max_power_pct")
        self.dt = 1.0 / float(attrs.get("tick_hz", 100))
        # The position is worked out at the moment it is asked (start + speed x time), not in clock
        # steps, so two wheels read a few ms apart still agree (odom needs that).
        self.pos0 = 0.0              # revolutions when the current command started
        self.t0 = 0.0
        self.power = 0.0             # -1..1, after the power rule
        self.target: Optional[float] = None   # go_for / go_to end position
        self.done = asyncio.Event()
        self.done.set()
        self.task = asyncio.create_task(self._loop())
        return self

    def _now(self):
        return asyncio.get_running_loop().time()

    def _position(self):
        if self.power == 0.0:
            return self.pos0
        p = self.pos0 + self.power * self.max_rpm / 60.0 * (self._now() - self.t0)
        if self.target is not None and (p - self.target) * math.copysign(1.0, self.power) >= 0:
            return self.target   # never past a go_for target
        return p

    async def _loop(self):
        """Ends go_for / go_to moves when the target is reached."""
        while True:
            await asyncio.sleep(self.dt)
            if self.target is not None and self.power != 0.0 and self._position() == self.target:
                self._halt()

    def _halt(self):
        self.pos0 = self._position()
        self.power = 0.0
        self.target = None
        self.done.set()

    def _set(self, power, target=None):
        self.pos0, self.t0 = self._position(), self._now()
        self.power = shape_power(power, self.lo, self.hi)
        self.target = target
        if self.power == 0.0:
            self._halt()
        else:
            self.done.clear()

    # ---- Motor API -----------------------------------------------------------------------

    async def set_power(self, power: float, *, extra=None, timeout=None, **kwargs):
        self._set(max(-1.0, min(1.0, power)))

    async def set_rpm(self, rpm: float, *, extra=None, timeout=None, **kwargs):
        self._set(max(-1.0, min(1.0, rpm / self.max_rpm)))

    async def go_for(self, rpm: float, revolutions: float, *, extra=None, timeout=None, **kwargs):
        if rpm == 0:
            raise ValueError("go_for needs a non-zero rpm")
        if revolutions == 0:   # Viam: 0 revolutions = run until stopped
            await self.set_rpm(rpm)
            return
        direction = math.copysign(1.0, rpm) * math.copysign(1.0, revolutions)
        self._set(direction * min(abs(rpm), self.max_rpm) / self.max_rpm, self._position() + direction * abs(revolutions))
        await self.done.wait()

    async def go_to(self, rpm: float, position_revolutions: float, *, extra=None, timeout=None, **kwargs):
        delta = position_revolutions - self._position()
        if abs(delta) < 1e-9:
            return
        await self.go_for(abs(rpm), delta)

    async def reset_zero_position(self, offset: float, *, extra=None, timeout=None, **kwargs):
        self._halt()
        self.pos0 = -offset

    async def get_position(self, *, extra=None, timeout=None, **kwargs) -> float:
        return self._position()

    async def get_properties(self, *, extra=None, timeout=None, **kwargs) -> Motor.Properties:
        return Motor.Properties(position_reporting=True)

    async def stop(self, *, extra=None, timeout=None, **kwargs):
        self._halt()

    async def is_powered(self, *, extra=None, timeout=None, **kwargs) -> Tuple[bool, float]:
        return self.power != 0.0, self.power

    async def is_moving(self) -> bool:
        return self.power != 0.0

    async def close(self):
        self.task.cancel()
