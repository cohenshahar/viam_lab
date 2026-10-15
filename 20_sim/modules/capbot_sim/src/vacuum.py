"""capbot:sim:vacuum - the suction tool as a Viam gripper (sim).

grab = suction on, open = suction off. In the sim there is no air, so grab always
"holds"; the program decides what is under the cup and moves that box with the
tool in its own WorldState.

Attributes:
    tool_box_mm (dict, required)  {"x", "y", "z", "translation": {"x","y","z"}} - the tool's
                                  collision box relative to the TCP (generated numbers)

DoCommand:
    {"command": "seal"} -> {"seal": 0.0 .. 1.0}   (1.0 while holding, like the thesis /tool/seal)
"""

import json
from typing import ClassVar, List, Mapping, Sequence, Tuple

from typing_extensions import Self
from viam.components.gripper import Gripper
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import KinematicsFileFormat, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict


def _box(attrs):
    box = attrs.get("tool_box_mm")
    if not isinstance(box, dict) or not all(isinstance(box.get(k), (int, float)) and box[k] > 0 for k in "xyz"):
        raise ValueError("tool_box_mm must be {\"x\", \"y\", \"z\" (mm, > 0), \"translation\": {...}}")
    t = box.get("translation", {})
    return {"type": "box", "x": box["x"], "y": box["y"], "z": box["z"],
            "translation": {k: float(t.get(k, 0.0)) for k in "xyz"}}


class Vacuum(Gripper, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("capbot", "sim"), "vacuum")

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        _box(struct_to_dict(config.attributes))
        return [], []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        self = super().new(config, dependencies)
        box = _box(struct_to_dict(config.attributes))
        # A model with no joints: one link that carries the tool box.
        self.kinematics = json.dumps({
            "name": "vacuum_tool", "kinematic_param_type": "SVA",
            "links": [{"id": "vacuum_tool", "parent": "world", "translation": {"x": 0, "y": 0, "z": 0},
                       "geometry": box}],
            "joints": [],
        }).encode()
        self.holding = False
        return self

    async def open(self, *, extra=None, timeout=None, **kwargs):
        self.holding = False

    async def grab(self, *, extra=None, timeout=None, **kwargs) -> bool:
        self.holding = True
        return True

    async def is_holding_something(self, *, extra=None, timeout=None, **kwargs) -> Gripper.HoldingStatus:
        return Gripper.HoldingStatus(is_holding_something=self.holding)

    async def stop(self, *, extra=None, timeout=None, **kwargs):
        pass  # suction keeps its state on stop, like the real valve

    async def is_moving(self) -> bool:
        return False

    async def get_kinematics(self, *, extra=None, timeout=None, **kwargs):
        return KinematicsFileFormat.KINEMATICS_FILE_FORMAT_SVA, self.kinematics

    async def get_current_inputs(self, *, extra=None, timeout=None, **kwargs) -> List[float]:
        return []

    async def go_to_inputs(self, values: List[float], *, extra=None, timeout=None, **kwargs):
        if values:
            raise ValueError("the vacuum tool has no joints")

    async def do_command(self, command: Mapping[str, ValueTypes], *, timeout=None, **kwargs) -> Mapping[str, ValueTypes]:
        if command.get("command") == "seal":
            return {"seal": 1.0 if self.holding else 0.0}
        return {"error": f"unknown command {command.get('command')!r}; known: seal"}
