"""capbot:sim:kr6 - the simulated KUKA KR6, with its CAD meshes for Viam's 3D scene (decision #30).

Does what rdk:builtin:simulated did for us (kr6.json kinematics, joint moves that take time), plus
get_3d_models: one GLB per kinematics link, made by build_viam_files.py from the thesis MuJoCo model.
The 3D scene draws each GLB in place of that link's collision box. The planner still uses the boxes.

Attributes:
    kinematics_path (str, required)      absolute path to 20_sim/kinematics/kr6.json
    visuals_dir (str, required)          absolute path to 20_sim/generated/visuals (has visuals.json)
    speed_deg_per_sec (number)           fastest joint speed, default 57.3 (= 1 rad/s, as the builtin sim)
    tick_hz (number)                     default 50

Joint positions are degrees, poses mm (Viam units). get_end_position is the flange pose in the arm's
base frame, from our own forward kinematics (20_sim/tools/viam_frames.py, checked against MuJoCo).
"""

import asyncio
import json
import sys
from pathlib import Path
from typing import Any, ClassVar, List, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.arm import Arm, JointPositions, Pose
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, KinematicsFileFormat, Mesh, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import struct_to_dict

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import viam_frames as vf  # noqa: E402

LOGGER = getLogger(__name__)


def _path(attrs, key):
    v = attrs.get(key)
    if not isinstance(v, str) or not v:
        raise ValueError(f"{key} must be an absolute path")
    if not Path(v).exists():
        raise ValueError(f"{key}: {v} does not exist on this computer (re-run make_configs.py here)")
    return Path(v)


def _positive(attrs, key, default):
    v = attrs.get(key, default)
    if not isinstance(v, (int, float)) or v <= 0:
        raise ValueError(f"{key} must be a number greater than zero")
    return float(v)


def plan_steps(start, goal, speed_deg_s, dt):
    """Joint positions every dt seconds from start to goal, all joints arriving together at the speed
    limit of the joint that moves most. Pure, for tests."""
    span = max(abs(g - s) for s, g in zip(start, goal))
    n = max(1, int(span / speed_deg_s / dt + 0.999999))
    return [[s + (g - s) * (k / n) for s, g in zip(start, goal)] for k in range(1, n + 1)]


class Kr6Arm(Arm, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("capbot", "sim"), "kr6")

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        _path(attrs, "kinematics_path")
        if not (_path(attrs, "visuals_dir") / "visuals.json").exists():
            raise ValueError("visuals_dir has no visuals.json (run build_viam_files.py)")
        _positive(attrs, "speed_deg_per_sec", 57.3)
        _positive(attrs, "tick_hz", 50)
        return [], []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        self = super().new(config, dependencies)
        attrs = struct_to_dict(config.attributes)
        kin_path = _path(attrs, "kinematics_path")
        self.kinematics = kin_path.read_bytes()
        self.model = json.loads(self.kinematics)
        self.limits = [(j["min"], j["max"]) for j in self.model["joints"]]
        vis = _path(attrs, "visuals_dir")
        manifest = json.loads((vis / "visuals.json").read_text())
        self.meshes = {link: Mesh(content_type="glb", mesh=(vis / info["file"]).read_bytes())
                       for link, info in manifest["kr6"].items()}
        self.speed = _positive(attrs, "speed_deg_per_sec", 57.3)
        self.dt = 1.0 / _positive(attrs, "tick_hz", 50)
        self.joints = [0.0] * len(self.limits)
        self.moving = False
        self.task: Optional[asyncio.Task] = None
        return self

    # ---- motion --------------------------------------------------------------------------

    def _check(self, values):
        if len(values) != len(self.limits):
            raise ValueError(f"the KR6 has {len(self.limits)} joints, got {len(values)} values")
        for i, (v, (lo, hi)) in enumerate(zip(values, self.limits)):
            if not lo - 1e-6 <= v <= hi + 1e-6:
                raise ValueError(f"joint a{i + 1} = {v:.2f} deg is outside its limits [{lo}, {hi}]")

    async def _run(self, goals):
        try:
            self.moving = True
            for goal in goals:
                for q in plan_steps(self.joints, goal, self.speed, self.dt):
                    await asyncio.sleep(self.dt)
                    self.joints = q
        finally:
            self.moving = False

    async def _move(self, goals):
        for g in goals:
            self._check(g)
        await self.stop()
        self.task = asyncio.create_task(self._run(goals))
        try:
            await self.task
        except asyncio.CancelledError:
            pass  # stopped

    async def move_to_joint_positions(self, positions: JointPositions, *, extra=None, timeout=None, **kwargs):
        await self._move([list(positions.values)])

    async def move_through_joint_positions(self, positions: List[JointPositions], options=None, *,
                                           extra=None, timeout=None, **kwargs):
        await self._move([list(p.values) for p in positions])

    async def move_through_joint_positions_streamed(self, batches, *, extra=None, timeout=None, **kwargs):
        raise NotImplementedError("capbot:sim:kr6 does not stream; use move_through_joint_positions")

    async def move_to_position(self, pose: Pose, *, extra=None, timeout=None, **kwargs):
        raise NotImplementedError("capbot:sim:kr6 moves to poses through the motion service (capbot_common.arm_move)")

    async def stop(self, *, extra=None, timeout=None, **kwargs):
        if self.task and not self.task.done():
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
        self.moving = False

    async def is_moving(self) -> bool:
        return self.moving

    # ---- state ---------------------------------------------------------------------------

    async def get_joint_positions(self, *, extra=None, timeout=None, **kwargs) -> JointPositions:
        return JointPositions(values=list(self.joints))

    async def get_end_position(self, *, extra=None, timeout=None, **kwargs) -> Pose:
        t = vf.sva_fk(self.model, self.joints)["flange"]
        ox, oy, oz, th = vf.matrix_to_ov(t[:3, :3], snap=False)
        return Pose(x=float(t[0, 3]), y=float(t[1, 3]), z=float(t[2, 3]), o_x=ox, o_y=oy, o_z=oz, theta=th)

    async def get_kinematics(self, *, extra=None, timeout=None, **kwargs):
        return KinematicsFileFormat.KINEMATICS_FILE_FORMAT_SVA, self.kinematics

    async def get_3d_models(self, *, extra=None, timeout=None, **kwargs) -> Mapping[str, Mesh]:
        return self.meshes

    async def get_geometries(self, *, extra=None, timeout=None) -> List[Geometry]:
        return []  # the frame system takes the link boxes from the kinematics

    async def set_manual_mode(self, manual_mode: bool, enabled_for: int = 0, *, extra=None, timeout=None, **kwargs):
        if manual_mode:
            raise NotImplementedError("the sim KR6 has no manual mode")

    async def get_manual_mode(self, *, extra=None, timeout=None, **kwargs) -> bool:
        return False

    async def get_properties(self, *, extra=None, timeout=None, **kwargs) -> Arm.Properties:
        return Arm.Properties(support_manual_mode=False, support_cartesian_commands=False)

    async def close(self):
        await self.stop()
