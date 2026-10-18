"""capbot:sim:scene - CAD look for Viam's 3D scene: cart, tool and room (decision #30).

A world state store service: the 3D scene asks it for shapes to draw. The planner never reads it.
Shapes come from 20_sim/generated/visuals (PLY meshes, metres, colour per vertex, made by
build_viam_files.py from the thesis MuJoCo model):
  - "cart" shapes hang on frame `cart`, the "tool" on frame `vacuum`: they move with the robot by themselves.
  - the "room" hangs on `world` (= the cart, decision #29). Its pose is the inverse of odom's cart pose,
    so the room turns and slides around the cart as it drives, like the room the planner gets per move.
  - the demo boxes (sim_world): resting ones are placed around the cart like the room; a held one rides on
    `vacuum`. Picking / placing moves them between the two.

Attributes:
    visuals_dir (str, required)   absolute path to 20_sim/generated/visuals
    odom (str)                    odometry component name, default "odom"; "" for a fixed arm (room stays put)
    poll_hz (number)              how often to read odom, default 10
"""

import asyncio
import base64
import json
import math
import uuid as uuidlib
from pathlib import Path
from typing import AsyncGenerator, ClassVar, Dict, List, Mapping, Optional, Sequence, Set, Tuple

from google.protobuf.field_mask_pb2 import FieldMask
from typing_extensions import Self
from viam.components.generic import Generic
from viam.logging import getLogger
from viam.proto.app.robot import ServiceConfig
from google.protobuf.struct_pb2 import Struct
from viam.proto.common import Geometry, Mesh, Pose, PoseInFrame, RectangularPrism, ResourceName, Transform, Vector3
from viam.proto.service.worldstatestore import StreamTransformChangesResponse, TransformChangeType
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.services.worldstatestore import WorldStateStore
from viam.utils import struct_to_dict

from sim_world import WORLD, Box

LOGGER = getLogger(__name__)
NS = uuidlib.UUID("6f1c2b0e-55a1-4d4e-9a57-cab07517e0a1")  # fixed, so a shape keeps its id across restarts


def shape_uuid(name: str) -> bytes:
    return uuidlib.uuid5(NS, name).bytes


BOX_RGB = (194, 154, 108)   # cardboard


def room_to_cart(cart: Mapping[str, float], x: float, y: float, yaw_deg: float) -> Tuple[float, float, float]:
    """A room point / heading seen from the cart (= Viam's world frame)."""
    yaw = math.radians(cart["yaw_deg"])
    dx, dy = x - cart["x_mm"], y - cart["y_mm"]
    return (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy, yaw_deg - cart["yaw_deg"])


def box_transform(box: Box, cart: Optional[Mapping[str, float]]) -> Optional[Transform]:
    """Where to draw a demo box: on `vacuum` while held, else placed around the cart. None if not in the sim."""
    meta = Struct()
    meta.update({"colors": base64.b64encode(bytes(BOX_RGB)).decode()})
    geom = Geometry(label=f"box-{box.name}", box=RectangularPrism(dims_mm=Vector3(x=box.size_mm[0], y=box.size_mm[1], z=box.size_mm[2])))
    if box.held:
        parent, pose = "vacuum", Pose(z=box.held_centre()[2], o_z=1.0)
    elif box.room is not None and cart is not None:
        x, y, yaw = room_to_cart(cart, box.room["x_mm"], box.room["y_mm"], box.room["yaw_deg"])
        parent, pose = "world", Pose(x=x, y=y, z=box.room["z_mm"], o_z=1.0, theta=yaw)
    else:
        return None
    return Transform(reference_frame=f"box-{box.name}", uuid=shape_uuid(f"box-{box.name}"), physical_object=geom,
                     pose_in_observer_frame=PoseInFrame(reference_frame=parent, pose=pose), metadata=meta)


def room_pose(cart: Mapping[str, float]) -> Pose:
    """Where the room's origin is, seen from the cart: the inverse of the cart's pose in the room."""
    yaw = math.radians(cart["yaw_deg"])
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = cart["x_mm"], cart["y_mm"]
    return Pose(x=-(c * x + s * y), y=-(-s * x + c * y), z=0.0, o_x=0.0, o_y=0.0, o_z=1.0, theta=-cart["yaw_deg"])


class Scene(WorldStateStore, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("capbot", "sim"), "scene")

    @classmethod
    def validate_config(cls, config: ServiceConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        d = attrs.get("visuals_dir")
        if not isinstance(d, str) or not (Path(d) / "visuals.json").exists():
            raise ValueError("visuals_dir must be the absolute path of 20_sim/generated/visuals (with visuals.json)")
        odom = attrs.get("odom", "odom")
        if not isinstance(odom, str):
            raise ValueError('odom must name the odometry component, e.g. "odom" ("" for a fixed arm)')
        hz = attrs.get("poll_hz", 10)
        if not isinstance(hz, (int, float)) or hz <= 0:
            raise ValueError("poll_hz must be a number greater than zero")
        return ([odom] if odom else []), []

    @classmethod
    def new(cls, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        self = super().new(config, dependencies)
        attrs = struct_to_dict(config.attributes)
        vis = Path(attrs["visuals_dir"])
        odom = attrs.get("odom", "odom")
        self.odom = dependencies[Generic.get_resource_name(odom)] if odom else None   # None: fixed arm, room stays put
        self.dt = 1.0 / float(attrs.get("poll_hz", 10))
        self.transforms: Dict[bytes, Transform] = {}
        for s in json.loads((vis / "visuals.json").read_text())["shapes"]:
            parent = "world" if s["parent"] == "room" else s["parent"]
            t = Transform(reference_frame=f"cad-{s['name']}",
                          pose_in_observer_frame=PoseInFrame(reference_frame=parent, pose=Pose(o_z=1.0)),
                          physical_object=Geometry(label=f"cad-{s['name']}",
                                                   mesh=Mesh(content_type="ply", mesh=(vis / s["file"]).read_bytes())),
                          uuid=shape_uuid(s["name"]))
            self.transforms[t.uuid] = t
        self.room_id = shape_uuid("room")
        self.last_cart: Optional[Tuple[float, float, float]] = None
        self.cart: Optional[Dict[str, float]] = None
        self.boxes: Dict[bytes, Transform] = {}          # what the scene has been sent for the boxes
        self.boxes_dirty = True
        WORLD.listeners.append(self._world_changed)
        self.subscribers: Set[asyncio.Queue] = set()
        self.task = asyncio.create_task(self._follow_odom())
        return self

    def _world_changed(self):
        self.boxes_dirty = True

    def _send(self, kind, transform, fields=None):
        for q in list(self.subscribers):
            q.put_nowait(StreamTransformChangesResponse(change_type=kind, transform=transform,
                                                        updated_fields=FieldMask(paths=fields or [])))

    def _sync_boxes(self):
        """Send added / moved / removed boxes."""
        now = {}
        for b in WORLD.boxes.values():
            t = box_transform(b, self.cart)
            if t is not None:
                now[t.uuid] = t
        for uid, t in now.items():
            if uid not in self.boxes:
                self._send(TransformChangeType.TRANSFORM_CHANGE_TYPE_ADDED, t)
            elif self.boxes[uid].pose_in_observer_frame != t.pose_in_observer_frame:
                self._send(TransformChangeType.TRANSFORM_CHANGE_TYPE_UPDATED,
                           Transform(uuid=uid, reference_frame=t.reference_frame, pose_in_observer_frame=t.pose_in_observer_frame),
                           ["pose_in_observer_frame"])
        for uid, t in self.boxes.items():
            if uid not in now:
                self._send(TransformChangeType.TRANSFORM_CHANGE_TYPE_REMOVED, Transform(uuid=uid, reference_frame=t.reference_frame))
        self.boxes = now

    async def _follow_odom(self):
        """Keep the room placed around the cart (odom is the cart's place in the room)."""
        while True:
            try:
                p = (await self.odom.do_command({"command": "pose"})) if self.odom else \
                    {"x_mm": 0.0, "y_mm": 0.0, "yaw_deg": 0.0}
                now = (p["x_mm"], p["y_mm"], p["yaw_deg"])
                moved = self.last_cart is None or max(abs(a - b) for a, b in zip(now, self.last_cart)) > 0.5
                if moved:
                    self.last_cart, self.cart = now, dict(p)
                    room = self.transforms[self.room_id]
                    room.pose_in_observer_frame.pose.CopyFrom(room_pose(p))
                    self._send(TransformChangeType.TRANSFORM_CHANGE_TYPE_UPDATED,
                               Transform(uuid=room.uuid, reference_frame=room.reference_frame,
                                         pose_in_observer_frame=room.pose_in_observer_frame),
                               ["pose_in_observer_frame.pose"])
                if moved or self.boxes_dirty:
                    self.boxes_dirty = False
                    self._sync_boxes()
            except Exception as e:  # keep following even if one read fails
                LOGGER.warning(f"scene: could not read odom: {e}")
            await asyncio.sleep(self.dt)

    async def list_uuids(self, *, extra=None, timeout=None) -> List[bytes]:
        return list(self.transforms) + list(self.boxes)

    async def get_transform(self, uuid: bytes, *, extra=None, timeout=None) -> Transform:
        t = self.transforms.get(uuid) or self.boxes.get(uuid)
        if t is None:
            raise ValueError(f"no shape with uuid {uuid.hex()}")
        return t

    async def stream_transform_changes(self, *, extra=None, timeout=None
                                       ) -> AsyncGenerator[StreamTransformChangesResponse, None]:
        q: asyncio.Queue = asyncio.Queue(maxsize=1000)
        self.subscribers.add(q)
        try:
            # Viam's stream protocol opens with one empty message, and viam-server drops the first message
            # it gets from a module (rdk services/worldstatestore server.go / client.go). Send it first.
            yield StreamTransformChangesResponse()
            while True:
                yield await q.get()
        finally:
            self.subscribers.discard(q)

    async def close(self):
        self.task.cancel()
        if self._world_changed in WORLD.listeners:
            WORLD.listeners.remove(self._world_changed)
