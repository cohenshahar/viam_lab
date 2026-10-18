"""capbot:sim:camera - a stand-in camera for the sim: a plain test image, and no point cloud.

Why not rdk:builtin:fake: every fake camera serves a sample point cloud (with or without its "model" attribute),
and the 3D scene draws each one as a big blue / yellow plane in the room (seen 18 Oct). This one answers
"no point cloud", so the scene shows only the camera's frame. The real cameras replace it under the same names.

Attributes:
    width, height (int)   image size, default 640 x 480
    shade (int 0..255)    grey level of the test image, default 160
"""

import struct
import zlib
from datetime import datetime
from typing import ClassVar, Mapping, Sequence, Tuple

from google.protobuf.timestamp_pb2 import Timestamp
from typing_extensions import Self
from viam.components.camera import Camera
from viam.media.video import CameraMimeType, NamedImage
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, Pose, RectangularPrism, ResourceName, ResponseMetadata, Vector3
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import struct_to_dict


def test_png(w: int, h: int, shade: int) -> bytes:
    """A grey PNG with a darker 8 px border and a centre cross, so it is clearly a test picture."""
    rows = []
    for y in range(h):
        row = bytearray([0])                       # filter type 0 for each row
        for x in range(w):
            edge = x < 8 or y < 8 or x >= w - 8 or y >= h - 8
            cross = abs(x - w // 2) < 2 or abs(y - h // 2) < 2
            v = max(0, shade - 90) if edge or cross else shade
            row += bytes((v, v, v))
        rows.append(bytes(row))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(b"".join(rows), 6)) + chunk(b"IEND", b""))


class SimCamera(Camera, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("capbot", "sim"), "camera")

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        a = struct_to_dict(config.attributes)
        for k in ("width", "height"):
            if k in a and (not isinstance(a[k], (int, float)) or not 16 <= a[k] <= 4096):
                raise ValueError(f"{k} must be a number of pixels from 16 to 4096")
        if "shade" in a and (not isinstance(a["shade"], (int, float)) or not 0 <= a["shade"] <= 255):
            raise ValueError("shade must be from 0 to 255")
        return [], []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        self = super().new(config, dependencies)
        a = struct_to_dict(config.attributes)
        self.png = test_png(int(a.get("width", 640)), int(a.get("height", 480)), int(a.get("shade", 160)))
        return self

    async def get_images(self, *, filter_source_names=None, extra=None, timeout=None, **kwargs):
        ts = Timestamp()
        ts.FromDatetime(datetime.utcnow())
        return [NamedImage(self.name, self.png, CameraMimeType.PNG)], ResponseMetadata(captured_at=ts)

    async def get_point_cloud(self, *, extra=None, timeout=None, **kwargs) -> Tuple[bytes, str]:
        raise NotImplementedError("capbot:sim:camera has no depth: no point cloud (the real camera will have one)")

    async def get_properties(self, *, timeout=None, **kwargs) -> Camera.Properties:
        return Camera.Properties(supports_pcd=False, mime_types=["image/png"])

    async def get_geometries(self, *, extra=None, timeout=None):
        # the same 40 x 40 x 10 mm box the fake camera reports: the planner keeps the camera body as a collider
        # (and the allowed pairs cam-wrist <-> kr6:link_4/5/6 need a cam-wrist shape to point at)
        return [Geometry(center=Pose(o_z=1), box=RectangularPrism(dims_mm=Vector3(x=40, y=40, z=10)), label="box")]
