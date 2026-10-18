"""capbot:sim:cap - the thesis primitives that Viam does not have, as DoCommand verbs (Phase 8, decision #13).

Viam first: the verbs only glue Viam calls together (motion.move for the arm, base spin / move_straight /
set_velocity for the cart, gripper grab / open). Verb list and names: 20_sim/primitive_map.md (decision #33).

Call:  await cap.do_command({"command": "<verb>", ...arguments})
Every answer is {"ok": true, ...} or {"error": "...", "needs_clarification": true|false} (never an exception).
needs_clarification = the request itself was unclear (unknown name, missing argument); false = it was clear
but could not be done (unreachable, collision, e-stop).

Verbs (lengths mm, angles deg; frame "world" = the cart frame (decision #29), "room" = the lab):
  status                                       e-stop, cart pose, arm joints, tool, boxes
  locations                                    named cart poses and box spots (locations.json)
  go_to_named_pose  name                       arm to a thesis named pose (collision-checked)
  approach          x y z ox oy oz [theta] [standoff_mm] [frame]   tool to a pose: standoff, then straight in
  move_relative     dx dy dz [frame: world|tool] [linear]           tool moves by an offset, orientation kept
  pick              [box]                      pick a crate from above (default: the one on the pick station)
  place             spot                       set the held crate down from above ("pallet": next free slot)
  drive_to          x_mm y_mm yaw_deg          cart to a room pose: back out, turn, straight, turn, drive in.
                                               The arm is tucked in first; the whole path is checked against the
                                               room and the crates and refused if the cart would touch anything
  go_to_location    name                       drive_to a named place
  drive_arc         radius_mm angle_deg [speed_mm_per_sec]
  estop / release_estop                        latch: stops arm + cart, refuses motion verbs until released
  reset_boxes                                  boxes back to their start spots (sim only)

Attributes (all paths absolute on this computer; written by make_configs.py):
  numbers_path, kinematics_path, locations_path
  has_cart (true): false for a fixed arm (the arm arena): no cart / odom; drive verbs refuse
  world_path: a fixed arm's room (obstacles + kr6_mount_mm), e.g. 20_sim/generated/arena/arena_numbers.json
  drive_speed_mm_per_sec (250), turn_speed_deg_per_sec (45), standoff_mm (100), lift_mm (30),
  retreat_mm (150), verb_timeout_sec (120), dock_mm (250: back out / drive in straight around spins),
  carry_tcp_mm ([350, 0, 1000]: cup position, tool down, while driving with a crate)
"""

import asyncio
import json
import math
import sys
from pathlib import Path
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
from typing_extensions import Self
from viam.components.arm import Arm
from viam.components.base import Base
from viam.components.generic import Generic as GenericComponent
from viam.components.gripper import Gripper
from viam.logging import getLogger
from viam.proto.app.robot import ServiceConfig
from viam.proto.common import (Geometry, GeometriesInFrame, Pose, PoseInFrame, RectangularPrism, ResourceName,
                               Transform, Vector3, WorldState)
from viam.proto.service.motion import CollisionSpecification, Constraints, LinearConstraint
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.services.generic import Generic
from viam.services.motion import Motion
from viam.utils import ValueTypes, struct_to_dict

import cap_math as cm
from sim_world import WORLD, near_face_contact

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import viam_frames as vf  # noqa: E402

LOGGER = getLogger(__name__)
DEPS = ["kr6", "vacuum", "builtin"]          # + "cart", "odom" when has_cart (the default)
CART_FOOTPRINT_MM = (-570.0, 518.0, -350.0, 350.0)   # cart outline in its own frame (mesh bounds); the arm stays
                                                     # inside it while driving (folded, or the carry pose)
# Shape pairs the planner may let touch (same list as capbot_common.arm_constraints):
#  - the wrist camera is bolted to the wrist (Viam hangs it on the arm's end; its 40 mm box sits in link 4's box)
#  - link_2 / link_4 and link_4 / link_6: their mesh-bound boxes overlap whenever the elbow or wrist folds
#    (home_folded, ready_high, drive_look, side picks); the real KR6 cannot hit itself there inside its joint limits
ALLOWED_PAIRS = [("cam-wrist", "kr6:link_4"), ("cam-wrist", "kr6:link_5"), ("cam-wrist", "kr6:link_6"),
                 ("kr6:link_2", "kr6:link_4"), ("kr6:link_4", "kr6:link_6")]


class Unclear(Exception):
    """The request itself is unclear -> needs_clarification: true."""


class Refused(Exception):
    """Clear request that cannot be done (e-stop, no box under the cup, ...)."""


def _num(cmd, key, default=None):
    v = cmd.get(key, default)
    if not isinstance(v, (int, float)) or isinstance(v, bool):
        raise Unclear(f"{key} must be a number" + ("" if default is None else f" (default {default})"))
    return float(v)


def _down(xyz, theta=0.0) -> Pose:
    """Tool pointing straight down at xyz (cart frame), turned theta degrees about the vertical."""
    return Pose(x=xyz[0], y=xyz[1], z=xyz[2], o_x=0.0, o_y=0.0, o_z=-1.0, theta=theta)


def _pose(xyz, d, theta=0.0) -> Pose:
    """Pose with the tool axis (+Z of the TCP) along direction d."""
    return Pose(x=xyz[0], y=xyz[1], z=xyz[2], o_x=d[0], o_y=d[1], o_z=d[2], theta=theta)


class Cap(Generic, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("capbot", "sim"), "cap")

    @classmethod
    def validate_config(cls, config: ServiceConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        for key in ("numbers_path", "kinematics_path", "locations_path"):
            v = attrs.get(key)
            if not isinstance(v, str) or not Path(v).exists():
                raise ValueError(f"{key} must be an absolute path that exists on this computer (re-run make_configs.py)")
        for key in ("drive_speed_mm_per_sec", "turn_speed_deg_per_sec", "standoff_mm", "lift_mm", "retreat_mm",
                    "verb_timeout_sec"):
            if key in attrs and (not isinstance(attrs[key], (int, float)) or attrs[key] <= 0):
                raise ValueError(f"{key} must be a number greater than zero")
        if "world_path" in attrs and (not isinstance(attrs["world_path"], str) or not Path(attrs["world_path"]).exists()):
            raise ValueError("world_path must be an absolute path that exists on this computer")
        return DEPS + (["cart", "odom"] if attrs.get("has_cart", True) else []), []

    @classmethod
    def new(cls, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        self = super().new(config, dependencies)
        a = struct_to_dict(config.attributes)
        self.arm = dependencies[Arm.get_resource_name("kr6")]
        self.has_cart = bool(a.get("has_cart", True))
        self.cart = dependencies[Base.get_resource_name("cart")] if self.has_cart else None
        self.vacuum = dependencies[Gripper.get_resource_name("vacuum")]
        self.odom = dependencies[GenericComponent.get_resource_name("odom")] if self.has_cart else None
        self.motion = dependencies[Motion.get_resource_name("builtin")]
        self.numbers = json.loads(Path(a["numbers_path"]).read_text())
        self.kr6 = json.loads(Path(a["kinematics_path"]).read_text())
        self.locations_path = Path(a["locations_path"])
        self.locations = json.loads(self.locations_path.read_text())
        vfr = self.numbers["viam_frames"]
        self.mount = vf.pose(t=np.array(vfr["kr6_translation_mm"], float))
        # a fixed arm (the arm arena): its own room and mount, in Viam's world frame (= the room)
        if a.get("world_path"):
            w = json.loads(Path(a["world_path"]).read_text())
            self.numbers = {**self.numbers, "world": {"obstacles": w["obstacles"]}}
            self.mount = vf.pose(t=np.array(w["kr6_mount_mm"], float))
        self.tcp = vf.pose(t=np.array(vfr["vacuum_translation_mm"], float))
        self.named = {k: v for k, v in self.numbers["named_arm_poses_deg"].items() if k != "source"}
        self.v = float(a.get("drive_speed_mm_per_sec", 250))
        self.w = float(a.get("turn_speed_deg_per_sec", 45))
        self.standoff = float(a.get("standoff_mm", 100))
        self.lift = float(a.get("lift_mm", 30))
        self.retreat = float(a.get("retreat_mm", 150))
        self.timeout = float(a.get("verb_timeout_sec", 120))
        self.dock = float(a.get("dock_mm", 250))
        self.carry_tcp = [float(v) for v in a.get("carry_tcp_mm", [350, 0, 1000])]
        self.estopped = False
        self.running: Optional[asyncio.Task] = None
        self.held_yaw_offset = 0.0    # box yaw minus tool heading, set when picked
        WORLD.load(self.locations_path)
        WORLD.release_pose = self._release_pose
        return self

    # ---- where things are ----------------------------------------------------------------------

    async def _cart_pose(self) -> Dict[str, float]:
        if not self.has_cart:   # fixed arm: the "cart" frame is the room
            return {"x_mm": 0.0, "y_mm": 0.0, "yaw_deg": 0.0}
        p = await self.odom.do_command({"command": "pose"})
        return {"x_mm": p["x_mm"], "y_mm": p["y_mm"], "yaw_deg": p["yaw_deg"]}

    async def _tcp_in_cart(self) -> np.ndarray:
        """4x4 TCP pose in the cart frame, from the arm joints (our FK, checked against MuJoCo and Viam)."""
        q = list((await self.arm.get_joint_positions()).values)
        return self.mount @ vf.sva_fk(self.kr6, q)["flange"] @ self.tcp

    @staticmethod
    def _tool_heading(t, cart) -> float:
        """Room heading (deg) of the tool's x axis: how the cup is turned about the vertical."""
        return cart["yaw_deg"] + math.degrees(math.atan2(t[1, 0], t[0, 0]))

    async def _carry(self):
        """Arm in its driving pose: folded when empty, the carry pose (crate over the cart) when holding."""
        if WORLD.held_box():
            c = self.carry_tcp
            await self._move_tool(_down(c, 0.0), step="carry pose over the cart")
        else:
            await self._named("home_folded")

    def _floor_plan(self, skip_held=True) -> Dict[str, list]:
        """2D outlines (room frame) the cart must not touch: room obstacles (with margin) and resting crates."""
        out = {}
        for name, o in self.numbers["world"]["obstacles"].items():
            if name == "floor":
                continue
            lo, hi = o["planning_box_mm"]["min"], o["planning_box_mm"]["max"]
            out[name] = [(lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1])]
        for b in WORLD.boxes.values():
            if b.room is not None:
                out[f"crate {b.name}"] = cm._rect(b.room["x_mm"], b.room["y_mm"], b.room["yaw_deg"],
                                                  -b.size_mm[0] / 2, b.size_mm[0] / 2, -b.size_mm[1] / 2, b.size_mm[1] / 2)
        return out

    async def _release_pose(self) -> Optional[Dict[str, float]]:
        """Room pose of the held box's centre right now (vacuum hook, used when the suction goes off)."""
        b = WORLD.held_box()
        if b is None:
            return None
        t, cart = await self._tcp_in_cart(), await self._cart_pose()
        d = t[:3, 2]
        centre = cm.cart_to_room(cart, cm.along(t[:3, 3], d, b.size_mm[2] / 2))
        return {"x_mm": centre[0], "y_mm": centre[1], "z_mm": centre[2],
                "yaw_deg": cm.wrap_deg(self._tool_heading(t, cart) + self.held_yaw_offset)}

    def _world_state(self, cart: Dict[str, float], held_modeled: bool) -> WorldState:
        """Room obstacles + resting boxes, in the cart frame; the held box rides on `vacuum` if modeled."""
        geoms = []
        for label, o in self.numbers["world"]["obstacles"].items():
            lo, hi = o["planning_box_mm"]["min"], o["planning_box_mm"]["max"]
            c = cm.room_to_cart(cart, [(lo[i] + hi[i]) / 2 for i in range(3)])
            geoms.append(Geometry(center=Pose(x=c[0], y=c[1], z=c[2], o_z=1.0, theta=-cart["yaw_deg"]), label=label,
                                  box=RectangularPrism(dims_mm=Vector3(x=hi[0] - lo[0], y=hi[1] - lo[1], z=hi[2] - lo[2]))))
        for b in WORLD.boxes.values():
            if b.room is not None:
                c = cm.room_to_cart(cart, (b.room["x_mm"], b.room["y_mm"], b.room["z_mm"]))
                geoms.append(Geometry(center=Pose(x=c[0], y=c[1], z=c[2], o_z=1.0, theta=b.room["yaw_deg"] - cart["yaw_deg"]),
                                      label=f"box-{b.name}", box=RectangularPrism(dims_mm=Vector3(x=b.size_mm[0], y=b.size_mm[1], z=b.size_mm[2]))))
        ws = WorldState(obstacles=[GeometriesInFrame(reference_frame="world", geometries=geoms)])
        held = WORLD.held_box()
        if held and held_modeled:
            # A shape given as an obstacle in the vacuum frame is pinned where it is at the start of the move
            # (found 17 Oct: place collided with its own box). A transform adds a frame that hangs on `vacuum`
            # and carries the box, so it really rides with the tool.
            ws.transforms.append(Transform(
                reference_frame=f"held-{held.name}",
                pose_in_observer_frame=PoseInFrame(reference_frame="vacuum", pose=Pose(z=held.held_centre()[2], o_z=1.0)),
                physical_object=Geometry(label=f"held-{held.name}", center=Pose(o_z=1.0), box=RectangularPrism(
                    dims_mm=Vector3(x=held.size_mm[0], y=held.size_mm[1], z=held.size_mm[2])))))
        return ws

    def _constraints(self, linear: bool) -> Constraints:
        allow = CollisionSpecification.AllowedFrameCollisions
        pairs = [allow(frame1=a, frame2=b) for a, b in ALLOWED_PAIRS]
        c = Constraints(collision_specification=[CollisionSpecification(allows=pairs)])
        if linear:
            c.linear_constraint.append(LinearConstraint(line_tolerance_mm=2.0, orientation_tolerance_degs=2.0))
        return c

    async def _move_tool(self, pose: Pose, linear=False, held_modeled=True, step=""):
        """vacuum to pose (cart frame) through Viam's motion service; the planner moves the arm only."""
        if self.estopped:
            raise Refused("e-stop is on: call release_estop first")
        cart = await self._cart_pose()
        try:
            await self.motion.move("vacuum", PoseInFrame(reference_frame="world", pose=pose),
                                   world_state=self._world_state(cart, held_modeled), constraints=self._constraints(linear))
        except Exception as e:
            where = f"step '{step}': " if step else ""
            raise Refused(f"{where}motion planner: {str(e).splitlines()[0][:300]}")

    # ---- verbs ---------------------------------------------------------------------------------

    async def v_status(self, cmd):
        t = await self._tcp_in_cart()
        return {"ok": True, "estop": self.estopped, "cart": await self._cart_pose(),
                "arm_joints_deg": [round(x, 3) for x in (await self.arm.get_joint_positions()).values],
                "tcp_in_cart_mm": [round(float(x), 1) for x in t[:3, 3]],
                "holding": (await self.vacuum.is_holding_something()).is_holding_something,
                "boxes": {n: ("held" if b.held else "waiting" if b.waiting else b.room) for n, b in WORLD.boxes.items()},
                "box_sizes_mm": {n: b.size_mm for n, b in WORLD.boxes.items()}, "has_cart": self.has_cart}

    async def v_locations(self, cmd):
        return {"ok": True, "places": self.locations["places"], "box_spots": self.locations.get("box_spots", {}),
                "named_arm_poses": sorted(self.named)}

    async def v_go_to_named_pose(self, cmd):
        name = cmd.get("name")
        if name not in self.named:
            raise Unclear(f"unknown pose {name!r}; known: {', '.join(sorted(self.named))}")
        await self._named(name)
        return {"ok": True, "pose": name}

    async def _named(self, name):
        if self.estopped:
            raise Refused("e-stop is on: call release_estop first")
        cart = await self._cart_pose()
        goal = {"configuration": {"kr6": [math.radians(q) for q in self.named[name]]}}
        here = await self._tcp_in_cart()
        try:   # a joint goal through the planner, so the move is collision-checked like every other
            await self.motion.move("vacuum", PoseInFrame(reference_frame="world", pose=_pose(here[:3, 3], here[:3, 2])),
                                   world_state=self._world_state(cart, True), constraints=self._constraints(False),
                                   extra={"goal_state": goal})
        except Exception as e:
            raise Refused(f"named pose {name}: motion planner: {str(e).splitlines()[0][:300]}")

    async def v_approach(self, cmd):
        p = [_num(cmd, k) for k in ("x", "y", "z")]
        d = cm.unit([_num(cmd, k) for k in ("ox", "oy", "oz")])
        theta = _num(cmd, "theta", 0.0)
        standoff = _num(cmd, "standoff_mm", self.standoff)
        frame = cmd.get("frame", "world")
        if frame == "room":
            cart = await self._cart_pose()
            p = list(cm.room_to_cart(cart, p))
            yaw = math.radians(-cart["yaw_deg"])
            d = (math.cos(yaw) * d[0] - math.sin(yaw) * d[1], math.sin(yaw) * d[0] + math.cos(yaw) * d[1], d[2])
        elif frame != "world":
            raise Unclear('frame must be "world" (the cart frame) or "room"')
        await self._move_tool(_pose(cm.along(p, d, -standoff), d, theta))
        await self._move_tool(_pose(p, d, theta), linear=True)
        return {"ok": True}

    async def v_move_relative(self, cmd):
        off = [_num(cmd, k, 0.0) for k in ("dx", "dy", "dz")]
        frame = cmd.get("frame", "world")
        t = await self._tcp_in_cart()
        if frame == "tool":
            off = list(t[:3, :3] @ np.array(off))
        elif frame != "world":
            raise Unclear('frame must be "world" (the cart frame) or "tool"')
        ox, oy, oz, th = vf.matrix_to_ov(t[:3, :3], snap=False)
        target = Pose(x=t[0, 3] + off[0], y=t[1, 3] + off[1], z=t[2, 3] + off[2], o_x=ox, o_y=oy, o_z=oz, theta=th)
        await self._move_tool(target, linear=bool(cmd.get("linear", True)))
        return {"ok": True}

    async def v_pick(self, cmd):
        name = cmd.get("box") or self._next_at_station()
        if name not in WORLD.boxes:
            raise Unclear(f"which box? known: {', '.join(WORLD.boxes) or 'none'}")
        if WORLD.boxes[name].waiting:
            raise Refused(f"{name} is still in the queue (it appears at the pick station when its turn comes)")
        if WORLD.held_box():
            raise Refused(f"already holding {WORLD.held_box().name}: place it first")
        box = WORLD.boxes[name]
        if box.room is None:
            raise Refused(f"{name} is not in the sim (call reset_boxes)")
        cart = await self._cart_pose()
        c = cm.room_to_cart(cart, (box.room["x_mm"], box.room["y_mm"], box.room["z_mm"]))
        top = (c[0], c[1], c[2] + box.size_mm[2] / 2)                           # centre of the crate's top face
        theta = cm.wrap_deg(box.room["yaw_deg"] - cart["yaw_deg"])              # cup square to the crate
        up = (0.0, 0.0, 1.0)
        await self._move_tool(_down(cm.along(top, up, self.standoff), theta), step="above the crate")  # 1.
        await self._move_tool(_down(top, theta), linear=True, step="straight down onto the crate")     # 2.
        t = await self._tcp_in_cart()                                           # 3. is the cup on its top?
        tool_dir_room = cm.unit(np.array(cm.cart_to_room(cart, t[:3, 2])) - np.array(cm.cart_to_room(cart, (0, 0, 0))))
        if not near_face_contact(cm.cart_to_room(cart, t[:3, 3]), tool_dir_room, box):
            raise Refused("the cup is not on the crate (no seal)")
        self.held_yaw_offset = cm.wrap_deg(box.room["yaw_deg"] - self._tool_heading(t, cart))
        WORLD.attach(name)                                                      # 4. suction on
        if not await self.vacuum.grab():
            WORLD.detach(box.room)
            raise Refused("suction did not hold")
        await self._move_tool(_down(cm.along(top, up, self.lift), theta), linear=True, held_modeled=False,
                              step="lift off the table")                        # 5. straight up (it touches the table)
        await self._move_tool(_down(cm.along(top, up, self.standoff), theta), linear=True,
                              step="up with the crate")                          # 6. up, crate modeled
        if self.has_cart:
            await self._carry()                                                 # 7. carry pose over the cart
        return {"ok": True, "box": name, "holding": (await self.vacuum.is_holding_something()).is_holding_something}

    def _next_at_station(self):
        """The box resting on the pick station (or None)."""
        s = self.locations.get("pick_station")
        for b in WORLD.boxes.values():
            if s and b.room is not None and math.dist((b.room["x_mm"], b.room["y_mm"], b.room["z_mm"]),
                                                        (s["x_mm"], s["y_mm"], s["z_mm"])) < 10.0:
                return b.name
        return None

    @staticmethod
    def _next_slot(spot):
        """First pallet slot with no box resting on it -> (index, slot) or (None, None) when full."""
        for i, s in enumerate(spot["slots"]):
            if not any(b.room is not None and math.dist((b.room["x_mm"], b.room["y_mm"], b.room["z_mm"]),
                                                         (s["x_mm"], s["y_mm"], s["z_mm"])) < 10.0
                       for b in WORLD.boxes.values()):
                return i, s
        return None, None

    async def v_place(self, cmd):
        spots = self.locations.get("box_spots", {})
        spot = cmd.get("spot")
        if spot not in spots:
            raise Unclear(f"which spot? known: {', '.join(spots) or 'none'}")
        held = WORLD.held_box()
        if not held:
            raise Refused("not holding a box")
        cart = await self._cart_pose()
        s, slot_no = spots[spot], None
        if s.get("type") == "pallet":   # like the course palletizer: the next free slot, layer by layer
            slot_no, s = self._next_slot(s)
            if s is None:
                raise Refused("the pallet is full")
        c = cm.room_to_cart(cart, (s["x_mm"], s["y_mm"], s["z_mm"]))
        h = held.size_mm[2]
        final = (c[0], c[1], c[2] + h / 2)                    # cup on the crate's top when it sits in the slot
        theta = cm.wrap_deg(s["yaw_deg"] - cart["yaw_deg"] - self.held_yaw_offset)
        up = (0.0, 0.0, 1.0)
        above = cm.along(final, up, h + self.lift)            # the hanging crate clears the crates beside its slot
        await self._move_tool(_down(above, theta), step="above the slot, crate modeled")          # 1.
        await self._move_tool(_down(final, theta), linear=True, held_modeled=False,
                              step="straight down into the slot")                                 # 2. (touches)
        await self.vacuum.open()                                                                    # 3. let go
        b = WORLD.boxes[held.name]
        if b.room is not None:   # sits square on its spot
            b.room.update({"x_mm": s["x_mm"], "y_mm": s["y_mm"], "z_mm": s["z_mm"], "yaw_deg": s["yaw_deg"]})
            WORLD.changed()
        await self._move_tool(_down(above, theta), linear=True, step="straight up")                # 4.
        out = {"ok": True, "box": held.name, "at": b.room}
        if slot_no is not None:
            out["pallet_slot"] = slot_no + 1
        return out

    async def _drive(self, goal):
        """Arm tucked in first, then the cart's whole path is checked against the room and the crates
        (Viam's planner does not plan a base), then turn / straight moves with docking."""
        if self.estopped:
            raise Refused("e-stop is on: call release_estop first")
        if not self.has_cart:
            raise Refused("this machine has no cart")
        start = await self._cart_pose()
        # back out as far as the place we are at asks (side-on spots need long run-ins), drive in as the goal asks
        here = next((p for p in self.locations["places"].values()
                     if math.hypot(p["x_mm"] - start["x_mm"], p["y_mm"] - start["y_mm"]) < 30), None)
        out_mm = (here or {}).get("dock_mm", self.dock)
        steps = cm.drive_to_plan(start, goal, dock_mm=out_mm, dock_in_mm=goal.get("dock_mm", self.dock))
        hit = cm.first_hit(cm.sweep(start, steps), CART_FOOTPRINT_MM, self._floor_plan())
        if hit:
            raise Refused(f"the cart would touch {hit[0]} at x={hit[1][0]} y={hit[1][1]} yaw={hit[1][2]}; not moving")
        if steps:
            await self._carry()
        for kind, value in steps:
            if self.estopped:
                raise Refused("e-stop")
            if kind == "spin":
                await self.cart.spin(value, self.w)
            else:
                await self.cart.move_straight(int(round(value)), self.v)
        return {"ok": True, "cart": await self._cart_pose()}

    async def v_drive_to(self, cmd):
        return await self._drive({"x_mm": _num(cmd, "x_mm"), "y_mm": _num(cmd, "y_mm"), "yaw_deg": _num(cmd, "yaw_deg")})

    async def v_go_to_location(self, cmd):
        places = self.locations["places"]
        name = cmd.get("name")
        if name not in places:
            raise Unclear(f"unknown place {name!r}; known: {', '.join(places)}")
        return await self._drive(places[name])

    async def v_drive_arc(self, cmd):
        if self.estopped:
            raise Refused("e-stop is on: call release_estop first")
        if not self.has_cart:
            raise Refused("this machine has no cart")
        try:
            v, w, secs = cm.arc_plan(_num(cmd, "radius_mm"), _num(cmd, "angle_deg"), _num(cmd, "speed_mm_per_sec", self.v))
        except ValueError as e:
            raise Unclear(str(e))
        start = await self._cart_pose()
        poses, p = [], start
        n = max(2, int(abs(v) * secs / 25))
        from drive_math import Pose as DPose, step as dstep
        q = DPose(start["x_mm"], start["y_mm"], start["yaw_deg"])
        for _ in range(n):
            q = dstep(q, v, w, secs / n)
            poses.append((q.x_mm, q.y_mm, q.yaw_deg))
        hit = cm.first_hit(poses, CART_FOOTPRINT_MM, self._floor_plan())
        if hit:
            raise Refused(f"the cart would touch {hit[0]} at x={hit[1][0]} y={hit[1][1]}; not moving")
        await self._carry()
        await self.cart.set_velocity(Vector3(y=v), Vector3(z=w))
        try:
            await asyncio.sleep(secs)
        finally:
            await self.cart.stop()
        return {"ok": True, "cart": await self._cart_pose()}

    async def v_estop(self, cmd):
        self.estopped = True
        if self.running and not self.running.done():
            self.running.cancel()
        await self.arm.stop()
        if self.cart:
            await self.cart.stop()
        return {"ok": True, "estop": True}

    async def v_release_estop(self, cmd):
        self.estopped = False
        return {"ok": True, "estop": False}

    async def v_reset_boxes(self, cmd):
        WORLD.load(self.locations_path)
        return {"ok": True, "boxes": {n: b.room for n, b in WORLD.boxes.items()}}

    # ---- DoCommand ------------------------------------------------------------------------------

    VERBS = ("status", "locations", "go_to_named_pose", "approach", "move_relative", "pick", "place", "drive_to",
             "go_to_location", "drive_arc", "estop", "release_estop", "reset_boxes")
    INSTANT = ("status", "locations", "estop", "release_estop", "reset_boxes")

    async def do_command(self, command: Mapping[str, ValueTypes], *, timeout=None, **kwargs) -> Mapping[str, ValueTypes]:
        verb = command.get("command")
        if verb not in self.VERBS:
            return {"error": f"unknown command {verb!r}; known: {', '.join(self.VERBS)}", "needs_clarification": True}
        fn = getattr(self, f"v_{verb}")
        try:
            if verb in self.INSTANT:
                return await fn(command)
            if self.running and not self.running.done():
                raise Refused("another motion verb is still running")
            self.running = asyncio.ensure_future(fn(command))
            return await asyncio.wait_for(self.running, self.timeout)
        except Unclear as e:
            return {"error": str(e), "needs_clarification": True}
        except Refused as e:
            return {"error": str(e), "needs_clarification": False}
        except asyncio.TimeoutError:
            await self.arm.stop()
            if self.cart:
                await self.cart.stop()
            return {"error": f"{verb} took longer than {self.timeout:.0f} s; arm and cart stopped", "needs_clarification": False}
        except asyncio.CancelledError:
            return {"error": f"{verb} stopped by e-stop", "needs_clarification": False}
        except Exception as e:   # anything unexpected: report it, never crash the verb call
            LOGGER.exception(f"cap {verb} failed")
            return {"error": f"{verb} failed: {e}", "needs_clarification": False}
