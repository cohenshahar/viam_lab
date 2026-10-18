"""The sim's loose objects (the demo boxes): where each one is. Shared by the parts of this module, which all
run in one process: `cap` moves boxes (pick / place), `vacuum` reports holding, `scene` draws them.

A box is either resting in the room (centre x, y, z in mm and yaw in deg, room frame) or held on the cup
(then it rides on the `vacuum` frame: centre on the tool axis, half a box in front of the cup face).
Nothing here talks to Viam; pure state + small math, so it can be tested anywhere.
"""

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Awaitable, Callable, Dict, List, Optional


@dataclass
class Box:
    name: str
    size_mm: List[float]
    room: Optional[Dict[str, float]] = None   # {"x_mm", "y_mm", "z_mm", "yaw_deg"} while resting
    held: bool = False
    start: Optional[Dict[str, float]] = None  # where it appears (the pick station)
    waiting: bool = False                     # in the queue, not in the sim yet

    def held_centre(self) -> List[float]:
        """Centre in the vacuum frame while held: on the tool axis (+Z), touching the cup face."""
        return [0.0, 0.0, self.size_mm[2] / 2.0]


@dataclass
class World:
    boxes: Dict[str, Box] = field(default_factory=dict)
    listeners: List[Callable[[], None]] = field(default_factory=list)
    # set by `cap`: where a held box ends up (room pose of its centre) when the suction is switched off
    release_pose: Optional[Callable[[], Awaitable[Optional[Dict[str, float]]]]] = None

    def load(self, locations_path: Path):
        """(Re)start the boxes from locations.json, like the course's pick station: the first box waits at its start
        spot, the others queue and appear there one by one (see feed)."""
        data = json.loads(Path(locations_path).read_text())
        self.boxes = {name: Box(name, list(b["size_mm"]), None, start=dict(b["start"]), waiting=True)
                      for name, b in data.get("boxes", {}).items()}
        self.feed()
        self.changed()

    def feed(self):
        """Put the next queued box on its start spot when that spot is free."""
        nxt = next((b for b in self.boxes.values() if b.waiting), None)
        if nxt is None:
            return
        s = nxt.start
        busy = any(b.room is not None and math.dist((b.room["x_mm"], b.room["y_mm"], b.room["z_mm"]),
                                                     (s["x_mm"], s["y_mm"], s["z_mm"])) < 10.0
                   for b in self.boxes.values())
        if not busy:
            nxt.waiting, nxt.room = False, dict(s)

    def held_box(self) -> Optional[Box]:
        return next((b for b in self.boxes.values() if b.held), None)

    def attach(self, name: str):
        b = self.boxes[name]
        b.held, b.room = True, None
        self.changed()

    def detach(self, room_pose: Optional[Dict[str, float]]):
        """Let go of the held box; it rests at room_pose (None: nobody knows where -> it leaves the sim)."""
        b = self.held_box()
        if b:
            b.held, b.room = False, (dict(room_pose) if room_pose else None)
            self.feed()   # the arm is away from the pick station now: the next box can appear
            self.changed()

    def changed(self):
        for f in list(self.listeners):
            f()


WORLD = World()


def near_face_contact(tcp_room, tool_dir_room, box: Box, tol_mm=15.0, tol_deg=10.0):
    """Is the cup pressed on the top or a side face of a resting box? tcp_room: TCP position (mm, room frame);
    tool_dir_room: unit vector of the tool axis (+Z of the TCP) in the room. The cup must sit on the face
    centre within tol_mm and point into the box within tol_deg. Returns the face name or None."""
    if box.room is None:
        return None
    c = box.room
    yaw = math.radians(c["yaw_deg"])
    ux, uy = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
    half = [s / 2.0 for s in box.size_mm]
    faces = {"top": ((0.0, 0.0, 1.0), half[2]),
             "+x": ((ux[0], ux[1], 0.0), half[0]), "-x": ((-ux[0], -ux[1], 0.0), half[0]),
             "+y": ((uy[0], uy[1], 0.0), half[1]), "-y": ((-uy[0], -uy[1], 0.0), half[1])}
    for name, (n, h) in faces.items():
        centre = (c["x_mm"] + n[0] * h, c["y_mm"] + n[1] * h, c["z_mm"] + n[2] * h)
        if math.dist(centre, tcp_room) > tol_mm:
            continue
        into = -sum(n[i] * tool_dir_room[i] for i in range(3))   # tool points against the face normal
        if into >= math.cos(math.radians(tol_deg)):
            return name
    return None
