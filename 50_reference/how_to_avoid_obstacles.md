# How to avoid obstacles that appear at run time (WorldState)

From Viam 101 (video 16 + exercise 17), 2026-10-14. Builds on `how_to_run_loop.md`.
Full working file: `10_course/palletizer_ex17.py`.

> **Note:** the pattern is general. Box sizes and names come from the course's simulated cell, not from my robot.

## Key idea: two kinds of obstacles

| | Static obstacles | Dynamic obstacles |
| --- | --- | --- |
| Example | the floor | boxes already placed on the pallet |
| Where they live | machine config (frame + geometry) | your code, passed **per move** |
| Planner sees them | always | only in moves you pass them to |

The planner only avoids what it knows about. Before this fix, the arm drove straight through placed boxes because nobody told the planner they were there.

## WorldState

A `WorldState` is a set of obstacles. Each obstacle = a **geometry** (here a box: `RectangularPrism`) with a **center pose**, wrapped in `GeometriesInFrame`, which names the **reference frame** it is measured in. A placed box doesn't move, so its frame is `world`, at the box's center.

## The code

Imports:

```python
from viam.proto.common import (
    Pose,
    PoseInFrame,
    WorldState,
    GeometriesInFrame,
    Geometry,
    RectangularPrism,
    Vector3,
)
```

The method (inside the class, indented like the other methods):

```python
    def obstacles(self):
        """Build the planner's obstacle set from the boxes already placed."""
        def cuboid(label, frame, x, y, z):
            return GeometriesInFrame(
                reference_frame=frame,
                geometries=[Geometry(
                    center=Pose(x=x, y=y, z=z, o_x=0, o_y=0, o_z=1, theta=0),
                    box=RectangularPrism(dims_mm=Vector3(x=BOX_W, y=BOX_L, z=BOX_H)),
                    label=label,
                )],
            )

        obs = [cuboid(f"placed-{i}", "world", x, y, z)
               for i, (x, y, z) in enumerate(self.placed)]
        return WorldState(obstacles=obs) if obs else None
```

Pass it on the moves near the stack:

```python
await self.move_gripper(home, self.obstacles())                     # in pick, twice
await self.move_gripper(down_pose(x, y, z_tip), self.obstacles())   # in place
```

## Block by block

- **`cuboid(...)`:** builds one obstacle: a box the size of a real box (`BOX_W x BOX_L x BOX_H`), centered at (x, y, z), upright (`o_z=1`), in the named frame, with a label.
- **`obs = [...]`:** one cuboid per entry in `self.placed`, all in `world`.
- **`WorldState(...) if obs else None`:** empty pallet → nothing to avoid → `None`.
- **`self.obstacles()` with parentheses:** call it on every move, so each move gets the **current** set. Without `()` you'd pass the method itself, not the obstacles.

## Result

The arm arcs around the stack instead of through it. If a move can only be done by going through a box, the planner now **refuses** it instead of doing it: no silent collisions.

## Mistake I made (2026-10-14)

Put `def obstacles(self):` at the left margin above `class Palletizer:`. It became a plain function, not a method, so `self.obstacles()` failed with `AttributeError: 'Palletizer' object has no attribute 'obstacles'`. Fix: move it inside the class and indent it to match the other methods (select + Tab in VS Code).

## Open point from the course

The arm body now avoids the boxes, but the box **in the gripper** still clips its neighbors. That is exercise 18.
