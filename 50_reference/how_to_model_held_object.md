# How to model a held object (moving obstacle) + approach-then-descend

From Viam 101 (exercise 18, the last build step), 2026-10-14. Builds on `how_to_avoid_obstacles.md`.
Full working file: `10_course/palletizer_final.py`.

> **Note:** the pattern is general. Box sizes, `gripper-1` and the 10 mm margin come from the course's simulated cell, not from my robot.

## Key ideas

1. **The frame decides fixed vs moving.** Same mechanism as the placed boxes (a geometry in a frame), but:
   - placed box → `world` frame → stays put.
   - held box → **gripper frame** → rides along with the gripper, so the planner keeps the whole load clear of the stack.
2. **The catch:** at the moment of placing, the held box touches its neighbors and the box below. The planner reads that as a collision and refuses the move.
3. **The fix: split the placement into two moves.**
   - **Approach:** go over the slot at a safe height, **with** the held box modeled (no dragging through anything).
   - **Descend:** lower straight down into the slot, **without** the held box (a vertical lower is not a drag).
4. **Model the held box on every move while it is held**, including the lift after grabbing.

## The code changes

`obstacles` gets a `held` flag:

```python
    def obstacles(self, held=False):
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
        if held:
            obs.append(cuboid("held", helpers.GRIPPER, 0, 0, BOX_H / 2))
        return WorldState(obstacles=obs) if obs else None
```

A safe transit height:

```python
    def _clear_tip(self, z_tip):
        if not self.placed:
            return z_tip
        max_top = max(z + BOX_H / 2 for (_, _, z) in self.placed)
        return max(z_tip, max_top + BOX_H + 10.0)
```

In `pick`, the lift after `grab()`:

```python
        await self.move_gripper(home, self.obstacles(held=True))
```

In `place`, two moves instead of one:

```python
        # 1) over the slot, held box modeled, so it cannot drag through the stack
        await self.move_gripper(
            down_pose(x, y, self._clear_tip(z_tip)), self.obstacles(held=True)
        )
        # 2) straight down into the slot (a vertical lower is not a drag)
        await self.move_gripper(down_pose(x, y, z_tip), self.obstacles())
```

## Block by block

- **`cuboid("held", helpers.GRIPPER, 0, 0, BOX_H / 2)`:** the held box, in the gripper's own frame, centered half a box height out from the cups (along the gripper's z-axis). It goes wherever the gripper goes.
- **`_clear_tip`:**
  - `max_top` = the top face of the tallest placed box (center + half a box).
  - `max_top + BOX_H + 10.0` = cups high enough that the **bottom** of the held box is 10 mm above that top face.
  - `max(z_tip, ...)`: never lower than the slot's own height. Empty pallet → just use the slot height.
- **Approach move:** to (x, y, safe height), with `held=True`.
- **Descend move:** to (x, y, slot height), without the held box, so touching neighbors is allowed.

## Result

The arm lifts each box up and over the stack, then lowers it straight into its slot. Eight boxes, two clean layers, no clipping.

## Mistake I made again (2026-10-14)

`def obstacles(self, held=False):` ended up at the left margin again. This time everything after it (`_clear_tip`, `_place_pose`, `place`, `run`) fell out of the class too. Rule: **every method starts 4 spaces in, lined up with `def __init__`.** If a method "doesn't exist", check its indentation first.
