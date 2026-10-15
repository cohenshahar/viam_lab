# How to place an object (one full pick-and-place cycle)

From Viam 101 (exercise 14), 2026-10-14. Builds on `how_to_pick.md`.

> **Note:** the pattern is general. Names (`pallet`, `helpers.pallet_top`) and sizes come from the course's simulated cell, not from my robot.

## Key ideas

1. **Ask the target where it is:** like the pick-station, the pallet knows its own position and size. `helpers.pallet_top(robot)` returns the centre of its top face `(cx, cy, z)`.
2. **Ask once, remember:** `_pallet_top` stores the answer in `self._top`, so later boxes don't ask again.
3. **Keep the placement rule separate:** `_place_pose(i, ...)` decides where box `i` goes. Change the pattern there without touching the cycle. For now: every box to the pallet centre, one layer up.
4. **One cycle = pick + move over target + lower + release.** `gripper.open()` releases (suction off).
5. **Remember what you placed:** `self.placed` holds each box's centre; `len(self.placed)` is the next box's number.

## The new code pieces (inside the class)

```python
    async def _pallet_top(self):
        if self._top is None:
            self._top = await helpers.pallet_top(self.robot)
        return self._top

    def _place_pose(self, i, cx, cy, top):
        """Box i -> (x, y, gripper-tip z). For now: the pallet center."""
        return cx, cy, top + BOX_H

    async def place(self):
        """One full pick-and-place of the next box onto the pallet."""
        cx, cy, top = await self._pallet_top()
        seq = len(self.placed)
        x, y, z_tip = self._place_pose(seq, cx, cy, top)
        await self.pick(seq)
        await self.move_gripper(down_pose(x, y, z_tip))
        await self.gripper.open()
        await helpers.show_box(self.robot, seq, x, y, z_tip - BOX_H / 2)
        self.placed.append((x, y, z_tip - BOX_H / 2))
```

And in `STEPS`: `"place": Palletizer.place,`

## Block by block

- **`_pallet_top`:** ask the pallet only the first time (`self._top is None`), then reuse.
- **`_place_pose`:** returns where the gripper's cups go. `top + BOX_H` = the top of a box standing on the pallet.
- **`place`:**
  - `seq = len(self.placed)`: next box number (0 for the first).
  - `self.pick(seq)`: the whole pick from exercise 13.
  - `move_gripper(down_pose(x, y, z_tip))`: carry the box over the pallet and lower it.
  - `gripper.open()`: release.
  - `show_box(...)`: draw the box resting on the pallet (centre = half a box below the cups).
  - `self.placed.append(...)`: record it.

## Run it

```bash
python palletizer.py place
```

A box goes from the pick-station to the pallet centre.

## Worth knowing (from reading the code, not from the course)

Each `python palletizer.py ...` run starts a new program, so `self.placed` starts empty every time. Running `place` twice puts both boxes at the same spot (box number 0 again). Filling a whole pallet needs the boxes placed inside one run.
