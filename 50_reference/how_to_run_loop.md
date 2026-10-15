# How to turn one cycle into a full job (placement pattern + loop)

From Viam 101 (exercise 15), 2026-10-14. Builds on `how_to_place.md`.

> **Note:** the pattern is general. The 2x2x2 grid, box sizes and names come from the course's simulated cell, not from my robot.

## Key ideas

1. **A job = the same cycle many times, to different spots.** The cycle (`place`) doesn't change; only the rule for *where* changes.
2. **The rule lives in one method** (`_place_pose`), so you swap the pattern without touching the cycle.
3. **Make the whole job the default:** a bare `python palletizer.py` runs everything.
4. **Iterate fast:** in simulation, edit → run → watch takes seconds.

## The placement pattern: two layers of a 2x2 grid

```python
    def _place_pose(self, i, cx, cy, top):
        """Box i -> (x, y, gripper-tip z): two layers of a 2x2 grid."""
        layer, slot = i // 4, i % 4
        col, row = slot % 2, slot // 2
        x = cx + (col - 0.5) * BOX_W
        y = cy + (row - 0.5) * BOX_L
        z_tip = top + (layer + 1) * BOX_H
        return x, y, z_tip
```

How the numbers work:

| i | layer = i // 4 | slot = i % 4 | col = slot % 2 | row = slot // 2 |
| --- | --- | --- | --- | --- |
| 0 | 0 | 0 | 0 | 0 |
| 1 | 0 | 1 | 1 | 0 |
| 2 | 0 | 2 | 0 | 1 |
| 3 | 0 | 3 | 1 | 1 |
| 4–7 | 1 | same as 0–3 | | |

- `col - 0.5` is -0.5 or +0.5, so x = centre ± half a box width: two boxes side by side, edge to edge, around the centre. Same for y with the box length.
- `z_tip = top + (layer + 1) * BOX_H`: the cups go to the top of a box on layer 0 or layer 1.

## The run method + default

```python
    async def run(self):
        """Pack the whole pallet: two layers of four."""
        await helpers.clear_boxes(self.robot)
        self.placed = []
        for _ in range(8):
            await self.place()
        print(f"packed {len(self.placed)} boxes")
```

`STEPS`: add `"run": Palletizer.run,`

Bottom of the file:

```python
if __name__ == "__main__":
    verb = sys.argv[1] if len(sys.argv) > 1 else "run"
    asyncio.run(main(verb))
```

- `clear_boxes` removes old box visuals from the scene; `self.placed = []` starts the count at 0.
- The loop runs `place` 8 times inside one program, so `self.placed` grows 0 → 8 (this fixes the "both boxes in the same spot" issue from exercise 14).
- No word after the file name → run `run`.

## Run it

```bash
python palletizer.py
```

## Open point from the course

The course says: watch the arm closely. Something in this pack is not physically possible. That is the next exercise.
