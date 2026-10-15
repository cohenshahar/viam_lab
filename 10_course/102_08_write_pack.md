# Viam 102 — Exercise 8: Write pack

**Date:** 2026-10-15
**Course:** learn.viam.com, Viam 102 (exercise 8)

## The code

```python
    async def pack(self):
        """Pack the whole pallet, answer with the count."""
        await self.clear_boxes()   # wipe the drawn boxes from the 3D scene
        self.placed = []           # wipe the record (slots + obstacles come from it)
        for _ in range(self.columns * self.rows * self.layers):
            await self.place()     # each call takes the next slot by itself
        return len(self.placed)
```

Route (above the catch-all): `if verb == "pack": return {"placed": await self.pack()}` → caller reads `{"placed": 8}`.

## Takeaways

- **A verb built from verbs:** `pack` = clear + `place` × 8, and `place` = `pick` + carry + descend + release.
- No slot number is passed. `place()` reads its slot from `len(self.placed)`, which grows by one each time.
- **Clear first, so a run can repeat.** Skip `self.placed = []` and the planner keeps avoiding boxes that are gone, and the slots start in the wrong place.
- `clear_boxes()` is `async`, so it needs `await`, or it never runs.
- **The big moment of the course:** the pack runs *inside* the machine, with no laptop script attached. That's the module idea from page 1.
- After a pack, `status` shows `boxes_packed` moved.
- A long DoCommand can finish without printing output. **LOGS** is the full record of what the module did.

## For my robot (my thinking)

- Same layering for the KR6 + cart: small verbs (`drive_to`, `pick`, `place`) composed into one job verb that runs on the robot computer.
