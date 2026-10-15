# Viam 102 — Exercise 7: Write place

**Date:** 2026-10-15
**Course:** learn.viam.com, Viam 102 (exercise 7)

## What `place` does (one box, one slot)

1. Get the pallet's top (`pallet_top()`, cached) and this box's slot: `seq = len(self.placed)` → `place_pose(seq, ...)` → `x, y, z_tip`.
2. `await self.pick(seq)` → the box ends up held.
3. Two poses, both facing straight down, different heights:
   - `safe_pose_facing_down_above_stack` at `clear_tip(z_tip)`, high enough to ride over the stack
   - `pose_at_slot_facing_down` at `z_tip`
4. The three lines I wrote:
   ```python
   await self.move_gripper(safe_pose_facing_down_above_stack, self.obstacles(held=True))  # TODO 1: carry
   await self.move_gripper(pose_at_slot_facing_down)                                      # TODO 2: descend, NO obstacles
   await self.gripper.open()                                                              # TODO 3: release vacuum
   ```
5. Record and draw the box at its **center** (`z_tip - box_h / 2`), not its top, or every later obstacle is half a box too tall.
6. Route it: `if verb == "place": return {"slot": await self.place()}`

Each call puts one box in the next slot. Four calls fill the first layer.

## Takeaways

- **Approach high, then descend straight down.** The same pattern as pick.
- **Carry with `held=True`** so the planner routes the box over the stack, not through it.
- **Descend with no obstacles.** If the held box is passed as an obstacle there, the planner refuses with "fatal early collision": it's avoiding the very box I'm placing.
- **Placed boxes become obstacles** for the next move (`self.placed` feeds `obstacles()`).
- The gripper is a simulated vacuum gripper (Robotiq EPick): `grab()` = suck, `open()` = release. Course values only, not my robot.

## My mistake (and the rule)

- I pasted the `place` route **below** the catch-all `return {"error": ...}` in `do_command`. Python leaves at the first `return`, so `place` was never reached → `unknown command: place`.
- **Rule: the catch-all always goes last. Every new verb goes above it.**
- After any edit: reload (`viam module reload-local ...`). The machine runs the last build, not my file.
