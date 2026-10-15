# Viam 102 — Exercise 5: DoCommand and hot reload

**Date:** 2026-10-15
**Course:** learn.viam.com, Viam 102 (exercise 5)

## DoCommand: the verb dispatch

- The caller's JSON arrives in `do_command` as `command`, already a Python dict. The dict I return goes back as JSON.
- I read `verb = command.get("command")` and route each verb to a method:
  ```python
  # pick: the starter's complete pick() method
  if verb == "pick":
      return {"picked": await self.pick()}
  ```
- An unrouted verb returns `{"error": "unknown command: ..."}` and logs a warning. The arm doesn't move and nothing crashes.
- `status` reports what the constructor actually holds. It's evidence, not a hard-coded list. Worth adding to every module I build.
- Verbs I used: `status` (report) and `pick` (move the arm, take a box, lift).
- Test from the app: CONFIGURE → `palletizer-1` card → DoCommand → Input / Execute / Output.

## Hot reload vs release

| | Hot reload | Release |
| --- | --- | --- |
| Command | `viam module reload-local --part-id "$VIAM_102_PART_ID"` | `viam module upload --version X ...` |
| Sends | whatever my working copy holds right now | a fixed version in the Registry |
| Goes to | one machine (by part id) | any machine that asks for it |
| Use for | development: edit, reload, test | when it's ready to share or deploy |

- **Part id** = the unique id of the machine's main part (status dropdown at the top of the machine page).
- `reload-local` builds on **this** computer, so it only fits a machine with the same architecture.
- For a machine with a different architecture, `viam module reload` runs the build in Viam's cloud.
- If the machine has no shell service, the reload adds one (it copies the build through it).
- Modules view after a reload shows a flame, a "Hot Reloading" toggle, and the reloaded version.

## For my robot (my thinking)

- This is the daily loop for the cart: edit in VS Code on the laptop, reload, test on the robot.
- Laptop (x86) → Pi (arm64): `reload-local` won't fit; use `viam module reload` (cloud build). Lab computer (Ubuntu, x86): `reload-local` should work. Not tested yet.

## Parked (not blocking)

- 3D scene sim is slow / choppy.
- The 3D scene warning from 101 ("could not load the current transforms").
