# How to pick an object (DoCommand + gripper API)

From Viam 101 (exercise 13), 2026-10-14. Builds on `how_to_move_with_motion.md`.

> **Note:** the pattern is general. Names (`pick-station`, `helpers.grasp_pose`, `show_box`), box size and GRASP_DEPTH come from the course's simulated cell, not from my robot.

## Key ideas

1. **DoCommand:** a component can offer extra commands beyond its standard API. The course's pick-station is a *generic* component; the helpers `grasp_pose` and `pick_home_pose` ask it, through DoCommand, where to grab and where to wait above the box.
2. **Poses come from config, not hidden code:** the pick-station's Frame puts it in the world; its Attributes say where the box sits, its angle, and how high to hover. Change those values (or the frame) and the same component describes a different station or a different cell. Same for the pallet.
3. **Gripper API:** `gripper.grab()` engages the gripper (here: suction). Every gripper model has the same API.
4. **Pick sequence:** safe pose above → go down onto the object → grab → back up to the safe pose.
5. **Box visuals are cosmetic:** the simulated gripper holds nothing, so `show_box` / `attach_box` tell the pack-sequencer service where to draw the box. It changes what you see, not the motion.

## The new code pieces

Near the top:

```python
from viam.components.gripper import Gripper

BOX_W, BOX_L, BOX_H = 200.0, 150.0, 100.0  # the box the pick-station presents
GRASP_DEPTH = 10.0  # press the cups this far onto the box top so they seat
```

New `__init__` (gripper handle + list of placed boxes):

```python
    def __init__(self, robot):
        self.robot = robot
        self.gripper = Gripper.from_robot(robot, helpers.GRIPPER)
        self.placed = []
        self._top = None
```

The pick method:

```python
    async def pick(self, seq=0):
        """Pick the box at the pick-station and lift it (box ends up held)."""
        home = await helpers.pick_home_pose(self.robot, BOX_H)
        grasp = await helpers.grasp_pose(self.robot, BOX_H)
        await self.move_gripper(home)
        await helpers.show_box(self.robot, seq, grasp.x, grasp.y, grasp.z - BOX_H / 2)
        await self.move_gripper(down_pose(grasp.x, grasp.y, grasp.z - GRASP_DEPTH))
        await self.gripper.grab()
        await helpers.attach_box(self.robot, seq, BOX_H)
        await self.move_gripper(home)
```

And in `STEPS`: `"pick": Palletizer.pick,`

## Block by block

- **`Gripper.from_robot(robot, helpers.GRIPPER)`:** get the gripper by its config name once, in `__init__`, so every step can use `self.gripper`.
- **`self.placed`, `self._top`:** empty for now; used later to remember what was put on the pallet.
- **`pick_home_pose` / `grasp_pose`:** ask the pick-station (DoCommand) for the hover pose and the grab pose, given the box height.
- **`move_gripper(home)`:** go to the safe pose above the box first.
- **`show_box(...)`:** draw box number `seq` resting on the station (its centre is half a box height below the top face).
- **`down_pose(..., grasp.z - GRASP_DEPTH)`:** go down onto the box, 10 mm "into" the top so the cups seat.
- **`gripper.grab()`:** suction on.
- **`attach_box(...)`:** from now on, draw the box riding the gripper.
- **`move_gripper(home)`:** lift back up.

## Run it

```bash
python palletizer.py pick
```

A box appears on the pick-station, the gripper goes down, grabs, and lifts it.

## For my own robot later (not from the course)

The "ask the station where the object is" pattern is the place to plug in a camera or a fixed known position. The sequence (above → down → grab → up) stays the same.
