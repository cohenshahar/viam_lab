# How to move to a pose with the motion service

From Viam 101 (exercise 12), 2026-10-14. Builds on `how_to_move_arm_joints.md` (the step program).

> **Note:** the pattern is general. Names (`arm-1`, `helpers.GRIPPER`, `helpers.MOTION`), the pose (400, -300, 400) and the 196 mm all come from the course's simulated cell, not from my robot.

## Key idea

With joint positions you work out six angles yourself. With the **motion service** you only say:
1. **which frame** to move (`component_name`), and
2. **where** it should end up (`destination`, a pose in a named frame).

The service plans a collision-free path and drives the arm there. No joint angles, no trajectory from you.

**Which frame you name matters:**
- Name the **gripper** → the gripper's frame (the vacuum cup face, set by its offset) lands on the pose.
- Name the **arm** → the arm's **flange** lands on the pose, so the cups end up one offset further on (in the course: 196 mm below).

## Pose

A pose = a point (x, y, z in mm) + an orientation (`o_x, o_y, o_z` = the direction the tool's z-axis points, `theta` = spin around that axis, in degrees).

`o_z = -1` → the tool points straight down at the floor.

## The new code pieces

Imports (top of the file):

```python
import helpers
from viam.services.motion import MotionClient
from viam.proto.common import Pose, PoseInFrame
```

Pose builder (above the class):

```python
def down_pose(x, y, z) -> Pose:
    """A target pose at (x, y, z) with the tool pointing straight down."""
    return Pose(x=x, y=y, z=z, o_x=0, o_y=0, o_z=-1, theta=0)
```

The motion call (inside the class):

```python
    async def move_gripper(self, pose, obstacles=None):
        """Move the gripper frame to `pose`, routing around `obstacles`."""
        motion = MotionClient.from_robot(self.robot, helpers.MOTION)
        destination = PoseInFrame(reference_frame="world", pose=pose)
        return await helpers.retry("move", lambda: motion.move(
            component_name=helpers.GRIPPER,
            destination=destination,
            world_state=obstacles,
            extra={"timeout": 15.0},
            timeout=120,
        ))
```

A step that uses it (inside the class) + its line in `STEPS`:

```python
    async def move(self):
        """Send the gripper to a reachable pose, pointing down."""
        await self.move_gripper(down_pose(400, -300, 400))
```

```python
    "move": Palletizer.move,
```

## Block by block

- **`MotionClient.from_robot(self.robot, helpers.MOTION)`:** gets the motion service by name (in the course the name is kept in `helpers.MOTION`; the built-in one is called `builtin`).
- **`PoseInFrame(reference_frame="world", pose=pose)`:** the pose plus the frame it is measured in. Here: the world frame.
- **`component_name=helpers.GRIPPER`:** the frame to move. `helpers.GRIPPER` is a constant that holds the name, so no quotes. Writing `"arm-1"` instead is a plain string with quotes.
- **`world_state=obstacles`:** extra obstacles for this move. Empty for now (`None`).
- **`extra={"timeout": 15.0}` / `timeout=120`:** stop a plan that can't be found, so the program doesn't hang.
- **`helpers.retry(...)`:** a course helper that tries again if the cloud connection blips.

## Run it

```bash
python palletizer.py move
```

The gripper goes to (400, -300, 400), pointing down, in the 3D scene.

Try it: change `component_name` to `"arm-1"` and run again. The flange goes to the pose and the cups end up one offset lower. Change it back afterwards.

## Joint positions vs motion service

| | Joint positions | Motion service |
| --- | --- | --- |
| You give | six angles | a frame + a pose |
| Who works out the joints | you | the service |
| Avoids obstacles | no | yes (the ones in the config + `world_state`) |
