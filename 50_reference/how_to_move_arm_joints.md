# How to move the arm from Python (joint positions + step program)

From Viam 101 (exercise 9), 2026-10-14.

> **Note:** the pattern is general. The names (`arm-1`, `Palletizer`, `palletizer.py`) and the joint numbers come from the course's simulated arm, not from my robot.

## Two ideas

1. **Joint positions:** you give the arm one angle per joint, in **degrees**, from base to wrist (six numbers for a 6-joint arm). The arm goes straight to those angles.
2. **One program, many steps:** a dictionary `STEPS` maps a word to a method, so `python palletizer.py wave` runs just that one step. Each new skill = one new method + one new line in `STEPS`. The course says the same map is how a Viam module exposes its commands, so this program is the start of a module.

## The code (course version)

```python
from helpers import connect
from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions


class Palletizer:
    def __init__(self, robot):
        self.robot = robot

    async def resources(self):
        """List the resources the machine exposes (a first, safe call)."""
        for name in sorted(rn.name for rn in self.robot.resource_names):
            print(" ", name)

    async def wave(self):
        """Move the arm by commanding its joint angles directly."""
        arm = Arm.from_robot(self.robot, "arm-1")
        await arm.move_to_joint_positions(
            JointPositions(values=[0, -45, -30, 0, 60, 0])
        )

    async def zero(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        arm = Arm.from_robot(self.robot, "arm-1")
        await arm.move_to_joint_positions(
            JointPositions(values=[0, 0, 0, 0, 0, 0])
        )


import asyncio
import sys

# verb -> method. One entry per capability.
STEPS = {
    "resources": Palletizer.resources,
    "wave": Palletizer.wave,
    "zero": Palletizer.zero,
}

async def main(verb):
    robot = await connect()
    palletizer = Palletizer(robot)
    try:
        step = STEPS.get(verb)
        if step is None:
            print(f"unknown step '{verb}'. steps: {', '.join(STEPS)}")
            return
        await step(palletizer)
    finally:
        await robot.close()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"usage: python palletizer.py <step>   steps: {', '.join(STEPS)}")
        sys.exit(1)
    asyncio.run(main(sys.argv[1]))
```

## Block by block

- **Imports:** `connect` (from the course's `helpers.py`) opens the connection. `Arm` is the arm API. `JointPositions` is the message that carries the joint angles.
- **`class Palletizer`:** holds the open connection (`self.robot`) so every step can use it.
- **`resources`:** a safe first call. Moves nothing, just prints what the machine has (arm, gripper, services). Good for checking names before you move anything.
- **`wave` / `zero`:** `Arm.from_robot(self.robot, "arm-1")` finds the arm **by its config name**. `move_to_joint_positions(...)` sends the six angles. `await` = wait until the call finishes.
- **`STEPS`:** the word → method map. Add a skill = add a method + one line here.
- **`main`:** connect → find the step → run it → **always** close the connection (`finally`), even if the step fails.
- **Bottom block:** reads the step word from the command line and runs `main`. It sits at the left margin, **outside** the class.

## Run it

```bash
python palletizer.py resources   # list what the machine has
python palletizer.py wave        # move to [0, -45, -30, 0, 60, 0]
python palletizer.py zero        # back to all zeros
```

Keep the **3D SCENE** tab open in another window to watch it move.

## Add your own position

```python
    async def my_pose(self):
        """Describe the pose here."""
        arm = Arm.from_robot(self.robot, "arm-1")
        await arm.move_to_joint_positions(
            JointPositions(values=[j1, j2, j3, j4, j5, j6])   # degrees, base to wrist
        )
```

and add `"my_pose": Palletizer.my_pose,` to `STEPS`.

## Limits of this method (from the course)

- You must work out the six angles yourself for every place you want to reach.
- Nothing checks for obstacles: the arm goes straight to the angles.
- The course says the rest of Viam 101 is about letting the **motion service** do this for you.
