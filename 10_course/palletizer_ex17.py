# Viam 101 course: my palletizer.py at the end of exercise 17 (avoid placed boxes).
# Course simulation only (palletizer cell), not my robot. Runs in the course workspace
# next to the course's helpers.py.
import asyncio
import sys

from helpers import connect
from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions
import helpers
from viam.services.motion import MotionClient
from viam.proto.common import (
    Pose,
    PoseInFrame,
    WorldState,
    GeometriesInFrame,
    Geometry,
    RectangularPrism,
    Vector3,
)
from viam.components.gripper import Gripper


BOX_W, BOX_L, BOX_H = 200.0, 150.0, 100.0  # the box the pick-station presents
GRASP_DEPTH = 10.0  # press the cups this far onto the box top so they seat


def down_pose(x, y, z) -> Pose:
    """A target pose at (x, y, z) with the tool pointing straight down."""
    return Pose(x=x, y=y, z=z, o_x=0, o_y=0, o_z=-1, theta=0)


class Palletizer:
    def __init__(self, robot):
        self.robot = robot
        self.gripper = Gripper.from_robot(robot, helpers.GRIPPER)
        self.placed = []
        self._top = None

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

    async def move(self):
        """Send the gripper to a reachable pose, pointing down."""
        await self.move_gripper(down_pose(400, -300, 400))

    async def pick(self, seq=0):
        """Pick the box at the pick-station and lift it (box ends up held)."""
        home = await helpers.pick_home_pose(self.robot, BOX_H)
        grasp = await helpers.grasp_pose(self.robot, BOX_H)
        await self.move_gripper(home, self.obstacles())
        await helpers.show_box(self.robot, seq, grasp.x, grasp.y, grasp.z - BOX_H / 2)
        await self.move_gripper(down_pose(grasp.x, grasp.y, grasp.z - GRASP_DEPTH))
        await self.gripper.grab()
        await helpers.attach_box(self.robot, seq, BOX_H)
        await self.move_gripper(home, self.obstacles())

    async def _pallet_top(self):
        if self._top is None:
            self._top = await helpers.pallet_top(self.robot)
        return self._top

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

    def _place_pose(self, i, cx, cy, top):
        """Box i -> (x, y, gripper-tip z): two layers of a 2x2 grid."""
        layer, slot = i // 4, i % 4
        col, row = slot % 2, slot // 2
        x = cx + (col - 0.5) * BOX_W
        y = cy + (row - 0.5) * BOX_L
        z_tip = top + (layer + 1) * BOX_H
        return x, y, z_tip

    async def place(self):
        """One full pick-and-place of the next box onto the pallet."""
        cx, cy, top = await self._pallet_top()
        seq = len(self.placed)
        x, y, z_tip = self._place_pose(seq, cx, cy, top)
        await self.pick(seq)
        await self.move_gripper(down_pose(x, y, z_tip), self.obstacles())
        await self.gripper.open()
        await helpers.show_box(self.robot, seq, x, y, z_tip - BOX_H / 2)
        self.placed.append((x, y, z_tip - BOX_H / 2))

    async def run(self):
        """Pack the whole pallet: two layers of four."""
        await helpers.clear_boxes(self.robot)
        self.placed = []
        for _ in range(8):
            await self.place()
        print(f"packed {len(self.placed)} boxes")


# verb -> method. One entry per capability.
STEPS = {
    "resources": Palletizer.resources,
    "wave": Palletizer.wave,
    "zero": Palletizer.zero,
    "move": Palletizer.move,
    "pick": Palletizer.pick,
    "place": Palletizer.place,
    "run": Palletizer.run,
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
    verb = sys.argv[1] if len(sys.argv) > 1 else "run"
    asyncio.run(main(verb))
