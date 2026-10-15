# Viam 101 — Recap (step 19) — course finished 2026-10-14

## What I built

A palletizing application in simulation, from an empty machine to a clean two-layer pack, with one command: `python palletizer.py`.
Final code: `10_course/palletizer_final.py`.

## What I configured (in the app)

A simulated arm, a vacuum gripper attached at the right offset, a floor for the planner to avoid, a pallet and a pick-station (from a fragment), and two scene services that draw the cell and the boxes.

## What I wrote

One program, `palletizer.py`, one method and one verb at a time:
- `move`: drive the gripper with the motion service
- `pick`: ask the pick-station where the box is (DoCommand), grab it, lift it
- `place`: set a box down, one full cycle
- `run`: pack the whole pallet, made collision-safe by passing the placed boxes and the held box to the planner as obstacles

Every step runs on its own (`python palletizer.py pick`) and they assemble into `run`.

## The ideas (from the course)

- **Components and services:** hardware, and the capabilities that run on top of it
- **The registry:** where the cell's parts come from
- **The frame system:** where everything is, relative to `world`
- **The motion service:** say where the gripper goes, not how
- **DoCommand:** ask a component for what the standard APIs don't cover
- **The two world states:** the **WorldStateStore** service only *draws* things (the boxes in the 3D scene); the **planner** only sees the `WorldState` you pass to each `Move` call. Drawing a box does not make the planner avoid it.
- **The connect-write-run loop:** edit, run one command, watch, iterate

## What comes next (from the course)

`palletizer.py` is already shaped like a Viam module: the `Palletizer` class is the module's logic, and `STEPS` (word → method) is how a module exposes commands through DoCommand. The next tutorial wraps it in a module and deploys it from a GitHub repo, so any machine in a fleet can run it with no laptop attached.

## Recipes from this course (50_reference/)

| Step | File |
| --- | --- |
| Restart a session | how_to_restart_course_session.md |
| Add + place a component | how_to_add_component.md |
| Connect Python | how_to_connect_python.md |
| Move by joints, step program | how_to_move_arm_joints.md |
| Obstacle by hand + fragment | how_to_add_fragment.md |
| Move with the motion service | how_to_move_with_motion.md |
| Pick | how_to_pick.md |
| Place | how_to_place.md |
| Full job loop | how_to_run_loop.md |
| Avoid placed boxes (WorldState) | how_to_avoid_obstacles.md |
| Model the held box | how_to_model_held_object.md |
