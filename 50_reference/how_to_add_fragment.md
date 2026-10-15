# How to add an obstacle (geometry) and a fragment

From Viam 101 (exercise 10), 2026-10-14.

> **Note:** the steps are general. The names (`floor`, `viam101-workcell`, pallet, pick-station, workcell-scene, pack-sequencer) and all numbers come from the course's simulated cell, not from my robot.

## Part 1: Add an obstacle by hand (the floor)

The motion planner only avoids what it knows about. A floor (or table, wall, cart top) must be in the config as a **frame + geometry**.

1. **+** → **Blocks** → search `fake generic`.
2. Two results are called `generic/fake`: one **component**, one **service**. Pick the one described as a **fake generic component**. The dialog title **Add component** confirms it.
3. Name it (course: `floor`) → **Add to machine**. No attributes to set.
4. Click the **Frame** tile → **JSON** → replace with:

```json
{
   "parent": "world",
   "translation": { "x": 0, "y": 0, "z": -5 },
   "geometry": { "type": "box", "x": 2000, "y": 2000, "z": 10 }
}
```

5. **Save**.

Why these numbers: a box 2000 x 2000 x 10 mm, centred 5 mm below z = 0, so its top face sits exactly at z = 0 (the ground).

**Frame vs geometry:** the frame says *where* a thing is. The geometry says *what space it fills*, so the planner routes around it. The arm doesn't need a geometry here: it brings its own from its kinematic model. Things like a floor have no volume until you give them one.

## Part 2: Add a fragment

A **fragment** is a reusable bundle of config blocks (components, services, and the module versions they need). Adding it writes all of them into the machine config in one step. It's how a team sets up ten identical machines without copying config by hand.

1. **+** → **Blocks** → search the fragment's name (course: `viam101-workcell`).
2. Pick the result tagged **FRAGMENT**.
3. **Add to machine**. No name to type, nothing to configure.
4. **Save**. viam-server downloads the modules the fragment needs.

What the course fragment delivered:
- `pallet`, `pick-station`: components placed in the world frame, sizes already set.
- `workcell-scene`: a service that draws the pallet and pick-station in the 3D scene.
- `pack-sequencer`: a service that tracks each box and draws it where the code says.
- The module versions those need, pinned to tested builds.

In CONFIGURE, parts that came from a fragment show where they came from (for example "viam101-workcell"), and the left rail shows **Fragments** and **Modules** counts.

## Check

- CONFIGURE: every new part shows **READY**.
- 3D SCENE: the cell renders (turn colliders on to see the floor's volume).
- `python palletizer.py resources` lists everything, plus `builtin` (the motion service every machine has).

## Known warning: "World state store: could not load the current transforms"

Seen on 2026-10-14 right after adding the fragment. A **world state store** is a service that publishes extra visuals (transforms) for the 3D scene ([docs](https://docs.viam.com/reference/apis/services/world-state-store/)). In this cell, the scene services from the fragment play that role. The pallet group still loaded, so the scene works. Most likely cause (not verified): one of the scene services was still starting, or has nothing to show yet (there are no boxes until the pick exercise). If it stays after a page refresh, check **LOGS** for errors from `workcell-scene` / `pack-sequencer`.
