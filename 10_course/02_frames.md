# Viam 101 — Module 2: Frames and geometry

**Date:** 2026-10-14
**Course:** learn.viam.com, Viam 101

## Takeaways (from the course)

- A frame is a coordinate system: an origin and three right-handed axes. The world frame is the fixed root everything else is measured against; in this cell it sits at the base of the arm.
- Every other frame is defined relative to a parent: you name a parent frame and give a translation and an orientation, both measured in the parent's axes. Viam composes the links into a tree rooted at `world`.
- Offsets ride their parents. The gripper's 196 mm offset is measured in the flange's axes, so when the arm moves or turns its wrist, the gripper frame follows with nothing recomputed.
- A geometry gives a frame volume: a box, sphere, or capsule the motion planner routes around. A frame says where; a geometry says how big.
- The 3D scene renders each component from the same frame data the planner uses, so a placement that looks right in the scene is what the planner will plan against.

## Example from the course cell

- `world` → at the base of the arm.
- arm → flange (end of the arm) → `gripper-1`, offset 196 mm in the flange's axes.

## Exercise 7: Place the arm and gripper

- Arm: kept the default frame (parent `world`, translation 0).
- Gripper: parent `arm-1`, translation z = 196 mm (flange to vacuum cup face).
- Checkpoint passed: gripper renders on the arm's wrist, World panel lists both.
- Step-by-step recipe: `50_reference/how_to_add_component.md`, section "Place a component".
