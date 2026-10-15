# How to add and place a component (Blocks → Frame → 3D scene)

From Viam 101, 2026-10-14. Same flow for every component: arm, gripper, camera.

> **Note:** the steps are general. All names and numbers in the examples (`arm-1`, `gripper-1`, EPick, 196 mm) come from the course's simulated palletizer cell, not from my robot.

## Key idea: model

A **model** is one implementation of a component type.
- `arm/simulated` is a model of arm (built into viam-server).
- `robotiq-epick/simulated-epick-vacuum-gripper` is a model of gripper (comes from the registry).

Every model of the same type exposes the same API (for example the gripper API). The model decides what sits behind that API: real hardware, or a simulation. So code written against the simulated part also runs against the real one.

## Steps

1. In the Viam app, open the machine → **CONFIGURE**. Click **+**, then **Blocks**.
2. Search for the model (for example `simulated epick`) and select it.
3. Click **Add to machine**. A dialog opens with a name filled in (for example `gripper-1`). Keep it or rename it, then click **Add to machine** again.
   - If the model comes from the registry, the dialog shows a supporting module. viam-server downloads and installs it by itself.
4. In **Attributes**, replace the empty `{}` with the settings the model needs. Example for the vacuum gripper:
   ```json
   {
      "grab_delay_ms": 250
   }
   ```
5. Scroll to the **Frame** section and click **Set defaults**. This puts the part in the 3D scene.
   - With defaults, the part sits at the world origin (0, 0, 0), not attached to anything. To attach it to a parent (for example gripper → arm), see **Place a component** below.
6. Click **Save**. viam-server picks up the new config.

## Place a component (set its frame)

After the part is added (steps 1–6 above), you tell Viam where it sits.

1. **CONFIGURE** → select the component → scroll to its **Frame** section.
2. Click **JSON** on the section's header. It shows the same frame as the form, in one short block you can read at once.
3. Read or edit the three fields:
   - `parent`: the frame this part is measured from. `world` = fixed to the cell. A component name (for example `arm-1`) = mounted on that component; for an arm, that means the end of its last link (the flange).
   - `translation`: the offset from the parent in mm, measured in the **parent's** axes.
   - `orientation`: how the part is turned. `ov_degrees` with `x:0, y:0, z:1, th:0` = no turn.
4. Click **Save**, then check the **3D SCENE** tab.

### Example: arm at the world origin (default, nothing to change)

```json
{
  "orientation": { "type": "ov_degrees", "value": { "th": 0, "x": 0, "y": 0, "z": 1 } },
  "parent": "world",
  "translation": { "x": 0, "y": 0, "z": 0 }
}
```

### Example: gripper mounted on the arm

```json
{
  "orientation": { "type": "ov_degrees", "value": { "th": 0, "x": 0, "y": 0, "z": 1 } },
  "parent": "arm-1",
  "translation": { "x": 0, "y": 0, "z": 196 }
}
```

Two changes from the default: `parent` is `arm-1`, and the gripper is moved 196 mm along z.

### Where an end-effector offset comes from

The 196 mm moves the gripper's frame from the arm's flange to the face of the vacuum cups, the point that actually touches a box. On real hardware, take it from the maker's datasheet (flange-to-tool-tip distance, often called the TCP, tool center point: the point on the tool that does the work) or measure it: flange face to the point that touches the work. The exact number matters once the motion service drives the gripper.

## Check

- Open the **3D SCENE** tab: the new part should show up, and the **World** panel lists it.
- Open **LOGS** if it doesn't: a missing module or a bad attribute shows up there.

## Added so far (course machine `palletizer-101`)

| Name | Type | Model | Attributes | Frame |
| --- | --- | --- | --- | --- |
| `arm-1` | arm | `arm/simulated` | (as in the course) | parent `world`, translation 0 |
| `gripper-1` | gripper | `robotiq-epick/simulated-epick-vacuum-gripper` | `{"grab_delay_ms": 250}` | parent `arm-1`, z = 196 mm |
