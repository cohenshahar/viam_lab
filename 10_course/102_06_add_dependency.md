# Viam 102 — Exercise 6: Add a dependency

**Date:** 2026-10-15
**Course:** learn.viam.com, Viam 102 (exercise 6)

## Declare, resolve, use

| Step | Where | Code |
| --- | --- | --- |
| Declare | `validate_config` | `req_deps = [GRIPPER, PICK_STATION, PALLET, PACK_SEQUENCER, MOTION]` |
| Resolve | `new` | `self.pallet = dependencies[GenericComponent.get_resource_name(PALLET)]` |
| Use | `status` → `held` | `PALLET: self.pallet,` |

Then `viam module reload-local --part-id "$VIAM_102_PART_ID"` and `{"command": "status"}` → five resources.

## Takeaways

- `viam-server` calls `validate_config` first. It creates the service only after every **required** dependency is up and healthy.
- The names (`PALLET = "pallet"`) must match the resource names in the machine config exactly.
- Use the **right type** for the lookup: the pallet is a generic *component* (`GenericComponent`), not the generic *service* (`Generic`) the class inherits from. Wrong type means the wrong key.
- Required vs optional: required if the logic can't work without it (fail at startup, not mid-pick). Optional (`dependencies.get(...)`, may be `None`) if the code can run without it, like the cell sensors.
- `status` only reports what I put in `held`. **New dependency means updating `status` too.**
- This closes the Wed open question: `pallet_top()` used `self.pallet`, which was never set.

## For my robot (my thinking)

- The KR6 logic service would declare `arm`, `base`, `motion` (and a camera) the same way, so after a reboot it starts only when all of them are ready.
