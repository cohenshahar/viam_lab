# Viam 102 — Pages 9–12 (to do Fri 16 Oct) + reference for my robot

**Status:** saved from the course pages on 2026-10-15, **not done yet**. Do them Fri at home.

## 9: Attributes and validation

Goal: move the pack pattern out of the code and into the machine config.

Attributes (named for the person reading the config, with the unit in the name):
```json
{ "columns": 2, "rows": 2, "layers": 2, "box_height_mm": 100 }
```

In `validate_config`, under "# Validate new attributes here":
```python
        # JSON numbers arrive as floats: 2 becomes 2.0, so check wholeness.
        attrs = struct_to_dict(config.attributes)
        columns = attrs.get("columns")
        if (columns is None or columns < 1
                or columns != int(columns)):
            raise ValueError("columns must be a positive whole number")
        rows = attrs.get("rows")
        if (rows is None or rows < 1 or rows != int(rows)):
            raise ValueError("rows must be a positive whole number")
        layers = attrs.get("layers")
        if (layers is None or layers < 1 or layers != int(layers)):
            raise ValueError("layers must be a positive whole number")
        box_h = attrs.get("box_height_mm")
        if box_h is None or box_h <= 0:
            raise ValueError("box_height_mm must be greater than zero")
```

In `new`, under "# Read validated attributes here":
```python
        attrs = struct_to_dict(config.attributes)
        self.columns = int(attrs["columns"])
        self.rows = int(attrs["rows"])
        self.layers = int(attrs["layers"])
        self.box_h = attrs["box_height_mm"]
```
Then delete the class-level defaults (`columns, rows, layers = 2, 2, 2` and `box_h = BOX_H`).

Steps: `course-tools check-exercise 9` → reload → LOGS shows "columns must be a positive whole number" (no attributes yet) → add the JSON on the `palletizer-1` card → Save → READY → `pack`.
Break it on purpose: `columns: 0` → FAILED with my message → back to 2 → READY in 10–15 s, **no reload**. `layers: 1` → pack 4 boxes, no reload. Set back to 2.

Key ideas:
- **Validate judges, the constructor stores.**
- `viam-server` runs `validate_config` on **every config change**.
- Required attribute: check presence and raise. Optional: `attrs.get("layers", 2)` with a default.
- Changing an attribute is just a config edit. No reload, no release.

## 10: GitHub and cloud build

Goal: put the module in a repo; cloud build publishes 0.2.0 for 3 platforms.

```bash
course-tools setup-github && source ~/.viam-102-env
git init -b main && git add . && git commit -m "Palletizer service"
gh repo create "${VIAM_102_REPO:?run course-tools setup-github first}" --public --source=. --remote=origin --push
```
**Decide before running:** the page makes the repo `--public`. Fine for a course repo; my lab/thesis modules stay private.

`meta.json`: set `"url"` to the repo address. The `build` section:
```json
"build": {
  "build": "./build.sh",
  "setup": "./setup.sh",
  "path": "dist/archive.tar.gz",
  "arch": ["linux/amd64", "linux/arm64", "darwin/arm64"]
}
```
```bash
viam module update
git add meta.json && git commit -m "Point meta.json at the repository" && git push
```
Document the model: replace the body of `shahartutorial_palletizer-102_palletizer.md` (below the title) with the course text: description, attribute template, attributes table, example config, DoCommand verb table (status/clear/pick/place/pack and their answers), example DoCommand. Then:
```bash
git add . && git commit -m "Document the palletizer model" && git push
viam module build start --version 0.2.0 | tee ~/.viam-102-build.txt
viam module build logs --wait --build-id "$(grep -oE '[0-9a-f-]{36}' ~/.viam-102-build.txt | head -1)"
```
Then on CONFIGURE → Modules → my module (flame icon) → **Hot Reloading toggle off** → Save. The machine goes back to the registry build and picks up the newest version by itself.

Key ideas:
- A compiled build runs only on a matching OS + CPU. Cloud build makes one per entry in `arch`.
- **Hot reload is sticky on purpose:** the machine stays on my working copy until I switch it off.
- Three ways to deploy:
  | Way | Use when |
  | --- | --- |
  | local build + upload | first release, or the target always matches my computer |
  | reload | code still moving, one machine |
  | cloud build | full release, every platform |
- The model `.md` file is what people read in the Registry. Write it for someone who never sees the code.

## 11: Pack on its own (`autopack`)

```python
    async def autopack(self, max_boxes_to_pack=None):
        """Place while there is room, swap a full tray, stop at max_boxes_to_pack."""
        boxes_per_pallet = self.columns * self.rows * self.layers
        if max_boxes_to_pack is None:
            max_boxes_to_pack = boxes_per_pallet
        placed = 0
        while placed < max_boxes_to_pack:
            if not await self.pallet_has_room():
                await self.swap_tray()
                continue
            await self.place()
            placed += 1
        result = {"Number Packed": placed, "Max Boxes": max_boxes_to_pack}
        self.logger.info(f"autopack: {result}")
        return result
```
Route (above the catch-all):
```python
        if verb == "autopack":
            max_boxes_to_pack = command.get("max_boxes_to_pack")
            if max_boxes_to_pack is not None:
                try:
                    max_boxes_to_pack = int(max_boxes_to_pack)
                except (TypeError, ValueError):
                    return {"error": "max_boxes_to_pack must be a number"}
            return await self.autopack(max_boxes_to_pack)
```
```bash
course-tools check-exercise 11
git add . && git commit -m "Add autopack" && git push
viam module build start --version 0.3.0
```
Test: `{"command": "clear"}`, then `{"command": "autopack", "max_boxes_to_pack": 12}` → 8 boxes, tray swap, 4 more, stop.

Key ideas:
- **Bounded loop:** it always has a stop (`max_boxes_to_pack`). Never an endless loop on a robot.
- Ask, then act: room? → place. Full? → swap tray (`continue`, no box counted).
- Bad input from a caller gets a clear error, not a crash.
- The call blocks until the run ends; LOGS keeps the final line if the panel gives up waiting.

## 12: Recap: the CLI commands for any module

1. `viam login`
2. `viam module generate` (scaffold + register)
3. `viam module update` (after every `meta.json` change)
4. `viam module build local` → `dist/archive.tar.gz`
5. `viam module upload --version X --platform <os/arch> --upload dist/archive.tar.gz` (versions are permanent)
6. `viam module reload-local --part-id "<part id>"` (dev loop, one machine)
7. `viam module build start --version X` (cloud build, all platforms, needs `url` in `meta.json`)

More options: https://docs.viam.com/cli/build-and-deploy-modules/

What the course says I learned:
- A module is code `viam-server` runs for me: it starts it, restarts it, and hands it its declared resources.
- The contract is 3 methods: `validate_config` (check config + declare deps), `new` (resolve), `do_command` (answer callers through my verb dispatch).
- **Attributes are the interface.** Anything I'd edit code to change belongs in an attribute, validated with a message the person configuring can act on.
- Reload and release are different loops. Use the shortest loop that proves the change.
- A bounded loop can drive the cell on its own.

## How to use this for my robot (my thinking, to plan Fri/Sat)

- **Module layout:** one logic service (generic service) for the KR6 + cart job, with verbs like `status`, `drive_to`, `pick`, `place`, `run`.
- **Dependencies:** arm (KR6), base (cart), motion, camera. All required except optional sensors.
- **Attributes:** anything that changes between setups (station positions, heights, speeds, counts) goes in config with units in the names, not in the code. **Real numbers come from 30_robot, never from the course cell.**
- **Platforms in `build.arch`:** the cart computer's (arm64 if it's a Pi), the lab computer's (linux/amd64).
- **Dev loop:** reload to the lab computer while coding; cloud build when a version works.
- **Safety:** every loop that moves the real robot needs a bound and a clean stop.
