# Viam 102 — Exercise 3: Create your module

**Date:** 2026-10-14
**Course:** learn.viam.com, Viam 102 (exercise 3)

## The workflow (same for every module)

| Step | Command | Why |
| --- | --- | --- |
| Create | `viam module generate` | makes the files and registers the module ID in my org |
| Declare models | edit `meta.json` → `"models"` | the Registry reads this list to know what the module gives |
| Push the manifest | `viam module update` | run it **after every `meta.json` change** |
| Install packages | `./setup.sh` | builds the module's own venv from `requirements.txt`; run again when I add a package |

Generate answers I used: Module, Private, keep namespace, Register = Yes, keep name, Python, **Generic Service**, model name `palletizer`.

## The files

- `meta.json` — the manifest (module ID, visibility, description, models).
- `src/models/palletizer.py` — the `Palletizer` class (replaced with the course starter by `course-tools copy-starter-code`).
- `requirements.txt` + `setup.sh` — the module's Python packages and the script that installs them.
- `build.sh`, `.github/` — packaging and cloud build (later pages).

## The model triplet

`shahartutorial:palletizer-102:palletizer` = namespace : module : model.
One module can hold several models, which is why there is a third part.

It must match in **two places**:
- `meta.json` → `"model": "shahartutorial:palletizer-102:palletizer"`
- `palletizer.py` → `Model(ModelFamily("shahartutorial", "palletizer-102"), "palletizer")`

If they differ, `viam-server` can't find my class (check 3.2 tests this).
`markdown_link` uses `_` instead of `:` because it is a file name.

## Service, not component

- **Component** = a device (arm, gripper, base).
- **Service** = software that uses devices. The palletizer is logic, so it is a service.
- For my robot: a KR6 driver would be a component (arm); the pick-and-place loop would be a service.

## The module contract (the big change from 101)

- In 101, *my script* fetched each resource by name.
- In a module, **`viam-server` hands them to me**:
  - `validate_config` returns two lists: required and optional dependencies.
  - `viam-server` waits until every required one is up, then calls `new` with them in `dependencies`.
  - `new` keeps them on `self` (`self.gripper`, `self.motion`, ...). Optional ones use `dependencies.get(...)` and may be `None`.
- `do_command` is the one entry point: JSON in, JSON out, routed on `command["command"]`. `status` and `clear` are wired; unknown verbs return an error instead of a crash.
- From the starter: `pick` is complete; `place` and `pack` are mine to write.

## Gotchas from my run

- First `generate` failed with namespace `"shaharlab"`: an old value was in my environment. `course-tools print-all` shows what's saved.
- One `generate` run had the cursor on **Public**. Check `"visibility"` in `meta.json` is `"private"`; if not, fix and run `viam module update`.
- Open question for the next page: `pallet_top()` uses `self.pallet`, but `new` never sets it and `PALLET` is not in `validate_config`'s list.
