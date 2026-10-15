# Viam 102 — Exercise 4: Release your module

**Date:** 2026-10-15
**Course:** learn.viam.com, Viam 102 (exercise 4)

## What I did

| Step | Command / action | What happened |
| --- | --- | --- |
| Build | `viam module build local` | PyInstaller packed the Python code and its packages into one program: `dist/archive.tar.gz` |
| Release | `viam module upload --version 0.1.0 --platform linux/amd64 --upload dist/archive.tar.gz` | published version 0.1.0 to the Registry |
| Install | CONFIGURE → `+` → Blocks → search `palletizer-102` → Add → name `palletizer-1` → **Save** | `viam-server` downloaded 0.1.0, ran `validate_config`, resolved the dependencies, ran `new` |
| Check | LOGS + DoCommand `{"command": "status"}` | module started, `palletizer-1` configured, resources `"ok"` |

## Takeaways

- **PyInstaller** = a tool that packs Python and its packages into one program. The result is compiled for one platform, so the upload must name it (`linux/amd64`).
- **Versions are permanent.** The next upload needs a new number (0.1.1, 0.2.0, ...).
- The machine config chooses which version to run and whether updates are automatic or pinned (docs: Manage software versions and rollouts).
- The Registry page is built from `meta.json`: description, models list, and the version history.
- Nothing happens until **Save**. "Unsaved changes" / "NOT SAVED" means `viam-server` doesn't know about the new block yet.
- Run every `viam module ...` command from **inside the module folder** (it reads `./meta.json`).

## For my robot (my thinking)

- A Raspberry Pi is `linux/arm64`, not `linux/amd64`. A module for the cart computer needs an arm64 build.
- A module and a machine are separate: the machine `viam-102` (in location `viam-education`) runs the module `palletizer-102` from the Registry.
