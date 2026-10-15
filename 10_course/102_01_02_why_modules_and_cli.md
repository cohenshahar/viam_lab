# Viam 102 — Pages 1–2: Why a module, and the CLI

**Date:** 2026-10-14
**Course:** learn.viam.com, Viam 102 (pages 1–2)

## Takeaways (from the course)

- Control code can run in two places:
  1. **SDK script** (the Viam 101 way): runs on my laptop and reaches the machine over the network. Close the lid or lose Wi-Fi and it stops.
  2. **Module**: `viam-server`, on the machine's own computer, starts my code, watches it, and brings it back after a crash or a reboot.
- Hardware drivers, vision services and so on are all modules. Writing my logic as one makes it configurable, restartable, and shippable to every machine through the **Registry** (Viam's store of modules).
- **SDK** = Software Development Kit: the `viam-sdk` Python library. Scripts and modules both use it. The difference is *where the code runs* and *who is in charge of running it*.

## The CLI

- **CLI** = Command Line Interface: the `viam` program. It does what the app does, from the terminal, plus the module commands (generate, update, build, upload, reload).
- `viam module reload` builds a module and syncs it to a running machine without restarting it (docs: CLI overview). This is the "write on the laptop, run on the Pi" loop.

## Exercise 2: what I set up

| Piece | What it is |
| --- | --- |
| `viam` CLI | installed in the course terminal, logged in with `viam login` |
| `viam-102` | the course sim machine, viam-server on port 8080 |
| `shahartutorial` | my org's **public namespace** (public and permanent) |
| `palletizer-102` | the module name, so the module ID is `shahartutorial:palletizer-102` |

- Namespace rules: lowercase letters and digits only. No `-`. Module names may have `-`.
- Namespace : module is like GitHub user / repo.
- Gotcha: I pasted the next command into the namespace prompt while it was waiting for an answer. When a tool asks a question, answer it first.
- `course-tools print-all` shows the saved values if I lose them.

## For my robot (my thinking, not from the course)

- One small computer on the cart runs one `viam-server`. The KUKA controller keeps doing the low-level motor and safety work.
- On that computer: a **base** component (cart), an **arm** component (a driver module that talks to the KUKA controller), and **my logic module** that uses both.
- Not verified: whether a Pi is strong enough for arm motion planning. If not, Viam can split one machine into parts (sub-part / remote part). Test before deciding.
