# Viam 101 — Module 1: Intro and running viam-server

**Date:** 2026-10-14
**Course:** learn.viam.com, Viam 101 (sections 1–3)

## Takeaways (from the course)

- A machine is defined by its configuration: one document, stored with your machine in the Viam app. viam-server downloads it and applies it.
- viam-server needs an implementation, a module, for every resource in the config. Some modules are built into viam-server; most download from the registry.
- Every module that implements a type of hardware implements the same API, so the code you write against the simulated arm runs against a real one.

## What I did

- Created the machine `palletizer-101` in the Viam app.
- Saved the machine cloud credentials as `viam.json` and ran `./viam-server -config viam.json`.
- Got viam-server running and the machine connected.

## Notes

- `viam.json` holds the machine's secret and an API key. Never commit it to git.
- The repeating `WARN ... packet loss` lines in the viam-server log are only a network self-check (the router doesn't answer ping). They are harmless.
- Clean-up at the end of the course: delete the course machine's API key in the app (or delete the machine).
