# How to restart a course session (after a restart / new session)

From Viam 101, 2026-10-14. For the course's workshop environment (the editor + terminal in the browser).

## Terminal 1: viam-server (leave it running)

```bash
./viam-server -config viam.json
```

If it says `viam-server: No such file or directory` (the file is gone), download it again first:

```bash
ARCH=$(uname -m) && curl -fsSL "https://storage.googleapis.com/packages.viam.com/apps/viam-server/viam-server-stable-${ARCH}" -o viam-server && chmod +x viam-server
```

If `viam.json` is gone: Viam app → status dropdown → **Machine cloud credentials** → paste into a new `viam.json`.

Check: the app shows **ONLINE** (refresh the page).

> **Watch out:** anything typed into the viam-server terminal does NOT run, it only shows up as text in the log. Switch terminals with the dropdown at the top right of the terminal panel.

## Terminal 2: Python (your code)

Open a second terminal (**+** in the terminal panel). The venv should already be on (prompt starts with `(.venv)`). If not:

```bash
source .venv/bin/activate
```

Check:

```bash
python helpers.py
```

Good result: `connected to <machine address>`.
