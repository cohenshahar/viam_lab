# How to connect Python to a machine

From Viam 101 (exercise 8), 2026-10-14.

> **Note:** the steps are general. File names (`helpers.py`, `api-key.json`) and the address `palletizer-101-main...` come from the course's workshop environment, not from my robot.

## What you need (three things)

1. **The Python SDK** (Software Development Kit: the `viam-sdk` library you code with), inside a virtual environment.
2. **An API key** (Application Programming Interface key: a key id + a key, the password your code uses to prove it is allowed in).
3. **The machine's address**: its Remote address, one hostname ending in `.viam.cloud`.

## Steps

### 1. Set up Python (once)

viam-server keeps its own terminal busy, so open a **second terminal** for your code.

```bash
python3 -m venv .venv && source .venv/bin/activate && pip install viam-sdk
```

Check it:

```bash
python -c "import viam; print('viam SDK ready')"
```

A virtual environment (`.venv`) is a private folder of Python packages for this project only, so the SDK version doesn't clash with anything else on the computer.

### 2. Get the API key

1. Viam app → machine page → **CONNECT** tab.
2. Sidebar → **API keys**.
3. Copy the **ID** and the **Key** of the row tagged **Machine Default** (copy button next to each).
4. Put them in the key file (in the course: `api-key.json`):

```json
{
  "key_id": "paste-your-key-id-here",
  "key": "paste-your-key-here"
}
```

This file is a credential. It never goes into git.

### 3. Get the machine's address

1. Machine page → **ONLINE** status dropdown at the top.
2. Click the copy icon next to **Remote address**. It copies just the hostname.
3. Paste it into the code (in the course: `MACHINE_ADDRESS` in `helpers.py`), in quotes:

```python
MACHINE_ADDRESS = "palletizer-101-main.xxxxxxxxxx.viam.cloud"
```

### 4. Prove the connection

```bash
python helpers.py
```

Good result: `connected to <your machine address>`.
If it prints `not connected`, the message names what to fix: the key file, the address, or the connection itself. Fix that step and run it again.

## Check list when it fails

- Is viam-server running and the machine **ONLINE** in the app?
- Is the venv active (prompt starts with `(.venv)`)?
- Key: did you copy both ID and Key, from the **Machine Default** row, with no extra spaces?
- Address: one hostname, ends in `.viam.cloud`, in quotes.

## On my Windows computer (not from the course)

- The project's `.venv` already exists with viam-sdk installed (see CLAUDE.md, Known facts).
- On Windows the venv is turned on with `.venv\Scripts\activate` (PowerShell or Command Prompt), not `source .venv/bin/activate`. That is standard Python, not Viam-specific.
