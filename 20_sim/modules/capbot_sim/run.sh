#!/usr/bin/env bash
# Start the capbot-sim module with the repo's Linux venv. viam-server runs this file.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
PY="$REPO/.venv-linux/bin/python"
if [ ! -x "$PY" ]; then
  echo "capbot-sim: no venv at $REPO/.venv-linux (see 20_sim/README.md, setup)" >&2
  exit 1
fi
cd "$HERE/src"
exec "$PY" main.py "$@"
