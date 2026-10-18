#!/usr/bin/env bash
# Run the armtable-sim machine on this PC, next to capbot-sim (which viam-agent runs as a service on port 8080).
# This one runs as your user on port 8082 (set in its config) until you press Ctrl+C.
#
# Once: in the app, create machine armtable-sim (location arm-table), copy its machine cloud credentials into
# ~/.viam/armtable-sim.json (a secret: never in git), paste 20_sim/configs/armtable-sim.json into its CONFIGURE -> JSON.
set -euo pipefail
CRED="${1:-$HOME/.viam/armtable-sim.json}"
if [ ! -f "$CRED" ]; then
  echo "no credentials file at $CRED (see the comment at the top of this script)" >&2
  exit 1
fi
mkdir -p "$HOME/.viam/armtable-sim-home"
exec env VIAM_HOME="$HOME/.viam/armtable-sim-home" /opt/viam/bin/viam-server -config "$CRED"
