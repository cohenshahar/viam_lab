#!/usr/bin/env bash
# Screen-record the 3D scene while the pallet demo runs, then make a 4x faster copy.
#
#   20_sim/scripts/record_demo.sh laptop 8     record the laptop screen, 8 crates
#   20_sim/scripts/record_demo.sh hdmi 2       record the HDMI monitor, 2 crates
#
# Before: open capbot-sim -> 3D SCENE on that screen, press F11, set the view. Do not touch the mouse while it runs.
# Output (not in git): 20_sim/scripts/out/demo_<date>_full.mp4 and demo_<date>_4x.mp4
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCREEN="${1:-laptop}"
CRATES="${2:-8}"
case "$SCREEN" in
  laptop) SIZE=1920x1080; AT=":0.0+0,0" ;;
  hdmi)   SIZE=1680x1050; AT=":0.0+1920,0" ;;
  *) echo "screen must be laptop or hdmi" >&2; exit 1 ;;
esac
mkdir -p "$HERE/out"
STAMP="$(date +%Y-%m-%d_%H%M)"
FULL="$HERE/out/demo_${STAMP}_full.mp4"
FAST="$HERE/out/demo_${STAMP}_4x.mp4"

echo "recording the $SCREEN screen ($SIZE) -> $FULL"
ffmpeg -hide_banner -loglevel error -y -f x11grab -framerate 25 -video_size "$SIZE" -i "$AT" \
       -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p "$FULL" &
REC=$!
sleep 3   # a moment of the start position
cd "$HERE"
../../.venv-linux/bin/python demo_pick_place.py --crates "$CRATES" 2>&1 | grep -v INFO || true
sleep 3
kill -INT "$REC"   # ffmpeg closes the file cleanly on Ctrl+C
wait "$REC" || true

echo "making the 4x copy -> $FAST"
ffmpeg -hide_banner -loglevel error -y -i "$FULL" -vf "setpts=PTS/4" -r 25 -an \
       -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p "$FAST"
ls -lh "$FULL" "$FAST"
