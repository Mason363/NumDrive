#!/bin/sh
# usage: run.sh <outdir> <level> <frames> <shots> <keys>
OUT=/home/user/NumDrive/host/$1
mkdir -p "$OUT"
find "$OUT" -maxdepth 1 -name '*.p?m' -delete
find "$OUT" -maxdepth 1 -name 'f*.png' -delete
ND_LEVEL=$2 ND_FRAMES=$3 ND_SHOTS=$4 ND_KEYS=$5 ND_OUT="$OUT" /home/user/NumDrive/host/nd
for f in "$OUT"/f*.ppm; do convert "$f" "${f%.ppm}.png"; done
montage "$OUT"/f*.png -tile 4x -geometry 320x240+2+2 "$OUT/montage.png"
