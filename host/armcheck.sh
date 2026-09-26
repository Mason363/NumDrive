#!/bin/sh
# Run every level on the 32-bit ARM harness (qemu-arm) with the device arena; report failures.
cd "$(dirname "$0")"
one() {
  r=$(timeout 600 qemu-arm ./nd_arm ND_NORENDER=1 ND_STATS=1 ND_LEVEL=$1 ND_FRAMES=${FRAMES:-600} ND_KEYS=${KEYS:-RIGHT:5:900} 2>&1 | grep -v PROF | tr '\n' ' ')
  echo "$1 $r"
}
if [ -n "$1" ]; then one "$1"; exit; fi
seq 0 $((${NLEVELS:-157} - 1)) | xargs -P 8 -I{} "$0" {} | sort -n
