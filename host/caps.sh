#!/bin/sh
# Measure per-level peak object / body / joint counts under several input patterns.
# Output: ../tools/caps.txt ("level objects bodies joints"), read by tools/pack.py.
cd "$(dirname "$0")"
one() {
  i=$1; mo=0; mb=0; mj=0
  for keys in "RIGHT:5:900" "LEFT:5:900" "RIGHT:5:120,LEFT:120:240,RIGHT:240:900" "RIGHT:300:301"; do
    r=$(ND_NORENDER=1 ND_NOCAPS=1 ND_STATS=1 ND_LEVEL=$i ND_FRAMES=900 ND_KEYS=$keys timeout 120 ./nd 2>/dev/null | grep -o 'STATS arena.*')
    o=$(echo "$r" | sed -n 's/.* obj=\([0-9]*\).*/\1/p'); b=$(echo "$r" | sed -n 's/.* bodies=\([0-9]*\).*/\1/p'); j=$(echo "$r" | sed -n 's/.* joints=\([0-9]*\).*/\1/p')
    [ -n "$o" ] && [ "$o" -gt "$mo" ] && mo=$o
    [ -n "$b" ] && [ "$b" -gt "$mb" ] && mb=$b
    [ -n "$j" ] && [ "$j" -gt "$mj" ] && mj=$j
  done
  echo "$i $mo $mb $mj"
}
if [ -n "$1" ]; then one "$1"; exit; fi
seq 0 199 | xargs -P 3 -I{} "$0" {} | sort -n > ../tools/caps.txt.tmp
mv ../tools/caps.txt.tmp ../tools/caps.txt
