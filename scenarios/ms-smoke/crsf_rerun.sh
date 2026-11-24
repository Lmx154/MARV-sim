#!/bin/bash
# usage: crsf_rerun.sh <out-dir> [marv-run binary] [suffix]: the CRSF re-run of N19's sim packet, item 2. Three existing
# CRSF runs: part 3's stick-loss cell group a-h10-t2-g100 (15 cells, FS_HOLD_EN 0, --ho), part 3's owner's-flight group
# m1-square (15 cells, --fs-hold; the stick loss of the model change), and the hand flight takeoff-land.toml (seed 1).
# Writes <out-dir>/<name><suffix>.txt; logs go to a temporary folder and are deleted.
set -u
O=$1; B=${2:-./target/release/marv-run}; X=${3:-.new}
N2=/home/luis/Documents/MARV-V2-drivers/_plan/flights/20261124-handsoff-sim-n2/tools/sp
M=/home/luis/Documents/MARV-V2-drivers/_plan/flights/20261124-mission-sim/tools/sp
cd /home/luis/Documents/MARV-ho/MARV-sim
T=$(mktemp -d); mkdir -p $O
run() { local name=$1; shift; "$B" --card marv-card/marv.toml --env env/default.toml "$@" > $O/$name$X.txt 2>&1; echo "exit $? flags: $*" >> $O/$name$X.txt; }
run a-h10-t2-g100 --spread $N2/a-h10-t2-g100.toml --out-dir $T/a --estimator eskf3 --ho &
run m1-square --spread $M/m1-square.toml --out-dir $T/m --estimator eskf3 --fs-hold --pilot-speed 3.0 &
run takeoff-land --scenario scenarios/takeoff-land.toml --seed 1 --out $T/tl.BIN &
wait; rm -rf $T
