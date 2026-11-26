#!/bin/bash
# usage: run_smoke.sh <out-dir> [marv-run binary]: N19 smoke family (items 3 and 4). One run each, nominal card, calm air, seed 1,
# eskf3, PILOT_SPEED_XY 3.0: ms-smoke, -a, -c.
# Writes <out-dir>/raw/<name>.txt (marv-run's output) and <out-dir>/csv/<name>-seed1.csv (MARV_MS_CSV); the .BIN logs are deleted.
set -u
O=$1; B=${2:-./target/release/marv-run}
cd /home/luis/Documents/MARV-ho/MARV-sim
T=$(mktemp -d); mkdir -p $O/raw $O/csv
run() { local name=$1; MARV_MS_CSV=$O/csv "$B" --card marv-card/marv.toml --env env/default.toml --scenario scenarios/ms-smoke/$name.toml \
  --seed 1 --out $T/$name.BIN --estimator eskf3 --pilot-speed 3.0 > $O/raw/$name.txt 2>&1
  echo "exit $? flags: --scenario scenarios/ms-smoke/$name.toml --seed 1 --estimator eskf3 --pilot-speed 3.0" >> $O/raw/$name.txt; }
run ms-smoke & run ms-smoke-a & run ms-smoke-c &
wait; rm -rf $T
