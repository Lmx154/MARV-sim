# Tools

Part of [MARV-sim](../README.md)

Three programs: `marv-run` flies scenarios, `marv-replay` replays a flown log through the estimators, and `card-check` validates the
card. All are built with cargo from the workspace root.

| Program | Crate | Source |
|---|---|---|
| `marv-run` | marv-sim | `marv-sim/src/bin/marv-run.rs` |
| `marv-replay` | marv-sim | `marv-sim/src/bin/marv-replay.rs` (log reader in `src/replay.rs`) |
| `card-check` | marv-card (feature `std`) | `marv-card/src/bin/card-check.rs` |

## marv-run

```
marv-run --card <marv.toml> --env <env.toml> --seed <u64> --scenario <file> --out <file.BIN> [--point <kf>,<inertia>,<lag|->] [options]
marv-run --card <marv.toml> --env <env.toml> [--seed <u64>] --spread <file> --out-dir <dir> [--only <cell>] [--logs fail|all] [options]
```

| Group | Options | Page |
|---|---|---|
| Inputs | `--card`, `--env`, `--home <env/home.local.toml>`, `--scenario`, `--seed`, `--point`, `--params` | [vehicle-model](vehicle-model.md), [scenarios](scenarios.md) |
| Output | `--out`, `--out-dir`, `--only`, `--logs fail\|all`, `--window <from,to>` | [campaigns](campaigns.md) |
| Spread | `--spread` | [campaigns](campaigns.md) |
| Estimators | `--estimator`, `--shadow`, `--preroll`, `--noaid-sigma`, `--tuning-override`, `--notch` | [estimators](estimators.md) |
| Hands-off and mission | `--fs-hold`, `--ho`, `--no-heartbeat`, `--x-inf`, `--pilot-speed`, `--ground-s`, `--gdn`, `--gdn-radius` | [campaigns](campaigns.md) |

`--seed` is given exactly when the spread file has no `seeds` axis. Exit status: 0 every check passed, 3 a check failed, 1 a load,
refusal or run error (message on stderr), 2 a usage error. A card whose sha256 differs from the compiled one is refused.

Output: a summary, one `CHECK <name> <value> <limit> PASS|FAIL` line per check, then `RESULT: PASS|FAIL`:

```
CHECK tilt_settled_deg 1.0852 <=1.5000 PASS
RESULT: PASS
```

Environment variables: `MARV_MS_CSV=<dir>` (one CSV per run, see [campaigns](campaigns.md)); `MARV_HO_DUMP` writes a per-step
hands-off dump.

## The log

An ArduPilot DataFlash `.BIN`. The header is the firmware's own (`fw_core::dflog::write_header`: the FMTs of all firmware record
types) followed by the FMTs of the sim-only records. `logs/` is git-ignored: a run with `--home` carries GNSS coordinates.

| Record | Source | Content |
|---|---|---|
| `MSG` | firmware | At time 0: sim version, firmware commit (`-dirty` if uncommitted), card hash, scenario, seed, spread point; cells add `cell`, `map`, `air`, `ground` |
| `IMU` | firmware | Instances 0 and 1: exactly what the flight step was given |
| `RCS` | firmware (`encode_rcs`) | One per stick report that reaches the flight step; none for a lost report |
| `NAV`, `SETP`, `DMD`, `MOT` | firmware (`encode_step`) | Every flight step, no decimation |
| `MODE`, `SPN` | firmware | Selected and effective mode at the first step and every change; the altitude target while the setpoint is Altitude |
| `TRU` (200) | sim | Truth: position, velocity, attitude quaternion, body rates |
| `ROT` (201) | sim | Levels the plant used and the true rotor speeds, by DShot line |
| `GND` (203) | sim, phase scenarios | Contact flag, normal force, roll/pitch reaction torque, every step |
| `PIL` (204) | sim, phase scenarios | Pilot phase, mode, reference, stick, integrator, drift correction |

The sim writes no `ESC` record (the motor pole count is not in the card or the firmware crates). Sim-only ids 200, 201, 203 and 204
are outside the firmware's 128 to 145; 202 is free.

## marv-replay

```
marv-replay <log.BIN> <outdir> [--every N] [--imu-period-us US] [--mag-expected UT,INC_DEG,STR_PCT,TOL_DEG] [--noaid-sigma M]
    [--no-baro-from-armed N] [--gps-delay-ms MS] [--gps-min-rate-hz HZ] [--no-mag] [--nki] [--steering mahony-baro|eskf2|eskf3]
    [--veld] [--holdout-vd-odd] [--vd-sigma M_S] [--notch] [--notch-harmonics 1|3] [--notch-fs HZ]
```

Feeds a log written by the board through the firmware's own estimators (`Mahony`, `MahonyBaro`, `Eskf`, all `Tuning::DEFAULT`) and
checks them against the log's own `NAV` and `NKF1` records. It writes one CSV per estimator (a row every N-th step, default 8),
prints the final `EskfStats` and a validation table per armed interval.

| Rule | Detail |
|---|---|
| Steps | One per logged IMU1 sample. Disarmed, the board logs every 8th sample, so in each gap the last sample is repeated at the nominal IMU period (median spacing of armed samples, or `--imu-period-us`); real and filled counts are printed |
| Slow sensors | Baro, compass and GNSS samples go to the first step whose IMU time is >= their own, the newest only. GNSS comes from `GPS`, `GPA`, `GPV`; `GPA` keeps only sqrt(lat^2+lon^2), split equally into latitude and longitude deviation |
| Armed state | `MOT.Arm` |
| Compass expectation | From the log's `PARM` records (`MAG_CAL_TYPE`, `MAG_FLD_UT`, `MAG_FLD_INC`, `MAG_HLT_STR`, `MAG_HLT_INC`); `--mag-expected` only for a log without `MAG_CAL_TYPE`. The source is printed |
| `--notch` | No estimator runs: the log's `NAV` rates and ESC eRPM go through the firmware's `CascadedPid` with the notch off, on, and on with every eRPM withheld (negative control: rates equal the input bit for bit); writes `<outdir>/notch_h<N>.csv` and prints the rms of the D torque above 50 Hz for the hover windows |

A log holds GPS coordinates: write the CSVs outside the repo and do not commit them.

## card-check

`card-check <card.toml> <env.toml>` prints `PASS`, exit 0, when every number is tagged and consistent (and the env's home override is
valid), then lists open items. Errors exit 1; a wrong argument count exits 2. See [vehicle-model](vehicle-model.md).

## Tests

`cargo test --workspace` (add `--release` for the long ones).

| Test file | Covers |
|---|---|
| `marv-card` tests | One passing card, one broken card per check, sensor fields, home override |
| `marv-sim/tests/invariants.rs` | Free fall, hover thrust equals weight, yaw torque sign matches rotor spin, ground contact (IMU reads gravity at rest, release when net force points up, touchdown records the pre-hold state) |
| `marv-sim/tests/slow_sensors.rs` | Each slow model against its definition |
| `marv-sim/tests/althold_verdict.rs` | The `alt_hold` mode-sequence and I6 checks fail on synthetic traces |
| `marv-sim/tests/nav_vertical.rs`, `nav_eskf.rs` | Estimator tests, see [estimators](estimators.md) |

## Related

[architecture](architecture.md), [campaigns](campaigns.md), [marv-sim folder](../marv-sim/README.md)
