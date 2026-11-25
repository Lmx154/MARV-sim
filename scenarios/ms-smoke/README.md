# ms-smoke

Part of [scenarios](../README.md)

Smoke scenarios for the mission switch (channel 8) against the firmware's mission code. The scripts and the parser here run the
scenarios and tabulate their CSVs.

## Scenarios

All: one run each, seed 1, nominal card, calm air, `--estimator eskf3 --pilot-speed 3.0`.

| Scenario | Flag | What it does |
|---|---|---|
| `ms-smoke` | `--fs-hold` | Arms with the mission switch OFF, hand take-off to 5 m, Position hover. Switch ON, Start 5 s after HOLD, then the square at 3 m/s. 45 s after the switch goes ON, in the last leg: switch OFF with arm ON, Position and sticks centred. 15 s later: arm OFF and switch ON |
| `ms-smoke-a` | `--fs-hold` | Arm ON for 5 s with the switch ON, then arm OFF, switch OFF, arm ON (the control) |
| `ms-smoke-b` | `--ho` | `FS_HOLD_EN` 0: arm ON for 5 s with the switch OFF |
| `ms-smoke-c` | `--fs-hold` | As `ms-smoke` up to AUTO, then at AUTO + 20 s arm OFF and switch OFF in one report |

## Scenario keys these use

| Key | Effect |
|---|---|
| `[scenario] rc_source = "receiver"` | Stick reports are receiver reports (`RcSample.source = Receiver`) at 100/s; `"crsf"` (default) keeps 250/s |
| `mission = true\|false` in `[rc]` rows | Channel 8, carried over like `arm` and `mode`, OFF at the start |
| `throttle = <0..1>` in a phase | A fixed throttle stick, reached at the stick rate limit |
| `lost` windows | Transmitter off; stick-loss detection uses the period of the scenario's own source |
| `MARV_MS_CSV=<dir>` | One CSV per run |

## Run

```
cargo build --release --bin marv-run
scenarios/ms-smoke/run_smoke.sh <out>
python3 -I scenarios/ms-smoke/parse_smoke.py <out> > <out>/smoke_numbers.txt
scenarios/ms-smoke/crsf_rerun.sh <out>/crsf-rerun [<other marv-run binary> <suffix>]
```

`run_smoke.sh` writes `<out>/raw/<name>.txt` and `<out>/csv/<name>-seed1.csv`. `crsf_rerun.sh` re-runs the CRSF scenarios of the
stick-loss and mission sets (`a-h10-t2-g100`, `m1-square`, `takeoff-land` seed 1) as a check that the receiver option leaves
existing scenarios unchanged.

See [campaigns](../../docs/campaigns.md).
