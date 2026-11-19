# MARV-sim

The simulation workspace for bringing up guidance, navigation and control on the MARV quadcopter. The plan, the
decisions and the owner's controlling document are in `MARV-V2-drivers/_plan` (`plan-sim-suite.md`,
`decision-sim-*-20261119.md`, `HANDOFF-sim-suite-20261118.md`).

State on 2026-11-19: build step 4 of 9. The vehicle card and its checker, the plant, the native backend, the IMU model,
the scenario file, the runner `marv-run` and the DataFlash log exist. The runner calls the firmware's real flight
step, which today holds stand-ins (no estimator, no controller, no allocator) and always answers Stop.

## What is here

| Path | What it is |
|---|---|
| `marv-card/marv.toml` | The vehicle card: the only source of the vehicle's physical numbers. Every number is a record with a value, a unit, a tag (`measured`, `datasheet`, `cad`, `derived`, `placeholder`) and a source. |
| `marv-card/src/` | The loader and the checks (`schema.rs`) and the `card-check` program. |
| `marv-sim/src/` | `plant.rs` (motor levels to a wrench), `backend.rs` (wrench to truth, RK4), `sensors.rs` (the IMU model), `rng.rs` (seeded noise), `scenario.rs`, `log.rs` (DataFlash), `math.rs`. |
| `marv-sim/src/bin/marv-run.rs` | The runner. |
| `marv-sim/tests/invariants.rs` | The three plant invariants. |
| `scenarios/hover.toml` | The one scenario: level, rotors at hover speed, ten seconds. |
| `env/default.toml` | Gravity, atmosphere, origin, magnetic field, wind. Its origin is a public reference point, never home. |
| `env/home.local.toml` | The real home location, if you create it. Git-ignored; never commit it. |

The firmware is a path dependency: `marv-sim` builds `fw-flight` and `fw-core` from `../MARV-V2-firmware` (read only
from here), so an edit to the flight code runs in the sim at once.

## Run the sim

    cargo run -q --release -p marv-sim --bin marv-run -- --card marv-card/marv.toml --env env/default.toml \
        --scenario scenarios/hover.toml --seed 1 --out logs/hover-seed1.BIN

All five options are required. It prints a summary and a last line `RESULT: not judged (the scenario has no thresholds
yet)`. Exit 0 is a finished run, 1 a load or refusal error (message on stderr), 2 a usage error. The same card,
environment, scenario and seed give a byte-identical log. `logs/` is git-ignored.

The loop: base tick 125 us; every 8 ticks the plant takes the newest motor command (the stale-command rule is step 5);
every tick the plant makes a wrench and the backend integrates it; every 10 ticks (800 Hz) both IMU models make a
sample and the flight step runs once.

The log is an ArduPilot DataFlash `.BIN`. It holds the firmware's records `MSG` (at time 0: sim version, firmware
commit with `-dirty` if the checkout has uncommitted changes, card hash, scenario name, seed) and `IMU` (instance 0 and
1: exactly what the flight step was given), and two sim-only records: `TRU` (truth: position, velocity, attitude
quaternion, body rates) and `ROT` (the levels the plant used and the true rotor speeds, by DShot line).

## Check the plant

    cargo test -p marv-sim

Three tests: the plant invariants (free fall, hover thrust equals weight, yaw torque sign matches rotor spin).

## Not built yet

Barometer, compass and GNSS models, RC and the stick script, thresholds and the pass/fail verdict, the stale-command
hold rule, parameter spread, wind gusts, the shadow estimator, the TCP link and the Gazebo backend (steps 5 to 9).

## Check the card

    cargo run -q -p marv-card --bin card-check -- marv-card/marv.toml env/default.toml

It prints `PASS` and exits 0 when every number has a tag and the card is consistent, then lists the open items: every
placeholder and every value or range that is still `"ask"`. Any error exits 1.

## Change the card

- The block between `# >>> BEGIN GENERATED` and `# <<< END GENERATED` comes from the CAD export. Do not edit it; the
  check notices. Regenerate it from the CAD repo, after committing `model/mass_properties.json` there:

      cd ~/Projects/CAD/drone && python3 tools/make_card.py ~/Documents/MARV-sim/marv-card/marv.toml

- Everything else is edited by hand. Give each number its tag and say in `src` where it came from. A `placeholder`
  needs a range, or `range = "ask"`.

## Tests

    cargo test --workspace

Six tests in `marv-card` (one card that passes, and one broken card per check) and the three plant invariants.
