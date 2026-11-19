# MARV-sim

The simulation workspace for bringing up guidance, navigation and control on the MARV quadcopter. The plan, the
decisions and the owner's controlling document are in `MARV-V2-drivers/_plan` (`plan-sim-suite.md`,
`decision-sim-*-20261119.md`, `HANDOFF-sim-suite-20261118.md`).

State on 2026-11-19: build step 5 of 9. The vehicle card and its checker, the plant, the native backend, the IMU model,
the scenario file, the runner `marv-run` and the DataFlash log exist. The runner flies the firmware's real flight code
(`Flight<Mahony, CascadedPid, CardAllocator>` with the gate, `Tuning::DEFAULT` and the compiled card) from a stick
script, and judges each run against the thresholds of `decision-first-pair-20261119.md` (sections 8 and 13): four
scenarios and a five-point parameter spread.

## What is here

| Path | What it is |
|---|---|
| `marv-card/marv.toml` | The vehicle card: the only source of the vehicle's physical numbers. Every number is a record with a value, a unit, a tag (`measured`, `datasheet`, `cad`, `derived`, `placeholder`) and a source. |
| `marv-card/src/` | The loader and the checks (`schema.rs`) and the `card-check` program. |
| `marv-card/build.rs` | Compiles the card into `const` data for the firmware (`no_std`, without feature `std`): rotor positions and spins, the mixer matrix, coefficients, levels, mass, the card's sha256. `marv-run` refuses a card whose sha256 differs. An `"ask"` in a number the firmware needs fails the build; placeholders show as cargo warnings. |
| `marv-sim/src/` | `plant.rs` (motor levels to a wrench), `backend.rs` (wrench to truth, RK4, and the ground), `sensors.rs` (the IMU model), `rng.rs` (seeded noise), `scenario.rs` (scenario and spread files), `verdict.rs` (the checks), `log.rs` (DataFlash), `math.rs`. |
| `marv-sim/src/bin/marv-run.rs` | The runner. |
| `marv-sim/tests/invariants.rs` | The plant invariants and the ground contact. |
| `scenarios/*.toml` | The four scenarios (`hover`, `steps`, `stick-sign`, `rc-loss`) and the spread (`spread.toml`). |
| `env/default.toml` | Gravity, atmosphere, origin, magnetic field, wind. Its origin is a public reference point, never home. |
| `env/home.local.toml` | The real home location, if you create it. Git-ignored; never commit it. |

The firmware is a path dependency: `marv-sim` builds `fw-flight` and `fw-core` from `../MARV-V2-firmware` (read only
from here), so an edit to the flight code runs in the sim at once.

## Run the sim

    cargo run -q --release -p marv-sim --bin marv-run -- --card marv-card/marv.toml --env env/default.toml \
        --scenario scenarios/hover.toml --seed 1 --out logs/hover-seed1.BIN

It prints a summary, then one line per check and a last line:

    CHECK tilt_settled_deg 1.0852 <=1.5000 PASS
    RESULT: PASS

Exit status: 0 every check passed, 3 a check failed, 1 a load, refusal or run error (message on stderr), 2 a usage
error. `--point "<kf_scale>,<inertia_scale>,<lag_s or ->"` runs one spread point (`-` keeps the card's motor time
constants). The same card, environment, scenario, point and seed give a byte-identical log. `logs/` is git-ignored.

The loop: base tick 125 us. Every 8 ticks (1 kHz) the plant takes `fw_flight::esc::hold(..).throttles()` of the
newest flight command, the board's own stale-command rule (0 for Stop). Every 32 ticks (4 ms) a stick report is made
from the script row in force, unless the link is lost then; the newest one goes to the next flight step, once. Every
tick the plant makes a wrench (plus any disturbance torque) and the backend integrates it. Every 10 ticks (800 Hz)
both IMU models make a sample and the flight step runs once.

## The plant, the ground and the drag

The plant adds the weight, the quadratic body drag (`drag.body`) and one linear rotor drag, `F_xy = -drag.rotor_linear *
va_xy` in body x and y, none in z (`va` = air-relative velocity in body FRD). The card holds it at 0 (term off); its
range [0, 0.41] N/(m/s) is an order-of-magnitude bound from another quadrotor (see its `src`).

The native backend has an optional ground plane at down = 0 for the centre of mass (`Start` in `backend.rs`;
`NativeBackend::new` has none, as before). In GROUND the position and attitude are held and velocity, rates, accel and
angular accel are 0, so the IMU model reads gravity only. The vehicle lifts off at the start of a tick when the applied
force in NED (weight included) points up, and that tick integrates normally. It touches down at the end of a tick with
down >= 0: the velocity, tilt and rates from before the hold go to `NativeBackend::touchdown()`, then down, velocity and
rates are set to 0 and the attitude is kept. `Truth` carries `on_ground` and `reaction` (normal force, horizontal force,
roll/pitch torque: minus the applied wrench). No friction, spring or pivot. No scenario file asks for the ground yet.

## The scenario file

Plain TOML; an unknown key is an error in every table and row. `[verdict]` is required: a file without it does not
load. Times are seconds; each script row holds until the next.

    [scenario]
    name = "hover"
    duration_s = 10.0
    [initial]                     # position, velocity, roll_pitch_yaw_deg (Z-Y-X), rates, rotors = "hover" | "stopped"
    ...
    [rc]
    script = [ { t_s = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0, throttle = 0.0, arm = false }, ... ]
    lost = [ { from_s = 3.0, to_s = 5.0 } ]          # optional: no report in [from, to)
    [[disturbance]]                                  # optional: body torque from from_s (to to_s if given)
    from_s = 5.0
    torque_nm = [0.046, 0.0, 0.0]
    [verdict]
    check = "hover"                                  # "hover" | "steps" | "stick_sign" | "rc_loss"
    max_tilt_deg = 1.5                               # ... that check's thresholds, each with its decision section

The four scenarios (all start level in the air with the rotors at hover speed; switch on at 0.020 s, throttle 0.5 at
0.040 s, except `rc-loss`, which plays the table of decision 8d):

- `hover.toml`: 10 s with a 0.046 N.m roll torque from 5 s; truth tilt, estimate-versus-truth tilt, heading change,
  peak tilt under the disturbance and the settled tilt.
- `steps.toml`: roll, pitch (15 deg) and yaw (75 deg/s) steps and returns; 10-90 % rise time and overshoot, roll and
  pitch on the estimate, yaw on truth body rate; the truth-versus-estimate gap is printed, not judged.
- `stick-sign.toml`: short roll, pitch, yaw and throttle pulses; truth must move the conventions-7b way, with little
  movement on the other axes.
- `rc-loss.toml`: arming rules, the RC-loss Stop within 2.5 ms of the 1.0 s timeout, no re-arm without a switch
  cycle, switch-off Stop; judged at the levels the plant is given.

## The parameter spread

    cargo run -q --release -p marv-sim --bin marv-run -- --card marv-card/marv.toml --env env/default.toml \
        --spread scenarios/spread.toml --seed 1 --out-dir logs/spread

Runs the four scenarios at the five points of `spread.toml` (nominal; k_F x0.7 or x1.3 with the inertia x1.25 or
x0.75, each with both motor time constants 0.060 or 0.030 s). Only the plant and the backend take a point; the flight
code keeps the compiled card. One log per run (`<scenario>-p<NN>-seed<S>.BIN`), one table (PASS, or FAIL with the
first failing check), exit 0 only if all 20 pass, 3 if any fails, 1 on an error.

## The log

The log is an ArduPilot DataFlash `.BIN`. It holds the firmware's records `MSG` (at time 0: sim version, firmware
commit with `-dirty` if the checkout has uncommitted changes, card hash, scenario name, seed, and the spread point if
one is used), `IMU` (instance 0 and 1: exactly what the flight step was given) and, for every flight step, `NAV`,
`SETP`, `DMD` and `MOT` (written by the firmware's `encode_step`); and three sim-only records: `TRU` (truth:
position, velocity, attitude quaternion, body rates), `ROT` (the levels the plant used and the true rotor speeds, by
DShot line) and `RCS` (at every report time: the sticks, the switch, and `Lost` = 1 inside a lost interval).

## Check the plant

    cargo test -p marv-sim

Four tests: the plant invariants (free fall, hover thrust equals weight, yaw torque sign matches rotor spin) and
`ground_contact` (at rest the IMU reads gravity only; release on the first tick the net force points up; a touchdown
records the state before the hold).

## Not built yet

Barometer, compass and GNSS models, wind gusts, the shadow estimator, the TCP link and the Gazebo backend (steps 6
to 9). The spread does not yet refuse a point outside a number's card range.

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

Six tests in `marv-card` (one card that passes, and one broken card per check) and the four of `marv-sim`.
