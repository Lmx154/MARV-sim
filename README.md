# MARV-sim

The simulation workspace for bringing up guidance, navigation and control on the MARV quadcopter. The plan, the
decisions and the owner's controlling document are in `MARV-V2-drivers/_plan` (`plan-sim-suite.md`,
`decision-sim-*-20261119.md`, `HANDOFF-sim-suite-20261118.md`).

State on 2026-11-19: build step 5 of 9. The vehicle card and its checker, the plant, the native backend, the IMU model,
the scenario file, the runner `marv-run` and the DataFlash log exist. The runner flies the firmware's real flight code
(`Flight<Mahony, CascadedPid, CardAllocator>` with the gate, `Tuning::DEFAULT` and the compiled card) from a stick
script, and judges each run against the thresholds of `decision-first-pair-20261119.md` (sections 8 and 13): four
scenarios and a five-point parameter spread. `takeoff-land` takes off from the ground, flies and lands with a sim pilot
(`decision-sim-takeoff-landing-20261119.md`).

## What is here

| Path | What it is |
|---|---|
| `marv-card/marv.toml` | The vehicle card: the only source of the vehicle's physical numbers. Every number is a record with a value, a unit, a tag (`measured`, `datasheet`, `cad`, `derived`, `placeholder`) and a source. |
| `marv-card/src/` | The loader and the checks (`schema.rs`) and the `card-check` program. |
| `marv-card/build.rs` | Compiles the card into `const` data for the firmware (`no_std`, without feature `std`): rotor positions and spins, the mixer matrix, coefficients, levels, mass, the card's sha256. `marv-run` refuses a card whose sha256 differs. An `"ask"` in a number the firmware needs fails the build; placeholders show as cargo warnings. |
| `marv-sim/src/` | `plant.rs` (motor levels to a wrench), `backend.rs` (wrench to truth, RK4, and the ground), `sensors.rs` (the IMU model), `rng.rs` (seeded noise), `scenario.rs` (scenario and spread files), `pilot.rs` (the sim pilot, a test device), `verdict.rs` (the checks), `log.rs` (DataFlash), `math.rs`. |
| `marv-sim/src/bin/marv-run.rs` | The runner. |
| `marv-sim/tests/invariants.rs` | The plant invariants and the ground contact. |
| `scenarios/*.toml` | The four scenarios (`hover`, `steps`, `stick-sign`, `rc-loss`), `takeoff-land`, the spread (`spread.toml`) and the take-off and landing matrix (`takeoff-land-matrix.toml`). |
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
rates are set to 0 and the attitude is kept for that tick. With a settle time (`with_settle`; `[initial] settle_s`,
0.08 s in `takeoff-land`, decision Amendment B6) the vehicle then turns at a constant body rate, shown in truth so the
gyro sees it, from the touchdown attitude to the ground plane's roll and pitch with its yaw kept; the centre of mass
stays put and accel is 0. A lift-off during the settle ends it where it is and integrates from the state shown.
`Truth` carries `on_ground` and `reaction` (normal force, horizontal force,
roll/pitch torque: minus the applied wrench). No friction, spring or pivot. `[initial] contact = "ground"` starts a
scenario held on it.

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
- `stick-sign.toml`: short roll, pitch, yaw and throttle pulses (throttle 0.6: 1.2 x weight under the flight code's
  2 x weight cap); truth must move the conventions-7b way, with little movement on the other axes.
- `rc-loss.toml`: arming rules, the RC-loss Stop within 2.5 ms of the 1.0 s timeout, no re-arm without a switch
  cycle, switch-off Stop; judged at the levels the plant is given.
- `takeoff-land.toml`: starts on the ground, motors stopped; phases instead of a script (below).

## Phases and the sim pilot

Instead of `script`, `[rc] phase = [...]` (a file has one or the other) with a `[pilot]` table. Each phase starts when
the previous one ends: after `after_s`, or at an `event` (`airborne`: height >= `[pilot] airborne_m`; `below`: height
< `below_m`; `landed`: contact at any tick since the previous report, so the first touchdown) with a `timeout_s`; a timeout is recorded and the next phase starts. The run
ends with the last phase (`duration_s` is an upper bound). `arm` is the switch (kept from the previous phase if not
given); `throttle = "zero" | "ramp"`, or `height_m`, or `descend_ms`; `roll`, `pitch`, `yaw` are scripted sticks;
`xy = "hold"` turns on the drift correction (not with roll or pitch). The runner prints each phase's start and how it
ended.

The pilot (`pilot.rs`) is a test device in the harness, NOT flight code: the flight code has no height hold, so the
harness flies the throttle as a human would by eye. It is given only a `PilotView` (truth height, vertical speed,
horizontal velocity in the heading frame, contact flag, contact at any tick since the last report, and heading)
delayed by
`delay_s`, and makes one stick report per 4 ms
through the RC path. It never sees the estimate, the allocator flags, the spread point or the level-to-speed map, and
never arms or disarms. Throttle: "zero" moves the stick to 0; "ramp" raises the throttle STICK from 0 at
`ramp_stick_per_s`; "height" and "descend" set `v_ref = clamp(k_h (h_ref - h), -v_dn, v_up)` or `-descend_ms`, then
the thrust ratio `1 + (clamp(k_v (v_ref - v), +/-accel_max_g g) + I) / g` with `I' = k_i (v_ref - v)`, frozen while
the stick is at 0 or 1. At the ramp-to-height switch `I = g (c_lift - 1)` within +/-0.5 g, where `c_lift` is the ratio
the pilot was commanding at the first report whose delayed view shows contact = false; the stick rate limit does the
smoothing (decision Amendment A1). The ratio becomes a stick by the inverse of the documented map 0 -> 0, 0.5 -> m g,
1 -> min(`stick_thrust_max_g`, 4 k_F w_max^2 / m g) x m g, computed from the nominal card (2.0: one straight line, as
the flight code's cap `pilot_thrust_max_g`; another value gives the two-slope map). Drift:
roll and pitch sticks `-(xy_k v_h + xy_ki * integral of v_h) / g / ANGLE_MAX` (30 deg), at most `xy_stick_max` in
length; the integral is kept fixed to the ground (turned by the heading change at every report, so the wind trim
survives a turn), runs only in xy = "hold" phases, is kept through the others and is frozen while the cap is
reached (Amendment B2). Every stick moves at most
`stick_rate_per_s` and stays in its range.

`check = "takeoff_land"` judges the table of the decision's section 3: Stop before switch-on, armed within 6.5 ms, idle
level exactly through "idle", the stance margin `tau_xy / (N r_stance)` while armed in contact with N above the floor, not judged on samples taken while the ground settle runs (the largest ratio skipped is printed as `INFO stance_in_settle`),
the tilt for 1 s after lift-off (first tick off the ground), no contact between lift-off and the first rise through
`airborne_m` (`liftoff_recontacts`), a single flight (one rise, never above `airborne_m` after the touchdown; backend
touchdowns, hops below `airborne_m` included, are printed), the peak height and climb speed from lift-off to touchdown,
the time from the touchdown until N >= 0.5 m g for good (`light_after_td_s`), no timeout, the height band, the
largest tilt in the air, horizontal speed, the angle between estimated and true down, no allocator flag from lift-off
to touchdown, the touchdown state before the hold (vertical and horizontal speed, tilt, rate; the worst over every
contact), idle level after the
throttle is cut, Stop after switch-off, and (nominal card only) the pilot's mean thrust ratio in hover. Touchdown is
the first contact after the first rise through `airborne_m` (Amendment B7). The pilot's
drift correction is printed.

## The parameter spread and the run matrix

    cargo run -q --release -p marv-sim --bin marv-run -- --card marv-card/marv.toml --env env/default.toml \
        --spread scenarios/spread.toml --seed 1 --out-dir logs/spread

Runs the four scenarios at the five points of `spread.toml` (nominal; k_F x0.7 or x1.3 with the inertia x1.25 or
x0.75, each with both motor time constants 0.060 or 0.030 s). Only the plant and the backend take a point; the flight
code keeps the compiled card.

A spread file is a matrix of cells: `scenarios`, `points` (required) and the axes `maps`, `air`, `ground`, `seeds`. A
missing axis has one default value that changes nothing; a missing `seeds` means `--seed` is the one seed (and
`--seed` is a usage error, exit 2, when the file has `seeds`). `scenarios/takeoff-land-matrix.toml` is `takeoff-land` at
5 points x 2 maps x 3 air x 3 ground x 3 seeds = 270 cells (decision-sim-takeoff-landing, section 4):

| axis | key | values |
|---|---|---|
| map | `speed_at_level_min` (plant only) | A: card (0); B: 128.9 rad/s |
| air | `drag` N/(m/s), `wind_ned` m/s | 1: drag 0, calm; 2: drag 0.407, calm; 3: drag 0.407, wind [0, 2, 0] |
| ground | `imu1_gyro_bias_rads`, `imu1_accel_bias_ms2`, `roll_pitch_deg` | 1: level; 2: IMU1 bias corner; 3: 5 deg roll slope |

Axis values change only the plant, the environment and the sensors, never the flight code or the pilot's stick map.

    cargo run -q --release -p marv-sim --bin marv-run -- --card marv-card/marv.toml --env env/default.toml \
        --spread scenarios/takeoff-land-matrix.toml --out-dir logs/tl [--logs fail|all] [--only <cell>]

A cell is named `p<point>-map<letter>-air<n>-gnd<n>-seed<s>` (letter and numbers by position in the file, the seed by
value), with `<scenario>-` in front when the file has more than one scenario: `p2-mapB-air3-gnd2-seed1`. One line per
cell (name, PASS or FAIL, the failing checks), then the failures per check, `PASS cells n of N`, the PASS count per ground case and per air case with the ground x air table of passing cells, the wall time (about
20 s for the 270 cells) and `RESULT`: PASS only if every cell passes. Exit 0 pass, 3 a check failed, 1 an error, 2 usage.

- `--only <cell>` runs that cell alone, prints its summary and checks, and writes its log. A rerun gives a
  byte-identical log.
- `--logs fail` (default) keeps `<out-dir>/<cell>.BIN` only for a cell with a failing check (about 11 MB each for
  `takeoff-land`); `--logs all` keeps every one.
- Range refusal (card decision, section 8): before any run, every cell is checked against the card's numeric range
  of each number it changes: k_F and the inertia scale (`range_pct`), `lag_s` (both time constants), `speed_at_level_min`,
  `drag.rotor_linear`, the IMU1 biases, and the wind. A cell outside one is refused, exit 1, naming the cell and the range
  (once per distinct fault). A number whose range is `"ask"` (the wind) has no numeric range and is not checked. A
  slope on a scenario that does not start on the ground is refused too.

The bias corner (`gnd2`): the sim reads gyro = w + b_g and accel = [b_ax, b_ay, -g] at rest. In the Mahony filter the
estimate settles at dx = -b_ax/g - b_gy/Kp and dy = -b_ay/g + b_gx/Kp (Kp 0.25 armed), so the tilt errors add on x
when b_ax and b_gy have the same sign and on y when b_ay and b_gx have opposite signs: accel +0.1961 on x and y, gyro x
-6.981e-3 and gyro y +6.981e-3. Check, nominal cell, seed 1, `nav_gap_deg` (limit 5): level 2.07; gyro alone 4.85; accel
alone 3.88; both, these signs 6.94; gyro x flipped 5.83; gyro y flipped 4.74; both gyro signs flipped 1.53; accel y
flipped 5.83; both accel signs flipped 2.89.

## The log

The log is an ArduPilot DataFlash `.BIN`. Its header is the firmware's own (`fw_core::dflog::write_header`: the FMTs of
all 18 firmware record types), followed by the FMTs of the sim-only records. It holds the firmware's records `MSG` (at
time 0: sim version, firmware commit with `-dirty` if the checkout has uncommitted changes, card hash, scenario name,
seed, and the spread point if one is used; a spread or matrix cell adds `cell`, `map`, `air` and `ground` lines with
its name and axis values), `IMU` (instance 0 and 1: exactly what the flight step was given), `RCS` (the firmware's
`encode_rcs`: one record for each stick report that reaches the flight step, with its own time, the sticks and the
switch; none for a lost report) and, for every flight step, `NAV`, `SETP`, `DMD` and `MOT` (written by the firmware's
`encode_step`); and two sim-only records: `TRU` (truth: position, velocity, attitude quaternion, body rates) and
`ROT` (the levels the plant used and the true rotor speeds, by DShot line). The sim writes nothing for `ESC` (the
firmware's id 143; its FMT is in the header): the pole count of the motors is not in the card or in `fw-core` or
`fw-flight`. Every step is logged, no decimation. A
phase-form scenario adds `GND` (every flight step: contact flag, normal force N, roll/pitch reaction torque) and `PIL`
(every report: phase index, mode 0 zero / 1 ramp / 2 height / 3 descend, h_ref or v_ref, throttle stick, integrator,
drift correction in roll and pitch after the cap, and its integral term before the cap); the script-form logs are
unchanged. The sim-only ids are 200 (`TRU`), 201 (`ROT`), 203 (`GND`) and 204 (`PIL`), none in the firmware's 128 to
145; 202 is free (it was the sim's own `RCS`).

The sim does not model the boot: the runner sets the arming permit once, before the first flight step, in place of
core 0 (the pre-arm rule is checked on the board).

## Check the plant

    cargo test -p marv-sim

Four tests: the plant invariants (free fall, hover thrust equals weight, yaw torque sign matches rotor spin) and
`ground_contact` (at rest the IMU reads gravity only; release on the first tick the net force points up; a touchdown
records the state before the hold).

## Not built yet

Barometer, compass and GNSS models, wind gusts, the shadow estimator, the TCP link and the Gazebo backend (steps 6
to 9).

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
