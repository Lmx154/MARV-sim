# MARV-sim

The simulation workspace for bringing up guidance, navigation and control on the MARV quadcopter. The plan, the
decisions and the owner's controlling document are in `MARV-V2-drivers/_plan` (`plan-sim-suite.md`,
`decision-sim-*-20261119.md`, `HANDOFF-sim-suite-20261118.md`).

State on 2026-11-21: build step 7 of 9 in progress (steps 1 to 6 done). The vehicle card and its checker, the plant, the
native backend, the IMU model, the barometer, compass and GNSS models (step 7, navigation phase 1, packet S1), the
scenario file, the runner `marv-run` and the DataFlash log exist. The runner flies the firmware's real flight code
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
| `marv-sim/src/` | `plant.rs` (motor levels to a wrench), `backend.rs` (wrench to truth, RK4, and the ground), `sensors.rs` (the IMU model), `slow_sensors.rs` (the barometer, compass and GNSS models and their path to the flight step), `rng.rs` (seeded noise), `scenario.rs` (scenario and spread files), `pilot.rs` (the sim pilot, a test device), `verdict.rs` (the checks), `log.rs` (DataFlash), `math.rs`. |
| `marv-sim/src/bin/marv-run.rs` | The runner. |
| `marv-sim/tests/invariants.rs` | The plant invariants and the ground contact. |
| `marv-sim/tests/slow_sensors.rs` | The barometer, compass and GNSS models against their definitions. |
| `scenarios/*.toml` | The four scenarios (`hover`, `steps`, `stick-sign`, `rc-loss`), `alt-hold` (Amendment F, F9), `takeoff-land`, the spread (`spread.toml`) and the take-off and landing matrix (`takeoff-land-matrix.toml`). |
| `env/default.toml` | Gravity, atmosphere, origin, magnetic field, wind. Its origin is a public reference point, never home: the Royal Observatory, Greenwich (51.47783 N, 0.00139 W, 68 m), with the WMM2025 field there for 2026-11-20 (19.561, 0.434, 45.067 uT NED; declination 1.271 deg, inclination 66.532 deg). |
| `env/home.local.toml` | The real home location, if you create it: records of `[origin]` and `[magnetic_field]` that replace the default's under `--home`. Git-ignored; never commit it. |

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
constants). `--home env/home.local.toml` (either form) runs at the owner's origin: the file must be the env's git-ignored
`home_override`, and each of its records replaces the same record of `--env`; a record that is still `"ask"` there and that
a model needs (today: `origin.altitude`) refuses the run. The same card, environment, scenario, point and seed give a
byte-identical log. `logs/` is git-ignored.

The loop: base tick 125 us. Every 8 ticks (1 kHz) the plant takes `fw_flight::esc::hold(..).throttles()` of the
newest flight command, the board's own stale-command rule (0 for Stop). Every 32 ticks (4 ms) a stick report is made
from the script row in force, unless the link is lost then; the newest one goes to the next flight step, once. Every
tick the plant makes a wrench (plus any disturbance torque) and the backend integrates it, and the slow sensors record
the truth; every 160 ticks (50 Hz) the barometer, every 80 ticks (100 Hz) the compass and every 1600 ticks (5 Hz; 8000 or
800 at 1 or 10 Hz) the GNSS make a sample, the newest of each waiting for the next flight step. Every 10 ticks (800 Hz)
both IMU models make a sample and the flight step runs once, with each waiting slow sample once (`Sensors.baro`, `mag`,
`gnss`; `None` when there is none new), as the board's slow-sensor path gives them.

## The slow sensors

`slow_sensors.rs`, decision-nav-eskf-20261120 section 9a-c. Each model makes the firmware's sample type
(`fw_core::sample::{BaroSample, MagSample, GnssSample}`) stamped `t_us` with the sim time of its tick and `seq` +1 per
sample, from the truth of the card's latency earlier (a truth history at the 125 us tick: 92, 40 and 3440 ticks). Each
draws from its own noise stream (3, 4, 5; the IMUs keep 1 and 2), so every existing run is unchanged.

- Barometer: the ISA troposphere (env `[atmosphere]`, specific gas constant 287.05287 J/(kg K) of ISO 2533) at the origin
  altitude minus down, after adding two height errors: a drift random walk whose 1-sigma size after `baro.drift_time` (840
  s) is `baro.drift_sd` (0.77 m), and, while not on the ground, white noise `baro.airborne_height_noise_sd` (0.11 m); then
  `baro.pressure_bias` and white noise `baro.pressure_noise_sd`. Also, while not on the ground, a motion-related height error (own stream 7): band-limited noise (`baro.motion_band_lo_hz` to `_hi_hz`, 0.08 to 0.5 Hz) of 1-sigma `baro.motion_error_per_deg` (0.0476 m/deg, fitted on logs 278 and 281) x the truth tilt; `[scenario] baro_motion = false` leaves it out. The mechanism is unknown. Temperature: ISA at that height. Not modelled: the
  ground-effect error (0.47 m near the ground under thrust) and the speed error (+0.6 m above 3 m/s).
- Compass: the env field turned into body FRD by the true attitude, then the TRUE distortion `compass.soft_iron x field +
  compass.hard_iron_bias` plus white noise `compass.field_noise_sd`, then the CALIBRATION APPLIED, `correction x (raw -
  hard_iron)` (`Calibration`). The runner applies none (`Calibration::NONE`), so the card's soft and hard iron are what the
  firmware's calibration leaves (as `hard_iron_bias`'s source says); with a perfect calibration (hard iron equal,
  correction the inverse of the soft iron) the sample is the env field in body FRD. No mounting rotation: the board hands
  flight code the field after `MAG_ORIENT`. Every sample also carries `MagSample.expected` (the health test's expected
  field, Amendment C4): the strength and inclination of the env field (49.131 uT, 66.532 deg down for the default env)
  with the firmware's default tolerances `MAG_HLT_STR` 10 % and `MAG_HLT_INC` 10 deg, converted as the core-0 fill does
  (fraction, radians). The sim does not depend on `fw-settings`, so the two tolerances are constants in `slow_sensors.rs`
  (the firmware's table test pins the defaults); `[scenario] mag_expected = false` leaves it `None`. It draws nothing.
- GNSS: truth at t - `gnss.latency`; north and east errors a first-order Gauss-Markov term (`gnss.slow_horizontal_sd`,
  `gnss.slow_time_constant`, starting in its stationary spread) plus white noise `gnss.horizontal_sd`; down error a
  Gauss-Markov term (`gnss.slow_vertical_sd`); velocity north and east plus white noise `gnss.velocity_sd`, as speed and
  course; the course is held below 0.3 m/s (none before the first motion). Fix type 3; the deviations are
  `gnss.horizontal_sd` (latitude, longitude) and `gnss.vertical_sd`; satellites, DOP and UTC are `None`. Position: the
  firmware has no conversion yet, so a local tangent plane at the env origin (WGS 84 meridian and prime-vertical radii at
  the origin latitude, longitude wrapped at 180 deg), rounded to 1e-7 deg and mm; `ned_from_geo` is the inverse in seams
  10's form. `vel_ned_mm_s` (north, east, down in mm/s, down positive down) is the same truth velocity plus white noise
  `gnss.velocity_sd` on each component, drawn from its own stream (`STREAM_GNSS_VEL` 6), so it is independent of the speed
  and course (which keep their own noise) and the position, speed and course samples are bit-identical with or without it;
  `[scenario] gnss_velocity = false` leaves it `None`. Rate 1, 5 or 10 Hz by `[scenario] gnss_rate_hz` (default 5, the board's `GPS_RATE_MS` 200). Not modelled:
  the receiver's smoothing and its hold.

The sim logs none of these samples yet (no record of the GNSS position goes in a log).

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
    gnss_rate_hz = 5              # optional: 1, 5 or 10 (default 5)
    mag_expected = true           # optional: false leaves MagSample.expected None (default true)
    gnss_velocity = true          # optional: false leaves GnssReading.vel_ned_mm_s None (default true)
    baro_lost = [[46.0, 51.5]]    # optional: no barometer sample reaches the flight code in [from, to) (the model keeps running)
    [initial]                     # position, velocity, roll_pitch_yaw_deg (Z-Y-X), rates, rotors = "hover" | "stopped"
    ...
    [rc]
    script = [ { t_s = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0, throttle = 0.0, arm = false }, ... ]
                                  # a row may add mode = "low" (Angle, the default) | "middle" (Altitude) | "high" (Position, capped to Altitude): the channel-6 switch
    lost = [ { from_s = 3.0, to_s = 5.0 } ]          # optional: no report in [from, to)
    [[disturbance]]                                  # optional: body torque from from_s (to to_s if given)
    from_s = 5.0
    torque_nm = [0.046, 0.0, 0.0]
    [verdict]
    check = "hover"                                  # "hover" | "steps" | "stick_sign" | "rc_loss" | "alt_hold"
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
- `alt-hold.toml` (Amendment F, F9; run with `--estimator eskf2 --preroll 2.0`, the sim's flight step on the GPS-less
  ESKF2): the same air start, mode middle from the throttle-0.5 row (0.04 s; F9 had 3.0 s, the lead's ruling of 2026-11-21 moved it: a 3 s Angle preamble at kF x0.7 is a free fall), the hover roll torque from 12 s, a climb stick (throttle 0.8) at
  23-24 s, a descent stick (0.2) at 34-36 s, and `baro_lost = [[46.0, 51.5]]`; 56 s. Judged over [5.05, 23] (5 s after entry),
  [29, 34], [41, 46] s: truth height against the target (SPN.PD) <= 0.5 m; estimate against truth with the barometer model's own
  drift taken out <= 0.3 m (the raw figure is printed, not judged; the owner's ruling on a2); the effective mode never
  Altitude on a step whose velocity_d or position_d flag is false (I6); the mode goes Angle -> Altitude on the first step after the 0.04 s row,
  Altitude -> Angle within one step of the flags dropping (`nav_aid_timeout_us` after the last barometer aid) and back
  to Altitude within one step of their return; every logged number finite; armed at the end. Printed, not judged: the
  climb and descent heights, the truth height excursion in the Angle fallback, the thrust over weight around each mode
  change, and a table every 2 s (with `--only`). It is in the spread (25 cells) and has no meaning on `--estimator mahony`
  (no height: the mode stays Angle and the mode checks fail). Negative controls, not in the spread:
  `scenarios/negative/alt-hold-switch-low.toml` (the switch never moved) and `marv-run --tuning-override psc_velz_i=0`
  (also `psc_velz_p`, `psc_posz_p`); the verdict's I6 and mode-sequence checks are also shown to fail on synthetic traces
  (`marv-sim/tests/althold_verdict.rs`).
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

`--estimator mahony|eskf1|eskf2|eskf3` puts that estimator in the flight step (default Mahony; eskf1 = `Aiding::Imu`, eskf2 =
`Aiding::ImuBaro`, built directly with `Eskf::with_aiding`; the Eskf's own rates, PT1 minus bias, are what the rate loop
gets) and adds one `steer <estimator>` line per run (per cell in a spread): tilt error against truth at the same step
(rms and max, first 5 s after the estimator's tilt is valid, and after), the largest |gyro bias| estimate (deg/s),
`motor_dlevel_rms` (rms of the step-to-step change of each motor's DShot level, pooled over the four motors and over
consecutive steps that both spin), the Eskf's re-initialisations, and `takeoff_rms_deg`/`takeoff_max_deg`/`takeoff_n`
(tilt error over the valid steps from the first armed step to 3 s after the first lift-off; n 0 without a lift-off).
With a pilot scenario each backend touchdown adds a `TD` note: truth state before the hold and the tilt error over the 2 s
before it. `--noaid-sigma <m>` replaces `Tuning::nav_noaid_sigma_m` (the armed no-aid position hold's sigma) for every
estimator. Without `--estimator` nothing changes and no `steer` line is printed.

`--estimator eskf3` (Amendments G and G-1, packet P-G4) puts `SteerMix` in the flight step: an `EskfSteer` around its own
`Eskf::new` core for attitude, rates and the horizontal state, an `EskfZ` (`Aiding::ImuBaro`) for the vertical, both in
leaked storage. Its `steer` line gains, after a `|`: the output predictor's tracking error at each correction (attitude,
velocity, position: rms and max), `skipped`, `ring_dropped`, the core's `yaw_aligns`, `yaw_align_s`, `yaw_resets`,
`gps_pos_fused`, `reinits`, whether the G6-B latch fired (`latch`, `latch_s`: the first NKI Src 8, Ok 0), the tilt error
against truth over the run and in the 5 s after the first yaw alignment (`align_airborne`: after the first lift-off),
the heading error against truth while `heading` is valid, and the horizontal position error against truth at current
time while `position_ne` is valid (the filter's origin removed), with the times the NE flags changed. `--window
<from_s>,<to_s>` (repeatable, scenario time) adds one `window` line per interval: tilt rms/max, the truth tilt (lean),
heading and horizontal error. Scenario fields for the diagnostic runs under `scenarios/fault/` (all off by default;
those scenarios are not in the spread): `gnss_lost = [[from_s, to_s]]` (no GNSS sample reaches the flight code),
`gnss_hold = [[from_s, to_s]]` (the samples repeat the last position before the interval and report zero speed and
velocity; after it the model's true solution returns with the jump), `mag_fault = { from_s, kind = "rotate" | "stuck",
deg, below_m, axis }` (`rotate`: the calibrated field vector turned about body `axis`, z by default, by `deg`; a
negative `from_s` reaches into the pre-roll; `below_m`: only while the truth height above the start is below it.
`stuck`: the field of the first sample at or after `from_s` is repeated while the sample's time and sequence advance;
a rotation about z keeps the field's strength and inclination, so it passes the compass health test, a rotation about x
or y does not), `gnss_latency_s` (replaces the GNSS model's 0.43 s).

`--preroll <seconds>` (decision-nav-eskf D5; air-start scenarios with a script only: a ground start or a pilot scenario has
its own ground phase and is a usage error, as is a non-zero initial velocity or rate) steps the flight code (and the
shadow, if any) for that long before scenario time 0, with the plant pinned at the scenario's initial state: attitude and
position as given, velocity and rates 0, rotors at their initial speed whatever the command (the plant is not stepped at
all), so the sensor models see a vehicle at rest there, noise streams running. RC is the script's t = 0 row (switch
off), so the gate cannot arm. The scenario then runs as without it: scripts, disturbances, verdict windows and the log's
time axis are scenario time. The pre-roll's own rows are not written to the log (a MSG record says so); the flight
code's clock is scenario time + the pre-roll, so it never goes backwards, and the `steer` line counts only scenario
rows. May be claimed: "ESKF2 steering, after T_pre disarmed at the scenario's initial attitude, passes hover, steps,
stick-sign and rc-loss across the spread." NOT claimed: air-start or arming without a ground alignment (the gate refuses
those by design); alignment and the take-off transition are verified only by the take-off matrix. Without `--preroll`
nothing changes.

## The parameter spread and the run matrix

    cargo run -q --release -p marv-sim --bin marv-run -- --card marv-card/marv.toml --env env/default.toml \
        --spread scenarios/spread.toml --seed 1 --out-dir logs/spread

Runs the five scenarios (the four, and `alt-hold`) at the five points of `spread.toml` (nominal; k_F x0.7 or x1.3 with the inertia x1.25 or
x0.75, each with both motor time constants 0.060 or 0.030 s). Only the plant and the backend take a point; the flight
code keeps the compiled card.

A spread file is a matrix of cells: `scenarios`, `points` (required) and the axes `maps`, `air`, `ground`, `seeds`. A
missing axis has one default value that changes nothing; a missing `seeds` means `--seed` is the one seed (and
`--seed` is a usage error, exit 2, when the file has `seeds`). `scenarios/takeoff-land-matrix.toml` is `takeoff-land` at
5 points x 2 maps x 3 air x 3 ground x 3 seeds = 270 cells (decision-sim-takeoff-landing, section 4):

| axis | key | values |
|---|---|---|
| map | `speed_at_level_min` (plant only) | A: card (94.8); B: 128.9 rad/s |
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
all 25 firmware record types), followed by the FMTs of the sim-only records. It holds the firmware's records `MSG` (at
time 0: sim version, firmware commit with `-dirty` if the checkout has uncommitted changes, card hash, scenario name,
seed, and the spread point if one is used; a spread or matrix cell adds `cell`, `map`, `air` and `ground` lines with
its name and axis values), `IMU` (instance 0 and 1: exactly what the flight step was given), `RCS` (the firmware's
`encode_rcs`: one record for each stick report that reaches the flight step, with its own time, the sticks and the
switch; none for a lost report) and, for every flight step, `NAV`, `SETP`, `DMD` and `MOT` (written by the firmware's
`encode_step`), `MODE` (the selected and the effective mode: at the first logged step and at every change) and `SPN` (the altitude target, with the step block's cadence, only while the setpoint is Altitude), both as the board writes them; and two sim-only records: `TRU` (truth: position, velocity, attitude quaternion, body rates) and
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

## Replay a flown log

`marv-replay` feeds a flight log written by the board (a `.BIN`, self-describing by its FMT records) through the
flight code's own estimators (`Mahony`, `MahonyBaro`, `Eskf`, all `Tuning::DEFAULT`; one line each in `table()` of
`marv-sim/src/bin/marv-replay.rs`) and checks them against the log's own NAV and NKF1 records. The log reader and the
rebuilding of the `Sensors` stream are in `marv-sim/src/replay.rs`.

    cargo run -q --release -p marv-sim --bin marv-replay -- <log.BIN> <outdir> [--every N] [--imu-period-us US] \
        [--mag-expected UT,INC_DEG,STR_PCT,TOL_DEG]

One step per logged IMU1 sample; the board logs only every 8th sample while disarmed, so in those gaps the last logged
sample is repeated at the nominal IMU period (the real and filled step counts are printed). The armed state is
`MOT.Arm`; the compass's expected field comes from the log's `PARM` records (the `--mag-expected` option is only for
a log without `MAG_CAL_TYPE`). GNSS epochs are rebuilt from GPS, GPA and GPV (GPA keeps only sqrt(lat^2+lon^2), which
is split equally into the latitude and longitude deviation). It writes one CSV per estimator (a row every N-th step,
default 8), prints the final `EskfStats` and a validation table per armed interval. A log holds GPS coordinates: write
the CSVs outside the repo and do not commit them.

## Check the plant

    cargo test -p marv-sim

Four tests: the plant invariants (free fall, hover thrust equals weight, yaw torque sign matches rotor spin) and
`ground_contact` (at rest the IMU reads gravity only; release on the first tick the net force points up; a touchdown
records the state before the hold). Nine in `tests/slow_sensors.rs`: each slow-sensor model against its definition over
long seeded runs (noise sd, Gauss-Markov sigma and time constant, drift size, latency, rates, course hold), the compass's
perfect-calibration identity, the GNSS position round trip and the 180 deg wrap, and every sample reaching a flight step
exactly once.

## The shadow estimator

`--shadow mahony-baro` (either form) runs `fw_flight::mahony_baro::MahonyBaro`, the phase-1 steering estimator
(Mahony plus the vertical filter `VertKf`, decision-nav-eskf-20261120 Amendment B, B1), in the runner's shadow slot
(decision-sim-seams 9): after each flight step it gets the same `Sensors` and the gate's armed state the step left.
The flight step still runs `Mahony`; the log, the verdict and every other output line are the same with and without
the flag. It adds one line per run (in a spread, after the cell's line), `shadow mahony-baro` then `key value` pairs:
`init_s` (VertKf's initialisation), then, over the steps from 5 s after it (`judged`), the rms and largest error of
D against truth (`d_rms`, `d_max`, m; truth down is taken relative to its value at the initialisation, when the
vehicle is still in every scenario) and of vD (`v_rms`, `v_max`, m/s), `v_3sigma` (steps with |vD error| above 3 of
VertKf's sigmas), the final `bd` (earth-down accelerometer bias, m/s^2), the barometer samples `gated` out and
`fused`, `resets`, `flags_lost` (judged steps without velocity_d or position_d), `identity_breaks` (steps whose
attitude, rates or tilt/rates flags differ from the flight step's Mahony, bit for bit: invariant I9), and the largest
errors and 3-sigma count before the first touchdown after lift-off (`*_pre_td`). Measurement columns for the owner
(they judge nothing): `d_max_nodrift` (|D error| with the barometer model's drift since the initialisation taken out),
`air_judged` and `v_3sigma_air` (judged airborne steps, airborne meaning `Truth::on_ground` false at every 125 us tick
since the previous step, and their 3-sigma count), `air_v3_first_s`/`air_v3_last_s` (time of those since the last
lift-off), `air_v3_ratio_max` (largest |vD error|/sigma), `air_v3_within_half_s`, `air_v3_within_2s`, `liftoffs`, and
at the first touchdown after lift-off `td_s`, the D error there (`d_td`, `d_td_nodrift`) and 2 s later (`d_td2`,
`d_td2_nodrift`), and `v_3sigma_pre_td_judged` (the judged steps before the first touchdown, the window of `v_3sigma_pre_td`).

Test a2 judges two columns, by the owner's ruling of 2026-11-21 (decision-nav-eskf-20261120, "Owner's ruling on how test
a2 of Amendment B is judged"). Height: `d_max_nodrift` against 0.3 m, because the barometer-only filter cannot see its own
sensor's drift; the raw `d_max` is printed, not judged. Vertical speed: `v_3sigma_pre_td` must be 0, over the steps from 5 s
after the initialisation until the first touchdown (the first tick with ground contact after the vehicle has been
airborne; a run that never lifts off or never touches down is judged over the whole window), because the plant stops the
vehicle in one tick at touchdown without the accelerometer seeing it; the raw `v_3sigma` is printed, not judged.

`--shadow eskf` (either form) runs `fw_flight::eskf::Eskf`, the 15-state shadow filter (decision-nav-eskf-20261120
Amendments C and C-1, packet E1b), in the same slot, with the declination taken from the env's field. Its state refers
to the horizon (`NavState.t_us`, 0.43 s behind the step), so it is compared with the truth of that step. One line per
run, `shadow eskf` then `key value` pairs: heading while `heading` is valid (`hdg_valid_s`, `hdg_bad15_s` seconds
valid with |error| > 15 deg, `hdg_first_s`, `hdg_worst_deg`); north-east while `position_ne` is valid, the offset
between the filter's origin and the env's removed (`ne_valid_s`, `ne_out3_s` seconds with a position or velocity axis
outside 3 of the filter's own sigma, `ne_p_rms`, `ne_p_max`, `ne_v_max`); the vertical as `mahony-baro`'s line
(`init_s`, `judged`, `d_rms`, `d_max`, `d_max_nodrift`, `v_rms`, `v_max`, `v_3sigma`, from 5 s after `position_d` is
first valid, and the a2 rule's `v_3sigma_pre_td` with `v_3sigma_pre_td_judged`, up to the first touchdown at the
horizon); tilt against the truth of the horizon step beside Mahony's on the same steps (`tilt_rms_deg`,
`tilt_max_deg`, `mahony_tilt_rms_deg`, `mahony_tilt_max_deg`), split at 5 s after the tilt alignment
(`tilt_rms_first5_deg`, `mahony_tilt_rms_first5_deg`, `tilt_rms_after5_deg`, `mahony_tilt_rms_after5_deg`), and the
Eskf against the truth of the current step for contrast (`tilt_rms_vs_current_truth_deg`); counts (`mag_refused`, `screens`, `gated`, `resets_height`,
`resets_yaw`, `resets_gps`, `reinits`), `yaw_aligned` at the end and `multi_job` (whether any step did more than one
heavy job). Tests in `tests/nav_eskf.rs`: c1 (heading never valid while more than 15 deg wrong; negative control a
compass turned 30 deg), c2 (log 186's disturbed take-off spot gives no alignment and no GNSS before take-off; negative
control the health test opened), c4 (north-east within 3 sigma with the GNSS latency 0.43 s; negative control delay 0),
judged by the owner's ruling of 2026-11-21 (decision-nav-eskf-20261120, "Owner's answers, 2026-11-21 (after the session
break)") against truth plus the simulated GNSS's own slow (Gauss-Markov) error at the measurement's time, because the
filter cannot see its sensor's slow error (as with the barometer's drift in a2); the count against plain truth is printed,
not judged, and the delay-0 run must have a judged count above 0, on a synthetic flight through the sensor models, and b2/c1/b3 over the
suite through `--shadow eskf` (height with the drift removed <= 0.3 m, no heading valid while wrong, one heavy job per
step; tilt printed only).

## Not built yet

Wind gusts (phase 2), the logging of the slow samples, the altitude and position
holds (step 7), the TCP link and the Gazebo backend (steps 8 and 9).

## Check the card

    cargo run -q -p marv-card --bin card-check -- marv-card/marv.toml env/default.toml

It prints `PASS` and exits 0 when every number has a tag and the card is consistent, then lists the open items: every
placeholder and every value or range that is still `"ask"`. Any error exits 1.

## Change the card

- The block between `# >>> BEGIN GENERATED` and `# <<< END GENERATED` comes from the CAD export. Do not edit it; the
  check notices. Regenerate it from the CAD repo, after committing `model/mass_properties.json` there:

      cd ~/Projects/CAD/drone && python3 tools/make_card.py ~/Documents/MARV-sim/marv-card/marv.toml

- Everything else is edited by hand. Give each number its tag and say in `src` where it came from. A `placeholder`
  needs a range, or `range = "ask"`. A `proposed` number has been put to the owner and is not yet approved; card-check
  lists it (today the three GNSS `slow_*` numbers).

## Tests

    cargo test --workspace

Eight tests in `marv-card` (one card that passes, one broken card per check, the sensor fields, the home override)
and the seventeen of `marv-sim`. Four of them, `tests/nav_vertical.rs`, are MahonyBaro's B4 tests on the sensor
models: a1 (bit identity with Mahony, with its negative control), a3 (an accelerometer bias at the card's range bound
is estimated; negative control: bias frozen), a4 (200 ms barometer latency compensated by the delay term; negative
control: delay 0), and a2, which runs the scenario suite and both spread files through `marv-run --shadow
mahony-baro` and requires |D error| <= 0.3 m, no vD error above 3 sigma and no identity break in every run (run it
with `--release`: it runs 295 flights). a2 FAILS today in `takeoff-land` (the barometer model's drift random walk,
0.77 m over 14 min, reaches 0.29 m within one run, and the touchdown hold zeroes the velocity in one tick, which no
IMU sample shows).
