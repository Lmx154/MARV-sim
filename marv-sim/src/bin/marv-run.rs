//! Runs the real firmware flight step against the plant, in a loop, writes a DataFlash log and judges the run.
//! Tick table: conventions 6. The loop is the one in decision-sim-seams 8, with RC and the ESC hold rule of
//! decision-first-pair-wiring B10; the verdict is B11; the spread is B12 with the five points of the rulings' ruling 4;
//! the matrix of cells (decision-sim-takeoff-landing, section 4) is the spread file with more axes.

use fw_core::handshake::Handshake;
use fw_core::rc::{RcSample, Sticks};
use fw_flight::alloc::CardAllocator;
use fw_flight::esc::hold;
use fw_flight::gate::MotorCommand;
use fw_flight::gnc::{Allocator, Controller, Estimator, Sensors, Setpoint};
use fw_flight::mahony::Mahony;
use fw_flight::pid::CascadedPid;
use fw_flight::step::{Flight, FlightInputs, StepOut};
use fw_flight::tuning::Tuning;
use marv_card::{Card, Env};
use marv_sim::log::Log;
use marv_sim::pilot::{Ending, Pilot, PilotView};
use marv_sim::rng::Rng;
use marv_sim::scenario::{Air, Cell, Ground, Map, Point, RcRow, Rotors, Scenario, Spread};
use marv_sim::sensors::{IMU_PERIOD_US, ImuModel};
use marv_sim::verdict::{Check, Sample, Trace, judge};
use marv_sim::{Backend, NativeBackend, Plant, Reaction, Start, Truth};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Instant;

const TICK_US: u64 = 125;
/// The ESC exchange: every 8 ticks, 1 kHz.
const ESC_TICKS: u64 = 8;
/// A stick report: every 32 ticks, 4 ms (B10).
const RC_TICKS: u64 = 32;
/// Noise streams, one per sensor (src/rng.rs).
const STREAM_IMU1: u64 = 1;
const STREAM_IMU2: u64 = 2;

const USAGE: &str = "usage: marv-run --card <marv.toml> --env <env.toml> --seed <u64> --scenario <file> --out <file.BIN> \
[--point <kf_scale>,<inertia_scale>,<lag_s or ->]
       marv-run --card <marv.toml> --env <env.toml> [--seed <u64>] --spread <file> --out-dir <dir> \
[--only <cell>] [--logs fail|all]
(--seed is given exactly when the spread file has no seeds axis)";

struct Args {
    card: PathBuf,
    env: PathBuf,
    seed: Option<u64>,
    job: Job,
}

/// Which logs a spread keeps: `Fail` only those of cells with a failing check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Logs {
    Fail,
    All,
}

enum Job {
    One { scenario: PathBuf, out: PathBuf, point: Option<Point> },
    Spread { spread: PathBuf, out_dir: PathBuf, only: Option<String>, logs: Logs },
}

/// `Ok(None)` means --help.
fn parse_args(args: &[String]) -> Result<Option<Args>, String> {
    let (mut card, mut env, mut scenario, mut seed, mut out) = (None, None, None, None, None);
    let (mut point, mut spread, mut out_dir) = (None, None, None);
    let (mut only, mut logs) = (None, None);
    let mut it = args.iter();
    while let Some(flag) = it.next() {
        if flag == "--help" || flag == "-h" {
            return Ok(None);
        }
        let value = it.next().ok_or(format!("{flag}: needs a value"))?;
        match flag.as_str() {
            "--card" => card = Some(PathBuf::from(value)),
            "--env" => env = Some(PathBuf::from(value)),
            "--scenario" => scenario = Some(PathBuf::from(value)),
            "--seed" => seed = Some(value.parse::<u64>().map_err(|_| format!("--seed: {value} is not a u64"))?),
            "--out" => out = Some(PathBuf::from(value)),
            "--point" => point = Some(Point::parse(value)?),
            "--spread" => spread = Some(PathBuf::from(value)),
            "--out-dir" => out_dir = Some(PathBuf::from(value)),
            "--only" => only = Some(value.clone()),
            "--logs" => {
                logs = Some(match value.as_str() {
                    "fail" => Logs::Fail,
                    "all" => Logs::All,
                    _ => return Err(format!("--logs: {value} is not fail or all")),
                })
            }
            _ => return Err(format!("unknown argument {flag}")),
        }
    }
    let job = match (scenario, out, spread, out_dir) {
        (Some(scenario), Some(out), None, None) if only.is_none() && logs.is_none() => Job::One { scenario, out, point },
        (None, None, Some(spread), Some(out_dir)) if point.is_none() => {
            Job::Spread { spread, out_dir, only, logs: logs.unwrap_or(Logs::Fail) }
        }
        _ => {
            return Err("give either --scenario and --out (and --point if wanted), or --spread and --out-dir \
(and --only, --logs if wanted)"
                .into());
        }
    };
    if matches!(job, Job::One { .. }) && seed.is_none() {
        return Err("--seed is required with --scenario".into());
    }
    Ok(Some(Args {
        card: card.ok_or("--card is required")?,
        env: env.ok_or("--env is required")?,
        seed,
        job,
    }))
}

/// The speed at which four rotors carry the weight.
fn hover_speed(card: &Card, env: &Env) -> Result<f64, String> {
    let mass = card.mass.mass.v.ok_or("mass.mass is \"ask\"")?;
    let g = env.gravity.v.ok_or("gravity.g is \"ask\"")?;
    let k = card.propulsion.thrust_coefficient.v.ok_or("propulsion.thrust_coefficient is \"ask\"")?;
    Ok((mass * g / (4.0 * k)).sqrt())
}

/// The card the plant and the backend read at a spread point: k_F and the inertia tensor scaled, both motor time
/// constants set to the lag; the cell's map shape and rotor drag. The flight code keeps the compiled card.
fn plant_card(card: &Card, point: Option<Point>, map: &Map, air: &Air) -> Card {
    let mut c = card.clone();
    if let Some(p) = point {
        let prop = &mut c.propulsion;
        prop.thrust_coefficient.v = prop.thrust_coefficient.v.map(|k| k * p.kf_scale);
        c.mass.inertia.v = c.mass.inertia.v.map(|j| j.map(|row| row.map(|x| x * p.inertia_scale)));
        if let Some(lag) = p.lag_s {
            prop.time_constant_up.v = Some(lag);
            prop.time_constant_down.v = Some(lag);
        }
    }
    if let Some(w) = map.speed_at_level_min {
        c.propulsion.speed_at_level_min.v = Some(w);
    }
    if let Some(d) = air.drag_rotor_linear {
        c.drag_rotor_linear.v = Some(d);
    }
    c
}

/// The simulated world: what the flight step is run against.
struct World {
    plant: Plant,
    backend: NativeBackend,
    imu1: ImuModel,
    imu2: ImuModel,
    wind_ned_ms: [f64; 3],
}

/// The script row in force at `t_us` (the first row is at 0).
fn row_at(script: &[RcRow], t_us: u64) -> RcRow {
    *script.iter().rfind(|r| r.t_us <= t_us).unwrap_or(&script[0])
}

/// What the verdict reads of one flight step.
fn sample(truth: &Truth, out: &StepOut) -> Sample {
    let (sp_q, sp_heading_rate) = match out.setpoint {
        Setpoint::Attitude { attitude, heading_rate_rads, .. } => (attitude.map(f64::from), f64::from(heading_rate_rads)),
        _ => ([1.0, 0.0, 0.0, 0.0], 0.0),
    };
    // The same bits as the firmware's DMD record (step.rs, encode_step).
    let flags = out.allocation.map(|a| {
        let s = a.saturation;
        u8::from(s.roll) | u8::from(s.pitch) << 1 | u8::from(s.yaw) << 2 | u8::from(s.thrust_low) << 3
            | u8::from(s.thrust_high) << 4
    });
    Sample {
        t_us: truth.t_us,
        truth_q: truth.attitude,
        truth_rates: truth.rates_rads,
        truth_accel: truth.accel_ms2,
        nav_q: out.nav.attitude.map(f64::from),
        sp_q,
        sp_heading_rate,
        armed: out.armed,
        flags,
        truth_pos: truth.position_m,
        truth_vel: truth.velocity_ms,
        on_ground: truth.on_ground,
        reaction: truth.reaction,
    }
}

/// Runs at most `ticks` base ticks (with a pilot, until its phase table ends); returns how many ticks and flight steps
/// it made and what the verdict needs.
fn run<E: Estimator, C: Controller, A: Allocator>(
    mut flight: Flight<E, C, A>,
    world: &mut World,
    scenario: &Scenario,
    mut pilot: Option<&mut Pilot>,
    ticks: u64,
    log: &mut Log,
) -> (u64, u64, Trace) {
    // The newest StepOut's time and command, as the ESC task sees them; none yet counts as (0, Stop).
    let mut newest = (0, MotorCommand::STOP);
    let mut levels = [0u16; 4];
    let mut pending: Option<RcSample> = None;
    let none = RcRow { t_us: 0, roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: 0.0, arm: false };
    let mut given = scenario.rc_script.first().copied().unwrap_or(none);
    let mut trace = Trace::default();
    let mut flight_steps = 0;
    let mut ran = 0;
    for k in 0..ticks {
        let now = world.backend.truth().t_us;
        if k.is_multiple_of(ESC_TICKS) {
            levels = hold(newest.0, newest.1, now).throttles().map_or([0; 4], |t| t.map(|x| x.level()));
            trace.esc.push((now, levels));
        }
        if k.is_multiple_of(RC_TICKS) {
            // The script row in force, or the pilot's report from what it sees (it flies on through a lost link).
            let row = match pilot.as_deref_mut() {
                None => row_at(&scenario.rc_script, now),
                Some(p) => {
                    let Some(report) = p.report(now, PilotView::see(world.backend.truth())) else { break };
                    log.pilot(&report);
                    trace.pilot.push(report);
                    report.row
                }
            };
            // Only the newest report reaches the next flight step, once.
            let lost = scenario.rc_lost.iter().any(|&(from, to)| now >= from && now < to);
            if !lost {
                given = row;
                let sticks = Sticks { roll: given.roll, pitch: given.pitch, throttle: given.throttle, yaw: given.yaw };
                pending = Some(RcSample { sticks, arm: given.arm, t_us: now });
                trace.rc_us.push(now);
            }
            log.rc(now, [given.roll, given.pitch, given.throttle, given.yaw], given.arm, lost);
        }
        let mut wrench = world.plant.step(levels, world.backend.truth(), world.wind_ned_ms, TICK_US);
        for d in scenario.disturbances.iter().filter(|d| now >= d.from_us && now < d.to_us) {
            for i in 0..3 {
                wrench.torque_nm[i] += d.torque_nm[i];
            }
        }
        let was_on_ground = world.backend.truth().on_ground;
        let truth = world.backend.step(&wrench, TICK_US).clone();
        ran += 1;
        if truth.on_ground && !was_on_ground {
            trace.touchdowns.extend(world.backend.touchdown().copied());
        } else if was_on_ground && !truth.on_ground {
            trace.liftoffs.push(truth.t_us);
        }

        if truth.t_us.is_multiple_of(IMU_PERIOD_US) {
            let s1 = world.imu1.sample(&truth);
            let s2 = world.imu2.sample(&truth);
            let sensors = Sensors { imu1: s1, imu2: Some(s2), baro: None, mag: None, gnss: None, external: None };
            let out = flight.step(&FlightInputs { sensors, rc: pending.take() });
            newest = (out.t_us, out.motors);
            flight_steps += 1;

            log.imu(truth.t_us, 0, &s1.reading);
            log.imu(truth.t_us, 1, &s2.reading);
            log.truth(&truth);
            log.rotors(truth.t_us, levels, world.plant.rotor_speeds());
            log.step(&out);
            if pilot.is_some() {
                log.ground(&truth);
            }
            trace.steps.push(sample(&truth, &out));
        }
    }
    (ran, flight_steps, trace)
}

fn firmware_text() -> String {
    let commit = env!("FW_COMMIT");
    let dirty = if env!("FW_DIRTY") == "1" { "-dirty" } else { "" };
    format!("firmware {}{dirty}", &commit[..commit.len().min(12)])
}

/// One judged run.
struct Outcome {
    summary: Vec<String>,
    checks: Vec<Check>,
    notes: Vec<String>,
}

/// The cell of a matrix run: its name and the values of its axes, for the plant, the sensors and the log.
struct Run<'a> {
    cell: &'a Cell,
    map: Map,
    air: Air,
    ground: Ground,
}

/// One run of `scenario` at `point` (None = the card as it is) and the cell's axis values; writes the log to `out`
/// when `keep` or when a check fails.
#[allow(clippy::too_many_arguments)]
fn run_one(card: &Card, env: &Env, scenario: &Scenario, seed: u64, point: Option<Point>, cell: Option<&Run>, out: &Path,
    keep: bool) -> Result<Outcome, String> {
    let (map, air, ground) = cell.map_or((Map::default(), Air::default(), Ground::default()), |r| (r.map, r.air, r.ground));
    let sloped;
    let scenario = match ground.roll_pitch_deg {
        Some(roll_pitch) => {
            sloped = scenario.clone().on_slope(roll_pitch)?;
            &sloped
        }
        None => scenario,
    };
    let plant_card = plant_card(card, point, &map, &air);
    let mut plant = Plant::new(&plant_card, env)?;
    if scenario.rotors == Rotors::Hover {
        plant.set_rotor_speeds([hover_speed(&plant_card, env)?; 4]);
    }
    let initial = Truth {
        t_us: 0,
        position_m: scenario.position_ned_m,
        velocity_ms: scenario.velocity_ned_ms,
        attitude: scenario.attitude,
        rates_rads: scenario.rates_rads,
        accel_ms2: [0.0; 3],
        ang_accel_rads2: [0.0; 3],
        on_ground: false,
        reaction: Reaction::default(),
    };
    let backend = NativeBackend::with_start(&plant_card, initial, scenario.start)?;
    // The cell's fixed IMU1 bias replaces the card's vector; IMU2 and the flight code are untouched.
    let mut imu1_profile = card.imu1.clone();
    if let Some(b) = ground.imu1_gyro_bias_rads {
        imu1_profile.gyro_bias.v = Some(b);
    }
    if let Some(b) = ground.imu1_accel_bias_ms2 {
        imu1_profile.accel_bias.v = Some(b);
    }
    let imu1 = ImuModel::new("imu1", &imu1_profile, env, Rng::new(seed, STREAM_IMU1))?;
    let imu2 = ImuModel::new("imu2", &card.imu2, env, Rng::new(seed, STREAM_IMU2))?;
    // Wind for this step is the mean only; gusts come in step 7.
    let wind = match air.wind_ned_ms {
        Some(w) => w,
        None => env.wind_mean_ned.v.ok_or("wind.mean_ned is \"ask\"")?,
    };

    let mut log = Log::with_header();
    log.msg(0, &format!("marv-sim {}", env!("CARGO_PKG_VERSION")));
    log.msg(0, &firmware_text());
    log.msg(0, &format!("card sha256 {}", &card.sha256[..16]));
    log.msg(0, &format!("scenario {}", scenario.name));
    log.msg(0, &format!("seed {}", seed));
    if let Some(p) = point {
        log.msg(0, &format!("point {}", p.text()));
    }
    if let Some(r) = cell {
        log.msg(0, &format!("cell {}", r.cell.name));
        log.msg(0, &format!("map {} {}", (b'A' + r.cell.map as u8) as char, map.text()));
        log.msg(0, &format!("air {} {}", r.cell.air + 1, air.text()));
        let parts = ground.parts();
        if parts.is_empty() {
            log.msg(0, &format!("ground {} card", r.cell.ground + 1));
        }
        for part in parts {
            log.msg(0, &format!("ground {} {part}", r.cell.ground + 1));
        }
    }
    // The pilot's idea of the stick map comes from the card as it is, never from a spread point.
    let mut pilot = match scenario.pilot {
        Some(p) => Some(Pilot::new(card, env, p, scenario.phases.clone())?),
        None => None,
    };
    if pilot.is_some() {
        log.declare_phase_records();
    }

    let ticks = (scenario.duration_s * 1e6 / TICK_US as f64).round() as u64;
    // The flight code with the compiled card and the default tuning, as on the board.
    let handshake: &'static Handshake = Box::leak(Box::new(Handshake::new()));
    let tuning = Tuning::DEFAULT;
    let flight = Flight::new(Mahony::new(&tuning), CascadedPid::new(&tuning), CardAllocator::new(), &tuning, handshake);
    let mut world = World { plant, backend, imu1, imu2, wind_ned_ms: wind };
    let (ran, flight_steps, mut trace) = run(flight, &mut world, scenario, pilot.as_mut(), ticks, &mut log);

    if let Some(p) = &pilot {
        trace.phases = p.timeline().to_vec();
        trace.weight_n = card.mass.mass.v.unwrap_or(f64::NAN) * env.gravity.v.unwrap_or(f64::NAN);
        // The nominal cell changes nothing of the card, the environment or the scenario's ground.
        let same = |set: Option<f64>, card: Option<f64>| set.is_none_or(|x| Some(x) == card);
        let same3 = |set: Option<[f64; 3]>, card: Option<[f64; 3]>| set.is_none_or(|x| Some(x) == card);
        trace.nominal = point.is_none_or(|q| q.kf_scale == 1.0 && q.inertia_scale == 1.0 && q.lag_s.is_none())
            && same(map.speed_at_level_min, card.propulsion.speed_at_level_min.v)
            && same(air.drag_rotor_linear, card.drag_rotor_linear.v)
            && same3(air.wind_ned_ms, env.wind_mean_ned.v)
            && same3(ground.imu1_gyro_bias_rads, card.imu1.gyro_bias.v)
            && same3(ground.imu1_accel_bias_ms2, card.imu1.accel_bias.v)
            && ground.roll_pitch_deg.is_none_or(|rp| rp == [0.0, 0.0]);
    }
    let (checks, notes) = judge(&scenario.verdict, &trace);
    let kept = keep || checks.iter().any(|c| !c.pass);
    if kept {
        log.write(out)?;
    }

    let last = world.backend.truth();
    let c = log.counts;
    let mut summary = vec![
        format!("scenario      {}", scenario.name),
        format!("point         {}", point.map_or("card".to_string(), |p| p.text())),
        format!("duration      {} s", scenario.duration_s),
        format!("ticks         {} (125 us each)", ran),
        format!("flight steps  {}", flight_steps),
        format!("records       MSG {}, IMU {}, TRU {}, ROT {}, NAV/SETP/DMD/MOT {} each, RCS {}",
            c.msg, c.imu, c.tru, c.rot, c.step, c.rcs),
        format!("final position NED {:.4?} m", last.position_m),
        format!("final velocity NED {:.4?} m/s", last.velocity_ms),
        if kept { format!("log           {}", out.display()) } else { "log           not kept (every check passed)".to_string() },
    ];
    if let Some(p) = &pilot {
        summary.push(format!("records       GND {}, PIL {}", c.gnd, c.pil));
        summary.push(format!("pilot         full collective / m g = {:.4} (nominal card)", p.full_ratio()));
        for (i, ph) in p.timeline().iter().enumerate() {
            let how = match ph.end {
                None => "not ended (duration_s reached)".to_string(),
                Some((t, Ending::Time)) => format!("ended {:.3} s by time", t as f64 * 1e-6),
                Some((t, Ending::Event)) => format!("ended {:.3} s by event", t as f64 * 1e-6),
                Some((t, Ending::Timeout)) => format!("ended {:.3} s by TIMEOUT", t as f64 * 1e-6),
            };
            summary.push(format!("PHASE {i:>2} {:<8} start {:>7.3} s  {how}", ph.name, ph.start_us as f64 * 1e-6));
        }
    }
    Ok(Outcome { summary, checks, notes })
}

/// What `--spread` refuses before any run: a value outside a card range, or a slope on a scenario in the air. The
/// first cell of each distinct fault is named.
fn refusals(spread: &Spread, scenarios: &[Scenario], cells: &[Cell], card: &Card, env: &Env) -> Vec<String> {
    let mut seen: Vec<String> = Vec::new();
    let mut out = Vec::new();
    for cell in cells {
        let mut faults = spread.range_errors(cell, card, env);
        if spread.ground[cell.ground].roll_pitch_deg.is_some() && !matches!(scenarios[cell.scenario].start, Start::Ground { .. }) {
            faults.push(format!("ground slope: scenario {} does not start on the ground", scenarios[cell.scenario].name));
        }
        for f in faults {
            if !seen.contains(&f) {
                out.push(format!("cell {}: {f}", cell.name));
                seen.push(f);
            }
        }
    }
    out
}

/// Why a run did not finish: a usage error (exit 2) or any other error (exit 1).
enum Failure {
    Usage(String),
    Error(String),
}

impl From<String> for Failure {
    fn from(e: String) -> Failure {
        Failure::Error(e)
    }
}

impl From<&str> for Failure {
    fn from(e: &str) -> Failure {
        Failure::Error(e.to_string())
    }
}

/// `Ok(true)` when every check passed.
fn go(args: &Args) -> Result<bool, Failure> {
    let join = |errs: Vec<String>| errs.join("\n");
    let card = Card::load(&args.card).map_err(join)?;
    // The constants compiled into the flight code and the card the plant reads must be the same file.
    if marv_card::CARD_SHA256 != card.sha256 {
        return Err(format!(
            "card {} (sha256 {}) is not the card compiled into the firmware constants (sha256 {}); rebuild",
            args.card.display(), card.sha256, marv_card::CARD_SHA256
        )
        .into());
    }
    let env = Env::load(&args.env).map_err(join)?;
    match &args.job {
        Job::One { scenario, out, point } => {
            let seed = args.seed.ok_or(Failure::Usage("--seed is required with --scenario".to_string()))?;
            let scenario = Scenario::load(scenario)?;
            let o = run_one(&card, &env, &scenario, seed, *point, None, out, true)?;
            for line in o.summary.iter().chain(&o.notes) {
                println!("{line}");
            }
            for c in &o.checks {
                println!("CHECK {} {} {} {}", c.name, c.measured, c.limit, if c.pass { "PASS" } else { "FAIL" });
            }
            let pass = o.checks.iter().all(|c| c.pass);
            println!("RESULT: {}", if pass { "PASS" } else { "FAIL" });
            Ok(pass)
        }
        Job::Spread { spread: spread_file, out_dir, only, logs } => {
            let started = Instant::now();
            let spread = Spread::load(spread_file)?;
            let scenarios = spread.scenarios.iter().map(Scenario::load).collect::<Result<Vec<_>, _>>()?;
            let names: Vec<String> = scenarios.iter().map(|s| s.name.clone()).collect();
            let mut cells = spread.cells(&names, args.seed).map_err(Failure::Usage)?;
            let refused = refusals(&spread, &scenarios, &cells, &card, &env);
            if !refused.is_empty() {
                return Err(format!("refused, nothing was run:\n{}", refused.join("\n")).into());
            }
            if let Some(name) = only {
                cells.retain(|c| &c.name == name);
                if cells.is_empty() {
                    return Err(format!("--only {name}: no such cell in {}", spread_file.display()).into());
                }
            }
            for (i, p) in spread.points.iter().enumerate() {
                println!("P{i}  {}", p.text());
            }
            for (i, m) in spread.maps.iter().enumerate() {
                println!("map {}  {}", (b'A' + i as u8) as char, m.text());
            }
            for (i, a) in spread.air.iter().enumerate() {
                println!("air{}  {}", i + 1, a.text());
            }
            for (i, g) in spread.ground.iter().enumerate() {
                let parts = g.parts();
                println!("gnd{}  {}", i + 1, if parts.is_empty() { "card".to_string() } else { parts.join("; ") });
            }
            let width = cells.iter().map(|c| c.name.len()).max().unwrap_or(0);
            // (check name, cells where it failed, cells where it was judged), in order of first appearance.
            let mut counts: Vec<(String, usize, usize)> = Vec::new();
            let mut passed = 0;
            for cell in &cells {
                let run = Run {
                    cell,
                    map: spread.maps[cell.map],
                    air: spread.air[cell.air],
                    ground: spread.ground[cell.ground],
                };
                let out = out_dir.join(format!("{}.BIN", cell.name));
                let keep = only.is_some() || *logs == Logs::All;
                let o = run_one(&card, &env, &scenarios[cell.scenario], cell.seed, Some(spread.points[cell.point]),
                    Some(&run), &out, keep)?;
                if only.is_some() {
                    for line in o.summary.iter().chain(&o.notes) {
                        println!("{line}");
                    }
                    for c in &o.checks {
                        println!("CHECK {} {} {} {}", c.name, c.measured, c.limit, if c.pass { "PASS" } else { "FAIL" });
                    }
                }
                let mut failing = Vec::new();
                for c in &o.checks {
                    let i = counts.iter().position(|(n, _, _)| *n == c.name).unwrap_or_else(|| {
                        counts.push((c.name.clone(), 0, 0));
                        counts.len() - 1
                    });
                    counts[i].2 += 1;
                    if !c.pass {
                        counts[i].1 += 1;
                        failing.push(c.name.as_str());
                    }
                }
                passed += usize::from(failing.is_empty());
                println!("{:<width$}  {}  {}", cell.name, if failing.is_empty() { "PASS" } else { "FAIL" }, failing.join(", "));
                let _ = std::io::stdout().flush();
            }
            println!("failures by check (cells failing / cells judged):");
            for (name, failed, judged) in &counts {
                println!("  {name:<22} {failed:>4} / {judged}");
            }
            println!("PASS cells {passed} of {}", cells.len());
            println!("wall time {:.1} s", started.elapsed().as_secs_f64());
            let all = passed == cells.len();
            println!("RESULT: {}", if all { "PASS" } else { "FAIL" });
            Ok(all)
        }
    }
}

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let args = match parse_args(&argv) {
        Ok(Some(args)) => args,
        Ok(None) => {
            println!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Err(e) => {
            eprintln!("marv-run: {e}\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    match go(&args) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(3),
        Err(Failure::Usage(e)) => {
            eprintln!("marv-run: {e}\n{USAGE}");
            ExitCode::from(2)
        }
        Err(Failure::Error(e)) => {
            eprintln!("marv-run: {e}");
            ExitCode::from(1)
        }
    }
}
