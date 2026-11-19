//! Runs the real firmware flight step against the plant, in a loop, writes a DataFlash log and judges the run.
//! Tick table: conventions 6. The loop is the one in decision-sim-seams 8, with RC and the ESC hold rule of
//! decision-first-pair-wiring B10; the verdict is B11; the spread is B12 with the five points of the rulings' ruling 4.

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
use marv_sim::rng::Rng;
use marv_sim::scenario::{Point, RcRow, Rotors, Scenario, Spread};
use marv_sim::sensors::{IMU_PERIOD_US, ImuModel};
use marv_sim::verdict::{Check, Sample, Trace, judge};
use marv_sim::{Backend, NativeBackend, Plant, Reaction, Truth};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

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
       marv-run --card <marv.toml> --env <env.toml> --seed <u64> --spread <file> --out-dir <dir>";

struct Args {
    card: PathBuf,
    env: PathBuf,
    seed: u64,
    job: Job,
}

enum Job {
    One { scenario: PathBuf, out: PathBuf, point: Option<Point> },
    Spread { spread: PathBuf, out_dir: PathBuf },
}

/// `Ok(None)` means --help.
fn parse_args(args: &[String]) -> Result<Option<Args>, String> {
    let (mut card, mut env, mut scenario, mut seed, mut out) = (None, None, None, None, None);
    let (mut point, mut spread, mut out_dir) = (None, None, None);
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
            _ => return Err(format!("unknown argument {flag}")),
        }
    }
    let job = match (scenario, out, spread, out_dir) {
        (Some(scenario), Some(out), None, None) => Job::One { scenario, out, point },
        (None, None, Some(spread), Some(out_dir)) if point.is_none() => Job::Spread { spread, out_dir },
        _ => return Err("give either --scenario and --out (and --point if wanted), or --spread and --out-dir".into()),
    };
    Ok(Some(Args {
        card: card.ok_or("--card is required")?,
        env: env.ok_or("--env is required")?,
        seed: seed.ok_or("--seed is required")?,
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
/// constants set to the lag. The flight code keeps the compiled card.
fn plant_card(card: &Card, point: Option<Point>) -> Card {
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
    Sample {
        t_us: truth.t_us,
        truth_q: truth.attitude,
        truth_rates: truth.rates_rads,
        truth_accel: truth.accel_ms2,
        nav_q: out.nav.attitude.map(f64::from),
        sp_q,
        sp_heading_rate,
    }
}

/// Runs `ticks` base ticks; returns how many flight steps it made and what the verdict needs.
fn run<E: Estimator, C: Controller, A: Allocator>(
    mut flight: Flight<E, C, A>,
    world: &mut World,
    scenario: &Scenario,
    ticks: u64,
    log: &mut Log,
) -> (u64, Trace) {
    // The newest StepOut's time and command, as the ESC task sees them; none yet counts as (0, Stop).
    let mut newest = (0, MotorCommand::STOP);
    let mut levels = [0u16; 4];
    let mut pending: Option<RcSample> = None;
    let mut given = scenario.rc_script[0];
    let mut trace = Trace::default();
    let mut flight_steps = 0;
    for k in 0..ticks {
        let now = world.backend.truth().t_us;
        if k.is_multiple_of(ESC_TICKS) {
            levels = hold(newest.0, newest.1, now).throttles().map_or([0; 4], |t| t.map(|x| x.level()));
            trace.esc.push((now, levels));
        }
        if k.is_multiple_of(RC_TICKS) {
            // Only the newest report reaches the next flight step, once.
            let lost = scenario.rc_lost.iter().any(|&(from, to)| now >= from && now < to);
            if !lost {
                given = row_at(&scenario.rc_script, now);
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
        let truth = world.backend.step(&wrench, TICK_US).clone();

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
            trace.steps.push(sample(&truth, &out));
        }
    }
    (flight_steps, trace)
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

/// One run of `scenario` at `point` (None = the card as it is); writes the log to `out`.
fn run_one(card: &Card, env: &Env, scenario: &Scenario, seed: u64, point: Option<Point>, out: &Path)
    -> Result<Outcome, String> {
    let plant_card = plant_card(card, point);
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
    let backend = NativeBackend::new(&plant_card, initial)?;
    let imu1 = ImuModel::new("imu1", &card.imu1, env, Rng::new(seed, STREAM_IMU1))?;
    let imu2 = ImuModel::new("imu2", &card.imu2, env, Rng::new(seed, STREAM_IMU2))?;
    // Wind for this step is the mean only; gusts come in step 7.
    let wind = env.wind_mean_ned.v.ok_or("wind.mean_ned is \"ask\"")?;

    let mut log = Log::with_header();
    log.msg(0, &format!("marv-sim {}", env!("CARGO_PKG_VERSION")));
    log.msg(0, &firmware_text());
    log.msg(0, &format!("card sha256 {}", &card.sha256[..16]));
    log.msg(0, &format!("scenario {}", scenario.name));
    log.msg(0, &format!("seed {}", seed));
    if let Some(p) = point {
        log.msg(0, &format!("point {}", p.text()));
    }

    let ticks = (scenario.duration_s * 1e6 / TICK_US as f64).round() as u64;
    // The flight code with the compiled card and the default tuning, as on the board.
    let handshake: &'static Handshake = Box::leak(Box::new(Handshake::new()));
    let tuning = Tuning::DEFAULT;
    let flight = Flight::new(Mahony::new(&tuning), CascadedPid::new(&tuning), CardAllocator::new(), &tuning, handshake);
    let mut world = World { plant, backend, imu1, imu2, wind_ned_ms: wind };
    let (flight_steps, trace) = run(flight, &mut world, scenario, ticks, &mut log);
    log.write(out)?;

    let last = world.backend.truth();
    let c = log.counts;
    let summary = vec![
        format!("scenario      {}", scenario.name),
        format!("point         {}", point.map_or("card".to_string(), |p| p.text())),
        format!("duration      {} s", scenario.duration_s),
        format!("ticks         {} (125 us each)", ticks),
        format!("flight steps  {}", flight_steps),
        format!("records       MSG {}, IMU {}, TRU {}, ROT {}, NAV/SETP/DMD/MOT {} each, RCS {}",
            c.msg, c.imu, c.tru, c.rot, c.step, c.rcs),
        format!("final position NED {:.4?} m", last.position_m),
        format!("final velocity NED {:.4?} m/s", last.velocity_ms),
        format!("log           {}", out.display()),
    ];
    let (checks, notes) = judge(&scenario.verdict, &trace);
    Ok(Outcome { summary, checks, notes })
}

/// `Ok(true)` when every check passed.
fn go(args: &Args) -> Result<bool, String> {
    let join = |errs: Vec<String>| errs.join("\n");
    let card = Card::load(&args.card).map_err(join)?;
    // The constants compiled into the flight code and the card the plant reads must be the same file.
    if marv_card::CARD_SHA256 != card.sha256 {
        return Err(format!(
            "card {} (sha256 {}) is not the card compiled into the firmware constants (sha256 {}); rebuild",
            args.card.display(), card.sha256, marv_card::CARD_SHA256
        ));
    }
    let env = Env::load(&args.env).map_err(join)?;
    match &args.job {
        Job::One { scenario, out, point } => {
            let scenario = Scenario::load(scenario)?;
            let o = run_one(&card, &env, &scenario, args.seed, *point, out)?;
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
        Job::Spread { spread, out_dir } => {
            let spread = Spread::load(spread)?;
            let scenarios = spread.scenarios.iter().map(Scenario::load).collect::<Result<Vec<_>, _>>()?;
            for (i, p) in spread.points.iter().enumerate() {
                println!("P{i}  {}", p.text());
            }
            print!("{:<12}", "scenario");
            for i in 0..spread.points.len() {
                print!(" {:<34}", format!("P{i}"));
            }
            println!();
            let mut all = true;
            for s in &scenarios {
                print!("{:<12}", s.name);
                for (i, p) in spread.points.iter().enumerate() {
                    let out = out_dir.join(format!("{}-p{i:02}-seed{}.BIN", s.name, args.seed));
                    let o = run_one(&card, &env, s, args.seed, Some(*p), &out)?;
                    let cell = match o.checks.iter().find(|c| !c.pass) {
                        None => "PASS".to_string(),
                        Some(c) => format!("FAIL {}", c.name),
                    };
                    all &= o.checks.iter().all(|c| c.pass);
                    print!(" {cell:<34}");
                }
                println!();
            }
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
        Err(e) => {
            eprintln!("marv-run: {e}");
            ExitCode::from(1)
        }
    }
}
