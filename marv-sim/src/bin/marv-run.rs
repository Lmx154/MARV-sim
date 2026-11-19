//! Runs the real firmware flight step against the plant, in a loop, and writes a DataFlash log.
//! Tick table: conventions 6. The loop is the one in decision-sim-seams 8, with the stale-command rule left for step 5.

use fw_flight::gate::MotorCommand;
use fw_flight::gnc::{Allocator, Controller, Estimator, NoAllocator, NoController, NoEstimator, Sensors};
use fw_flight::step::{Flight, FlightInputs};
use marv_card::{Card, Env};
use marv_sim::log::Log;
use marv_sim::rng::Rng;
use marv_sim::scenario::{Rotors, Scenario};
use marv_sim::sensors::{IMU_PERIOD_US, ImuModel};
use marv_sim::{Backend, NativeBackend, Plant, Truth};
use std::path::PathBuf;
use std::process::ExitCode;

const TICK_US: u64 = 125;
/// The ESC exchange: every 8 ticks, 1 kHz.
const ESC_TICKS: u64 = 8;
/// Noise streams, one per sensor (src/rng.rs).
const STREAM_IMU1: u64 = 1;
const STREAM_IMU2: u64 = 2;

const USAGE: &str = "usage: marv-run --card <marv.toml> --env <env.toml> --scenario <file> --seed <u64> --out <file.BIN>";

struct Args {
    card: PathBuf,
    env: PathBuf,
    scenario: PathBuf,
    seed: u64,
    out: PathBuf,
}

/// `Ok(None)` means --help.
fn parse_args(args: &[String]) -> Result<Option<Args>, String> {
    let (mut card, mut env, mut scenario, mut seed, mut out) = (None, None, None, None, None);
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
            _ => return Err(format!("unknown argument {flag}")),
        }
    }
    Ok(Some(Args {
        card: card.ok_or("--card is required")?,
        env: env.ok_or("--env is required")?,
        scenario: scenario.ok_or("--scenario is required")?,
        seed: seed.ok_or("--seed is required")?,
        out: out.ok_or("--out is required")?,
    }))
}

/// The speed at which four rotors carry the weight.
fn hover_speed(card: &Card, env: &Env) -> Result<f64, String> {
    let mass = card.mass.mass.v.ok_or("mass.mass is \"ask\"")?;
    let g = env.gravity.v.ok_or("gravity.g is \"ask\"")?;
    let k = card.propulsion.thrust_coefficient.v.ok_or("propulsion.thrust_coefficient is \"ask\"")?;
    Ok((mass * g / (4.0 * k)).sqrt())
}

/// The simulated world: what the flight step is run against.
struct World {
    plant: Plant,
    backend: NativeBackend,
    imu1: ImuModel,
    imu2: ImuModel,
    wind_ned_ms: [f64; 3],
}

/// Runs `ticks` base ticks and returns how many flight steps it made.
fn run<E: Estimator, C: Controller, A: Allocator>(
    mut flight: Flight<E, C, A>,
    world: &mut World,
    ticks: u64,
    log: &mut Log,
) -> u64 {
    let mut newest = MotorCommand::STOP;
    let mut levels = [0u16; 4];
    let mut flight_steps = 0;
    for k in 0..ticks {
        if k.is_multiple_of(ESC_TICKS) {
            // The stale-command hold rule (a command too old counts as Stop) comes in step 5.
            levels = match newest.throttles() {
                None => [0; 4],
                Some(throttles) => throttles.map(|t| t.level()),
            };
        }
        let wrench = world.plant.step(levels, world.backend.truth(), world.wind_ned_ms, TICK_US);
        let truth = world.backend.step(&wrench, TICK_US).clone();

        if truth.t_us.is_multiple_of(IMU_PERIOD_US) {
            let s1 = world.imu1.sample(&truth);
            let s2 = world.imu2.sample(&truth);
            let sensors = Sensors { imu1: s1, imu2: Some(s2), baro: None, mag: None, gnss: None, external: None };
            let out = flight.step(&FlightInputs { sensors, rc: None });
            newest = out.motors;
            flight_steps += 1;

            log.imu(truth.t_us, 0, &s1.reading);
            log.imu(truth.t_us, 1, &s2.reading);
            log.truth(&truth);
            log.rotors(truth.t_us, levels, world.plant.rotor_speeds());
        }
    }
    flight_steps
}

fn firmware_text() -> String {
    let commit = env!("FW_COMMIT");
    let dirty = if env!("FW_DIRTY") == "1" { "-dirty" } else { "" };
    format!("firmware {}{dirty}", &commit[..commit.len().min(12)])
}

fn go(args: &Args) -> Result<(), String> {
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
    let scenario = Scenario::load(&args.scenario)?;

    let mut plant = Plant::new(&card, &env)?;
    if scenario.rotors == Rotors::Hover {
        plant.set_rotor_speeds([hover_speed(&card, &env)?; 4]);
    }
    let initial = Truth {
        t_us: 0,
        position_m: scenario.position_ned_m,
        velocity_ms: scenario.velocity_ned_ms,
        attitude: scenario.attitude,
        rates_rads: scenario.rates_rads,
        accel_ms2: [0.0; 3],
        ang_accel_rads2: [0.0; 3],
    };
    let backend = NativeBackend::new(&card, initial)?;
    let imu1 = ImuModel::new("imu1", &card.imu1, &env, Rng::new(args.seed, STREAM_IMU1))?;
    let imu2 = ImuModel::new("imu2", &card.imu2, &env, Rng::new(args.seed, STREAM_IMU2))?;
    // Wind for this step is the mean only; gusts come in step 7.
    let wind = env.wind_mean_ned.v.ok_or("wind.mean_ned is \"ask\"")?;

    let mut log = Log::with_header();
    log.msg(0, &format!("marv-sim {}", env!("CARGO_PKG_VERSION")));
    log.msg(0, &firmware_text());
    log.msg(0, &format!("card sha256 {}", &card.sha256[..16]));
    log.msg(0, &format!("scenario {}", scenario.name));
    log.msg(0, &format!("seed {}", args.seed));

    let ticks = (scenario.duration_s * 1e6 / TICK_US as f64).round() as u64;
    let flight = Flight::new(NoEstimator, NoController, NoAllocator);
    let mut world = World { plant, backend, imu1, imu2, wind_ned_ms: wind };
    let flight_steps = run(flight, &mut world, ticks, &mut log);
    log.write(&args.out)?;

    let last = world.backend.truth();
    let c = log.counts;
    println!("scenario      {}", scenario.name);
    println!("duration      {} s", scenario.duration_s);
    println!("ticks         {} (125 us each)", ticks);
    println!("flight steps  {}", flight_steps);
    println!("records       MSG {}, IMU {}, TRU {}, ROT {}", c.msg, c.imu, c.tru, c.rot);
    println!("final position NED {:.4?} m", last.position_m);
    println!("final velocity NED {:.4?} m/s", last.velocity_ms);
    println!("log           {}", args.out.display());
    println!("RESULT: not judged (the scenario has no thresholds yet)");
    Ok(())
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
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("marv-run: {e}");
            ExitCode::from(1)
        }
    }
}
