//! Runs the real firmware flight step against the plant, in a loop, writes a DataFlash log and judges the run.
//! Tick table: conventions 6. The loop is the one in decision-sim-seams 8, with RC and the ESC hold rule of
//! decision-first-pair-wiring B10; the verdict is B11; the spread is B12 with the five points of the rulings' ruling 4;
//! the matrix of cells (decision-sim-takeoff-landing, section 4) is the spread file with more axes.

use fw_core::handshake::Handshake;
use fw_core::rc::{RcSample, Sticks};
use fw_flight::alloc::CardAllocator;
use fw_core::esc::{LINES, line_of_element};
use fw_flight::esc::hold;
use fw_flight::gate::MotorCommand;
use fw_flight::gnc::{Allocator, Controller, Estimator, GeoPoint, Sensors, Setpoint};
use fw_flight::guidance::Guidance;
use fw_flight::gnc::NavState;
use fw_flight::mahony::Mahony;
use fw_flight::eskf::{Aiding, Eskf, EskfStats, RING, code};
use fw_flight::eskf_out::{EskfSteer, SteerMix};
use fw_flight::mahony_baro::MahonyBaro;
use fw_flight::pid::CascadedPid;
use fw_flight::step::{Flight, FlightInputs, ModeLog, StepOut, log_due, mode_code};
use fw_flight::tuning::{GAIN_NAMES, Gains, Tuning};
use marv_card::{Card, Env};
use marv_sim::log::Log;
use marv_sim::pilot::{Ending, Pilot, PilotView};
use marv_sim::rng::Rng;
use marv_sim::scenario::{Air, Cell, Ground, MagFaultKind, Map, Point, RcRow, Rotors, Scenario, Spread};
use marv_sim::sensors::{IMU_PERIOD_US, ImuModel};
use marv_sim::slow_sensors::{Origin, SlowSensors, geo_from_ned, ned_from_geo};
use marv_sim::verdict::{Check, Sample, Trace, judge};
use marv_sim::{Backend, NativeBackend, Plant, Reaction, Start, Truth};
use std::cell::{Cell as StdCell, RefCell};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::rc::Rc;
use std::time::Instant;

const TICK_US: u64 = 125;
/// The ESC exchange: every 8 ticks, 1 kHz.
const ESC_TICKS: u64 = 8;
/// The controller's sample rate: the IMU models' rate.
const IMU_RATE_HZ: f32 = 1.0e6 / IMU_PERIOD_US as f32;
/// The motors' pole count (poles, not pole pairs; the firmware's `MOT_POLES` default): the one place the sim takes it from,
/// for the eRPM it hands the flight step and for `set_notch`.
const MOTOR_POLES: i32 = 14;
/// `--notch`: the flight step's RPM notch is switched on (before arming).
static NOTCH: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
/// `--fs-hold`, `--no-heartbeat`, `--x-inf` (hands-off campaign).
static FS_HOLD: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static NO_HEARTBEAT: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static X_INF: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
/// The truth yaw (rad, f64 bits) of the plant at the flight step about to run, for the `yaw_hold` hook in `Mix3`.
/// `--pilot-speed <m/s>`: PILOT_SPEED_XY through Flight::set_pilot_speed_xy before arming (hands-off campaign H13 item 5).
static PILOT_SPEED: std::sync::OnceLock<f32> = std::sync::OnceLock::new();
/// The GNSS scalars the core gated in the latest update (bit = event source: 0 N, 1 E, 2 VN, 3 VE, 12 VD), for the dump.
static GATED_STEP: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
static TRUTH_YAW: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
fn flag(f: &std::sync::atomic::AtomicBool) -> bool {
    f.load(std::sync::atomic::Ordering::Relaxed)
}

/// The eRPM the ESCs report, by DShot line, from the plant's rotor speeds (rad/s, element `e` = motor `e + 1`, which is
/// line `line_of_element(e)`): rev/min times pole pairs, stamped `t_us` on the flight step's clock.
fn erpm_lines(speeds_rads: [f64; 4], t_us: u64) -> [Option<(u32, u64)>; LINES] {
    let mut erpm = [None; LINES];
    for (e, w) in speeds_rads.iter().enumerate() {
        let line = line_of_element(e).expect("four elements");
        let rpm = w.abs() * 60.0 / std::f64::consts::TAU;
        erpm[line] = Some(((rpm * f64::from(MOTOR_POLES / 2)).round() as u32, t_us));
    }
    erpm
}
/// A stick report: every 32 ticks, 4 ms (B10).
const RC_TICKS: u64 = 32;
/// Noise streams, one per sensor (src/rng.rs).
const STREAM_IMU1: u64 = 1;
const STREAM_IMU2: u64 = 2;

const USAGE: &str = "usage: marv-run --card <marv.toml> --env <env.toml> --seed <u64> --scenario <file> --out <file.BIN> \
[--point <kf_scale>,<inertia_scale>,<lag_s or ->] [--home <env/home.local.toml>] [--notch] [--params <file>]
       marv-run --card <marv.toml> --env <env.toml> [--seed <u64>] --spread <file> --out-dir <dir> \
[--only <cell>] [--logs fail|all] [--home <env/home.local.toml>] [--notch] [--params <file>]
--params <file>: the ground station's parameter file (NAME,VALUE per line, # comments); the 14 ATC_* gains in it reach the flight
step through Flight::set_gains before arming, as on the board; others are ignored, missing gains keep their defaults; a gain
out of range or not a number stops the run; the summary lists the gains that differ from Tuning::DEFAULT.
(--seed is given exactly when the spread file has no seeds axis)
Either form takes --shadow mahony-baro or --shadow eskf: that estimator runs in the shadow slot and one `shadow` line
reports it, and --preroll <seconds> (air-start scenarios only, not a ground start: that has its own ground phase; the
flight code is stepped that long with the plant pinned at the initial state, disarmed, before scenario time 0), and --estimator mahony|eskf1|eskf2|eskf3 (the flight step's estimator, eskf3 = SteerMix; one `steer` line reports it; --window <from_s>,<to_s>, repeatable, adds a `window` line per interval) with
and --noaid-sigma <m> (replaces
Tuning::nav_noaid_sigma_m, the armed no-aid position hold's sigma).
--gdn <north_m>,<east_m> (repeatable, at most 4; one run only) runs the firmware's shadow guidance as the board does, with targets
at those offsets from the origin the estimator reports, and --gdn-radius <m> (default 2.0) the arrival radius: one GDN record
every 8th step, and one `gdn` line. Without --gdn the guidance is not called and no GDN record is written.
--fs-hold (hands-off campaign): FS_HOLD_EN 1 before arming, the radio HEARTBEAT time given fresh at every flight step (the
step's own time) unless --no-heartbeat, heading_ok given after every step as the estimator's own heading flag (H12: the firmware
adds NOT track_fault), and one `hands_off` line; --x-inf: the H12 control, the track distance forgotten after every step
(Flight::sim_clear_track_error) so the 5 m rule cannot trip.";

/// `--noaid-sigma`: replaces `Tuning::nav_noaid_sigma_m` (the armed no-aid position hold's sigma) for every estimator.
static NOAID_SIGMA: std::sync::OnceLock<f32> = std::sync::OnceLock::new();

/// `--tuning-override <name>=<value>` (a negative-control option, not for the flight's own runs): replaces one of
/// `psc_posz_p`, `psc_velz_p`, `psc_velz_i` in the flight code's `Tuning`.
/// `--ground-s <s>` (P-G4b): replaces the `after_s` of a pilot scenario's first phase (the ground phase, disarmed) and
/// lengthens `duration_s` by the difference; the scenario files are not changed.
static GROUND_S: std::sync::OnceLock<f64> = std::sync::OnceLock::new();

/// `--gdn <north_m>,<east_m>` (repeatable): the shadow guidance's targets, as offsets from the estimator's origin.
static GDN_TARGETS: std::sync::Mutex<Vec<[f64; 2]>> = std::sync::Mutex::new(Vec::new());
/// `--gdn-radius <m>`: the shadow guidance's arrival radius.
static GDN_RADIUS: std::sync::OnceLock<f32> = std::sync::OnceLock::new();
/// The sim world's origin (the env's), set by `run_one` when targets are given, for the `gdn` line.
static GDN_SIM_ORIGIN: std::sync::OnceLock<Origin> = std::sync::OnceLock::new();

/// `--params <file>`: the 14 gains the file gives (the defaults for those it lacks).
static PARAM_GAINS: std::sync::OnceLock<Gains> = std::sync::OnceLock::new();

/// The station's parameter file (`gcs-mavlink` param_file.rs `parse`): one `NAME,VALUE` per line, `#` starts a comment,
/// blank lines skipped, an optional BOM, name up to 16 characters ended by a comma or whitespace, the value a finite f64.
fn parse_param_file(text: &str) -> Result<Vec<(String, f64)>, String> {
    let mut entries = Vec::new();
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    for (number, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let at = |m: String| format!("line {}: {m}", number + 1);
        let name_end = line.find(|c: char| c == ',' || c.is_whitespace()).unwrap_or(line.len());
        let (name, rest) = line.split_at(name_end);
        if name.is_empty() {
            return Err(at("no parameter name".to_owned()));
        }
        if name.len() > 16 {
            return Err(at(format!("name {name} is longer than 16 characters")));
        }
        let rest = rest.trim_start();
        let rest = rest.strip_prefix(',').unwrap_or(rest).trim();
        if rest.is_empty() {
            return Err(at(format!("no value for {name}")));
        }
        match rest.parse::<f64>() {
            Ok(value) if value.is_finite() => entries.push((name.to_owned(), value)),
            _ => return Err(at(format!("{rest} is not a number"))),
        }
    }
    Ok(entries)
}

/// The gains a parameter file gives: the defaults, with the ATC_* gains of the file (the last line of a name wins) in place.
/// A value that is not an f32 or is outside `Gains::MIN`..`Gains::MAX` is an error naming the parameter.
fn gains_from_param_file(text: &str) -> Result<Gains, String> {
    let mut g = Gains::of(&Tuning::DEFAULT);
    for (name, number) in parse_param_file(text)? {
        let Some(i) = GAIN_NAMES.iter().position(|n| *n == name) else { continue };
        let v = number as f32;
        if !v.is_finite() || !(Gains::MIN.0[i]..=Gains::MAX.0[i]).contains(&v) {
            return Err(format!("{name}: {number} is outside {} to {}", Gains::MIN.0[i], Gains::MAX.0[i]));
        }
        g.0[i] = v;
    }
    Ok(g)
}

/// Hands `PARAM_GAINS` (if given) to the flight step, as the board does before arming.
fn apply_param_gains<E: Estimator, C: Controller, A: Allocator>(mut flight: Flight<E, C, A>) -> Result<Flight<E, C, A>, String> {
    if let Some(g) = PARAM_GAINS.get() {
        if !flight.set_gains(*g) {
            return Err("--params: Flight::set_gains refused the gains".to_string());
        }
    }
    Ok(flight)
}

static TUNING_OVERRIDE: std::sync::Mutex<Vec<(String, f32)>> = std::sync::Mutex::new(Vec::new());

struct Args {
    card: PathBuf,
    env: PathBuf,
    /// The env's git-ignored `home_override`: its records replace the env file's (origin and field).
    home: Option<PathBuf>,
    seed: Option<u64>,
    /// `--shadow mahony-baro` or `--shadow eskf`: the estimator in the shadow slot.
    shadow: Option<ShadowKind>,
    /// `--estimator`: the flight step's estimator; `None` is the default Mahony with no `steer` line.
    steer: Option<SteerCfg>,
    /// `--preroll <s>`: the runner's pre-roll before scenario time 0 (decision-nav-eskf D5), in base ticks.
    preroll: Option<u64>,
    /// `--window <from_s>,<to_s>` (repeatable): scenario-time intervals the `steer` run reports one `window` line for.
    windows: Vec<(u64, u64)>,
    job: Job,
}

/// The flight step's estimator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EstKind {
    Mahony,
    Eskf1,
    Eskf2,
    /// G-1b: `SteerMix` of `EskfSteer` around an `Eskf::new` core (attitude, rates, horizontal) and an `EskfZ` (vertical).
    Eskf3,
}

#[derive(Debug, Clone, Copy)]
struct SteerCfg {
    est: EstKind,
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
    let (mut only, mut logs, mut home) = (None, None, None);
    let mut shadow = None;
    let (mut estimator, mut preroll) = (None, None);
    let mut windows = Vec::new();
    let mut it = args.iter();
    while let Some(flag) = it.next() {
        if flag == "--help" || flag == "-h" {
            return Ok(None);
        }
        if flag == "--notch" {
            NOTCH.store(true, std::sync::atomic::Ordering::Relaxed);
            continue;
        }
        let switch = match flag.as_str() {
            "--fs-hold" => Some(&FS_HOLD),
            "--no-heartbeat" => Some(&NO_HEARTBEAT),
            "--x-inf" => Some(&X_INF),
            _ => None,
        };
        if let Some(sw) = switch {
            sw.store(true, std::sync::atomic::Ordering::Relaxed);
            continue;
        }
        let value = it.next().ok_or(format!("{flag}: needs a value"))?;
        match flag.as_str() {
            "--card" => card = Some(PathBuf::from(value)),
            "--env" => env = Some(PathBuf::from(value)),
            "--home" => home = Some(PathBuf::from(value)),
            "--scenario" => scenario = Some(PathBuf::from(value)),
            "--seed" => seed = Some(value.parse::<u64>().map_err(|_| format!("--seed: {value} is not a u64"))?),
            "--out" => out = Some(PathBuf::from(value)),
            "--point" => point = Some(Point::parse(value)?),
            "--spread" => spread = Some(PathBuf::from(value)),
            "--out-dir" => out_dir = Some(PathBuf::from(value)),
            "--only" => only = Some(value.clone()),
            "--shadow" if value == "mahony-baro" => shadow = Some(ShadowKind::MahonyBaro),
            "--shadow" if value == "eskf" => shadow = Some(ShadowKind::Eskf),
            "--shadow" => return Err(format!("--shadow: {value} is not mahony-baro or eskf")),
            "--params" => {
                let text = std::fs::read_to_string(value).map_err(|e| format!("--params {value}: {e}"))?;
                let g = gains_from_param_file(&text).map_err(|e| format!("--params {value}: {e}"))?;
                let _ = PARAM_GAINS.set(g);
            }
            "--noaid-sigma" => {
                let v = value.parse::<f32>().ok().filter(|v| v.is_finite() && *v > 0.0).ok_or(format!("--noaid-sigma: {value} is not a positive number"))?;
                let _ = NOAID_SIGMA.set(v);
            }
            "--tuning-override" => {
                let (name, v) = value.split_once('=').ok_or(format!("--tuning-override: {value} is not name=value"))?;
                let v = v.parse::<f32>().ok().filter(|v| v.is_finite()).ok_or(format!("--tuning-override: {value}: not a number"))?;
                if !["psc_posz_p", "psc_velz_p", "psc_velz_i", "nav_baro_sigma_m", "notch_harmonics", "notch_min_hz", "notch_fade_hz"].contains(&name) {
                    return Err(format!("--tuning-override: {name} is not psc_posz_p, psc_velz_p, psc_velz_i, nav_baro_sigma_m, notch_harmonics, notch_min_hz or notch_fade_hz"));
                }
                if name == "notch_harmonics" && ![1.0, 2.0, 3.0].contains(&v) {
                    return Err(format!("--tuning-override: {value}: notch_harmonics is 1, 2 or 3"));
                }
                TUNING_OVERRIDE.lock().unwrap().push((name.to_string(), v));
            }
            "--preroll" => {
                let s = value.parse::<f64>().map_err(|_| format!("--preroll: {value} is not a number"))?;
                if !(s.is_finite() && s > 0.0 && s <= 60.0) {
                    return Err(format!("--preroll: {value} is not in (0, 60] seconds"));
                }
                preroll = Some((s * 1e6 / TICK_US as f64).round() as u64);
            }
            "--gdn" => {
                let bad = || format!("--gdn: {value} is not <north_m>,<east_m>");
                let (a, b) = value.split_once(',').ok_or_else(bad)?;
                let (a, b) = (a.parse::<f64>().map_err(|_| bad())?, b.parse::<f64>().map_err(|_| bad())?);
                if !(a.is_finite() && b.is_finite()) {
                    return Err(bad());
                }
                let mut t = GDN_TARGETS.lock().unwrap();
                if t.len() >= GDN_SLOTS {
                    return Err(format!("--gdn: at most {GDN_SLOTS} targets"));
                }
                t.push([a, b]);
            }
            "--gdn-radius" => {
                let v = value.parse::<f32>().ok().filter(|v| v.is_finite() && *v >= 0.0).ok_or(format!("--gdn-radius: {value} is not a number >= 0"))?;
                let _ = GDN_RADIUS.set(v);
            }
            "--pilot-speed" => {
                let v = value.parse::<f32>().ok().filter(|v| v.is_finite()).ok_or(format!("--pilot-speed: {value} is not a number"))?;
                let _ = PILOT_SPEED.set(v);
            }
            "--ground-s" => {
                let v = value.parse::<f64>().ok().filter(|v| v.is_finite() && *v > 0.0 && *v <= 300.0).ok_or(format!("--ground-s: {value} is not in (0, 300] seconds"))?;
                let _ = GROUND_S.set(v);
            }
            "--window" => {
                let bad = || format!("--window: {value} is not <from_s>,<to_s> with 0 <= from_s < to_s");
                let (a, b) = value.split_once(',').ok_or_else(bad)?;
                let (a, b) = (a.parse::<f64>().map_err(|_| bad())?, b.parse::<f64>().map_err(|_| bad())?);
                if !(a.is_finite() && b.is_finite() && a >= 0.0 && a < b) {
                    return Err(bad());
                }
                windows.push(((a * 1e6).round() as u64, (b * 1e6).round() as u64));
            }
            "--estimator" => {
                estimator = Some(match value.as_str() {
                    "mahony" => EstKind::Mahony,
                    "eskf1" => EstKind::Eskf1,
                    "eskf2" => EstKind::Eskf2,
                    "eskf3" => EstKind::Eskf3,
                    _ => return Err(format!("--estimator: {value} is not mahony, eskf1, eskf2 or eskf3")),
                })
            }
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
    if GDN_TARGETS.lock().unwrap().is_empty() && GDN_RADIUS.get().is_some() {
        return Err("--gdn-radius needs --gdn".into());
    }
    if !GDN_TARGETS.lock().unwrap().is_empty() && !matches!(job, Job::One { .. }) {
        return Err("--gdn is for --scenario runs only".into());
    }
    let steer = estimator.map(|est| SteerCfg { est });
    Ok(Some(Args {
        steer,
        preroll,
        windows,
        card: card.ok_or("--card is required")?,
        env: env.ok_or("--env is required")?,
        home,
        shadow,
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

/// The shadow slot (decision-sim-seams 9): `MahonyBaro` fed the same sensors as the flight step and the gate's armed
/// state the step left, compared with the flight step's estimator (I9) and with truth (B4 a2). Nothing it does
/// reaches the flight step, the log or the verdict.
struct Shadow {
    est: MahonyBaro,
    /// The gate's state after the previous step, as `Flight::step` gives its own estimator.
    armed: bool,
    /// Step time and truth down at VertKf's initialisation (the vehicle is still then in every scenario).
    init: Option<(u64, f64)>,
    /// Steps judged (from 5 s after the initialisation), sums of squares and largest errors, D (m) and vD (m/s).
    n: u64,
    d2: f64,
    d_max: f64,
    v2: f64,
    v_max: f64,
    /// Judged steps with |vD error| above 3 sigma (B3's test-only sigma check).
    v_3sigma: u64,
    /// Judged steps with velocity_d or position_d false.
    flags_lost: u64,
    /// Airborne seen since the initialisation, then the first touchdown after it; the largest errors and 3-sigma count
    /// before that touchdown (the touchdown hold zeroes the velocity in one tick, which no IMU sample shows).
    airborne: bool,
    landed: bool,
    pre_td: (f64, f64, u64),
    /// Judged steps before the first touchdown: the window of the owner's vertical-speed rule (2026-11-21), whose
    /// 3-sigma count is `pre_td.2`; a run that never lifts off or never touches down has the whole judged window.
    pre_td_n: u64,
    /// The barometer model's drift state at the initialisation, m, and the largest |D error| with the drift since then
    /// taken out (the model's h = true height + drift, so D - truth = -(drift - drift0) + the filter's own error).
    drift0: f64,
    d_max_nodrift: f64,
    /// Airborne = the plant's `Truth::on_ground` false at every 125 us tick since the previous step (a touchdown that
    /// lifts off at the next tick shows). The step of the last lift-off (contact in the previous step's ticks, none in
    /// this one's); before any, the run start. Lift-offs counted.
    was_ground: Option<bool>,
    liftoff_us: u64,
    liftoffs: u64,
    /// Judged airborne steps; of them, |vD error| above 3 sigma: count, first and last time since lift-off (s), the
    /// largest |error|/sigma, and how many within 2 s of lift-off.
    air_n: u64,
    air_v3: u64,
    air_v3_since: (f64, f64),
    air_v3_ratio: f64,
    air_v3_2s: u64,
    air_v3_half_s: u64,
    /// The first touchdown after lift-off: its step, and the D error (raw, drift taken out) there and 2 s later.
    td_us: Option<u64>,
    d_td: Option<(f64, f64)>,
    d_td2: Option<(f64, f64)>,
    /// Steps whose attitude, rates or tilt/rates flags differ from the flight step's estimator, bit for bit (I9).
    identity_breaks: u64,
}

/// The judged window starts this long after the initialisation, us (B4 a2: "from 5 s after initialisation").
const SHADOW_SETTLE_US: u64 = 5_000_000;

impl Shadow {
    fn new(tuning: &Tuning) -> Shadow {
        Shadow { est: MahonyBaro::new(tuning), armed: false, init: None, n: 0, d2: 0.0, d_max: 0.0, v2: 0.0, v_max: 0.0,
            v_3sigma: 0, flags_lost: 0, airborne: false, landed: false, pre_td: (0.0, 0.0, 0), pre_td_n: 0, drift0: 0.0, d_max_nodrift: 0.0,
            was_ground: None, liftoff_us: 0, liftoffs: 0, air_n: 0, air_v3: 0, air_v3_since: (f64::NAN, f64::NAN), air_v3_ratio: 0.0,
            air_v3_2s: 0, air_v3_half_s: 0, td_us: None, d_td: None, d_td2: None, identity_breaks: 0 }
    }

    /// After each flight step, as `Flight::step` runs its estimator (a non-finite IMU sample never reaches it).
    fn feed(&mut self, sensors: &Sensors, out: &StepOut, truth: &Truth, drift_m: f64, ground: bool) {
        self.est.set_armed(self.armed);
        self.armed = out.armed;
        let r = &sensors.imu1.reading;
        if !r.gyro_rads.iter().chain(&r.accel_ms2).all(|v| v.is_finite()) {
            return;
        }
        let nav: NavState = self.est.update(sensors);
        let same = nav.attitude.map(f32::to_bits) == out.nav.attitude.map(f32::to_bits)
            && nav.rates_rads.map(f32::to_bits) == out.nav.rates_rads.map(f32::to_bits)
            && nav.valid.tilt == out.nav.valid.tilt
            && nav.valid.rates == out.nav.valid.rates;
        self.identity_breaks += u64::from(!same);
        if self.init.is_none() && self.est.vert().initialised() {
            self.init = Some((nav.t_us, truth.position_m[2]));
            self.drift0 = drift_m;
        }
        if self.was_ground == Some(true) && !ground {
            self.liftoff_us = nav.t_us;
            self.liftoffs += 1;
        }
        self.was_ground = Some(ground);
        let Some((t0, d0)) = self.init else { return };
        let ed = f64::from(nav.position_m[2]) - (truth.position_m[2] - d0);
        let ed_nd = ed + (drift_m - self.drift0);
        let ev = f64::from(nav.velocity_ms[2]) - truth.velocity_ms[2];
        let sigma_v = nav.sigma.map_or(f64::INFINITY, |s| f64::from(s.velocity_d_ms));
        self.airborne |= !ground;
        if self.airborne && ground && !self.landed {
            self.td_us = Some(nav.t_us);
            self.d_td = Some((ed, ed_nd));
        }
        self.landed |= self.airborne && ground;
        if self.d_td2.is_none() && self.td_us.is_some_and(|t| nav.t_us >= t + 2_000_000) {
            self.d_td2 = Some((ed, ed_nd));
        }
        if nav.t_us < t0 + SHADOW_SETTLE_US {
            return;
        }
        self.d_max_nodrift = self.d_max_nodrift.max(ed_nd.abs());
        if !ground {
            self.air_n += 1;
            if ev.abs() > 3.0 * sigma_v {
                let since = (nav.t_us - self.liftoff_us) as f64 * 1e-6;
                self.air_v3 += 1;
                self.air_v3_since = (self.air_v3_since.0.min(since), self.air_v3_since.1.max(since));
                self.air_v3_ratio = self.air_v3_ratio.max(ev.abs() / sigma_v);
                self.air_v3_2s += u64::from(since < 2.0);
                self.air_v3_half_s += u64::from(since < 0.5);
            }
        }
        self.n += 1;
        self.d2 += ed * ed;
        self.d_max = self.d_max.max(ed.abs());
        self.v2 += ev * ev;
        self.v_max = self.v_max.max(ev.abs());
        self.v_3sigma += u64::from(ev.abs() > 3.0 * sigma_v);
        self.flags_lost += u64::from(!(nav.valid.velocity_d && nav.valid.position_d));
        if !self.landed {
            let (d, v, n3) = self.pre_td;
            self.pre_td_n += 1;
            self.pre_td = (d.max(ed.abs()), v.max(ev.abs()), n3 + u64::from(ev.abs() > 3.0 * sigma_v));
        }
    }

    /// One line, `key value` pairs after `shadow mahony-baro`.
    fn line(&self) -> String {
        let v = self.est.vert();
        let n = self.n.max(1) as f64;
        format!(
            "shadow mahony-baro init_s {} judged {} d_rms {:.4} d_max {:.4} v_rms {:.4} v_max {:.4} v_3sigma {} bd {:.4} \
gated {} fused {} resets {} flags_lost {} identity_breaks {} d_max_pre_td {:.4} v_max_pre_td {:.4} v_3sigma_pre_td {} \
d_max_nodrift {:.4} air_judged {} v_3sigma_air {} air_v3_first_s {:.3} air_v3_last_s {:.3} air_v3_ratio_max {:.2} \
air_v3_within_2s {} air_v3_within_half_s {} liftoffs {} td_s {} d_td {} d_td_nodrift {} d_td2 {} d_td2_nodrift {} v_3sigma_pre_td_judged {}",
            self.init.map_or("none".to_string(), |(t, _)| format!("{:.3}", t as f64 * 1e-6)),
            self.n, (self.d2 / n).sqrt(), self.d_max, (self.v2 / n).sqrt(), self.v_max, self.v_3sigma, v.bias_d(),
            v.rejected(), v.fused(), v.resets(), self.flags_lost, self.identity_breaks, self.pre_td.0, self.pre_td.1,
            self.pre_td.2, self.d_max_nodrift, self.air_n, self.air_v3, self.air_v3_since.0, self.air_v3_since.1,
            self.air_v3_ratio, self.air_v3_2s, self.air_v3_half_s, self.liftoffs,
            self.td_us.map_or("none".to_string(), |t| format!("{:.3}", t as f64 * 1e-6)),
            self.d_td.map_or("none".to_string(), |d| format!("{:.4}", d.0)),
            self.d_td.map_or("none".to_string(), |d| format!("{:.4}", d.1)),
            self.d_td2.map_or("none".to_string(), |d| format!("{:.4}", d.0)),
            self.d_td2.map_or("none".to_string(), |d| format!("{:.4}", d.1)), self.pre_td_n
        )
    }
}

/// Which estimator `--shadow` puts in the shadow slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ShadowKind {
    MahonyBaro,
    Eskf,
}

/// A shadow slot's estimator: fed after each flight step, reported in one line.
trait Slot {
    fn feed(&mut self, sensors: &Sensors, out: &StepOut, truth: &Truth, drift_m: f64, ground: bool);
    fn line(&self) -> String;
}

impl Slot for Shadow {
    fn feed(&mut self, sensors: &Sensors, out: &StepOut, truth: &Truth, drift_m: f64, ground: bool) {
        Shadow::feed(self, sensors, out, truth, drift_m, ground);
    }
    fn line(&self) -> String {
        Shadow::line(self)
    }
}

/// Tilt error, rad: the angle between NED down seen in the body frame (the third row of the body-to-NED matrix) for two
/// attitudes [w, x, y, z]; it does not depend on the yaw of either (the same form as verdict.rs `down_gap_deg`).
fn tilt_err(a: [f64; 4], b: [f64; 4]) -> f64 {
    let z = |q: [f64; 4]| {
        let [w, x, y, zz] = q;
        [2.0 * (x * zz - w * y), 2.0 * (y * zz + w * x), 1.0 - 2.0 * (x * x + y * y)]
    };
    let (u, v) = (z(a), z(b));
    (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]).clamp(-1.0, 1.0).acos()
}

/// The 321 yaw of [w, x, y, z], rad.
fn yaw_of(q: [f64; 4]) -> f64 {
    let [w, x, y, z] = q;
    (2.0 * (x * y + w * z)).atan2(1.0 - 2.0 * (y * y + z * z))
}

fn wrap_pi(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI
}

/// The shadow slot with `Eskf` (decision-nav-eskf-20261120, E1b item 5): fed as `MahonyBaro`'s slot; its state refers
/// to the horizon (`NavState.t_us`), so it is compared with the truth of that step. Heading against truth while
/// `heading` is valid; north-east against truth (the offset between the filter's origin and the sim's removed) while
/// `position_ne` is valid; vertical as `mahony-baro`'s line; tilt beside Mahony's (the flight step's estimator, at the
/// step time) on the same steps. Nothing it does reaches the flight step, the log or the verdict.
struct EskfShadow {
    est: Eskf,
    armed: bool,
    sim_origin: Origin,
    /// Truth of the recent steps: t_us, attitude, velocity, position, contact since the previous step.
    hist: std::collections::VecDeque<(u64, [f64; 4], [f64; 3], [f64; 3], bool)>,
    dt_s: f64,
    last_t: Option<u64>,
    // heading
    h_valid_s: f64,
    h_bad_s: f64,
    h_first_s: Option<f64>,
    h_worst: f64,
    // north-east
    ne_valid_s: f64,
    ne_out3_s: f64,
    ne_n: u64,
    ne_p2: f64,
    ne_pmax: f64,
    ne_vmax: f64,
    // vertical (as `mahony-baro`)
    init: Option<(u64, f64)>,
    drift0: f64,
    v_n: u64,
    d2: f64,
    d_max: f64,
    d_max_nodrift: f64,
    vd2: f64,
    vd_max: f64,
    vd_3sigma: u64,
    /// The a2 rule's window (5 s after the initialisation to the first touchdown, both at the horizon): steps judged and
    /// 3-sigma vD steps; airborne seen, landed.
    pre_td_n: u64,
    pre_td_v3: u64,
    airborne: bool,
    landed: bool,
    // tilt: Eskf (at the horizon) and Mahony (at the step) on the same steps
    t_n: u64,
    te2: f64,
    te_max: f64,
    tm2: f64,
    tm_max: f64,
    /// Tilt alignment time, and the tilt sums split at 5 s after it: (n, Eskf^2, Mahony^2) for the first 5 s and after;
    /// and the Eskf against the truth of the current step (not the horizon), for contrast.
    t_align: Option<u64>,
    split: [(u64, f64, f64); 2],
    te_now2: f64,
}

impl EskfShadow {
    /// The declination from the sim's field, as the tests set it (C4).
    fn new(tuning: &Tuning, env: &Env) -> Result<EskfShadow, String> {
        let mut t = *tuning;
        let f = env.magnetic_field_ned.v.ok_or("magnetic_field.ned is \"ask\"")?;
        t.nav_mag_declination_rad = f[1].atan2(f[0]) as f32;
        let est = Eskf::new(&t).map_err(|e| format!("Eskf: {e:?}"))?;
        Ok(EskfShadow { est, armed: false, sim_origin: Origin::from_env(env)?, hist: Default::default(), dt_s: 0.0, last_t: None,
            h_valid_s: 0.0, h_bad_s: 0.0, h_first_s: None, h_worst: 0.0, ne_valid_s: 0.0, ne_out3_s: 0.0, ne_n: 0, ne_p2: 0.0,
            ne_pmax: 0.0, ne_vmax: 0.0, init: None, drift0: 0.0, v_n: 0, d2: 0.0, d_max: 0.0, d_max_nodrift: 0.0, vd2: 0.0,
            vd_max: 0.0, vd_3sigma: 0, pre_td_n: 0, pre_td_v3: 0, airborne: false, landed: false, t_n: 0, te2: 0.0, te_max: 0.0,
            tm2: 0.0, tm_max: 0.0, t_align: None, split: [(0, 0.0, 0.0); 2], te_now2: 0.0 })
    }
}

impl Slot for EskfShadow {
    fn feed(&mut self, sensors: &Sensors, out: &StepOut, truth: &Truth, drift_m: f64, ground: bool) {
        self.est.set_armed(self.armed);
        self.armed = out.armed;
        let r = &sensors.imu1.reading;
        if !r.gyro_rads.iter().chain(&r.accel_ms2).all(|v| v.is_finite()) {
            return;
        }
        self.hist.push_back((truth.t_us, truth.attitude, truth.velocity_ms, truth.position_m, ground));
        while self.hist.len() > 1_000 {
            self.hist.pop_front();
        }
        let nav = self.est.update(sensors);
        self.dt_s = self.last_t.map_or(0.0, |t| (nav.t_us.saturating_sub(t)) as f64 * 1e-6);
        self.last_t = Some(nav.t_us);
        let Some(&(_, q, v, p, h_ground)) = self.hist.iter().rev().find(|h| h.0 == nav.t_us) else { return };
        let dt = self.dt_s;
        let qe = nav.attitude.map(f64::from);
        let sig = self.est.sigmas();
        if nav.valid.tilt {
            let (e, m) = (tilt_err(qe, q), tilt_err(out.nav.attitude.map(f64::from), truth.attitude));
            self.t_n += 1;
            self.te2 += e * e;
            self.te_max = self.te_max.max(e);
            self.tm2 += m * m;
            self.tm_max = self.tm_max.max(m);
            let t_al = *self.t_align.get_or_insert(nav.t_us);
            let k = usize::from(nav.t_us >= t_al + 5_000_000);
            self.split[k] = (self.split[k].0 + 1, self.split[k].1 + e * e, self.split[k].2 + m * m);
            let now = tilt_err(qe, truth.attitude);
            self.te_now2 += now * now;
        }
        if nav.valid.heading {
            let e = wrap_pi(yaw_of(qe) - yaw_of(q)).abs();
            self.h_valid_s += dt;
            self.h_first_s.get_or_insert(nav.t_us as f64 * 1e-6);
            self.h_bad_s += if e > 15f64.to_radians() { dt } else { 0.0 };
            self.h_worst = self.h_worst.max(e);
        }
        if nav.valid.position_ne {
            let off = self.est.origin().map_or([0.0; 3], |o| ned_from_geo(&self.sim_origin, o.lat_e7, o.lon_e7, o.alt_mm));
            let pe = [f64::from(nav.position_m[0]) + off[0] - p[0], f64::from(nav.position_m[1]) + off[1] - p[1]];
            let ve = [f64::from(nav.velocity_ms[0]) - v[0], f64::from(nav.velocity_ms[1]) - v[1]];
            let s = |i: usize| f64::from(sig[i]);
            let out3 = pe[0].abs() > 3.0 * s(6) || pe[1].abs() > 3.0 * s(7) || ve[0].abs() > 3.0 * s(3) || ve[1].abs() > 3.0 * s(4);
            let h = pe[0].hypot(pe[1]);
            self.ne_valid_s += dt;
            self.ne_out3_s += if out3 { dt } else { 0.0 };
            self.ne_n += 1;
            self.ne_p2 += h * h;
            self.ne_pmax = self.ne_pmax.max(h);
            self.ne_vmax = self.ne_vmax.max(ve[0].hypot(ve[1]));
        }
        if self.init.is_none() && nav.valid.position_d {
            self.init = Some((nav.t_us, p[2]));
            self.drift0 = drift_m;
        }
        let Some((t0, d0)) = self.init else { return };
        if nav.t_us < t0 + SHADOW_SETTLE_US {
            return;
        }
        let ed = f64::from(nav.position_m[2]) - (p[2] - d0);
        let ev = f64::from(nav.velocity_ms[2]) - v[2];
        self.v_n += 1;
        self.d2 += ed * ed;
        self.d_max = self.d_max.max(ed.abs());
        self.d_max_nodrift = self.d_max_nodrift.max((ed + (drift_m - self.drift0)).abs());
        self.vd2 += ev * ev;
        self.vd_max = self.vd_max.max(ev.abs());
        self.vd_3sigma += u64::from(ev.abs() > 3.0 * f64::from(sig[5]));
        self.airborne |= !h_ground;
        self.landed |= self.airborne && h_ground;
        if !self.landed {
            self.pre_td_n += 1;
            self.pre_td_v3 += u64::from(ev.abs() > 3.0 * f64::from(sig[5]));
        }
    }

    /// One line, `key value` pairs after `shadow eskf`.
    fn line(&self) -> String {
        let s = self.est.stats();
        let rms = |sum: f64, n: u64| (sum / n.max(1) as f64).sqrt();
        let d = f64::to_degrees;
        format!(
            "shadow eskf hdg_valid_s {:.2} hdg_bad15_s {:.2} hdg_first_s {} hdg_worst_deg {:.2} ne_valid_s {:.2} ne_out3_s {:.2} \
ne_p_rms {:.3} ne_p_max {:.3} ne_v_max {:.3} init_s {} judged {} d_rms {:.4} d_max {:.4} d_max_nodrift {:.4} v_rms {:.4} v_max {:.4} \
v_3sigma {} v_3sigma_pre_td {} v_3sigma_pre_td_judged {} tilt_rms_deg {:.3} tilt_max_deg {:.3} mahony_tilt_rms_deg {:.3} mahony_tilt_max_deg {:.3} tilt_rms_first5_deg {:.3} \
mahony_tilt_rms_first5_deg {:.3} tilt_rms_after5_deg {:.3} mahony_tilt_rms_after5_deg {:.3} tilt_rms_vs_current_truth_deg {:.3} mag_refused {} \
screens {} gated {} resets_height {} resets_yaw {} resets_gps {} reinits {} yaw_aligned {} multi_job {}",
            self.h_valid_s, self.h_bad_s, self.h_first_s.map_or("none".to_string(), |t| format!("{t:.3}")), d(self.h_worst),
            self.ne_valid_s, self.ne_out3_s, rms(self.ne_p2, self.ne_n), self.ne_pmax, self.ne_vmax,
            self.init.map_or("none".to_string(), |(t, _)| format!("{:.3}", t as f64 * 1e-6)), self.v_n, rms(self.d2, self.v_n),
            self.d_max, self.d_max_nodrift, rms(self.vd2, self.v_n), self.vd_max, self.vd_3sigma, self.pre_td_v3, self.pre_td_n,
            d(rms(self.te2, self.t_n)), d(self.te_max), d(rms(self.tm2, self.t_n)), d(self.tm_max),
            d(rms(self.split[0].1, self.split[0].0)), d(rms(self.split[0].2, self.split[0].0)),
            d(rms(self.split[1].1, self.split[1].0)), d(rms(self.split[1].2, self.split[1].0)),
            d(rms(self.te_now2, self.t_n)), s.mag_refused, s.gps_e1s + s.gps_e3b,
            s.baro_gated + s.mag_gated + s.gps_pos_gated + s.gps_vel_gated, s.height_resets, s.yaw_resets, s.gps_resets,
            s.reinits, self.est.yaw_aligned(), if s.max_jobs_per_step > 1 { "yes" } else { "no" }
        )
    }
}

/// The `Eskf` steering the flight step, wrapped only to read its gyro-bias estimate and re-initialisation count after
/// the run (`Flight` owns its estimator). Everything it returns is the Eskf's own.
struct Steering {
    eskf: Eskf,
    /// Largest |gyro bias| component seen, rad/s, and the Eskf's re-initialisations: read after the run.
    info: Rc<StdCell<(f32, u32)>>,
}

impl Estimator for Steering {
    fn update(&mut self, sensors: &Sensors) -> NavState {
        let nav = self.eskf.update(sensors);
        let bg = self.eskf.biases().0;
        let (m, _) = self.info.get();
        let big = bg.iter().fold(m, |a, b| a.max(b.abs()));
        self.info.set((big, self.eskf.stats().reinits));
        nav
    }
    fn origin(&self) -> Option<fw_flight::gnc::GeoPoint> {
        self.eskf.origin()
    }
    fn set_armed(&mut self, armed: bool) {
        self.eskf.set_armed(armed);
    }
}

/// What the eskf3 run reads of the composite after each step (the `Flight` owns it): the core's counters, the output's
/// tracking errors at each correction, the G6-B latch time and the origin the horizontal position refers to.
#[derive(Default)]
struct Info3 {
    /// The pre-roll's offset: board time minus scenario time, us.
    off_us: u64,
    stats: EskfStats,
    vert_reinits: u32,
    skipped: u32,
    ring_dropped: u32,
    corrected: u32,
    origin: Option<fw_flight::gnc::GeoPoint>,
    /// Scenario time (us) of the first G6-B latch record (NKI Src 8, Ok 0) and of the first yaw alignment.
    latch_t_us: Option<u64>,
    align_t_us: Option<u64>,
    /// Tracking error at each correction: attitude (rad), velocity (m/s), position (m): count, sums of squares, largest.
    track: [(u64, f64, f64); 3],
    /// Largest |gyro bias| component of the core, rad/s, the board time (us) and the axis where it was first reached.
    bias_max: f32,
    bias_max_t_us: u64,
    bias_max_axis: usize,
    /// The core's gyro bias (rad/s) at the last step of each whole board second.
    bias_by_s: Vec<(u64, [f32; 3])>,
    /// HM3b: the yaw injection (scenario time).
    hm_inject_t_us: Option<u64>,
}

/// `SteerMix` over leaked storage (the host is not memory-constrained), wrapped to fill `Info3`. HM3b: `inject` is the
/// scenario's `yaw_inject` (board time, rad), applied once before the first update at or after its time; `air` is the
/// airborne latch Flight last handed over.
struct Mix3 {
    mix: SteerMix<'static, RING>,
    info: Rc<RefCell<Info3>>,
    inject: Option<(u64, f32)>,
    /// `yaw_hold` (board time, rad): from then on, before every update, the filter's yaw is set to the truth yaw + rad.
    hold: Option<(u64, f32)>,
    /// The yaw of the latest output, rad.
    last_yaw: Option<f32>,
}

impl Estimator for Mix3 {
    fn update(&mut self, sensors: &Sensors) -> NavState {
        if let Some((_, rad)) = self.inject.filter(|(at, _)| sensors.imu1.t_us >= *at) {
            self.mix.sim_rotate_yaw(rad);
            self.inject = None;
            let mut i = self.info.borrow_mut();
            i.hm_inject_t_us = Some(sensors.imu1.t_us.saturating_sub(i.off_us));
        }
        if let (Some((at, rad)), Some(yaw)) = (self.hold, self.last_yaw) {
            if sensors.imu1.t_us >= at {
                let truth = f64::from_bits(TRUTH_YAW.load(std::sync::atomic::Ordering::Relaxed));
                self.mix.sim_rotate_yaw(wrap_pi(truth + f64::from(rad) - f64::from(yaw)) as f32);
            }
        }
        let nav = self.mix.update(sensors);
        self.last_yaw = Some(yaw_of(nav.attitude.map(f64::from)) as f32);
        let steer = self.mix.steer();
        let core = steer.core();
        let mut i = self.info.borrow_mut();
        let t = nav.t_us.saturating_sub(i.off_us);
        i.stats = *core.stats();
        i.vert_reinits = self.mix.vert().stats().reinits;
        i.skipped = steer.skipped();
        i.ring_dropped = steer.ring_dropped();
        if steer.corrected() != i.corrected {
            i.corrected = steer.corrected();
            let tr = steer.tracking();
            for (k, x) in [tr.att_rad, tr.vel_ms, tr.pos_m].into_iter().enumerate() {
                let x = f64::from(x);
                let a = i.track[k];
                i.track[k] = (a.0 + 1, a.1 + x * x, a.2.max(x));
            }
        }
        if i.latch_t_us.is_none() && core.events().any(|e| e.src == code::YAW_RESET && e.ok == code::GATED) {
            i.latch_t_us = Some(t);
        }
        let gated = core.events().filter(|e| e.ok == code::GATED && matches!(e.src, 0..=3 | 12)).fold(0u32, |m, e| m | 1 << e.src);
        GATED_STEP.store(gated, std::sync::atomic::Ordering::Relaxed);
        if i.align_t_us.is_none() && i.stats.yaw_aligns > 0 {
            i.align_t_us = Some(t);
        }
        let bg = core.biases().0;
        for (k, b) in bg.iter().enumerate() {
            if b.abs() > i.bias_max {
                i.bias_max = b.abs();
                i.bias_max_t_us = nav.t_us;
                i.bias_max_axis = k;
            }
        }
        match i.bias_by_s.last_mut() {
            Some(l) if l.0 == nav.t_us / 1_000_000 => l.1 = bg,
            _ => i.bias_by_s.push((nav.t_us / 1_000_000, bg)),
        }
        i.origin = self.mix.origin();
        nav
    }
    fn origin(&self) -> Option<fw_flight::gnc::GeoPoint> {
        self.mix.origin()
    }
    fn set_armed(&mut self, armed: bool) {
        self.mix.set_armed(armed);
    }
}

/// The eskf3 additions of the `steer` line: heading error against truth while `heading` is valid and the horizontal
/// position error against truth (current time) while `position_ne` is valid, and every valid step's figures for the
/// `--window` lines.
struct Ext {
    info: Rc<RefCell<Info3>>,
    sim_origin: Origin,
    /// (steps, sum of squares, largest), rad and m.
    hdg: (u64, f64, f64),
    ne: (u64, f64, f64),
    /// Steps with the heading flag, with the NE flags.
    hdg_flag_steps: u64,
    ne_flag_steps: u64,
    steps: u64,
    /// Per step: t_us, tilt error (None if tilt invalid), heading error, horizontal error, truth tilt, all rad or m.
    rows: Vec<(u64, Option<f64>, Option<f64>, Option<f64>, f64)>,
    /// Scenario times of the first and last step with NE flags dropped after having been valid, and of NE valid again.
    ne_prev: bool,
    ne_drops: Vec<(u64, bool)>,
}

/// The `steer` line's accumulators: the steering estimator's tilt error against truth at the same step (rad), split at
/// 5 s after its tilt first becomes valid, and the step-to-step change of the motor levels.
#[derive(Default)]
struct Steer {
    t0: Option<u64>,
    /// [first 5 s, after]: steps, sum of squares, largest.
    tilt: [(u64, f64, f64); 2],
    prev: Option<[u16; 4]>,
    /// Sum of squared level changes (DShot counts) over the four motors, and the count of motor changes: consecutive
    /// steps that both spin.
    dm2: f64,
    dm_n: u64,
    /// Every valid step's time and tilt error (rad), for the per-touchdown figures.
    hist: Vec<(u64, f64)>,
    /// The first step with the gate armed.
    t_arm: Option<u64>,
    /// Largest |yaw difference| between the estimate and truth over the valid-tilt steps, rad.
    yaw_max: f64,
    /// eskf3 only.
    ext: Option<Ext>,
}

impl Steer {
    fn feed(&mut self, out: &StepOut, truth: &Truth) {
        if out.armed && self.t_arm.is_none() {
            self.t_arm = Some(out.t_us);
        }
        if let Some(x) = self.ext.as_mut() {
            x.steps += 1;
            let v = &out.nav.valid;
            let tilt = v.tilt.then(|| tilt_err(out.nav.attitude.map(f64::from), truth.attitude));
            let hdg = v.heading.then(|| wrap_pi(yaw_of(out.nav.attitude.map(f64::from)) - yaw_of(truth.attitude)).abs());
            let ne = v.position_ne.then(|| {
                let off = x.info.borrow().origin.map_or([0.0; 3], |o| ned_from_geo(&x.sim_origin, o.lat_e7, o.lon_e7, o.alt_mm));
                let pe = [f64::from(out.nav.position_m[0]) + off[0] - truth.position_m[0],
                    f64::from(out.nav.position_m[1]) + off[1] - truth.position_m[1]];
                pe[0].hypot(pe[1])
            });
            if let Some(h) = hdg {
                x.hdg = (x.hdg.0 + 1, x.hdg.1 + h * h, x.hdg.2.max(h));
            }
            if let Some(h) = ne {
                x.ne = (x.ne.0 + 1, x.ne.1 + h * h, x.ne.2.max(h));
            }
            x.hdg_flag_steps += u64::from(v.heading);
            x.ne_flag_steps += u64::from(v.position_ne);
            if v.position_ne != x.ne_prev {
                x.ne_drops.push((out.t_us, v.position_ne));
                x.ne_prev = v.position_ne;
            }
            let z = truth.attitude;
            let truth_tilt = (1.0 - 2.0 * (z[1] * z[1] + z[2] * z[2])).clamp(-1.0, 1.0).acos();
            x.rows.push((out.t_us, tilt, hdg, ne, truth_tilt));
        }
        if out.nav.valid.tilt {
            let e = tilt_err(out.nav.attitude.map(f64::from), truth.attitude);
            let t0 = *self.t0.get_or_insert(out.t_us);
            let k = usize::from(out.t_us >= t0 + 5_000_000);
            self.hist.push((out.t_us, e));
            self.yaw_max = self.yaw_max.max(wrap_pi(yaw_of(out.nav.attitude.map(f64::from)) - yaw_of(truth.attitude)).abs());
            let t = &mut self.tilt[k];
            *t = (t.0 + 1, t.1 + e * e, t.2.max(e));
        }
        let now = out.motors.throttles().map(|t| t.map(|x| x.level()));
        if let (Some(p), Some(n)) = (self.prev, now) {
            for i in 0..4 {
                let d = f64::from(n[i]) - f64::from(p[i]);
                self.dm2 += d * d;
                self.dm_n += 1;
            }
        }
        self.prev = now;
    }

    fn line(&self, cfg: SteerCfg, info: Option<(f32, u32)>, t_lo: Option<u64>, windows: &[(u64, u64)]) -> String {
        let d = f64::to_degrees;
        let rms = |t: (u64, f64, f64)| d((t.1 / t.0.max(1) as f64).sqrt());
        let (bias, reinits) = info.unwrap_or((0.0, 0));
        let est = match cfg.est {
            EstKind::Mahony => "mahony",
            EstKind::Eskf1 => "eskf1",
            EstKind::Eskf2 => "eskf2",
            EstKind::Eskf3 => "eskf3",
        };
        // The take-off window: tilt error (valid steps) from the first armed step to 3 s after the first lift-off.
        let w: Vec<f64> = match (self.t_arm, t_lo) {
            (Some(a), Some(l)) => self.hist.iter().filter(|(t, _)| *t >= a && *t <= l + 3_000_000).map(|h| h.1).collect(),
            _ => Vec::new(),
        };
        let to_rms = d((w.iter().map(|e| e * e).sum::<f64>() / w.len().max(1) as f64).sqrt());
        let to_max = d(w.iter().copied().fold(0.0, f64::max));
        let mut text = format!(
            "steer {est} tilt_rms_first5_deg {:.4} tilt_max_first5_deg {:.4} tilt_rms_after_deg {:.4} tilt_max_after_deg {:.4} \
tilt_n_first5 {} tilt_n_after {} bias_max_dps {:.4} motor_dlevel_rms {:.4} motor_dlevel_n {} reinits {} takeoff_rms_deg {:.4} takeoff_max_deg {:.4} takeoff_n {} yaw_diff_max_deg {:.3}",
            rms(self.tilt[0]), d(self.tilt[0].2), rms(self.tilt[1]), d(self.tilt[1].2), self.tilt[0].0, self.tilt[1].0,
            d(f64::from(bias)), (self.dm2 / self.dm_n.max(1) as f64).sqrt(), self.dm_n, reinits, to_rms, to_max, w.len(), d(self.yaw_max)
        );
        if let Some(x) = &self.ext {
            let i = x.info.borrow();
            let s = &i.stats;
            let rms3 = |t: (u64, f64, f64)| (t.1 / t.0.max(1) as f64).sqrt();
            let tr = |k: usize, unit: fn(f64) -> f64| (unit(rms3(i.track[k])), unit(i.track[k].2));
            let (a_rms, a_max) = tr(0, f64::to_degrees);
            let (v_rms, v_max) = tr(1, |x| x);
            let (p_rms, p_max) = tr(2, |x| x);
            let secs = |t: Option<u64>| t.map_or("none".to_string(), |t| format!("{:.2}", t as f64 * 1e-6));
            let all: Vec<f64> = x.rows.iter().filter_map(|r| r.1).collect();
            let tmax = d(all.iter().copied().fold(0.0, f64::max));
            let trms = d((all.iter().map(|e| e * e).sum::<f64>() / all.len().max(1) as f64).sqrt());
            let airborne = match (i.align_t_us, t_lo) {
                (Some(a), Some(l)) => if a > l { "yes" } else { "no" },
                (Some(_), None) => "no",
                _ => "n/a",
            };
            let after: Vec<f64> = i.align_t_us.map_or(Vec::new(), |a| {
                x.rows.iter().filter(|r| r.0 >= a && r.0 < a + 5_000_000).filter_map(|r| r.1).collect()
            });
            let edges: Vec<String> = x.ne_drops.iter().take(8).map(|(t, v)| format!("{:.1}:{}", *t as f64 * 1e-6, if *v { "T" } else { "F" })).collect();
            let bias_at: Vec<String> = [1u64, 2, 3, 5, 10, 20, 40, 60, 62, 65, 70, 80]
                .iter()
                .filter_map(|&b| i.bias_by_s.iter().find(|e| e.0 == b))
                .map(|e| format!("{}:{:.2}/{:.2}/{:.2}", e.0, d(f64::from(e.1[0])), d(f64::from(e.1[1])), d(f64::from(e.1[2]))))
                .collect();
            text += &format!(
                " | bias_max_board_s {:.2} bias_max_axis {} scenario_start_board_s {:.2} bias_dps_xyz_by_board_s {} out_att_track_rms_deg {a_rms:.4} out_att_track_max_deg {a_max:.4} out_vel_track_rms {v_rms:.4} out_vel_track_max {v_max:.4} \
out_pos_track_rms {p_rms:.4} out_pos_track_max {p_max:.4} corrections {} skipped {} ring_dropped {} yaw_aligns {} yaw_align_s {} \
align_airborne {airborne} tilt_5s_after_align_rms_deg {:.4} tilt_5s_after_align_max_deg {:.4} yaw_resets {} gps_pos_fused {} gps_resets {} \
reinits {} vert_reinits {} latch {} latch_s {} tilt_all_rms_deg {trms:.4} tilt_all_max_deg {tmax:.4} hdg_rms_deg {:.4} hdg_max_deg {:.4} hdg_flag_steps {} \
ne_rms_m {:.4} ne_max_m {:.4} ne_flag_steps {} steps {} ne_flag_edges {}",
                i.bias_max_t_us as f64 * 1e-6, ["x", "y", "z"][i.bias_max_axis], i.off_us as f64 * 1e-6, bias_at.join(","),
                i.corrected, i.skipped, i.ring_dropped, s.yaw_aligns, secs(i.align_t_us),
                d((after.iter().map(|e| e * e).sum::<f64>() / after.len().max(1) as f64).sqrt()), d(after.iter().copied().fold(0.0, f64::max)),
                s.yaw_resets, s.gps_pos_fused, s.gps_resets, s.reinits, i.vert_reinits,
                if i.latch_t_us.is_some() { "yes" } else { "no" }, secs(i.latch_t_us),
                d(rms3(x.hdg)), d(x.hdg.2), x.hdg_flag_steps, rms3(x.ne), x.ne.2, x.ne_flag_steps, x.steps,
                if edges.is_empty() { "none".to_string() } else { edges.join(",") }
            );
            for &(a, b) in windows {
                let r: Vec<_> = x.rows.iter().filter(|r| r.0 >= a && r.0 < b).collect();
                let tl: Vec<f64> = r.iter().filter_map(|r| r.1).collect();
                let hl: Vec<f64> = r.iter().filter_map(|r| r.2).collect();
                let nl: Vec<f64> = r.iter().filter_map(|r| r.3).collect();
                let mx = |v: &[f64]| v.iter().copied().fold(0.0, f64::max);
                let rm = |v: &[f64]| (v.iter().map(|e| e * e).sum::<f64>() / v.len().max(1) as f64).sqrt();
                text += &format!(
                    "\nwindow {:.2}-{:.2} s tilt_rms_deg {:.4} tilt_max_deg {:.4} tilt_n {} truth_tilt_max_deg {:.3} hdg_max_deg {:.3} hdg_n {} ne_rms_m {:.4} ne_max_m {:.4} ne_n {}",
                    a as f64 * 1e-6, b as f64 * 1e-6, d(rm(&tl)), d(mx(&tl)), tl.len(),
                    d(r.iter().map(|r| r.4).fold(0.0, f64::max)), d(mx(&hl)), hl.len(), rm(&nl), mx(&nl), nl.len()
                );
            }
        }
        text
    }
}

/// The simulated world: what the flight step is run against.
struct World {
    plant: Plant,
    backend: NativeBackend,
    imu1: ImuModel,
    imu2: ImuModel,
    slow: SlowSensors,
    wind_ned_ms: [f64; 3],
}

/// The script row in force at `t_us` (the first row is at 0).
fn row_at(script: &[RcRow], t_us: u64) -> RcRow {
    *script.iter().rfind(|r| r.t_us <= t_us).unwrap_or(&script[0])
}

/// What the verdict reads of one flight step.
fn sample(truth: &Truth, out: &StepOut, baro_drift_m: f64) -> Sample {
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
    let target_d = match out.setpoint {
        Setpoint::Altitude { down_m, .. } => f64::from(down_m),
        Setpoint::Position { position_m, .. } => f64::from(position_m[2]),
        _ => f64::NAN,
    };
    let pos_target_ne = match out.setpoint {
        Setpoint::Position { position_m, .. } => [f64::from(position_m[0]), f64::from(position_m[1])],
        _ => [f64::NAN; 2],
    };
    let position_covered = out.nav.valid.covers(
        &Setpoint::Position { position_m: [0.0; 3], velocity_ms: [0.0; 3], heading_rad: 0.0, heading_rate_rads: 0.0 }.requires());
    let mut logged: Vec<f32> = [out.nav.attitude.as_slice(), &out.nav.rates_rads, &out.nav.velocity_ms, &out.nav.position_m].concat();
    match out.setpoint {
        Setpoint::Attitude { attitude, heading_rate_rads, thrust_n } => {
            logged.extend(attitude);
            logged.extend([heading_rate_rads, thrust_n]);
        }
        Setpoint::Altitude { attitude, heading_rate_rads, down_m, velocity_d_ms } => {
            logged.extend(attitude);
            logged.extend([heading_rate_rads, down_m, velocity_d_ms]);
        }
        Setpoint::Position { position_m, velocity_ms, heading_rad, heading_rate_rads } => {
            logged.extend(position_m);
            logged.extend(velocity_ms);
            logged.extend([heading_rad, heading_rate_rads]);
            logged.extend(out.attitude_sp);
        }
        _ => {}
    }
    if let Ok(d) = out.demand {
        logged.push(d.thrust_n);
        logged.extend(d.torque_nm);
    }
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
        settling: truth.settling,
        reaction: truth.reaction,
        selected: mode_code(out.selected),
        mode: mode_code(out.mode),
        vel_d_valid: out.nav.valid.velocity_d,
        pos_d_valid: out.nav.valid.position_d,
        nav_d: f64::from(out.nav.position_m[2]),
        target_d,
        thrust_n: out.demand.map_or(0.0, |d| f64::from(d.thrust_n)),
        finite: logged.iter().all(|x| x.is_finite()),
        baro_drift_m,
        nav_ne: [f64::from(out.nav.position_m[0]), f64::from(out.nav.position_m[1])],
        pos_target_ne,
        position_covered,
        attitude_sp: out.attitude_sp.map(f64::from),
    }
}

/// The scenario's GNSS hold and compass faults (G4), applied to the samples the flight code is given: the last GNSS
/// sample outside a hold and the stuck compass field.
#[derive(Default)]
struct Faults {
    gnss_prev: Option<fw_core::sample::GnssSample>,
    mag_stuck: Option<[f32; 3]>,
}

impl Faults {
    /// `t_us` is scenario time (negative in the pre-roll), `height_m` the truth height above the start.
    fn apply(&mut self, sc: &Scenario, t_us: i64, height_m: f64, s: &mut Sensors) {
        let within = |w: &[(u64, u64)]| t_us >= 0 && w.iter().any(|&(from, to)| t_us as u64 >= from && (t_us as u64) < to);
        if within(&sc.gnss_lost) {
            s.gnss = None;
        } else if let Some(g) = s.gnss.as_mut() {
            match self.gnss_prev.as_ref().filter(|_| within(&sc.gnss_hold)) {
                Some(prev) => {
                    g.reading.lat_e7 = prev.reading.lat_e7;
                    g.reading.lon_e7 = prev.reading.lon_e7;
                    g.reading.alt_mm = prev.reading.alt_mm;
                    g.reading.speed_mm_s = Some(0);
                    g.reading.vel_ned_mm_s = g.reading.vel_ned_mm_s.map(|_| [0; 3]);
                }
                None if within(&sc.gnss_hold) => {}
                None => self.gnss_prev = Some(*g),
            }
            // Hands-off campaign: the reported position moved by north_m, east_m from `at` on.
            if let Some((at, [n, e])) = sc.gnss_jump.filter(|j| t_us >= 0 && t_us as u64 >= j.0) {
                let _ = at;
                if let (Some(la), Some(lo)) = (g.reading.lat_e7.as_mut(), g.reading.lon_e7.as_mut()) {
                    let lat = *la as f64 * 1e-7;
                    let r = 6_378_137.0;
                    *la += (n / r * 180.0 / std::f64::consts::PI * 1e7).round() as i32;
                    *lo += (e / (r * lat.to_radians().cos()) * 180.0 / std::f64::consts::PI * 1e7).round() as i32;
                }
            }
        }
        if let (Some(f), Some(m)) = (sc.mag_fault, s.mag.as_mut()) {
            if t_us >= f.from_us {
                match f.kind {
                    MagFaultKind::Rotate if f.below_m.is_none_or(|b| height_m < b) => {
                        let (a, b) = ((f.axis + 1) % 3, (f.axis + 2) % 3);
                        // HM3b: a ramp of rate_rad_s from `from`, up to deg in size; without a rate the step to deg.
                        let ang = match f.rate_rad_s {
                            None => f.deg_rad,
                            Some(r) => f.deg_rad.signum() * (r * (t_us - f.from_us) as f64 * 1e-6).min(f.deg_rad.abs()),
                        };
                        let (sn, cs) = (ang.sin() as f32, ang.cos() as f32);
                        let v = m.reading.field_ut;
                        m.reading.field_ut[a] = cs * v[a] - sn * v[b];
                        m.reading.field_ut[b] = sn * v[a] + cs * v[b];
                    }
                    MagFaultKind::Rotate => {}
                    MagFaultKind::Stuck => m.reading.field_ut = *self.mag_stuck.get_or_insert(m.reading.field_ut),
                }
            }
        }
    }
}

/// The shadow guidance's slots on the board (`fw_settings::flight::GDN_SLOTS`).
const GDN_SLOTS: usize = 4;

/// `--gdn`: the board's `Stepper::guide` (src/flight.rs) around `Guidance`, owned outside `Flight`. The targets are taken on
/// the first disarmed step that has an origin (the board takes new targets on a disarmed step); arming resets; from then
/// on, every 8th step, `anchor`, `step` and one GDN record. Nothing of it reaches the flight code.
struct GdnShadow {
    offsets: Vec<[f64; 2]>,
    radius_m: f32,
    guidance: Guidance<GDN_SLOTS>,
    was_armed: bool,
    /// The estimator's origin and the time (scenario) the targets were taken, with the targets' GeoPoints.
    taken: Option<(GeoPoint, u64, Vec<GeoPoint>)>,
}

impl GdnShadow {
    fn from_args() -> Option<GdnShadow> {
        let offsets = GDN_TARGETS.lock().unwrap().clone();
        (!offsets.is_empty()).then(|| GdnShadow {
            offsets,
            radius_m: GDN_RADIUS.get().copied().unwrap_or(2.0),
            guidance: Guidance::new(),
            was_armed: false,
            taken: None,
        })
    }

    fn guide<E: Estimator, C: Controller, A: Allocator>(&mut self, flight: &Flight<E, C, A>, out: &StepOut, step: u32, log: &mut Log) {
        let origin = flight.estimator().origin();
        if self.taken.is_none() && !out.armed {
            if let Some(o) = origin {
                let base = Origin { lat_e7: o.lat_e7, lon_e7: o.lon_e7, alt_mm: o.alt_mm };
                let targets: Vec<GeoPoint> = self
                    .offsets
                    .iter()
                    .map(|&[n, e]| {
                        let (lat_e7, lon_e7, alt_mm) = geo_from_ned(&base, [n, e, 0.0]);
                        GeoPoint { lat_e7, lon_e7, alt_mm }
                    })
                    .collect();
                self.guidance.set_targets(&targets, self.radius_m);
                self.taken = Some((o, out.t_us, targets));
            }
        }
        if out.armed && !self.was_armed {
            self.guidance.reset();
        }
        self.was_armed = out.armed;
        if self.taken.is_none() || step % 8 != 0 {
            return;
        }
        let nav = out.nav;
        let speed = flight.speed_xy_in_force();
        self.guidance.anchor(&nav);
        let g = self.guidance.step(&nav, origin, speed, out.t_us);
        let (position, velocity) = match g.setpoint {
            Some(Setpoint::Position { position_m, velocity_ms, .. }) => ([position_m[0], position_m[1]], [velocity_ms[0], velocity_ms[1]]),
            _ => ([0.0; 2], [0.0; 2]),
        };
        log.gdn(out.t_us, g.active, g.status, g.target_ne, position, velocity, g.dist_m, g.bearing_rad);
    }

    /// The `gdn` line: where the estimator's origin is in the sim world (north, east, m), and each target's NE in the sim
    /// world, from its own geodetic point.
    fn line(&self, log: &Log) -> String {
        let Some((o, t_us, targets)) = &self.taken else {
            return "gdn targets never taken (no estimator origin on a disarmed step), 0 records".to_string();
        };
        let Some(sim) = GDN_SIM_ORIGIN.get() else { return "gdn no sim origin".to_string() };
        let at = |p: &GeoPoint| ned_from_geo(sim, p.lat_e7, p.lon_e7, p.alt_mm);
        let eo = at(o);
        let ts: Vec<String> = targets.iter().map(|p| { let v = at(p); format!("{:.3},{:.3}", v[0], v[1]) }).collect();
        format!(
            "gdn radius_m {} taken_t_s {:.3} est_origin_in_sim_ne_m {:.3},{:.3} targets_true_ne_m {} records {}",
            self.radius_m, *t_us as f64 * 1e-6, eo[0], eo[1], ts.join(" "), log.counts.gdn
        )
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
    mut shadow: Option<&mut (dyn Slot + 'static)>,
    mut steer: Option<&mut Steer>,
    preroll_ticks: u64,
) -> (u64, u64, Trace, Vec<HoStep>) {
    let mut ho: Vec<HoStep> = Vec::new();
    // The newest StepOut's time and command, as the ESC task sees them; none yet counts as (0, Stop).
    let mut newest = (0, MotorCommand::STOP);
    let mut levels = [0u16; 4];
    let mut pending: Option<RcSample> = None;
    let mut trace = Trace::default();
    // The barometer model's drift when the flight code is first given a barometer sample (the pre-roll's included).
    let mut drift0: Option<f64> = None;
    let mut mode_log = ModeLog::new();
    let mut gdn = GdnShadow::from_args();
    let mut flight_steps: u64 = 0;
    let mut ran = 0;
    // The scenario's GNSS hold and compass faults (G4): the last GNSS sample outside a hold, the stuck compass field.
    let mut faults = Faults::default();
    // Contact at any tick since the last stick report: what the pilot's eye catches between reports (Amendment B1).
    let mut touched = false;
    // Contact at any tick since the last flight step, for the shadow's airborne time.
    let mut contact = false;
    // The pre-roll (decision-nav-eskf D5): the flight code runs on the board clock 0..off with the plant pinned at the
    // initial state; the scenario then starts at board time `off`. The flight code, the shadow and the sensors' stamps
    // see board time = scenario time + off; the log, the verdict trace and the ESC rule see scenario time.
    let off = preroll_ticks * TICK_US;
    flight.set_notch(NOTCH.load(std::sync::atomic::Ordering::Relaxed), MOTOR_POLES);
    flight.set_fs_hold_en(flag(&FS_HOLD));
    if let Some(&v) = PILOT_SPEED.get() {
        assert!(flight.set_pilot_speed_xy(v), "--pilot-speed {v} refused by Flight::set_pilot_speed_xy");
    }
    if preroll_ticks > 0 {
        let mut pinned = world.backend.truth().clone();
        let row = row_at(&scenario.rc_script, 0);
        let mut pending: Option<RcSample> = None;
        let mut contact = false;
        for k in 1..=preroll_ticks {
            pinned.t_us = k * TICK_US;
            world.slow.tick(&pinned);
            if (k - 1).is_multiple_of(RC_TICKS) {
                let sticks = Sticks { roll: row.roll, pitch: row.pitch, throttle: row.throttle, yaw: row.yaw };
                pending = Some(RcSample { sticks, arm: row.arm, switch6: row.mode, t_us: pinned.t_us - TICK_US });
            }
            if pinned.t_us.is_multiple_of(IMU_PERIOD_US) {
                let s1 = world.imu1.sample(&pinned);
                let s2 = world.imu2.sample(&pinned);
                let (baro, mag, gnss) = world.slow.take();
                let mut sensors = Sensors { imu1: s1, imu2: Some(s2), baro, mag, gnss, external: None };
                faults.apply(scenario, pinned.t_us as i64 - off as i64, -pinned.position_m[2], &mut sensors);
                if baro.is_some() && drift0.is_none() {
                    drift0 = Some(world.slow.baro.drift_m());
                }
                if flag(&FS_HOLD) {
                    flight.set_last_radio_heartbeat_us((!flag(&NO_HEARTBEAT)).then_some(sensors.imu1.t_us));
                }
                TRUTH_YAW.store(yaw_of(pinned.attitude).to_bits(), std::sync::atomic::Ordering::Relaxed);
                let out = flight.step(&FlightInputs { sensors, rc: pending.take(), erpm: erpm_lines(world.plant.rotor_speeds(), pinned.t_us) });
                if flag(&FS_HOLD) {
                    flight.set_heading_ok(out.nav.valid.heading);
                }
                if flag(&X_INF) {
                    flight.sim_clear_track_error();
                }
                if let Some(sh) = shadow.as_deref_mut() {
                    sh.feed(&sensors, &out, &pinned, world.slow.baro.drift_m(), std::mem::replace(&mut contact, false));
                }
            }
        }
        // A sample made at the last pre-roll ticks, not yet taken, would carry a pre-roll stamp into the shifted clock.
        let _ = world.slow.take();
    }
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
                    let view = PilotView::see(world.backend.truth(), touched);
                    touched = false;
                    let Some(report) = p.report(now, view) else { break };
                    log.pilot(&report);
                    trace.pilot.push(report);
                    report.row
                }
            };
            // Only the newest report reaches the next flight step, once.
            let lost = scenario.rc_lost.iter().any(|&(from, to)| now >= from && now < to);
            if !lost {
                let given = row;
                let sticks = Sticks { roll: given.roll, pitch: given.pitch, throttle: given.throttle, yaw: given.yaw };
                pending = Some(RcSample { sticks, arm: given.arm, switch6: given.mode, t_us: now + off });
                trace.rc_us.push(now);
                log.rc(now, [given.roll, given.pitch, given.throttle, given.yaw], given.arm);
            }
        }
        // The scenario's wind steps (P7): the mean wind plus every step whose time has come.
        let mut wind = world.wind_ned_ms;
        for &(_, delta) in scenario.wind_steps.iter().filter(|(at_us, _)| now >= *at_us) {
            for k in 0..3 {
                wind[k] += delta[k];
            }
        }
        if let Some((at, line, scale)) = scenario.motor_thrust.filter(|m| now >= m.0) {
            let _ = at;
            world.plant.set_thrust_scale(line, scale);
        }
        let mut wrench = world.plant.step(levels, world.backend.truth(), wind, TICK_US);
        for d in scenario.disturbances.iter().filter(|d| now >= d.from_us && now < d.to_us) {
            for i in 0..3 {
                wrench.torque_nm[i] += d.torque_nm[i];
            }
        }
        let was_on_ground = world.backend.truth().on_ground;
        let truth = world.backend.step(&wrench, TICK_US).clone();
        ran += 1;
        touched |= truth.on_ground;
        contact |= truth.on_ground;
        world.slow.tick(&truth);
        if truth.on_ground && !was_on_ground {
            trace.touchdowns.extend(world.backend.touchdown().copied());
        } else if was_on_ground && !truth.on_ground {
            trace.liftoffs.push(truth.t_us);
        }

        if truth.t_us.is_multiple_of(IMU_PERIOD_US) {
            let s1 = world.imu1.sample(&truth);
            let s2 = world.imu2.sample(&truth);
            let (baro, mag, gnss) = world.slow.take();
            let sensors = Sensors { imu1: s1, imu2: Some(s2), baro, mag, gnss, external: None };
            let mut sensors_f = sensors;
            sensors_f.imu1.t_us += off;
            if let Some(i) = sensors_f.imu2.as_mut() {
                i.t_us += off;
            }
            if scenario.baro_lost.iter().any(|&(from, to)| truth.t_us >= from && truth.t_us < to) {
                sensors_f.baro = None;
            }
            faults.apply(scenario, truth.t_us as i64, -truth.position_m[2], &mut sensors_f);
            if sensors_f.baro.is_some() && drift0.is_none() {
                drift0 = Some(world.slow.baro.drift_m());
            }
            if let Some(b) = sensors_f.baro.as_mut() {
                b.t_us += off;
            }
            if let Some(m) = sensors_f.mag.as_mut() {
                m.t_us += off;
            }
            if let Some(g) = sensors_f.gnss.as_mut() {
                g.t_us += off;
            }
            if flag(&FS_HOLD) {
                flight.set_last_radio_heartbeat_us((!flag(&NO_HEARTBEAT)).then_some(sensors_f.imu1.t_us));
            }
            TRUTH_YAW.store(yaw_of(truth.attitude).to_bits(), std::sync::atomic::Ordering::Relaxed);
            let out_f = flight.step(&FlightInputs { sensors: sensors_f, rc: pending.take(), erpm: erpm_lines(world.plant.rotor_speeds(), sensors_f.imu1.t_us) });
            if flag(&FS_HOLD) {
                flight.set_heading_ok(out_f.nav.valid.heading);
            }
            let anchor = flight.sim_anchor();
            if flag(&X_INF) {
                flight.sim_clear_track_error();
            }
            ho.push(HoStep { state: flight.hands_off_state().code(), tf: flight.sim_track_fault(), hdg: out_f.nav.valid.heading, anchor,
                x_m: fw_flight::hands_off::track_fault_m(flight.speed_xy_in_force()), vne: out_f.nav.valid.velocity_ne,
                pne: out_f.nav.valid.position_ne, nav_v: [out_f.nav.velocity_ms[0], out_f.nav.velocity_ms[1]],
                gated: GATED_STEP.swap(0, std::sync::atomic::Ordering::Relaxed) });
            let mut out = out_f;
            out.t_us -= off;
            out.nav.t_us -= off;
            let ground = std::mem::replace(&mut contact, false);
            newest = (out.t_us, out.motors);
            if let Some(sh) = shadow.as_deref_mut() {
                let mut truth_f = truth.clone();
                truth_f.t_us += off;
                sh.feed(&sensors_f, &out_f, &truth_f, world.slow.baro.drift_m(), ground);
            }
            if let Some(st) = steer.as_deref_mut() {
                st.feed(&out, &truth);
            }
            flight_steps += 1;

            // The firmware's rule (`log_due`): IMU1 every step; NAV, SETP, DMD, MOT every step armed, every 8th disarmed.
            let due = log_due((flight_steps - 1) as u32, out.armed);
            if due.imu {
                log.imu(truth.t_us, 0, &s1.reading);
            }
            log.imu(truth.t_us, 1, &s2.reading);
            log.truth(&truth);
            log.rotors(truth.t_us, levels, world.plant.rotor_speeds());
            if due.block {
                log.step(&out);
            }
            // The board's MODE (at the first step and at every change of the selected or effective mode) and SPN (the
            // step block's cadence, only while the setpoint is Altitude), src/flight.rs.
            if mode_log.pending(&out) {
                log.mode(&out);
                mode_log.done(&out);
            }
            if due.block && matches!(out.setpoint, Setpoint::Altitude { .. } | Setpoint::Position { .. }) {
                log.spn(&out);
            }
            if pilot.is_some() {
                log.ground(&truth);
            }
            if let Some(g) = gdn.as_mut() {
                g.guide(&flight, &out, (flight_steps - 1) as u32, log);
            }
            trace.steps.push(sample(&truth, &out, world.slow.baro.drift_m()));
        }
    }
    trace.baro_drift0_m = drift0.unwrap_or(0.0);
    if let Some(g) = &gdn {
        println!("{}", g.line(log));
    }
    (ran, flight_steps, trace, ho)
}

/// Hands-off campaign: what the run records of the state machine after every flight step (aligned with `trace.steps`):
/// the state code, the H12 track fault, the estimator's heading flag.
#[derive(Clone, Copy)]
struct HoStep {
    state: u8,
    tf: bool,
    hdg: bool,
    /// H13: the anchor (nav NE) and the braking allowance b after the step (read before the rule-off control clears it);
    /// X in force.
    anchor: Option<([f32; 2], f32)>,
    x_m: f32,
    /// For the `MARV_HO_DUMP` per-step file: Valid.velocity_ne, Valid.position_ne, the nav NE velocity, the gated GNSS scalars.
    vne: bool,
    pne: bool,
    nav_v: [f32; 2],
    gated: u32,
}

/// The `hands_off` verdict line (hands-off campaign). Times are scenario seconds. The hold point is the Position target at the
/// first HOLD step moved into the truth frame by that step's offset between truth and the estimate (truth N, E at that step
/// when the setpoint is not Position); every distance is the TRUE horizontal distance from it. The reference is the first
/// scheduled fault at or after 0 (yaw inject or hold, compass fault, GNSS loss or jump, thrust loss, wind step), else HOLD
/// entry. land_by: what was below T3' when the 1.0 s debounce that ended in LAND began (tf track fault, hdg heading flag, ne
/// position flags), or link when T3' held to LAND (the 10 s link rule).
fn ho_line(scenario: &Scenario, trace: &Trace, ho: &[HoStep]) -> String {
    let st = &trace.steps;
    let n = st.len().min(ho.len());
    if n == 0 {
        return "hands_off no steps".to_string();
    }
    let secs = |t: u64| t as f64 * 1e-6;
    let opt = |t: Option<u64>| t.map_or("none".to_string(), |t| format!("{:.3}", secs(t)));
    let num = |v: Option<f64>, p: usize| v.map_or("none".to_string(), |v| format!("{v:.p$}"));
    let letter = |c: u8| ["?", "P", "T", "H", "A", "R", "L", "S", "B"].get(usize::from(c)).copied().unwrap_or("?");
    let mut hist: Vec<String> = Vec::new();
    for k in 0..n {
        if k == 0 || ho[k].state != ho[k - 1].state {
            hist.push(format!("{}@{:.3}", letter(ho[k].state), secs(st[k].t_us)));
        }
    }
    // The hands-off entry: the first HOLD step, else (stick loss straight into LAND or BLIND) the first hands-off step.
    let k_hold = (0..n).find(|&k| ho[k].state == 3).or_else(|| (0..n).find(|&k| !matches!(ho[k].state, 1 | 7)));
    let hp = k_hold.map(|k| {
        let p = &st[k];
        if p.pos_target_ne.iter().all(|v| v.is_finite()) {
            [p.pos_target_ne[0] + p.truth_pos[0] - p.nav_ne[0], p.pos_target_ne[1] + p.truth_pos[1] - p.nav_ne[1]]
        } else {
            [p.truth_pos[0], p.truth_pos[1]]
        }
    });
    let d = |k: usize| hp.map(|h| (st[k].truth_pos[0] - h[0]).hypot(st[k].truth_pos[1] - h[1]));
    let k0 = k_hold.unwrap_or(n);
    let first = |from: usize, f: &dyn Fn(usize) -> bool| (from..n).find(|&k| f(k));
    let k_tf = first(k0, &|k| ho[k].tf);
    let k_land = first(k0, &|k| ho[k].state == 6 || ho[k].state == 8);
    let k_blind = first(k0, &|k| ho[k].state == 8);
    let k_stop = first(k0, &|k| ho[k].state == 7);
    let k_disarm = first(k0, &|k| !st[k].armed);
    let t3 = |k: usize| st[k].position_covered && ho[k].hdg && !ho[k].tf;
    let land_by = k_land.map(|kl| {
        let mut k = kl;
        while k > k0 && !t3(k - 1) {
            k -= 1;
        }
        if k == kl {
            "link".to_string()
        } else {
            let mut w = Vec::new();
            if ho[k].tf {
                w.push("tf");
            }
            if !ho[k].hdg {
                w.push("hdg");
            }
            if !st[k].position_covered {
                w.push("ne");
            }
            format!("{}@{:.3}", w.join("+"), secs(st[k].t_us))
        }
    });
    let mut refs: Vec<u64> = Vec::new();
    refs.extend(scenario.yaw_inject.map(|y| y.at_us));
    refs.extend(scenario.yaw_hold.map(|y| y.at_us));
    refs.extend(scenario.mag_fault.filter(|f| f.from_us >= 0).map(|f| f.from_us as u64));
    refs.extend(scenario.gnss_lost.iter().map(|w| w.0));
    refs.extend(scenario.gnss_jump.map(|j| j.0));
    refs.extend(scenario.motor_thrust.map(|m| m.0));
    refs.extend(scenario.wind_steps.iter().map(|w| w.0));
    let t_ref = refs.into_iter().min().or(k_hold.map(|k| st[k].t_us));
    let dt = |k: Option<usize>| k.zip(t_ref).map(|(k, r)| secs(st[k].t_us) - secs(r));
    let dmax = |from: usize, to: usize| (from..to).filter_map(|k| d(k).map(|v| (v, k))).fold(None, |a: Option<(f64, usize)>, b| match a {
        Some(a) if a.0 >= b.0 => Some(a),
        _ => Some(b),
    });
    let all = dmax(k0, n);
    let hold_max = dmax(k0, k_land.unwrap_or(n));
    let trk_max = (k0..k_land.unwrap_or(n)).filter(|&k| ho[k].state == 3)
        .map(|k| (st[k].nav_ne[0] - st[k].pos_target_ne[0]).hypot(st[k].nav_ne[1] - st[k].pos_target_ne[1]))
        .filter(|v| v.is_finite()).fold(None, |a: Option<f64>, b| Some(a.map_or(b, |a| a.max(b))));
    let drop = |f: &dyn Fn(usize) -> bool| first(k0, &|k| !f(k)).map(|k| st[k].t_us);
    let hdg_drop = drop(&|k| ho[k].hdg);
    let ne_drop = drop(&|k| st[k].position_covered);
    let e_deg = |k: usize| wrap_pi(yaw_of(st[k].nav_q) - yaw_of(st[k].truth_q)).to_degrees();
    let t_hold = k_hold.map(|k| st[k].t_us).unwrap_or(0);
    let tds: Vec<&marv_sim::backend::Touchdown> = trace.touchdowns.iter().filter(|t| t.t_us >= t_hold).collect();
    let td = k_land.and_then(|kl| tds.iter().find(|t| t.t_us >= st[kl].t_us).copied()).or(tds.first().copied());
    let k_td = td.map(|t| st[..n].partition_point(|p| p.t_us < t.t_us).min(n - 1));
    let lift_after = td.map_or(0, |t| trace.liftoffs.iter().filter(|&&l| l > t.t_us).count());
    let mot_off = trace.esc.last().is_some_and(|e| e.1 == [0; 4]);
    let hp_off = k_hold.zip(hp).map(|(k, h)| (st[k].truth_pos[0] - h[0]).hypot(st[k].truth_pos[1] - h[1]));
    let lost = scenario.rc_lost.first().map(|w| w.0);
    let e_max = (k0..k_land.unwrap_or(n)).map(|k| e_deg(k).abs()).fold(None, |a: Option<f64>, b| Some(a.map_or(b, |a| a.max(b))));
    let k_end = k_stop.unwrap_or(n);
    // The Position target's walk in HOLD (nav frame) from the first HOLD step's target, and the time the target moved between
    // consecutive HOLD steps (unlocked: the shaper's target follows the position while braking); the nav gap |(nav - truth)
    // - (nav - truth) at HOLD entry| in HOLD (I-H3).
    let k_end_hold = k_land.unwrap_or(n);
    let r0 = k_hold.map(|k| st[k].pos_target_ne);
    let r_walk = r0.and_then(|r| (k0..k_end_hold).filter(|&k| ho[k].state == 3).map(|k| (st[k].pos_target_ne[0] - r[0]).hypot(st[k].pos_target_ne[1] - r[1]))
        .filter(|v| v.is_finite()).fold(None, |a: Option<f64>, b| Some(a.map_or(b, |a| a.max(b)))));
    let r_moving_s = (k0 + 1..k_end_hold).filter(|&k| ho[k].state == 3 && st[k].pos_target_ne != st[k - 1].pos_target_ne).count() as f64 * IMU_PERIOD_US as f64 * 1e-6;
    let g0 = k_hold.map(|k| [st[k].nav_ne[0] - st[k].truth_pos[0], st[k].nav_ne[1] - st[k].truth_pos[1]]);
    let nav_gap = g0.and_then(|g| (k0..k_end_hold).map(|k| (st[k].nav_ne[0] - st[k].truth_pos[0] - g[0]).hypot(st[k].nav_ne[1] - st[k].truth_pos[1] - g[1]))
        .fold(None, |a: Option<f64>, b| Some(a.map_or(b, |a| a.max(b)))));
    // H13: the anchor in the truth frame (moved by the truth - nav offset of the step it was first seen), true and nav
    // distances from it; which distance tripped; the stopping distance (true |p - A| at the first step after the anchor
    // with true horizontal speed < 0.3 m/s); position-flag drops shorter than 1.0 s while hands-off before LAND.
    let k_a = (0..n).find(|&k| ho[k].anchor.is_some());
    let an = k_a.and_then(|k| ho[k].anchor.map(|(a, b)| (k, [f64::from(a[0]), f64::from(a[1])], f64::from(b))));
    let a_true = an.map(|(k, a, _)| [a[0] + st[k].truth_pos[0] - st[k].nav_ne[0], a[1] + st[k].truth_pos[1] - st[k].nav_ne[1]]);
    let da_t = |k: usize| a_true.map(|a| (st[k].truth_pos[0] - a[0]).hypot(st[k].truth_pos[1] - a[1]));
    let da_n = |k: usize| an.map(|(_, a, _)| (st[k].nav_ne[0] - a[0]).hypot(st[k].nav_ne[1] - a[1]));
    let x_m = f64::from(ho[n - 1].x_m);
    let trip_by = k_tf.map(|k| match (da_n(k), an) {
        (Some(d), Some((_, _, b))) if st[k].position_covered && d > x_m + b => "anchor",
        _ => "target",
    });
    let ka0 = k_a.unwrap_or(n);
    let fold_max = |from: usize, to: usize, f: &dyn Fn(usize) -> Option<f64>| (from..to).filter_map(|k| f(k)).fold(None, |a: Option<f64>, b| Some(a.map_or(b, |a| a.max(b))));
    let da_max_pre = fold_max(ka0, k_land.unwrap_or(n), &da_t);
    let da_max = fold_max(ka0, n, &da_t);
    let dan_max_pre = fold_max(ka0, k_land.unwrap_or(n), &|k| if st[k].position_covered { da_n(k) } else { None });
    let k_stopped = (ka0..n).find(|&k| st[k].truth_vel[0].hypot(st[k].truth_vel[1]) < 0.3);
    let v_entry = k_a.map(|k| st[k].truth_vel[0].hypot(st[k].truth_vel[1]));
    let mut short_drops = 0;
    let mut run_from: Option<u64> = None;
    for k in k0..k_land.unwrap_or(n) {
        if !st[k].position_covered {
            run_from.get_or_insert(st[k].t_us);
        } else if let Some(f) = run_from.take() {
            short_drops += usize::from(st[k].t_us - f < 1_000_000);
        }
    }
    let hop = k_td.map(|kt| (kt..k_end).map(|k| -st[k].truth_pos[2]).fold(0.0, f64::max));
    format!(
        "hands_off hb {} xinf {} lost_s {} hold_s {} hp_off_m {} t_ref_s {} states {} tf_s {} land_s {} land_by {} blind_s {} td_s {} stop_s {} disarm_s {} \
dt_tf_s {} dt_land_s {} d_tf_m {} d_land_m {} d_td_m {} d_stop_m {} d_max_m {} d_max_s {} d_hold_max_m {} trk_max_m {} hdg_drop_s {} ne_drop_s {} \
e_land_deg {} h_land_m {} h_stop_m {} vz_stop_ms {} td_vz_ms {} td_vh_ms {} td_tilt_deg {} n_td {} liftoff_after_td {} mot_off {} armed_end {} d_end_m {} h_end_m {} e_hold_deg {} e_max_hold_deg {} hop_after_td_m {} r_walk_max_m {} r_moving_s {:.1} nav_gap_max_m {} \
anchor_s {} b_m {} x_m {:.2} v_entry_ms {} trip_by {} dA_tf_m {} dAn_tf_m {} dA_land_m {} dAn_land_m {} dA_max_pre_land_m {} dAn_max_pre_land_m {} dA_max_m {} \
dA_td_m {} stop_dist_m {} stopped_s {} ne_short_drops {}",
        if flag(&NO_HEARTBEAT) { "absent" } else { "fed" }, u8::from(flag(&X_INF)), opt(lost), opt(k_hold.map(|k| st[k].t_us)), num(hp_off, 3),
        opt(t_ref), hist.join(","), opt(k_tf.map(|k| st[k].t_us)), opt(k_land.map(|k| st[k].t_us)), land_by.unwrap_or("none".to_string()),
        opt(k_blind.map(|k| st[k].t_us)), opt(td.map(|t| t.t_us)), opt(k_stop.map(|k| st[k].t_us)), opt(k_disarm.map(|k| st[k].t_us)),
        num(dt(k_tf), 3), num(dt(k_land), 3), num(k_tf.and_then(d), 2), num(k_land.and_then(d), 2), num(k_td.and_then(d), 2),
        num(k_stop.and_then(d), 2), num(all.map(|a| a.0), 2), opt(all.map(|a| st[a.1].t_us)), num(hold_max.map(|a| a.0), 2), num(trk_max, 2),
        opt(hdg_drop), opt(ne_drop), num(k_land.map(e_deg), 1), num(k_land.map(|k| -st[k].truth_pos[2]), 2),
        num(k_stop.map(|k| -st[k].truth_pos[2]), 3), num(k_stop.map(|k| st[k].truth_vel[2]), 3),
        num(td.map(|t| t.velocity_ms[2]), 3), num(td.map(|t| t.velocity_ms[0].hypot(t.velocity_ms[1])), 3), num(td.map(|t| t.tilt_rad.to_degrees()), 2),
        tds.len(), lift_after, if mot_off { "yes" } else { "no" }, st[n - 1].armed, num(d(n - 1), 2), format!("{:.2}", -st[n - 1].truth_pos[2]),
        num(k_hold.map(e_deg), 1), num(e_max, 1), num(hop, 3), num(r_walk, 2), r_moving_s, num(nav_gap, 2),
        opt(k_a.map(|k| st[k].t_us)), num(an.map(|a| a.2), 3), x_m, num(v_entry, 2), trip_by.unwrap_or("none"),
        num(k_tf.and_then(da_t), 2), num(k_tf.and_then(da_n), 2), num(k_land.and_then(da_t), 2), num(k_land.and_then(da_n), 2),
        num(da_max_pre, 2), num(dan_max_pre, 2), num(da_max, 2), num(k_td.and_then(da_t), 2), num(k_stopped.and_then(da_t), 2),
        opt(k_stopped.map(|k| st[k].t_us)), short_drops
    )
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
    /// The shadow's line, with `--shadow`.
    shadow: Option<String>,
    /// The `steer` line, with `--estimator`.
    steer: Option<String>,
    /// The `hands_off` line, with `--fs-hold`.
    hands_off: Option<String>,
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
    keep: bool, shadow: Option<ShadowKind>, steer_cfg: Option<SteerCfg>, preroll: Option<u64>, windows: &[(u64, u64)]) -> Result<Outcome, String> {
    let (map, air, ground) = cell.map_or((Map::default(), Air::default(), Ground::default()), |r| (r.map, r.air, r.ground));
    let sloped;
    let scenario = match ground.roll_pitch_deg {
        Some(roll_pitch) => {
            sloped = scenario.clone().on_slope(roll_pitch)?;
            &sloped
        }
        None => scenario,
    };
    let lengthened;
    let scenario = match (GROUND_S.get(), scenario.phases.first()) {
        (Some(&g), Some(first)) if matches!(first.end, marv_sim::scenario::PhaseEnd::After { .. }) => {
            let mut sc = scenario.clone();
            let marv_sim::scenario::PhaseEnd::After { us } = sc.phases[0].end else { unreachable!() };
            let new = (g * 1e6).round() as u64;
            sc.phases[0].end = marv_sim::scenario::PhaseEnd::After { us: new };
            sc.duration_s += (new as f64 - us as f64) * 1e-6;
            lengthened = sc;
            &lengthened
        }
        _ => scenario,
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
        settling: false,
        reaction: Reaction::default(),
    };
    let backend = NativeBackend::with_start(&plant_card, initial, scenario.start)?.with_settle(scenario.settle_s);
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
    let slow = SlowSensors::new(card, env, scenario.gnss_rate_hz, seed)?
        .with_outputs(scenario.mag_expected, scenario.gnss_velocity, scenario.baro_motion);
    let slow = match scenario.gnss_latency_s {
        Some(l) => slow.with_gnss_latency_s(l),
        None => slow,
    };
    // Wind for this step is the mean only; gusts come in step 7.
    let wind = match air.wind_ned_ms {
        Some(w) => w,
        None => env.wind_mean_ned.v.ok_or("wind.mean_ned is \"ask\"")?,
    };

    if !GDN_TARGETS.lock().unwrap().is_empty() {
        let _ = GDN_SIM_ORIGIN.set(Origin::from_env(env)?);
    }
    let mut log = Log::with_header();
    log.msg(0, &format!("marv-sim {}", env!("CARGO_PKG_VERSION")));
    log.msg(0, &firmware_text());
    log.msg(0, &format!("card sha256 {}", &card.sha256[..16]));
    log.msg(0, &format!("scenario {}", scenario.name));
    log.msg(0, &format!("seed {}", seed));
    if let Some(n) = preroll {
        log.msg(0, &format!(
            "preroll {} s: flight code stepped with the plant pinned before scenario time 0; its rows are not in this log; \
every row's time is scenario time (the flight code's clock was scenario time + {} us)", n * TICK_US / 1000 / 1000, n * TICK_US));
    }
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
    // The sim has no boot sequence and no core 0, so the runner plays core 0's part: it sets the arming permit once,
    // before the first flight step (the board sets it after its boot, when IMU1 and the ESCs are healthy and nothing
    // is running; the pre-arm rule is checked on the board, not here).
    handshake.set_permit(true);
    let mut tuning = Tuning::DEFAULT;
    if let Some(s) = NOAID_SIGMA.get() {
        tuning.nav_noaid_sigma_m = *s;
    }
    for (name, v) in TUNING_OVERRIDE.lock().unwrap().iter() {
        match name.as_str() {
            "psc_posz_p" => tuning.psc_posz_p = *v,
            "psc_velz_p" => tuning.psc_velz_p = *v,
            "nav_baro_sigma_m" => tuning.nav_baro_sigma_m = *v,
            "notch_harmonics" => tuning.notch_harmonics = *v as usize,
            "notch_min_hz" => tuning.notch_min_hz = *v,
            "notch_fade_hz" => tuning.notch_fade_hz = *v,
            _ => tuning.psc_velz_i = *v,
        }
    }
    let mut world = World { plant, backend, imu1, imu2, slow, wind_ned_ms: wind };
    let mut shadow: Option<Box<dyn Slot>> = match shadow {
        None => None,
        Some(ShadowKind::MahonyBaro) => Some(Box::new(Shadow::new(&tuning))),
        Some(ShadowKind::Eskf) => Some(Box::new(EskfShadow::new(&tuning, env)?)),
    };
    let mut steer = steer_cfg.map(|_| Steer::default());
    let info = Rc::new(StdCell::new((0.0f32, 0u32)));
    let info3 = Rc::new(RefCell::new(Info3 { off_us: preroll.unwrap_or(0) * TICK_US, ..Info3::default() }));
    if let (Some(st), Some(SteerCfg { est: EstKind::Eskf3 })) = (steer.as_mut(), steer_cfg) {
        st.ext = Some(Ext { info: info3.clone(), sim_origin: Origin::from_env(env)?, hdg: (0, 0.0, 0.0), ne: (0, 0.0, 0.0),
            hdg_flag_steps: 0, ne_flag_steps: 0, steps: 0, rows: Vec::new(), ne_prev: false, ne_drops: Vec::new() });
    }
    if (scenario.yaw_inject.is_some() || scenario.yaw_hold.is_some()) && !matches!(steer_cfg, Some(SteerCfg { est: EstKind::Eskf3 })) {
        return Err(format!("scenario {}: yaw_inject needs --estimator eskf3", scenario.name));
    }
    let (ran, flight_steps, mut trace, ho) = match steer_cfg {
        None | Some(SteerCfg { est: EstKind::Mahony, .. }) => {
            let flight = Flight::new(Mahony::new(&tuning), CascadedPid::new(&tuning, IMU_RATE_HZ), CardAllocator::new(), &tuning, handshake);
            let flight = apply_param_gains(flight)?;
            run(flight, &mut world, scenario, pilot.as_mut(), ticks, &mut log, shadow.as_deref_mut(), steer.as_mut(), preroll.unwrap_or(0))
        }
        Some(SteerCfg { est: EstKind::Eskf3 }) => {
            let mut t = tuning;
            let f = env.magnetic_field_ned.v.ok_or("magnetic_field.ned is \"ask\"")?;
            t.nav_mag_declination_rad = f[1].atan2(f[0]) as f32;
            let core: &'static mut Eskf = Box::leak(Box::new(Eskf::new(&t).map_err(|e| format!("Eskf: {e:?}"))?));
            let vert: &'static mut Eskf = Box::leak(Box::new(Eskf::with_aiding(&t, Aiding::ImuBaro).map_err(|e| format!("Eskf (vertical): {e:?}"))?));
            let h = EskfSteer::new(core, &t).map_err(|e| format!("EskfSteer: {e:?}"))?;
            let mix = SteerMix::new(h, vert);
            let inject = scenario.yaw_inject.map(|y| (y.at_us + preroll.unwrap_or(0) * TICK_US, y.rad as f32));
            let hold = scenario.yaw_hold.map(|y| (y.at_us + preroll.unwrap_or(0) * TICK_US, y.rad as f32));
            let est = Mix3 { mix, info: info3.clone(), inject, hold, last_yaw: None };
            let flight = Flight::new(est, CascadedPid::new(&tuning, IMU_RATE_HZ), CardAllocator::new(), &tuning, handshake);
            let flight = apply_param_gains(flight)?;
            run(flight, &mut world, scenario, pilot.as_mut(), ticks, &mut log, shadow.as_deref_mut(), steer.as_mut(), preroll.unwrap_or(0))
        }
        Some(cfg) => {
            let mut t = tuning;
            let f = env.magnetic_field_ned.v.ok_or("magnetic_field.ned is \"ask\"")?;
            t.nav_mag_declination_rad = f[1].atan2(f[0]) as f32;
            let aiding = if cfg.est == EstKind::Eskf1 { Aiding::Imu } else { Aiding::ImuBaro };
            let eskf = Eskf::with_aiding(&t, aiding).map_err(|e| format!("Eskf: {e:?}"))?;
            let est = Steering { eskf, info: info.clone() };
            let flight = Flight::new(est, CascadedPid::new(&tuning, IMU_RATE_HZ), CardAllocator::new(), &tuning, handshake);
            let flight = apply_param_gains(flight)?;
            run(flight, &mut world, scenario, pilot.as_mut(), ticks, &mut log, shadow.as_deref_mut(), steer.as_mut(), preroll.unwrap_or(0))
        }
    };

    trace.weight_n = card.mass.mass.v.unwrap_or(f64::NAN) * env.gravity.v.unwrap_or(f64::NAN);
    if let Some(p) = &pilot {
        trace.phases = p.timeline().to_vec();
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
    let (checks, mut notes) = judge(&scenario.verdict, &trace);
    if let Some(st) = &steer {
        // One note per backend touchdown (the verdict's touchdown is the first after its first rise through
        // airborne_m: verdict.rs takeoff_land, t_td; the INFO note names it): the truth state before the hold
        // (backend.rs Touchdown) and the steering estimator's tilt error over the 2 s before it.
        for d in &trace.touchdowns {
            let w: Vec<f64> = st.hist.iter().filter(|(t, _)| *t <= d.t_us && *t + 2_000_000 > d.t_us).map(|h| h.1).collect();
            let deg = f64::to_degrees;
            notes.push(format!(
                "TD t_s {:.4} tilt_err_max_2s_deg {:.4} tilt_err_mean_2s_deg {:.4} n {} truth_rate_dps {:.3} truth_tilt_deg {:.3} \
vh_ms {:.4} vz_ms {:.4}",
                d.t_us as f64 * 1e-6, deg(w.iter().copied().fold(0.0, f64::max)),
                deg(w.iter().sum::<f64>() / w.len().max(1) as f64), w.len(),
                deg(d.rates_rads.iter().map(|r| r * r).sum::<f64>().sqrt()), deg(d.tilt_rad),
                d.velocity_ms[0].hypot(d.velocity_ms[1]), d.velocity_ms[2]));
        }
    }
    let gains_line = PARAM_GAINS.get().map(|g| {
        let d = Gains::of(&Tuning::DEFAULT);
        let diff: Vec<String> = (0..GAIN_NAMES.len()).filter(|&i| g.0[i].to_bits() != d.0[i].to_bits())
            .map(|i| format!("{} {} (default {})", GAIN_NAMES[i], g.0[i], d.0[i])).collect();
        if diff.is_empty() { "gains         from --params: all 14 at the defaults".to_string() } else { format!("gains         from --params, differing from the defaults: {}", diff.join(", ")) }
    });
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
    summary.extend(gains_line);
    if let Some(p) = &pilot {
        summary.push(format!("records       GND {}, PIL {}", c.gnd, c.pil));
        summary.push(format!("pilot         full collective / m g = {:.4} (nominal card)", p.full_ratio()));
        let (c_lift, at_switch) = p.lift_ratios();
        summary.push(format!(
            "pilot         thrust ratio at lift-off seen (c_lift) {:.4}, at the ramp-to-height switch {:.4}",
            c_lift.unwrap_or(f64::NAN),
            at_switch.unwrap_or(f64::NAN)
        ));
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
    let hands_off = flag(&FS_HOLD).then(|| ho_line(scenario, &trace, &ho));
    // `MARV_HO_DUMP=<dir>`: one CSV per run of every flight step from 2 s before the stick loss to 25 s after it.
    if let (Ok(dir), Some(&(lost, _))) = (std::env::var("MARV_HO_DUMP"), scenario.rc_lost.first()) {
        let name = cell.map_or(format!("{}-seed{seed}", scenario.name), |r| r.cell.name.clone());
        let mut text = String::from("t_s,state,tf,vel_ne,pos_ne,heading,pos_covered,pos_d,vel_d,nav_vn,nav_ve,true_vn,true_ve,true_n,true_e,true_h,gated,anchor\n");
        for (p, h) in trace.steps.iter().zip(&ho) {
            if p.t_us + 2_000_000 < lost || p.t_us > lost + 25_000_000 {
                continue;
            }
            text.push_str(&format!("{:.5},{},{},{},{},{},{},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{},{}\n", p.t_us as f64 * 1e-6, h.state,
                u8::from(h.tf), u8::from(h.vne), u8::from(h.pne), u8::from(h.hdg), u8::from(p.position_covered), u8::from(p.pos_d_valid),
                u8::from(p.vel_d_valid), h.nav_v[0], h.nav_v[1], p.truth_vel[0], p.truth_vel[1], p.truth_pos[0], p.truth_pos[1],
                -p.truth_pos[2], h.gated, u8::from(h.anchor.is_some())));
        }
        std::fs::write(Path::new(&dir).join(format!("{name}.csv")), text).map_err(|e| format!("MARV_HO_DUMP: {e}"))?;
    }
    let steer = steer_cfg.zip(steer).map(|(cfg, st)| {
        let info = match cfg.est {
            EstKind::Mahony => None,
            EstKind::Eskf3 => {
                let i = info3.borrow();
                Some((i.bias_max, i.stats.reinits))
            }
            _ => Some(info.get()),
        };
        st.line(cfg, info, trace.liftoffs.first().copied(), windows)
    });
    Ok(Outcome { summary, checks, notes, shadow: shadow.map(|s| s.line()), steer, hands_off })
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

/// What `--preroll` accepts: an air start at rest, with no pilot (the pre-roll's RC is the script's first row).
fn preroll_ok(sc: &Scenario) -> Result<(), String> {
    let at = |m: &str| format!("--preroll: scenario {} {m}", sc.name);
    if !matches!(sc.start, Start::Air | Start::AirOverGround) || sc.pilot.is_some() {
        return Err(at("is a ground start or has a pilot; it has its own ground phase"));
    }
    if sc.velocity_ned_ms != [0.0; 3] || sc.rates_rads != [0.0; 3] {
        return Err(at("has a non-zero initial velocity or rate"));
    }
    if row_at(&sc.rc_script, 0).arm {
        return Err(at("arms at time 0"));
    }
    Ok(())
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
    let env = match &args.home {
        None => Env::load(&args.env).map_err(join)?,
        Some(home) => {
            // Only a git-ignored override is read, so the private origin cannot be one that is committed.
            marv_card::check_home_override(&args.env, &home.to_string_lossy())?;
            Env::load_with_home(&args.env, home).map_err(join)?
        }
    };
    match &args.job {
        Job::One { scenario, out, point } => {
            let seed = args.seed.ok_or(Failure::Usage("--seed is required with --scenario".to_string()))?;
            let scenario = Scenario::load(scenario)?;
            if args.preroll.is_some() {
                preroll_ok(&scenario).map_err(Failure::Usage)?;
            }
            let o = run_one(&card, &env, &scenario, seed, *point, None, out, true, args.shadow, args.steer, args.preroll, &args.windows)?;
            for line in o.summary.iter().chain(&o.notes).chain(&o.shadow).chain(&o.steer).chain(&o.hands_off) {
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
            if args.preroll.is_some() {
                for sc in &scenarios {
                    preroll_ok(sc).map_err(Failure::Usage)?;
                }
            }
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
            let mut cell_pass: Vec<bool> = Vec::new();
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
                    Some(&run), &out, keep, args.shadow, args.steer, args.preroll, &args.windows)?;
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
                cell_pass.push(failing.is_empty());
                println!("{:<width$}  {}  {}", cell.name, if failing.is_empty() { "PASS" } else { "FAIL" }, failing.join(", "));
                if let Some(line) = &o.shadow {
                    println!("{:<width$}  {line}", cell.name);
                }
                if let Some(line) = &o.steer {
                    println!("{:<width$}  {line}", cell.name);
                }
                if let Some(line) = &o.hands_off {
                    println!("{:<width$}  {line}", cell.name);
                }
                let _ = std::io::stdout().flush();
            }
            println!("failures by check (cells failing / cells judged):");
            for (name, failed, judged) in &counts {
                println!("  {name:<22} {failed:>4} / {judged}");
            }
            println!("PASS cells {passed} of {}", cells.len());
            let mut grid = vec![vec![(0usize, 0usize); spread.air.len()]; spread.ground.len()];
            for (cell, ok) in cells.iter().zip(&cell_pass) {
                let g = &mut grid[cell.ground][cell.air];
                g.0 += usize::from(*ok);
                g.1 += 1;
            }
            for g in 0..spread.ground.len() {
                let (p, n) = grid[g].iter().fold((0, 0), |a, c| (a.0 + c.0, a.1 + c.1));
                println!("PASS gnd{} {p} of {n}", g + 1);
            }
            for a in 0..spread.air.len() {
                let (p, n) = grid.iter().fold((0, 0), |s, row| (s.0 + row[a].0, s.1 + row[a].1));
                println!("PASS air{} {p} of {n}", a + 1);
            }
            println!("PASS by ground (rows) x air (columns), cells passing / cells run:");
            for (g, row) in grid.iter().enumerate() {
                let cols: Vec<String> = row.iter().map(|(p, n)| format!("{p:>3} / {n:<3}")).collect();
                println!("  gnd{}  {}", g + 1, cols.join("  "));
            }
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

#[cfg(test)]
mod tests {
    use super::tilt_err;

    /// Z-Y-X quaternion [w, x, y, z] from roll, pitch, yaw (rad).
    fn q(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
        let (cr, sr, cp, sp, cy, sy) = ((roll / 2.0).cos(), (roll / 2.0).sin(), (pitch / 2.0).cos(), (pitch / 2.0).sin(),
            (yaw / 2.0).cos(), (yaw / 2.0).sin());
        [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
    }

    #[test]
    fn tilt_error_ignores_yaw() {
        let (r, p) = (0.3f64, -0.2f64);
        for yaw in [90f64, 180.0] {
            assert!(tilt_err(q(r, p, 0.0), q(r, p, yaw.to_radians())) < 1e-9, "yaw {yaw}");
        }
        let e = tilt_err(q(0.0, 0.0, 0.7), q(10f64.to_radians(), 0.0, 0.7));
        assert!((e.to_degrees() - 10.0).abs() < 1e-9, "{}", e.to_degrees());
    }
}
