//! The scenario file and the spread file. Unknown keys are an error in every table and every row.

use crate::backend::Start;
use crate::math::{Quat, quat_mul};
use fw_core::rc::Switch3;
use marv_card::{Card, Env, Range};
use std::path::{Path, PathBuf};
use toml::{Table, Value};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rotors {
    /// Each rotor at the speed where four of them carry the weight.
    Hover,
    Stopped,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Scenario {
    pub name: String,
    pub duration_s: f64,
    /// `[scenario] gnss_rate_hz`: 1, 5 or 10; 5 when not given (the board's default).
    pub gnss_rate_hz: u32,
    /// `[scenario] mag_expected`: whether the compass sample carries the expected field (`MagSample.expected`); true
    /// when not given.
    pub mag_expected: bool,
    /// `[scenario] gnss_velocity`: whether the GNSS sample carries the velocity north, east and down
    /// (`GnssReading.vel_ned_mm_s`); true when not given.
    pub gnss_velocity: bool,
    /// `[scenario] baro_motion`: whether the barometer has its motion-related height error (card `baro.motion_*`); true
    /// when not given.
    pub baro_motion: bool,
    pub position_ned_m: [f64; 3],
    pub velocity_ned_ms: [f64; 3],
    /// Body to world `[w, x, y, z]`, from the file's roll, pitch, yaw.
    pub attitude: Quat,
    pub rates_rads: [f64; 3],
    pub rotors: Rotors,
    /// `[initial] contact`: `Start::Air` ("air", the default) or `Start::Ground` ("ground").
    pub start: Start,
    /// `[initial] settle_s` (ground contact only, required there): the settle onto the ground plane after a touchdown,
    /// s (take-off decision, Amendment B6); 0 in the air form.
    pub settle_s: f64,
    /// The stick script, by time; each row holds until the next. The first row is at 0. Empty in the phase form.
    pub rc_script: Vec<RcRow>,
    /// `[rc] phase`: each phase starts when the previous one ends. Empty in the script form.
    pub phases: Vec<Phase>,
    /// `[pilot]`: present exactly when `phases` is not empty.
    pub pilot: Option<PilotParams>,
    /// `[from_us, to_us)` intervals in which no stick report is made.
    pub rc_lost: Vec<(u64, u64)>,
    /// `[scenario] baro_lost = [[from_s, to_s], ...]`: `[from_us, to_us)` intervals in which no barometer sample reaches
    /// the flight code (the model keeps running).
    pub baro_lost: Vec<(u64, u64)>,
    /// `[scenario] gnss_lost = [[from_s, to_s], ...]` (G4, diagnostic runs): `[from_us, to_us)` intervals in which no GNSS
    /// sample reaches the flight code (the model keeps running).
    pub gnss_lost: Vec<(u64, u64)>,
    /// `[scenario] gnss_hold = [[from_s, to_s], ...]` (G4): in these intervals the samples repeat the last position
    /// before the interval and report zero speed and velocity (the receiver's low-speed freeze); after it the model's
    /// true solution returns with the jump.
    pub gnss_hold: Vec<(u64, u64)>,
    /// `[scenario] mag_fault = { from_s, kind, deg, below_m, axis, rate_dps }` (G4; `rate_dps` HM3b).
    pub mag_fault: Option<MagFault>,
    /// `[scenario] yaw_inject = { at_s, deg }` (HM3b, decision-heading-monitor H5 (ii)): at scenario time `at_s` (>= 0)
    /// the filter's yaw is turned by `deg` (yaw + deg) through the firmware's `sim-fault` hook; eskf3 runs only.
    pub yaw_inject: Option<YawInject>,
    /// `[scenario] yaw_hold = { at_s, deg }` (hands-off campaign, decision-heading-monitor H12 I-H5): from scenario time `at_s`
    /// on, before every flight step the filter's yaw is set to the truth yaw + `deg` through the firmware's `sim-fault` hook,
    /// so the error cannot heal; eskf3 runs only.
    pub yaw_hold: Option<YawInject>,
    /// `[scenario] gnss_jump = { at_s, north_m, east_m }` (hands-off campaign): from scenario time `at_s` on, every GNSS
    /// position the flight code gets is moved by this many metres (a persistent jump of the reported position).
    pub gnss_jump: Option<(u64, [f64; 2])>,
    /// `[scenario] motor_thrust = { at_s, line, scale }` (hands-off campaign): from scenario time `at_s` on, the thrust (and
    /// the reaction torque) of the rotor on DShot line `line` is scaled by `scale` in the plant.
    pub motor_thrust: Option<(u64, usize, f64)>,
    /// `[scenario] push = { from_s, to_s, north_n, east_n }` (hands-off campaign H14): a horizontal world-frame force (N) on the
    /// vehicle from `from_s` to `to_s`, added to the plant's wrench.
    pub push: Option<(u64, u64, [f64; 2])>,
    /// `[scenario] gnss_latency_s` (G4): replaces the model's latency (the card's); `None` leaves it.
    pub gnss_latency_s: Option<f64>,
    /// `[scenario] wind_steps = [[t_s, d_north, d_east, d_down], ...]` (Amendment P, P6 and P7): from scenario time `t_s`
    /// on, the wind the plant sees is the mean wind plus this delta (m/s), summed over every step whose time has come.
    pub wind_steps: Vec<(u64, [f64; 3])>,
    pub disturbances: Vec<Disturbance>,
    pub verdict: Verdict,
}

/// `[scenario] mag_fault` (G4): what is done to the compass sample the flight code gets, from `from_us` on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagFault {
    /// Scenario time; negative reaches into the pre-roll (the run's time before scenario time 0).
    pub from_us: i64,
    pub kind: MagFaultKind,
    /// `Rotate`: the angle, rad; unused by `Stuck`.
    pub deg_rad: f64,
    /// `Rotate` only: the field is rotated only while the truth height above the start is below this, m.
    pub below_m: Option<f64>,
    /// `Rotate` only: the body axis (0 x, 1 y, 2 z) the field is rotated about; z when not given.
    pub axis: usize,
    /// `Rotate` only (HM3b): a ramp, rad/s (> 0): the angle is `rate x (t - from)`, up to `deg` in size, with `deg`'s
    /// sign; `None` is the step to `deg` at `from`.
    pub rate_rad_s: Option<f64>,
}

/// `[scenario] yaw_inject` (HM3b): the time (scenario, us) and the angle (rad) of the filter yaw rotation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct YawInject {
    pub at_us: u64,
    pub rad: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MagFaultKind {
    /// The calibrated field vector turned about a body axis by `deg`.
    Rotate,
    /// The field vector of the first sample at or after `from_us` is repeated; the sample's time and sequence advance.
    Stuck,
}

/// One row of `[rc] script`. Roll, pitch, yaw -1..1 (right, forward, clockwise +); throttle 0..1; arm = the switch;
/// `mode` = the channel-6 switch ("low", the default, "middle" or "high").
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcRow {
    pub t_us: u64,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub arm: bool,
    pub mode: Switch3,
}

/// `[pilot]`: the sim pilot's limits and gains (decision-sim-takeoff-landing, sections 2 and 10).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PilotParams {
    pub delay_s: f64,
    pub stick_rate_per_s: f64,
    pub ramp_stick_per_s: f64,
    pub airborne_m: f64,
    pub k_h: f64,
    pub k_v: f64,
    pub k_i: f64,
    pub accel_max_g: f64,
    pub v_up_ms: f64,
    pub v_dn_ms: f64,
    pub xy_k: f64,
    pub xy_stick_max: f64,
    /// Integral gain of the drift correction, 1/s^2 (Amendment B2; provisional until the owner approves it).
    pub xy_ki: f64,
    /// The documented stick map's top point, x m g (the flight code's cap, mirrored here; never read from it).
    pub stick_thrust_max_g: f64,
}

/// How a phase ends. An event is seen through the pilot's delay; `timeout_us` ends the phase as a timeout.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PhaseEnd {
    After { us: u64 },
    /// Height at or above `[pilot] airborne_m`.
    Airborne { timeout_us: u64 },
    /// Height below `below_m`.
    Below { below_m: f64, timeout_us: u64 },
    /// The contact flag.
    Landed { timeout_us: u64 },
}

/// What the pilot does with the throttle in a phase.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Throttle {
    /// Stick to 0 at the rate limit.
    Zero,
    /// Throttle stick up from 0 at `ramp_stick_per_s`.
    Ramp,
    /// Hold this height, m.
    Height(f64),
    /// Descend at this speed, m/s.
    Descend(f64),
}

/// One row of `[rc] phase`. Roll, pitch and yaw are scripted sticks; `arm = None` keeps the previous phase's switch.
#[derive(Debug, Clone, PartialEq)]
pub struct Phase {
    pub name: String,
    pub end: PhaseEnd,
    pub arm: Option<bool>,
    pub throttle: Throttle,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    /// `xy = "hold"`: the pilot's drift correction on roll and pitch.
    pub xy_hold: bool,
    /// The channel-6 switch from this phase on; `None` keeps the previous phase's (Low at the start).
    pub mode: Option<Switch3>,
}

/// A body torque added to the plant's wrench over `[from_us, to_us)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Disturbance {
    pub from_us: u64,
    /// `u64::MAX` when the file gives no `to_s`.
    pub to_us: u64,
    pub torque_nm: [f64; 3],
}

/// One stick-sign pulse: the truth axis it must move (0 roll, 1 pitch, 2 yaw) and the sense (+1 up, -1 down).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pulse {
    pub from_s: f64,
    pub to_s: f64,
    pub axis: usize,
    pub sense: f64,
}

/// `[verdict]`: which check, and its thresholds and times as the file gives them (seconds, degrees, percent).
#[derive(Debug, Clone, PartialEq)]
pub enum Verdict {
    Hover {
        from_s: f64,
        disturbance_s: f64,
        settle_from_s: f64,
        max_tilt_deg: f64,
        max_nav_gap_deg: f64,
        max_heading_change_deg: f64,
        max_peak_tilt_deg: f64,
    },
    Steps {
        roll_at_s: f64,
        pitch_at_s: f64,
        yaw_at_s: f64,
        hold_s: f64,
        step_angle_deg: f64,
        step_yaw_rate_dps: f64,
        max_rise_s: f64,
        max_overshoot_rp_pct: f64,
        max_overshoot_yaw_pct: f64,
    },
    StickSign {
        pulses: Vec<Pulse>,
        min_move_deg: f64,
        max_off_axis_ratio: f64,
        throttle_from_s: f64,
        throttle_to_s: f64,
        min_accel_rise_ms2: f64,
    },
    RcLoss {
        on_high_s: f64,
        low_s: f64,
        arm_s: f64,
        throttle_s: f64,
        lost_at_s: f64,
        rc_fs_timeout_s: f64,
        stop_bound_s: f64,
        resume_s: f64,
        rearm_s: f64,
        off_s: f64,
        off_bound_s: f64,
    },
    /// decision-nav-eskf-20261120, Amendment F, F9: altitude hold on ESKF2 (windows, heights and the mode sequence).
    AltHold {
        windows: Vec<(f64, f64)>,
        max_height_error_m: f64,
        max_nav_gap_m: f64,
        climb_from_s: f64,
        climb_to_s: f64,
        descent_from_s: f64,
        descent_to_s: f64,
        enter_s: f64,
        enter_bound_s: f64,
        baro_lost_from_s: f64,
        baro_lost_to_s: f64,
    },
    /// Amendment P, P7 (PS1): position hold, c1 to c7 over the hold windows. The thresholds are the file's; the tilt
    /// limit and the ground threshold are the flight code's `Tuning::DEFAULT`.
    PosHold {
        windows: Vec<(f64, f64)>,
        max_arm_to_position_s: f64,
        max_hold_error_m: f64,
        max_nav_gap_m: f64,
        max_height_error_m: f64,
        releases_s: Vec<f64>,
        modes: Vec<u8>,
        selected: Vec<u8>,
    },
    /// decision-sim-takeoff-landing, section 3. `airborne_m` is `[pilot] airborne_m`, the verdict's lift-off height.
    TakeoffLand {
        airborne_m: f64,
        arm_bound_s: f64,
        r_stance_m: f64,
        stance_floor_weight: f64,
        max_stance_ratio: f64,
        max_light_after_td_s: f64,
        max_peak_height_m: f64,
        max_peak_climb_ms: f64,
        liftoff_window_s: f64,
        max_liftoff_tilt_deg: f64,
        height_band_m: f64,
        max_tilt_deg: f64,
        max_speed_h_ms: f64,
        max_nav_gap_deg: f64,
        max_td_vz_ms: f64,
        max_td_vh_ms: f64,
        max_td_tilt_deg: f64,
        max_td_rate_dps: f64,
        idle_after_s: f64,
        stop_bound_s: f64,
        hover_trim_tolerance: f64,
    },
}

/// One plant point of the spread (decision first-pair 8e). The flight code keeps the compiled card.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub kf_scale: f64,
    /// Scales the whole inertia tensor.
    pub inertia_scale: f64,
    /// Both motor time constants; `None` keeps the card's.
    pub lag_s: Option<f64>,
}

/// One value of the matrix axis "map shape": the plant's level-to-speed map. `None` keeps the card's number.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Map {
    pub speed_at_level_min: Option<f64>,
}

/// One value of the axis "air": the plant's linear rotor drag and the wind. `None` keeps the card's or the
/// environment's number.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Air {
    pub drag_rotor_linear: Option<f64>,
    pub wind_ned_ms: Option<[f64; 3]>,
}

/// One value of the axis "ground": a fixed IMU1 bias (replaces the card's vector) and the ground's roll and pitch
/// (replaces the scenario's; a ground-start scenario only). `None` keeps the card's or the scenario's.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Ground {
    pub imu1_gyro_bias_rads: Option<[f64; 3]>,
    pub imu1_accel_bias_ms2: Option<[f64; 3]>,
    pub roll_pitch_deg: Option<[f64; 2]>,
}

/// The spread or matrix file: the scenario files (relative to the file) and the axes. A missing axis has one default
/// value, which changes nothing; `seeds` missing means the runner's `--seed` is the one value.
#[derive(Debug, Clone, PartialEq)]
pub struct Spread {
    pub scenarios: Vec<PathBuf>,
    pub points: Vec<Point>,
    pub maps: Vec<Map>,
    pub air: Vec<Air>,
    pub ground: Vec<Ground>,
    pub seeds: Option<Vec<u64>>,
}

/// One run of the matrix: an index into each axis and the seed.
#[derive(Debug, Clone, PartialEq)]
pub struct Cell {
    /// `[<scenario>-]p<point>-map<A..>-air<n>-gnd<n>-seed<s>`; the scenario only when the file has more than one.
    pub name: String,
    pub scenario: usize,
    pub point: usize,
    pub map: usize,
    pub air: usize,
    pub ground: usize,
    pub seed: u64,
}

/// The longest name that still fits, with its prefix, in the log's MSG text (64 bytes).
const NAME_MAX: usize = 50;

fn table<'a>(root: &'a Table, name: &str, allowed: &[&str]) -> Result<&'a Table, String> {
    let t = root
        .get(name)
        .and_then(Value::as_table)
        .ok_or_else(|| format!("[{name}]: missing table"))?;
    known_keys(t, name, allowed)?;
    Ok(t)
}

fn known_keys(t: &Table, name: &str, allowed: &[&str]) -> Result<(), String> {
    for key in t.keys() {
        if !allowed.contains(&key.as_str()) {
            return Err(format!("{name}.{key}: unknown key"));
        }
    }
    Ok(())
}

/// The rows of an array of tables, each checked for unknown keys. A missing key gives an empty list.
fn rows<'a>(t: &'a Table, name: &str, key: &str, allowed: &[&str]) -> Result<Vec<&'a Table>, String> {
    let Some(v) = t.get(key) else { return Ok(Vec::new()) };
    let label = if name.is_empty() { key.to_string() } else { format!("{name}.{key}") };
    let bad = || format!("{label}: must be a list of tables");
    let mut out = Vec::new();
    for (i, row) in v.as_array().ok_or_else(bad)?.iter().enumerate() {
        let row = row.as_table().ok_or_else(bad)?;
        known_keys(row, &format!("{label}[{i}]"), allowed)?;
        out.push(row);
    }
    Ok(out)
}

/// Seconds to microseconds; refuses negative times.
fn micros(t: &Table, table: &str, key: &str) -> Result<u64, String> {
    let s = scalar(t, table, key)?;
    if s < 0.0 {
        return Err(format!("{table}.{key}: must be >= 0"));
    }
    Ok((s * 1e6).round() as u64)
}

/// A number in `lo..=hi`, as f32.
fn ranged(t: &Table, table: &str, key: &str, lo: f64, hi: f64) -> Result<f32, String> {
    let x = scalar(t, table, key)?;
    if !(lo..=hi).contains(&x) {
        return Err(format!("{table}.{key}: must be in {lo}..{hi}"));
    }
    Ok(x as f32)
}

fn rc_script(rc: &Table) -> Result<Vec<RcRow>, String> {
    let keys = ["t_s", "roll", "pitch", "yaw", "throttle", "arm", "mode"];
    let mut script: Vec<RcRow> = Vec::new();
    for (i, row) in rows(rc, "rc", "script", &keys)?.into_iter().enumerate() {
        let name = format!("rc.script[{i}]");
        let t_us = micros(row, &name, "t_s")?;
        if script.last().is_some_and(|prev| prev.t_us >= t_us) || (script.is_empty() && t_us != 0) {
            return Err(format!("{name}.t_s: the first row must be at 0 and the times must increase"));
        }
        script.push(RcRow {
            t_us,
            roll: ranged(row, &name, "roll", -1.0, 1.0)?,
            pitch: ranged(row, &name, "pitch", -1.0, 1.0)?,
            yaw: ranged(row, &name, "yaw", -1.0, 1.0)?,
            throttle: ranged(row, &name, "throttle", 0.0, 1.0)?,
            arm: get(row, &name, "arm")?.as_bool().ok_or(format!("{name}.arm: must be true or false"))?,
            mode: match row.get("mode") {
                None => Switch3::Low,
                Some(Value::String(m)) if m == "low" => Switch3::Low,
                Some(Value::String(m)) if m == "middle" => Switch3::Middle,
                Some(Value::String(m)) if m == "high" => Switch3::High,
                Some(_) => return Err(format!("{name}.mode: must be \"low\", \"middle\" or \"high\"")),
            },
        });
    }
    if script.is_empty() {
        return Err("rc.script: missing or empty".to_string());
    }
    Ok(script)
}

fn pilot(p: &Table) -> Result<PilotParams, String> {
    let f = |key: &str| {
        let x = scalar(p, "pilot", key)?;
        if x < 0.0 {
            return Err(format!("pilot.{key}: must be >= 0"));
        }
        Ok(x)
    };
    Ok(PilotParams {
        delay_s: f("delay_s")?,
        stick_rate_per_s: f("stick_rate_per_s")?,
        ramp_stick_per_s: f("ramp_stick_per_s")?,
        airborne_m: f("airborne_m")?,
        k_h: f("k_h")?,
        k_v: f("k_v")?,
        k_i: f("k_i")?,
        accel_max_g: f("accel_max_g")?,
        v_up_ms: f("v_up_ms")?,
        v_dn_ms: f("v_dn_ms")?,
        xy_k: f("xy_k")?,
        xy_stick_max: f("xy_stick_max")?,
        xy_ki: f("xy_ki")?,
        stick_thrust_max_g: f("stick_thrust_max_g")?,
    })
}

const PILOT_KEYS: [&str; 14] = ["delay_s", "stick_rate_per_s", "ramp_stick_per_s", "airborne_m", "k_h", "k_v", "k_i",
    "accel_max_g", "v_up_ms", "v_dn_ms", "xy_k", "xy_stick_max", "xy_ki", "stick_thrust_max_g"];

fn rc_phases(rc: &Table) -> Result<Vec<Phase>, String> {
    let keys = ["name", "after_s", "event", "below_m", "timeout_s", "arm", "throttle", "height_m", "descend_ms", "xy",
        "roll", "pitch", "yaw", "mode"];
    let mut phases: Vec<Phase> = Vec::new();
    for (i, row) in rows(rc, "rc", "phase", &keys)?.into_iter().enumerate() {
        let at = format!("rc.phase[{i}]");
        let name = match get(row, &at, "name")? {
            Value::String(n) if !n.is_empty() => n.clone(),
            _ => return Err(format!("{at}.name: must be a non-empty string")),
        };
        if phases.iter().any(|p| p.name == name) {
            return Err(format!("{at}.name: \"{name}\" is used twice"));
        }
        let has = |k: &str| row.contains_key(k);
        let event = if has("event") {
            Some(get(row, &at, "event")?.as_str().ok_or(format!("{at}.event: must be a string"))?)
        } else {
            None
        };
        if has("below_m") && event != Some("below") {
            return Err(format!("{at}.below_m: only with event = \"below\""));
        }
        let end = match (has("after_s"), event) {
            (true, None) if !has("timeout_s") => PhaseEnd::After { us: micros(row, &at, "after_s")? },
            (false, Some(e)) => {
                let timeout_us = micros(row, &at, "timeout_s")?;
                match e {
                    "airborne" => PhaseEnd::Airborne { timeout_us },
                    "below" => PhaseEnd::Below { below_m: scalar(row, &at, "below_m")?, timeout_us },
                    "landed" => PhaseEnd::Landed { timeout_us },
                    _ => return Err(format!("{at}.event: must be \"airborne\", \"below\" or \"landed\"")),
                }
            }
            _ => return Err(format!("{at}: needs either after_s, or event with timeout_s")),
        };
        let throttle = match (has("throttle"), has("height_m"), has("descend_ms")) {
            (true, false, false) => match get(row, &at, "throttle")?.as_str() {
                Some("zero") => Throttle::Zero,
                Some("ramp") => Throttle::Ramp,
                _ => return Err(format!("{at}.throttle: must be \"zero\" or \"ramp\"")),
            },
            (false, true, false) => Throttle::Height(scalar(row, &at, "height_m")?),
            (false, false, true) => Throttle::Descend(scalar(row, &at, "descend_ms")?),
            _ => return Err(format!("{at}: needs exactly one of throttle, height_m, descend_ms")),
        };
        let xy_hold = match row.get("xy") {
            None => false,
            Some(Value::String(x)) if x == "hold" => true,
            Some(_) => return Err(format!("{at}.xy: must be \"hold\"")),
        };
        if xy_hold && (has("roll") || has("pitch")) {
            return Err(format!("{at}.xy: the drift correction is off in a phase that scripts roll or pitch"));
        }
        let stick = |k: &str| if has(k) { ranged(row, &at, k, -1.0, 1.0) } else { Ok(0.0) };
        let arm = match row.get("arm") {
            None => None,
            Some(v) => Some(v.as_bool().ok_or(format!("{at}.arm: must be true or false"))?),
        };
        let mode = match row.get("mode") {
            None => None,
            Some(Value::String(m)) if m == "low" => Some(Switch3::Low),
            Some(Value::String(m)) if m == "middle" => Some(Switch3::Middle),
            Some(Value::String(m)) if m == "high" => Some(Switch3::High),
            Some(_) => return Err(format!("{at}.mode: must be \"low\", \"middle\" or \"high\"")),
        };
        phases.push(Phase { name, end, arm, throttle, roll: stick("roll")?, pitch: stick("pitch")?, yaw: stick("yaw")?,
            xy_hold, mode });
    }
    if phases.is_empty() {
        return Err("rc.phase: empty".to_string());
    }
    Ok(phases)
}

fn verdict(root: &Table) -> Result<Verdict, String> {
    let v = root
        .get("verdict")
        .and_then(Value::as_table)
        .ok_or("[verdict]: missing table (a scenario without a verdict is a load error)")?;
    let check = get(v, "verdict", "check")?.as_str().unwrap_or("");
    let keys: &[&str] = match check {
        "hover" => &["from_s", "disturbance_s", "settle_from_s", "max_tilt_deg", "max_nav_gap_deg",
            "max_heading_change_deg", "max_peak_tilt_deg"],
        "steps" => &["roll_at_s", "pitch_at_s", "yaw_at_s", "hold_s", "step_angle_deg", "step_yaw_rate_dps",
            "max_rise_s", "max_overshoot_rp_pct", "max_overshoot_yaw_pct"],
        "stick_sign" => &["pulses", "min_move_deg", "max_off_axis_ratio", "throttle_from_s", "throttle_to_s",
            "min_accel_rise_ms2"],
        "rc_loss" => &["on_high_s", "low_s", "arm_s", "throttle_s", "lost_at_s", "rc_fs_timeout_s", "stop_bound_s",
            "resume_s", "rearm_s", "off_s", "off_bound_s"],
        "alt_hold" => &["windows", "max_height_error_m", "max_nav_gap_m", "climb_from_s", "climb_to_s",
            "descent_from_s", "descent_to_s", "enter_s", "enter_bound_s", "baro_lost_from_s", "baro_lost_to_s"],
        "pos_hold" => &["windows", "max_arm_to_position_s", "max_hold_error_m", "max_nav_gap_m", "max_height_error_m",
            "releases_s", "modes", "selected"],
        "takeoff_land" => &["arm_bound_s", "r_stance_m", "stance_floor_weight", "max_stance_ratio",
            "max_light_after_td_s", "max_peak_height_m", "max_peak_climb_ms", "liftoff_window_s",
            "max_liftoff_tilt_deg", "height_band_m", "max_tilt_deg",
            "max_speed_h_ms", "max_nav_gap_deg",
            "max_td_vz_ms", "max_td_vh_ms", "max_td_tilt_deg", "max_td_rate_dps", "idle_after_s", "stop_bound_s",
            "hover_trim_tolerance"],
        _ => return Err("verdict.check: must be \"hover\", \"steps\", \"stick_sign\", \"rc_loss\", \"alt_hold\", \"pos_hold\" or \"takeoff_land\""
            .to_string()),
    };
    known_keys(v, "verdict", &[&["check"], keys].concat())?;
    let f = |key: &str| scalar(v, "verdict", key);
    Ok(match check {
        "hover" => Verdict::Hover {
            from_s: f("from_s")?,
            disturbance_s: f("disturbance_s")?,
            settle_from_s: f("settle_from_s")?,
            max_tilt_deg: f("max_tilt_deg")?,
            max_nav_gap_deg: f("max_nav_gap_deg")?,
            max_heading_change_deg: f("max_heading_change_deg")?,
            max_peak_tilt_deg: f("max_peak_tilt_deg")?,
        },
        "steps" => Verdict::Steps {
            roll_at_s: f("roll_at_s")?,
            pitch_at_s: f("pitch_at_s")?,
            yaw_at_s: f("yaw_at_s")?,
            hold_s: f("hold_s")?,
            step_angle_deg: f("step_angle_deg")?,
            step_yaw_rate_dps: f("step_yaw_rate_dps")?,
            max_rise_s: f("max_rise_s")?,
            max_overshoot_rp_pct: f("max_overshoot_rp_pct")?,
            max_overshoot_yaw_pct: f("max_overshoot_yaw_pct")?,
        },
        "stick_sign" => {
            let mut pulses = Vec::new();
            for (i, row) in rows(v, "verdict", "pulses", &["from_s", "to_s", "axis", "sense"])?.into_iter().enumerate() {
                let name = format!("verdict.pulses[{i}]");
                let axis = match get(row, &name, "axis")?.as_str() {
                    Some("roll") => 0,
                    Some("pitch") => 1,
                    Some("yaw") => 2,
                    _ => return Err(format!("{name}.axis: must be \"roll\", \"pitch\" or \"yaw\"")),
                };
                let sense = scalar(row, &name, "sense")?;
                if sense != 1.0 && sense != -1.0 {
                    return Err(format!("{name}.sense: must be 1 or -1"));
                }
                pulses.push(Pulse { from_s: scalar(row, &name, "from_s")?, to_s: scalar(row, &name, "to_s")?, axis, sense });
            }
            Verdict::StickSign {
                pulses,
                min_move_deg: f("min_move_deg")?,
                max_off_axis_ratio: f("max_off_axis_ratio")?,
                throttle_from_s: f("throttle_from_s")?,
                throttle_to_s: f("throttle_to_s")?,
                min_accel_rise_ms2: f("min_accel_rise_ms2")?,
            }
        }
        "alt_hold" => Verdict::AltHold {
            windows: pairs(v, "verdict", "windows")?,
            max_height_error_m: f("max_height_error_m")?,
            max_nav_gap_m: f("max_nav_gap_m")?,
            climb_from_s: f("climb_from_s")?,
            climb_to_s: f("climb_to_s")?,
            descent_from_s: f("descent_from_s")?,
            descent_to_s: f("descent_to_s")?,
            enter_s: f("enter_s")?,
            enter_bound_s: f("enter_bound_s")?,
            baro_lost_from_s: f("baro_lost_from_s")?,
            baro_lost_to_s: f("baro_lost_to_s")?,
        },
        "pos_hold" => Verdict::PosHold {
            windows: pairs(v, "verdict", "windows")?,
            max_arm_to_position_s: f("max_arm_to_position_s")?,
            max_hold_error_m: f("max_hold_error_m")?,
            max_nav_gap_m: f("max_nav_gap_m")?,
            max_height_error_m: f("max_height_error_m")?,
            releases_s: numbers(v, "verdict", "releases_s")?,
            modes: codes(v, "verdict", "modes")?,
            selected: codes(v, "verdict", "selected")?,
        },
        "takeoff_land" => Verdict::TakeoffLand {
            airborne_m: root
                .get("pilot")
                .and_then(Value::as_table)
                .ok_or("verdict.check: \"takeoff_land\" needs [pilot] (airborne_m is the lift-off height)".to_string())
                .and_then(|p| scalar(p, "pilot", "airborne_m"))?,
            arm_bound_s: f("arm_bound_s")?,
            r_stance_m: f("r_stance_m")?,
            stance_floor_weight: f("stance_floor_weight")?,
            max_stance_ratio: f("max_stance_ratio")?,
            max_light_after_td_s: f("max_light_after_td_s")?,
            max_peak_height_m: f("max_peak_height_m")?,
            max_peak_climb_ms: f("max_peak_climb_ms")?,
            liftoff_window_s: f("liftoff_window_s")?,
            max_liftoff_tilt_deg: f("max_liftoff_tilt_deg")?,
            height_band_m: f("height_band_m")?,
            max_tilt_deg: f("max_tilt_deg")?,
            max_speed_h_ms: f("max_speed_h_ms")?,
            max_nav_gap_deg: f("max_nav_gap_deg")?,
            max_td_vz_ms: f("max_td_vz_ms")?,
            max_td_vh_ms: f("max_td_vh_ms")?,
            max_td_tilt_deg: f("max_td_tilt_deg")?,
            max_td_rate_dps: f("max_td_rate_dps")?,
            idle_after_s: f("idle_after_s")?,
            stop_bound_s: f("stop_bound_s")?,
            hover_trim_tolerance: f("hover_trim_tolerance")?,
        },
        _ => Verdict::RcLoss {
            on_high_s: f("on_high_s")?,
            low_s: f("low_s")?,
            arm_s: f("arm_s")?,
            throttle_s: f("throttle_s")?,
            lost_at_s: f("lost_at_s")?,
            rc_fs_timeout_s: f("rc_fs_timeout_s")?,
            stop_bound_s: f("stop_bound_s")?,
            resume_s: f("resume_s")?,
            rearm_s: f("rearm_s")?,
            off_s: f("off_s")?,
            off_bound_s: f("off_bound_s")?,
        },
    })
}

/// A list of `[from_s, to_s]` pairs, 0 <= from_s < to_s.
fn pairs(t: &Table, table: &str, key: &str) -> Result<Vec<(f64, f64)>, String> {
    let bad = || format!("{table}.{key}: must be a list of [from_s, to_s] pairs, 0 <= from_s < to_s");
    let mut out = Vec::new();
    for pair in get(t, table, key)?.as_array().ok_or_else(bad)? {
        match pair.as_array().map(|p| p.iter().map(number).collect::<Vec<_>>()).as_deref() {
            Some([Some(from), Some(to)]) if *from >= 0.0 && from < to => out.push((*from, *to)),
            _ => return Err(bad()),
        }
    }
    Ok(out)
}

/// A list of numbers.
fn numbers(t: &Table, table: &str, key: &str) -> Result<Vec<f64>, String> {
    let bad = || format!("{table}.{key}: must be a list of numbers");
    get(t, table, key)?.as_array().ok_or_else(bad)?.iter().map(|v| number(v).ok_or_else(bad)).collect()
}

/// A list of mode codes, 1 Angle, 2 Altitude, 3 Position.
fn codes(t: &Table, table: &str, key: &str) -> Result<Vec<u8>, String> {
    let bad = || format!("{table}.{key}: must be a list of mode codes 1, 2 or 3");
    let list = get(t, table, key)?.as_array().ok_or_else(bad)?;
    list.iter().map(|v| v.as_integer().filter(|c| (1..=3).contains(c)).map(|c| c as u8).ok_or_else(bad)).collect()
}

fn number(v: &Value) -> Option<f64> {
    match v {
        Value::Float(x) if x.is_finite() => Some(*x),
        Value::Integer(i) => Some(*i as f64),
        _ => None,
    }
}

fn get<'a>(t: &'a Table, table: &str, key: &str) -> Result<&'a Value, String> {
    t.get(key).ok_or_else(|| format!("{table}.{key}: missing"))
}

fn scalar(t: &Table, table: &str, key: &str) -> Result<f64, String> {
    number(get(t, table, key)?).ok_or_else(|| format!("{table}.{key}: must be a number"))
}

fn vec3(t: &Table, table: &str, key: &str) -> Result<[f64; 3], String> {
    let bad = || format!("{table}.{key}: must be three numbers");
    let a = get(t, table, key)?.as_array().ok_or_else(bad)?;
    if a.len() != 3 {
        return Err(bad());
    }
    Ok([
        number(&a[0]).ok_or_else(bad)?,
        number(&a[1]).ok_or_else(bad)?,
        number(&a[2]).ok_or_else(bad)?,
    ])
}

/// Yaw, then pitch, then roll (conventions 7c): `q = qz(yaw) * qy(pitch) * qx(roll)`, with `w >= 0`.
fn quat_from_euler_deg(roll_pitch_yaw: [f64; 3]) -> Quat {
    let [roll, pitch, yaw] = roll_pitch_yaw.map(|d| d.to_radians() * 0.5);
    let qx = [roll.cos(), roll.sin(), 0.0, 0.0];
    let qy = [pitch.cos(), 0.0, pitch.sin(), 0.0];
    let qz = [yaw.cos(), 0.0, 0.0, yaw.sin()];
    let q = quat_mul(qz, quat_mul(qy, qx));
    let sign = if q[0] < 0.0 { -1.0 } else { 1.0 };
    q.map(|x| sign * x)
}

impl Scenario {
    pub fn load(path: impl AsRef<Path>) -> Result<Scenario, String> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: cannot read: {e}", path.display()))?;
        Scenario::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn parse(text: &str) -> Result<Scenario, String> {
        let root: Table = text.parse().map_err(|e: toml::de::Error| format!("toml: {}", e.to_string().trim_end()))?;
        for key in root.keys() {
            if !["scenario", "initial", "pilot", "rc", "disturbance", "verdict"].contains(&key.as_str()) {
                return Err(format!("{key}: unknown table"));
            }
        }
        let s = table(&root, "scenario", &["name", "duration_s", "gnss_rate_hz", "mag_expected", "gnss_velocity", "baro_motion", "baro_lost", "gnss_lost", "gnss_hold", "mag_fault", "gnss_latency_s", "wind_steps", "yaw_inject", "yaw_hold", "gnss_jump", "motor_thrust", "push"])?;
        let ground = match root.get("initial").and_then(Value::as_table).and_then(|i| i.get("contact")) {
            None => false,
            Some(Value::String(c)) if c == "air" => false,
            Some(Value::String(c)) if c == "ground" => true,
            Some(Value::String(c)) if c == "air_over_ground" => false,
            Some(_) => return Err("initial.contact: must be \"air\", \"air_over_ground\" or \"ground\"".to_string()),
        };
        // Hands-off campaign: an air start with the ground plane at down = 0 under it (`Start::AirOverGround`).
        let over = root.get("initial").and_then(Value::as_table).and_then(|i| i.get("contact")).and_then(Value::as_str) == Some("air_over_ground");
        let i = if ground {
            table(&root, "initial", &["contact", "ground_roll_pitch_deg", "yaw_deg", "rotors", "settle_s"])?
        } else {
            table(
                &root,
                "initial",
                &["contact", "position_ned_m", "velocity_ned_ms", "roll_pitch_yaw_deg", "rates_rads", "rotors", "settle_s"][..if over { 7 } else { 6 }],
            )?
        };

        let name = match get(s, "scenario", "name")? {
            Value::String(n) if !n.is_empty() && n.len() <= NAME_MAX => n.clone(),
            _ => return Err(format!("scenario.name: must be a string of 1 to {NAME_MAX} bytes")),
        };
        let duration_s = scalar(s, "scenario", "duration_s")?;
        if duration_s <= 0.0 {
            return Err("scenario.duration_s: must be > 0".to_string());
        }
        let gnss_rate_hz = match optional_scalar(s, "scenario", "gnss_rate_hz")? {
            None => crate::slow_sensors::GNSS_DEFAULT_RATE_HZ,
            Some(r) if crate::slow_sensors::GNSS_RATES_HZ.iter().any(|&x| f64::from(x) == r) => r as u32,
            Some(_) => return Err("scenario.gnss_rate_hz: must be 1, 5 or 10".to_string()),
        };
        let flag = |key: &str| match s.get(key) {
            None => Ok(true),
            Some(Value::Boolean(b)) => Ok(*b),
            Some(_) => Err(format!("scenario.{key}: must be true or false")),
        };
        let mag_expected = flag("mag_expected")?;
        let gnss_velocity = flag("gnss_velocity")?;
        let baro_motion = flag("baro_motion")?;
        let rotors = match get(i, "initial", "rotors")? {
            Value::String(r) if r == "hover" => Rotors::Hover,
            Value::String(r) if r == "stopped" => Rotors::Stopped,
            _ => return Err("initial.rotors: must be \"hover\" or \"stopped\"".to_string()),
        };
        let rc = table(&root, "rc", &["script", "phase", "lost"])?;
        let (rc_script, phases, pilot) = match (rc.contains_key("script"), rc.contains_key("phase")) {
            (true, false) => {
                if root.contains_key("pilot") {
                    return Err("[pilot]: only with [rc] phase".to_string());
                }
                (rc_script(rc)?, Vec::new(), None)
            }
            (false, true) => {
                let p = pilot(table(&root, "pilot", &PILOT_KEYS)?)?;
                (Vec::new(), rc_phases(rc)?, Some(p))
            }
            _ => return Err("rc: needs either script or phase, not both".to_string()),
        };
        let mut rc_lost = Vec::new();
        for (i, row) in rows(rc, "rc", "lost", &["from_s", "to_s"])?.into_iter().enumerate() {
            let name = format!("rc.lost[{i}]");
            rc_lost.push((micros(row, &name, "from_s")?, micros(row, &name, "to_s")?));
        }
        let baro_lost = match s.get("baro_lost") {
            None => Vec::new(),
            Some(_) => pairs(s, "scenario", "baro_lost")?
                .into_iter()
                .map(|(from, to)| ((from * 1e6).round() as u64, (to * 1e6).round() as u64))
                .collect(),
        };
        let intervals = |key: &str| -> Result<Vec<(u64, u64)>, String> {
            match s.get(key) {
                None => Ok(Vec::new()),
                Some(_) => Ok(pairs(s, "scenario", key)?
                    .into_iter()
                    .map(|(from, to)| ((from * 1e6).round() as u64, (to * 1e6).round() as u64))
                    .collect()),
            }
        };
        let gnss_lost = intervals("gnss_lost")?;
        let gnss_hold = intervals("gnss_hold")?;
        let gnss_latency_s = optional_scalar(s, "scenario", "gnss_latency_s")?;
        if gnss_latency_s.is_some_and(|l| !(0.0..=1.0).contains(&l)) {
            return Err("scenario.gnss_latency_s: must be in [0, 1] s".to_string());
        }
        let wind_steps = match s.get("wind_steps") {
            None => Vec::new(),
            Some(v) => {
                let bad = || "scenario.wind_steps: must be a list of [t_s, d_north, d_east, d_down] rows, t_s >= 0".to_string();
                let mut out = Vec::new();
                for row in v.as_array().ok_or_else(bad)? {
                    let r: Vec<f64> = row.as_array().ok_or_else(bad)?.iter().map(|x| number(x).ok_or_else(bad)).collect::<Result<_, _>>()?;
                    let [t, n, e, d] = r.as_slice() else { return Err(bad()) };
                    if *t < 0.0 {
                        return Err(bad());
                    }
                    out.push(((t * 1e6).round() as u64, [*n, *e, *d]));
                }
                out
            }
        };
        let mag_fault = match s.get("mag_fault") {
            None => None,
            Some(Value::Table(t)) => {
                known_keys(t, "scenario.mag_fault", &["from_s", "kind", "deg", "below_m", "axis", "rate_dps"])?;
                let kind = match t.get("kind").and_then(Value::as_str) {
                    Some("rotate") => MagFaultKind::Rotate,
                    Some("stuck") => MagFaultKind::Stuck,
                    _ => return Err("scenario.mag_fault.kind: must be \"rotate\" or \"stuck\"".to_string()),
                };
                let from_s = scalar(t, "scenario.mag_fault", "from_s")?;
                let deg = if kind == MagFaultKind::Rotate { scalar(t, "scenario.mag_fault", "deg")? } else { 0.0 };
                let axis = match t.get("axis").map(Value::as_str) {
                    None | Some(Some("z")) => 2,
                    Some(Some("x")) => 0,
                    Some(Some("y")) => 1,
                    _ => return Err("scenario.mag_fault.axis: must be \"x\", \"y\" or \"z\"".to_string()),
                };
                let rate_dps = optional_scalar(t, "scenario.mag_fault", "rate_dps")?;
                if rate_dps.is_some_and(|r| kind != MagFaultKind::Rotate || !(r.is_finite() && r > 0.0)) {
                    return Err("scenario.mag_fault.rate_dps: only with kind \"rotate\", and > 0".to_string());
                }
                Some(MagFault {
                    from_us: (from_s * 1e6).round() as i64,
                    kind,
                    deg_rad: deg.to_radians(),
                    below_m: optional_scalar(t, "scenario.mag_fault", "below_m")?,
                    axis,
                    rate_rad_s: rate_dps.map(f64::to_radians),
                })
            }
            Some(_) => return Err("scenario.mag_fault: must be an inline table".to_string()),
        };
        let yaw_inject = match s.get("yaw_inject") {
            None => None,
            Some(Value::Table(t)) => {
                known_keys(t, "scenario.yaw_inject", &["at_s", "deg"])?;
                let at_s = scalar(t, "scenario.yaw_inject", "at_s")?;
                let deg = scalar(t, "scenario.yaw_inject", "deg")?;
                if !(at_s.is_finite() && at_s >= 0.0 && deg.is_finite()) {
                    return Err("scenario.yaw_inject: at_s must be >= 0 and deg finite".to_string());
                }
                Some(YawInject { at_us: (at_s * 1e6).round() as u64, rad: deg.to_radians() })
            }
            Some(_) => return Err("scenario.yaw_inject: must be an inline table".to_string()),
        };
        let yaw_hold = match s.get("yaw_hold") {
            None => None,
            Some(Value::Table(t)) => {
                known_keys(t, "scenario.yaw_hold", &["at_s", "deg"])?;
                let at_s = scalar(t, "scenario.yaw_hold", "at_s")?;
                let deg = scalar(t, "scenario.yaw_hold", "deg")?;
                if !(at_s.is_finite() && at_s >= 0.0 && deg.is_finite()) {
                    return Err("scenario.yaw_hold: at_s must be >= 0 and deg finite".to_string());
                }
                Some(YawInject { at_us: (at_s * 1e6).round() as u64, rad: deg.to_radians() })
            }
            Some(_) => return Err("scenario.yaw_hold: must be an inline table".to_string()),
        };
        let gnss_jump = match s.get("gnss_jump") {
            None => None,
            Some(Value::Table(t)) => {
                known_keys(t, "scenario.gnss_jump", &["at_s", "north_m", "east_m"])?;
                let at_s = scalar(t, "scenario.gnss_jump", "at_s")?;
                let (n, e) = (scalar(t, "scenario.gnss_jump", "north_m")?, scalar(t, "scenario.gnss_jump", "east_m")?);
                if !(at_s.is_finite() && at_s >= 0.0 && n.is_finite() && e.is_finite()) {
                    return Err("scenario.gnss_jump: at_s must be >= 0 and the metres finite".to_string());
                }
                Some(((at_s * 1e6).round() as u64, [n, e]))
            }
            Some(_) => return Err("scenario.gnss_jump: must be an inline table".to_string()),
        };
        let motor_thrust = match s.get("motor_thrust") {
            None => None,
            Some(Value::Table(t)) => {
                known_keys(t, "scenario.motor_thrust", &["at_s", "line", "scale"])?;
                let at_s = scalar(t, "scenario.motor_thrust", "at_s")?;
                let line = scalar(t, "scenario.motor_thrust", "line")?;
                let scale = scalar(t, "scenario.motor_thrust", "scale")?;
                if !(at_s.is_finite() && at_s >= 0.0 && [0.0, 1.0, 2.0, 3.0].contains(&line) && (0.0..=1.0).contains(&scale)) {
                    return Err("scenario.motor_thrust: at_s >= 0, line 0 to 3, scale 0 to 1".to_string());
                }
                Some(((at_s * 1e6).round() as u64, line as usize, scale))
            }
            Some(_) => return Err("scenario.motor_thrust: must be an inline table".to_string()),
        };
        let push = match s.get("push") {
            None => None,
            Some(Value::Table(t)) => {
                known_keys(t, "scenario.push", &["from_s", "to_s", "north_n", "east_n"])?;
                let (a, b) = (scalar(t, "scenario.push", "from_s")?, scalar(t, "scenario.push", "to_s")?);
                let (n, e) = (scalar(t, "scenario.push", "north_n")?, scalar(t, "scenario.push", "east_n")?);
                if !(a.is_finite() && b.is_finite() && a >= 0.0 && b > a && n.is_finite() && e.is_finite()) {
                    return Err("scenario.push: 0 <= from_s < to_s and finite forces".to_string());
                }
                Some(((a * 1e6).round() as u64, (b * 1e6).round() as u64, [n, e]))
            }
            Some(_) => return Err("scenario.push: must be an inline table".to_string()),
        };
        let mut disturbances = Vec::new();
        for (i, row) in rows(&root, "", "disturbance", &["from_s", "to_s", "torque_nm"])?.into_iter().enumerate() {
            let name = format!("disturbance[{i}]");
            let to_us = if row.contains_key("to_s") { micros(row, &name, "to_s")? } else { u64::MAX };
            disturbances.push(Disturbance { from_us: micros(row, &name, "from_s")?, to_us, torque_nm: vec3(row, &name, "torque_nm")? });
        }
        let settle_s = if ground || over {
            let x = scalar(i, "initial", "settle_s")?;
            if x < 0.0 {
                return Err("initial.settle_s: must be >= 0".to_string());
            }
            x
        } else {
            0.0
        };
        let (position_ned_m, velocity_ned_ms, attitude, rates_rads, start) = if ground {
            let bad = || "initial.ground_roll_pitch_deg: must be two numbers".to_string();
            let a = get(i, "initial", "ground_roll_pitch_deg")?.as_array().ok_or_else(bad)?;
            let [r, p] = a.as_slice() else { return Err(bad()) };
            let (r, p) = (number(r).ok_or_else(bad)?, number(p).ok_or_else(bad)?);
            let yaw = scalar(i, "initial", "yaw_deg")?;
            let start = Start::Ground { roll_pitch_rad: [r.to_radians(), p.to_radians()], yaw_rad: yaw.to_radians() };
            ([0.0; 3], [0.0; 3], quat_from_euler_deg([r, p, yaw]), [0.0; 3], start)
        } else {
            (
                vec3(i, "initial", "position_ned_m")?,
                vec3(i, "initial", "velocity_ned_ms")?,
                quat_from_euler_deg(vec3(i, "initial", "roll_pitch_yaw_deg")?),
                vec3(i, "initial", "rates_rads")?,
                if over { Start::AirOverGround } else { Start::Air },
            )
        };
        Ok(Scenario {
            name,
            duration_s,
            gnss_rate_hz,
            mag_expected,
            gnss_velocity,
            baro_motion,
            position_ned_m,
            velocity_ned_ms,
            attitude,
            rates_rads,
            rotors,
            start,
            settle_s,
            rc_script,
            phases,
            pilot,
            rc_lost,
            baro_lost,
            gnss_lost,
            gnss_hold,
            mag_fault,
            yaw_inject,
            yaw_hold,
            gnss_jump,
            motor_thrust,
            push,
            gnss_latency_s,
            wind_steps,
            disturbances,
            verdict: verdict(&root)?,
        })
    }
}

impl Point {
    /// `"<kf_scale>,<inertia_scale>,<lag_s or ->"`, as `--point` takes it.
    pub fn parse(text: &str) -> Result<Point, String> {
        let bad = || format!("--point {text}: must be \"<kf_scale>,<inertia_scale>,<lag_s or ->\"");
        let parts: Vec<&str> = text.split(',').map(str::trim).collect();
        let [kf, inertia, lag] = parts.as_slice() else { return Err(bad()) };
        let num = |s: &str| s.parse::<f64>().ok().filter(|x| x.is_finite() && *x > 0.0).ok_or_else(bad);
        let lag_s = if *lag == "-" { None } else { Some(num(lag)?) };
        Ok(Point { kf_scale: num(kf)?, inertia_scale: num(inertia)?, lag_s })
    }

    /// For the log and the table.
    pub fn text(&self) -> String {
        let lag = self.lag_s.map_or("card".to_string(), |l| format!("{l:.3}"));
        format!("kF x{} J x{} lag {lag}", self.kf_scale, self.inertia_scale)
    }
}

impl Map {
    /// For the log and the table.
    pub fn text(&self) -> String {
        self.speed_at_level_min.map_or("card".to_string(), |w| format!("speed_at_level_min {w} rad/s"))
    }
}

impl Air {
    pub fn text(&self) -> String {
        let drag = self.drag_rotor_linear.map_or("card".to_string(), |d| format!("{d}"));
        let wind = self.wind_ned_ms.map_or("env".to_string(), |w| format!("{w:?}"));
        format!("drag {drag} wind_ned {wind}")
    }
}

impl Ground {
    /// One piece of text per thing it changes; none when it changes nothing.
    pub fn parts(&self) -> Vec<String> {
        let mut parts = Vec::new();
        if let Some(b) = self.imu1_gyro_bias_rads {
            parts.push(format!("imu1 gyro_bias {b:?} rad/s"));
        }
        if let Some(b) = self.imu1_accel_bias_ms2 {
            parts.push(format!("imu1 accel_bias {b:?} m/s^2"));
        }
        if let Some([r, p]) = self.roll_pitch_deg {
            parts.push(format!("slope roll {r} pitch {p} deg"));
        }
        parts
    }
}

/// Relative slack for a value that sits on the edge of a range (0.7 against 1 - 30 / 100).
const EDGE: f64 = 1e-9;

/// The numeric interval of a record's range. `range = "ask"` and no range have none: there is nothing to leave.
fn interval(v: Option<f64>, range: &Option<Range>) -> Option<(f64, f64)> {
    match *range {
        Some(Range::Abs(lo, hi)) => Some((lo, hi)),
        Some(Range::Pct(lo, hi)) => {
            let v = v?;
            let (a, b) = (v * (1.0 + lo / 100.0), v * (1.0 + hi / 100.0));
            Some((a.min(b), a.max(b)))
        }
        _ => None,
    }
}

/// `Some(text)` when `x` lies outside the numeric range of the card number `key`.
fn outside(key: &str, unit: &str, x: f64, v: Option<f64>, range: &Option<Range>) -> Option<String> {
    let (lo, hi) = interval(v, range)?;
    let slack = EDGE * lo.abs().max(hi.abs());
    (!(x >= lo - slack && x <= hi + slack))
        .then(|| format!("{key} = {x} {unit} is outside its card range [{lo}, {hi}] {unit}"))
}

fn optional_scalar(t: &Table, table: &str, key: &str) -> Result<Option<f64>, String> {
    if t.contains_key(key) { scalar(t, table, key).map(Some) } else { Ok(None) }
}

fn optional_vec3(t: &Table, table: &str, key: &str) -> Result<Option<[f64; 3]>, String> {
    if t.contains_key(key) { vec3(t, table, key).map(Some) } else { Ok(None) }
}

impl Scenario {
    /// The same scenario on a ground of this roll and pitch (deg); an error unless it starts held on the ground.
    pub fn on_slope(mut self, roll_pitch_deg: [f64; 2]) -> Result<Scenario, String> {
        let Start::Ground { yaw_rad, .. } = self.start else {
            return Err(format!("scenario {}: a ground slope needs [initial] contact = \"ground\"", self.name));
        };
        let [r, p] = roll_pitch_deg;
        self.start = Start::Ground { roll_pitch_rad: [r.to_radians(), p.to_radians()], yaw_rad };
        self.attitude = quat_from_euler_deg([r, p, yaw_rad.to_degrees()]);
        Ok(self)
    }
}

impl Spread {
    pub fn load(path: impl AsRef<Path>) -> Result<Spread, String> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: cannot read: {e}", path.display()))?;
        let at = |e: String| format!("{}: {e}", path.display());
        let root: Table = text.parse().map_err(|e: toml::de::Error| at(format!("toml: {}", e.to_string().trim_end())))?;
        known_keys(&root, "spread", &["scenarios", "points", "maps", "air", "ground", "seeds"]).map_err(at)?;
        let dir = path.parent().unwrap_or(Path::new(""));
        let names = get(&root, "spread", "scenarios").map_err(at)?.as_array().ok_or(at("scenarios: must be a list".into()))?;
        let mut scenarios = Vec::new();
        for n in names {
            scenarios.push(dir.join(n.as_str().ok_or(at("scenarios: must be a list of file names".into()))?));
        }
        let mut points = Vec::new();
        for (i, row) in rows(&root, "spread", "points", &["kf_scale", "inertia_scale", "lag_s"]).map_err(at)?.into_iter().enumerate() {
            let name = format!("points[{i}]");
            let lag_s = if row.contains_key("lag_s") { Some(scalar(row, &name, "lag_s").map_err(at)?) } else { None };
            points.push(Point {
                kf_scale: scalar(row, &name, "kf_scale").map_err(at)?,
                inertia_scale: scalar(row, &name, "inertia_scale").map_err(at)?,
                lag_s,
            });
        }
        if scenarios.is_empty() || points.is_empty() {
            return Err(at("needs at least one scenario and one point".into()));
        }

        let mut maps = Vec::new();
        for (i, row) in rows(&root, "spread", "maps", &["speed_at_level_min"]).map_err(at)?.into_iter().enumerate() {
            let name = format!("maps[{i}]");
            maps.push(Map { speed_at_level_min: optional_scalar(row, &name, "speed_at_level_min").map_err(at)? });
        }
        let mut air = Vec::new();
        for (i, row) in rows(&root, "spread", "air", &["drag", "wind_ned"]).map_err(at)?.into_iter().enumerate() {
            let name = format!("air[{i}]");
            air.push(Air {
                drag_rotor_linear: optional_scalar(row, &name, "drag").map_err(at)?,
                wind_ned_ms: optional_vec3(row, &name, "wind_ned").map_err(at)?,
            });
        }
        let mut ground = Vec::new();
        let ground_keys = ["imu1_gyro_bias_rads", "imu1_accel_bias_ms2", "roll_pitch_deg"];
        for (i, row) in rows(&root, "spread", "ground", &ground_keys).map_err(at)?.into_iter().enumerate() {
            let name = format!("ground[{i}]");
            let roll_pitch_deg = match row.get("roll_pitch_deg") {
                None => None,
                Some(v) => match v.as_array().map(Vec::as_slice) {
                    Some([r, p]) => match (number(r), number(p)) {
                        (Some(r), Some(p)) => Some([r, p]),
                        _ => return Err(at(format!("{name}.roll_pitch_deg: must be two numbers"))),
                    },
                    _ => return Err(at(format!("{name}.roll_pitch_deg: must be two numbers"))),
                },
            };
            ground.push(Ground {
                imu1_gyro_bias_rads: optional_vec3(row, &name, "imu1_gyro_bias_rads").map_err(at)?,
                imu1_accel_bias_ms2: optional_vec3(row, &name, "imu1_accel_bias_ms2").map_err(at)?,
                roll_pitch_deg,
            });
        }
        let seeds = match root.get("seeds") {
            None => None,
            Some(v) => {
                let bad = || at("seeds: must be a non-empty list of distinct integers >= 0".to_string());
                let list: Vec<u64> = v
                    .as_array()
                    .ok_or_else(bad)?
                    .iter()
                    .map(|x| x.as_integer().and_then(|i| u64::try_from(i).ok()).ok_or_else(bad))
                    .collect::<Result<_, _>>()?;
                if list.is_empty() || list.iter().enumerate().any(|(i, s)| list[..i].contains(s)) {
                    return Err(bad());
                }
                Some(list)
            }
        };
        if maps.len() > 26 {
            return Err(at("maps: at most 26 values (the cell name letters A to Z)".into()));
        }
        if maps.is_empty() && root.contains_key("maps") || air.is_empty() && root.contains_key("air")
            || ground.is_empty() && root.contains_key("ground") {
            return Err(at("maps, air and ground: a list that is given needs at least one value".into()));
        }
        if maps.is_empty() {
            maps.push(Map::default());
        }
        if air.is_empty() {
            air.push(Air::default());
        }
        if ground.is_empty() {
            ground.push(Ground::default());
        }
        Ok(Spread { scenarios, points, maps, air, ground, seeds })
    }

    /// Every cell, scenarios outermost, seeds innermost. `scenario_names` are the loaded scenarios' names, in the
    /// file's order. `default_seed` is `--seed`: needed when the file has no `seeds` axis, refused when it has one.
    pub fn cells(&self, scenario_names: &[String], default_seed: Option<u64>) -> Result<Vec<Cell>, String> {
        let seeds = match (&self.seeds, default_seed) {
            (Some(list), None) => list.clone(),
            (None, Some(seed)) => vec![seed],
            (Some(_), Some(_)) => return Err("--seed: this file has a seeds axis; remove --seed".to_string()),
            (None, None) => return Err("--seed is required: this file has no seeds axis".to_string()),
        };
        let prefix = scenario_names.len() > 1;
        let mut cells = Vec::new();
        for (scenario, scenario_name) in scenario_names.iter().enumerate() {
            for point in 0..self.points.len() {
                for map in 0..self.maps.len() {
                    for air in 0..self.air.len() {
                        for ground in 0..self.ground.len() {
                            for &seed in &seeds {
                                let head = if prefix { format!("{scenario_name}-") } else { String::new() };
                                let name = format!("{head}p{point}-map{}-air{}-gnd{}-seed{seed}",
                                    (b'A' + map as u8) as char, air + 1, ground + 1);
                                cells.push(Cell { name, scenario, point, map, air, ground, seed });
                            }
                        }
                    }
                }
            }
        }
        Ok(cells)
    }

    /// What this cell changes that lies outside the card's numeric range of that number (card decision, section 8).
    /// A number whose range is "ask" or missing has no numeric range and is not checked.
    pub fn range_errors(&self, cell: &Cell, card: &Card, env: &Env) -> Vec<String> {
        let mut errs = Vec::new();
        let p = self.points[cell.point];
        let prop = &card.propulsion;
        let thrust = &prop.thrust_coefficient;
        if let Some(k) = thrust.v {
            errs.extend(outside("point kf_scale -> propulsion.thrust_coefficient", &thrust.unit, k * p.kf_scale, thrust.v, &thrust.range));
        }
        let inertia = &card.mass.inertia;
        if let Some(j) = inertia.v {
            for (r, row) in j.iter().enumerate() {
                for (c, &x) in row.iter().enumerate() {
                    errs.extend(outside(&format!("point inertia_scale -> mass.inertia[{r}][{c}]"), &inertia.unit,
                        x * p.inertia_scale, Some(x), &inertia.range));
                }
            }
        }
        if let Some(lag) = p.lag_s {
            for (key, rec) in [("propulsion.time_constant_up", &prop.time_constant_up),
                ("propulsion.time_constant_down", &prop.time_constant_down)] {
                errs.extend(outside(&format!("point lag_s -> {key}"), &rec.unit, lag, rec.v, &rec.range));
            }
        }
        if let Some(w) = self.maps[cell.map].speed_at_level_min {
            let r = &prop.speed_at_level_min;
            errs.extend(outside("map speed_at_level_min -> propulsion.speed_at_level_min", &r.unit, w, r.v, &r.range));
        }
        let air = &self.air[cell.air];
        if let Some(d) = air.drag_rotor_linear {
            let r = &card.drag_rotor_linear;
            errs.extend(outside("air drag -> drag.rotor_linear", &r.unit, d, r.v, &r.range));
        }
        if let Some(w) = air.wind_ned_ms {
            let r = &env.wind_mean_ned;
            for (k, x) in w.into_iter().enumerate() {
                errs.extend(outside(&format!("air wind_ned[{k}] -> wind.mean_ned"), &r.unit, x, r.v.map(|v| v[k]), &r.range));
            }
        }
        let g = &self.ground[cell.ground];
        for (key, set, rec) in [
            ("imu1.gyro_bias", g.imu1_gyro_bias_rads, &card.imu1.gyro_bias),
            ("imu1.accel_bias", g.imu1_accel_bias_ms2, &card.imu1.accel_bias),
        ] {
            for (k, x) in set.into_iter().flatten().enumerate() {
                errs.extend(outside(&format!("ground {key}[{k}]"), &rec.unit, x, rec.v.map(|v| v[k]), &rec.range));
            }
        }
        errs
    }
}
