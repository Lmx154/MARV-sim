//! The scenario file and the spread file. Unknown keys are an error in every table and every row.

use crate::backend::Start;
use crate::math::{Quat, quat_mul};
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
    pub position_ned_m: [f64; 3],
    pub velocity_ned_ms: [f64; 3],
    /// Body to world `[w, x, y, z]`, from the file's roll, pitch, yaw.
    pub attitude: Quat,
    pub rates_rads: [f64; 3],
    pub rotors: Rotors,
    /// `[initial] contact`: `Start::Air` ("air", the default) or `Start::Ground` ("ground").
    pub start: Start,
    /// The stick script, by time; each row holds until the next. The first row is at 0. Empty in the phase form.
    pub rc_script: Vec<RcRow>,
    /// `[rc] phase`: each phase starts when the previous one ends. Empty in the script form.
    pub phases: Vec<Phase>,
    /// `[pilot]`: present exactly when `phases` is not empty.
    pub pilot: Option<PilotParams>,
    /// `[from_us, to_us)` intervals in which no stick report is made.
    pub rc_lost: Vec<(u64, u64)>,
    pub disturbances: Vec<Disturbance>,
    pub verdict: Verdict,
}

/// One row of `[rc] script`. Roll, pitch, yaw -1..1 (right, forward, clockwise +); throttle 0..1; arm = the switch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcRow {
    pub t_us: u64,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub arm: bool,
}

/// `[pilot]`: the sim pilot's limits and gains (decision-sim-takeoff-landing, sections 2 and 10).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PilotParams {
    pub delay_s: f64,
    pub stick_rate_per_s: f64,
    pub ramp_ratio_per_s: f64,
    pub airborne_m: f64,
    pub k_h: f64,
    pub k_v: f64,
    pub k_i: f64,
    pub accel_max_g: f64,
    pub v_up_ms: f64,
    pub v_dn_ms: f64,
    pub xy_k: f64,
    pub xy_stick_max: f64,
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
    /// Thrust ratio up from 0 at `ramp_ratio_per_s`.
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
    /// decision-sim-takeoff-landing, section 3. `airborne_m` is `[pilot] airborne_m`, the verdict's lift-off height.
    TakeoffLand {
        airborne_m: f64,
        arm_bound_s: f64,
        r_stance_m: f64,
        stance_floor_weight: f64,
        max_stance_ratio: f64,
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

/// The spread file: the scenario files (relative to the spread file) and the points.
#[derive(Debug, Clone, PartialEq)]
pub struct Spread {
    pub scenarios: Vec<PathBuf>,
    pub points: Vec<Point>,
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
    let keys = ["t_s", "roll", "pitch", "yaw", "throttle", "arm"];
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
        ramp_ratio_per_s: f("ramp_ratio_per_s")?,
        airborne_m: f("airborne_m")?,
        k_h: f("k_h")?,
        k_v: f("k_v")?,
        k_i: f("k_i")?,
        accel_max_g: f("accel_max_g")?,
        v_up_ms: f("v_up_ms")?,
        v_dn_ms: f("v_dn_ms")?,
        xy_k: f("xy_k")?,
        xy_stick_max: f("xy_stick_max")?,
    })
}

const PILOT_KEYS: [&str; 12] = ["delay_s", "stick_rate_per_s", "ramp_ratio_per_s", "airborne_m", "k_h", "k_v", "k_i",
    "accel_max_g", "v_up_ms", "v_dn_ms", "xy_k", "xy_stick_max"];

fn rc_phases(rc: &Table) -> Result<Vec<Phase>, String> {
    let keys = ["name", "after_s", "event", "below_m", "timeout_s", "arm", "throttle", "height_m", "descend_ms", "xy",
        "roll", "pitch", "yaw"];
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
        phases.push(Phase { name, end, arm, throttle, roll: stick("roll")?, pitch: stick("pitch")?, yaw: stick("yaw")?,
            xy_hold });
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
        "takeoff_land" => &["arm_bound_s", "r_stance_m", "stance_floor_weight", "max_stance_ratio", "liftoff_window_s",
            "max_liftoff_tilt_deg", "height_band_m", "max_tilt_deg", "max_speed_h_ms", "max_nav_gap_deg",
            "max_td_vz_ms", "max_td_vh_ms", "max_td_tilt_deg", "max_td_rate_dps", "idle_after_s", "stop_bound_s",
            "hover_trim_tolerance"],
        _ => return Err("verdict.check: must be \"hover\", \"steps\", \"stick_sign\", \"rc_loss\" or \"takeoff_land\""
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
        let s = table(&root, "scenario", &["name", "duration_s"])?;
        let ground = match root.get("initial").and_then(Value::as_table).and_then(|i| i.get("contact")) {
            None => false,
            Some(Value::String(c)) if c == "air" => false,
            Some(Value::String(c)) if c == "ground" => true,
            Some(_) => return Err("initial.contact: must be \"air\" or \"ground\"".to_string()),
        };
        let i = if ground {
            table(&root, "initial", &["contact", "ground_roll_pitch_deg", "yaw_deg", "rotors"])?
        } else {
            table(
                &root,
                "initial",
                &["contact", "position_ned_m", "velocity_ned_ms", "roll_pitch_yaw_deg", "rates_rads", "rotors"],
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
        let mut disturbances = Vec::new();
        for (i, row) in rows(&root, "", "disturbance", &["from_s", "to_s", "torque_nm"])?.into_iter().enumerate() {
            let name = format!("disturbance[{i}]");
            let to_us = if row.contains_key("to_s") { micros(row, &name, "to_s")? } else { u64::MAX };
            disturbances.push(Disturbance { from_us: micros(row, &name, "from_s")?, to_us, torque_nm: vec3(row, &name, "torque_nm")? });
        }
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
                Start::Air,
            )
        };
        Ok(Scenario {
            name,
            duration_s,
            position_ned_m,
            velocity_ned_ms,
            attitude,
            rates_rads,
            rotors,
            start,
            rc_script,
            phases,
            pilot,
            rc_lost,
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

impl Spread {
    pub fn load(path: impl AsRef<Path>) -> Result<Spread, String> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: cannot read: {e}", path.display()))?;
        let at = |e: String| format!("{}: {e}", path.display());
        let root: Table = text.parse().map_err(|e: toml::de::Error| at(format!("toml: {}", e.to_string().trim_end())))?;
        known_keys(&root, "spread", &["scenarios", "points"]).map_err(at)?;
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
        Ok(Spread { scenarios, points })
    }
}
