//! The scenario file and the spread file. Unknown keys are an error in every table and every row.

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
    /// The stick script, by time; each row holds until the next. The first row is at 0.
    pub rc_script: Vec<RcRow>,
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

fn verdict(root: &Table) -> Result<Verdict, String> {
    let v = root
        .get("verdict")
        .and_then(Value::as_table)
        .ok_or("[verdict]: missing table (a scenario without a verdict is a load error)")?;
    let check = get(v, "verdict", "check")?.as_str().unwrap_or("");
    let keys: &[&str] = match check {
        "hover" => &["from_s", "disturbance_s", "settle_from_s", "max_tilt_deg", "max_nav_gap_deg",
            "max_heading_change_deg", "max_peak_tilt_deg"],
        "steps" => &["roll_at_s", "pitch_at_s", "yaw_at_s", "hold_s", "max_rise_s", "max_overshoot_rp_pct",
            "max_overshoot_yaw_pct"],
        "stick_sign" => &["pulses", "min_move_deg", "max_off_axis_ratio", "throttle_from_s", "throttle_to_s",
            "min_accel_rise_ms2"],
        "rc_loss" => &["on_high_s", "low_s", "arm_s", "throttle_s", "lost_at_s", "rc_fs_timeout_s", "stop_bound_s",
            "resume_s", "rearm_s", "off_s", "off_bound_s"],
        _ => return Err("verdict.check: must be \"hover\", \"steps\", \"stick_sign\" or \"rc_loss\"".to_string()),
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
            if !["scenario", "initial", "rc", "disturbance", "verdict"].contains(&key.as_str()) {
                return Err(format!("{key}: unknown table"));
            }
        }
        let s = table(&root, "scenario", &["name", "duration_s"])?;
        let i = table(
            &root,
            "initial",
            &["position_ned_m", "velocity_ned_ms", "roll_pitch_yaw_deg", "rates_rads", "rotors"],
        )?;

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
        let rc = table(&root, "rc", &["script", "lost"])?;
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
        Ok(Scenario {
            name,
            duration_s,
            position_ned_m: vec3(i, "initial", "position_ned_m")?,
            velocity_ned_ms: vec3(i, "initial", "velocity_ned_ms")?,
            attitude: quat_from_euler_deg(vec3(i, "initial", "roll_pitch_yaw_deg")?),
            rates_rads: vec3(i, "initial", "rates_rads")?,
            rotors,
            rc_script: rc_script(rc)?,
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
