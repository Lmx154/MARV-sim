//! The scenario file: what this step uses of it. Unknown keys are an error.

use crate::math::{Quat, quat_mul};
use std::path::Path;
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
}

/// The longest name that still fits, with its prefix, in the log's MSG text (64 bytes).
const NAME_MAX: usize = 50;

fn table<'a>(root: &'a Table, name: &str, allowed: &[&str]) -> Result<&'a Table, String> {
    let t = root
        .get(name)
        .and_then(Value::as_table)
        .ok_or_else(|| format!("[{name}]: missing table"))?;
    for key in t.keys() {
        if !allowed.contains(&key.as_str()) {
            return Err(format!("{name}.{key}: unknown key"));
        }
    }
    Ok(t)
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
            if key != "scenario" && key != "initial" {
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
        Ok(Scenario {
            name,
            duration_s,
            position_ned_m: vec3(i, "initial", "position_ned_m")?,
            velocity_ned_ms: vec3(i, "initial", "velocity_ned_ms")?,
            attitude: quat_from_euler_deg(vec3(i, "initial", "roll_pitch_yaw_deg")?),
            rates_rads: vec3(i, "initial", "rates_rads")?,
            rotors,
        })
    }
}
