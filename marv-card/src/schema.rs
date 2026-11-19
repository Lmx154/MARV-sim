//! Parsing and checking of the MARV vehicle card and environment file.
//!
//! Schema and rules: MARV-V2-drivers/_plan/decision-sim-card-20261119.md.
//! This file uses only std, `toml` and `sha2`, so a later `build.rs` can `include!` it.
//! The toml values are walked by hand: one function checks one record, the rest is plain lookups.

use sha2::{Digest, Sha256};
use std::collections::HashSet;
use std::path::Path;
use std::process::Command;
use toml::{Table, Value};

// ---------------------------------------------------------------------------
// The record
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tag {
    Measured,
    Datasheet,
    Cad,
    Derived,
    Placeholder,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Range {
    /// `range = [min, max]`, same unit, applies to every component.
    Abs(f64, f64),
    /// `range_pct = [lo, hi]`, percent of `v`.
    Pct(f64, f64),
    /// `range = "ask"`: the owner has not given it.
    Ask,
}

/// One `{ v, unit, tag, src, range }` record. `v == None` means `v = "ask"`.
#[derive(Debug, Clone, PartialEq)]
pub struct Record<T> {
    pub v: Option<T>,
    pub unit: String,
    pub tag: Tag,
    pub src: String,
    pub range: Option<Range>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Spin {
    Cw,
    Ccw,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Motor {
    pub dshot_line: u8,
    pub spin: Spin,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Motors {
    pub front_right: Motor,
    pub rear_right: Motor,
    pub front_left: Motor,
    pub rear_left: Motor,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CadExport {
    pub file: String,
    pub sha256: String,
    pub cad_commit: String,
    pub configuration: String,
    pub measured_share_item_pct: f64,
    pub region_sha256: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Mass {
    pub mass: Record<f64>,
    pub com_in_cad: Record<[f64; 3]>,
    pub inertia: Record<[[f64; 3]; 3]>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RotorPositions {
    pub front_right: Record<[f64; 3]>,
    pub rear_right: Record<[f64; 3]>,
    pub front_left: Record<[f64; 3]>,
    pub rear_left: Record<[f64; 3]>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Propulsion {
    pub thrust_coefficient: Record<f64>,
    pub moment_coefficient: Record<f64>,
    pub max_rotor_speed: Record<f64>,
    pub time_constant_up: Record<f64>,
    pub time_constant_down: Record<f64>,
    pub level_min: Record<f64>,
    pub level_max: Record<f64>,
    pub idle_level: Record<f64>,
    pub speed_at_level_min: Record<f64>,
}

/// One IMU's model numbers: position from the centre of mass (FRD, m), noise sd and bias per axis, latency.
#[derive(Debug, Clone, PartialEq)]
pub struct ImuProfile {
    pub position: Record<[f64; 3]>,
    pub gyro_noise_sd: Record<[f64; 3]>,
    pub accel_noise_sd: Record<[f64; 3]>,
    pub gyro_bias: Record<[f64; 3]>,
    pub accel_bias: Record<[f64; 3]>,
    pub latency: Record<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Card {
    pub name: String,
    pub schema: i64,
    pub motors: Motors,
    pub cad_export: CadExport,
    pub mass: Mass,
    pub rotor_position: RotorPositions,
    pub propulsion: Propulsion,
    pub drag_body: Record<[f64; 3]>,
    pub battery_voltage: Record<f64>,
    pub imu1: ImuProfile,
    pub imu2: ImuProfile,
    /// Lowercase hex SHA-256 of the file bytes.
    pub sha256: String,
    open: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Env {
    pub name: String,
    pub schema: i64,
    pub home_override: String,
    pub gravity: Record<f64>,
    pub sea_level_pressure: Record<f64>,
    pub sea_level_temperature: Record<f64>,
    pub lapse_rate: Record<f64>,
    pub origin_latitude: Record<f64>,
    pub origin_longitude: Record<f64>,
    pub origin_altitude: Record<f64>,
    pub magnetic_field_ned: Record<[f64; 3]>,
    pub wind_mean_ned: Record<[f64; 3]>,
    pub wind_gust_sd: Record<f64>,
    open: Vec<String>,
}

const CARD_EXEMPT: [&str; 4] = ["card", "motors", "cad_export", "rotor_spin_cad"];
const ENV_EXEMPT: [&str; 1] = ["env"];
const POSITIONS: [&str; 4] = ["front_right", "rear_right", "front_left", "rear_left"];
const EMPTY_REGION: &str = "generated region is empty: run CAD:tools/make_card.py";

// ---------------------------------------------------------------------------
// Check 1 and 2: one record
// ---------------------------------------------------------------------------

fn num(v: &Value) -> Option<f64> {
    match v {
        // TOML allows `nan` and `inf`; neither is a number a card may carry.
        Value::Float(x) if x.is_finite() => Some(*x),
        Value::Integer(i) => Some(*i as f64),
        _ => None,
    }
}

fn vec3(v: &Value) -> Option<[f64; 3]> {
    let a = v.as_array()?;
    if a.len() != 3 {
        return None;
    }
    Some([num(&a[0])?, num(&a[1])?, num(&a[2])?])
}

fn mat3(v: &Value) -> Option<[[f64; 3]; 3]> {
    let a = v.as_array()?;
    if a.len() != 3 {
        return None;
    }
    Some([vec3(&a[0])?, vec3(&a[1])?, vec3(&a[2])?])
}

/// Every component of a number, a 3-vector or a 3x3 matrix, or None if `v` is none of these.
fn components(v: &Value) -> Option<Vec<f64>> {
    if let Some(x) = num(v) {
        return Some(vec![x]);
    }
    if let Some(a) = vec3(v) {
        return Some(a.to_vec());
    }
    mat3(v).map(|m| m.concat())
}

fn pair(v: &Value) -> Option<(f64, f64)> {
    let a = v.as_array()?;
    if a.len() != 2 {
        return None;
    }
    Some((num(&a[0])?, num(&a[1])?))
}

fn text_field(t: &Table, path: &str, key: &str, errs: &mut Vec<String>) -> String {
    match t.get(key) {
        Some(Value::String(s)) if !s.is_empty() => s.clone(),
        Some(_) => {
            errs.push(format!("{path}: {key} must be a non-empty string"));
            String::new()
        }
        None => {
            errs.push(format!("{path}: missing {key}"));
            String::new()
        }
    }
}

/// Parse and check one record (checks 1 and 2). `v` is kept as a toml value.
fn parse_raw(path: &str, val: &Value) -> Result<Record<Value>, Vec<String>> {
    let Some(t) = val.as_table() else {
        return Err(vec![format!(
            "{path}: expected a record {{ v, unit, tag, src }}, found a bare {}",
            val.type_str()
        )]);
    };
    let mut errs = Vec::new();
    for k in t.keys() {
        if !["v", "unit", "tag", "src", "range", "range_pct"].contains(&k.as_str()) {
            errs.push(format!("{path}: unknown key {k}"));
        }
    }

    let mut v = None;
    let mut ask = false;
    match t.get("v") {
        None => errs.push(format!("{path}: missing v")),
        Some(Value::String(s)) if s == "ask" => ask = true,
        Some(x) if components(x).is_some() => v = Some(x.clone()),
        Some(_) => errs.push(format!(
            "{path}: v must be a number, a 3-vector, a 3x3 matrix or \"ask\""
        )),
    }

    let unit = text_field(t, path, "unit", &mut errs);
    let src = text_field(t, path, "src", &mut errs);

    let tag = match t.get("tag") {
        None => {
            errs.push(format!("{path}: missing tag"));
            None
        }
        Some(Value::String(s)) => match s.as_str() {
            "measured" => Some(Tag::Measured),
            "datasheet" => Some(Tag::Datasheet),
            "cad" => Some(Tag::Cad),
            "derived" => Some(Tag::Derived),
            "placeholder" => Some(Tag::Placeholder),
            _ => {
                errs.push(format!(
                    "{path}: tag must be one of measured, datasheet, cad, derived, placeholder (got \"{s}\")"
                ));
                None
            }
        },
        Some(_) => {
            errs.push(format!("{path}: tag must be a string"));
            None
        }
    };

    let mut range = None;
    if t.contains_key("range") && t.contains_key("range_pct") {
        errs.push(format!("{path}: give only one of range and range_pct"));
    } else if let Some(r) = t.get("range") {
        match r {
            Value::String(s) if s == "ask" => range = Some(Range::Ask),
            _ => match pair(r) {
                Some((lo, hi)) if lo <= hi => range = Some(Range::Abs(lo, hi)),
                Some(_) => errs.push(format!("{path}: range must have min <= max")),
                None => errs.push(format!(
                    "{path}: range must be [min, max] or \"ask\""
                )),
            },
        }
    } else if let Some(r) = t.get("range_pct") {
        match pair(r) {
            Some((lo, hi)) if lo <= 0.0 && 0.0 <= hi => range = Some(Range::Pct(lo, hi)),
            Some(_) => errs.push(format!("{path}: range_pct must have lo <= 0 <= hi")),
            None => errs.push(format!("{path}: range_pct must be [lo, hi]")),
        }
    }

    // Check 2
    if tag == Some(Tag::Placeholder) && range.is_none() {
        errs.push(format!(
            "{path}: a placeholder must carry range = [min, max], range_pct = [lo, hi] or range = \"ask\""
        ));
    }
    if ask && tag.is_some() && tag != Some(Tag::Placeholder) {
        errs.push(format!(
            "{path}: v = \"ask\" is allowed only with tag placeholder"
        ));
    }
    if let (Some(Range::Abs(lo, hi)), Some(x)) = (range, &v)
        && let Some(cs) = components(x)
        && cs.iter().any(|c| *c < lo || *c > hi)
    {
        errs.push(format!("{path}: v is outside range [{lo}, {hi}]"));
    }

    let Some(tag) = tag else {
        return Err(errs);
    };
    if !errs.is_empty() {
        return Err(errs);
    }
    Ok(Record {
        v,
        unit,
        tag,
        src,
        range,
    })
}

/// Walk every non-exempt table (checks 1 and 2, and the open list), typed or not.
fn walk(root: &Table, exempt: &[&str], errs: &mut Vec<String>, open: &mut Vec<String>) {
    for (name, val) in root {
        if exempt.contains(&name.as_str()) {
            continue;
        }
        let Some(table) = val.as_table() else {
            errs.push(format!(
                "{name}: expected a table of records, found a bare {}",
                val.type_str()
            ));
            continue;
        };
        for (key, rec) in table {
            let path = format!("{name}.{key}");
            match parse_raw(&path, rec) {
                Err(e) => errs.extend(e),
                Ok(r) => {
                    let mut why = Vec::new();
                    if r.tag == Tag::Placeholder {
                        why.push("placeholder");
                    }
                    if r.v.is_none() {
                        why.push("v = \"ask\"");
                    }
                    if r.range == Some(Range::Ask) {
                        why.push("range = \"ask\"");
                    }
                    if !why.is_empty() {
                        open.push(format!("{path}: {} -- {}", why.join(", "), r.src));
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Typed lookups
// ---------------------------------------------------------------------------

trait Shape: Sized {
    const NAME: &'static str;
    fn from_value(v: &Value) -> Option<Self>;
}

impl Shape for f64 {
    const NAME: &'static str = "a number";
    fn from_value(v: &Value) -> Option<Self> {
        num(v)
    }
}

impl Shape for [f64; 3] {
    const NAME: &'static str = "a 3-vector";
    fn from_value(v: &Value) -> Option<Self> {
        vec3(v)
    }
}

impl Shape for [[f64; 3]; 3] {
    const NAME: &'static str = "a 3x3 matrix";
    fn from_value(v: &Value) -> Option<Self> {
        mat3(v)
    }
}

fn lookup<'a>(root: &'a Table, table: &str, key: &str, errs: &mut Vec<String>) -> Option<&'a Value> {
    let found = root
        .get(table)
        .and_then(Value::as_table)
        .and_then(|t| t.get(key));
    if found.is_none() {
        errs.push(format!("{table}.{key}: missing"));
    }
    found
}

fn rec<T: Shape>(root: &Table, table: &str, key: &str, errs: &mut Vec<String>) -> Option<Record<T>> {
    let val = lookup(root, table, key, errs)?;
    let path = format!("{table}.{key}");
    let raw = match parse_raw(&path, val) {
        Ok(r) => r,
        Err(e) => {
            errs.extend(e);
            return None;
        }
    };
    let v = match &raw.v {
        None => None,
        Some(x) => {
            let Some(t) = T::from_value(x) else {
                errs.push(format!("{path}: v must be {}", T::NAME));
                return None;
            };
            Some(t)
        }
    };
    Some(Record {
        v,
        unit: raw.unit,
        tag: raw.tag,
        src: raw.src,
        range: raw.range,
    })
}

fn str_at(root: &Table, table: &str, key: &str, errs: &mut Vec<String>) -> Option<String> {
    let val = lookup(root, table, key, errs)?;
    match val {
        Value::String(s) => Some(s.clone()),
        _ => {
            errs.push(format!("{table}.{key}: must be a string"));
            None
        }
    }
}

fn int(root: &Table, table: &str, key: &str, errs: &mut Vec<String>) -> Option<i64> {
    let val = lookup(root, table, key, errs)?;
    match val {
        Value::Integer(i) => Some(*i),
        _ => {
            errs.push(format!("{table}.{key}: must be an integer"));
            None
        }
    }
}

fn float(root: &Table, table: &str, key: &str, errs: &mut Vec<String>) -> Option<f64> {
    let val = lookup(root, table, key, errs)?;
    let n = num(val);
    if n.is_none() {
        errs.push(format!("{table}.{key}: must be a number"));
    }
    n
}

fn parse_toml(text: &str) -> Result<Table, Vec<String>> {
    text.parse::<Table>()
        .map_err(|e| vec![format!("toml: {}", e.to_string().trim_end())])
}

fn dedupe(errs: Vec<String>) -> Vec<String> {
    let mut seen = HashSet::new();
    errs.into_iter().filter(|e| seen.insert(e.clone())).collect()
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

// ---------------------------------------------------------------------------
// Check 3: generated region
// ---------------------------------------------------------------------------

/// Lowercase hex SHA-256 of the lines strictly between the BEGIN and END marker lines,
/// leaving out the `region_sha256 =` line, each remaining line ended by one `\n`.
pub fn region_hash(card_text: &str) -> Result<String, String> {
    let lines: Vec<&str> = card_text.split('\n').collect();
    let begins: Vec<usize> = (0..lines.len())
        .filter(|i| lines[*i].starts_with("# >>> BEGIN GENERATED"))
        .collect();
    let ends: Vec<usize> = (0..lines.len())
        .filter(|i| lines[*i].starts_with("# <<< END GENERATED"))
        .collect();
    if begins.len() != 1 || ends.len() != 1 {
        return Err(format!(
            "generated region: need exactly one BEGIN and one END marker line (found {} and {})",
            begins.len(),
            ends.len()
        ));
    }
    if begins[0] >= ends[0] {
        return Err("generated region: the BEGIN marker must come before the END marker".into());
    }
    let inside = &lines[begins[0] + 1..ends[0]];
    if inside
        .iter()
        .all(|l| l.trim().is_empty() || l.trim_start().starts_with('#'))
    {
        return Err(EMPTY_REGION.into());
    }
    let mut hasher = Sha256::new();
    for l in inside {
        if l.starts_with("region_sha256 =") {
            continue;
        }
        hasher.update(l.as_bytes());
        hasher.update(b"\n");
    }
    Ok(format!("{:x}", hasher.finalize()))
}

// ---------------------------------------------------------------------------
// Check 4: frame sanity
// ---------------------------------------------------------------------------

fn check_frame(rp: &RotorPositions, mass: &Mass, errs: &mut Vec<String>) {
    let rotors = [
        ("front_right", &rp.front_right, 1.0, 1.0),
        ("rear_right", &rp.rear_right, -1.0, 1.0),
        ("front_left", &rp.front_left, 1.0, -1.0),
        ("rear_left", &rp.rear_left, -1.0, -1.0),
    ];
    for (name, r, sx, sy) in rotors {
        let Some(p) = r.v else { continue };
        if p[0] * sx <= 0.0 {
            let want = if sx > 0.0 { "> 0 (front)" } else { "< 0 (rear)" };
            errs.push(format!("rotor_position.{name}: x must be {want}, got {}", p[0]));
        }
        if p[1] * sy <= 0.0 {
            let want = if sy > 0.0 { "> 0 (right)" } else { "< 0 (left)" };
            errs.push(format!("rotor_position.{name}: y must be {want}, got {}", p[1]));
        }
    }
    if let Some(m) = mass.mass.v
        && m <= 0.0
    {
        errs.push(format!("mass.mass: must be > 0, got {m}"));
    }
    let Some(j) = mass.inertia.v else { return };
    let mut symmetric = true;
    for (a, b) in [(0, 1), (0, 2), (1, 2)] {
        let (x, y) = (j[a][b], j[b][a]);
        if (x - y).abs() > 1e-9 * x.abs().max(y.abs()) {
            symmetric = false;
            errs.push(format!(
                "mass.inertia: not symmetric, element [{a}][{b}] = {x} but [{b}][{a}] = {y}"
            ));
        }
    }
    if symmetric {
        let det2 = j[0][0] * j[1][1] - j[0][1] * j[0][1];
        let det3 = j[0][0] * (j[1][1] * j[2][2] - j[1][2] * j[2][1])
            - j[0][1] * (j[1][0] * j[2][2] - j[1][2] * j[2][0])
            + j[0][2] * (j[1][0] * j[2][1] - j[1][1] * j[2][0]);
        if j[0][0] <= 0.0 || det2 <= 0.0 || det3 <= 0.0 {
            errs.push("mass.inertia: not positive definite".into());
        }
    }
    let (xx, yy, zz) = (j[0][0], j[1][1], j[2][2]);
    for (a, b, c, label) in [
        (xx, yy, zz, "Jxx + Jyy >= Jzz"),
        (yy, zz, xx, "Jyy + Jzz >= Jxx"),
        (zz, xx, yy, "Jzz + Jxx >= Jyy"),
    ] {
        if a + b < c {
            errs.push(format!("mass.inertia: violates {label}"));
        }
    }
}

// ---------------------------------------------------------------------------
// Check 5: motors
// ---------------------------------------------------------------------------

fn spin_of(s: &str) -> Option<Spin> {
    match s {
        "cw" => Some(Spin::Cw),
        "ccw" => Some(Spin::Ccw),
        _ => None,
    }
}

fn parse_motor(val: Option<&Value>, name: &str, errs: &mut Vec<String>) -> Option<Motor> {
    let path = format!("motors.{name}");
    let Some(t) = val.and_then(Value::as_table) else {
        errs.push(format!("{path}: missing or not a {{ dshot_line, spin }} table"));
        return None;
    };
    for k in t.keys() {
        if k != "dshot_line" && k != "spin" {
            errs.push(format!("{path}: unknown key {k}"));
        }
    }
    let line = match t.get("dshot_line") {
        Some(Value::Integer(i)) if (0..=3).contains(i) => Some(*i as u8),
        _ => {
            errs.push(format!("{path}: dshot_line must be an integer 0 to 3"));
            None
        }
    };
    let spin = match t.get("spin") {
        Some(Value::String(s)) => spin_of(s),
        _ => None,
    };
    if spin.is_none() {
        errs.push(format!("{path}: spin must be \"cw\" or \"ccw\""));
    }
    Some(Motor {
        dshot_line: line?,
        spin: spin?,
    })
}

fn imu_profile(root: &Table, table: &str, errs: &mut Vec<String>) -> Option<ImuProfile> {
    Some(ImuProfile {
        position: rec(root, table, "position", errs)?,
        gyro_noise_sd: rec(root, table, "gyro_noise_sd", errs)?,
        accel_noise_sd: rec(root, table, "accel_noise_sd", errs)?,
        gyro_bias: rec(root, table, "gyro_bias", errs)?,
        accel_bias: rec(root, table, "accel_bias", errs)?,
        latency: rec(root, table, "latency", errs)?,
    })
}

fn parse_motors(root: &Table, errs: &mut Vec<String>) -> Option<Motors> {
    let Some(t) = root.get("motors").and_then(Value::as_table) else {
        errs.push("motors: missing table".into());
        return None;
    };
    for k in t.keys() {
        if k != "src" && !POSITIONS.contains(&k.as_str()) {
            errs.push(format!("motors.{k}: unexpected key"));
        }
    }
    if !matches!(t.get("src"), Some(Value::String(s)) if !s.is_empty()) {
        errs.push("motors.src: must be a non-empty string".into());
    }
    let fr = parse_motor(t.get("front_right"), "front_right", errs);
    let rr = parse_motor(t.get("rear_right"), "rear_right", errs);
    let fl = parse_motor(t.get("front_left"), "front_left", errs);
    let rl = parse_motor(t.get("rear_left"), "rear_left", errs);
    if let (Some(a), Some(b), Some(c), Some(d)) = (fr, rr, fl, rl) {
        let mut lines = [a.dshot_line, b.dshot_line, c.dshot_line, d.dshot_line];
        lines.sort_unstable();
        if lines != [0, 1, 2, 3] {
            errs.push(format!(
                "motors: dshot_line values must be 0, 1, 2, 3 once each (got {:?})",
                [a.dshot_line, b.dshot_line, c.dshot_line, d.dshot_line]
            ));
        }
    }
    Some(Motors {
        front_right: fr?,
        rear_right: rr?,
        front_left: fl?,
        rear_left: rl?,
    })
}

fn check_spin_cad(root: &Table, motors: &Option<Motors>, errs: &mut Vec<String>) {
    let Some(t) = root.get("rotor_spin_cad").and_then(Value::as_table) else {
        errs.push("rotor_spin_cad: missing table".into());
        return;
    };
    for k in t.keys() {
        if !POSITIONS.contains(&k.as_str()) {
            errs.push(format!("rotor_spin_cad.{k}: unexpected key"));
        }
    }
    for name in POSITIONS {
        let Some(Value::String(s)) = t.get(name) else {
            errs.push(format!("rotor_spin_cad.{name}: missing or not a string"));
            continue;
        };
        let Some(m) = motors else { continue };
        let want = match name {
            "front_right" => m.front_right.spin,
            "rear_right" => m.rear_right.spin,
            "front_left" => m.front_left.spin,
            _ => m.rear_left.spin,
        };
        if spin_of(s) != Some(want) {
            errs.push(format!(
                "rotor_spin_cad.{name}: \"{s}\" differs from motors.{name} spin {want:?}"
            ));
        }
    }
}

// ---------------------------------------------------------------------------
// Card
// ---------------------------------------------------------------------------

impl Card {
    pub fn load(path: impl AsRef<Path>) -> Result<Card, Vec<String>> {
        let path = path.as_ref();
        let bytes = std::fs::read(path)
            .map_err(|e| vec![format!("{}: cannot read: {e}", path.display())])?;
        let text = String::from_utf8(bytes)
            .map_err(|_| vec![format!("{}: not valid UTF-8", path.display())])?;
        Card::parse(&text)
    }

    /// Checks 1 to 5 on the text of a card; returns every error found.
    pub fn parse(text: &str) -> Result<Card, Vec<String>> {
        let root = parse_toml(text)?;
        let mut errs = Vec::new();
        let mut open = Vec::new();
        walk(&root, &CARD_EXEMPT, &mut errs, &mut open);

        let name = str_at(&root, "card", "name", &mut errs);
        let schema = int(&root, "card", "schema", &mut errs);
        let motors = parse_motors(&root, &mut errs);

        let p = |k: &str, errs: &mut Vec<String>| rec::<f64>(&root, "propulsion", k, errs);
        let propulsion = (|| {
            Some(Propulsion {
                thrust_coefficient: p("thrust_coefficient", &mut errs)?,
                moment_coefficient: p("moment_coefficient", &mut errs)?,
                max_rotor_speed: p("max_rotor_speed", &mut errs)?,
                time_constant_up: p("time_constant_up", &mut errs)?,
                time_constant_down: p("time_constant_down", &mut errs)?,
                level_min: p("level_min", &mut errs)?,
                level_max: p("level_max", &mut errs)?,
                idle_level: p("idle_level", &mut errs)?,
                speed_at_level_min: p("speed_at_level_min", &mut errs)?,
            })
        })();
        let drag_body = rec::<[f64; 3]>(&root, "drag", "body", &mut errs);
        let battery_voltage = rec::<f64>(&root, "battery", "voltage", &mut errs);
        let imu1 = imu_profile(&root, "imu1", &mut errs);
        let imu2 = imu_profile(&root, "imu2", &mut errs);

        // Generated region (check 3) and the tables inside it. An empty region reports one
        // error; the missing tables behind it are not reported again.
        let region = region_hash(text);
        let empty = region.as_ref().is_err_and(|e| e == EMPTY_REGION);
        let mut gen_errs = Vec::new();
        let cad_export = (|| {
            let e = &mut gen_errs;
            Some(CadExport {
                file: str_at(&root, "cad_export", "file", e)?,
                sha256: str_at(&root, "cad_export", "sha256", e)?,
                cad_commit: str_at(&root, "cad_export", "cad_commit", e)?,
                configuration: str_at(&root, "cad_export", "configuration", e)?,
                measured_share_item_pct: float(&root, "cad_export", "measured_share_item_pct", e)?,
                region_sha256: str_at(&root, "cad_export", "region_sha256", e)?,
            })
        })();
        let mass = (|| {
            Some(Mass {
                mass: rec(&root, "mass", "mass", &mut gen_errs)?,
                com_in_cad: rec(&root, "mass", "com_in_cad", &mut gen_errs)?,
                inertia: rec(&root, "mass", "inertia", &mut gen_errs)?,
            })
        })();
        let rotor_position = (|| {
            let r = |k: &str, e: &mut Vec<String>| rec::<[f64; 3]>(&root, "rotor_position", k, e);
            Some(RotorPositions {
                front_right: r("front_right", &mut gen_errs)?,
                rear_right: r("rear_right", &mut gen_errs)?,
                front_left: r("front_left", &mut gen_errs)?,
                rear_left: r("rear_left", &mut gen_errs)?,
            })
        })();
        check_spin_cad(&root, &motors, &mut gen_errs);
        if let (Some(m), Some(rp)) = (&mass, &rotor_position) {
            check_frame(rp, m, &mut gen_errs);
        }
        match (&region, &cad_export) {
            (Ok(h), Some(ce)) if *h != ce.region_sha256 => gen_errs.push(format!(
                "cad_export.region_sha256: the generated region changed (file says {}, computed {h}); rerun CAD:tools/make_card.py",
                ce.region_sha256
            )),
            (Err(e), _) => errs.push(e.clone()),
            _ => {}
        }
        if !empty {
            errs.extend(gen_errs);
        }

        let errs = dedupe(errs);
        if !errs.is_empty() {
            return Err(errs);
        }
        (|| {
            Some(Card {
                name: name?,
                schema: schema?,
                motors: motors?,
                cad_export: cad_export?,
                mass: mass?,
                rotor_position: rotor_position?,
                propulsion: propulsion?,
                drag_body: drag_body?,
                battery_voltage: battery_voltage?,
                imu1: imu1?,
                imu2: imu2?,
                sha256: sha256_hex(text.as_bytes()),
                open,
            })
        })()
        .ok_or_else(|| vec!["internal error: a card field was not read".to_string()])
    }

    /// One line per placeholder record, `v = "ask"` or `range = "ask"`, across all tables.
    pub fn open_items(&self) -> Vec<String> {
        self.open.clone()
    }
}

// ---------------------------------------------------------------------------
// Env
// ---------------------------------------------------------------------------

impl Env {
    pub fn load(path: impl AsRef<Path>) -> Result<Env, Vec<String>> {
        let path = path.as_ref();
        let bytes = std::fs::read(path)
            .map_err(|e| vec![format!("{}: cannot read: {e}", path.display())])?;
        let text = String::from_utf8(bytes)
            .map_err(|_| vec![format!("{}: not valid UTF-8", path.display())])?;
        Env::parse(&text)
    }

    /// Checks 1 and 2 on the text of an environment file.
    pub fn parse(text: &str) -> Result<Env, Vec<String>> {
        let root = parse_toml(text)?;
        let mut errs = Vec::new();
        let mut open = Vec::new();
        walk(&root, &ENV_EXEMPT, &mut errs, &mut open);

        let name = str_at(&root, "env", "name", &mut errs);
        let schema = int(&root, "env", "schema", &mut errs);
        let home_override = str_at(&root, "env", "home_override", &mut errs);
        let gravity = rec(&root, "gravity", "g", &mut errs);
        let sea_level_pressure = rec(&root, "atmosphere", "sea_level_pressure", &mut errs);
        let sea_level_temperature = rec(&root, "atmosphere", "sea_level_temperature", &mut errs);
        let lapse_rate = rec(&root, "atmosphere", "lapse_rate", &mut errs);
        let origin_latitude = rec(&root, "origin", "latitude", &mut errs);
        let origin_longitude = rec(&root, "origin", "longitude", &mut errs);
        let origin_altitude = rec(&root, "origin", "altitude", &mut errs);
        let magnetic_field_ned = rec(&root, "magnetic_field", "ned", &mut errs);
        let wind_mean_ned = rec(&root, "wind", "mean_ned", &mut errs);
        let wind_gust_sd = rec(&root, "wind", "gust_sd", &mut errs);

        let errs = dedupe(errs);
        if !errs.is_empty() {
            return Err(errs);
        }
        (|| {
            Some(Env {
                name: name?,
                schema: schema?,
                home_override: home_override?,
                gravity: gravity?,
                sea_level_pressure: sea_level_pressure?,
                sea_level_temperature: sea_level_temperature?,
                lapse_rate: lapse_rate?,
                origin_latitude: origin_latitude?,
                origin_longitude: origin_longitude?,
                origin_altitude: origin_altitude?,
                magnetic_field_ned: magnetic_field_ned?,
                wind_mean_ned: wind_mean_ned?,
                wind_gust_sd: wind_gust_sd?,
                open,
            })
        })()
        .ok_or_else(|| vec!["internal error: an env field was not read".to_string()])
    }

    pub fn open_items(&self) -> Vec<String> {
        self.open.clone()
    }
}

// ---------------------------------------------------------------------------
// Check 6: home override ignored (run by card-check only)
// ---------------------------------------------------------------------------

/// `home_override` is a path relative to the root of the git repo that contains `env_path`.
/// It must be git-ignored there.
pub fn check_home_override(env_path: &Path, home_override: &str) -> Result<(), String> {
    let dir = match env_path.parent() {
        Some(d) if !d.as_os_str().is_empty() => d,
        _ => Path::new("."),
    };
    let top = Command::new("git")
        .arg("-C")
        .arg(dir)
        .args(["rev-parse", "--show-toplevel"])
        .output()
        .map_err(|e| format!("env.home_override: cannot run git: {e}"))?;
    if !top.status.success() {
        return Err(format!(
            "env.home_override: {} is not inside a git work tree, so \"{home_override}\" cannot be shown to be ignored",
            env_path.display()
        ));
    }
    let root = String::from_utf8_lossy(&top.stdout).trim().to_string();
    let out = Command::new("git")
        .arg("-C")
        .arg(&root)
        .args(["check-ignore", "-q", "--", home_override])
        .output()
        .map_err(|e| format!("env.home_override: cannot run git: {e}"))?;
    match out.status.code() {
        Some(0) => Ok(()),
        Some(1) => Err(format!(
            "env.home_override: \"{home_override}\" is not git-ignored in {root}; add it to .gitignore"
        )),
        _ => Err(format!(
            "env.home_override: git check-ignore failed: {}",
            String::from_utf8_lossy(&out.stderr).trim()
        )),
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Obviously fake values.
    const CARD: &str = r#"
[card]
name = "FAKE"
schema = 1

[motors]
src = "fixture"
front_right = { dshot_line = 0, spin = "ccw" }
rear_right  = { dshot_line = 1, spin = "cw" }
front_left  = { dshot_line = 2, spin = "cw" }
rear_left   = { dshot_line = 3, spin = "ccw" }

# >>> BEGIN GENERATED by fixture
[cad_export]
file = "fixture.json"
sha256 = "00"
cad_commit = "fake"
configuration = "fake"
measured_share_item_pct = 50.0
region_sha256 = "@HASH@"
[mass]
mass = { v = 1.0, unit = "kg", tag = "cad", src = "fake" }
com_in_cad = { v = [0.0, 0.0, 0.0], unit = "m", tag = "cad", src = "fake" }
inertia = { v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], unit = "kg.m^2", tag = "cad", src = "fake" }
[rotor_position]
front_right = { v = [1.0, 1.0, 0.0], unit = "m", tag = "cad", src = "fake" }
rear_right = { v = [-1.0, 1.0, 0.0], unit = "m", tag = "cad", src = "fake" }
front_left = { v = [1.0, -1.0, 0.0], unit = "m", tag = "cad", src = "fake" }
rear_left = { v = [-1.0, -1.0, 0.0], unit = "m", tag = "cad", src = "fake" }
[rotor_spin_cad]
front_right = "ccw"
rear_right = "cw"
front_left = "cw"
rear_left = "ccw"
# <<< END GENERATED

[propulsion]
thrust_coefficient = { v = 1.0, unit = "N/(rad/s)^2", tag = "derived", range_pct = [-10.0, 10.0], src = "fake" }
moment_coefficient = { v = 1.0, unit = "m", tag = "placeholder", range = "ask", src = "fake" }
max_rotor_speed = { v = 1.0, unit = "rad/s", tag = "derived", src = "fake" }
time_constant_up = { v = 1.0, unit = "s", tag = "placeholder", range = [0.5, 2.0], src = "fake" }
time_constant_down = { v = 1.0, unit = "s", tag = "derived", src = "fake" }
level_min = { v = 1, unit = "level", tag = "datasheet", src = "fake" }
level_max = { v = 2, unit = "level", tag = "datasheet", src = "fake" }
idle_level = { v = "ask", unit = "level", tag = "placeholder", range = "ask", src = "fake" }
speed_at_level_min = { v = 0.0, unit = "rad/s", tag = "placeholder", range = "ask", src = "fake" }

[drag]
body = { v = [0.0, 0.0, 0.0], unit = "N/(m/s)^2", tag = "placeholder", range = "ask", src = "fake" }

[battery]
voltage = { v = 1.0, unit = "V", tag = "datasheet", src = "fake" }

[imu1]
position = { v = [0.0, 0.0, 0.0], unit = "m", tag = "derived", src = "fake" }
gyro_noise_sd = { v = [1.0, 1.0, 1.0], unit = "rad/s", tag = "derived", src = "fake" }
accel_noise_sd = { v = [1.0, 1.0, 1.0], unit = "m/s^2", tag = "derived", src = "fake" }
gyro_bias = { v = [0.0, 0.0, 0.0], unit = "rad/s", tag = "derived", src = "fake" }
accel_bias = { v = [0.0, 0.0, 0.0], unit = "m/s^2", tag = "derived", src = "fake" }
latency = { v = 0.0, unit = "s", tag = "derived", src = "fake" }

[imu2]
position = { v = [0.0, 0.0, 0.0], unit = "m", tag = "derived", src = "fake" }
gyro_noise_sd = { v = [1.0, 1.0, 1.0], unit = "rad/s", tag = "derived", src = "fake" }
accel_noise_sd = { v = [1.0, 1.0, 1.0], unit = "m/s^2", tag = "derived", src = "fake" }
gyro_bias = { v = [0.0, 0.0, 0.0], unit = "rad/s", tag = "derived", src = "fake" }
accel_bias = { v = [0.0, 0.0, 0.0], unit = "m/s^2", tag = "derived", src = "fake" }
latency = { v = 0.0, unit = "s", tag = "derived", src = "fake" }
"#;

    const ENV: &str = r#"
[env]
name = "fake"
schema = 1
home_override = "env/home.local.toml"

[gravity]
g = { v = 1.0, unit = "m/s^2", tag = "datasheet", src = "fake" }

[atmosphere]
sea_level_pressure = { v = 1.0, unit = "Pa", tag = "datasheet", src = "fake" }
sea_level_temperature = { v = 1.0, unit = "K", tag = "datasheet", src = "fake" }
lapse_rate = { v = 1.0, unit = "K/m", tag = "datasheet", src = "fake" }

[origin]
latitude = { v = "ask", unit = "deg", tag = "placeholder", range = "ask", src = "fake" }
longitude = { v = "ask", unit = "deg", tag = "placeholder", range = "ask", src = "fake" }
altitude = { v = "ask", unit = "m", tag = "placeholder", range = "ask", src = "fake" }

[magnetic_field]
ned = { v = [1.0, 0.0, 0.0], unit = "uT", tag = "derived", src = "fake" }

[wind]
mean_ned = { v = [0.0, 0.0, 0.0], unit = "m/s", tag = "placeholder", range = "ask", src = "fake" }
gust_sd = { v = 0.0, unit = "m/s", tag = "placeholder", range = "ask", src = "fake" }
"#;

    /// Fill in the region hash of an (edited) fixture.
    fn hashed(raw: &str) -> String {
        raw.replace("@HASH@", &region_hash(raw).unwrap())
    }

    fn edited(from: &str, to: &str) -> String {
        assert!(CARD.contains(from), "fixture lacks {from}");
        hashed(&CARD.replace(from, to))
    }

    fn errors(text: &str) -> Vec<String> {
        Card::parse(text).expect_err("expected errors")
    }

    fn has(errs: &[String], needle: &str) -> bool {
        errs.iter().any(|e| e.contains(needle))
    }

    #[test]
    fn fixture_passes() {
        let card = Card::parse(&hashed(CARD)).unwrap();
        assert_eq!(card.open_items().len(), 5);
        let env = Env::parse(ENV).unwrap();
        assert_eq!(env.open_items().len(), 5);
    }

    #[test]
    fn check1_missing_tag() {
        let errs = errors(&edited(
            "level_min = { v = 1, unit = \"level\", tag = \"datasheet\", src = \"fake\" }",
            "level_min = { v = 1, unit = \"level\", src = \"fake\" }",
        ));
        assert!(has(&errs, "propulsion.level_min: missing tag"), "{errs:?}");
        let errs = errors(&edited(
            "position = { v = [0.0, 0.0, 0.0], unit = \"m\", tag = \"derived\", src = \"fake\" }",
            "position = [0.0, 0.0, 0.0]",
        ));
        assert!(has(&errs, "imu1.position: expected a record"), "{errs:?}");
    }

    #[test]
    fn check2_placeholder_range_and_ask() {
        let errs = errors(&edited(
            "range = [0.5, 2.0], ",
            "",
        ));
        assert!(
            has(&errs, "propulsion.time_constant_up: a placeholder must carry"),
            "{errs:?}"
        );
        let errs = errors(&edited(
            "voltage = { v = 1.0,",
            "voltage = { v = \"ask\",",
        ));
        assert!(has(&errs, "battery.voltage: v = \"ask\" is allowed only"), "{errs:?}");
    }

    #[test]
    fn check3_region_hash() {
        let text = hashed(CARD).replace("mass = { v = 1.0,", "mass = { v = 1.5,");
        let errs = errors(&text);
        assert!(has(&errs, "cad_export.region_sha256: the generated region changed"), "{errs:?}");
    }

    #[test]
    fn check4_frame() {
        let errs = errors(&edited(
            "front_right = { v = [1.0, 1.0, 0.0]",
            "front_right = { v = [-1.0, 1.0, 0.0]",
        ));
        assert!(has(&errs, "rotor_position.front_right: x must be > 0"), "{errs:?}");
    }

    #[test]
    fn check5_motors() {
        let errs = errors(&edited(
            "rear_right  = { dshot_line = 1, spin = \"cw\" }",
            "rear_right  = { dshot_line = 0, spin = \"cw\" }",
        ));
        assert!(has(&errs, "motors: dshot_line values must be 0, 1, 2, 3 once each"), "{errs:?}");
        let errs = errors(&edited("rear_right = \"cw\"", "rear_right = \"ccw\""));
        assert!(has(&errs, "rotor_spin_cad.rear_right"), "{errs:?}");
    }
}
