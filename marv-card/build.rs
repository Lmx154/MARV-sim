//! Turns marv.toml into `const` data for firmware code (`$OUT_DIR/card_consts.rs`).
//! It loads the card with the same `src/schema.rs` the std loader uses, so the same checks run.

#[allow(dead_code)]
#[path = "src/schema.rs"]
mod schema;

use schema::{Card, Record, Spin, Tag};
use std::fmt::Write;

fn main() {
    println!("cargo:rerun-if-changed=marv.toml");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/schema.rs");
    if let Err(msg) = run() {
        eprintln!("{msg}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let dir = std::env::var("CARGO_MANIFEST_DIR").map_err(|e| e.to_string())?;
    let card = Card::load(format!("{dir}/marv.toml")).map_err(|e| e.join("\n"))?;

    let mut n = 0u32;

    let m = &card.motors;
    let p = &card.rotor_position;
    // (motor, position record, key) for each rotor, then ordered by DShot line.
    let rotors = [
        (m.front_right, &p.front_right, "rotor_position.front_right"),
        (m.rear_right, &p.rear_right, "rotor_position.rear_right"),
        (m.front_left, &p.front_left, "rotor_position.front_left"),
        (m.rear_left, &p.rear_left, "rotor_position.rear_left"),
    ];
    let mut by_line = [([0.0f64; 3], 0.0f64); 4];
    let mut seen = [false; 4];
    for (motor, rec, key) in rotors {
        let line = motor.dshot_line as usize;
        if line > 3 || seen[line] {
            return Err(format!("marv.toml: [motors] dshot_line {line} is out of range or used twice"));
        }
        seen[line] = true;
        let pos = get(&mut n, key, rec)?;
        let spin = if motor.spin == Spin::Ccw { 1.0 } else { -1.0 };
        by_line[line] = (pos, spin);
    }

    let pr = &card.propulsion;
    let k_f = get(&mut n, "propulsion.thrust_coefficient", &pr.thrust_coefficient)?;
    let k_m = get(&mut n, "propulsion.moment_coefficient", &pr.moment_coefficient)?;
    let max_speed = get(&mut n, "propulsion.max_rotor_speed", &pr.max_rotor_speed)?;
    let speed_min = get(&mut n, "propulsion.speed_at_level_min", &pr.speed_at_level_min)?;
    let level_min = get(&mut n, "propulsion.level_min", &pr.level_min)?;
    let level_max = get(&mut n, "propulsion.level_max", &pr.level_max)?;
    let idle = get(&mut n, "propulsion.idle_level", &pr.idle_level)?;
    let mass = get(&mut n, "mass.mass", &card.mass.mass)?;

    for (key, v) in [("level_min", level_min), ("level_max", level_max), ("idle_level", idle)] {
        if v.fract() != 0.0 || !(0.0..=2000.0).contains(&v) {
            return Err(format!("marv.toml: propulsion.{key} = {v} is not a whole number in 0..=2000"));
        }
    }
    if !(level_min <= idle && idle <= level_max) {
        return Err("marv.toml: propulsion needs level_min <= idle_level <= level_max".to_string());
    }

    // M maps rotor thrusts to [thrust, torque_x, torque_y, torque_z]; MIX is its inverse.
    let mut mat = [[0.0f64; 4]; 4];
    for (i, (pos, spin)) in by_line.iter().enumerate() {
        mat[0][i] = 1.0;
        mat[1][i] = -pos[1];
        mat[2][i] = pos[0];
        mat[3][i] = spin * k_m;
    }
    let mix = invert(mat)?;
    for (r, row) in mat.iter().enumerate() {
        for c in 0..4 {
            let sum: f64 = row.iter().zip(&mix).map(|(m, mix_row)| m * mix_row[c]).sum();
            let want = if r == c { 1.0 } else { 0.0 };
            if (sum - want).abs() >= 1e-9 {
                return Err(format!("marv.toml: M * MIX - I is {:e} at [{r}][{c}], the rotor layout is singular", sum - want));
            }
        }
    }

    let mut out = String::new();
    out.push_str("// Generated from marv.toml by marv-card/build.rs. Do not edit.\n\n");
    out.push_str("/// One rotor: its thrust point, FRD from the centre of mass, m; spin +1.0 for CCW seen from above, -1.0 for CW.\n");
    out.push_str("pub struct Rotor { pub position_m: [f32; 3], pub spin: f32 }\n\n");
    out.push_str("pub const ROTORS: [Rotor; 4] = [\n");
    for (pos, spin) in by_line {
        let _ = writeln!(out, "    Rotor {{ position_m: [{:e}, {:e}, {:e}], spin: {:e} }},", pos[0] as f32, pos[1] as f32, pos[2] as f32, spin as f32);
    }
    out.push_str("];\n");
    for (name, v, doc) in [
        ("K_F", k_f, "propulsion.thrust_coefficient, N/(rad/s)^2"),
        ("K_M", k_m, "propulsion.moment_coefficient, m"),
        ("MAX_ROTOR_SPEED", max_speed, "rad/s"),
        ("SPEED_AT_LEVEL_MIN", speed_min, "rad/s"),
        ("MASS", mass, "kg"),
    ] {
        let _ = writeln!(out, "/// {doc}\npub const {name}: f32 = {:e};", v as f32);
    }
    let _ = writeln!(out, "pub const LEVEL_MIN: u16 = {level_min};");
    let _ = writeln!(out, "pub const LEVEL_MAX: u16 = {level_max};");
    let _ = writeln!(out, "pub const IDLE_LEVEL: u16 = {idle};");
    out.push_str("/// Rotor thrusts from a demand: T = MIX * [thrust_n, torque_x, torque_y, torque_z]; rows are DShot lines.\n");
    out.push_str("pub const MIX: [[f32; 4]; 4] = [\n");
    for row in mix {
        let _ = writeln!(out, "    [{:e}, {:e}, {:e}, {:e}],", row[0] as f32, row[1] as f32, row[2] as f32, row[3] as f32);
    }
    out.push_str("];\n");
    let _ = writeln!(out, "/// Hex sha256 of marv.toml's bytes.\npub const CARD_SHA256: &str = \"{}\";", card.sha256);
    let _ = writeln!(out, "/// How many of the records above are tagged placeholder.\npub const PLACEHOLDERS_USED: u32 = {n};");

    let dest = std::env::var("OUT_DIR").map_err(|e| e.to_string())?;
    std::fs::write(format!("{dest}/card_consts.rs"), out).map_err(|e| e.to_string())
}

/// Every number the firmware uses goes through here: "ask" fails, a placeholder is counted and warned.
fn get<T: Copy>(n: &mut u32, key: &str, r: &Record<T>) -> Result<T, String> {
    if r.tag == Tag::Placeholder {
        *n += 1;
        println!("cargo:warning=marv-card: {key} is a placeholder");
    }
    r.v.ok_or(format!("marv.toml: {key} is \"ask\", the firmware needs a number"))
}

/// Gauss-Jordan inverse with partial pivoting.
fn invert(mut a: [[f64; 4]; 4]) -> Result<[[f64; 4]; 4], String> {
    let mut inv = [[0.0f64; 4]; 4];
    for (i, row) in inv.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for col in 0..4 {
        let pivot = (col..4).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs())).unwrap();
        if a[pivot][col].abs() < 1e-12 {
            return Err("marv.toml: the rotor layout gives a singular allocation matrix".to_string());
        }
        a.swap(col, pivot);
        inv.swap(col, pivot);
        let d = a[col][col];
        for k in 0..4 {
            a[col][k] /= d;
            inv[col][k] /= d;
        }
        for r in 0..4 {
            if r != col {
                let f = a[r][col];
                for k in 0..4 {
                    a[r][k] -= f * a[col][k];
                    inv[r][k] -= f * inv[col][k];
                }
            }
        }
    }
    Ok(inv)
}
