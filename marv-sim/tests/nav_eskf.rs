//! Eskf, the shadow filter (decision-nav-eskf-20261120, Amendment C, C7 tests c1, c2, c4 and B4 b2, b3; Amendment C-1:
//! GNSS only while armed, the heading and north-east flags only while armed), on the simulator's sensor models. c1, c2
//! and c4 drive the card's IMU1 model and the S1 barometer, compass and GNSS models from a synthetic truth (still and
//! disarmed, armed, then legs with stick-sized tilt and turns); faults edit the sample stream. b2, b3 and the suite
//! part of c1 run the scenario suite through `marv-run --shadow eskf`. Expected values come from the card, the env,
//! `Tuning` and the decision's numbers.

use fw_flight::eskf::Eskf;
use fw_flight::gnc::{Estimator, Sensors};
use fw_flight::tuning::Tuning;
use marv_card::{Card, Env};
use marv_sim::rng::Rng;
use marv_sim::sensors::{IMU_PERIOD_US, ImuModel};
use marv_sim::slow_sensors::{GNSS_DEFAULT_RATE_HZ, Origin, SlowSensors, ned_from_geo};
use marv_sim::{Reaction, Truth};
use std::collections::VecDeque;
use std::f64::consts::PI;
use std::process::Command;

/// C1d / 9g: heading bound, deg (= nav_heading_valid_sigma_rad).
const HEADING_LIMIT_DEG: f64 = 15.0;
/// B3: the test-only sigma honesty check.
const SIGMAS: f64 = 3.0;
/// B4 a2 as ruled 2026-11-21: |nav D - truth D| with the barometer model's drift taken out, m.
const D_LIMIT_M: f64 = 0.3;
const TICK_US: u64 = 125;
/// The synthetic flight: armed at 5 s, "take-off" (the end of a ground anomaly) at 7 s, 70 s long.
const ARM_S: f64 = 5.0;
const TAKEOFF_S: f64 = 7.0;
const SECS: f64 = 70.0;

fn root() -> String {
    format!("{}/..", env!("CARGO_MANIFEST_DIR"))
}

fn load() -> (Card, Env) {
    let card = Card::load(format!("{}/marv-card/marv.toml", root())).expect("card");
    let env = Env::load(format!("{}/env/default.toml", root())).expect("env");
    (card, env)
}

/// The tuning with the declination of the sim's field (C4: tests set it from the env).
fn tuning(env: &Env) -> Tuning {
    let f = env.magnetic_field_ned.v.expect("field");
    Tuning { nav_mag_declination_rad: f[1].atan2(f[0]) as f32, ..Tuning::DEFAULT }
}

#[derive(Clone, Copy, PartialEq)]
enum MagEdit {
    None,
    /// The compass turned about body z, rad (strength and inclination unchanged: it passes the health test).
    Turn(f64),
    /// Log 186's take-off spot until TAKEOFF_S (A4, C7 c2): 0.84 of the strength, inclination 75 deg, heading 25 deg
    /// off; `health_off` widens the sample's tolerances so that the test passes everything (the negative control).
    Anomaly { health_off: bool },
}

#[derive(Debug, Default)]
struct Score {
    hdg_valid_s: f64,
    hdg_bad_s: f64,
    hdg_worst_deg: f64,
    ne_valid_s: f64,
    ne_out3: u64,
    ne_steps: u64,
    ne_pmax: f64,
    aligned_before_takeoff: bool,
    gps_fused_before_takeoff: u32,
    aligned: bool,
    tilt_rms_deg: f64,
}

/// Smooth bump: peak * sin^2 over [t0, t0 + len].
fn bump(t: f64, t0: f64, len: f64, peak: f64) -> f64 {
    if t < t0 || t > t0 + len { 0.0 } else { peak * (PI * (t - t0) / len).sin().powi(2) }
}

/// NED acceleration and yaw rate of the synthetic flight at `t` (s): a 1 m climb, then legs north, east, south-west
/// with 2 m/s^2 bumps (tilt about 11 deg) and 90 deg turns between them.
fn profile(t: f64) -> ([f64; 3], f64) {
    let mut a = [0.0; 3];
    a[2] += bump(t, 8.0, 1.0, -2.0) + bump(t, 9.0, 1.0, 2.0);
    let legs: [(f64, [f64; 2]); 3] = [(12.0, [1.0, 0.0]), (30.0, [0.0, 1.0]), (48.0, [-0.707, -0.707])];
    for (t0, dir) in legs {
        for (dt, sign) in [(0.0, 1.0), (8.0, -1.0)] {
            for i in 0..2 {
                a[i] += bump(t, t0 + dt, 2.0, 2.0 * sign * dir[i]);
            }
        }
    }
    let yaw_rate = bump(t, 23.0, 4.0, PI / 4.0) + bump(t, 41.0, 4.0, PI / 4.0) + bump(t, 59.0, 4.0, PI / 4.0);
    (a, yaw_rate)
}

/// Body to NED from the thrust direction (body -z along the specific force) and the yaw.
fn attitude(a: [f64; 3], yaw: f64, g: f64) -> [f64; 4] {
    let f = [a[0], a[1], a[2] - g];
    let n = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt();
    let z = [-f[0] / n, -f[1] / n, -f[2] / n];
    let xc = [yaw.cos(), yaw.sin(), 0.0];
    let cross = |u: [f64; 3], v: [f64; 3]| [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    let y = cross(z, xc);
    let yn = (y[0] * y[0] + y[1] * y[1] + y[2] * y[2]).sqrt();
    let y = y.map(|c| c / yn);
    let x = cross(y, z);
    let r = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]];
    let w = (1.0 + r[0][0] + r[1][1] + r[2][2]).max(0.0).sqrt() / 2.0;
    let q = [w, (r[2][1] - r[1][2]) / (4.0 * w), (r[0][2] - r[2][0]) / (4.0 * w), (r[1][0] - r[0][1]) / (4.0 * w)];
    let qn = (q.iter().map(|c| c * c).sum::<f64>()).sqrt();
    q.map(|c| c / qn)
}

fn rot(q: [f64; 4]) -> [[f64; 3]; 3] {
    let [w, x, y, z] = q;
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ]
}

fn yaw_of(q: [f64; 4]) -> f64 {
    let r = rot(q);
    r[1][0].atan2(r[0][0])
}

fn wrap(a: f64) -> f64 {
    (a + PI).rem_euclid(2.0 * PI) - PI
}

/// Flies the synthetic flight with the card's sensor models (seed 1), the given tuning and compass edit.
fn fly(card: &Card, env: &Env, t: &Tuning, edit: MagEdit) -> Score {
    let mut imu = ImuModel::new("imu1", &card.imu1, env, Rng::new(1, 1)).expect("imu1");
    let mut slow = SlowSensors::new(card, env, GNSS_DEFAULT_RATE_HZ, 1).expect("slow sensors");
    let origin: Origin = slow.gnss.origin();
    let field = env.magnetic_field_ned.v.expect("field");
    let g = env.gravity.v.expect("g");
    let mut e = Eskf::new(t).expect("eskf");
    let (mut p, mut v, mut yaw) = ([0.0f64; 3], [0.0f64; 3], 0.5f64);
    let mut q_prev = attitude([0.0; 3], yaw, g);
    let mut hist: VecDeque<(u64, [f64; 4], [f64; 3], [f64; 3])> = VecDeque::new();
    let mut s = Score::default();
    let (mut tilt2, mut tilt_n) = (0.0, 0u64);
    let dt = TICK_US as f64 * 1e-6;
    for k in 1..=(SECS * 1e6) as u64 / TICK_US {
        let t_us = k * TICK_US;
        let ts = t_us as f64 * 1e-6;
        let (a, yr) = profile(ts);
        for i in 0..3 {
            p[i] += v[i] * dt + 0.5 * a[i] * dt * dt;
            v[i] += a[i] * dt;
        }
        yaw += yr * dt;
        let q = attitude(a, yaw, g);
        // Body rates from the attitude change over the tick: 2 vec(q_prev^* q) / dt.
        let [w0, x0, y0, z0] = q_prev;
        let [w1, x1, y1, z1] = q;
        let rel = [
            w0 * x1 - x0 * w1 - y0 * z1 + z0 * y1,
            w0 * y1 + x0 * z1 - y0 * w1 - z0 * x1,
            w0 * z1 - x0 * y1 + y0 * x1 - z0 * w1,
        ];
        q_prev = q;
        let truth = Truth {
            t_us,
            position_m: p,
            velocity_ms: v,
            attitude: q,
            rates_rads: rel.map(|c| 2.0 * c / dt),
            accel_ms2: a,
            ang_accel_rads2: [0.0; 3],
            on_ground: false,
            settling: false,
            reaction: Reaction::default(),
        };
        slow.tick(&truth);
        if !t_us.is_multiple_of(IMU_PERIOD_US) {
            continue;
        }
        let (baro, mut mag, gnss) = slow.take();
        if let Some(m) = mag.as_mut() {
            match edit {
                MagEdit::None => {}
                MagEdit::Turn(ang) => {
                    let [x, y, z] = m.reading.field_ut.map(f64::from);
                    let (c, sn) = (ang.cos(), ang.sin());
                    m.reading.field_ut = [c * x - sn * y, sn * x + c * y, z].map(|c| c as f32);
                }
                MagEdit::Anomaly { health_off } => {
                    if ts < TAKEOFF_S {
                        let strength = (field[0] * field[0] + field[1] * field[1] + field[2] * field[2]).sqrt() * 0.84;
                        let dec = field[1].atan2(field[0]) + 25f64.to_radians();
                        let inc = 75f64.to_radians();
                        let mn = [inc.cos() * dec.cos(), inc.cos() * dec.sin(), inc.sin()].map(|c| c * strength);
                        let r = rot(q);
                        let mb: [f64; 3] = std::array::from_fn(|j| (0..3).map(|i| r[i][j] * mn[i]).sum());
                        m.reading.field_ut = mb.map(|c| c as f32);
                    }
                    if health_off {
                        if let Some(x) = m.expected.as_mut() {
                            x.tol_strength_frac = 10.0;
                            x.tol_inclination_rad = PI as f32;
                        }
                    }
                }
            }
        }
        let sensors = Sensors { imu1: imu.sample(&truth), imu2: None, baro, mag, gnss, external: None };
        hist.push_back((t_us, q, v, p));
        while hist.len() > 1_000 {
            hist.pop_front();
        }
        e.set_armed(ts >= ARM_S);
        let nav = e.update(&sensors);
        if ts < TAKEOFF_S {
            s.aligned_before_takeoff |= e.yaw_aligned();
            s.gps_fused_before_takeoff = e.stats().gps_pos_fused + e.stats().gps_resets;
        }
        let Some(&(_, tq, tv, tp)) = hist.iter().rev().find(|h| h.0 == nav.t_us) else { continue };
        let qe = nav.attitude.map(f64::from);
        let step_s = IMU_PERIOD_US as f64 * 1e-6;
        if nav.valid.tilt {
            let (re, rt) = (rot(qe), rot(tq));
            let c = (0..3).map(|i| re[i][2] * rt[i][2]).sum::<f64>().clamp(-1.0, 1.0);
            tilt2 += c.acos().powi(2);
            tilt_n += 1;
        }
        if nav.valid.heading {
            let err = wrap(yaw_of(qe) - yaw_of(tq)).abs().to_degrees();
            s.hdg_valid_s += step_s;
            s.hdg_bad_s += if err > HEADING_LIMIT_DEG { step_s } else { 0.0 };
            s.hdg_worst_deg = s.hdg_worst_deg.max(err);
        }
        if nav.valid.position_ne {
            let o = e.origin().expect("origin with position_ne");
            let off = ned_from_geo(&origin, o.lat_e7, o.lon_e7, o.alt_mm);
            let sig = e.sigmas();
            let pe = [f64::from(nav.position_m[0]) + off[0] - tp[0], f64::from(nav.position_m[1]) + off[1] - tp[1]];
            let ve = [f64::from(nav.velocity_ms[0]) - tv[0], f64::from(nav.velocity_ms[1]) - tv[1]];
            let out = (0..2).any(|i| pe[i].abs() > SIGMAS * f64::from(sig[6 + i]) || ve[i].abs() > SIGMAS * f64::from(sig[3 + i]));
            s.ne_valid_s += step_s;
            s.ne_steps += 1;
            s.ne_out3 += u64::from(out);
            s.ne_pmax = s.ne_pmax.max(pe[0].hypot(pe[1]));
        }
    }
    s.aligned = e.yaw_aligned();
    s.tilt_rms_deg = (tilt2 / tilt_n.max(1) as f64).sqrt().to_degrees();
    println!("  stats {:?}", e.stats());
    s
}

#[test]
fn c1_honest_heading_and_its_negative_control() {
    let (card, env) = load();
    let t = tuning(&env);
    let ok = fly(&card, &env, &t, MagEdit::None);
    let turned = fly(&card, &env, &t, MagEdit::Turn(30f64.to_radians()));
    println!("c1: {ok:?}\nc1 negative control (compass turned 30 deg): {turned:?}");
    assert!(ok.hdg_valid_s > SECS - ARM_S - 2.0, "heading valid only {} s", ok.hdg_valid_s);
    assert_eq!(ok.hdg_bad_s, 0.0, "heading valid while more than {HEADING_LIMIT_DEG} deg wrong");
    assert!(turned.hdg_bad_s > 0.0, "negative control: a compass turned 30 deg must make c1 fail");
}

#[test]
fn c2_a_disturbed_take_off_spot_gives_no_alignment_and_no_gps_and_its_negative_control() {
    let (card, env) = load();
    let t = tuning(&env);
    let a = fly(&card, &env, &t, MagEdit::Anomaly { health_off: false });
    let off = fly(&card, &env, &t, MagEdit::Anomaly { health_off: true });
    println!("c2: {a:?}\nc2 negative control (health test off): {off:?}");
    assert!(!a.aligned_before_takeoff && a.gps_fused_before_takeoff == 0);
    assert!(a.aligned && a.hdg_bad_s == 0.0);
    assert!(off.aligned_before_takeoff && off.hdg_bad_s > 0.0, "negative control: heading valid while wrong");
}

#[test]
fn c4_gps_fused_at_its_own_time_stays_within_3_sigma_and_delay_0_fails() {
    let (card, env) = load();
    let t = tuning(&env);
    let lat = card.gnss.latency.v.expect("gnss.latency");
    assert_eq!((lat * 1e6).round() as u64, t.nav_gps_delay_us, "the card's latency is the approved delay");
    let ok = fly(&card, &env, &t, MagEdit::None);
    let zero = fly(&card, &env, &Tuning { nav_gps_delay_us: 0, ..t }, MagEdit::None);
    println!("c4: {ok:?}\nc4 negative control (nav_gps_delay_us 0): {zero:?}");
    assert!(ok.ne_valid_s > 30.0);
    assert!(zero.ne_out3 > 0, "negative control: delay 0 must fail the 3-sigma bound");
    assert_eq!(
        ok.ne_out3, 0,
        "SPECIFICATION (C7 c4 against sections 1a and C-1 finding N): {} of {} north-east valid steps outside {SIGMAS} sigma; \
         the GNSS model's slow error (Gauss-Markov 0.3 m, 60 s, approved) is not a filter state, and the position sigma \
         sits near the 0.3 m floor, so position against truth leaves 3 sigma (largest {:.2} m); delay 0 is worse",
        ok.ne_out3, ok.ne_steps, ok.ne_pmax
    );
}

/// The values after `shadow eskf` on one line, by key.
fn field(line: &str, key: &str) -> String {
    let kv: Vec<&str> = line.split("shadow eskf").nth(1).expect("shadow line").split_whitespace().collect();
    let i = kv.iter().position(|k| *k == key).unwrap_or_else(|| panic!("{key} in {line}"));
    kv[i + 1].to_string()
}

/// b2 (the a2 height check, as ruled, on the shadow: `d_max_nodrift` <= D_LIMIT_M), c1 over the suite (no heading
/// valid while more than 15 deg wrong), I2 (no step with more than one heavy job); b3, the tilt against truth beside
/// Mahony's, is printed only (B4: no pass mark).
#[test]
fn b2_c1_b3_over_the_scenario_suite() {
    let out = env!("CARGO_TARGET_TMPDIR");
    let common = ["--card", "marv-card/marv.toml", "--env", "env/default.toml", "--shadow", "eskf"];
    let mut jobs: Vec<Vec<String>> = ["hover", "steps", "stick-sign", "rc-loss", "takeoff-land"]
        .iter()
        .map(|s| {
            ["--scenario", &format!("scenarios/{s}.toml"), "--seed", "1", "--out", &format!("{out}/b2-{s}.BIN")]
                .map(String::from)
                .to_vec()
        })
        .collect();
    jobs.push(["--spread", "scenarios/spread.toml", "--seed", "1", "--out-dir", &format!("{out}/b2-spread")].map(String::from).to_vec());
    jobs.push(["--spread", "scenarios/takeoff-land-matrix.toml", "--out-dir", &format!("{out}/b2-tl")].map(String::from).to_vec());
    let (mut runs, mut failing) = (0, Vec::new());
    let (mut worst_d, mut worst_tilt, mut worst_mtilt) = (0.0f64, 0.0f64, 0.0f64);
    for job in jobs {
        let o = Command::new(env!("CARGO_BIN_EXE_marv-run")).current_dir(root()).args(common).args(&job).output().expect("marv-run");
        let text = String::from_utf8_lossy(&o.stdout);
        for line in text.lines().filter(|l| l.contains("shadow eskf")) {
            runs += 1;
            let num = |k: &str| field(line, k).parse::<f64>().unwrap_or(f64::NAN);
            let (d, bad, multi) = (num("d_max_nodrift"), num("hdg_bad15_s"), field(line, "multi_job"));
            worst_d = worst_d.max(d);
            worst_tilt = worst_tilt.max(num("tilt_rms_deg"));
            worst_mtilt = worst_mtilt.max(num("mahony_tilt_rms_deg"));
            if !(d <= D_LIMIT_M) || bad > 0.0 || multi != "no" {
                failing.push(format!("{}: d_max_nodrift {d} hdg_bad15_s {bad} multi_job {multi}", line.split_whitespace().next().unwrap()));
            }
        }
    }
    println!("b2/c1: {runs} runs, {} failing; worst d_max_nodrift {worst_d:.4} m; b3 (reported only): worst tilt rms Eskf \
{worst_tilt:.3} deg, Mahony {worst_mtilt:.3} deg", failing.len());
    for f in &failing {
        println!("  {f}");
    }
    assert_eq!(runs, 5 + 20 + 270);
    assert!(failing.is_empty(), "{} of {runs} runs fail", failing.len());
}
