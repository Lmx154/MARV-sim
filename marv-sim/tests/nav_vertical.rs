//! MahonyBaro, the phase-1 steering estimator (decision-nav-eskf-20261120, Amendment B, B4 tests a1-a4), on the
//! simulator's sensor models. a2 runs the scenario suite through `marv-run --shadow mahony-baro`; a1, a3 and a4 drive
//! the IMU and barometer models from a synthetic truth (level; still or climbing). Expected values come from the card,
//! `Tuning` and B3's thresholds.

use fw_flight::gnc::{Estimator, NavState, Sensors};
use fw_flight::mahony::Mahony;
use fw_flight::mahony_baro::MahonyBaro;
use fw_flight::tuning::Tuning;
use marv_card::{Card, Env};
use marv_sim::rng::Rng;
use marv_sim::sensors::{IMU_PERIOD_US, ImuModel};
use marv_sim::slow_sensors::{GNSS_DEFAULT_RATE_HZ, SlowSensors};
use marv_sim::{Reaction, Truth};
use std::process::Command;

/// B3: |nav D - truth D| limit, m.
const D_LIMIT_M: f64 = 0.3;
/// B3: the test-only sigma honesty check.
const SIGMAS: f64 = 3.0;
/// B4 a2: judged from 5 s after the initialisation, us.
const SETTLE_US: u64 = 5_000_000;
const TICK_US: u64 = 125;

fn root() -> String {
    format!("{}/..", env!("CARGO_MANIFEST_DIR"))
}

fn load() -> (Card, Env) {
    let card = Card::load(format!("{}/marv-card/marv.toml", root())).expect("card");
    let env = Env::load(format!("{}/env/default.toml", root())).expect("env");
    (card, env)
}

/// What one synthetic flight gives.
struct Flown {
    d_max: f64,
    v_3sigma: u64,
    judged: u64,
    bias_d: f32,
    bias_d_sigma: f32,
    /// Steps where MahonyBaro's attitude, rates or tilt/rates flags differ from Mahony's, bit for bit.
    identity_breaks: u64,
    /// Steps where a Mahony given one gyro value one ulp up (IMU sample 1000) differs from MahonyBaro.
    perturbed_differs: u64,
}

/// Level, in the air, for `secs`: still for the first 5 s (VertKf initialises still), then a constant acceleration up
/// to `climb_ms` over the next 5 s, then climbing at `climb_ms` (0: still throughout). The card's IMU1 and barometer
/// models, seed 1.
fn fly(card: &Card, env: &Env, tuning: &Tuning, secs: f64, climb_ms: f64) -> Flown {
    let mut imu = ImuModel::new("imu1", &card.imu1, env, Rng::new(1, 1)).expect("imu1");
    let mut slow = SlowSensors::new(card, env, GNSS_DEFAULT_RATE_HZ, 1).expect("slow sensors");
    let latency_s = card.baro.latency.v.expect("baro.latency");
    let (mut mb, mut m, mut mp) = (MahonyBaro::new(tuning), Mahony::new(tuning), Mahony::new(tuning));
    let mut f = Flown { d_max: 0.0, v_3sigma: 0, judged: 0, bias_d: 0.0, bias_d_sigma: 0.0, identity_breaks: 0,
        perturbed_differs: 0 };
    let same = |a: &NavState, b: &NavState| {
        a.attitude.map(f32::to_bits) == b.attitude.map(f32::to_bits)
            && a.rates_rads.map(f32::to_bits) == b.rates_rads.map(f32::to_bits)
            && (a.valid.tilt, a.valid.rates) == (b.valid.tilt, b.valid.rates)
    };
    // Truth down at the datum sample's measurement (its read time minus the latency), and the initialisation time.
    let mut init: Option<(u64, f64)> = None;
    let mut samples = 0u64;
    for k in 1..=(secs * 1e6) as u64 / TICK_US {
        let t_us = k * TICK_US;
        let t = t_us as f64 * 1e-6;
        let ramp_s = SETTLE_US as f64 * 1e-6;
        let a = climb_ms / ramp_s;
        let (up, v, acc) = match t - ramp_s {
            r if r < 0.0 => (0.0, 0.0, 0.0),
            r if r < ramp_s => (0.5 * a * r * r, a * r, a),
            r => (0.5 * a * ramp_s * ramp_s + climb_ms * (r - ramp_s), climb_ms, 0.0),
        };
        let truth = Truth {
            t_us,
            position_m: [0.0, 0.0, -up],
            velocity_ms: [0.0, 0.0, -v],
            attitude: [1.0, 0.0, 0.0, 0.0],
            rates_rads: [0.0; 3],
            accel_ms2: [0.0, 0.0, -acc],
            ang_accel_rads2: [0.0; 3],
            on_ground: false,
            settling: false,
            reaction: Reaction::default(),
        };
        slow.tick(&truth);
        if !t_us.is_multiple_of(IMU_PERIOD_US) {
            continue;
        }
        let (baro, _, _) = slow.take();
        let s = Sensors { imu1: imu.sample(&truth), imu2: None, baro, mag: None, gnss: None, external: None };
        let nav = mb.update(&s);
        f.identity_breaks += u64::from(!same(&m.update(&s), &nav));
        let mut sp = s;
        if samples == 1000 {
            sp.imu1.reading.gyro_rads[0] = f32::from_bits(sp.imu1.reading.gyro_rads[0].to_bits() + 1);
        }
        f.perturbed_differs += u64::from(!same(&mp.update(&sp), &nav));
        samples += 1;
        if init.is_none() && mb.vert().initialised() {
            let b = baro.expect("initialised on a barometer sample");
            assert!(b.t_us as f64 * 1e-6 - latency_s < ramp_s, "initialised after the climb began");
            init = Some((t_us, 0.0));
        }
        let Some((t0, d0)) = init else { continue };
        if t_us < t0 + SETTLE_US {
            continue;
        }
        let ed = f64::from(nav.position_m[2]) - (truth.position_m[2] - d0);
        let ev = f64::from(nav.velocity_ms[2]) - truth.velocity_ms[2];
        let sigma_v = f64::from(nav.sigma.expect("sigmas").velocity_d_ms);
        f.d_max = f.d_max.max(ed.abs());
        f.v_3sigma += u64::from(ev.abs() > SIGMAS * sigma_v);
        f.judged += 1;
    }
    f.bias_d = mb.vert().bias_d();
    f.bias_d_sigma = mb.vert().bias_d_sigma();
    f
}

/// The card with the barometer's drift random walk off: a1, a3 and a4 isolate one property each; the drift's share
/// of the error is what a2 reports on the full model.
fn card_without_drift() -> (Card, Env) {
    let (mut card, env) = load();
    card.baro.drift_sd.v = Some(0.0);
    (card, env)
}

#[test]
fn a1_sim_attitude_rates_and_flags_are_mahonys_bit_for_bit() {
    let (card, env) = load();
    let f = fly(&card, &env, &Tuning::DEFAULT, 20.0, 1.0);
    println!("a1 (sim): identity breaks {}, negative control differs on {} steps", f.identity_breaks, f.perturbed_differs);
    assert_eq!(f.identity_breaks, 0);
    assert!(f.perturbed_differs > 0, "negative control: a perturbed input must change Mahony's output");
}

#[test]
fn a3_a_constant_accelerometer_bias_is_estimated_and_a2_holds() {
    let (mut card, env) = card_without_drift();
    // IMU1's accelerometer bias at the card's range bound on body z (marv.toml [imu1] accel_bias range +/-0.1961
    // m/s^2: zero-g tolerance +/-20 mg, DS ICM-45686.txt:817-819). Level, body z is earth down.
    let b = 0.1961;
    card.imu1.accel_bias.v = Some([0.0, 0.0, b]);
    let f = fly(&card, &env, &Tuning::DEFAULT, 60.0, 0.0);
    println!("a3: bD {} +/- {} m/s^2 (true {b}); D error max {:.3} m; vD > 3 sigma on {} of {}", f.bias_d,
        f.bias_d_sigma, f.d_max, f.v_3sigma, f.judged);
    assert!((f64::from(f.bias_d) - b).abs() <= SIGMAS * f64::from(f.bias_d_sigma), "bD did not converge");
    assert!(f.bias_d_sigma < Tuning::DEFAULT.nav_p0_ba_ms2);
    assert!(f.d_max <= D_LIMIT_M && f.v_3sigma == 0);
    // Negative control: bD frozen (ba_rw = 0, P0 = 0): a2 fails.
    let frozen = Tuning { nav_ba_rw: 0.0, nav_p0_ba_ms2: 0.0, ..Tuning::DEFAULT };
    let g = fly(&card, &env, &frozen, 60.0, 0.0);
    println!("a3 negative control (bD frozen): bD {}; D error max {:.3} m; vD > 3 sigma on {} of {}", g.bias_d, g.d_max,
        g.v_3sigma, g.judged);
    assert_eq!(g.bias_d, 0.0);
    assert!(g.d_max > D_LIMIT_M || g.v_3sigma > 0, "negative control: a frozen bias must fail a2");
}

#[test]
fn a4_barometer_latency_200_ms_is_compensated_by_the_delay_term() {
    let (mut card, env) = card_without_drift();
    // B4 a4: the barometer's latency 200 ms; climbing at twice the speed at which that lag alone reaches the limit.
    let latency_s = 0.2;
    card.baro.latency.v = Some(latency_s);
    let climb = 2.0 * D_LIMIT_M / latency_s;
    let delay_us = (latency_s * 1e6) as u64;
    let with = fly(&card, &env, &Tuning { nav_baro_delay_us: delay_us, ..Tuning::DEFAULT }, 40.0, climb);
    let without = fly(&card, &env, &Tuning { nav_baro_delay_us: 0, ..Tuning::DEFAULT }, 40.0, climb);
    println!("a4: climb {climb} m/s; D error max {:.3} m with nav_baro_delay_us {delay_us} (vD > 3 sigma on {}), \
{:.3} m with 0", with.d_max, with.v_3sigma, without.d_max);
    assert!(with.d_max <= D_LIMIT_M && with.v_3sigma == 0);
    assert!(without.d_max > D_LIMIT_M, "negative control: no delay term must fail the limit");
}

/// The values after `shadow mahony-baro` on one line, by key.
fn field(line: &str, key: &str) -> f64 {
    let kv: Vec<&str> = line.split("shadow mahony-baro").nth(1).expect("shadow line").split_whitespace().collect();
    let i = kv.iter().position(|k| *k == key).unwrap_or_else(|| panic!("{key} in {line}"));
    kv[i + 1].parse().expect("number")
}

/// B4 a2, judged by the owner's ruling of 2026-11-21 (decision-nav-eskf-20261120, "Owner's ruling on how test a2 of
/// Amendment B is judged"). Height: `d_max_nodrift` (the error with the barometer model's own drift taken out) against
/// D_LIMIT_M from 5 s after the initialisation, because the barometer-only filter cannot see its own sensor's drift; the
/// raw `d_max` is printed, not judged. Vertical speed: `v_3sigma_pre_td` (3-sigma steps from 5 s after the
/// initialisation until the first touchdown) must be 0, because the plant stops the vehicle in one tick at touchdown
/// without the accelerometer seeing it; the raw `v_3sigma` is printed, not judged. Runs that never touch down are
/// judged over the whole window.
#[test]
fn a2_vertical_error_in_every_scenario_across_the_spread() {
    let out = env!("CARGO_TARGET_TMPDIR");
    let common = ["--card", "marv-card/marv.toml", "--env", "env/default.toml", "--shadow", "mahony-baro"];
    let mut jobs: Vec<Vec<String>> = ["hover", "steps", "stick-sign", "rc-loss", "takeoff-land"]
        .iter()
        .map(|s| {
            ["--scenario", &format!("scenarios/{s}.toml"), "--seed", "1", "--out", &format!("{out}/a2-{s}.BIN")]
                .map(String::from)
                .to_vec()
        })
        .collect();
    jobs.push(["--spread", "scenarios/spread.toml", "--seed", "1", "--out-dir", &format!("{out}/a2-spread")]
        .map(String::from).to_vec());
    jobs.push(["--spread", "scenarios/takeoff-land-matrix.toml", "--out-dir", &format!("{out}/a2-tl")]
        .map(String::from).to_vec());
    let mut failing = Vec::new();
    let mut runs = 0;
    let (mut worst_d, mut worst_raw, mut raw_v3, mut judged_v3) = (0.0f64, 0.0f64, 0.0, 0.0);
    for job in jobs {
        let o = Command::new(env!("CARGO_BIN_EXE_marv-run")).current_dir(root()).args(common).args(&job).output()
            .expect("marv-run");
        let text = String::from_utf8_lossy(&o.stdout);
        // alt-hold is a steering scenario with its own verdict (Amendment F, F9; lead 2026-11-21): not judged by a2.
        for line in text.lines().filter(|l| l.contains("shadow mahony-baro") && !l.starts_with("alt-hold-")) {
            runs += 1;
            let name = if line.starts_with("shadow") { job[1].clone() } else { line.split_whitespace().next().unwrap().to_string() };
            let (d_raw, d_nd, v3_raw, v3, v3_n, breaks) = (field(line, "d_max"), field(line, "d_max_nodrift"),
                field(line, "v_3sigma"), field(line, "v_3sigma_pre_td"), field(line, "v_3sigma_pre_td_judged"),
                field(line, "identity_breaks"));
            worst_d = worst_d.max(d_nd);
            worst_raw = worst_raw.max(d_raw);
            raw_v3 += v3_raw;
            judged_v3 += v3;
            if d_nd > D_LIMIT_M || v3 > 0.0 || breaks > 0.0 {
                failing.push(format!("{name}: judged d_max_nodrift {d_nd} (raw d_max {d_raw}) judged v_3sigma_pre_td {v3} of \
{v3_n} steps (raw v_3sigma {v3_raw}) identity_breaks {breaks}"));
            }
        }
    }
    println!("a2: {runs} runs, {} failing; judged: worst d_max_nodrift {worst_d:.4} m (limit {D_LIMIT_M}), v_3sigma_pre_td total \
{judged_v3}; raw: worst d_max {worst_raw:.4} m, v_3sigma total {raw_v3}", failing.len());
    for f in &failing {
        println!("  {f}");
    }
    assert_eq!(runs, 5 + 20 + 270);
    assert!(failing.is_empty(), "a2 fails in {} of {runs} runs", failing.len());
}
