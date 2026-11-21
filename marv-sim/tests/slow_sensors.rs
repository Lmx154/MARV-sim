//! The barometer, compass and GNSS models against their definitions (decision-nav-eskf-20261120, section 9a-c), on the
//! real card and the default (public) environment, long seeded runs.

use marv_card::{Card, Env};
use marv_sim::math::{mat_inv, mat_t_vec, rot_matrix};
use marv_sim::rng::Rng;
use marv_sim::slow_sensors::*;
use marv_sim::{Reaction, Truth};

fn card() -> Card {
    Card::load(concat!(env!("CARGO_MANIFEST_DIR"), "/../marv-card/marv.toml")).unwrap()
}

fn env() -> Env {
    Env::load(concat!(env!("CARGO_MANIFEST_DIR"), "/../env/default.toml")).unwrap()
}

fn still(down: f64, on_ground: bool) -> Snapshot {
    Snapshot { position_m: [0.0, 0.0, down], velocity_ms: [0.0; 3], attitude: [1.0, 0.0, 0.0, 0.0], on_ground }
}

fn truth(t_us: u64, position_m: [f64; 3], velocity_ms: [f64; 3]) -> Truth {
    Truth {
        t_us,
        position_m,
        velocity_ms,
        attitude: [1.0, 0.0, 0.0, 0.0],
        rates_rads: [0.0; 3],
        accel_ms2: [0.0; 3],
        ang_accel_rads2: [0.0; 3],
        on_ground: false,
        settling: false,
        reaction: Reaction::default(),
    }
}

fn mean_sd(x: &[f64]) -> (f64, f64) {
    let m = x.iter().sum::<f64>() / x.len() as f64;
    (m, (x.iter().map(|v| (v - m).powi(2)).sum::<f64>() / (x.len() - 1) as f64).sqrt())
}

/// Lag-`k` autocorrelation.
fn autocorr(x: &[f64], k: usize) -> f64 {
    let (m, sd) = mean_sd(x);
    let n = x.len() - k;
    (0..n).map(|i| (x[i] - m) * (x[i + k] - m)).sum::<f64>() / (n as f64 * sd * sd)
}

#[test]
fn baro_white_noise_bias_and_isa() {
    let (card, env) = (card(), env());
    let mut b = BaroModel::new(&card, &env, Rng::new(7, STREAM_BARO)).unwrap();
    let alt0 = env.origin_altitude.v.unwrap();
    // On the ground (no airborne term): pressure - ISA(origin altitude - down - drift) is the white noise.
    let n = 20_000;
    let mut white = Vec::new();
    for k in 0..n {
        let s = b.sample(k * BARO_PERIOD_US, &still(-2.0, true));
        let (isa, _) = b.isa(alt0 + 2.0 + b.drift_m());
        white.push(f64::from(s.reading.pressure_pa) - isa);
    }
    let (m, sd) = mean_sd(&white);
    let want = card.baro.pressure_noise_sd.v.unwrap();
    // f32 pressure near 1e5 Pa has a 0.0078 Pa step: sd within 5 %, mean within 4 standard errors (bias 0).
    assert!((sd / want - 1.0).abs() < 0.05, "white sd {sd} want {want}");
    assert!(m.abs() < 4.0 * want / (n as f64).sqrt() + 0.004, "mean {m}");
    // ISA at sea level and 1000 m (ISO 2533 table: 89874.6 Pa).
    assert!((b.isa(0.0).0 - 101_325.0).abs() < 1e-6);
    assert!((b.isa(1000.0).0 - 89_874.6).abs() < 0.5, "{}", b.isa(1000.0).0);
}

#[test]
fn baro_airborne_noise_and_drift_size() {
    let (card, env) = (card(), env());
    let air_sd = card.baro.airborne_height_noise_sd.v.unwrap();
    let mut b = BaroModel::new(&card, &env, Rng::new(8, STREAM_BARO)).unwrap();
    let alt0 = env.origin_altitude.v.unwrap();
    // Airborne: the height error (pressure back through ISA near the origin) minus the drift has sd
    // sqrt(air^2 + (white / (rho g))^2).
    let mut err = Vec::new();
    for k in 0..20_000u64 {
        let s = b.sample(k * BARO_PERIOD_US, &still(-5.0, false));
        let p = f64::from(s.reading.pressure_pa);
        let (p_true, t) = b.isa(alt0 + 5.0);
        let rho_g = p_true / (287.05287 * t) * env.gravity.v.unwrap();
        err.push(-(p - p_true) / rho_g - b.drift_m());
    }
    let (_, sd) = mean_sd(&err);
    let white_m = card.baro.pressure_noise_sd.v.unwrap() / 11.9;
    let want = (air_sd * air_sd + white_m * white_m).sqrt();
    assert!((sd / want - 1.0).abs() < 0.05, "airborne sd {sd} want {want}");
    // Drift: across 400 seeds, its sd after drift_time (840 s = 42 000 samples) is drift_sd within 10 %.
    let (drift_sd, drift_time) = (card.baro.drift_sd.v.unwrap(), card.baro.drift_time.v.unwrap());
    let steps = (drift_time * 1e6 / BARO_PERIOD_US as f64) as u64;
    let ends: Vec<f64> = (0..400)
        .map(|seed| {
            let mut b = BaroModel::new(&card, &env, Rng::new(seed, STREAM_BARO)).unwrap();
            for k in 0..steps {
                b.sample(k * BARO_PERIOD_US, &still(0.0, true));
            }
            b.drift_m()
        })
        .collect();
    let rms = (ends.iter().map(|x| x * x).sum::<f64>() / ends.len() as f64).sqrt();
    assert!((rms / drift_sd - 1.0).abs() < 0.10, "drift rms {rms} want {drift_sd}");
}

#[test]
fn compass_perfect_calibration_gives_the_body_field() {
    let (mut card, env) = (card(), env());
    let soft = [[1.10, 0.05, -0.02], [0.03, 0.92, 0.04], [-0.01, 0.02, 0.81]];
    let hard = [12.0, -7.5, 30.0];
    card.compass.soft_iron.v = Some(soft);
    card.compass.hard_iron_bias.v = Some(hard);
    let perfect = Calibration { hard_iron_ut: hard, correction: mat_inv(&soft) };
    let mut m = MagModel::new(&card, &env, perfect, Rng::new(1, STREAM_MAG)).unwrap().without_noise();
    let field = env.magnetic_field_ned.v.unwrap();
    // 30 deg yaw, 10 deg pitch (Z-Y-X), as a quaternion.
    let (y, p) = (30f64.to_radians() / 2.0, 10f64.to_radians() / 2.0);
    let q = [y.cos() * p.cos(), -y.sin() * p.sin(), y.cos() * p.sin(), y.sin() * p.cos()];
    let snap = Snapshot { position_m: [0.0; 3], velocity_ms: [0.0; 3], attitude: q, on_ground: false };
    let s = m.sample(0, &snap);
    let body = mat_t_vec(&rot_matrix(q), field);
    for i in 0..3 {
        assert!((f64::from(s.reading.field_ut[i]) - body[i]).abs() < 1e-4, "{:?} vs {body:?}", s.reading.field_ut);
    }
    // No calibration: the distortion shows (sensor = soft x body + hard).
    let mut raw = MagModel::new(&card, &env, Calibration::NONE, Rng::new(1, STREAM_MAG)).unwrap().without_noise();
    let r = raw.sample(0, &snap);
    assert!((f64::from(r.reading.field_ut[2]) - (marv_sim::math::mat_vec(&soft, body)[2] + hard[2])).abs() < 1e-4);
}

#[test]
fn compass_noise_matches_the_card() {
    let (card, env) = (card(), env());
    let mut m = MagModel::new(&card, &env, Calibration::NONE, Rng::new(2, STREAM_MAG)).unwrap();
    let field = env.magnetic_field_ned.v.unwrap();
    let samples: Vec<[f32; 3]> = (0..20_000).map(|k| m.sample(k * MAG_PERIOD_US, &still(0.0, true)).reading.field_ut).collect();
    let want = card.compass.field_noise_sd.v.unwrap();
    for i in 0..3 {
        let x: Vec<f64> = samples.iter().map(|s| f64::from(s[i]) - field[i]).collect();
        let (mean, sd) = mean_sd(&x);
        assert!((sd / want[i] - 1.0).abs() < 0.05, "axis {i} sd {sd} want {}", want[i]);
        assert!(mean.abs() < 4.0 * want[i] / 141.0, "axis {i} mean {mean}");
    }
}

#[test]
fn gnss_slow_error_sigma_and_time_constant() {
    let (card, env) = (card(), env());
    let g = &card.gnss;
    let (h, v, tau) = (g.slow_horizontal_sd.v.unwrap(), g.slow_vertical_sd.v.unwrap(), g.slow_time_constant.v.unwrap());
    // 1 Hz, 200 000 s (about 3300 time constants).
    let mut m = GnssModel::new(&card, &env, 1, Rng::new(3, STREAM_GNSS)).unwrap();
    let (mut n, mut d) = (Vec::new(), Vec::new());
    for k in 0..200_000u64 {
        m.sample(k * 1_000_000, &still(0.0, false));
        n.push(m.slow_m()[0]);
        d.push(m.slow_m()[2]);
    }
    let (_, sn) = mean_sd(&n);
    let (_, sd) = mean_sd(&d);
    assert!((sn / h - 1.0).abs() < 0.08, "north slow sd {sn} want {h}");
    assert!((sd / v - 1.0).abs() < 0.08, "down slow sd {sd} want {v}");
    // Autocorrelation at lag tau is 1/e.
    let r = autocorr(&n, tau as usize);
    assert!((r - (-1f64).exp()).abs() < 0.06, "lag-tau autocorrelation {r}");
}

#[test]
fn gnss_white_noise_velocity_course_hold_and_sigmas() {
    let (card, env) = (card(), env());
    let g = &card.gnss;
    let mut m = GnssModel::new(&card, &env, 5, Rng::new(4, STREAM_GNSS)).unwrap();
    let o = m.origin();
    // Fast-varying part of north: position - slow error is white at horizontal_sd.
    let mut white = Vec::new();
    let mut speed = Vec::new();
    for k in 0..20_000u64 {
        let snap = Snapshot { position_m: [0.0; 3], velocity_ms: [0.0, 2.0, 0.0], attitude: [1.0, 0.0, 0.0, 0.0], on_ground: false };
        let s = m.sample(k * 200_000, &snap);
        let r = s.reading;
        let ned = ned_from_geo(&o, r.lat_e7.unwrap(), r.lon_e7.unwrap(), r.alt_mm.unwrap());
        white.push(ned[0] - m.slow_m()[0]);
        speed.push(f64::from(r.speed_mm_s.unwrap()) * 1e-3);
        assert_eq!(r.fix_type, 3);
        assert_eq!((r.lat_std_mm, r.lon_std_mm, r.alt_std_mm), (Some(180), Some(180), Some(410)));
    }
    let (_, sd) = mean_sd(&white);
    let want = g.horizontal_sd.v.unwrap();
    assert!((sd / want - 1.0).abs() < 0.05, "white north sd {sd} want {want}");
    // At 2 m/s east the speed noise is about the along-track velocity noise.
    let (ms, ss) = mean_sd(&speed);
    assert!((ms - 2.0).abs() < 0.01 && (ss / g.velocity_sd.v.unwrap() - 1.0).abs() < 0.05, "speed {ms} sd {ss}");

    // Course: east at 2 m/s reads about 90 deg; once at rest it is held (noise alone stays below 0.3 m/s mostly,
    // so the held value repeats exactly while speed < 0.3).
    let mut m = GnssModel::new(&card, &env, 5, Rng::new(5, STREAM_GNSS)).unwrap();
    let rest = still(0.0, false);
    let first = m.sample(0, &rest).reading;
    if f64::from(first.speed_mm_s.unwrap()) < 300.0 {
        assert_eq!(first.course_cdeg, None, "no course before the first motion");
    }
    let moving = Snapshot { velocity_ms: [0.0, 2.0, 0.0], ..rest };
    let c = m.sample(200_000, &moving).reading.course_cdeg.unwrap();
    assert!((8000..=10000).contains(&c), "course {c}");
    let mut held = c;
    let mut checked = 0;
    for k in 2..2000u64 {
        let r = m.sample(k * 200_000, &rest).reading;
        if r.speed_mm_s.unwrap() < 300 {
            assert_eq!(r.course_cdeg, Some(held));
            checked += 1;
        } else {
            held = r.course_cdeg.unwrap();
        }
    }
    assert!(checked > 1000);
}

#[test]
fn gnss_position_round_trip_and_wrap() {
    let env = env();
    let o = Origin::from_env(&env).unwrap();
    for ned in [[0.0, 0.0, 0.0], [123.456, -78.9, -12.345], [-2500.0, 4000.0, 3.0], [0.004, -0.004, 0.0]] {
        let (lat, lon, alt) = geo_from_ned(&o, ned);
        let back = ned_from_geo(&o, lat, lon, alt);
        // Half a 1e-7 degree step: 5.6 mm north; east less; 0.5 mm in altitude.
        assert!((back[0] - ned[0]).abs() < 0.006 && (back[1] - ned[1]).abs() < 0.006 && (back[2] - ned[2]).abs() < 0.0006,
            "{ned:?} -> {back:?}");
    }
    // 1 km north, by hand: WGS 84 meridian radius at 51.47783 deg = 6 374 560 m, so 1000 m = 8.98821e-3 deg = 89 882 units.
    let (lat, _, _) = geo_from_ned(&o, [1000.0, 0.0, 0.0]);
    assert!((lat - o.lat_e7 - 89_882).abs() <= 2, "{}", lat - o.lat_e7);
    // Across the 180 deg meridian: a synthetic origin just west of it.
    let w = Origin { lat_e7: 100_000_000, lon_e7: 1_799_999_000, alt_mm: 0 };
    let (_, lon, _) = geo_from_ned(&w, [0.0, 50.0, 0.0]);
    assert!(lon < -1_799_990_000, "wrapped longitude {lon}");
    let back = ned_from_geo(&w, 100_000_000, lon, 0);
    assert!((back[1] - 50.0).abs() < 0.006, "{back:?}");
}

#[test]
fn rates_latency_and_fresh_once() {
    let (card, env) = (card(), env());
    for rate in GNSS_RATES_HZ {
        let mut s = SlowSensors::new(&card, &env, rate, 1).unwrap();
        // 3 s of ticks; the truth moves north at 1 m/s from t = 1 s and climbs 1 m/s (down falls) from t = 1 s.
        let mut seen = (Vec::new(), Vec::new(), Vec::new());
        let mut t = 0;
        while t < 3_000_000 {
            t += TICK_US;
            let x = if t > 1_000_000 { (t - 1_000_000) as f64 * 1e-6 } else { 0.0 };
            s.tick(&truth(t, [x, 0.0, -x], [if t > 1_000_000 { 1.0 } else { 0.0 }, 0.0, 0.0]));
            // A flight step every 1250 us takes what is there.
            if t.is_multiple_of(1250) {
                let (b, m, g) = s.take();
                assert_eq!(s.take(), (None, None, None), "a second take gives nothing");
                if let Some(b) = b { seen.0.push((t, b)); }
                if let Some(m) = m { seen.1.push((t, m)); }
                if let Some(g) = g { seen.2.push((t, g)); }
            }
        }
        // Every sample reaches a step exactly once, in order, at its own time, at the device's rate.
        for (n, period, stamps) in [
            (seen.0.len(), BARO_PERIOD_US, seen.0.iter().map(|(t, b)| (*t, b.t_us, b.seq)).collect::<Vec<_>>()),
            (seen.1.len(), MAG_PERIOD_US, seen.1.iter().map(|(t, m)| (*t, m.t_us, m.seq)).collect()),
            (seen.2.len(), 1_000_000 / u64::from(rate), seen.2.iter().map(|(t, g)| (*t, g.t_us, g.seq)).collect()),
        ] {
            assert_eq!(n as u64, 3_000_000 / period);
            for (i, (step_t, t_us, seq)) in stamps.into_iter().enumerate() {
                assert_eq!((seq, t_us, step_t), (i as u32, (i as u64 + 1) * period, (i as u64 + 1) * period));
            }
        }
        // GNSS latency: the first sample whose truth has moved is stamped after 1 s + 0.43 s (the slow error drifts by
        // millimetres; 0.05 m of motion is unmistakable against the 0.18 m white noise only on average, so use the
        // speed, which is exact truth plus 0.15 m/s noise: it jumps from about 0 to about 1 m/s).
        let first_fast = seen.2.iter().find(|(_, g)| g.reading.speed_mm_s.unwrap() > 600).unwrap().1.t_us;
        let lat_us = (card.gnss.latency.v.unwrap() * 1e6) as u64;
        let period = 1_000_000 / u64::from(rate);
        assert!(first_fast > 1_000_000 + lat_us - period && first_fast <= 1_000_000 + lat_us + period, "rate {rate}: {first_fast}");
    }
}

#[test]
fn latency_in_ticks_is_the_cards() {
    let (card, env) = (card(), env());
    let s = SlowSensors::new(&card, &env, 5, 1).unwrap();
    // 11.5 ms, 5 ms and 0.43 s at 125 us.
    assert_eq!((s.baro.latency_ticks, s.mag.latency_ticks, s.gnss.latency_ticks), (92, 40, 3440));
    // The truth delay returns exactly that many ticks ago.
    let mut d = TruthDelay::new(3440);
    for k in 0..5000 {
        d.push(still(k as f64, false));
    }
    assert_eq!(d.ago(3440).position_m[2], (4999 - 3440) as f64);
    assert_eq!(d.ago(92).position_m[2], (4999 - 92) as f64);
}
