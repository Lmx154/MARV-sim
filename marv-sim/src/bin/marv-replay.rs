//! `marv-replay`: replays a flown flight log (`.BIN`) through the flight code's own estimators and checks them against
//! what the board logged. See `--help` and the README.

use fw_flight::eskf::{Aiding, Eskf, EskfStats};
use fw_flight::gnc::{Estimator, NavState, Valid};
use fw_flight::mahony::Mahony;
use fw_flight::mahony_baro::MahonyBaro;
use fw_flight::tuning::Tuning;
use marv_sim::replay::{ExpectedFrom, Log, Stream, StreamOpts, Table, build_stream};
use std::collections::HashMap;
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::PathBuf;

const HELP: &str = "\
marv-replay <log.BIN> <outdir> [--every N] [--imu-period-us US] [--mag-expected UT,INC_DEG,STR_PCT,TOL_DEG] [--noaid-sigma M] [--no-baro-from-armed N] [--steering mahony-baro|eskf2]

Replays a flight log through the flight code's estimators (the real Rust code, Tuning::DEFAULT) and prints a
validation table against the log's own NAV and NKF1 records. One CSV per estimator goes into <outdir>
(a row every N-th step, default 8).

The step stream: one step per logged IMU1 sample. Disarmed, the board logs only every 8th IMU sample, so in each
such gap the last logged sample is REPEATED at the nominal IMU period (the median spacing of consecutive armed
samples, or --imu-period-us) to keep the elapsed time right; the number of real and filled steps is printed.
Baro, compass and GNSS samples (GNSS from GPS + GPA + GPV; GPA gives only sqrt(lat^2+lon^2), split equally into
the lat and lon deviations) go to the first step whose IMU time is >= their own, the newest only. The armed state is
MOT.Arm. The compass's expected field comes from the log's PARM records (MAG_CAL_TYPE, MAG_FLD_UT, MAG_FLD_INC,
MAG_HLT_STR, MAG_HLT_INC); only a log without MAG_CAL_TYPE uses --mag-expected. The source is printed.
";

/// When the estimator is told the armed state: the steering estimator gets the gate's state after the previous step
/// (`Flight::step`), the shadow the state after the same step (`Flight::step_with_shadow`).
#[derive(Clone, Copy, PartialEq)]
enum ArmedAt {
    Before,
    After,
}

/// An estimator the replay can run, with its counters if it has any.
trait Replayed: Estimator {
    fn eskf_stats(&self) -> Option<EskfStats> {
        None
    }
    /// Gyro bias (rad/s) and accelerometer bias (m/s^2).
    fn biases(&self) -> Option<([f32; 3], [f32; 3])> {
        None
    }
}
impl Replayed for Mahony {}
impl Replayed for MahonyBaro {}
impl Replayed for Eskf {
    fn eskf_stats(&self) -> Option<EskfStats> {
        Some(*self.stats())
    }
    fn biases(&self) -> Option<([f32; 3], [f32; 3])> {
        Some(Eskf::biases(self))
    }
}

struct Row {
    name: &'static str,
    armed_at: ArmedAt,
    est: Box<dyn Replayed>,
    nav: Vec<NavState>,
    /// Gyro and accelerometer bias after each step, for the estimators that have them.
    bias: Vec<[f32; 6]>,
}

fn row(name: &'static str, armed_at: ArmedAt, est: Box<dyn Replayed>) -> Row {
    Row { name, armed_at, est, nav: Vec::new(), bias: Vec::new() }
}

/// The estimators to run: one line each.
/// `steer_eskf2`: eskf2 steers (the state after the previous step, as `Flight::step` sets it); else it is a shadow.
fn table(t: &Tuning, steer_eskf2: bool) -> Vec<Row> {
    vec![
        row("Mahony", ArmedAt::Before, Box::new(Mahony::new(t))),
        row("MahonyBaro", ArmedAt::Before, Box::new(MahonyBaro::new(t))),
        row("eskf1", ArmedAt::After, Box::new(Eskf::with_aiding(t, Aiding::Imu).expect("Tuning::DEFAULT is accepted"))),
        row("eskf2", if steer_eskf2 { ArmedAt::Before } else { ArmedAt::After }, Box::new(Eskf::with_aiding(t, Aiding::ImuBaro).expect("Tuning::DEFAULT is accepted"))),
        row("eskf3", ArmedAt::After, Box::new(Eskf::new(t).expect("Tuning::DEFAULT is accepted"))),
    ]
}

struct Args {
    log: PathBuf,
    out: PathBuf,
    every: usize,
    opts: StreamOpts,
    /// `--noaid-sigma`: replaces `Tuning::nav_noaid_sigma_m` (the armed no-aid position hold's sigma) in every estimator.
    noaid: Option<f32>,
    /// `--no-baro-from-armed N`: from the N-th arming on, no barometer sample reaches any estimator.
    no_baro_from_armed: Option<usize>,
    /// `--steering mahony-baro|eskf2`: which estimator steered on the board (default: read from the log).
    steering: Option<bool>,
}

fn parse_args() -> Result<Args, String> {
    let mut pos = Vec::new();
    let mut every = 8usize;
    let mut noaid = None;
    let mut no_baro_from_armed = None;
    let mut steering = None;
    let mut opts = StreamOpts { imu_period_us: None, expected_cli: None };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "-h" | "--help" => {
                print!("{HELP}");
                std::process::exit(0);
            }
            "--noaid-sigma" => {
                noaid = Some(it.next().and_then(|v| v.parse::<f32>().ok()).filter(|v| v.is_finite() && *v > 0.0).ok_or("--noaid-sigma M")?)
            }
            "--no-baro-from-armed" => {
                no_baro_from_armed = Some(it.next().and_then(|v| v.parse::<usize>().ok()).filter(|&v| v > 0).ok_or("--no-baro-from-armed N (N >= 1)")?)
            }
            "--steering" => {
                steering = Some(match it.next().as_deref() {
                    Some("eskf2") => true,
                    Some("mahony-baro") => false,
                    _ => return Err("--steering mahony-baro|eskf2".into()),
                })
            }
            "--every" => every = it.next().and_then(|v| v.parse().ok()).filter(|&v| v > 0).ok_or("--every N")?,
            "--imu-period-us" => {
                opts.imu_period_us = Some(it.next().and_then(|v| v.parse().ok()).ok_or("--imu-period-us US")?)
            }
            "--mag-expected" => {
                let v: Vec<f32> = it
                    .next()
                    .ok_or("--mag-expected UT,INC,STR,TOL")?
                    .split(',')
                    .map(|x| x.parse().map_err(|_| "--mag-expected: four numbers"))
                    .collect::<Result<_, _>>()?;
                opts.expected_cli = Some(v.try_into().map_err(|_| "--mag-expected: four numbers")?);
            }
            s if s.starts_with("--") => return Err(format!("unknown option {s}")),
            _ => pos.push(a),
        }
    }
    let [log, out]: [String; 2] = pos.try_into().map_err(|_| "usage: marv-replay <log.BIN> <outdir> [options]; --help")?;
    Ok(Args { log: log.into(), out: out.into(), every, opts, noaid, no_baro_from_armed, steering })
}

fn bits(v: &Valid) -> [u8; 7] {
    [v.tilt, v.heading, v.rates, v.velocity_ne, v.velocity_d, v.position_ne, v.position_d].map(u8::from)
}

fn write_csv(path: &std::path::Path, stream: &Stream, nav: &[NavState], bias: &[[f32; 6]], every: usize) -> std::io::Result<()> {
    let mut w = std::io::BufWriter::new(std::fs::File::create(path)?);
    writeln!(
        w,
        "t_us,step_t_us,armed,qw,qx,qy,qz,rx,ry,rz,vn,ve,vd,pn,pe,pd,v_tilt,v_heading,v_rates,v_vel_ne,v_vel_d,v_pos_ne,v_pos_d,s_tilt,s_heading,s_vel_ne,s_vel_d,s_pos_ne,s_pos_d,gbx,gby,gbz,abx,aby,abz"
    )?;
    for (k, (s, n)) in stream.steps.iter().zip(nav).enumerate() {
        if k % every != 0 {
            continue;
        }
        let mut l = format!("{},{},{}", n.t_us, s.sensors.imu1.t_us, u8::from(s.armed));
        for v in n.attitude.iter().chain(&n.rates_rads).chain(&n.velocity_ms).chain(&n.position_m) {
            let _ = write!(l, ",{v}");
        }
        for b in bits(&n.valid) {
            let _ = write!(l, ",{b}");
        }
        match n.sigma {
            Some(g) => {
                for v in [g.tilt_rad, g.heading_rad, g.velocity_ne_ms, g.velocity_d_ms, g.position_ne_m, g.position_d_m] {
                    let _ = write!(l, ",{v}");
                }
            }
            None => l.push_str(",,,,,,"),
        }
        match bias.get(k) {
            Some(b) => {
                for v in b {
                    let _ = write!(l, ",{v}");
                }
            }
            None => l.push_str(",,,,,,"),
        }
        writeln!(w, "{l}")?;
    }
    w.flush()
}

/// Angle between two attitudes, degrees.
fn angle_deg(a: [f32; 4], b: [f32; 4]) -> f64 {
    let (a, b) = (a.map(f64::from), b.map(f64::from));
    // conj(a) * b
    let w = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    let x = a[0] * b[1] - a[1] * b[0] - a[2] * b[3] + a[3] * b[2];
    let y = a[0] * b[2] + a[1] * b[3] - a[2] * b[0] - a[3] * b[1];
    let z = a[0] * b[3] - a[1] * b[2] + a[2] * b[1] - a[3] * b[0];
    (2.0 * (x * x + y * y + z * z).sqrt().atan2(w.abs())).to_degrees()
}

/// Angle between the body-frame down directions of two attitudes (their tilt difference), degrees.
fn tilt_deg(a: [f32; 4], b: [f32; 4]) -> f64 {
    let down = |q: [f32; 4]| {
        let [w, x, y, z] = q.map(f64::from);
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)]
    };
    let (u, v) = (down(a), down(b));
    let dot = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    let cross = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt().atan2(dot).to_degrees()
}

/// The attitude difference as [angle, tilt part, rest (yaw)] in degrees.
fn att_diff(a: [f32; 4], b: [f32; 4]) -> [f64; 3] {
    let (ang, tilt) = (angle_deg(a, b), tilt_deg(a, b));
    [ang, tilt, (ang * ang - tilt * tilt).max(0.0).sqrt()]
}

/// Armed intervals as (first armed step time, last armed step time), us.
fn intervals(stream: &Stream) -> Vec<(u64, u64)> {
    let mut v: Vec<(u64, u64)> = Vec::new();
    let mut prev = false;
    for s in &stream.steps {
        let t = s.sensors.imu1.t_us;
        if s.armed {
            if prev {
                v.last_mut().unwrap().1 = t;
            } else {
                v.push((t, t));
            }
        }
        prev = s.armed;
    }
    v
}

/// What the handed GNSS epochs look like to the Eskf's acceptance (5c), in the order of `Eskf::gnss_arrival`'s checks:
/// the epochs the acceptance refuses are the ones its `gps_unaccepted` counts.
fn gnss_diagnostics(stream: &Stream, t: &Tuning) {
    let max_interval_us = 1e6 / t.nav_gps_min_rate_hz;
    let (mut n, mut nofix, mut nostd, mut slow, mut first) = (0, 0, 0, 0, 0);
    let mut prev: Option<u64> = None;
    for g in stream.steps.iter().filter_map(|s| s.sensors.gnss.as_ref()) {
        n += 1;
        let r = &g.reading;
        if r.lat_e7.is_none() || r.lon_e7.is_none() || r.fix_type < 3 {
            nofix += 1;
            continue;
        }
        let before = prev.replace(g.t_us);
        if r.lat_std_mm.is_none() || r.lon_std_mm.is_none() {
            nostd += 1;
        } else if let Some(p) = before {
            if (g.t_us - p) as f32 > max_interval_us {
                slow += 1;
            }
        } else {
            first += 1;
        }
    }
    println!(
        "GNSS epochs handed {n}: no position or fix type < 3: {nofix}; 3D fix without lat/lon deviation (GPA HAcc NaN): {nostd}; interval to the previous 3D fix over {:.2} s (1/nav_gps_min_rate_hz): {slow}; first 3D fix: {first}",
        max_interval_us * 1e-6
    );
}

const SETTLE_US: u64 = 10_000_000;

/// One comparison: the quantities' names and, per logged record, its time and the differences replay - log.
struct Comparison {
    title: String,
    names: &'static [&'static str],
    units: &'static str,
    samples: Vec<(u64, Vec<f64>)>,
}

fn report(c: &Comparison, iv: &[(u64, u64)], settle: bool) -> String {
    let mut out = format!("{}\n", c.title);
    let _ = write!(out, "  {:>3} {:>7} {:>7} {:>7}", "int", "from s", "to s", "window");
    let _ = write!(out, " {:>7}", "n");
    for n in c.names {
        let _ = write!(out, " {:>9} {:>9}", format!("{n} rms"), format!("{n} max"));
    }
    out.push('\n');
    for (k, &(t0, t1)) in iv.iter().enumerate() {
        for (label, from) in [("whole", t0), ("+10s", t0 + SETTLE_US)].into_iter().take(if settle { 2 } else { 1 }) {
            let sel: Vec<&Vec<f64>> =
                c.samples.iter().filter(|(t, _)| *t >= from && *t <= t1).map(|(_, d)| d).collect();
            let _ = write!(
                out,
                "  {:>3} {:>7.1} {:>7.1} {:>7} {:>7}",
                k + 1,
                t0 as f64 * 1e-6,
                t1 as f64 * 1e-6,
                label,
                sel.len()
            );
            for q in 0..c.names.len() {
                if sel.is_empty() {
                    let _ = write!(out, " {:>9} {:>9}", "-", "-");
                    continue;
                }
                let rms = (sel.iter().map(|d| d[q] * d[q]).sum::<f64>() / sel.len() as f64).sqrt();
                let max = sel.iter().map(|d| d[q].abs()).fold(0.0, f64::max);
                let _ = write!(out, " {rms:>9.4} {max:>9.4}");
            }
            out.push('\n');
        }
    }
    let _ = writeln!(out, "  (units: {})", c.units);
    out
}

fn main() {
    if let Err(e) = run() {
        eprintln!("marv-replay: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let bytes = std::fs::read(&args.log).map_err(|e| format!("{}: {e}", args.log.display()))?;
    let log = Log::parse(&bytes)?;
    drop(bytes);
    std::fs::create_dir_all(&args.out).map_err(|e| format!("{}: {e}", args.out.display()))?;
    let mut stream = build_stream(&log, &args.opts)?;
    if let Some(n) = args.no_baro_from_armed {
        let (mut seen, mut prev, mut from) = (0usize, false, None);
        for (k, s) in stream.steps.iter().enumerate() {
            if s.armed && !prev {
                seen += 1;
                if seen == n {
                    from = Some(k);
                    break;
                }
            }
            prev = s.armed;
        }
        let from = from.ok_or_else(|| format!("--no-baro-from-armed {n}: the log has only {seen} armings"))?;
        let mut withheld = 0usize;
        for s in &mut stream.steps[from..] {
            withheld += usize::from(s.sensors.baro.take().is_some());
        }
        println!(
            "barometer withheld from the arming {n} (step time {:.3} s) to the end: {withheld} samples not handed to any estimator",
            stream.steps[from].sensors.imu1.t_us as f64 * 1e-6
        );
    }

    println!("log: {} bytes skipped between records: {}", args.log.file_name().map_or(String::new(), |n| n.to_string_lossy().into_owned()), log.bad_bytes);
    println!(
        "steps: {} real (logged IMU1 samples), {} filled (last logged sample repeated in disarmed gaps, period {} us); total {}",
        stream.real,
        stream.filled,
        stream.imu_period_us,
        stream.steps.len()
    );
    println!(
        "slow samples handed over / in the log: baro {}/{}, compass {}/{}, GNSS {}/{}",
        stream.handed[0], stream.present[0], stream.handed[1], stream.present[1], stream.handed[2], stream.present[2]
    );
    match (&stream.expected, &stream.expected_from) {
        (Some(e), from) => println!(
            "compass expected: from the {}: strength {:.3} uT, inclination {:.3} deg, tolerance {:.1} %, {:.1} deg",
            if matches!(from, ExpectedFrom::Log) { "log's PARM records" } else { "command line" },
            e.strength_ut,
            e.inclination_rad.to_degrees(),
            e.tol_strength_frac * 100.0,
            e.tol_inclination_rad.to_degrees()
        ),
        (None, from) => println!(
            "compass expected: None ({}: MAG_CAL_TYPE is not 2 or MAG_FLD_UT is 0, or nothing given); the Eskf will not use the compass",
            if matches!(from, ExpectedFrom::Log) { "log's PARM records" } else { "command line" }
        ),
    }
    let iv = intervals(&stream);
    println!("armed intervals: {}", iv.len());
    let mut tuning = Tuning::DEFAULT;
    if let Some(s) = args.noaid {
        tuning.nav_noaid_sigma_m = s;
    }
    gnss_diagnostics(&stream, &tuning);

    let (steer, why) = match args.steering {
        Some(v) => (v, "given on the command line".to_string()),
        None => {
            let t = log.table("NKF2")?;
            let (ci, cg) = (t.col("I")?, t.col("GBX")?);
            let (mut n, mut nz) = (0usize, 0usize);
            for k in (0..t.rows()).filter(|&k| t.u8(k, ci) == 0) {
                n += 1;
                nz += usize::from((0..3).any(|c| t.f32(k, cg + c) != 0.0));
            }
            (nz > 0, format!("read from the log: NKF2 I=0 has a non-zero gyro bias in {nz} of {n} records (MahonyBaro logs gyro bias 0)"))
        }
    };
    println!("steering estimator: {} ({why})", if steer { "eskf2" } else { "mahony-baro" });
    let mut rows = table(&tuning, steer);
    for r in &mut rows {
        r.nav.reserve(stream.steps.len());
    }
    let mut prev_armed = false;
    for s in &stream.steps {
        for r in &mut rows {
            r.est.set_armed(if r.armed_at == ArmedAt::Before { prev_armed } else { s.armed });
            r.nav.push(r.est.update(&s.sensors));
            if let Some((g, a)) = r.est.biases() {
                r.bias.push([g[0], g[1], g[2], a[0], a[1], a[2]]);
            }
        }
        prev_armed = s.armed;
    }

    for r in &rows {
        let path = args.out.join(format!("{}.csv", r.name));
        write_csv(&path, &stream, &r.nav, &r.bias, args.every).map_err(|e| format!("{}: {e}", path.display()))?;
        println!("wrote {}", path.display());
    }
    for r in &rows {
        if let Some(s) = r.est.eskf_stats() {
            println!("\n{} final EskfStats:\n{s:#?}", r.name);
        }
    }

    // Validation.
    let find = |n: &str| rows.iter().find(|r| r.name == n);
    let by_time = |nav: &[NavState]| -> HashMap<u64, usize> { nav.iter().enumerate().map(|(k, n)| (n.t_us, k)).collect() };
    println!("\nValidation (replay - log; angles in degrees, per armed interval, whole and from 10 s after arming)\n");

    let nav_t = log.table("NAV")?;
    let nkf = log.table("NKF1")?;
    let (cq, ci) = (nkf.col("Q1")?, nkf.col("I")?);
    let (cvn, cpn) = (nkf.col("VN")?, nkf.col("PN")?);
    let cnq = nav_t.col("Q1")?;
    let quat = |t: &Table, r: usize, c: usize| [t.f32(r, c), t.f32(r, c + 1), t.f32(r, c + 2), t.f32(r, c + 3)];

    for name in if steer { vec![] } else { vec!["Mahony", "MahonyBaro"] } {
        if let Some(r) = find(name) {
            let idx = by_time(&r.nav);
            let samples = (0..nav_t.rows())
                .filter_map(|k| {
                    let t = nav_t.time(k);
                    let n = &r.nav[*idx.get(&t)?];
                    Some((t, att_diff(n.attitude, quat(nav_t, k, cnq)).to_vec()))
                })
                .collect();
            print!(
                "{}",
                report(
                    &Comparison { title: format!("{name} attitude vs logged NAV (same TimeUS)"), names: &["angle", "tilt", "yaw"], units: "deg; tilt: gravity direction in the body frame, yaw: the rest", samples },
                    &iv, true
                )
            );
        }
    }
    if let Some(r) = find("MahonyBaro").filter(|_| !steer) {
        let idx = by_time(&r.nav);
        let samples = (0..nkf.rows())
            .filter(|&k| nkf.u8(k, ci) == 0)
            .filter_map(|k| {
                let t = nkf.time(k);
                let n = &r.nav[*idx.get(&t)?];
                let d = [
                    f64::from(n.position_m[2]) - f64::from(nkf.f32(k, cpn + 2)),
                    f64::from(n.velocity_ms[2]) - f64::from(nkf.f32(k, cvn + 2)),
                ];
                d.iter().all(|x| x.is_finite()).then(|| (t, d.to_vec()))
            })
            .collect();
        print!(
            "{}",
            report(
                &Comparison {
                    title: "MahonyBaro height and vertical speed vs logged NKF1 I=0 (D down, VD)".into(),
                    names: &["PD", "VD"],
                    units: "m, m/s",
                    samples,
                },
                &iv, true
            )
        );
    }
    if let Some(r) = find("eskf3") {
        let idx = by_time(&r.nav);
        let mut unmatched = 0usize;
        let samples = (0..nkf.rows())
            .filter(|&k| nkf.u8(k, ci) == 1)
            .filter_map(|k| {
                let t = nkf.time(k);
                let Some(&j) = idx.get(&t) else {
                    unmatched += usize::from(iv.iter().any(|&(a, b)| t >= a && t <= b));
                    return None;
                };
                let n = &r.nav[j];
                let mut d = att_diff(n.attitude, quat(nkf, k, cq)).to_vec();
                for c in 0..3 {
                    d.push(f64::from(n.position_m[c]) - f64::from(nkf.f32(k, cpn + c)));
                }
                for c in 0..3 {
                    d.push(f64::from(n.velocity_ms[c]) - f64::from(nkf.f32(k, cvn + c)));
                }
                d.iter().all(|x| x.is_finite()).then_some((t, d))
            })
            .collect();
        print!(
            "{}",
            report(
                &Comparison {
                    title: "eskf3 vs logged NKF1 I=1 (TimeUS = horizon time = replayed NavState t_us)".into(),
                    names: &["att", "tilt", "yaw", "PN", "PE", "PD", "VN", "VE", "VD"],
                    units: "deg (att, tilt, yaw as above), m, m/s",
                    samples,
                },
                &iv, true
            )
        );
        println!("  NKF1 I=1 records inside an armed interval with no replayed NavState at their TimeUS: {unmatched}");
    }
    steering_report(&log, &rows, &iv, steer)?;
    log_figures(&log, &iv)?;
    Ok(())
}

/// The steering estimator is the replayed eskf2: its output against the logged NAV, NKF1 I=0 and NKF2 I=0, per armed
/// interval and over the 60 s before each arming (disarmed).
fn steering_report(log: &Log, rows: &[Row], iv: &[(u64, u64)], steer: bool) -> Result<(), String> {
    if !steer {
        return Ok(());
    }
    let r = rows.iter().find(|r| r.name == "eskf2").ok_or("no eskf2 row")?;
    let idx: HashMap<u64, usize> = r.nav.iter().enumerate().map(|(k, n)| (n.t_us, k)).collect();
    let pre: Vec<(u64, u64)> = iv.iter().map(|&(a, _)| (a.saturating_sub(60_000_000), a.saturating_sub(1))).collect();
    let nav_t = log.table("NAV")?;
    let cnq = nav_t.col("Q1")?;
    let att: Vec<(u64, Vec<f64>)> = (0..nav_t.rows())
        .filter_map(|k| {
            let t = nav_t.time(k);
            let n = &r.nav[*idx.get(&t)?];
            let q = [nav_t.f32(k, cnq), nav_t.f32(k, cnq + 1), nav_t.f32(k, cnq + 2), nav_t.f32(k, cnq + 3)];
            Some((t, att_diff(n.attitude, q).to_vec()))
        })
        .collect();
    let nkf1 = log.table("NKF1")?;
    let (ci, cvn, cpn) = (nkf1.col("I")?, nkf1.col("VN")?, nkf1.col("PN")?);
    let hv: Vec<(u64, Vec<f64>)> = (0..nkf1.rows())
        .filter(|&k| nkf1.u8(k, ci) == 0)
        .filter_map(|k| {
            let t = nkf1.time(k);
            let n = &r.nav[*idx.get(&t)?];
            Some((t, vec![
                f64::from(n.position_m[2]) - f64::from(nkf1.f32(k, cpn + 2)),
                f64::from(n.velocity_ms[2]) - f64::from(nkf1.f32(k, cvn + 2)),
            ]))
        })
        .collect();
    let nkf2 = log.table("NKF2")?;
    let (ci2, cg) = (nkf2.col("I")?, nkf2.col("GBX")?);
    let bias: Vec<(u64, Vec<f64>)> = (0..nkf2.rows())
        .filter(|&k| nkf2.u8(k, ci2) == 0)
        .filter_map(|k| {
            let t = nkf2.time(k);
            let b = r.bias.get(*idx.get(&t)?)?;
            Some((t, (0..6).map(|c| {
                let d = f64::from(b[c]) - f64::from(nkf2.f32(k, cg + c));
                if c < 3 { d.to_degrees() } else { d }
            }).collect()))
        })
        .collect();
    println!("\nSteering estimator = replayed eskf2 (armed state: after the previous step). Windows: armed intervals, and the 60 s before each arming (disarmed; the log keeps every 8th disarmed IMU sample, so the replay cannot match there exactly).");
    for (title, names, units, samples) in [
        ("eskf2 attitude vs logged NAV (same TimeUS)", &["angle", "tilt", "yaw"][..], "deg; angle = full attitude, tilt = body-frame down axes, yaw = the rest", att),
        ("eskf2 D and VD vs logged NKF1 I=0", &["PD", "VD"][..], "m, m/s", hv),
        ("eskf2 biases vs logged NKF2 I=0 (gyro bias x y z, accel bias x y z)", &["gbx", "gby", "gbz", "abx", "aby", "abz"][..], "deg/s, m/s^2", bias),
    ] {
        let c = Comparison { title: title.into(), names, units, samples };
        println!("\n[armed intervals]");
        print!("{}", report(&c, iv, true));
        println!("[60 s disarmed before each arming]");
        print!("{}", report(&c, &pre, false));
    }
    Ok(())
}

fn down_of(q: [f32; 4]) -> [f64; 3] {
    let [w, x, y, z] = q.map(f64::from);
    [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)]
}

/// Roll and pitch (deg) of a body-frame down vector.
fn roll_pitch(d: [f64; 3]) -> (f64, f64) {
    (d[1].atan2(d[2]).to_degrees(), (-d[0]).atan2(d[1].hypot(d[2])).to_degrees())
}

fn angle3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let n = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let c = ((a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (n(a) * n(b))).clamp(-1.0, 1.0);
    c.acos().to_degrees()
}

/// Figures straight from the log: the arming edges, the record rates, the steering height against the pressure.
fn log_figures(log: &Log, iv: &[(u64, u64)]) -> Result<(), String> {
    let nav = log.table("NAV")?;
    let cq = nav.col("Q1")?;
    let imu = log.table("IMU")?;
    let (cii, cax) = (imu.col("I")?, imu.col("AccX")?);
    let nkf = log.table("NKI").ok().zip(log.table("NKF1").ok()).filter(|(a, b)| a.rows() > 0 && b.rows() > 0);
    println!("\nArming edges, from the log's own records (no replay). Tilt = roll / pitch in deg of the down axis.");
    println!("  edge s      NAV 0.5 s before   NAV 0.5 s after    accel mean 1 s before   angle NAV-before vs accel   NAV-after vs accel   NAV before->after");
    let us = 1_000_000u64;
    for &(a, _) in iv {
        let mut nb = ([0.0f64; 3], 0usize);
        let mut na = ([0.0f64; 3], 0usize);
        for k in 0..nav.rows() {
            let t = nav.time(k);
            let d = down_of([nav.f32(k, cq), nav.f32(k, cq + 1), nav.f32(k, cq + 2), nav.f32(k, cq + 3)]);
            let tgt = if t + us / 2 >= a && t < a {
                &mut nb
            } else if t >= a && t < a + us / 2 {
                &mut na
            } else {
                continue;
            };
            for c in 0..3 {
                tgt.0[c] += d[c];
            }
            tgt.1 += 1;
        }
        let mut f = ([0.0f64; 3], 0usize);
        for k in 0..imu.rows() {
            let t = imu.time(k);
            if imu.u8(k, cii) == 0 && t + us >= a && t < a {
                for c in 0..3 {
                    f.0[c] -= f64::from(imu.f32(k, cax + c));
                }
                f.1 += 1;
            }
        }
        let fmt = |v: ([f64; 3], usize)| {
            let (r, p) = roll_pitch(v.0);
            format!("{r:+7.2} / {p:+7.2} (n={})", v.1)
        };
        println!(
            "  {:>8.2}  {:<26} {:<26} {:<30} {:>8.2} {:>8.2} {:>8.2}",
            a as f64 * 1e-6,
            fmt(nb),
            fmt(na),
            fmt(f),
            angle3(nb.0, f.0),
            angle3(na.0, f.0),
            angle3(nb.0, na.0)
        );
    }
    // A sim log has no NKF1 or NKI records (the board writes them): the figures that need them are skipped.
    let Some((nki, nkf1)) = nkf else {
        println!("\nno NKF records in this log: the NKI and NKF1 figures are skipped.");
        return Ok(());
    };
    let (cki, csrc, cok) = (nki.col("I")?, nki.col("Src")?, nki.col("Ok")?);
    let (c1i, cpn) = (nkf1.col("I")?, nkf1.col("PN")?);
    println!("\nNKI I=0 records within +-0.2 s of each arming edge (Src code: Ok code = count; Src 4 baro, 5 no-aid, 6 height reset, 7 compass yaw, 8 yaw align/reset, 9 GPS reset; Ok 0 gated, 1 fused, 2 refused, 3 screened)");
    for &(a, _) in iv {
        let mut m: std::collections::BTreeMap<(u8, u8), usize> = Default::default();
        for k in 0..nki.rows() {
            let t = nki.time(k);
            if nki.u8(k, cki) == 0 && t + 200_000 >= a && t <= a + 200_000 {
                *m.entry((nki.u8(k, csrc), nki.u8(k, cok))).or_default() += 1;
            }
        }
        println!("  edge {:.2} s: {}", a as f64 * 1e-6, if m.is_empty() { "none".to_string() } else { m.iter().map(|((s, o), n)| format!("{s}:{o} = {n}")).collect::<Vec<_>>().join(", ") });
    }
    // Rates while armed and while disarmed.
    let t_end = (0..nav.rows()).map(|k| nav.time(k)).max().unwrap_or(0);
    let t_start = (0..nav.rows()).map(|k| nav.time(k)).min().unwrap_or(0);
    let armed_s: f64 = iv.iter().map(|&(a, b)| (b - a) as f64 * 1e-6).sum();
    let dis_s = (t_end - t_start) as f64 * 1e-6 - armed_s;
    println!("\nRecords per second (armed {armed_s:.1} s, disarmed {dis_s:.1} s):");
    let inside = |t: u64| iv.iter().any(|&(a, b)| t >= a && t <= b);
    for (name, t, ic, which) in [("NKF1 I=0", nkf1, c1i, 0u8), ("NKF1 I=1", nkf1, c1i, 1), ("NKI I=0", nki, cki, 0), ("NKI I=1", nki, cki, 1)] {
        let (mut a, mut d) = (0usize, 0usize);
        for k in 0..t.rows() {
            if t.u8(k, ic) == which {
                if inside(t.time(k)) { a += 1 } else { d += 1 }
            }
        }
        println!("  {name}: armed {:.2}/s ({a}), disarmed {:.2}/s ({d})", a as f64 / armed_s, d as f64 / dis_s);
    }
    // Steering D extremes against the barometer.
    let baro = log.table("BARO")?;
    let cp = baro.col("Press")?;
    let (mut hi, mut lo) = ((f32::MIN, 0u64), (f32::MAX, 0u64));
    for k in (0..nkf1.rows()).filter(|&k| nkf1.u8(k, c1i) == 0) {
        let d = nkf1.f32(k, cpn + 2);
        if d.is_finite() {
            if d > hi.0 { hi = (d, nkf1.time(k)) }
            if d < lo.0 { lo = (d, nkf1.time(k)) }
        }
    }
    let press = |t: u64| {
        let k = (0..baro.rows()).min_by_key(|&k| baro.time(k).abs_diff(t)).unwrap_or(0);
        baro.f32(k, cp)
    };
    let (ph, pl) = (press(hi.1), press(lo.1));
    let h = |p: f32, p0: f32| 44330.77 * (1.0 - (f64::from(p) / f64::from(p0)).powf(0.190263));
    println!("\nLogged steering D (NKF1 I=0 PD, m, down positive): largest {:.2} at {:.1} s, smallest {:.2} at {:.1} s, span {:.2} m.", hi.0, hi.1 as f64 * 1e-6, lo.0, lo.1 as f64 * 1e-6, hi.0 - lo.0);
    println!("  Barometer pressure at those times: {ph:.1} Pa (largest D), {pl:.1} Pa (smallest D); the ISA height difference of the two pressures is {:.2} m (D = minus height, so a pressure rise gives a larger D).", h(pl, ph));
    let mut pmin = (f32::MAX, 0u64);
    let mut pmax = (f32::MIN, 0u64);
    for k in 0..baro.rows() {
        let p = baro.f32(k, cp);
        if p < pmin.0 { pmin = (p, baro.time(k)) }
        if p > pmax.0 { pmax = (p, baro.time(k)) }
    }
    println!("  Barometer over the whole log: lowest {:.1} Pa at {:.1} s, highest {:.1} Pa at {:.1} s (ISA height difference {:.2} m).", pmin.0, pmin.1 as f64 * 1e-6, pmax.0, pmax.1 as f64 * 1e-6, h(pmin.0, pmax.0));
    Ok(())
}
