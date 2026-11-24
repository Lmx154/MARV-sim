//! `marv-replay`: replays a flown flight log (`.BIN`) through the flight code's own estimators and checks them against
//! what the board logged. See `--help` and the README.

use fw_flight::eskf::{Aiding, Eskf, EskfStats, Nki, code};
use fw_flight::eskf_out::EskfSteer;
use fw_flight::gnc::{Controller, Estimator, NavState, Saturation, Setpoint, Valid};
use fw_flight::heading_mon::{HeadingMon, SIG_CONF_RAD};
use fw_flight::mahony::{DT_MAX_US, Mahony};
use fw_flight::mahony_baro::MahonyBaro;
use fw_flight::pid::CascadedPid;
use fw_flight::tuning::{GAIN_NAMES, Gains, Tuning};
use marv_sim::replay::{ExpectedFrom, Log, Stream, StreamOpts, Table, build_stream};
use std::collections::HashMap;
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::PathBuf;

const HELP: &str = "\
marv-replay <log.BIN> <outdir> [--every N] [--imu-period-us US] [--mag-expected UT,INC_DEG,STR_PCT,TOL_DEG] [--noaid-sigma M] [--no-baro-from-armed N] [--gps-delay-ms MS] [--gps-min-rate-hz HZ] [--no-mag] [--nki] [--steering mahony-baro|eskf2|eskf3] [--veld] [--holdout-vd-odd] [--vd-sigma M_S] [--notch] [--notch-harmonics 1|3] [--notch-fs HZ] [--heading-mon]

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

--notch: no estimator runs. The log's NAV rates (GX, GY, GZ, the measured rate of the rate loop) and the ESC eRPM (each
ESC record held until the next, an eRPM 100 ms or more older than the step counts as missing; f1 = eRPM x 2 / (14 x 60))
go through the firmware's own CascadedPid (Tuning::DEFAULT, sample rate 803.86 Hz): once with the notch off, once on,
and once on with every eRPM withheld (negative control: its rates must equal the input bit for bit). Writes
<outdir>/notch_h<N>.csv (every NAV step) and prints, for the two hover windows of t292-notch.py, the rms of the D torque
above 50 Hz (Welch, Hann, 2048 samples, 50 % overlap, mean removed, 803.86 Hz) on X and Y. --notch-harmonics 1|3 sets
Tuning::notch_harmonics (default 3, as built). --notch-fs HZ is the sample rate the controller is built for
(default 800.0 = 1e6 / 1250 us, the board's configured IMU1 period); the metric always uses the measured 803.86 Hz.

--heading-mon: the eskf3 row's heading monitor (NKI Src 13) is told the airborne state: Flight's airborne latch (step.rs)
rebuilt from the log, MOT.Arm and each step's DMD (Ok, Thr against Tuning::ground_thrust_g x weight, Sat bit 3 thrust_low,
land_detect_us), the value after the previous step, as Flight::step passes it. Prints per armed session the evaluated,
R2-screened and R2'-dropped pairs, informed % (sig <= 15 deg), max |d_hat| over informed epochs, the time from take-off
to confirmed, any fault, the blind time; writes <outdir>/eskf3_headmon.csv (every Src 13 record).
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
    /// This step's NKI records (`--nki`).
    fn nki(&self) -> Vec<Nki> {
        Vec::new()
    }
    /// Extra CSV columns of this estimator after this step (`eskf3out`): tracking errors, T_d, counters.
    fn extras(&self) -> Option<[f32; 9]> {
        None
    }
    /// The heading monitor's output and its Src 13 slot drops (`--heading-mon`, eskf3).
    fn heading_mon(&self) -> Option<(HeadingMon, u32)> {
        None
    }
}
impl Replayed for Mahony {}
impl Replayed for MahonyBaro {}
/// `eskf3out`: the output predictor around its own `Eskf::new` core. The core is leaked: the wrapper borrows it for
/// the life of the process (the board builds it as a static).
impl Replayed for EskfSteer<'static> {
    fn eskf_stats(&self) -> Option<EskfStats> {
        Some(*self.core().stats())
    }
    fn biases(&self) -> Option<([f32; 3], [f32; 3])> {
        Some(self.core().biases())
    }
    fn extras(&self) -> Option<[f32; 9]> {
        let t = self.tracking();
        Some([t.att_rad, t.vel_ms, t.pos_m, t.t_d_s, self.skipped() as f32, self.pushed() as f32, self.corrected() as f32, self.ring_dropped() as f32, self.core().stats().reinits as f32])
    }
}
impl Replayed for Eskf {
    fn eskf_stats(&self) -> Option<EskfStats> {
        Some(*self.stats())
    }
    fn biases(&self) -> Option<([f32; 3], [f32; 3])> {
        Some(Eskf::biases(self))
    }
    fn nki(&self) -> Vec<Nki> {
        self.events().copied().collect()
    }
    fn heading_mon(&self) -> Option<(HeadingMon, u32)> {
        Some((Eskf::heading_mon(self), self.heading_mon_dropped()))
    }
}

struct Row {
    name: &'static str,
    armed_at: ArmedAt,
    est: Box<dyn Replayed>,
    nav: Vec<NavState>,
    /// Gyro and accelerometer bias after each step, for the estimators that have them.
    bias: Vec<[f32; 6]>,
    /// NKI records of every step (`--nki`, eskf3 only).
    nki: Vec<Nki>,
    /// `Replayed::extras` after every step (`eskf3out`).
    extra: Vec<[f32; 9]>,
}

fn row(name: &'static str, armed_at: ArmedAt, est: Box<dyn Replayed>) -> Row {
    Row { name, armed_at, est, nav: Vec::new(), bias: Vec::new(), nki: Vec::new(), extra: Vec::new() }
}

/// The estimators to run: one line each.
/// `steer_eskf2`: eskf2 steers (the state after the previous step, as `Flight::step` sets it); else it is a shadow.
fn table(t: &Tuning, steer_eskf2: bool, steer_eskf3: bool) -> Vec<Row> {
    vec![
        row("Mahony", ArmedAt::Before, Box::new(Mahony::new(t))),
        row("MahonyBaro", ArmedAt::Before, Box::new(MahonyBaro::new(t))),
        row("eskf1", ArmedAt::After, Box::new(Eskf::with_aiding(t, Aiding::Imu).expect("Tuning::DEFAULT is accepted"))),
        row("eskf2", if steer_eskf2 { ArmedAt::Before } else { ArmedAt::After }, Box::new(Eskf::with_aiding(t, Aiding::ImuBaro).expect("Tuning::DEFAULT is accepted"))),
        row("eskf3", ArmedAt::After, Box::new(Eskf::new(t).expect("Tuning::DEFAULT is accepted"))),
        row("eskf3out", if steer_eskf3 { ArmedAt::Before } else { ArmedAt::After }, Box::new(eskf3out(t))),
    ]
}

fn eskf3out(t: &Tuning) -> EskfSteer<'static> {
    let core: &'static mut Eskf = Box::leak(Box::new(Eskf::new(t).expect("Tuning::DEFAULT is accepted")));
    EskfSteer::new(core, t).expect("the output ring is sized for Tuning::DEFAULT")
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
    /// `--gps-delay-ms MS`: replaces `Tuning::nav_gps_delay_us` in every estimator.
    gps_delay_ms: Option<f32>,
    /// `--gps-min-rate-hz HZ`: replaces `Tuning::nav_gps_min_rate_hz` in every estimator.
    gps_min_rate_hz: Option<f32>,
    /// `--no-mag`: no compass sample reaches any estimator.
    no_mag: bool,
    /// `--nki`: also write `eskf3_nki.csv` (the filter's NKI records: t_us, src, ok, innovation, variance).
    nki: bool,
    /// `--steering mahony-baro|eskf2`: which estimator steered on the board (default: read from the log).
    steering: Option<bool>,
    /// `--steering eskf3`: eskf3out is the steering estimator of the log (armed state after the previous step). No flown log
    /// has it yet; the old rows' steering is then read from the log as without the option.
    steer_eskf3: bool,
    /// `--veld`: also run `eskf2vd`, ESKF2 with the GPS VelD option (`Eskf::with_gps_vd`, Amendment V), armed as eskf2.
    veld: bool,
    /// `--holdout-vd-odd`: the `eskf2vd` row is handed the odd GNSS epochs (`seq` odd) without a velocity; no other row sees a difference.
    holdout_vd_odd: bool,
    /// `--vd-sigma M_S`: replaces `Tuning::nav_gps_vd_sigma_ms` for the `eskf2vd` row only.
    vd_sigma: Option<f32>,
    /// `--notch`: replay the log's NAV rates and ESC eRPM through the firmware's notch and D path (no estimators).
    notch: bool,
    /// `--notch-harmonics N`: `Tuning::notch_harmonics` for `--notch` (1 or 3).
    notch_harmonics: usize,
    /// `--notch-fs HZ`: the `fs_hz` of `CascadedPid::new` for `--notch` (default 800.0, the board's).
    notch_fs: f32,
    /// `--heading-mon`: the eskf3 row's heading monitor gets the rebuilt airborne latch; per-session report and CSV.
    heading_mon: bool,
}

fn parse_args() -> Result<Args, String> {
    parse_args_from(std::env::args().skip(1))
}

fn parse_args_from(mut it: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut pos = Vec::new();
    let mut every = 8usize;
    let mut noaid = None;
    let mut no_baro_from_armed = None;
    let (mut gps_delay_ms, mut gps_min_rate_hz, mut no_mag, mut nki) = (None, None, false, false);
    let mut steering = None;
    let mut steer_eskf3 = false;
    let (mut veld, mut holdout_vd_odd) = (false, false);
    let mut vd_sigma = None;
    let (mut notch, mut notch_harmonics, mut notch_fs) = (false, 3usize, 800.0f32);
    let mut opts = StreamOpts { imu_period_us: None, expected_cli: None };
    let mut heading_mon = false;
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
            "--gps-delay-ms" => {
                gps_delay_ms = Some(it.next().and_then(|v| v.parse::<f32>().ok()).filter(|v| v.is_finite() && *v >= 0.0).ok_or("--gps-delay-ms MS")?)
            }
            "--gps-min-rate-hz" => {
                gps_min_rate_hz = Some(it.next().and_then(|v| v.parse::<f32>().ok()).filter(|v| v.is_finite() && *v > 0.0).ok_or("--gps-min-rate-hz HZ")?)
            }
            "--no-mag" => no_mag = true,
            "--nki" => nki = true,
            "--veld" => veld = true,
            "--holdout-vd-odd" => holdout_vd_odd = true,
            "--vd-sigma" => vd_sigma = Some(it.next().and_then(|v| v.parse::<f32>().ok()).filter(|v| v.is_finite() && *v > 0.0).ok_or("--vd-sigma M_S")?),
            "--notch" => notch = true,
            "--heading-mon" => heading_mon = true,
            "--notch-harmonics" => notch_harmonics = it.next().and_then(|v| v.parse::<usize>().ok()).filter(|v| [1, 3].contains(v)).ok_or("--notch-harmonics 1|3")?,
            "--notch-fs" => notch_fs = it.next().and_then(|v| v.parse::<f32>().ok()).filter(|v| v.is_finite() && *v > 0.0).ok_or("--notch-fs HZ")?,
            "--steering" => match it.next().as_deref() {
                Some("eskf2") => steering = Some(true),
                Some("mahony-baro") => steering = Some(false),
                Some("eskf3") => steer_eskf3 = true,
                _ => return Err("--steering mahony-baro|eskf2|eskf3".into()),
            },
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
    Ok(Args { log: log.into(), out: out.into(), every, opts, noaid, no_baro_from_armed, gps_delay_ms, gps_min_rate_hz, no_mag, nki, steering, steer_eskf3, veld, holdout_vd_odd, vd_sigma, notch, notch_harmonics, notch_fs, heading_mon })
}

fn bits(v: &Valid) -> [u8; 7] {
    [v.tilt, v.heading, v.rates, v.velocity_ne, v.velocity_d, v.position_ne, v.position_d].map(u8::from)
}

fn write_csv(path: &std::path::Path, stream: &Stream, nav: &[NavState], bias: &[[f32; 6]], extra: &[[f32; 9]], every: usize) -> std::io::Result<()> {
    let mut w = std::io::BufWriter::new(std::fs::File::create(path)?);
    let mut head = String::from(
        "t_us,step_t_us,armed,qw,qx,qy,qz,rx,ry,rz,vn,ve,vd,pn,pe,pd,v_tilt,v_heading,v_rates,v_vel_ne,v_vel_d,v_pos_ne,v_pos_d,s_tilt,s_heading,s_vel_ne,s_vel_d,s_pos_ne,s_pos_d,gbx,gby,gbz,abx,aby,abz",
    );
    if !extra.is_empty() {
        head.push_str(",trk_att_rad,trk_vel_ms,trk_pos_m,trk_td_s,skipped,pushed,corrected,ring_dropped,core_reinits");
    }
    writeln!(w, "{head}")?;
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
        if let Some(x) = extra.get(k) {
            for v in x {
                let _ = write!(l, ",{v}");
            }
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

/// `--heading-mon`: Flight's airborne latch (step.rs, after the gate), rebuilt from what the log says of each step.
#[derive(Default)]
struct Latch {
    airborne: bool,
    low_us: u64,
    last_us: Option<u64>,
}

impl Latch {
    /// One step at `t_us`: `armed` = MOT.Arm after the step, `dmd` = its DMD (Ok, Thr N, Sat bits) or None if the log has
    /// none for the step (the latch then holds). Returns the latch after the step.
    fn step(&mut self, t_us: u64, armed: bool, dmd: Option<(bool, f32, u8)>, ground_n: f32, land_us: u64) -> bool {
        let dt_us = self.last_us.map_or(0, |prev| t_us.saturating_sub(prev).min(DT_MAX_US));
        self.last_us = Some(t_us);
        if !armed {
            self.airborne = false;
            self.low_us = 0;
            return false;
        }
        let Some((ok, thrust_n, sat)) = dmd else {
            return self.airborne;
        };
        // No allocation without a demand (DMD Sat 0), so a refused demand is low, as in Flight.
        let (thrust_ok, low) = (ok && thrust_n >= ground_n, !ok || sat & 8 != 0);
        if !low {
            if thrust_ok {
                self.airborne = true;
            }
            self.low_us = 0;
        } else {
            self.low_us += dt_us;
            if self.low_us >= land_us {
                self.airborne = false;
            }
        }
        self.airborne
    }
}

/// The rebuilt latch after each step of `stream` (DMD matched by TimeUS = the step's IMU1 time).
fn airborne_latch(log: &Log, stream: &Stream, tuning: &Tuning) -> Result<Vec<bool>, String> {
    let t = log.table("DMD")?;
    let (co, ct, cs) = (t.col("Ok")?, t.col("Thr")?, t.col("Sat")?);
    let dmd: HashMap<u64, (bool, f32, u8)> = (0..t.rows()).map(|k| (t.time(k), (t.u8(k, co) != 0, t.f32(k, ct), t.u8(k, cs)))).collect();
    let ground_n = tuning.ground_thrust_g * marv_card::MASS * fw_core::calib::GRAVITY_MSS;
    let mut latch = Latch::default();
    let mut missing = 0usize;
    let air = stream
        .steps
        .iter()
        .map(|s| {
            let d = dmd.get(&s.sensors.imu1.t_us).copied();
            missing += usize::from(s.armed && d.is_none());
            latch.step(s.sensors.imu1.t_us, s.armed, d, ground_n, tuning.land_detect_us)
        })
        .collect();
    println!(
        "heading-mon airborne: Flight's latch rebuilt from MOT.Arm and DMD (ground thrust {ground_n:.3} N, land detect {} us); armed steps without a DMD record (latch held): {missing}",
        tuning.land_detect_us
    );
    Ok(air)
}

/// One armed session of the `--heading-mon` report. Times are board us.
#[derive(Default)]
struct HmSession {
    arm_us: u64,
    end_us: u64,
    takeoff_us: Option<u64>,
    air_us: u64,
    evaluated: u32,
    screened: u32,
    informed: u32,
    max_d_rad: f32,
    confirmed_us: Option<u64>,
    fault_us: Option<u64>,
    blind_us: u64,
    /// The monitor's output after the session's last step.
    out: Option<HeadingMon>,
    slot0: u32,
    slot_drops: u32,
}

/// `--heading-mon`: the eskf3 monitor's Src 13 records and per-session figures, fed after every step.
#[derive(Default)]
struct HmReport {
    sessions: Vec<HmSession>,
    prev_armed: bool,
    last_us: Option<u64>,
    /// (step time, session index, the record).
    rows: Vec<(u64, usize, Nki)>,
}

impl HmReport {
    fn step(&mut self, t_us: u64, armed: bool, air: bool, o: &HeadingMon, slot_dropped: u32, events: &[Nki]) {
        let dt_us = self.last_us.map_or(0, |prev| t_us.saturating_sub(prev));
        self.last_us = Some(t_us);
        if armed && !self.prev_armed {
            self.sessions.push(HmSession { arm_us: t_us, slot0: slot_dropped, ..HmSession::default() });
        }
        self.prev_armed = armed;
        let Some(n) = self.sessions.len().checked_sub(1).filter(|_| armed) else {
            return;
        };
        let s = &mut self.sessions[n];
        s.end_us = t_us;
        if air {
            s.takeoff_us.get_or_insert(t_us);
            s.air_us += dt_us;
        }
        if o.blind {
            s.blind_us += dt_us;
        }
        if o.confirmed {
            s.confirmed_us.get_or_insert(t_us);
        }
        if o.fault {
            s.fault_us.get_or_insert(t_us);
        }
        s.out = Some(*o);
        s.slot_drops = slot_dropped - s.slot0;
        for e in events.iter().filter(|e| e.src == code::HEADING_MON) {
            if e.ok & code::HM_SCREENED != 0 {
                s.screened += 1;
            } else {
                s.evaluated += 1;
                if e.var.sqrt() <= SIG_CONF_RAD {
                    s.informed += 1;
                    s.max_d_rad = s.max_d_rad.max(e.inn.abs());
                }
            }
            self.rows.push((t_us, n, *e));
        }
    }

    fn text(&self) -> String {
        let mut o = String::from("\nheading monitor (eskf3, NKI Src 13), per armed session; times in s of board time\n");
        let sec = |us: u64| us as f64 * 1e-6;
        for (k, s) in self.sessions.iter().enumerate() {
            let _ = write!(o, "  session {k}: armed {:.3}..{:.3}", sec(s.arm_us), sec(s.end_us));
            let Some(t0) = s.takeoff_us else {
                let _ = writeln!(o, ", never airborne; Src 13 records {}", s.evaluated + s.screened);
                continue;
            };
            let after = |t: Option<u64>| t.map_or("none".to_string(), |t| format!("at {:.3} ({:.3} after take-off)", sec(t), sec(t) - sec(t0)));
            let Some(m) = s.out else { continue };
            let _ = writeln!(
                o,
                ", take-off {:.3}, airborne {:.1}; evaluated {} screened(R2) {} dropped(R2') {} non-finite {} out-of-interval {} slot drops {}; informed {:.1} %; max |d_hat| informed {:.2} deg; confirmed {}; fault {}; blind {:.2}; last d_hat {:.2} deg sig {:.2} deg",
                sec(t0), sec(s.air_us), s.evaluated, s.screened, m.dropped_r2p, m.non_finite, m.out_of_interval, s.slot_drops,
                100.0 * f64::from(s.informed) / f64::from(s.evaluated.max(1)), s.max_d_rad.to_degrees(), after(s.confirmed_us),
                after(s.fault_us), sec(s.blind_us), m.delta_rad.to_degrees(), m.sigma_rad.to_degrees()
            );
        }
        o
    }

    fn write_csv(&self, path: &std::path::Path) -> std::io::Result<()> {
        let mut w = std::io::BufWriter::new(std::fs::File::create(path)?);
        writeln!(w, "step_t_us,t_us,sess,d_hat_rad,var_rad2,ok")?;
        for (t, n, e) in &self.rows {
            writeln!(w, "{t},{},{n},{},{},{}", e.t_us, e.inn, e.var, e.ok)?;
        }
        w.flush()
    }
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

/// `--notch`: the sample rate of the D-torque metric, Hz: the 803.86 Hz of t292-notch.py (the log's measured IMU1 step is 1244 us).
const METRIC_FS_HZ: f32 = 803.86;
/// `--notch`: the hover windows of t292-notch.py, s: name, from, to, and its what-if figures (D torque above 50 Hz, N.m,
/// X and Y) from t292-notch.out: without the notch, h1 only (no fade, minimum 60 Hz), h1-h3 (no fade, minimum 60 Hz).
const NOTCH_WINDOWS: [(&str, f64, f64, [[f64; 2]; 3]); 2] = [
    ("Pos hover", 753.0, 825.0, [[0.0155, 0.0215], [0.0061, 0.0099], [0.0044, 0.0072]]),
    ("Alt hover", 398.0, 412.5, [[0.0149, 0.0227], [0.0052, 0.0073], [0.0037, 0.0057]]),
];

/// In-place radix-2 FFT of `re` + i `im` (length a power of 2).
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -2.0 * std::f64::consts::PI / len as f64;
        for s in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (wr, wi) = ((ang * k as f64).cos(), (ang * k as f64).sin());
                let (a, b) = (s + k, s + k + len / 2);
                let (tr, ti) = (re[b] * wr - im[b] * wi, re[b] * wi + im[b] * wr);
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
        len <<= 1;
    }
}

/// The rms of `x` above `lo_hz`, as t292-notch.py `hf`: `scipy.signal.welch(x - mean(x), fs, nperseg=2048)` (Hann, 50 %
/// overlap, constant detrend per segment, one-sided density, the DC and Nyquist bins not doubled), then
/// sqrt(sum of the density over the bins at or above `lo_hz` x the bin width).
fn hf_rms(x: &[f32], fs: f64, lo_hz: f64) -> Result<f64, String> {
    const N: usize = 2048;
    const HOP: usize = N / 2;
    if x.len() < N {
        return Err(format!("window of {} samples is shorter than one 2048-sample segment", x.len()));
    }
    let mean = x.iter().map(|&v| f64::from(v)).sum::<f64>() / x.len() as f64;
    let win: Vec<f64> = (0..N).map(|j| 0.5 - 0.5 * (2.0 * std::f64::consts::PI * j as f64 / N as f64).cos()).collect();
    let scale = 1.0 / (fs * win.iter().map(|w| w * w).sum::<f64>());
    let nseg = (x.len() - HOP) / HOP;
    let mut acc = vec![0.0f64; N / 2 + 1];
    for s in 0..nseg {
        let seg: Vec<f64> = x[s * HOP..s * HOP + N].iter().map(|&v| f64::from(v) - mean).collect();
        let m = seg.iter().sum::<f64>() / N as f64;
        let mut re: Vec<f64> = seg.iter().zip(&win).map(|(v, w)| (v - m) * w).collect();
        let mut im = vec![0.0; N];
        fft(&mut re, &mut im);
        for (k, a) in acc.iter_mut().enumerate() {
            *a += re[k] * re[k] + im[k] * im[k];
        }
    }
    let df = fs / N as f64;
    let mut sum = 0.0;
    for (k, a) in acc.iter().enumerate() {
        if k as f64 * df >= lo_hz {
            let dbl = if k == 0 || k == N / 2 { 1.0 } else { 2.0 };
            sum += a / nseg as f64 * scale * dbl;
        }
    }
    Ok((sum * df).sqrt())
}

/// What one pass of the rate loop left for one NAV step.
#[derive(Clone, Copy)]
struct Pass {
    rate: [f32; 3],
    d: [f32; 3],
}

/// The log's ATC_* gain PARM records as (TimeUS, the 14 gains in force after it): each record changes its gain on top of the
/// previous set (the defaults at the start). `None` when the log has no PARM record of any of the 14. A value outside
/// `Gains::MIN`..`Gains::MAX` is an error.
fn parm_gains(log: &Log) -> Result<Option<Vec<(u64, Gains)>>, String> {
    let Ok(t) = log.table("PARM") else { return Ok(None) };
    let (cn, cv) = (t.col("Name")?, t.col("Value")?);
    let mut g = Gains::of(&Tuning::DEFAULT);
    let mut ev = Vec::new();
    for r in 0..t.rows() {
        let name = t.text(r, cn);
        let Some(i) = GAIN_NAMES.iter().position(|n| *n == name) else { continue };
        let v = t.f32(r, cv);
        if !(Gains::MIN.0[i]..=Gains::MAX.0[i]).contains(&v) {
            return Err(format!("PARM {name} = {v} at {} us is outside {} to {}", t.time(r), Gains::MIN.0[i], Gains::MAX.0[i]));
        }
        g.0[i] = v;
        ev.push((t.time(r), g));
    }
    Ok(if ev.is_empty() { None } else { Some(ev) })
}

/// `--notch`: the log's NAV rates and ESC eRPM through the firmware's `CascadedPid` (the notch, the rate error and the D
/// path), as `Flight::step` drives it: `reset` when the gate was disarmed at the start of the step, then `set_rotor_hz`,
/// then `update`. Three controllers see the same steps: notch off, notch on, notch on with every eRPM withheld.
fn notch_replay(log: &Log, args: &Args) -> Result<(), String> {
    let nav = log.table("NAV")?;
    let (cgx, cgy, cgz) = (nav.col("GX")?, nav.col("GY")?, nav.col("GZ")?);
    let mot = log.table("MOT")?;
    let carm = mot.col("Arm")?;
    let esc = log.table("ESC")?;
    let ce = [esc.col("E0")?, esc.col("E1")?, esc.col("E2")?, esc.col("E3")?];
    if mot.rows() != nav.rows() || (0..nav.rows()).any(|k| mot.time(k) != nav.time(k)) {
        return Err("NAV and MOT records are not one per step with the same TimeUS".into());
    }
    let mut tuning = Tuning::DEFAULT;
    tuning.notch_harmonics = args.notch_harmonics;
    // The gains in force: the newest ATC_* PARM at or before each arming (the board latches them on disarmed steps);
    // Tuning::DEFAULT when the log carries none (log 292 and older).
    let gain_events = parm_gains(log)?;
    println!("gains source: {}", match &gain_events {
        None => "Tuning::DEFAULT (the log has no ATC_* PARM records)".to_string(),
        Some(ev) => format!("the log's PARM records ({} ATC_* records; a gain the log lacks stays at its default)", ev.len()),
    });
    let mut next_ev = 0usize;
    let poles = 14.0f32; // MOT_POLES default (step.rs NOTCH_POLES_DEFAULT)
    let sat = Saturation { roll: false, pitch: false, yaw: false, thrust_low: false, thrust_high: false };
    let mut off = CascadedPid::new(&tuning, args.notch_fs);
    let mut on = CascadedPid::new(&tuning, args.notch_fs);
    let mut ctl = CascadedPid::new(&tuning, args.notch_fs);
    let n = nav.rows();
    let (mut p_off, mut p_on, mut p_ctl) = (Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n));
    let mut f1_used = Vec::with_capacity(n);
    let mut input = Vec::with_capacity(n);
    let mut j = 0usize; // newest ESC record with TimeUS <= the step's
    let (mut armed_steps, mut armed_all4) = (0usize, 0usize);
    for k in 0..n {
        let t = nav.time(k);
        while j + 1 < esc.rows() && esc.time(j + 1) <= t {
            j += 1;
        }
        let f1: [Option<f32>; 4] = std::array::from_fn(|l| {
            if esc.time(j) > t || t - esc.time(j) >= tuning.notch_hold_us {
                return None;
            }
            Some(esc.u32(j, ce[l]) as f32 * 2.0 / (poles * 60.0))
        });
        let rates = [nav.f32(k, cgx), nav.f32(k, cgy), nav.f32(k, cgz)];
        let n_state = NavState {
            t_us: t,
            attitude: [1.0, 0.0, 0.0, 0.0],
            rates_rads: rates,
            velocity_ms: [0.0; 3],
            position_m: [0.0; 3],
            valid: Valid { tilt: true, rates: true, ..Valid::NONE },
            sigma: None,
        };
        let sp = Setpoint::Rates { rates_rads: [0.0; 3], thrust_n: marv_card::MASS * fw_core::calib::GRAVITY_MSS };
        let armed_before = k > 0 && mot.u8(k - 1, carm) != 0;
        if let (Some(ev), false) = (&gain_events, armed_before) {
            while next_ev < ev.len() && ev[next_ev].0 <= nav.time(k) {
                next_ev += 1;
            }
            if next_ev > 0 {
                for c in [&mut off, &mut on, &mut ctl] {
                    c.set_gains(&ev[next_ev - 1].1);
                }
            }
        }
        let one = |c: &mut CascadedPid, f: [Option<f32>; 4], sw: bool| -> Result<Pass, String> {
            if !armed_before {
                c.reset();
            }
            c.set_rotor_hz(f, sw);
            c.update(&n_state, &sp, &sat).map_err(|_| format!("step {k}: the controller refused a rate setpoint"))?;
            let l = c.rate_log();
            Ok(Pass { rate: l.rate_rads, d: l.d_nm })
        };
        p_off.push(one(&mut off, f1, false)?);
        p_on.push(one(&mut on, f1, true)?);
        p_ctl.push(one(&mut ctl, [None; 4], true)?);
        f1_used.push(f1.map(|f| f.unwrap_or(0.0)));
        input.push(rates);
        if mot.u8(k, carm) != 0 {
            armed_steps += 1;
            armed_all4 += usize::from(f1.iter().all(Option::is_some));
        }
    }
    println!("notch replay: {n} NAV steps ({armed_steps} armed, of which {armed_all4} with all four eRPM usable), harmonics {}, controller fs {} Hz, Tuning::DEFAULT otherwise", tuning.notch_harmonics, args.notch_fs);

    // Negative control: every eRPM withheld -> the rates the loop used are the input, bit for bit, and the D torque is the
    // notch-off one.
    let (mut rate_diff, mut d_diff) = (0usize, 0usize);
    for k in 0..n {
        rate_diff += (0..3).filter(|&a| p_ctl[k].rate[a].to_bits() != input[k][a].to_bits()).count();
        d_diff += (0..3).filter(|&a| p_ctl[k].d[a].to_bits() != p_off[k].d[a].to_bits()).count();
    }
    println!(
        "negative control (notch on, every eRPM withheld): rates differing from the input bit for bit: {rate_diff} of {} values; D torque differing from notch-off: {d_diff} of {}: {}",
        3 * n,
        3 * n,
        if rate_diff == 0 && d_diff == 0 { "PASS" } else { "FAIL" }
    );
    let off_diff = (0..n).flat_map(|k| (0..3).map(move |a| (k, a))).filter(|&(k, a)| p_off[k].rate[a].to_bits() != input[k][a].to_bits()).count();
    println!("notch off: rates differing from the input bit for bit: {off_diff} of {} values", 3 * n);

    let path = args.out.join(format!("notch_h{}.csv", tuning.notch_harmonics));
    let mut w = std::io::BufWriter::new(std::fs::File::create(&path).map_err(|e| format!("{}: {e}", path.display()))?);
    writeln!(w, "t_us,armed,f1_0,f1_1,f1_2,f1_3,gx_in,gy_in,gz_in,gx_off,gy_off,gz_off,gx_on,gy_on,gz_on,dx_off,dy_off,dz_off,dx_on,dy_on,dz_on").map_err(|e| e.to_string())?;
    for k in 0..n {
        let mut l = format!("{},{}", nav.time(k), mot.u8(k, carm));
        for v in f1_used[k].iter().chain(&input[k]).chain(&p_off[k].rate).chain(&p_on[k].rate).chain(&p_off[k].d).chain(&p_on[k].d) {
            let _ = write!(l, ",{v}");
        }
        writeln!(w, "{l}").map_err(|e| e.to_string())?;
    }
    w.flush().map_err(|e| e.to_string())?;
    println!("wrote {}", path.display());

    println!("\nD torque above 50 Hz (rms of the Welch density from 50 Hz up, N.m; Hann, 2048 samples, 50 % overlap, mean removed, {METRIC_FS_HZ} Hz); what-if = t292-notch.out");
    println!("  {:<10} {:>26} {:>12} {:>12} {:>9}  {:>12} {:>12}", "window", "case", "X", "Y", "cut X/Y %", "what-if X", "what-if Y");
    let fs = f64::from(METRIC_FS_HZ);
    for (name, a, b, whatif) in NOTCH_WINDOWS {
        let sel: Vec<usize> = (0..n).filter(|&k| nav.time(k) as f64 * 1e-6 >= a && (nav.time(k) as f64 * 1e-6) < b).collect();
        let hf = |p: &[Pass], axis: usize| hf_rms(&sel.iter().map(|&k| p[k].d[axis]).collect::<Vec<f32>>(), fs, 50.0);
        let base = [hf(&p_off, 0)?, hf(&p_off, 1)?];
        let withh = [hf(&p_on, 0)?, hf(&p_on, 1)?];
        let cut = |i: usize| 100.0 * (1.0 - withh[i] / base[i]);
        let h_label = if tuning.notch_harmonics == 1 { "notch h1" } else { "notch h1-h3" };
        let w_idx = if tuning.notch_harmonics == 1 { 1 } else { 2 };
        println!("  {:<10} {:>26} {:>12.4} {:>12.4} {:>9}  {:>12.4} {:>12.4}   ({} samples)", name, "no notch", base[0], base[1], "", whatif[0][0], whatif[0][1], sel.len());
        println!(
            "  {:<10} {:>26} {:>12.4} {:>12.4} {:>4.1}/{:<4.1}  {:>12.4} {:>12.4}   what-if cut {:.1}/{:.1} %",
            name, h_label, withh[0], withh[1], cut(0), cut(1), whatif[w_idx][0], whatif[w_idx][1],
            100.0 * (1.0 - whatif[w_idx][0] / whatif[0][0]), 100.0 * (1.0 - whatif[w_idx][1] / whatif[0][1])
        );
    }
    Ok(())
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
    if args.notch {
        return notch_replay(&log, &args);
    }
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

    if args.no_mag {
        let withheld: usize = stream.steps.iter_mut().map(|s| usize::from(s.sensors.mag.take().is_some())).sum();
        println!("compass withheld: {withheld} samples not handed to any estimator");
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
    if let Some(ms) = args.gps_delay_ms {
        tuning.nav_gps_delay_us = (ms * 1000.0).round() as u64;
    }
    if let Some(hz) = args.gps_min_rate_hz {
        tuning.nav_gps_min_rate_hz = hz;
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
    let mut rows = table(&tuning, steer, args.steer_eskf3);
    if args.veld {
        let at = if steer { ArmedAt::Before } else { ArmedAt::After };
        let mut t_vd = tuning;
        if let Some(s) = args.vd_sigma {
            t_vd.nav_gps_vd_sigma_ms = s;
        }
        rows.push(row("eskf2vd", at, Box::new(Eskf::with_gps_vd(&t_vd).expect("Tuning::DEFAULT is accepted"))));
    }
    for r in &mut rows {
        r.nav.reserve(stream.steps.len());
    }
    let air = if args.heading_mon { airborne_latch(&log, &stream, &tuning)? } else { Vec::new() };
    let mut hm = args.heading_mon.then(HmReport::default);
    let mut prev_armed = false;
    for (k, s) in stream.steps.iter().enumerate() {
        for r in &mut rows {
            r.est.set_armed(if r.armed_at == ArmedAt::Before { prev_armed } else { s.armed });
            if args.heading_mon && r.name == "eskf3" {
                r.est.set_airborne(k > 0 && air[k - 1]);
            }
            let mut held = None;
            if args.holdout_vd_odd && r.name == "eskf2vd" && s.sensors.gnss.is_some_and(|g| g.seq % 2 == 1) {
                let mut h = s.sensors.clone();
                h.gnss.as_mut().map(|g| g.reading.vel_ned_mm_s = None);
                held = Some(h);
            }
            r.nav.push(r.est.update(held.as_ref().unwrap_or(&s.sensors)));
            if let Some(x) = r.est.extras() {
                r.extra.push(x);
            }
            if args.nki && r.name == "eskf3" {
                r.nki.extend(r.est.nki());
            }
            if let (Some(h), Some((o, dropped)), true) = (hm.as_mut(), r.est.heading_mon(), r.name == "eskf3") {
                h.step(s.sensors.imu1.t_us, s.armed, air[k], &o, dropped, &r.est.nki());
            }
            if let Some((g, a)) = r.est.biases() {
                r.bias.push([g[0], g[1], g[2], a[0], a[1], a[2]]);
            }
        }
        prev_armed = s.armed;
    }

    // The largest per-step change of the output's down-in-body vector beyond what the gyro accounts for:
    // |d_k - d_(k-1)| - |omega_k| dt, at every step (not every 8th), for the rows named below.
    for r in rows.iter().filter(|r| ["eskf3out", "eskf3", "eskf2"].contains(&r.name)) {
        let down = |q: &[f32; 4]| [2.0 * (q[1] * q[3] - q[0] * q[2]), 2.0 * (q[2] * q[3] + q[0] * q[1]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])];
        let (mut all, mut armed) = ((f32::MIN, 0u64), (f32::MIN, 0u64));
        for k in 1..r.nav.len() {
            let (a, b) = (&r.nav[k - 1], &r.nav[k]);
            if a.t_us == 0 || b.t_us == 0 || !b.valid.tilt || !a.valid.tilt {
                continue;
            }
            let (da, db) = (down(&a.attitude), down(&b.attitude));
            let dd = ((db[0] - da[0]).powi(2) + (db[1] - da[1]).powi(2) + (db[2] - da[2]).powi(2)).sqrt();
            let g = stream.steps[k].sensors.imu1.reading.gyro_rads;
            let dt = stream.steps[k].sensors.imu1.t_us.saturating_sub(stream.steps[k - 1].sensors.imu1.t_us).min(20_000) as f32 * 1e-6;
            let ex = dd - (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt() * dt;
            let t = stream.steps[k].sensors.imu1.t_us;
            if ex > all.0 {
                all = (ex, t);
            }
            if stream.steps[k].armed && ex > armed.0 {
                armed = (ex, t);
            }
        }
        println!(
            "step excess {}: largest |d down| - |omega| dt over all steps {:.3e} rad at {:.3} s; while armed {:.3e} rad at {:.3} s",
            r.name,
            all.0,
            all.1 as f64 * 1e-6,
            armed.0,
            armed.1 as f64 * 1e-6
        );
    }
    for r in &rows {
        let path = args.out.join(format!("{}.csv", r.name));
        write_csv(&path, &stream, &r.nav, &r.bias, &r.extra, args.every).map_err(|e| format!("{}: {e}", path.display()))?;
        println!("wrote {}", path.display());
    }
    if args.nki {
        for r in rows.iter().filter(|r| r.name == "eskf3") {
            let path = args.out.join("eskf3_nki.csv");
            let mut w = std::io::BufWriter::new(std::fs::File::create(&path).map_err(|e| format!("{}: {e}", path.display()))?);
            writeln!(w, "t_us,src,ok,inn,var").map_err(|e| e.to_string())?;
            for n in &r.nki {
                writeln!(w, "{},{},{},{},{}", n.t_us, n.src, n.ok, n.inn, n.var).map_err(|e| e.to_string())?;
            }
            w.flush().map_err(|e| e.to_string())?;
            println!("wrote {}", path.display());
        }
    }
    if let Some(h) = &hm {
        let path = args.out.join("eskf3_headmon.csv");
        h.write_csv(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        print!("{}", h.text());
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

#[cfg(test)]
mod tests {
    use super::*;

    fn args(v: &[&str]) -> Result<Args, String> {
        parse_args_from(v.iter().map(|s| s.to_string()))
    }

    #[test]
    fn heading_mon_option_parses() {
        assert!(!args(&["a.BIN", "out"]).unwrap().heading_mon);
        let a = args(&["a.BIN", "out", "--heading-mon", "--nki"]).unwrap();
        assert!(a.heading_mon && a.nki);
        assert!(args(&["--heading-mon", "a.BIN", "out"]).unwrap().heading_mon);
        assert!(args(&["a.BIN", "--heading-mon"]).is_err(), "the out dir is still required");
        assert!(args(&["a.BIN", "out", "--heading-monitor"]).is_err());
    }

    /// The rebuilt latch on a synthetic DMD stream: rises only armed with a demand at or above the ground thrust and not
    /// low; holds over a missing record; falls after `land_detect_us` low (refused demand or Sat bit 3), dt capped at
    /// DT_MAX_US; a disarmed step clears it at once.
    #[test]
    fn airborne_latch_follows_flights_rule() {
        let (g, land, p) = (5.0f32, 1_000_000u64, 1_250u64);
        let mut l = Latch::default();
        let t = std::cell::Cell::new(0u64);
        let run = |l: &mut Latch, n: usize, armed: bool, dmd: Option<(bool, f32, u8)>| {
            let mut a = false;
            for _ in 0..n {
                t.set(t.get() + p);
                a = l.step(t.get(), armed, dmd, g, land);
            }
            a
        };
        assert!(!run(&mut l, 10, false, Some((true, 9.0, 0))), "disarmed");
        assert!(!run(&mut l, 10, true, Some((true, 4.9, 0))), "below the ground thrust");
        assert!(!run(&mut l, 10, true, Some((true, 6.0, 8))), "thrust_low");
        assert!(!run(&mut l, 10, true, Some((false, 6.0, 0))), "refused demand");
        assert!(run(&mut l, 1, true, Some((true, 5.0, 0))), "at the ground thrust, not low");
        assert!(run(&mut l, 10, true, None), "no DMD: held");
        assert!(run(&mut l, 10, true, Some((true, 3.0, 0))), "below the ground thrust but not low: stays");
        assert!(run(&mut l, 799, true, Some((true, 3.0, 8))), "low 0.99875 s");
        assert!(!run(&mut l, 1, true, Some((true, 3.0, 8))), "low 1.0 s");
        assert!(run(&mut l, 1, true, Some((true, 6.0, 0))));
        assert!(run(&mut l, 1, true, Some((false, 0.0, 0))), "refused: low from here");
        t.set(t.get() + 2_000_000);
        assert!(l.step(t.get(), true, Some((false, 0.0, 0)), g, land), "a 2 s gap adds only DT_MAX_US");
        assert!(run(&mut l, 790, true, Some((false, 0.0, 0))));
        assert!(!run(&mut l, 9, true, Some((false, 0.0, 0))));
        assert!(run(&mut l, 1, true, Some((true, 6.0, 0))));
        assert!(!run(&mut l, 1, false, Some((true, 6.0, 0))), "disarm clears at once");
    }

    fn mon(confirmed: bool, fault: bool, blind: bool) -> HeadingMon {
        HeadingMon { ok: confirmed && !fault && !blind, confirmed, fault, delta_rad: 0.0, sigma_rad: f32::INFINITY, info: 0.0,
            screened_r2: 0, dropped_r2p: 0, non_finite: 0, out_of_interval: 0, blind }
    }

    /// The session figures from synthetic steps: screened records are not evaluated, only informed ones (sig <= 15 deg)
    /// enter max |d_hat|, times count from the first airborne step, disarmed steps belong to no session.
    #[test]
    fn heading_mon_report_counts_a_synthetic_session() {
        let rec = |inn: f32, sig_deg: f32, ok: u8| Nki { t_us: 0, src: code::HEADING_MON, inn, var: sig_deg.to_radians().powi(2), ok };
        let other = Nki { t_us: 0, src: code::HEADING_MON - 1, inn: 9.0, var: 0.0, ok: 0 };
        let mut h = HmReport::default();
        h.step(1_000_000, false, false, &mon(false, false, false), 0, &[rec(0.1, 1.0, 0)]);
        h.step(2_000_000, true, false, &mon(false, false, false), 0, &[]);
        h.step(3_000_000, true, true, &mon(false, false, false), 0, &[rec(0.3, 20.0, 0), other]);
        h.step(4_000_000, true, true, &mon(false, false, true), 0, &[rec(0.2, 10.0, 0)]);
        h.step(5_000_000, true, true, &mon(true, false, false), 1, &[rec(0.5, 10.0, code::HM_SCREENED | code::HM_CONFIRMED)]);
        h.step(6_000_000, true, true, &mon(true, true, false), 1, &[rec(-0.25, 14.9, code::HM_CONFIRMED | code::HM_FAULT)]);
        h.step(7_000_000, false, false, &mon(true, true, false), 1, &[rec(0.9, 1.0, 0)]);
        h.step(8_000_000, true, false, &mon(false, false, false), 1, &[]);
        assert_eq!(h.sessions.len(), 2);
        let s = &h.sessions[0];
        assert_eq!((s.arm_us, s.end_us, s.takeoff_us, s.air_us), (2_000_000, 6_000_000, Some(3_000_000), 4_000_000));
        assert_eq!((s.evaluated, s.screened, s.informed), (3, 1, 2));
        assert!((s.max_d_rad - 0.25).abs() < 1e-6, "{}", s.max_d_rad);
        assert_eq!((s.confirmed_us, s.fault_us, s.blind_us, s.slot_drops), (Some(5_000_000), Some(6_000_000), 1_000_000, 1));
        assert_eq!(h.rows.len(), 4);
        assert!(h.sessions[1].takeoff_us.is_none() && h.sessions[1].slot_drops == 0);
        let text = h.text();
        assert!(text.contains("confirmed at 5.000 (2.000 after take-off)") && text.contains("never airborne"), "{text}");
    }
}
