//! The slow sensors: the barometer, compass and GNSS models (decision-nav-eskf-20261120, section 9a-c). Each makes the
//! firmware's own sample type (`fw_core::sample`) at the device's rate, stamped `t_us` on the sim clock with a `seq`
//! +1 per sample, from the truth of `latency` earlier. `SlowSensors` keeps the newest sample of each device until the
//! next flight step takes it, once: the board's slow-sensor path (decision section 4a, `fresh.then_some(..)`).
//! Each model draws from its own noise stream, so the IMU streams and every existing run are unchanged.

use crate::backend::Truth;
use crate::math::{Mat3, Quat, Vec3, mat_t_vec, mat_vec, rot_matrix};
use crate::rng::Rng;
use fw_core::sample::{BaroReading, BaroSample, GnssReading, GnssSample, MagReading, MagSample};
use marv_card::{Card, Env, Record};
use std::collections::VecDeque;

/// The base tick, us (conventions 6).
pub const TICK_US: u64 = 125;
/// Barometer: 50 Hz, the firmware default (card `[baro]`).
pub const BARO_PERIOD_US: u64 = 20_000;
/// Compass: 100 Hz, the firmware default (card `[compass]`).
pub const MAG_PERIOD_US: u64 = 10_000;
/// The GNSS rates a scenario may choose (section 9c), and the default: the board's `GPS_RATE_MS` 200.
pub const GNSS_RATES_HZ: [u32; 3] = [1, 5, 10];
pub const GNSS_DEFAULT_RATE_HZ: u32 = 5;
/// Below this ground speed the receiver holds its last course (section 9c), m/s.
pub const COURSE_HOLD_MS: f64 = 0.3;
/// Noise streams (src/rng.rs); IMU1 and IMU2 are 1 and 2.
pub const STREAM_BARO: u64 = 3;
pub const STREAM_MAG: u64 = 4;
pub const STREAM_GNSS: u64 = 5;
/// ISA (ISO 2533): the specific gas constant of dry air, J/(kg K); with the env's ISA numbers it gives the
/// troposphere's pressure law.
const ISA_GAS_CONSTANT: f64 = 287.05287;
/// WGS 84 ellipsoid: semi-major axis, m, and flattening (the local-tangent radii of `geo_from_ned`).
const WGS84_A: f64 = 6_378_137.0;
const WGS84_F: f64 = 1.0 / 298.257_223_563;

fn need<T: Copy>(record: &Record<T>, key: &str) -> Result<T, String> {
    record.v.ok_or_else(|| format!("{key} is \"ask\", the slow-sensor models need a value"))
}

fn ticks(seconds: f64) -> usize {
    (seconds * 1e6 / TICK_US as f64).round() as usize
}

/// What the models read of the truth at one tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Snapshot {
    pub position_m: Vec3,
    pub velocity_ms: Vec3,
    pub attitude: Quat,
    pub on_ground: bool,
}

impl Snapshot {
    pub fn of(truth: &Truth) -> Snapshot {
        Snapshot {
            position_m: truth.position_m,
            velocity_ms: truth.velocity_ms,
            attitude: truth.attitude,
            on_ground: truth.on_ground,
        }
    }
}

/// The truth of the last `depth + 1` ticks, newest at the back: where every latency is taken from.
pub struct TruthDelay {
    buf: VecDeque<Snapshot>,
    depth: usize,
}

impl TruthDelay {
    pub fn new(depth: usize) -> TruthDelay {
        TruthDelay { buf: VecDeque::with_capacity(depth + 1), depth }
    }

    pub fn push(&mut self, s: Snapshot) {
        self.buf.push_back(s);
        if self.buf.len() > self.depth + 1 {
            self.buf.pop_front();
        }
    }

    /// The truth `ticks` ticks ago; before that much history exists, the oldest. Needs one push first.
    pub fn ago(&self, ticks: usize) -> Snapshot {
        let n = self.buf.len();
        self.buf[n - 1 - ticks.min(n - 1)]
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Barometer (9a)
// ---------------------------------------------------------------------------------------------------------------------

/// ISA pressure at the origin altitude minus down, with the drift random walk and, in the air, the extra height noise
/// (both in height, before the pressure law), then the card's pressure bias and white pressure noise. Not modelled:
/// the ground-effect error and the speed error.
pub struct BaroModel {
    origin_alt_m: f64,
    p0: f64,
    t0: f64,
    lapse: f64,
    exponent: f64,
    bias_pa: f64,
    noise_pa: f64,
    airborne_sd_m: f64,
    /// The drift's step per sample, 1 sigma, m: drift_sd x sqrt(period / drift_time).
    drift_step_m: f64,
    drift_m: f64,
    pub latency_ticks: usize,
    rng: Rng,
    seq: u32,
}

impl BaroModel {
    pub fn new(card: &Card, env: &Env, rng: Rng) -> Result<BaroModel, String> {
        let b = &card.baro;
        let g = need(&env.gravity, "gravity.g")?;
        let lapse = need(&env.lapse_rate, "atmosphere.lapse_rate")?;
        let drift_time = need(&b.drift_time, "baro.drift_time")?;
        Ok(BaroModel {
            origin_alt_m: need(&env.origin_altitude, "origin.altitude")?,
            p0: need(&env.sea_level_pressure, "atmosphere.sea_level_pressure")?,
            t0: need(&env.sea_level_temperature, "atmosphere.sea_level_temperature")?,
            lapse,
            exponent: g / (lapse * ISA_GAS_CONSTANT),
            bias_pa: need(&b.pressure_bias, "baro.pressure_bias")?,
            noise_pa: need(&b.pressure_noise_sd, "baro.pressure_noise_sd")?,
            airborne_sd_m: need(&b.airborne_height_noise_sd, "baro.airborne_height_noise_sd")?,
            drift_step_m: need(&b.drift_sd, "baro.drift_sd")? * (BARO_PERIOD_US as f64 * 1e-6 / drift_time).sqrt(),
            drift_m: 0.0,
            latency_ticks: ticks(need(&b.latency, "baro.latency")?),
            rng,
            seq: 0,
        })
    }

    /// ISA troposphere at `height_m` above mean sea level: (pressure Pa, temperature K).
    pub fn isa(&self, height_m: f64) -> (f64, f64) {
        let t = self.t0 - self.lapse * height_m;
        (self.p0 * (t / self.t0).powf(self.exponent), t)
    }

    /// The sample read at `t_us` from `past`, the truth `latency_ticks` earlier.
    pub fn sample(&mut self, t_us: u64, past: &Snapshot) -> BaroSample {
        // Fixed draw order (drift, airborne, pressure) whatever the contact, so the stream does not depend on it.
        self.drift_m += self.drift_step_m * self.rng.gaussian();
        let airborne = self.airborne_sd_m * self.rng.gaussian();
        let white = self.noise_pa * self.rng.gaussian();
        let height = self.origin_alt_m - past.position_m[2];
        let error_m = self.drift_m + if past.on_ground { 0.0 } else { airborne };
        let (p, _) = self.isa(height + error_m);
        let (_, kelvin) = self.isa(height);
        let s = BaroSample {
            reading: BaroReading { pressure_pa: (p + self.bias_pa + white) as f32, temperature_c: (kelvin - 273.15) as f32 },
            t_us,
            seq: self.seq,
        };
        self.seq = self.seq.wrapping_add(1);
        s
    }

    /// The drift's present value, m (for the tests).
    pub fn drift_m(&self) -> f64 {
        self.drift_m
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Compass (9b)
// ---------------------------------------------------------------------------------------------------------------------

/// The calibration the firmware applies to the raw field: `correction x (raw - hard_iron_ut)` (body FRD, uT, after
/// `MAG_ORIENT`). A perfect calibration has `hard_iron_ut` = the true hard iron and `correction` = the inverse of the true
/// soft iron.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Calibration {
    pub hard_iron_ut: Vec3,
    pub correction: Mat3,
}

impl Calibration {
    /// No correction: the runner uses this, so the card's soft and hard iron are what the firmware's calibration leaves.
    pub const NONE: Calibration =
        Calibration { hard_iron_ut: [0.0; 3], correction: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] };
}

/// The env field turned into body FRD by the true attitude; then the TRUE distortion, `soft_iron x field +
/// hard_iron_bias` (the card's `compass.soft_iron`, `compass.hard_iron_bias`), plus the card's measured noise; then the
/// CALIBRATION APPLIED (`Calibration`). The sample is the calibrated field, what the board hands to flight code.
pub struct MagModel {
    field_ned_ut: Vec3,
    soft_iron: Mat3,
    hard_iron_ut: Vec3,
    noise_ut: Vec3,
    calibration: Calibration,
    temperature_c: f32,
    pub latency_ticks: usize,
    rng: Rng,
    seq: u32,
}

impl MagModel {
    pub fn new(card: &Card, env: &Env, calibration: Calibration, rng: Rng) -> Result<MagModel, String> {
        let c = &card.compass;
        Ok(MagModel {
            field_ned_ut: need(&env.magnetic_field_ned, "magnetic_field.ned")?,
            soft_iron: need(&c.soft_iron, "compass.soft_iron")?,
            hard_iron_ut: need(&c.hard_iron_bias, "compass.hard_iron_bias")?,
            noise_ut: need(&c.field_noise_sd, "compass.field_noise_sd")?,
            calibration,
            temperature_c: (need(&env.sea_level_temperature, "atmosphere.sea_level_temperature")? - 273.15) as f32,
            latency_ticks: ticks(need(&c.latency, "compass.latency")?),
            rng,
            seq: 0,
        })
    }

    /// Without noise (the tests set it to zero to check the calibration identity).
    pub fn without_noise(mut self) -> MagModel {
        self.noise_ut = [0.0; 3];
        self
    }

    pub fn sample(&mut self, t_us: u64, past: &Snapshot) -> MagSample {
        let body = mat_t_vec(&rot_matrix(past.attitude), self.field_ned_ut);
        let distorted = mat_vec(&self.soft_iron, body);
        let mut raw = [0.0; 3];
        for i in 0..3 {
            raw[i] = distorted[i] + self.hard_iron_ut[i] + self.noise_ut[i] * self.rng.gaussian();
        }
        let cal = &self.calibration;
        let field = mat_vec(&cal.correction, [0, 1, 2].map(|i| raw[i] - cal.hard_iron_ut[i]));
        let s = MagSample {
            reading: MagReading { field_ut: field.map(|x| x as f32), temperature_c: self.temperature_c },
            t_us,
            seq: self.seq,
        };
        self.seq = self.seq.wrapping_add(1);
        s
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// GNSS (9c)
// ---------------------------------------------------------------------------------------------------------------------

/// The NED origin of the sim's truth frame: the env's `[origin]`, kept in the receiver's units (1e-7 deg, mm).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Origin {
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub alt_mm: i32,
}

impl Origin {
    pub fn from_env(env: &Env) -> Result<Origin, String> {
        let lat = need(&env.origin_latitude, "origin.latitude")?;
        let lon = need(&env.origin_longitude, "origin.longitude")?;
        let alt = need(&env.origin_altitude, "origin.altitude")?;
        Ok(Origin { lat_e7: (lat * 1e7).round() as i32, lon_e7: (lon * 1e7).round() as i32, alt_mm: (alt * 1e3).round() as i32 })
    }

    /// The WGS 84 meridian and prime-vertical radii at the origin's latitude, m.
    fn radii(&self) -> (f64, f64) {
        let e2 = WGS84_F * (2.0 - WGS84_F);
        let s = (self.lat_e7 as f64 * 1e-7).to_radians().sin();
        let w = 1.0 - e2 * s * s;
        (WGS84_A * (1.0 - e2) / (w * w.sqrt()), WGS84_A / w.sqrt())
    }
}

const E7_TURN: i64 = 3_600_000_000;

/// Local tangent plane at the origin (the firmware has no conversion yet; seams 10 fixes the form): north over the
/// meridian radius, east over the prime-vertical radius times cos(origin latitude), longitude wrapped to
/// [-180, 180) deg; altitude = origin altitude - down. Rounded to the receiver's units.
pub fn geo_from_ned(o: &Origin, ned_m: Vec3) -> (i32, i32, i32) {
    let (rm, rn) = o.radii();
    let lat0 = (o.lat_e7 as f64 * 1e-7).to_radians();
    let dlat_e7 = (ned_m[0] / rm).to_degrees() * 1e7;
    let dlon_e7 = (ned_m[1] / (rn * lat0.cos())).to_degrees() * 1e7;
    let lat = o.lat_e7 as i64 + dlat_e7.round() as i64;
    let lon = (o.lon_e7 as i64 + dlon_e7.round() as i64 + E7_TURN / 2).rem_euclid(E7_TURN) - E7_TURN / 2;
    let alt = o.alt_mm as i64 - (ned_m[2] * 1e3).round() as i64;
    (lat as i32, lon as i32, alt as i32)
}

/// The inverse of `geo_from_ned`, seams 10's form: i64 differences first, the longitude difference wrapped.
pub fn ned_from_geo(o: &Origin, lat_e7: i32, lon_e7: i32, alt_mm: i32) -> Vec3 {
    let (rm, rn) = o.radii();
    let lat0 = (o.lat_e7 as f64 * 1e-7).to_radians();
    let dlat = lat_e7 as i64 - o.lat_e7 as i64;
    let dlon = (lon_e7 as i64 - o.lon_e7 as i64 + E7_TURN / 2).rem_euclid(E7_TURN) - E7_TURN / 2;
    let rad = |e7: i64| (e7 as f64 * 1e-7).to_radians();
    [rad(dlat) * rm, rad(dlon) * rn * lat0.cos(), -((alt_mm as i64 - o.alt_mm as i64) as f64) * 1e-3]
}

/// Truth at t - latency; north and east errors: a first-order Gauss-Markov term plus white noise at the card's rest
/// scatter (`horizontal_sd`); down error: a Gauss-Markov term; velocity north and east with white noise
/// (`velocity_sd`), turned into speed and course, the course held below `COURSE_HOLD_MS` (none before the first);
/// the deviations of the sample from the card's `horizontal_sd` (latitude and longitude) and `vertical_sd`; fix
/// type 3. Not modelled: the receiver's smoothing and its hold; satellites, DOP and UTC are left `None`.
pub struct GnssModel {
    origin: Origin,
    white_h_m: f64,
    velocity_sd_ms: f64,
    slow_sd_m: Vec3,
    /// Per sample: exp(-period / tau).
    decay: f64,
    slow_m: Vec3,
    std_mm: [u32; 3],
    course_cdeg: Option<u16>,
    pub period_us: u64,
    pub latency_ticks: usize,
    rng: Rng,
    seq: u32,
}

impl GnssModel {
    pub fn new(card: &Card, env: &Env, rate_hz: u32, mut rng: Rng) -> Result<GnssModel, String> {
        if !GNSS_RATES_HZ.contains(&rate_hz) {
            return Err(format!("GNSS rate {rate_hz} Hz: must be 1, 5 or 10"));
        }
        let g = &card.gnss;
        let h = need(&g.slow_horizontal_sd, "gnss.slow_horizontal_sd")?;
        let v = need(&g.slow_vertical_sd, "gnss.slow_vertical_sd")?;
        let tau = need(&g.slow_time_constant, "gnss.slow_time_constant")?;
        let horizontal_sd = need(&g.horizontal_sd, "gnss.horizontal_sd")?;
        let vertical_sd = need(&g.vertical_sd, "gnss.vertical_sd")?;
        let period_us = 1_000_000 / u64::from(rate_hz);
        let slow_sd_m = [h, h, v];
        // The slow error starts in its stationary distribution.
        let slow_m = slow_sd_m.map(|sd| sd * rng.gaussian());
        let mm = |m: f64| (m * 1e3).round() as u32;
        Ok(GnssModel {
            origin: Origin::from_env(env)?,
            white_h_m: horizontal_sd,
            velocity_sd_ms: need(&g.velocity_sd, "gnss.velocity_sd")?,
            slow_sd_m,
            decay: (-(period_us as f64 * 1e-6) / tau).exp(),
            slow_m,
            std_mm: [mm(horizontal_sd), mm(horizontal_sd), mm(vertical_sd)],
            course_cdeg: None,
            period_us,
            latency_ticks: ticks(need(&g.latency, "gnss.latency")?),
            rng,
            seq: 0,
        })
    }

    pub fn origin(&self) -> Origin {
        self.origin
    }

    /// The slow error's present value, NED m (for the tests).
    pub fn slow_m(&self) -> Vec3 {
        self.slow_m
    }

    pub fn sample(&mut self, t_us: u64, past: &Snapshot) -> GnssSample {
        let keep = (1.0 - self.decay * self.decay).sqrt();
        for i in 0..3 {
            self.slow_m[i] = self.decay * self.slow_m[i] + keep * self.slow_sd_m[i] * self.rng.gaussian();
        }
        let p = past.position_m;
        let north = p[0] + self.slow_m[0] + self.white_h_m * self.rng.gaussian();
        let east = p[1] + self.slow_m[1] + self.white_h_m * self.rng.gaussian();
        let down = p[2] + self.slow_m[2];
        let vn = past.velocity_ms[0] + self.velocity_sd_ms * self.rng.gaussian();
        let ve = past.velocity_ms[1] + self.velocity_sd_ms * self.rng.gaussian();
        let speed = vn.hypot(ve);
        if speed >= COURSE_HOLD_MS {
            let cdeg = (ve.atan2(vn).to_degrees().rem_euclid(360.0) * 100.0).round() as u32 % 36_000;
            self.course_cdeg = Some(cdeg as u16);
        }
        let (lat, lon, alt) = geo_from_ned(&self.origin, [north, east, down]);
        let s = GnssSample {
            reading: GnssReading {
                fix_type: 3,
                lat_e7: Some(lat),
                lon_e7: Some(lon),
                alt_mm: Some(alt),
                speed_mm_s: Some((speed * 1e3).round() as u32),
                course_cdeg: self.course_cdeg,
                sats_used: None,
                hdop_x100: None,
                vdop_x100: None,
                time: None,
                date: None,
                lat_std_mm: Some(self.std_mm[0]),
                lon_std_mm: Some(self.std_mm[1]),
                alt_std_mm: Some(self.std_mm[2]),
            },
            t_us,
            seq: self.seq,
        };
        self.seq = self.seq.wrapping_add(1);
        s
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The path to the flight step
// ---------------------------------------------------------------------------------------------------------------------

/// The three models, the truth history their latencies read, and the newest sample of each not yet given to a flight
/// step.
pub struct SlowSensors {
    history: TruthDelay,
    pub baro: BaroModel,
    pub mag: MagModel,
    pub gnss: GnssModel,
    pending: (Option<BaroSample>, Option<MagSample>, Option<GnssSample>),
}

impl SlowSensors {
    /// The runner's: the card as it is, no compass calibration applied (`Calibration::NONE`), own streams of `seed`.
    pub fn new(card: &Card, env: &Env, gnss_rate_hz: u32, seed: u64) -> Result<SlowSensors, String> {
        let baro = BaroModel::new(card, env, Rng::new(seed, STREAM_BARO))?;
        let mag = MagModel::new(card, env, Calibration::NONE, Rng::new(seed, STREAM_MAG))?;
        let gnss = GnssModel::new(card, env, gnss_rate_hz, Rng::new(seed, STREAM_GNSS))?;
        let depth = baro.latency_ticks.max(mag.latency_ticks).max(gnss.latency_ticks);
        Ok(SlowSensors { history: TruthDelay::new(depth), baro, mag, gnss, pending: (None, None, None) })
    }

    /// Once per base tick, after the backend step: records the truth; each device whose period ends at this tick makes a
    /// sample, which replaces any not yet taken (newest only, as the board's triple buffers).
    pub fn tick(&mut self, truth: &Truth) {
        self.history.push(Snapshot::of(truth));
        let t = truth.t_us;
        if t.is_multiple_of(BARO_PERIOD_US) {
            self.pending.0 = Some(self.baro.sample(t, &self.history.ago(self.baro.latency_ticks)));
        }
        if t.is_multiple_of(MAG_PERIOD_US) {
            self.pending.1 = Some(self.mag.sample(t, &self.history.ago(self.mag.latency_ticks)));
        }
        if t.is_multiple_of(self.gnss.period_us) {
            self.pending.2 = Some(self.gnss.sample(t, &self.history.ago(self.gnss.latency_ticks)));
        }
    }

    /// For one flight step: each sample made since the last call, once (`None` when there is none).
    pub fn take(&mut self) -> (Option<BaroSample>, Option<MagSample>, Option<GnssSample>) {
        (self.pending.0.take(), self.pending.1.take(), self.pending.2.take())
    }
}
