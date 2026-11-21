//! Replay of a flown `.BIN` through the flight code's estimators (`marv-replay`): the self-describing log reader and the
//! rebuilding of the `Sensors` stream the flight step saw. Formats: `fw_core::dflog` (record table), board glue
//! `src/flight.rs` and `src/logger.rs` of the firmware.

use fw_core::sample::{
    BaroReading, BaroSample, GnssReading, GnssSample, MagExpected, MagReading, MagSample,
};
use fw_flight::gnc::Sensors;
use fw_flight::imu::{ImuReading, ImuSample};
use std::collections::HashMap;

const HEAD1: u8 = 0xA3;
const HEAD2: u8 = 0x95;
const ID_FMT: u8 = 128;
const FMT_LEN: usize = 89;

/// All the rows of one record type, as raw payload bytes (header stripped), with the column offsets of its format.
pub struct Table {
    pub name: String,
    pub columns: Vec<String>,
    offsets: Vec<usize>,
    /// Payload length of one row.
    len: usize,
    data: Vec<u8>,
}

impl Table {
    pub fn rows(&self) -> usize {
        self.data.len().checked_div(self.len).unwrap_or(0)
    }

    /// Index of the column `name`.
    pub fn col(&self, name: &str) -> Result<usize, String> {
        self.columns
            .iter()
            .position(|c| c == name)
            .ok_or_else(|| format!("record {} has no column {name}", self.name))
    }

    fn at(&self, row: usize, col: usize, n: usize) -> &[u8] {
        let s = row * self.len + self.offsets[col];
        &self.data[s..s + n]
    }

    pub fn u8(&self, row: usize, col: usize) -> u8 {
        self.at(row, col, 1)[0]
    }
    pub fn u16(&self, row: usize, col: usize) -> u16 {
        u16::from_le_bytes(self.at(row, col, 2).try_into().unwrap())
    }
    pub fn i16(&self, row: usize, col: usize) -> i16 {
        i16::from_le_bytes(self.at(row, col, 2).try_into().unwrap())
    }
    pub fn u32(&self, row: usize, col: usize) -> u32 {
        u32::from_le_bytes(self.at(row, col, 4).try_into().unwrap())
    }
    pub fn i32(&self, row: usize, col: usize) -> i32 {
        i32::from_le_bytes(self.at(row, col, 4).try_into().unwrap())
    }
    pub fn f32(&self, row: usize, col: usize) -> f32 {
        f32::from_le_bytes(self.at(row, col, 4).try_into().unwrap())
    }
    pub fn u64(&self, row: usize, col: usize) -> u64 {
        u64::from_le_bytes(self.at(row, col, 8).try_into().unwrap())
    }
    /// A `c` column (i16, value x 100) as the value.
    pub fn centi(&self, row: usize, col: usize) -> f32 {
        f32::from(self.i16(row, col)) / 100.0
    }
    /// A text column (`n`, `N`, `Z`) up to its first NUL.
    pub fn text(&self, row: usize, col: usize) -> String {
        let width = self.offsets.get(col + 1).copied().unwrap_or(self.len) - self.offsets[col];
        let b = self.at(row, col, width);
        let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
        String::from_utf8_lossy(&b[..end]).into_owned()
    }
    /// `TimeUS` of a row (column 0 of every data type).
    pub fn time(&self, row: usize) -> u64 {
        self.u64(row, 0)
    }
}

fn char_size(c: u8) -> Option<usize> {
    Some(match c {
        b'b' | b'B' => 1,
        b'H' | b'h' | b'c' => 2,
        b'i' | b'I' | b'e' | b'f' | b'L' | b'n' => 4,
        b'Q' => 8,
        b'N' => 16,
        b'Z' => 64,
        _ => return None,
    })
}

/// A parsed log: a table per record name.
pub struct Log {
    pub tables: HashMap<String, Table>,
    /// Bytes skipped between records (no header, unknown type).
    pub bad_bytes: usize,
}

fn cstr(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

impl Log {
    /// Reads every record by the FMT records the file starts with (and any later ones).
    pub fn parse(bytes: &[u8]) -> Result<Log, String> {
        let mut by_id: HashMap<u8, Table> = HashMap::new();
        let mut order: Vec<u8> = Vec::new();
        let mut bad = 0usize;
        let mut i = 0usize;
        while i + 3 <= bytes.len() {
            if bytes[i] != HEAD1 || bytes[i + 1] != HEAD2 {
                bad += 1;
                i += 1;
                continue;
            }
            let id = bytes[i + 2];
            if id == ID_FMT {
                if i + FMT_LEN > bytes.len() {
                    break;
                }
                let p = &bytes[i + 3..i + FMT_LEN];
                let (ty, length) = (p[0], usize::from(p[1]));
                let name = cstr(&p[2..6]);
                let fmt = cstr(&p[6..22]);
                let columns: Vec<String> = cstr(&p[22..86]).split(',').map(str::to_owned).collect();
                let mut offsets = Vec::new();
                let mut off = 0;
                for c in fmt.bytes() {
                    offsets.push(off);
                    off += char_size(c).ok_or_else(|| format!("FMT {name}: format character {}", c as char))?;
                }
                if off + 3 != length || columns.len() != offsets.len() {
                    return Err(format!("FMT {name}: length {length} does not match format {fmt}"));
                }
                if !by_id.contains_key(&ty) {
                    order.push(ty);
                }
                by_id.insert(ty, Table { name, columns, offsets, len: off, data: Vec::new() });
                i += FMT_LEN;
                continue;
            }
            match by_id.get_mut(&id) {
                Some(t) if i + 3 + t.len <= bytes.len() => {
                    t.data.extend_from_slice(&bytes[i + 3..i + 3 + t.len]);
                    i += 3 + t.len;
                }
                Some(_) => break,
                None => {
                    bad += 1;
                    i += 1;
                }
            }
        }
        if by_id.is_empty() {
            return Err("no FMT record: not a flight log".into());
        }
        let tables = order.into_iter().filter_map(|id| by_id.remove(&id)).map(|t| (t.name.clone(), t)).collect();
        Ok(Log { tables, bad_bytes: bad })
    }

    pub fn table(&self, name: &str) -> Result<&Table, String> {
        self.tables.get(name).ok_or_else(|| format!("the log has no {name} records"))
    }

    /// The last value of parameter `name` the log's PARM records carry.
    pub fn param(&self, name: &str) -> Option<f32> {
        let t = self.tables.get("PARM")?;
        let (cn, cv) = (t.col("Name").ok()?, t.col("Value").ok()?);
        (0..t.rows()).rev().find(|&r| t.text(r, cn) == name).map(|r| t.f32(r, cv))
    }
}

/// The compass's expected field as `fw_settings::mag::expected` builds it (units converted once: degrees to radians,
/// percent to a fraction): `None` unless the type is 2 and the field above 0.
pub fn mag_expected(cal_type: i32, fld_ut: f32, inc_deg: f32, hlt_str_pct: f32, hlt_inc_deg: f32) -> Option<MagExpected> {
    if fld_ut <= 0.0 || cal_type != 2 {
        return None;
    }
    Some(MagExpected {
        strength_ut: fld_ut,
        inclination_rad: inc_deg.to_radians(),
        tol_strength_frac: hlt_str_pct / 100.0,
        tol_inclination_rad: hlt_inc_deg.to_radians(),
    })
}

/// Where the compass's `expected` came from.
pub enum ExpectedFrom {
    /// The log's PARM records.
    Log,
    /// The command line (the log carries no MAG_CAL_TYPE).
    CommandLine,
}

/// One flight step as the replay sees it.
#[derive(Clone, Copy)]
pub struct Step {
    pub sensors: Sensors,
    /// The gate's armed state after this step (logged MOT.Arm; carried over the filled steps).
    pub armed: bool,
    /// A logged IMU sample (false: filled in a disarmed gap).
    pub real: bool,
}

pub struct Stream {
    pub steps: Vec<Step>,
    pub real: usize,
    pub filled: usize,
    pub imu_period_us: u64,
    pub expected: Option<MagExpected>,
    pub expected_from: ExpectedFrom,
    /// Baro, compass and GNSS samples handed over / present in the log.
    pub handed: [usize; 3],
    pub present: [usize; 3],
}

pub struct StreamOpts {
    /// The nominal IMU period for the fill, us; `None`: the median spacing of consecutive armed samples.
    pub imu_period_us: Option<u64>,
    /// MAG_FLD_UT, MAG_FLD_INC (deg), MAG_HLT_STR (%), MAG_HLT_INC (deg) when the log has no parameters.
    pub expected_cli: Option<[f32; 4]>,
}

struct Slow<T> {
    items: Vec<(u64, T)>,
    next: usize,
}

impl<T: Copy> Slow<T> {
    /// The newest item with time <= `t`, if one is new since the last call.
    fn take(&mut self, t: u64) -> Option<T> {
        let mut got = None;
        while let Some(&(ti, v)) = self.items.get(self.next) {
            if ti > t {
                break;
            }
            got = Some(v);
            self.next += 1;
        }
        got
    }
}

fn median(mut v: Vec<u64>) -> Option<u64> {
    if v.is_empty() {
        return None;
    }
    v.sort_unstable();
    Some(v[v.len() / 2])
}

pub fn build_stream(log: &Log, opts: &StreamOpts) -> Result<Stream, String> {
    let imu = log.table("IMU")?;
    let (ci, cgx, cax, ct) = (imu.col("I")?, imu.col("GyrX")?, imu.col("AccX")?, imu.col("T")?);
    let mot = log.table("MOT")?;
    let carm = mot.col("Arm")?;

    // IMU1 samples, each with its MOT.Arm (same TimeUS: one block of the board's step).
    struct Row {
        t: u64,
        r: ImuReading,
        armed: bool,
    }
    let mut rows: Vec<Row> = Vec::new();
    let mut m = 0usize;
    for k in 0..imu.rows() {
        if imu.u8(k, ci) != 0 {
            continue;
        }
        let t = imu.time(k);
        while m < mot.rows() && mot.time(m) < t {
            m += 1;
        }
        if m >= mot.rows() || mot.time(m) != t {
            return Err(format!("IMU1 sample at {t} us has no MOT record: the log is not in the flight-step layout"));
        }
        let f3 = |c: usize| [imu.f32(k, c), imu.f32(k, c + 1), imu.f32(k, c + 2)];
        rows.push(Row {
            t,
            r: ImuReading { gyro_rads: f3(cgx), accel_ms2: f3(cax), temperature_c: imu.f32(k, ct) },
            armed: mot.u8(m, carm) != 0,
        });
    }
    if rows.is_empty() {
        return Err("no IMU1 samples".into());
    }
    let period = match opts.imu_period_us {
        Some(p) => p,
        None => median(
            rows.windows(2).filter(|w| w[0].armed && w[1].armed).map(|w| w[1].t - w[0].t).collect(),
        )
        .ok_or("no two consecutive armed samples to measure the IMU period from; give --imu-period-us")?,
    };
    if period == 0 {
        return Err("IMU period 0".into());
    }

    // Compass expectation: the log's parameters, else the command line.
    let (expected, expected_from) = match log.param("MAG_CAL_TYPE") {
        Some(kind) => {
            let g = |n: &str| log.param(n);
            let e = (|| {
                mag_expected(kind as i32, g("MAG_FLD_UT")?, g("MAG_FLD_INC")?, g("MAG_HLT_STR")?, g("MAG_HLT_INC")?)
            })();
            (e, ExpectedFrom::Log)
        }
        None => (
            opts.expected_cli.and_then(|[u, i, s, h]| mag_expected(2, u, i, s, h)),
            ExpectedFrom::CommandLine,
        ),
    };

    // Slow samples.
    let mut seq = 0u32;
    let mut baro: Slow<BaroSample> = Slow { items: Vec::new(), next: 0 };
    if let Ok(t) = log.table("BARO") {
        let (cp, ctm) = (t.col("Press")?, t.col("Temp")?);
        for k in 0..t.rows() {
            seq += 1;
            baro.items.push((
                t.time(k),
                BaroSample {
                    reading: BaroReading { pressure_pa: t.f32(k, cp), temperature_c: t.centi(k, ctm) },
                    t_us: t.time(k),
                    seq,
                },
            ));
        }
    }
    let mut seq = 0u32;
    let mut mag: Slow<MagSample> = Slow { items: Vec::new(), next: 0 };
    if let Ok(t) = log.table("MAG") {
        let cx = t.col("MagX")?;
        for k in 0..t.rows() {
            seq += 1;
            let ut = |c: usize| f32::from(t.i16(k, c)) / 10.0;
            mag.items.push((
                t.time(k),
                MagSample {
                    reading: MagReading { field_ut: [ut(cx), ut(cx + 1), ut(cx + 2)], temperature_c: 0.0 },
                    t_us: t.time(k),
                    seq,
                    expected,
                },
            ));
        }
    }
    let mut gnss: Slow<GnssSample> = Slow { items: Vec::new(), next: 0 };
    if let Ok(t) = log.table("GPS") {
        let c = |n: &str| t.col(n);
        let (cst, cns, chd, cla, cln, cal, csp, ccr) =
            (c("Status")?, c("NSats")?, c("HDop")?, c("Lat")?, c("Lng")?, c("Alt")?, c("Spd")?, c("GCrs")?);
        // GPA and GPV by TimeUS (the sample's own time).
        let gpa = log.table("GPA").ok();
        let gpv = log.table("GPV").ok();
        let mut ga: HashMap<u64, usize> = HashMap::new();
        if let Some(a) = gpa {
            for k in 0..a.rows() {
                ga.insert(a.time(k), k);
            }
        }
        let mut gv: HashMap<u64, usize> = HashMap::new();
        if let Some(v) = gpv {
            for k in 0..v.rows() {
                gv.insert(v.time(k), k);
            }
        }
        let mut seq = 0u32;
        for k in 0..t.rows() {
            seq += 1;
            let status = t.u8(k, cst);
            let pos = status >= 2;
            let hd = t.i16(k, chd);
            let hdop = (hd != i16::MAX && hd >= 0).then(|| hd as u16);
            let (mut vdop_x100, mut lat_std, mut lon_std, mut alt_std) = (None, None, None, None);
            if let (Some(a), Some(&r)) = (gpa, ga.get(&t.time(k))) {
                let vd = a.i16(r, a.col("VDop")?);
                vdop_x100 = (vd != i16::MAX && vd >= 0).then(|| vd as u16);
                let (hacc, vacc) = (a.f32(r, a.col("HAcc")?), a.f32(r, a.col("VAcc")?));
                // GPA keeps only sqrt(lat^2 + lon^2): split equally (lat and lon deviations are not recoverable).
                if hacc.is_finite() {
                    let each = (hacc * 1000.0 / std::f32::consts::SQRT_2).round() as u32;
                    lat_std = Some(each);
                    lon_std = Some(each);
                }
                if vacc.is_finite() {
                    alt_std = Some((vacc * 1000.0).round() as u32);
                }
            }
            let vel = match (gpv, gv.get(&t.time(k))) {
                (Some(v), Some(&r)) => {
                    let c0 = v.col("VelN")?;
                    Some([v.i32(r, c0), v.i32(r, c0 + 1), v.i32(r, c0 + 2)])
                }
                _ => None,
            };
            gnss.items.push((
                t.time(k),
                GnssSample {
                    reading: GnssReading {
                        fix_type: status,
                        lat_e7: pos.then(|| t.i32(k, cla)),
                        lon_e7: pos.then(|| t.i32(k, cln)),
                        alt_mm: pos.then(|| t.i32(k, cal).saturating_mul(10)),
                        speed_mm_s: pos.then(|| (t.f32(k, csp) * 1000.0).round() as u32),
                        course_cdeg: pos.then(|| (t.f32(k, ccr) * 100.0).round() as u16),
                        sats_used: Some(t.u8(k, cns)),
                        hdop_x100: hdop,
                        vdop_x100,
                        time: None,
                        date: None,
                        lat_std_mm: lat_std,
                        lon_std_mm: lon_std,
                        alt_std_mm: alt_std,
                        vel_ned_mm_s: vel.filter(|_| pos),
                    },
                    t_us: t.time(k),
                    seq,
                },
            ));
        }
    }
    let present = [baro.items.len(), mag.items.len(), gnss.items.len()];
    let mut handed = [0usize; 3];

    let mut steps: Vec<Step> = Vec::with_capacity(rows.len() * 2);
    let (mut real, mut filled) = (0usize, 0usize);
    let mut push = |t: u64, row: &Row, armed: bool, is_real: bool, steps: &mut Vec<Step>| {
        let b = baro.take(t);
        let m = mag.take(t);
        let g = gnss.take(t);
        handed[0] += usize::from(b.is_some());
        handed[1] += usize::from(m.is_some());
        handed[2] += usize::from(g.is_some());
        steps.push(Step {
            sensors: Sensors {
                imu1: ImuSample { reading: row.r, t_us: t, seq: steps.len() as u32 },
                imu2: None,
                baro: b,
                mag: m,
                gnss: g,
                external: None,
                },
            armed,
            real: is_real,
        });
    };
    for (n, row) in rows.iter().enumerate() {
        push(row.t, row, row.armed, true, &mut steps);
        real += 1;
        if let Some(next) = rows.get(n + 1) {
            // Disarmed, the board logs every 8th sample: the steps between ran with the same values at the nominal period.
            let gap = next.t.saturating_sub(row.t);
            let missing = ((gap + period / 2) / period).saturating_sub(1);
            for k in 1..=missing {
                let t = row.t + k * period;
                if t >= next.t {
                    break;
                }
                // All armed steps are logged: a gap is disarmed (also right after a disarm).
                push(t, row, row.armed && next.armed, false, &mut steps);
                filled += 1;
            }
        }
    }
    Ok(Stream { steps, real, filled, imu_period_us: period, expected, expected_from, handed, present })
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::dflog;

    fn log(records: &[Vec<u8>]) -> Vec<u8> {
        let mut out = vec![0u8; dflog::HEADER_SEQUENCE_LEN];
        let n = dflog::write_header(&mut out);
        out.truncate(n);
        for r in records {
            out.extend_from_slice(r);
        }
        out
    }

    fn rec(f: impl FnOnce(&mut [u8]) -> usize) -> Vec<u8> {
        let mut b = vec![0u8; 255];
        let n = f(&mut b);
        b.truncate(n);
        b
    }

    fn imu_block(t: u64, armed: bool, gx: f32) -> Vec<Vec<u8>> {
        vec![
            rec(|b| dflog::encode_imu(b, t, 0, [gx, 0.0, 0.0], [0.0, 0.0, -9.8], 30.0)),
            rec(|b| dflog::encode_mot(b, t, armed, 1, [0; 4], [0; 4])),
        ]
    }

    #[test]
    fn fills_disarmed_gaps_and_hands_slow_samples_once() {
        let mut r = Vec::new();
        r.push(rec(|b| dflog::encode_parm(b, 0, b"MAG_CAL_TYPE", 2.0, 0.0)));
        for (n, v) in [("MAG_FLD_UT", 50.0), ("MAG_FLD_INC", 60.0), ("MAG_HLT_STR", 10.0), ("MAG_HLT_INC", 10.0)] {
            r.push(rec(|b| dflog::encode_parm(b, 0, n.as_bytes(), v, 0.0)));
        }
        // Armed at 1000 us spacing, then disarmed every 8th: 8000 us gap.
        for k in 0..4u64 {
            r.extend(imu_block(1000 * k, true, 0.1));
        }
        r.extend(imu_block(11_000, false, 0.2));
        r.extend(imu_block(19_000, false, 0.3));
        r.push(rec(|b| dflog::encode_baro(b, 2_500, 0, 101_325.0, 20.0)));
        r.push(rec(|b| dflog::encode_mag(b, 2_600, 0, [100, -200, 300])));
        let l = Log::parse(&log(&r)).unwrap();
        let s = build_stream(&l, &StreamOpts { imu_period_us: None, expected_cli: None }).unwrap();
        assert_eq!(s.imu_period_us, 1000);
        // 6 real; the 3000 -> 11000 gap holds 7 filled steps, 11000 -> 19000 holds 7 more.
        assert_eq!((s.real, s.filled), (6, 14));
        assert_eq!(s.steps.len(), 20);
        assert_eq!(s.steps[4].sensors.imu1.t_us, 4000);
        assert!(!s.steps[4].real);
        assert_eq!(s.steps[4].sensors.imu1.reading.gyro_rads[0], 0.1);
        // The baro sample (2500) goes to the first step at or after it (t = 3000, step 3), the compass (2600) too.
        assert!(s.steps[3].sensors.baro.is_some() && s.steps[3].sensors.mag.is_some());
        assert!(s.steps.iter().filter(|x| x.sensors.baro.is_some()).count() == 1);
        let m = s.steps[3].sensors.mag.unwrap();
        assert_eq!(m.reading.field_ut, [10.0, -20.0, 30.0]);
        let e = m.expected.unwrap();
        assert!((e.strength_ut - 50.0).abs() < 1e-6 && (e.tol_strength_frac - 0.1).abs() < 1e-6);
        assert!(matches!(s.expected_from, ExpectedFrom::Log));
        assert!(s.steps[0].armed && !s.steps[4].armed && !s.steps[11].armed);
    }
}
