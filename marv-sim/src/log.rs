//! The sim's DataFlash log: the firmware's own records (header, MSG, IMU, and the flight step's NAV, SETP, DMD, MOT)
//! through `fw_core::dflog` and `fw_flight::step::encode_step`, and the sim-only truth (TRU), rotor (ROT) and stick
//! (RCS) records, written by hand with their FMT.

use crate::backend::Truth;
use fw_core::dflog::{self, HEAD1, HEAD2, RecordDef};
use fw_flight::imu::ImuReading;
use fw_flight::step::{StepOut, encode_step};
use std::path::Path;

const ID_TRU: u8 = 200;
const ID_ROT: u8 = 201;
const ID_RCS: u8 = 202;

/// Format characters used by the sim-only records: Q is u64, H is u16, B is u8, f is f32. Same sizes as the firmware's.
fn char_size(c: char) -> usize {
    match c {
        'Q' => 8,
        'H' => 2,
        'B' => 1,
        'f' => 4,
        _ => panic!("sim-only record: unknown format character {c}"),
    }
}

/// A sim-only record type. The length is computed from the format, the one place that says what is in it.
fn sim_def(type_id: u8, name: &'static str, format: &'static str, columns: &'static str) -> RecordDef {
    let length = 3 + format.chars().map(char_size).sum::<usize>();
    RecordDef { type_id, name, format, columns, length: length as u8 }
}

fn tru_def() -> RecordDef {
    sim_def(ID_TRU, "TRU", "Qfffffffffffff", "TimeUS,PN,PE,PD,VN,VE,VD,Q1,Q2,Q3,Q4,P,Q,R")
}

fn rot_def() -> RecordDef {
    sim_def(ID_ROT, "ROT", "QHHHHffff", "TimeUS,L0,L1,L2,L3,W0,W1,W2,W3")
}

/// The sticks as the flight step was given them: one record per RC sample time (decision-first-pair-wiring A6).
fn rcs_def() -> RecordDef {
    sim_def(ID_RCS, "RCS", "QffffBB", "TimeUS,R,P,T,Y,Arm,Lost")
}

/// One column of a sim-only record.
enum Field {
    Q(u64),
    H(u16),
    B(u8),
    F(f32),
}

/// Appends one sim-only record. It checks the fields against the format string and the length, so the FMT written
/// at the top of the file and the bytes written here cannot disagree.
fn put(out: &mut Vec<u8>, def: &RecordDef, fields: &[Field]) {
    let start = out.len();
    out.extend_from_slice(&[HEAD1, HEAD2, def.type_id]);
    let mut letters = String::new();
    for f in fields {
        match f {
            Field::Q(v) => {
                letters.push('Q');
                out.extend_from_slice(&v.to_le_bytes());
            }
            Field::H(v) => {
                letters.push('H');
                out.extend_from_slice(&v.to_le_bytes());
            }
            Field::B(v) => {
                letters.push('B');
                out.push(*v);
            }
            Field::F(v) => {
                letters.push('f');
                out.extend_from_slice(&v.to_le_bytes());
            }
        }
    }
    assert_eq!(letters, def.format, "{}: fields do not match the format", def.name);
    assert_eq!(out.len() - start, def.length as usize, "{}: length does not match the format", def.name);
}

/// How many records of each type have been written (the FMT records are not counted).
#[derive(Debug, Default, Clone, Copy)]
pub struct Counts {
    pub msg: u64,
    pub imu: u64,
    pub tru: u64,
    pub rot: u64,
    /// Flight steps: each wrote one NAV, SETP, DMD and MOT.
    pub step: u64,
    pub rcs: u64,
}

pub struct Log {
    bytes: Vec<u8>,
    pub counts: Counts,
}

impl Log {
    /// The header: the firmware's 11 FMTs, the sim-only TRU and ROT, the four flight-step types, then RCS.
    pub fn with_header() -> Log {
        let mut bytes = vec![0u8; dflog::HEADER_SEQUENCE_LEN];
        let n = dflog::write_header(&mut bytes);
        assert_eq!(n, dflog::HEADER_SEQUENCE_LEN);
        let [nav, setp, dmd, mot] = dflog::FLIGHT_TYPES;
        for def in [tru_def(), rot_def(), nav, setp, dmd, mot, rcs_def()] {
            let mut fmt = [0u8; 128];
            let n = dflog::fmt_record(&def, &mut fmt);
            assert!(n > 0, "FMT does not fit its buffer");
            bytes.extend_from_slice(&fmt[..n]);
        }
        Log { bytes, counts: Counts::default() }
    }

    /// Appends what a firmware encoder wrote into `buf`; 0 bytes would mean the buffer was too small.
    fn append(&mut self, buf: &[u8], n: usize) {
        assert!(n > 0, "record does not fit its buffer");
        self.bytes.extend_from_slice(&buf[..n]);
    }

    /// One fact per record; the text is cut at 64 bytes by the encoder.
    pub fn msg(&mut self, t_us: u64, text: &str) {
        let mut buf = [0u8; 128];
        let n = dflog::encode_msg(&mut buf, t_us, text.as_bytes());
        self.append(&buf, n);
        self.counts.msg += 1;
    }

    /// `inst` is 0 for IMU1 and 1 for IMU2. The values are exactly those the flight step was given.
    pub fn imu(&mut self, t_us: u64, inst: u8, r: &ImuReading) {
        let mut buf = [0u8; 128];
        let n = dflog::encode_imu(&mut buf, t_us, inst, r.gyro_rads, r.accel_ms2, r.temperature_c);
        self.append(&buf, n);
        self.counts.imu += 1;
    }

    pub fn truth(&mut self, truth: &Truth) {
        let mut fields = vec![Field::Q(truth.t_us)];
        let values = truth
            .position_m
            .into_iter()
            .chain(truth.velocity_ms)
            .chain(truth.attitude)
            .chain(truth.rates_rads);
        fields.extend(values.map(|x| Field::F(x as f32)));
        put(&mut self.bytes, &tru_def(), &fields);
        self.counts.tru += 1;
    }

    /// The levels the plant is using and the true rotor speeds, by DShot line.
    pub fn rotors(&mut self, t_us: u64, levels: [u16; 4], speeds_rads: [f64; 4]) {
        let mut fields = vec![Field::Q(t_us)];
        fields.extend(levels.map(Field::H));
        fields.extend(speeds_rads.map(|w| Field::F(w as f32)));
        put(&mut self.bytes, &rot_def(), &fields);
        self.counts.rot += 1;
    }

    /// The four records of one flight step, written by the firmware's own encoder.
    pub fn step(&mut self, out: &StepOut) {
        let mut buf = [0u8; 256];
        let n = encode_step(out, &mut buf);
        self.append(&buf, n);
        self.counts.step += 1;
    }

    /// Sticks in the order of the columns R, P, T, Y; `lost` = 1 inside a lost interval (the sticks are then the last
    /// ones given).
    pub fn rc(&mut self, t_us: u64, roll_pitch_throttle_yaw: [f32; 4], arm: bool, lost: bool) {
        let mut fields = vec![Field::Q(t_us)];
        fields.extend(roll_pitch_throttle_yaw.map(Field::F));
        fields.extend([Field::B(u8::from(arm)), Field::B(u8::from(lost))]);
        put(&mut self.bytes, &rcs_def(), &fields);
        self.counts.rcs += 1;
    }

    /// Creates the parent folder if it is missing.
    pub fn write(&self, path: &Path) -> Result<(), String> {
        if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
            std::fs::create_dir_all(dir).map_err(|e| format!("{}: cannot create: {e}", dir.display()))?;
        }
        std::fs::write(path, &self.bytes).map_err(|e| format!("{}: cannot write: {e}", path.display()))
    }
}
