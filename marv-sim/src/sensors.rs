//! The IMU model: truth in, the sample the flight step is given out. One `ImuModel` per IMU.

use crate::backend::Truth;
use crate::math::{Vec3, cross, mat_t_vec, rot_matrix};
use crate::rng::Rng;
use fw_flight::imu::{ImuReading, ImuSample};
use marv_card::{Env, ImuProfile, Record};
use std::collections::VecDeque;

/// The IMU period, 10 ticks of 125 us (conventions 6).
pub const IMU_PERIOD_US: u64 = 1250;

pub struct ImuModel {
    position: Vec3,
    gyro_bias: Vec3,
    accel_bias: Vec3,
    gyro_noise_sd: Vec3,
    accel_noise_sd: Vec3,
    gravity: f64,
    /// Ambient only: there is no self-heating model.
    temperature_c: f32,
    /// Readings computed `latency` ago are delivered now; this many IMU ticks.
    latency_ticks: usize,
    /// (gyro, accel) in f64, newest at the back; at most `latency_ticks + 1` long.
    queue: VecDeque<(Vec3, Vec3)>,
    rng: Rng,
    seq: u32,
}

fn need<T: Copy>(record: &Record<T>, key: &str) -> Result<T, String> {
    record.v.ok_or_else(|| format!("{key} is \"ask\", the IMU model needs a value"))
}

impl ImuModel {
    /// `name` is the card table ("imu1"), for the error text. Refuses if a number the model reads is "ask".
    pub fn new(name: &str, profile: &ImuProfile, env: &Env, rng: Rng) -> Result<Self, String> {
        let latency = need(&profile.latency, &format!("{name}.latency"))?;
        let kelvin = need(&env.sea_level_temperature, "atmosphere.sea_level_temperature")?;
        Ok(ImuModel {
            position: need(&profile.position, &format!("{name}.position"))?,
            gyro_bias: need(&profile.gyro_bias, &format!("{name}.gyro_bias"))?,
            accel_bias: need(&profile.accel_bias, &format!("{name}.accel_bias"))?,
            gyro_noise_sd: need(&profile.gyro_noise_sd, &format!("{name}.gyro_noise_sd"))?,
            accel_noise_sd: need(&profile.accel_noise_sd, &format!("{name}.accel_noise_sd"))?,
            gravity: need(&env.gravity, "gravity.g")?,
            temperature_c: (kelvin - 273.15) as f32,
            latency_ticks: (latency * 1e6 / IMU_PERIOD_US as f64).round() as usize,
            queue: VecDeque::new(),
            rng,
            seq: 0,
        })
    }

    /// One sample from `truth`, stamped with the truth's time.
    pub fn sample(&mut self, truth: &Truth) -> ImuSample {
        let w = truth.rates_rads;
        let r = self.position;

        // Specific force at the IMU: the centre of mass's, plus the lever-arm terms.
        let a = truth.accel_ms2;
        let specific = mat_t_vec(&rot_matrix(truth.attitude), [a[0], a[1], a[2] - self.gravity]);
        let tangential = cross(truth.ang_accel_rads2, r);
        let centripetal = cross(w, cross(w, r));

        let mut gyro = [0.0; 3];
        let mut accel = [0.0; 3];
        for i in 0..3 {
            gyro[i] = w[i] + self.gyro_bias[i] + self.gyro_noise_sd[i] * self.rng.gaussian();
        }
        for i in 0..3 {
            accel[i] = specific[i] + tangential[i] + centripetal[i] + self.accel_bias[i]
                + self.accel_noise_sd[i] * self.rng.gaussian();
        }

        // Before `latency_ticks` readings exist, the oldest one is delivered.
        self.queue.push_back((gyro, accel));
        if self.queue.len() > self.latency_ticks + 1 {
            self.queue.pop_front();
        }
        let (gyro, accel) = self.queue[0];

        let sample = ImuSample {
            reading: ImuReading {
                gyro_rads: gyro.map(|x| x as f32),
                accel_ms2: accel.map(|x| x as f32),
                temperature_c: self.temperature_c,
            },
            t_us: truth.t_us,
            seq: self.seq,
        };
        self.seq += 1;
        sample
    }
}
