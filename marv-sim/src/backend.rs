//! The backend: integrates a wrench into truth. It never computes forces.

use crate::math::{Mat3, Vec3, cross, mat_inv, mat_vec, quat_mul, quat_norm, rot_matrix};
use crate::plant::Wrench;
use marv_card::Card;

/// The true state. World = NED, body = FRD.
#[derive(Debug, Clone, PartialEq)]
pub struct Truth {
    pub t_us: u64,
    /// NED, from the origin.
    pub position_m: [f64; 3],
    /// NED.
    pub velocity_ms: [f64; 3],
    /// Body to world `[w, x, y, z]`, unit, `w >= 0`.
    pub attitude: [f64; 4],
    /// Body rates.
    pub rates_rads: [f64; 3],
    /// NED acceleration of the centre of mass at the end of the last step.
    pub accel_ms2: [f64; 3],
    /// Body angular acceleration at the end of the last step.
    pub ang_accel_rads2: [f64; 3],
    /// True while the ground holds the vehicle (state GROUND). Always false without a ground plane.
    pub on_ground: bool,
    /// What the ground supplies in the last step; zero in the air.
    pub reaction: Reaction,
}

/// The contact reaction of the last step: minus the applied wrench, as far as the ground supplies it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Reaction {
    /// Normal force N, up on the vehicle: the down component of the applied force in NED, >= 0.
    pub normal_n: f64,
    /// Horizontal force on the vehicle, NED north and east: minus the applied force's.
    pub horizontal_n: [f64; 2],
    /// Roll and pitch torque on the vehicle, body x and y: minus the applied torque's.
    pub torque_xy_nm: [f64; 2],
}

/// The state just before a touchdown hold: what the touchdown checks judge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Touchdown {
    pub t_us: u64,
    /// NED, at the end of the touchdown tick, before the hold zeroed it.
    pub velocity_ms: [f64; 3],
    /// Angle between body z and world down, rad, before the hold (the hold keeps the attitude).
    pub tilt_rad: f64,
    /// Body rates before the hold zeroed them.
    pub rates_rads: [f64; 3],
}

/// Where the vehicle starts, and whether the ground plane at down = 0 (centre of mass) exists.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Start {
    /// No ground plane: the vehicle is never caught or held.
    Air,
    /// The ground plane exists; the vehicle starts in the air at the initial state.
    AirOverGround,
    /// The ground plane exists; the vehicle starts held on it at the initial north and east, down = 0, at rest, with
    /// this attitude (Z-Y-X roll and pitch, then yaw, rad).
    Ground { roll_pitch_rad: [f64; 2], yaw_rad: f64 },
}

/// Integrates a wrench into truth. The plant never integrates; a backend never computes forces.
pub trait Backend {
    fn truth(&self) -> &Truth;
    /// Holds `wrench` constant in the body frame for `dt_us` and advances the state.
    fn step(&mut self, wrench: &Wrench, dt_us: u64) -> &Truth;
}

pub struct NativeBackend {
    mass: f64,
    inertia: Mat3,
    inertia_inv: Mat3,
    truth: Truth,
    plane: bool,
    touchdown: Option<Touchdown>,
}

/// The integrated state as one flat array: position 0..3, velocity 3..6, quaternion 6..10, rates 10..13.
type State = [f64; 13];

impl NativeBackend {
    /// The vehicle starts in the air and there is no ground plane.
    pub fn new(card: &Card, initial: Truth) -> Result<Self, String> {
        Self::with_start(card, initial, Start::Air)
    }

    pub fn with_start(card: &Card, mut initial: Truth, start: Start) -> Result<Self, String> {
        if let Start::Ground { roll_pitch_rad, yaw_rad } = start {
            // Yaw, then pitch, then roll (conventions 7c): q = qz(yaw) * qy(pitch) * qx(roll).
            let [roll, pitch, yaw] = [roll_pitch_rad[0], roll_pitch_rad[1], yaw_rad].map(|a| a * 0.5);
            let qx = [roll.cos(), roll.sin(), 0.0, 0.0];
            let qy = [pitch.cos(), 0.0, pitch.sin(), 0.0];
            let qz = [yaw.cos(), 0.0, 0.0, yaw.sin()];
            let q = quat_mul(qz, quat_mul(qy, qx));
            let sign = if q[0] < 0.0 { -1.0 } else { 1.0 };
            initial.attitude = q.map(|x| sign * x);
            initial.position_m[2] = 0.0;
            initial.velocity_ms = [0.0; 3];
            initial.rates_rads = [0.0; 3];
            initial.accel_ms2 = [0.0; 3];
            initial.ang_accel_rads2 = [0.0; 3];
            initial.on_ground = true;
        }
        let mass = card.mass.mass.v.ok_or("mass.mass is \"ask\", the backend needs a value")?;
        let inertia = card.mass.inertia.v.ok_or("mass.inertia is \"ask\", the backend needs a value")?;
        if (quat_norm(initial.attitude) - 1.0).abs() > 1e-9 {
            return Err(format!("initial attitude {:?} is not a unit quaternion", initial.attitude));
        }
        Ok(NativeBackend {
            mass,
            inertia,
            inertia_inv: mat_inv(&inertia),
            truth: initial,
            plane: start != Start::Air,
            touchdown: None,
        })
    }

    /// The most recent touchdown, if any: the state before its hold.
    pub fn touchdown(&self) -> Option<&Touchdown> {
        self.touchdown.as_ref()
    }

    /// `J^-1 * (tau - w x (J w))`
    fn ang_accel(&self, rates: Vec3, torque: Vec3) -> Vec3 {
        let spin = cross(rates, mat_vec(&self.inertia, rates));
        mat_vec(&self.inertia_inv, [torque[0] - spin[0], torque[1] - spin[1], torque[2] - spin[2]])
    }

    /// `R(q) * F / m`
    fn accel(&self, q: [f64; 4], force: Vec3) -> Vec3 {
        let a = mat_vec(&rot_matrix(q), force);
        [a[0] / self.mass, a[1] / self.mass, a[2] / self.mass]
    }

    /// Time derivative of the state under the held wrench.
    fn derivative(&self, s: &State, wrench: &Wrench) -> State {
        let q = [s[6], s[7], s[8], s[9]];
        let rates = [s[10], s[11], s[12]];
        let dq = quat_mul(q, [0.0, rates[0], rates[1], rates[2]]);
        let dv = self.accel(q, wrench.force_n);
        let dw = self.ang_accel(rates, wrench.torque_nm);
        [
            s[3], s[4], s[5],
            dv[0], dv[1], dv[2],
            0.5 * dq[0], 0.5 * dq[1], 0.5 * dq[2], 0.5 * dq[3],
            dw[0], dw[1], dw[2],
        ]
    }
}

/// `s + k * h`
fn advance(s: &State, k: &State, h: f64) -> State {
    let mut out = *s;
    for i in 0..13 {
        out[i] += k[i] * h;
    }
    out
}

/// The reaction that holds the vehicle: minus the applied wrench. `force_ned` is the applied force in NED.
fn hold_reaction(force_ned: Vec3, wrench: &Wrench) -> Reaction {
    Reaction {
        normal_n: force_ned[2],
        horizontal_n: [-force_ned[0], -force_ned[1]],
        torque_xy_nm: [-wrench.torque_nm[0], -wrench.torque_nm[1]],
    }
}

impl Backend for NativeBackend {
    fn truth(&self) -> &Truth {
        &self.truth
    }

    fn step(&mut self, wrench: &Wrench, dt_us: u64) -> &Truth {
        if self.truth.on_ground {
            // GROUND: lift off at the start of the tick when the applied force (weight included) points up.
            let force_ned = mat_vec(&rot_matrix(self.truth.attitude), wrench.force_n);
            if force_ned[2] >= 0.0 {
                self.truth.t_us += dt_us;
                self.truth.velocity_ms = [0.0; 3];
                self.truth.rates_rads = [0.0; 3];
                self.truth.accel_ms2 = [0.0; 3];
                self.truth.ang_accel_rads2 = [0.0; 3];
                self.truth.reaction = hold_reaction(force_ned, wrench);
                return &self.truth;
            }
            // AIR again: this tick integrates normally.
            self.truth.on_ground = false;
            self.truth.reaction = Reaction::default();
        }
        let t = &self.truth;
        let s: State = [
            t.position_m[0], t.position_m[1], t.position_m[2],
            t.velocity_ms[0], t.velocity_ms[1], t.velocity_ms[2],
            t.attitude[0], t.attitude[1], t.attitude[2], t.attitude[3],
            t.rates_rads[0], t.rates_rads[1], t.rates_rads[2],
        ];

        // Classic RK4.
        let h = dt_us as f64 * 1e-6;
        let k1 = self.derivative(&s, wrench);
        let k2 = self.derivative(&advance(&s, &k1, 0.5 * h), wrench);
        let k3 = self.derivative(&advance(&s, &k2, 0.5 * h), wrench);
        let k4 = self.derivative(&advance(&s, &k3, h), wrench);
        let mut n = s;
        for i in 0..13 {
            n[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }

        // Keep q unit and in the w >= 0 half (q and -q are the same rotation).
        let len = quat_norm([n[6], n[7], n[8], n[9]]);
        let sign = if n[6] < 0.0 { -1.0 } else { 1.0 };
        let q = [sign * n[6] / len, sign * n[7] / len, sign * n[8] / len, sign * n[9] / len];
        let rates = [n[10], n[11], n[12]];

        if self.plane && n[2] >= 0.0 {
            // Touchdown at the end of the tick: record the state before the hold, then hold.
            let rot = rot_matrix(q);
            self.touchdown = Some(Touchdown {
                t_us: self.truth.t_us + dt_us,
                velocity_ms: [n[3], n[4], n[5]],
                tilt_rad: rot[2][2].clamp(-1.0, 1.0).acos(),
                rates_rads: rates,
            });
            let force_ned = mat_vec(&rot, wrench.force_n);
            self.truth = Truth {
                t_us: self.truth.t_us + dt_us,
                position_m: [n[0], n[1], 0.0],
                velocity_ms: [0.0; 3],
                attitude: q,
                rates_rads: [0.0; 3],
                accel_ms2: [0.0; 3],
                ang_accel_rads2: [0.0; 3],
                on_ground: true,
                reaction: if force_ned[2] >= 0.0 { hold_reaction(force_ned, wrench) } else { Reaction::default() },
            };
            return &self.truth;
        }

        let accel_ms2 = self.accel(q, wrench.force_n);
        let ang_accel_rads2 = self.ang_accel(rates, wrench.torque_nm);
        self.truth = Truth {
            t_us: self.truth.t_us + dt_us,
            position_m: [n[0], n[1], n[2]],
            velocity_ms: [n[3], n[4], n[5]],
            attitude: q,
            rates_rads: rates,
            accel_ms2,
            ang_accel_rads2,
            on_ground: false,
            reaction: Reaction::default(),
        };
        &self.truth
    }
}
