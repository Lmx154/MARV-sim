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
}

/// The integrated state as one flat array: position 0..3, velocity 3..6, quaternion 6..10, rates 10..13.
type State = [f64; 13];

impl NativeBackend {
    pub fn new(card: &Card, initial: Truth) -> Result<Self, String> {
        let mass = card.mass.mass.v.ok_or("mass.mass is \"ask\", the backend needs a value")?;
        let inertia = card.mass.inertia.v.ok_or("mass.inertia is \"ask\", the backend needs a value")?;
        if (quat_norm(initial.attitude) - 1.0).abs() > 1e-9 {
            return Err(format!("initial attitude {:?} is not a unit quaternion", initial.attitude));
        }
        Ok(NativeBackend { mass, inertia, inertia_inv: mat_inv(&inertia), truth: initial })
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

impl Backend for NativeBackend {
    fn truth(&self) -> &Truth {
        &self.truth
    }

    fn step(&mut self, wrench: &Wrench, dt_us: u64) -> &Truth {
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
        };
        &self.truth
    }
}
