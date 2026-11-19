//! The plant: motor levels in, one wrench on the body out. It never integrates the body state.

use crate::backend::Truth;
use crate::math::{Vec3, cross, mat_t_vec, rot_matrix};
use marv_card::{Card, Env, Record, Spin};

/// Force and torque on the body, body frame FRD, at the centre of mass. N and N.m.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wrench {
    pub force_n: [f64; 3],
    pub torque_nm: [f64; 3],
}

/// The numbers are copied out of the card and the environment once; rotor arrays are by DShot line.
pub struct Plant {
    mass: f64,
    gravity: f64,
    drag_body: Vec3,
    drag_rotor_linear: f64,
    thrust_coefficient: f64,
    moment_coefficient: f64,
    max_rotor_speed: f64,
    speed_at_level_min: f64,
    level_min: f64,
    level_max: f64,
    time_constant_up: f64,
    time_constant_down: f64,
    rotor_position: [Vec3; 4],
    /// +1 for a CCW rotor, -1 for a CW rotor, seen from above.
    spin_sign: [f64; 4],
    rotor_speed: [f64; 4],
}

/// The value of a record, or an error naming the key when the card says "ask".
fn need<T: Copy>(record: &Record<T>, key: &str) -> Result<T, String> {
    record
        .v
        .ok_or_else(|| format!("{key} is \"ask\", the plant needs a value"))
}

impl Plant {
    /// Refuses if a number the plant reads is "ask". Numbers it does not read are not looked at.
    pub fn new(card: &Card, env: &Env) -> Result<Self, String> {
        let p = &card.propulsion;
        let m = &card.motors;
        let r = &card.rotor_position;
        let mut rotor_position = [[0.0; 3]; 4];
        let mut spin_sign = [0.0; 4];
        let rotors = [
            (m.front_right, &r.front_right, "rotor_position.front_right"),
            (m.rear_right, &r.rear_right, "rotor_position.rear_right"),
            (m.front_left, &r.front_left, "rotor_position.front_left"),
            (m.rear_left, &r.rear_left, "rotor_position.rear_left"),
        ];
        for (motor, position, key) in rotors {
            let line = motor.dshot_line as usize;
            rotor_position[line] = need(position, key)?;
            spin_sign[line] = match motor.spin {
                Spin::Ccw => 1.0,
                Spin::Cw => -1.0,
            };
        }
        Ok(Plant {
            mass: need(&card.mass.mass, "mass.mass")?,
            gravity: need(&env.gravity, "gravity.g")?,
            drag_body: need(&card.drag_body, "drag.body")?,
            drag_rotor_linear: need(&card.drag_rotor_linear, "drag.rotor_linear")?,
            thrust_coefficient: need(&p.thrust_coefficient, "propulsion.thrust_coefficient")?,
            moment_coefficient: need(&p.moment_coefficient, "propulsion.moment_coefficient")?,
            max_rotor_speed: need(&p.max_rotor_speed, "propulsion.max_rotor_speed")?,
            speed_at_level_min: need(&p.speed_at_level_min, "propulsion.speed_at_level_min")?,
            level_min: need(&p.level_min, "propulsion.level_min")?,
            level_max: need(&p.level_max, "propulsion.level_max")?,
            time_constant_up: need(&p.time_constant_up, "propulsion.time_constant_up")?,
            time_constant_down: need(&p.time_constant_down, "propulsion.time_constant_down")?,
            rotor_position,
            spin_sign,
            rotor_speed: [0.0; 4],
        })
    }

    /// One tick: moves the rotor speeds toward the commanded speeds, then returns the wrench for this tick.
    /// `levels` are DShot levels by DShot line, 1..=2000, and 0 means Stop.
    pub fn step(&mut self, levels: [u16; 4], truth: &Truth, wind_ned_ms: [f64; 3], dt_us: u64) -> Wrench {
        let dt = dt_us as f64 * 1e-6;
        let commands = levels.map(|level| self.speed_command(level));
        for (w, command) in self.rotor_speed.iter_mut().zip(commands) {
            let tau = if command > *w { self.time_constant_up } else { self.time_constant_down };
            // Exact for a first-order lag whose command is constant over the tick.
            *w += (command - *w) * (1.0 - (-dt / tau).exp());
        }

        let mut force = [0.0, 0.0, 0.0];
        let mut torque = [0.0, 0.0, 0.0];
        for i in 0..4 {
            let thrust = self.thrust_coefficient * self.rotor_speed[i] * self.rotor_speed[i];
            let push = [0.0, 0.0, -thrust];
            // Thrust torque from the arm, plus the reaction torque of the spinning rotor about z.
            let arm = cross(self.rotor_position[i], push);
            let reaction = self.spin_sign[i] * self.moment_coefficient * thrust;
            for k in 0..3 {
                force[k] += push[k];
                torque[k] += arm[k];
            }
            torque[2] += reaction;
        }

        let rot = rot_matrix(truth.attitude);
        let weight = mat_t_vec(&rot, [0.0, 0.0, self.mass * self.gravity]);
        let air = mat_t_vec(
            &rot,
            [
                truth.velocity_ms[0] - wind_ned_ms[0],
                truth.velocity_ms[1] - wind_ned_ms[1],
                truth.velocity_ms[2] - wind_ned_ms[2],
            ],
        );
        for k in 0..3 {
            force[k] += weight[k] - self.drag_body[k] * air[k] * air[k].abs();
        }
        // Linear rotor drag in body x and y only.
        for k in 0..2 {
            force[k] -= self.drag_rotor_linear * air[k];
        }
        Wrench { force_n: force, torque_nm: torque }
    }

    pub fn rotor_speeds(&self) -> [f64; 4] {
        self.rotor_speed
    }

    /// For the invariant tests and for scenario initial conditions: sets the rotor speeds directly.
    pub fn set_rotor_speeds(&mut self, speeds_rads: [f64; 4]) {
        self.rotor_speed = speeds_rads;
    }

    /// Level 0 is Stop; otherwise linear from (level_min, speed_at_level_min) to (level_max, max_rotor_speed).
    fn speed_command(&self, level: u16) -> f64 {
        if level == 0 {
            return 0.0;
        }
        let span = (level as f64 - self.level_min) / (self.level_max - self.level_min);
        self.speed_at_level_min + span * (self.max_rotor_speed - self.speed_at_level_min)
    }
}
