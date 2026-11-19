//! The three plant invariants, on the real card and environment, at the 125 us tick.
//! The tolerances are numerical, not vehicle numbers.

use marv_card::{Card, Env, Spin};
use marv_sim::math::{mat_t_vec, rot_matrix};
use marv_sim::{Backend, NativeBackend, Plant, Truth, Wrench};

const TICK_US: u64 = 125;
const ONE_SECOND: u64 = 1_000_000 / TICK_US;
const NO_WIND: [f64; 3] = [0.0; 3];

fn load() -> (Card, Env) {
    let root = env!("CARGO_MANIFEST_DIR");
    let card = Card::load(format!("{root}/../marv-card/marv.toml")).expect("card");
    let env = Env::load(format!("{root}/../env/default.toml")).expect("env");
    (card, env)
}

/// Level and at rest at the origin.
fn at_rest() -> Truth {
    Truth {
        t_us: 0,
        position_m: [0.0; 3],
        velocity_ms: [0.0; 3],
        attitude: [1.0, 0.0, 0.0, 0.0],
        rates_rads: [0.0; 3],
        accel_ms2: [0.0; 3],
        ang_accel_rads2: [0.0; 3],
    }
}

/// Rotor speed at which four equal rotors carry the weight.
fn hover_speed(card: &Card, env: &Env) -> f64 {
    let mass = card.mass.mass.v.unwrap();
    let g = env.gravity.v.unwrap();
    let k = card.propulsion.thrust_coefficient.v.unwrap();
    (mass * g / (4.0 * k)).sqrt()
}

/// The wrench for rotors held exactly at `speeds`. With dt = 0 the lag factor is exactly 0, so the speeds do not
/// move and the levels do not matter. An integer level cannot hold an arbitrary speed, so the tests use this.
fn wrench_at(plant: &mut Plant, speeds: [f64; 4], truth: &Truth) -> Wrench {
    plant.set_rotor_speeds(speeds);
    plant.step([0; 4], truth, NO_WIND, 0)
}

#[test]
fn free_fall_accelerates_at_g() {
    let (card, env) = load();
    let g = env.gravity.v.unwrap();
    let mut plant = Plant::new(&card, &env).unwrap();
    let mut backend = NativeBackend::new(&card, at_rest()).unwrap();

    for _ in 0..ONE_SECOND {
        let wrench = plant.step([0; 4], backend.truth(), NO_WIND, TICK_US);
        backend.step(&wrench, TICK_US);
    }

    let t = ONE_SECOND as f64 * TICK_US as f64 * 1e-6;
    let truth = backend.truth();
    for i in 0..3 {
        let want = if i == 2 { g } else { 0.0 };
        assert!((truth.accel_ms2[i] - want).abs() < 1e-9, "accel {:?}", truth.accel_ms2);
    }
    assert!((truth.velocity_ms[2] - g * t).abs() < 1e-9 * g * t, "v {:?}", truth.velocity_ms);
    assert!((truth.position_m[2] - 0.5 * g * t * t).abs() < 1e-9 * 0.5 * g * t * t, "p {:?}", truth.position_m);
    // Specific force: what an accelerometer would read. Free fall reads nothing.
    let rot = rot_matrix(truth.attitude);
    let specific = mat_t_vec(&rot, [truth.accel_ms2[0], truth.accel_ms2[1], truth.accel_ms2[2] - g]);
    assert!(specific.iter().all(|x| x.abs() < 1e-9), "specific force {specific:?}");
    assert_eq!(truth.attitude, at_rest().attitude);
}

#[test]
fn hover_thrust_balances_weight() {
    let (card, env) = load();
    let mass = card.mass.mass.v.unwrap();
    let g = env.gravity.v.unwrap();
    let speeds = [hover_speed(&card, &env); 4];
    let mut plant = Plant::new(&card, &env).unwrap();
    let mut backend = NativeBackend::new(&card, at_rest()).unwrap();

    let wrench = wrench_at(&mut plant, speeds, backend.truth());
    assert!(wrench.force_n[2].abs() < 1e-9 * mass * g, "force {:?}", wrench.force_n);

    for _ in 0..ONE_SECOND {
        let wrench = wrench_at(&mut plant, speeds, backend.truth());
        backend.step(&wrench, TICK_US);
    }
    let truth = backend.truth();
    assert!(truth.velocity_ms[2].abs() < 1e-6, "v {:?}", truth.velocity_ms);
    assert!(truth.position_m[2].abs() < 1e-6, "p {:?}", truth.position_m);
}

/// Total thrust equals the weight, one spin pair faster than the other. Returns the first torque_z and the yaw rate
/// after 800 ticks.
fn yaw_run(card: &Card, env: &Env, ccw_pair_faster: bool) -> (f64, f64) {
    let hover = hover_speed(card, env);
    let fast = 1.1 * hover;
    let slow = (2.0 * hover * hover - fast * fast).sqrt();
    let m = &card.motors;
    let mut speeds = [0.0; 4];
    for motor in [m.front_right, m.rear_right, m.front_left, m.rear_left] {
        let is_ccw = motor.spin == Spin::Ccw;
        speeds[motor.dshot_line as usize] = if is_ccw == ccw_pair_faster { fast } else { slow };
    }

    let mut plant = Plant::new(card, env).unwrap();
    let mut backend = NativeBackend::new(card, at_rest()).unwrap();
    let first = wrench_at(&mut plant, speeds, backend.truth()).torque_nm[2];
    for _ in 0..800 {
        let wrench = wrench_at(&mut plant, speeds, backend.truth());
        backend.step(&wrench, TICK_US);
    }
    (first, backend.truth().rates_rads[2])
}

#[test]
fn yaw_torque_sign_matches_rotor_spin() {
    let (card, env) = load();
    let hover = hover_speed(&card, &env);
    let m = &card.motors;
    let r = &card.rotor_position;

    // (a) Each rotor alone. Last two numbers: the wanted sign of the roll torque (right: roll left, -x) and of the
    // pitch torque (front: nose up, +y), from the conventions 7e.
    let rotors = [
        (m.front_right, &r.front_right, -1.0, 1.0),
        (m.rear_right, &r.rear_right, -1.0, -1.0),
        (m.front_left, &r.front_left, 1.0, 1.0),
        (m.rear_left, &r.rear_left, 1.0, -1.0),
    ];
    for (motor, position, roll_sign, pitch_sign) in rotors {
        let mut speeds = [0.0; 4];
        speeds[motor.dshot_line as usize] = hover;
        let mut plant = Plant::new(&card, &env).unwrap();
        let torque = wrench_at(&mut plant, speeds, &at_rest()).torque_nm;
        let spin_sign = if motor.spin == Spin::Ccw { 1.0 } else { -1.0 };
        let arm = position.v.unwrap();
        assert!(torque[2] * spin_sign > 0.0, "line {} yaw {torque:?}", motor.dshot_line);
        assert!(torque[0] * roll_sign > 0.0, "line {} roll {torque:?} arm {arm:?}", motor.dshot_line);
        assert!(torque[1] * pitch_sign > 0.0, "line {} pitch {torque:?} arm {arm:?}", motor.dshot_line);
    }

    // (b) A faster CCW pair yaws right (+z); a faster CW pair yaws left (-z).
    let (torque_z, rate_z) = yaw_run(&card, &env, true);
    assert!(torque_z > 0.0 && rate_z > 0.0, "ccw faster: torque {torque_z} rate {rate_z}");
    let (torque_z, rate_z) = yaw_run(&card, &env, false);
    assert!(torque_z < 0.0 && rate_z < 0.0, "cw faster: torque {torque_z} rate {rate_z}");
}
