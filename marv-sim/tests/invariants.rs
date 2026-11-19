//! The three plant invariants, on the real card and environment, at the 125 us tick.
//! The tolerances are numerical, not vehicle numbers.

use marv_card::{Card, Env, Spin};
use marv_sim::math::{mat_t_vec, rot_matrix};
use marv_sim::{Backend, NativeBackend, Plant, Reaction, Start, Truth, Wrench};

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
        on_ground: false,
        settling: false,
        reaction: Reaction::default(),
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

/// The IMU model's specific force (sensors.rs): `R^T (a - g)` in the body frame.
fn specific_force(truth: &Truth, g: f64) -> [f64; 3] {
    let a = truth.accel_ms2;
    mat_t_vec(&rot_matrix(truth.attitude), [a[0], a[1], a[2] - g])
}

#[test]
fn ground_contact() {
    let (card, env) = load();
    let mass = card.mass.mass.v.unwrap();
    let g = env.gravity.v.unwrap();
    let k = card.propulsion.thrust_coefficient.v.unwrap();

    // (a) At rest on the ground at idle thrust for 10 s: position and attitude do not move and the accelerometer reads
    // gravity only (a clamp that kept the wrench's acceleration would read free fall).
    let idle = [card.propulsion.idle_level.v.unwrap() as u16; 4];
    let start = Start::Ground { roll_pitch_rad: [0.05, -0.03], yaw_rad: 0.5 };
    let mut plant = Plant::new(&card, &env).unwrap();
    let mut backend = NativeBackend::with_start(&card, at_rest(), start).unwrap();
    let (position, attitude) = (backend.truth().position_m, backend.truth().attitude);
    for _ in 0..10 * ONE_SECOND {
        let wrench = plant.step(idle, backend.truth(), NO_WIND, TICK_US);
        let truth = backend.step(&wrench, TICK_US);
        assert!(truth.on_ground, "left the ground at {} us", truth.t_us);
        assert_eq!(truth.position_m, position);
        assert_eq!(truth.attitude, attitude);
        assert_eq!(truth.velocity_ms, [0.0; 3]);
        assert_eq!(truth.rates_rads, [0.0; 3]);
        let f = specific_force(truth, g);
        let norm = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt();
        assert!((norm - g).abs() < 1e-9, "specific force {f:?} at {} us", truth.t_us);
    }
    // The reaction is minus the applied wrench: N = m g - T cos(tilt) (no drag, no wind).
    let thrust: f64 = plant.rotor_speeds().iter().map(|w| k * w * w).sum();
    let normal = mass * g - thrust * rot_matrix(attitude)[2][2];
    assert!((backend.truth().reaction.normal_n - normal).abs() < 1e-9, "N {:?}", backend.truth().reaction);

    // (b) A thrust ramp on a tilted ground: release on the first tick where T cos(tilt) > m g, not before.
    let tilt = Start::Ground { roll_pitch_rad: [0.3, -0.2], yaw_rad: 0.0 };
    let mut plant = Plant::new(&card, &env).unwrap();
    let mut backend = NativeBackend::with_start(&card, at_rest(), tilt).unwrap();
    let cos_tilt = rot_matrix(backend.truth().attitude)[2][2];
    let held = backend.truth().position_m;
    let ramp_ticks = 2 * ONE_SECOND;
    let mut released = None;
    for i in 1..=ramp_ticks {
        // Total thrust rises from 0 to 1.5 m g over the ramp.
        let speed = (1.5 * mass * g * i as f64 / ramp_ticks as f64 / (4.0 * k)).sqrt();
        let wrench = wrench_at(&mut plant, [speed; 4], backend.truth());
        let net_up = 4.0 * k * speed * speed * cos_tilt > mass * g;
        let truth = backend.step(&wrench, TICK_US);
        assert_eq!(truth.on_ground, !net_up, "tick {i}: T cos(tilt) {} m g", if net_up { ">" } else { "<=" });
        if net_up {
            // The release tick integrates normally: the vehicle is already moving up.
            assert!(truth.position_m[2] < 0.0 && truth.velocity_ms[2] < 0.0, "tick {i}: {truth:?}");
            released = Some(i);
            break;
        }
        assert_eq!(truth.position_m, held);
    }
    let first = released.expect("never released");
    assert!(first > 1, "released at the first tick");

    // (c) Touchdown records the state before the hold. A vehicle with the ground plane falls from 1 m, tilted and
    // turning; its twin without a plane runs the same arithmetic through the plane, so it shows the pre-hold state.
    let falling = Truth {
        position_m: [0.0, 0.0, -1.0],
        attitude: [0.9689, 0.1237, 0.2150, 0.0369],
        rates_rads: [0.3, -0.2, 0.5],
        ..at_rest()
    };
    let norm = falling.attitude.iter().map(|x| x * x).sum::<f64>().sqrt();
    let falling = Truth { attitude: falling.attitude.map(|x| x / norm), ..falling };
    let mut plant_a = Plant::new(&card, &env).unwrap();
    let mut plant_b = Plant::new(&card, &env).unwrap();
    let mut free = NativeBackend::new(&card, falling.clone()).unwrap();
    let mut over_ground = NativeBackend::with_start(&card, falling, Start::AirOverGround).unwrap();
    let mut landed = false;
    for _ in 0..2 * ONE_SECOND {
        let wa = plant_a.step([0; 4], free.truth(), NO_WIND, TICK_US);
        let wb = plant_b.step([0; 4], over_ground.truth(), NO_WIND, TICK_US);
        free.step(&wa, TICK_US);
        over_ground.step(&wb, TICK_US);
        if !over_ground.truth().on_ground {
            assert!(free.truth().position_m[2] < 0.0, "the twin is below the plane before contact");
            assert!(over_ground.touchdown().is_none());
            continue;
        }
        let before = free.truth();
        let td = over_ground.touchdown().expect("on the ground without a touchdown record");
        assert!(before.position_m[2] >= 0.0, "contact when the twin is above the plane");
        assert_eq!(td.t_us, before.t_us);
        assert_eq!(td.velocity_ms, before.velocity_ms);
        assert_eq!(td.rates_rads, before.rates_rads);
        let tilt_before = rot_matrix(before.attitude)[2][2].acos();
        assert!((td.tilt_rad - tilt_before).abs() < 1e-12, "tilt {} vs {tilt_before}", td.tilt_rad);
        assert!(td.velocity_ms[2] > 4.0 && td.rates_rads[0] != 0.0, "{td:?}");
        let held = over_ground.truth();
        assert_eq!(held.position_m[2], 0.0);
        assert_eq!(held.velocity_ms, [0.0; 3]);
        assert_eq!(held.rates_rads, [0.0; 3]);
        assert_eq!(held.attitude, before.attitude);
        landed = true;
        break;
    }
    assert!(landed, "never touched down");
}
