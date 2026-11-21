//! The alt-hold verdict on synthetic traces (decision-nav-eskf-20261120, Amendment F, F9, negative control n3): the
//! simulator cannot make the firmware's effective mode ignore coverage, so the verdict's I6 and mode-sequence checks are
//! shown to fail on traces whose mode does.

use marv_sim::verdict::{Sample, Trace, alt_hold};
use marv_sim::Reaction;
use marv_sim::scenario::Scenario;

const STEP_US: u64 = 1250;

/// A flight step at `t_us` holding the target at D = 1.5 m with the truth on it. The vertical flags are down from
/// `drop_us` to `back_us`; the effective mode is Altitude from 0.04 s on, except where `mode_in(t, flags_ok)` says Angle.
fn trace(drop_us: u64, back_us: u64, mode: impl Fn(u64, bool) -> u8) -> Trace {
    let steps = (0..=56_000_000 / STEP_US)
        .map(|k| {
            let t_us = k * STEP_US;
            let flags_ok = t_us < drop_us || t_us >= back_us;
            Sample {
                t_us,
                truth_q: [1.0, 0.0, 0.0, 0.0],
                truth_rates: [0.0; 3],
                truth_accel: [0.0; 3],
                nav_q: [1.0, 0.0, 0.0, 0.0],
                sp_q: [1.0, 0.0, 0.0, 0.0],
                sp_heading_rate: 0.0,
                armed: true,
                flags: Some(0),
                truth_pos: [0.0, 0.0, 1.5],
                truth_vel: [0.0; 3],
                on_ground: false,
                settling: false,
                reaction: Reaction::default(),
                selected: if t_us >= 40_000 { 2 } else { 1 },
                mode: mode(t_us, flags_ok),
                vel_d_valid: flags_ok,
                pos_d_valid: flags_ok,
                nav_d: 1.5,
                target_d: 1.5,
                thrust_n: 8.96,
                finite: true,
                baro_drift_m: 0.0,
            }
        })
        .collect();
    Trace { steps, weight_n: 8.96, ..Trace::default() }
}

fn failing(t: &Trace) -> Vec<String> {
    let scenario = Scenario::load("../scenarios/alt-hold.toml").expect("scenarios/alt-hold.toml");
    let (checks, _) = alt_hold(&scenario.verdict, t);
    checks.into_iter().filter(|c| !c.pass).map(|c| c.name).collect()
}

/// The mode the firmware's rule gives: Altitude from the switch at 0.04 s while the flags are up, else Angle.
fn by_coverage(t_us: u64, flags_ok: bool) -> u8 {
    if t_us >= 40_000 && flags_ok { 2 } else { 1 }
}

#[test]
fn the_firmwares_rule_passes_every_check() {
    assert_eq!(failing(&trace(50_980_000, 51_500_000, by_coverage)), Vec::<String>::new());
}

#[test]
fn n3_a_mode_that_ignores_coverage_fails_i6_and_the_exit() {
    let f = failing(&trace(50_980_000, 51_500_000, |t, _| if t >= 40_000 { 2 } else { 1 }));
    assert!(f.contains(&"alt_i6_altitude_without_coverage_steps".to_string()), "{f:?}");
    assert!(f.contains(&"alt_exit_steps_after_flags_drop".to_string()), "{f:?}");
}

#[test]
fn n3_an_exit_three_steps_late_fails_i6_and_the_exit() {
    let late = |t: u64, ok: bool| if t >= 40_000 && (ok && t < 50_980_000 || t < 50_980_000 + 3 * STEP_US) { 2 } else { 1 };
    let f = failing(&trace(50_980_000, 51_500_000, late));
    assert!(f.contains(&"alt_i6_altitude_without_coverage_steps".to_string()), "{f:?}");
    assert!(f.contains(&"alt_exit_steps_after_flags_drop".to_string()), "{f:?}");
}

#[test]
fn a_mode_that_never_enters_altitude_fails_the_enter_and_the_sequence() {
    let f = failing(&trace(50_980_000, 51_500_000, |_, _| 1));
    assert!(f.contains(&"alt_enter_delay_s".to_string()), "{f:?}");
    assert!(f.contains(&"alt_mode_changes".to_string()), "{f:?}");
}
