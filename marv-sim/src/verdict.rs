//! The verdict: the scenario's checks, computed from what the run recorded (decision-first-pair 8 and 13; wiring B11).
//! Angles are Euler Z-Y-X (conventions 7c) from the quaternions, in degrees. A window `[from, to)` holds the flight
//! steps (800 Hz) or ESC ticks (1 kHz) at or after `from` and before `to`. An empty window or a crossing that never
//! happens gives NaN, and NaN fails every check.

use crate::backend::{Reaction, Touchdown};
use crate::pilot::{Ending, PhaseRun, Report};
use crate::scenario::{Pulse, Verdict};

/// One flight step as the checks see it. Quaternions are body to world `[w, x, y, z]`.
pub struct Sample {
    pub t_us: u64,
    pub truth_q: [f64; 4],
    /// Truth body rates, rad/s.
    pub truth_rates: [f64; 3],
    /// Truth NED acceleration of the centre of mass, m/s^2.
    pub truth_accel: [f64; 3],
    pub nav_q: [f64; 4],
    /// `StepOut.setpoint`: its attitude and heading rate (rad/s); identity and 0 for any other kind.
    pub sp_q: [f64; 4],
    pub sp_heading_rate: f64,
    /// `StepOut.armed`.
    pub armed: bool,
    /// The allocator's saturation bits as the MOT/DMD log has them (roll 1, pitch 2, yaw 4, thrust low 8, thrust high
    /// 16); `None` when there was no allocation (the demand was refused).
    pub flags: Option<u8>,
    /// Truth NED position, m, and velocity, m/s.
    pub truth_pos: [f64; 3],
    pub truth_vel: [f64; 3],
    pub on_ground: bool,
    pub settling: bool,
    pub reaction: Reaction,
}

/// A value read from one flight step (degrees or degrees per second).
type Signal = fn(&Sample) -> f64;

/// What a run recorded for the verdict.
#[derive(Default)]
pub struct Trace {
    pub steps: Vec<Sample>,
    /// The levels the plant was given at each ESC tick (0 = Stop), with the tick time.
    pub esc: Vec<(u64, [u16; 4])>,
    /// The times of the RC samples given to the flight step.
    pub rc_us: Vec<u64>,
    /// Phase form only: every pilot report, the phases as they ran, every touchdown (the state before its hold), the
    /// time of every lift-off (the end of the first tick off the ground), the weight m g (N) and whether the plant
    /// runs the card as it is (no spread point, or the nominal one).
    pub pilot: Vec<Report>,
    pub phases: Vec<PhaseRun>,
    pub touchdowns: Vec<Touchdown>,
    pub liftoffs: Vec<u64>,
    pub weight_n: f64,
    pub nominal: bool,
}

/// One `CHECK` line. `limit` carries its comparison, for example `<=1.5000`.
pub struct Check {
    pub name: String,
    pub measured: String,
    pub limit: String,
    pub pass: bool,
}

fn at_most(name: &str, value: f64, limit: f64) -> Check {
    Check { name: name.into(), measured: format!("{value:.4}"), limit: format!("<={limit:.4}"), pass: value <= limit }
}

fn at_least(name: &str, value: f64, limit: f64) -> Check {
    Check { name: name.into(), measured: format!("{value:.4}"), limit: format!(">={limit:.4}"), pass: value >= limit }
}

fn above(name: &str, value: f64, limit: f64) -> Check {
    Check { name: name.into(), measured: format!("{value:.4}"), limit: format!(">{limit:.4}"), pass: value > limit }
}

fn us(s: f64) -> u64 {
    (s * 1e6).round() as u64
}

fn secs(t_us: u64) -> f64 {
    t_us as f64 * 1e-6
}

/// Roll, pitch, yaw in degrees, Z-Y-X.
fn euler_deg(q: [f64; 4]) -> [f64; 3] {
    let [w, x, y, z] = q;
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    [roll, pitch, yaw].map(f64::to_degrees)
}

/// The angle between body z and world down, degrees.
fn tilt_deg(q: [f64; 4]) -> f64 {
    let [_, x, y, _] = q;
    (1.0 - 2.0 * (x * x + y * y)).clamp(-1.0, 1.0).acos().to_degrees()
}

fn wrap_deg(a: f64) -> f64 {
    (a + 180.0).rem_euclid(360.0) - 180.0
}

fn window(trace: &Trace, from_s: f64, to_s: f64) -> impl Iterator<Item = &Sample> {
    let (from, to) = (us(from_s), us(to_s));
    trace.steps.iter().filter(move |s| s.t_us >= from && s.t_us < to)
}

/// The largest value; NaN when there is none, and NaN when any value is NaN, so that one bad sample fails the check
/// instead of being skipped.
fn largest(values: impl Iterator<Item = f64>) -> f64 {
    let mut out = f64::NAN;
    for v in values {
        if v.is_nan() {
            return f64::NAN;
        }
        out = if out.is_nan() { v } else { out.max(v) };
    }
    out
}

/// How far a commanded step may differ from the size the scenario expects, deg or deg/s: float rounding only.
const SETPOINT_TOLERANCE: f64 = 0.01;

fn mean(values: impl Iterator<Item = f64>) -> f64 {
    let (sum, n) = values.fold((0.0, 0), |(s, n), v| (s + v, n + 1));
    sum / n as f64
}

/// The first flight step at or after `s`.
fn sample_at(trace: &Trace, s: f64) -> Option<&Sample> {
    trace.steps.iter().find(|x| x.t_us >= us(s))
}

/// Runs the scenario's check. Returns the CHECK lines and notes that are printed but not judged.
pub fn judge(verdict: &Verdict, trace: &Trace) -> (Vec<Check>, Vec<String>) {
    match *verdict {
        Verdict::Hover { from_s, disturbance_s, settle_from_s, max_tilt_deg, max_nav_gap_deg, max_heading_change_deg,
            max_peak_tilt_deg } => {
            let end = f64::INFINITY;
            let tilt = |from, to| largest(window(trace, from, to).map(|s| tilt_deg(s.truth_q)));
            let gap = largest(window(trace, from_s, end).map(|s| (tilt_deg(s.nav_q) - tilt_deg(s.truth_q)).abs()));
            let checks = vec![
                at_most("tilt_before_disturbance_deg", tilt(from_s, disturbance_s), max_tilt_deg),
                at_most("nav_tilt_gap_deg", gap, max_nav_gap_deg),
                at_most("heading_change_deg", heading_change(trace, from_s), max_heading_change_deg),
                at_most("peak_tilt_disturbed_deg", tilt(disturbance_s, end), max_peak_tilt_deg),
                at_most("tilt_settled_deg", tilt(settle_from_s, end), max_tilt_deg),
            ];
            (checks, Vec::new())
        }
        Verdict::Steps { roll_at_s, pitch_at_s, yaw_at_s, hold_s, step_angle_deg, step_yaw_rate_dps, max_rise_s,
            max_overshoot_rp_pct, max_overshoot_yaw_pct } => {
            let mut checks = Vec::new();
            // (axis, step instant, signal, target, expected step size, overshoot limit)
            let axes: [(&str, f64, Signal, Signal, f64, f64); 3] = [
                ("roll", roll_at_s, |s| euler_deg(s.nav_q)[0], |s| euler_deg(s.sp_q)[0], step_angle_deg,
                    max_overshoot_rp_pct),
                ("pitch", pitch_at_s, |s| euler_deg(s.nav_q)[1], |s| euler_deg(s.sp_q)[1], step_angle_deg,
                    max_overshoot_rp_pct),
                ("yaw", yaw_at_s, |s| s.truth_rates[2].to_degrees(), |s| s.sp_heading_rate.to_degrees(),
                    step_yaw_rate_dps, max_overshoot_yaw_pct),
            ];
            for (axis, at_s, signal, target, size, max_overshoot) in axes {
                for (part, start) in [("step", at_s), ("return", at_s + hold_s)] {
                    let (rise, overshoot, amplitude) = step_response(trace, start, hold_s, signal, target);
                    // The flight code's own setpoint must be the step the scenario asked for: a wrong stick scale
                    // would otherwise be followed faithfully and pass.
                    checks.push(at_most(&format!("{axis}_{part}_size_error"), (amplitude.abs() - size).abs(),
                        SETPOINT_TOLERANCE));
                    checks.push(at_most(&format!("{axis}_{part}_rise_s"), rise, max_rise_s));
                    checks.push(at_most(&format!("{axis}_{part}_overshoot_pct"), overshoot, max_overshoot));
                }
            }
            let during = |i: usize| {
                largest(window(trace, roll_at_s, yaw_at_s + 2.0 * hold_s)
                    .map(|s| (euler_deg(s.truth_q)[i] - euler_deg(s.nav_q)[i]).abs()))
            };
            let note = format!("INFO largest truth-nav gap during the steps: roll {:.3} deg, pitch {:.3} deg (not judged, 8b)",
                during(0), during(1));
            (checks, vec![note])
        }
        Verdict::StickSign { ref pulses, min_move_deg, max_off_axis_ratio, throttle_from_s, throttle_to_s,
            min_accel_rise_ms2 } => {
            let mut checks = Vec::new();
            for p in pulses {
                checks.extend(pulse_checks(trace, p, min_move_deg, max_off_axis_ratio));
            }
            checks.push(at_least("throttle_accel_rise_ms2",
                accel_rise(trace, throttle_from_s, throttle_to_s), min_accel_rise_ms2));
            (checks, Vec::new())
        }
        Verdict::RcLoss { on_high_s, low_s, arm_s, throttle_s, lost_at_s, rc_fs_timeout_s, stop_bound_s, resume_s,
            rearm_s, off_s, off_bound_s } => {
            let idle = [marv_card::IDLE_LEVEL; 4];
            let at_arm = levels_before(trace, throttle_s);
            let at_rearm = levels_before(trace, off_s);
            let t_last = trace.rc_us.iter().copied().filter(|&t| t < us(lost_at_s)).max();
            let loss = t_last.map_or(f64::NAN, |t| first_stop(trace, t).map_or(f64::NAN, |z| secs(z - t)));
            let off = first_stop(trace, us(off_s).saturating_sub(1)).map_or(f64::NAN, |z| secs(z) - off_s);
            let checks = vec![
                stop_check("stop_throttle_high", trace, on_high_s, low_s),
                stop_check("stop_until_cycled", trace, low_s, arm_s),
                levels_check("arm_at_idle_level", at_arm, |l| l == idle, &format!("={idle:?}")),
                above("spin_until_timeout_s", loss, rc_fs_timeout_s),
                at_most("all_zero_by_s", loss, rc_fs_timeout_s + stop_bound_s),
                stop_check("stop_after_resume", trace, resume_s, rearm_s),
                levels_check("rearm_spinning", at_rearm, |l| l.iter().all(|&x| x > 0), "all>0"),
                at_most("stop_after_switch_off_s", off, off_bound_s),
            ];
            (checks, Vec::new())
        }
        Verdict::TakeoffLand { .. } => takeoff_land(verdict, trace),
    }
}

/// The phases the takeoff_land checks name (decision-sim-takeoff-landing, section 3 table).
const IDLE_PHASE: &str = "idle";
const HOVER_PHASE: &str = "hover";
const BAND_PHASES: [&str; 4] = ["hover", "settle", "yaw", "steady"];
const SPEED_PHASES: [&str; 5] = ["hover", "yaw", "steady", "descend", "land"];

/// The index and `[start, end)` of the named phase; an unended phase runs to the end of the trace.
fn phase(trace: &Trace, name: &str) -> Option<(usize, u64, u64)> {
    let i = trace.phases.iter().position(|p| p.name == name)?;
    let p = &trace.phases[i];
    Some((i, p.start_us, p.end.map_or(u64::MAX, |(t, _)| t)))
}

/// Flight steps in `[from_us, to_us)`.
fn between(trace: &Trace, from_us: u64, to_us: u64) -> impl Iterator<Item = &Sample> {
    trace.steps.iter().filter(move |s| s.t_us >= from_us && s.t_us < to_us)
}

/// Flight steps in any of the named phases; `None` when one of them did not run.
fn in_phases<'a>(trace: &'a Trace, names: &[&str]) -> Option<Vec<&'a Sample>> {
    let mut out = Vec::new();
    for n in names {
        let (_, from, to) = phase(trace, n)?;
        out.extend(between(trace, from, to));
    }
    Some(out)
}

/// The angle between the estimate's and the truth's down, degrees.
fn down_gap_deg(a: [f64; 4], b: [f64; 4]) -> f64 {
    let down = |[w, x, y, z]: [f64; 4]| [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)];
    let (u, v) = (down(a), down(b));
    (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]).clamp(-1.0, 1.0).acos().to_degrees()
}

fn count_check(name: &str, bad: Option<usize>) -> Check {
    Check {
        name: name.into(),
        measured: bad.map_or("NaN".into(), |n| n.to_string()),
        limit: "=0".into(),
        pass: bad == Some(0),
    }
}

/// decision-sim-takeoff-landing, section 3: one check per row of its table. Lift-off is the end of the first tick off
/// the ground (backend); touchdown is the first backend contact after the first rise through airborne_m (Amendment B7;
/// a contact between lift-off and that rise is a lift-off recontact); switch-on and switch-off are the first pilot
/// reports with the switch on, then off again.
fn takeoff_land(verdict: &Verdict, trace: &Trace) -> (Vec<Check>, Vec<String>) {
    let Verdict::TakeoffLand { airborne_m, arm_bound_s, r_stance_m, stance_floor_weight, max_stance_ratio,
        max_light_after_td_s, max_peak_height_m, max_peak_climb_ms, liftoff_window_s, max_liftoff_tilt_deg, height_band_m,
        max_tilt_deg, max_speed_h_ms, max_nav_gap_deg, max_td_vz_ms, max_td_vh_ms, max_td_tilt_deg, max_td_rate_dps, idle_after_s, stop_bound_s,
        hover_trim_tolerance } = *verdict else { unreachable!() };
    let idle = [marv_card::IDLE_LEVEL; 4];
    let t_on = trace.pilot.iter().find(|r| r.row.arm).map(|r| r.row.t_us);
    let t_off = t_on.and_then(|on| trace.pilot.iter().find(|r| r.row.t_us > on && !r.row.arm).map(|r| r.row.t_us));
    let t_lo = trace.liftoffs.first().copied();
    let height = |s: &Sample| -s.truth_pos[2];
    let rising = |w: &&[Sample]| height(&w[0]) < airborne_m && height(&w[1]) >= airborne_m;
    let t_rise = trace.steps.windows(2).find(rising).map(|w| w[1].t_us);
    let t_td = t_rise.and_then(|r| trace.touchdowns.iter().find(|d| d.t_us > r)).map(|d| d.t_us);
    let mut checks = Vec::new();

    // stop_disarmed, armed_on_ground, idle_exact.
    checks.push(count_check("stop_disarmed", t_on.and_then(|on| {
        let ticks: Vec<_> = trace.esc.iter().filter(|(t, _)| *t < on).collect();
        (!ticks.is_empty()).then(|| ticks.iter().filter(|(_, l)| *l != [0; 4]).count())
    })));
    let t_spin = t_on.and_then(|on| trace.esc.iter().find(|(t, l)| *t >= on && *l != [0; 4]).map(|(t, _)| *t));
    let armed_delay = match (t_on, t_spin) {
        (Some(on), Some(spin)) => secs(spin - on),
        _ => f64::NAN,
    };
    checks.push(at_most("armed_on_ground_s", armed_delay, arm_bound_s));
    checks.push(count_check("idle_exact", match (t_on, t_spin, phase(trace, IDLE_PHASE)) {
        (Some(on), Some(spin), Some((_, _, end))) if spin < end => {
            let lines = trace.esc.iter().filter(|(t, l)| *t >= spin && *t < end && *l != idle).count();
            Some(lines + between(trace, on, end).filter(|s| !s.on_ground).count())
        }
        _ => None,
    }));

    // stance_margin: armed, in contact, N >= the floor.
    let floor = stance_floor_weight * trace.weight_n;
    let ratio = |s: &Sample| {
        let [tx, ty] = s.reaction.torque_xy_nm;
        (tx * tx + ty * ty).sqrt() / (s.reaction.normal_n * r_stance_m)
    };
    let loaded = |s: &&Sample| s.armed && s.on_ground && s.reaction.normal_n >= floor;
    let stance = largest(trace.steps.iter().filter(loaded).filter(|s| !s.settling).map(ratio));
    let in_settle = largest(trace.steps.iter().filter(loaded).filter(|s| s.settling).map(ratio));
    checks.push(at_most("stance_margin", stance, max_stance_ratio));

    // liftoff_tilt over [lift-off, + window).
    let lift_tilt = t_lo.map_or(f64::NAN, |lo| {
        largest(between(trace, lo, lo + us(liftoff_window_s)).map(|s| tilt_deg(s.truth_q)))
    });
    checks.push(at_most("liftoff_tilt_deg", lift_tilt, max_liftoff_tilt_deg));

    // liftoff_recontacts (Amendment B7): backend contacts between lift-off and the first rise through airborne_m.
    checks.push(count_check("liftoff_recontacts", t_lo.map(|lo| {
        trace.touchdowns.iter().filter(|d| d.t_us >= lo && t_rise.is_none_or(|r| d.t_us < r)).count()
    })));

    // single_flight (Amendment A3(b)): one rise through airborne_m, never above airborne_m after the touchdown; no
    // touchdown at all fails. Backend touchdowns (hops below airborne_m included) are printed only.
    let rises = trace.steps.windows(2).filter(rising).count();
    let above = t_td.map(|td| trace.steps.iter().filter(|s| s.t_us >= td && height(s) > airborne_m).count());
    checks.push(Check {
        name: "single_flight".into(),
        measured: format!("{rises}/{}", above.map_or("none".into(), |n| n.to_string())),
        limit: "=1/0".into(),
        pass: rises == 1 && above == Some(0),
    });

    // Climb (Amendment B4): the largest height and climb speed from lift-off to touchdown.
    let flight: Vec<&Sample> = t_lo.map_or(Vec::new(), |lo| between(trace, lo, t_td.unwrap_or(u64::MAX)).collect());
    let peak = |f: fn(&Sample) -> f64| if flight.is_empty() { f64::NAN } else { largest(flight.iter().map(|s| f(s))) };
    checks.push(at_most("peak_height_m", peak(|s| -s.truth_pos[2]), max_peak_height_m));
    checks.push(at_most("peak_climb_ms", peak(|s| -s.truth_vel[2]), max_peak_climb_ms));

    // light_after_td_s (Amendment A3(b)): from the touchdown until N >= the floor for good, hops included.
    let light = t_td.map_or(f64::NAN, |td| {
        let after: Vec<&Sample> = trace.steps.iter().filter(|s| s.t_us >= td).collect();
        let last_light = after.iter().rposition(|s| !(s.on_ground && s.reaction.normal_n >= floor));
        match last_light {
            None => 0.0,
            Some(i) => after.get(i + 1).map_or(f64::NAN, |s| secs(s.t_us - td)),
        }
    });
    checks.push(at_most("light_after_td_s", light, max_light_after_td_s));

    // no_timeout: no phase timed out, and the table ran to its end.
    let timeouts = trace.phases.iter().filter(|p| !matches!(p.end, Some((_, Ending::Time | Ending::Event)))).count();
    checks.push(count_check("no_timeout", (!trace.phases.is_empty()).then_some(timeouts)));

    // height_band: abs(h - h_ref) in the named phases, h_ref from the pilot's own reference in that phase.
    let band = largest(BAND_PHASES.iter().flat_map(|n| {
        let found = phase(trace, n);
        let h_ref = found.and_then(|(i, _, _)| trace.pilot.iter().find(|r| r.phase == i && r.mode == 2))
            .map_or(f64::NAN, |r| r.reference);
        let samples: Vec<f64> = found.map_or(vec![f64::NAN], |(_, from, to)| {
            between(trace, from, to).map(|s| (height(s) - h_ref).abs()).collect()
        });
        samples
    }));
    checks.push(at_most("height_band_m", band, height_band_m));

    checks.push(at_most("tilt_max_deg", largest(trace.steps.iter().filter(|s| !s.on_ground).map(|s| tilt_deg(s.truth_q))),
        max_tilt_deg));
    let speed = in_phases(trace, &SPEED_PHASES).map_or(f64::NAN, |v| {
        largest(v.into_iter().map(|s| s.truth_vel[0].hypot(s.truth_vel[1])))
    });
    checks.push(at_most("speed_h_ms", speed, max_speed_h_ms));
    checks.push(at_most("nav_gap_deg", largest(trace.steps.iter().filter(|s| s.armed).map(|s| down_gap_deg(s.nav_q, s.truth_q))),
        max_nav_gap_deg));

    // no_saturation from lift-off to touchdown; a refused demand counts as a flag.
    checks.push(count_check("no_saturation", match (t_lo, t_td) {
        (Some(lo), Some(td)) => Some(between(trace, lo, td).filter(|s| s.flags != Some(0)).count()),
        _ => None,
    }));

    // Touchdown: the worst over every contact (lift-off recontacts included), the state before the hold.
    let td = |f: fn(&Touchdown) -> f64| {
        if trace.touchdowns.is_empty() { f64::NAN } else { largest(trace.touchdowns.iter().map(f)) }
    };
    checks.push(at_most("td_vz_ms", td(|d| d.velocity_ms[2].abs()), max_td_vz_ms));
    checks.push(at_most("td_vh_ms", td(|d| d.velocity_ms[0].hypot(d.velocity_ms[1])), max_td_vh_ms));
    checks.push(at_most("td_tilt_deg", td(|d| d.tilt_rad.to_degrees()), max_td_tilt_deg));
    checks.push(at_most("td_rate_dps", td(|d| {
        let [p, q, r] = d.rates_rads;
        (p * p + q * q + r * r).sqrt().to_degrees()
    }), max_td_rate_dps));

    // idle_after: from the first throttle-0 report after touchdown, + idle_after_s, to switch-off.
    let t_zero = t_td.and_then(|td| trace.pilot.iter().find(|r| r.row.t_us >= td && r.row.throttle == 0.0)).map(|r| r.row.t_us);
    checks.push(count_check("idle_after", match (t_zero, t_off) {
        (Some(z), Some(off)) => {
            let ticks: Vec<_> = trace.esc.iter().filter(|(t, _)| *t >= z + us(idle_after_s) && *t < off).collect();
            (!ticks.is_empty()).then(|| ticks.iter().filter(|(_, l)| *l != idle).count())
        }
        _ => None,
    }));

    // stop_after: the first ESC tick from which every line stays 0 to the end, after switch-off.
    let stop = t_off.map_or(f64::NAN, |off| {
        let last_spin = trace.esc.iter().rposition(|(_, l)| *l != [0; 4]);
        let from = last_spin.map_or(0, |i| i + 1);
        trace.esc[from..].iter().find(|(t, _)| *t >= off).map_or(f64::NAN, |(t, _)| secs(*t - off))
    });
    checks.push(at_most("stop_after_s", stop, stop_bound_s));

    // hover_trim: the pilot's mean thrust ratio in hover, judged at the nominal cell only.
    let trim = phase(trace, HOVER_PHASE).map_or(f64::NAN, |(i, _, _)| {
        mean(trace.pilot.iter().filter(|r| r.phase == i).map(|r| r.ratio))
    });
    let mut notes = Vec::new();
    if trace.nominal {
        checks.push(Check {
            name: "hover_trim".into(),
            measured: format!("{trim:.4}"),
            limit: format!("1+/-{hover_trim_tolerance:.4}"),
            pass: (trim - 1.0).abs() <= hover_trim_tolerance,
        });
    } else {
        notes.push(format!("INFO hover_trim {trim:.4} (judged at the nominal cell only)"));
    }
    let held: Vec<&Report> = trace.pilot.iter().filter(|r| r.xy != [0.0; 2]).collect();
    let xy = |r: &&Report| r.xy[0].hypot(r.xy[1]);
    notes.push(format!("INFO pilot xy correction: largest {:.4}, mean {:.4} of stick (cap at 0.25 = 7.5 deg)",
        largest(held.iter().map(xy)), mean(held.iter().map(xy))));
    notes.push(format!("INFO pilot xy integral term: largest {:.4} of stick (before the cap)",
        largest(trace.pilot.iter().map(|r| r.xy_trim[0].hypot(r.xy_trim[1])))));
    notes.push(format!("INFO stance_in_settle {in_settle:.4} (largest stance ratio skipped while the ground settle runs)"));
    notes.push(format!("INFO lift-off {:.4} s, touchdown {:.4} s, backend touchdowns {}",
        t_lo.map_or(f64::NAN, secs), t_td.map_or(f64::NAN, secs), trace.touchdowns.len()));
    (checks, notes)
}

/// Hover: the largest change of truth heading from its value at the first step at or after `from_s`, to the end.
fn heading_change(trace: &Trace, from_s: f64) -> f64 {
    let Some(first) = sample_at(trace, from_s) else { return f64::NAN };
    let y0 = euler_deg(first.truth_q)[2];
    largest(window(trace, from_s, f64::INFINITY).map(|s| wrap_deg(euler_deg(s.truth_q)[2] - y0).abs()))
}

/// Rise time (s), overshoot (%) and the commanded amplitude of one step of `signal` toward `target`, over the hold
/// window `[at, at + hold)`.
/// Amplitude: the target at the window's last step minus the target at the last step before `at`.
/// Rise time: the time between the first crossing of 10 % and the first crossing of 90 % of the amplitude (measured
/// from the target before the step), both searched from the step instant.
/// Overshoot: the largest excursion of `signal` beyond the target (in the step's direction) within the window, as a
/// percentage of the amplitude; 0 if it never passes the target.
fn step_response(trace: &Trace, at_s: f64, hold_s: f64, signal: Signal, target: Signal) -> (f64, f64, f64) {
    let before = trace.steps.iter().rfind(|s| s.t_us < us(at_s));
    let held: Vec<&Sample> = window(trace, at_s, at_s + hold_s).collect();
    let (Some(before), Some(last)) = (before, held.last()) else { return (f64::NAN, f64::NAN, f64::NAN) };
    let (base, goal) = (target(before), target(last));
    let amplitude = goal - base;
    let dir = amplitude.signum();
    let crossing = |fraction: f64| {
        let level = base + fraction * amplitude;
        held.iter().find(|s| (signal(s) - level) * dir >= 0.0).map_or(f64::NAN, |s| secs(s.t_us))
    };
    let rise = crossing(0.9) - crossing(0.1);
    // `f64::max` would turn a NaN into 0 here, so the NaN case is kept apart.
    let beyond = largest(held.iter().map(|s| (signal(s) - goal) * dir));
    let overshoot = if beyond.is_nan() { f64::NAN } else { 100.0 * beyond.max(0.0) / amplitude.abs() };
    (rise, overshoot, amplitude)
}

/// Stick sign: the change of truth roll, pitch and yaw from the pulse's start to its end. The primary axis must
/// move by at least `min_move_deg` in the pulse's sense; each other axis by at most `ratio` of the primary's move.
fn pulse_checks(trace: &Trace, p: &Pulse, min_move_deg: f64, ratio: f64) -> [Check; 2] {
    let name = format!("{}{}", ["roll", "pitch", "yaw"][p.axis], if p.sense > 0.0 { "+" } else { "-" });
    let change = match (sample_at(trace, p.from_s), sample_at(trace, p.to_s)) {
        (Some(a), Some(b)) => {
            let (ea, eb) = (euler_deg(a.truth_q), euler_deg(b.truth_q));
            [0, 1, 2].map(|i| wrap_deg(eb[i] - ea[i]))
        }
        _ => [f64::NAN; 3],
    };
    let primary = change[p.axis];
    let off_axis = largest((0..3).filter(|&i| i != p.axis).map(|i| change[i].abs()));
    [
        at_least(&format!("{name}_move_deg"), primary * p.sense, min_move_deg),
        at_most(&format!("{name}_off_axis_deg"), off_axis, ratio * primary.abs()),
    ]
}

/// Throttle pulse: the mean upward truth acceleration (-accel down) over `[from, to)` minus its mean over the same
/// length just before `from`.
fn accel_rise(trace: &Trace, from_s: f64, to_s: f64) -> f64 {
    let up = |a: f64, b: f64| mean(window(trace, a, b).map(|s| -s.truth_accel[2]));
    up(from_s, to_s) - up(from_s - (to_s - from_s), from_s)
}

/// RC loss: the number of ESC ticks in `[from, to]` at which the plant was given any level other than 0.
fn stop_check(name: &str, trace: &Trace, from_s: f64, to_s: f64) -> Check {
    let n = trace.esc.iter().filter(|(t, l)| *t >= us(from_s) && *t <= us(to_s) && l.iter().any(|&x| x > 0)).count();
    let any = trace.esc.iter().any(|(t, _)| *t >= us(from_s) && *t <= us(to_s));
    Check { name: name.into(), measured: n.to_string(), limit: "=0".into(), pass: any && n == 0 }
}

/// The levels the plant was given at the last ESC tick before `s`.
fn levels_before(trace: &Trace, s: f64) -> Option<[u16; 4]> {
    trace.esc.iter().rfind(|(t, _)| *t < us(s)).map(|(_, l)| *l)
}

fn levels_check(name: &str, levels: Option<[u16; 4]>, ok: impl Fn([u16; 4]) -> bool, limit: &str) -> Check {
    Check {
        name: name.into(),
        measured: levels.map_or("none".into(), |l| format!("{l:?}").replace(' ', "")),
        limit: limit.replace(' ', ""),
        pass: levels.is_some_and(ok),
    }
}

/// The first ESC tick after `after_us` at which all four plant levels are 0.
fn first_stop(trace: &Trace, after_us: u64) -> Option<u64> {
    trace.esc.iter().find(|(t, l)| *t > after_us && *l == [0; 4]).map(|(t, _)| *t)
}
