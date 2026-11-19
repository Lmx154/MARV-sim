//! The sim pilot: a TEST DEVICE in the harness, not flight code (decision-sim-takeoff-landing-20261119, section 2).
//! It flies the throttle from truth, as a human flies it by eye, with a human's limits: it sees only a `PilotView`
//! (truth height, vertical speed, horizontal velocity in the heading frame, contact flag) delayed by `delay_s`, and
//! makes one stick report per 4 ms. It never sees the estimate, the allocator flags, the spread point or the plant's
//! level-to-speed map. The arm switch and the phase table are scripted, never the pilot's.

use crate::backend::Truth;
use crate::scenario::{Phase, PhaseEnd, PilotParams, RcRow, Throttle};
use marv_card::{Card, Env};
use std::collections::VecDeque;

/// One stick report every 4 ms (marv-run.rs: RC_TICKS of 125 us).
pub const REPORT_US: u64 = 4000;

/// The stick's feel in roll and pitch: full stick is ANGLE_MAX, 30 deg (decision section 2; fw-flight step.rs:73-74).
/// The pilot knows it; it is not read from the flight code.
const ANGLE_MAX_RAD: f64 = 30.0 * std::f64::consts::PI / 180.0;

/// All the pilot is given of the world, once per report.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PilotView {
    /// Up, m (minus truth down).
    pub height_m: f64,
    /// Up, m/s.
    pub climb_ms: f64,
    /// Truth horizontal velocity in the heading frame: forward, right, m/s.
    pub v_heading_ms: [f64; 2],
    /// Truth contact flag.
    pub on_ground: bool,
}

impl PilotView {
    /// What the pilot sees of `truth`.
    pub fn see(truth: &Truth) -> PilotView {
        let [w, x, y, z] = truth.attitude;
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        let [vn, ve, vd] = truth.velocity_ms;
        PilotView {
            height_m: -truth.position_m[2],
            climb_ms: -vd,
            v_heading_ms: [vn * yaw.cos() + ve * yaw.sin(), -vn * yaw.sin() + ve * yaw.cos()],
            on_ground: truth.on_ground,
        }
    }
}

/// How a phase ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ending {
    Time,
    Event,
    Timeout,
}

/// One phase as it ran: its start and, once it ended, when and how.
#[derive(Debug, Clone, PartialEq)]
pub struct PhaseRun {
    pub name: String,
    pub start_us: u64,
    pub end: Option<(u64, Ending)>,
}

/// One report and what the pilot used to make it (the PIL record).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Report {
    pub row: RcRow,
    /// Index into the phase table.
    pub phase: usize,
    /// 0 zero, 1 ramp, 2 height, 3 descend.
    pub mode: u8,
    /// h_ref (m) in height, v_ref (m/s, up) in descend, 0 otherwise.
    pub reference: f64,
    pub integrator: f64,
    /// The drift correction in the roll and pitch sticks.
    pub xy: [f64; 2],
    /// The thrust ratio (of weight) the throttle stick stands for, by the pilot's map.
    pub ratio: f64,
}

/// The bound on I at the ramp-to-height switch: |I| <= 0.5 g (decision Amendment A1).
const SWITCH_I_MAX_G: f64 = 0.5;

/// Thrust ratio c (of m g) to throttle stick: the documented two-slope map 0 -> 0, 0.5 -> m g, 1 -> full, inverted.
/// `full` is the full collective over m g (2.0 with the flight code's cap: one straight line).
pub fn ratio_to_stick(c: f64, full: f64) -> f64 {
    if c <= 1.0 { c / 2.0 } else { 0.5 + (c - 1.0) / (2.0 * (full - 1.0)) }
}

/// The map itself: throttle stick to thrust ratio.
pub fn stick_to_ratio(u: f64, full: f64) -> f64 {
    if u <= 0.5 { 2.0 * u } else { 1.0 + (2.0 * u - 1.0) * (full - 1.0) }
}

pub struct Pilot {
    p: PilotParams,
    phases: Vec<Phase>,
    gravity: f64,
    /// Full collective over m g, the documented map's top point from the nominal card:
    /// min(stick_thrust_max_g, 4 k_F w_max^2 / (m g)).
    full: f64,
    delay_reports: usize,
    /// What the pilot saw, newest at the back; the front is what it acts on now.
    seen: VecDeque<PilotView>,
    index: usize,
    start_us: u64,
    arm: bool,
    /// Roll, pitch, throttle, yaw as last sent.
    sticks: [f64; 4],
    integrator: f64,
    /// The thrust ratio commanded at the first report whose delayed view shows contact = false.
    c_lift: Option<f64>,
    /// The thrust ratio the stick stood for at the ramp-to-height switch.
    switch_ratio: Option<f64>,
    timeline: Vec<PhaseRun>,
}

impl Pilot {
    /// `card` is the NOMINAL card (not a spread point's): the pilot's idea of the stick map.
    pub fn new(card: &Card, env: &Env, p: PilotParams, phases: Vec<Phase>) -> Result<Pilot, String> {
        let mass = card.mass.mass.v.ok_or("mass.mass is \"ask\"")?;
        let g = env.gravity.v.ok_or("gravity.g is \"ask\"")?;
        let k = card.propulsion.thrust_coefficient.v.ok_or("propulsion.thrust_coefficient is \"ask\"")?;
        let w_max = card.propulsion.max_rotor_speed.v.ok_or("propulsion.max_rotor_speed is \"ask\"")?;
        let full = p.stick_thrust_max_g.min(4.0 * k * w_max * w_max / (mass * g));
        if full <= 1.0 {
            return Err(format!("pilot: the stick's full collective {full} x m g must be above m g"));
        }
        let first = phases.first().ok_or("pilot: no phases")?;
        let timeline = vec![PhaseRun { name: first.name.clone(), start_us: 0, end: None }];
        let arm = first.arm.unwrap_or(false);
        Ok(Pilot {
            delay_reports: (p.delay_s * 1e6 / REPORT_US as f64).round() as usize,
            p,
            phases,
            gravity: g,
            full,
            seen: VecDeque::new(),
            index: 0,
            start_us: 0,
            arm,
            sticks: [0.0; 4],
            integrator: 0.0,
            c_lift: None,
            switch_ratio: None,
            timeline,
        })
    }

    /// Full collective over m g, as the pilot believes it.
    pub fn full_ratio(&self) -> f64 {
        self.full
    }

    /// c_lift and the thrust ratio at the ramp-to-height switch, once they happened.
    pub fn lift_ratios(&self) -> (Option<f64>, Option<f64>) {
        (self.c_lift, self.switch_ratio)
    }

    pub fn timeline(&self) -> &[PhaseRun] {
        &self.timeline
    }

    /// v_ref of a closed-loop phase, from what the pilot sees.
    fn v_ref(&self, throttle: Throttle, v: &PilotView) -> f64 {
        match throttle {
            Throttle::Height(h) => (self.p.k_h * (h - v.height_m)).clamp(-self.p.v_dn_ms, self.p.v_up_ms),
            Throttle::Descend(s) => -s,
            Throttle::Zero | Throttle::Ramp => 0.0,
        }
    }

    fn accel_max(&self) -> f64 {
        self.p.accel_max_g * self.gravity
    }

    /// Starts phase `i` at `now_us`. From "ramp" into a closed-loop phase, I = g (c_lift - 1), within +/-0.5 g: the
    /// pilot goes back to where it came off the ground, and the stick rate limit does the smoothing (Amendment A1).
    /// Before any lift-off is seen, c_lift is the ratio the stick stands for now.
    fn enter(&mut self, i: usize, now_us: u64) {
        let was = self.phases[self.index].throttle;
        self.index = i;
        self.start_us = now_us;
        let Some(ph) = self.phases.get(i) else { return };
        self.timeline.push(PhaseRun { name: ph.name.clone(), start_us: now_us, end: None });
        self.arm = ph.arm.unwrap_or(self.arm);
        if was == Throttle::Ramp && matches!(ph.throttle, Throttle::Height(_) | Throttle::Descend(_)) {
            let now = stick_to_ratio(self.sticks[2], self.full);
            self.switch_ratio = Some(now);
            let c_lift = self.c_lift.unwrap_or(now);
            let bound = SWITCH_I_MAX_G * self.gravity;
            self.integrator = (self.gravity * (c_lift - 1.0)).clamp(-bound, bound);
        }
    }

    /// The report at `now_us` from what the pilot sees now (`view`, delayed here). `None` once the phase table has
    /// ended: the run is over.
    pub fn report(&mut self, now_us: u64, view: PilotView) -> Option<Report> {
        self.seen.push_back(view);
        if self.seen.len() > self.delay_reports + 1 {
            self.seen.pop_front();
        }
        let v = self.seen[0];
        // End phases whose time, event or timeout has come; a new phase's event is looked at at once.
        while let Some(ph) = self.phases.get(self.index) {
            let elapsed = now_us - self.start_us;
            let event = |timeout_us: u64, seen: bool| {
                if seen {
                    Some(Ending::Event)
                } else if elapsed >= timeout_us {
                    Some(Ending::Timeout)
                } else {
                    None
                }
            };
            let ending = match ph.end {
                PhaseEnd::After { us } => (elapsed >= us).then_some(Ending::Time),
                PhaseEnd::Airborne { timeout_us } => event(timeout_us, v.height_m >= self.p.airborne_m),
                PhaseEnd::Below { below_m, timeout_us } => event(timeout_us, v.height_m < below_m),
                PhaseEnd::Landed { timeout_us } => event(timeout_us, v.on_ground),
            };
            let Some(ending) = ending else { break };
            self.timeline[self.index].end = Some((now_us, ending));
            self.enter(self.index + 1, now_us);
        }
        let ph = self.phases.get(self.index)?.clone();

        let dt = REPORT_US as f64 * 1e-6;
        let elapsed_s = (now_us - self.start_us) as f64 * 1e-6;
        let (mode, reference, target) = match ph.throttle {
            Throttle::Zero => (0, 0.0, 0.0),
            Throttle::Ramp => (1, 0.0, self.p.ramp_stick_per_s * elapsed_s),
            Throttle::Height(_) | Throttle::Descend(_) => {
                let v_ref = self.v_ref(ph.throttle, &v);
                let e = v_ref - v.climb_ms;
                let a = (self.p.k_v * e).clamp(-self.accel_max(), self.accel_max()) + self.integrator;
                // Frozen while the stick is at 0 or 1.
                if self.sticks[2] > 0.0 && self.sticks[2] < 1.0 {
                    self.integrator += self.p.k_i * e * dt;
                }
                let reference = if let Throttle::Height(h) = ph.throttle { (2, h) } else { (3, v_ref) };
                (reference.0, reference.1, ratio_to_stick(1.0 + a / self.gravity, self.full))
            }
        };
        let xy = if ph.xy_hold {
            let g = self.gravity;
            let [fwd, right] = v.v_heading_ms;
            let c = [-self.p.xy_k * right / g / ANGLE_MAX_RAD, -self.p.xy_k * fwd / g / ANGLE_MAX_RAD];
            let n = (c[0] * c[0] + c[1] * c[1]).sqrt();
            if n > self.p.xy_stick_max { c.map(|x| x * self.p.xy_stick_max / n) } else { c }
        } else {
            [0.0; 2]
        };
        // Roll, pitch, throttle, yaw: each moves at most the stick rate per report, within its range.
        let targets = [f64::from(ph.roll) + xy[0], f64::from(ph.pitch) + xy[1], target, f64::from(ph.yaw)];
        let step = self.p.stick_rate_per_s * dt;
        for (i, t) in targets.into_iter().enumerate() {
            let lo = if i == 2 { 0.0 } else { -1.0 };
            self.sticks[i] = (self.sticks[i] + (t - self.sticks[i]).clamp(-step, step)).clamp(lo, 1.0);
        }
        if self.c_lift.is_none() && !v.on_ground {
            self.c_lift = Some(stick_to_ratio(self.sticks[2], self.full));
        }
        let [roll, pitch, throttle, yaw] = self.sticks.map(|x| x as f32);
        Some(Report {
            row: RcRow { t_us: now_us, roll, pitch, yaw, throttle, arm: self.arm },
            phase: self.index,
            mode,
            reference,
            integrator: self.integrator,
            xy,
            ratio: stick_to_ratio(self.sticks[2], self.full),
        })
    }
}
