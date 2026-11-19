//! The physics core: a plant (motor levels to one wrench) and a backend (wrench to truth), the IMU model, the scenario
//! file and the log, which the `marv-run` binary puts together with the firmware's flight step.
//! Conventions: MARV-V2-drivers/_plan/decision-sim-conventions-20261119.md.
//! World NED, body FRD at the centre of mass, quaternion body to world `[w, x, y, z]`, SI, time in u64 us.

pub mod backend;
pub mod log;
pub mod math;
pub mod plant;
pub mod rng;
pub mod scenario;
pub mod sensors;
pub mod verdict;

pub use backend::{Backend, NativeBackend, Reaction, Start, Touchdown, Truth};
pub use plant::{Plant, Wrench};
