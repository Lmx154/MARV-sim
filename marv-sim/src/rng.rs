//! Seeded random numbers by hand: SplitMix64 seeds, xoshiro256** generates, Box-Muller makes Gaussians.
//! Each sensor takes its own stream from the run seed, so adding a sensor later leaves the others' noise unchanged.

pub struct Rng {
    s: [u64; 4],
    /// Box-Muller makes two Gaussians at a time; this is the second one.
    spare: Option<f64>,
}

fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

impl Rng {
    /// `stream` is a fixed number per sensor (IMU1 = 1, IMU2 = 2).
    pub fn new(seed: u64, stream: u64) -> Rng {
        // Hash the seed and the stream number separately, so two streams do not walk along the same sequence.
        let (mut a, mut b) = (seed, stream);
        let mut sm = splitmix64(&mut a) ^ splitmix64(&mut b);
        let s = [splitmix64(&mut sm), splitmix64(&mut sm), splitmix64(&mut sm), splitmix64(&mut sm)];
        Rng { s, spare: None }
    }

    fn next_u64(&mut self) -> u64 {
        let result = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// Standard normal: mean 0, sd 1.
    pub fn gaussian(&mut self) -> f64 {
        if let Some(z) = self.spare.take() {
            return z;
        }
        // u1 in (0, 1] so that ln(u1) is finite; u2 in [0, 1).
        let u1 = ((self.next_u64() >> 11) + 1) as f64 * (1.0 / (1u64 << 53) as f64);
        let u2 = (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64);
        let radius = (-2.0 * u1.ln()).sqrt();
        let angle = std::f64::consts::TAU * u2;
        self.spare = Some(radius * angle.sin());
        radius * angle.cos()
    }
}
