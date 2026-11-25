# Sensors

Part of [MARV-sim](../README.md)

`sensors.rs` models the two IMUs; `slow_sensors.rs` models the barometer, compass and GNSS. Each makes the firmware's own sample
type (`fw_core::sample::{BaroSample, MagSample, GnssSample}`), stamped `t_us` with the sim time and `seq` +1 per sample, from the
truth of the card's latency earlier (a truth history at the 125 us tick: 92, 40 and 3440 ticks). Each slow model draws from its own
noise stream.

| Sensor | Rate | Latency | Model |
|---|---|---|---|
| IMU 1, 2 | 800 Hz | none | Gyro and accel with card noise and biases; at rest in GROUND reads gravity only |
| Barometer | 50 Hz | 92 ticks (11.5 ms) | ISA troposphere at origin altitude minus down; drift random walk (1-sigma `baro.drift_sd` 0.77 m after `baro.drift_time` 840 s); airborne white height noise `baro.airborne_height_noise_sd` 0.11 m; `baro.pressure_bias`; pressure noise `baro.pressure_noise_sd`; while airborne a band-limited motion error 0.08 to 0.5 Hz, 0.0476 m per deg of truth tilt (`[scenario] baro_motion = false` leaves it out). Temperature: ISA at that height |
| Compass | 100 Hz | 40 ticks (5 ms) | Env field in body FRD by the true attitude, then true distortion `compass.soft_iron x field + compass.hard_iron_bias` plus noise `compass.field_noise_sd`, then the applied calibration (`Calibration::NONE` in the runner). No mounting rotation. Each sample carries `MagSample.expected` (49.131 uT, 66.532 deg for the default env; tolerances `MAG_HLT_STR` 10 %, `MAG_HLT_INC` 10 deg; `[scenario] mag_expected = false` leaves it `None`) |
| GNSS | 5 Hz (1, 5, 10 by `gnss_rate_hz`) | 3440 ticks (0.43 s) | Position error = first-order Gauss-Markov term (`gnss.slow_horizontal_sd`, `gnss.slow_time_constant`) plus white `gnss.horizontal_sd`; vertical Gauss-Markov `gnss.slow_vertical_sd`; velocity plus `gnss.velocity_sd`, as speed and course (course held below 0.3 m/s); fix type 3; satellites, DOP, UTC `None`; local tangent plane at the env origin (WGS 84 radii), rounded to 1e-7 deg and mm; `vel_ned_mm_s` on its own stream (`gnss_velocity = false` leaves it `None`) |

Not modelled: barometer ground-effect error (0.47 m near the ground under thrust) and speed error (+0.6 m above 3 m/s), the GNSS
receiver's smoothing and hold. The sim does not log the slow samples.

## Injected faults

Scenario fields, all off by default, used by the diagnostic scenarios in [campaigns](campaigns.md):

| Field | Effect |
|---|---|
| `baro_lost = [[from, to]]` | No barometer sample reaches the flight code in [from, to); the model keeps running |
| `gnss_lost = [[from, to]]` | No GNSS sample reaches the flight code |
| `gnss_hold = [[from, to]]` | Samples repeat the last position and report zero speed and velocity; the true solution returns after, with the jump |
| `gnss_latency_s` | Replaces the 0.43 s GNSS latency |
| `mag_fault = { from_s, kind, deg, below_m, axis }` | `rotate`: calibrated field turned about body `axis` (z default) by `deg`; `stuck`: the field of the first sample at or after `from_s` repeats while time and sequence advance. A rotation about z keeps strength and inclination so it passes the health test; about x or y it does not. `below_m`: only while height above start is below it |
| `wind_steps = [[t_s, dN, dE, dD], ...]` | From `t_s` the plant's wind is the mean wind plus the delta, m/s |

## Code

`marv-sim/src/sensors.rs`, `slow_sensors.rs`; tests in `marv-sim/tests/slow_sensors.rs` (noise sd, Gauss-Markov sigma and time
constant, drift size, latency, rates, course hold, compass identity under perfect calibration, GNSS round trip and 180 deg wrap,
every sample reaching a flight step exactly once).

## Related

[architecture](architecture.md), [vehicle-model](vehicle-model.md), [estimators](estimators.md)
