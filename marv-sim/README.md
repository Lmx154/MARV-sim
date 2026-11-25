# marv-sim

Part of [MARV-sim](../README.md)

The simulator crate: a library (plant, backend, sensor models, scenario loader, pilot, verdict, log) and two programs,
`marv-run` and `marv-replay`. It depends on the firmware's `fw-flight` and `fw-core` by path (see the
[top README](../README.md)) and on `marv-card` with feature `std`. `build.rs` records the firmware checkout's commit.

| Path | Contents | Page |
|---|---|---|
| `src/plant.rs` | Motor levels to a wrench | [vehicle-model](../docs/vehicle-model.md) |
| `src/backend.rs` | Wrench to truth (RK4), ground contact | [vehicle-model](../docs/vehicle-model.md) |
| `src/sensors.rs`, `slow_sensors.rs`, `rng.rs` | IMU, barometer, compass, GNSS models; seeded noise | [sensors](../docs/sensors.md) |
| `src/scenario.rs` | Scenario and spread files | [scenarios](../docs/scenarios.md), [campaigns](../docs/campaigns.md) |
| `src/pilot.rs` | The sim pilot, a test device | [scenarios](../docs/scenarios.md) |
| `src/verdict.rs` | The checks | [scenarios](../docs/scenarios.md) |
| `src/log.rs` | DataFlash `.BIN` writer | [tools](../docs/tools.md) |
| `src/replay.rs` | Log reader, `Sensors` stream rebuilt from a log | [tools](../docs/tools.md) |
| `src/math.rs` | Small vector and quaternion helpers | |
| `src/bin/marv-run.rs` | The runner | [tools](../docs/tools.md), [architecture](../docs/architecture.md) |
| `src/bin/marv-replay.rs` | Replay of a flown log | [tools](../docs/tools.md) |
| `tests/` | `invariants`, `slow_sensors`, `althold_verdict`, `nav_vertical`, `nav_eskf` | [tools](../docs/tools.md#tests) |

```
cargo test -p marv-sim
cargo run -q --release -p marv-sim --bin marv-run -- --help
```
