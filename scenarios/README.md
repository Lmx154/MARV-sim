# scenarios

Part of [MARV-sim](../README.md)

Scenario files (what the vehicle and the pilot do, and how the run is judged) and spread files (grids of scenarios, plant points
and seeds). Format: [scenarios](../docs/scenarios.md); how grids run: [campaigns](../docs/campaigns.md).

| Path | Contents |
|---|---|
| `hover.toml`, `steps.toml`, `stick-sign.toml`, `rc-loss.toml`, `alt-hold.toml`, `takeoff-land.toml` | The basic scenarios |
| `pos-hold*.toml` | Position-hold variants with wind |
| `spread.toml` | The basic scenarios at five plant points |
| `takeoff-land-matrix.toml` | `takeoff-land` over 5 points x 2 maps x 3 air x 3 ground x 3 seeds = 270 cells |
| `fault/` | Diagnostic runs with injected GNSS, compass, latency and yaw faults |
| `headmon/` | Heading-monitor runs with an injected filter yaw error |
| `handsoff/`, `handsoff-n/`, `handsoff-n2/` | Hands-off behaviour after stick loss |
| `mission/`, `mission-p3/`, `mission-r/` | Waypoint missions: AUTO, RETURN, battery and heartbeat conditions |
| `negative/` | Negative controls that must FAIL |
| `ms-smoke/` | [Mission-switch smoke scenarios](ms-smoke/README.md) |

A campaign run writes its per-cell figures (`summary.txt`, `results*.txt`) into its folder; they are not committed.

```
cargo run -q --release -p marv-sim --bin marv-run -- --card marv-card/marv.toml --env env/default.toml \
    --spread scenarios/spread.toml --seed 1 --out-dir logs/spread
```
