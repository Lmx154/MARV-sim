# marv-card

Part of [MARV-sim](../README.md)

The vehicle card and its tooling. `marv.toml` is the only source of the vehicle's physical numbers; each is a record with a value,
unit, tag and source. The crate has two faces: without feature `std` it is `no_std` and holds only the constants `build.rs` compiles
from the card (the firmware uses these); with `std` it adds the loader, the checks and `card-check`.

| Path | Contents |
|---|---|
| `marv.toml` | The card. The block between `# >>> BEGIN GENERATED` and `# <<< END GENERATED` comes from the CAD export; edit the rest by hand |
| `build.rs` | Compiles the card into `const` data: rotor positions and spins, mixer matrix, coefficients, levels, mass, sha256. An `"ask"` the firmware needs fails the build; placeholders warn |
| `src/lib.rs`, `src/schema.rs` | The loader (`Card`, `Env`), the checks, the home-override check |
| `src/bin/card-check.rs` | `card-check <card.toml> <env.toml>` |

```
cargo run -q -p marv-card --bin card-check -- marv-card/marv.toml env/default.toml
```

The firmware's `fw-flight` pins this crate from the MARV-sim repository by git revision; the workspace patches that pin to this folder.

Details: [vehicle-model](../docs/vehicle-model.md), [tools](../docs/tools.md#card-check).
