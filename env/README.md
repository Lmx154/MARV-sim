# env

Part of [MARV-sim](../README.md)

The simulated world: gravity, ISA atmosphere, origin, magnetic field and wind, in the same tagged-record form as the card.

| File | Contents |
|---|---|
| `default.toml` | Standard gravity, ISA sea-level pressure, temperature and lapse rate, origin at the Royal Observatory, Greenwich (a public reference point, never home), the WMM2025 field there, zero mean wind and gusts (placeholders) |
| `home.local.toml` | Optional, git-ignored, never committed: records of `[origin]` and `[magnetic_field]` that replace the default's under `marv-run --home` |

`default.toml` names `home.local.toml` as its `home_override`; `marv-run --home` accepts only that file. Scenario `[scenario]` fields
(`wind_ned`, `wind_steps`, spread `air` axis) set wind per run.

Details: [vehicle-model](../docs/vehicle-model.md#the-environment).
