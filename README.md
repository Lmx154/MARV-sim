# MARV-sim

The simulation workspace for bringing up guidance, navigation and control on the MARV quadcopter. The plan, the
decisions and the owner's controlling document are in `MARV-V2-drivers/_plan` (`plan-sim-suite.md`,
`decision-sim-*-20261119.md`, `HANDOFF-sim-suite-20261118.md`).

State on 2026-11-19: build step 2 of 9. Only the vehicle card, the environment file and their checker exist. There is
no plant, no runner and no flight code here yet.

## What is here

| Path | What it is |
|---|---|
| `marv-card/marv.toml` | The vehicle card: the only source of the vehicle's physical numbers. Every number is a record with a value, a unit, a tag (`measured`, `datasheet`, `cad`, `derived`, `placeholder`) and a source. |
| `marv-card/src/` | The loader and the checks (`schema.rs`) and the `card-check` program. |
| `env/default.toml` | Gravity, atmosphere, origin, magnetic field, wind. Its origin is a public reference point, never home. |
| `env/home.local.toml` | The real home location, if you create it. Git-ignored; never commit it. |

## Check the card

    cargo run -q -p marv-card --bin card-check -- marv-card/marv.toml env/default.toml

It prints `PASS` and exits 0 when every number has a tag and the card is consistent, then lists the open items: every
placeholder and every value or range that is still `"ask"`. Any error exits 1.

## Change the card

- The block between `# >>> BEGIN GENERATED` and `# <<< END GENERATED` comes from the CAD export. Do not edit it; the
  check notices. Regenerate it from the CAD repo, after committing `model/mass_properties.json` there:

      cd ~/Projects/CAD/drone && python3 tools/make_card.py ~/Documents/MARV-sim/marv-card/marv.toml

- Everything else is edited by hand. Give each number its tag and say in `src` where it came from. A `placeholder`
  needs a range, or `range = "ask"`.

## Tests

    cargo test -p marv-card

Six tests: one card that passes, and one broken card per check.
