# N19 "mission switch" smoke (decision-mavlink-mode-20261123 sections 67-70)

Firmware: `../MARV-V2-firmware` on branch `mission-switch-sim` (= mission-switch e5aed63 + the four sim hook commits).
Simulator: this repo, branch `mission-switch-sim`. Results: `MARV-V2-drivers/_plan/flights/20261124-ms-sim/smoke/`.

## What marv-run gained

- `[scenario] rc_source = "receiver"`: stick reports are receiver reports (`RcSample.source = Receiver`) at 100 reports/s
  (every 10 ms). The pilot still makes a report every 4 ms and the newest one is sent. `"crsf"`, the default, keeps CRSF
  reports at 250/s, so existing scenarios run unchanged.
- Channel 8 (`RcSample.mission`): `mission = true|false` in `[rc] phase` rows (it carries over like `arm` and `mode`, and is
  OFF at the start) and in `[rc] script` rows (default false).
- `throttle = <0..1>` in a phase: a fixed throttle stick (0.5 is the centre), reached at the stick rate limit.
- Transmitter off: the existing `[rc] lost = [{ from_s, to_s }]` windows and `lost = true` phases also apply to receiver
  reports, judged at each 10 ms report time. Stick-loss detection uses the period of the scenario's own source.
- `MARV_MS_CSV=<dir>`: writes one CSV per run, with every 8th flight step and every state change: state, mode, leg, armed,
  motors running, truth position and velocity.

## Reproduce

    # build (one cargo at a time on this machine)
    cargo build --release --bin marv-run
    # item 2: the CRSF re-run (part 3's stick-loss group a-h10-t2-g100, part 3's owner's-flight group m1-square, takeoff-land seed 1)
    scenarios/ms-smoke/crsf_rerun.sh <out>/crsf-rerun                      # new binary, files *.new.txt
    scenarios/ms-smoke/crsf_rerun.sh <out>/crsf-rerun <part-3 binary> .p3bin   # the part-3 binary, for comparison
    # items 3 and 4: ms-smoke, ms-smoke-a, -b, -c (one run each, seed 1, nominal card, calm air)
    scenarios/ms-smoke/run_smoke.sh <out>
    python3 -I scenarios/ms-smoke/parse_smoke.py <out> > <out>/smoke_numbers.txt

| Scenario | Flags | What it does |
|---|---|---|
| ms-smoke | --fs-hold | Arms with the mission switch OFF, hand take-off to 5 m, Position hover. Mission switch ON, then Start 5 s after HOLD, then the square at 3 m/s. 45 s after the switch goes ON, in the last leg: mission switch OFF with arm ON, Position and sticks centred. 15 s later: arm OFF + mission switch ON. |
| ms-smoke-a | --fs-hold | Arm ON for 5 s with the mission switch ON. Then arm OFF, mission switch OFF, arm ON (the control). |
| ms-smoke-b | --ho | FS_HOLD_EN 0: arm ON for 5 s with the mission switch OFF. |
| ms-smoke-c | --fs-hold | As ms-smoke up to AUTO, then at AUTO + 20 s arm OFF + mission switch OFF in one report. |
