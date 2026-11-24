#!/usr/bin/env python3
"""usage: python3 -I parse_smoke.py <out-dir>: the numbers of N19's smoke family from <out-dir>/raw/*.txt (marv-run's hands_off line)
and <out-dir>/csv/*-seed1.csv (MARV_MS_CSV). State codes: 1 PILOT, 2 TAKEOVER, 3 HOLD, 4 AUTO, 5 RETURN, 6 LAND, 7 STOP, 8 BLIND."""
import csv, math, sys

O = sys.argv[1]
WP = [(30.0, 0.0, 10.0), (30.0, 30.0, 15.0), (0.0, 30.0, 10.0), (0.0, 0.0, 10.0)]
NAMES = {1: 'PILOT', 2: 'TAKEOVER', 3: 'HOLD', 4: 'AUTO', 5: 'RETURN', 6: 'LAND', 7: 'STOP', 8: 'BLIND'}


def ho(name):
    for line in open(f'{O}/raw/{name}.txt'):
        if line.startswith('hands_off ') or ' hands_off ' in line:
            t = line.split('hands_off ', 1)[1].split()
            return dict(zip(t[0::2], t[1::2]))
    return {}


def rows(name):
    with open(f'{O}/csv/{name}-seed1.csv') as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def seg_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - u * dx, py - ay - u * dy)


def states(r):
    out, prev = [], None
    for x in r:
        s = int(x['state'])
        if s != prev:
            out.append((x['t_s'], s, x['true_h']))
            prev = s
    return out


def motor_stops(r):
    """(t, h) of every motors-running -> stopped edge."""
    return [(b['t_s'], b['true_h']) for a, b in zip(r, r[1:]) if a['motors'] == 1 and b['motors'] == 0]


def smoke():
    r, d = rows('ms-smoke'), ho('ms-smoke')
    print('== ms-smoke (FS_HOLD_EN 1, receiver source, nominal card, calm air, seed 1)')
    print('state sequence (t s, state, true h m): ' + ', '.join(f'{t:.3f} {NAMES[s]}({s}) h {h:.2f}' for t, s, h in states(r)))
    print(f"firmware fields: states {d.get('states')} | Start {d.get('msn_cmds')} | waypoints reached at {d.get('msn_wp_t_s')} "
          f"(legs {d.get('msn_legs')}) | arrival true distance {d.get('msn_wp_d_arr_m')} m | arrival height error {d.get('msn_wp_herr_m')} m")
    auto = [x for x in r if x['state'] == 4]
    hold_pt = next((x['true_n'], x['true_e']) for x in r if x['state'] == 4)
    xt, herr = 0.0, 0.0
    for x in auto:
        leg = int(x['leg'])
        a = hold_pt if leg == 0 else WP[leg - 1][:2]
        xt = max(xt, seg_dist((x['true_n'], x['true_e']), a, WP[leg][:2]))
    # Height error on the constant-height leg 3 (10 m -> 10 m) and over all AUTO against the active waypoint's height.
    h3 = [abs(x['true_h'] - 10.0) for x in auto if int(x['leg']) == 3]
    hall = [abs(x['true_h'] - WP[int(x['leg'])][2]) for x in auto]
    vmax = max(math.hypot(x['true_vn'], x['true_ve']) for x in auto)
    print(f"AUTO {auto[0]['t_s']:.3f}-{auto[-1]['t_s']:.3f} s: largest true distance from the leg's segment {xt:.2f} m (the sim's per-leg "
          f"cross-track max {d.get('msn_leg_xt_max_m')}); largest |h - leg height| on the 10 m -> 10 m leg 3 {max(h3):.2f} m; largest "
          f"|h - active waypoint height| over all AUTO {max(hall):.2f} m (includes the 10 -> 15 -> 10 m climbs and descents); sim's true "
          f"height sag max {d.get('msn_sag_true_max_m')} m; largest horizontal speed {vmax:.2f} m/s")
    i = next(k for k, x in enumerate(r) if x['state'] == 1 and r[k - 1]['state'] == 4)
    hb = r[i]
    win = [x for x in r[i:] if x['state'] == 1]
    v0 = math.hypot(hb['true_vn'], hb['true_ve'])
    hs = [x['true_h'] for x in win]
    k_stop = next((x for x in win if math.hypot(x['true_vn'], x['true_ve']) < 0.3), None)
    end = win[-1]
    print(f"hand-back AUTO -> PILOT at {hb['t_s']:.3f} s (phase with the mission switch OFF from 142.472 s, receiver reports every 10 ms; leg {int(r[i-1]['leg'])}), h {hb['true_h']:.2f} m, "
          f"horizontal speed {v0:.2f} m/s; in PILOT ({win[0]['t_s']:.2f}-{end['t_s']:.2f} s, centred sticks, Position): height change "
          f"{min(hs) - hb['true_h']:+.2f} / {max(hs) - hb['true_h']:+.2f} m (min / max), speed below 0.3 m/s at {k_stop['t_s'] - hb['t_s']:.2f} s, "
          f"{math.hypot(k_stop['true_n'] - hb['true_n'], k_stop['true_e'] - hb['true_e']):.2f} m from the hand-back point; at the end of PILOT "
          f"{math.hypot(end['true_n'] - k_stop['true_n'], end['true_e'] - k_stop['true_e']):.2f} m from where it stopped, h {end['true_h']:.2f} m, "
          f"speed {math.hypot(end['true_vn'], end['true_ve']):.2f} m/s")
    land = next(x for x in r if x['state'] == 6)
    print(f"LAND at {land['t_s']:.3f} s (phase with arm OFF + mission switch ON from 157.472 s) from h {land['true_h']:.2f} m; touchdown "
          f"{d.get('td_s')} s at {d.get('td_vz_ms')} m/s down, {d.get('td_vh_ms')} m/s horizontal, {d.get('land_to_td_m')} m from the LAND start; "
          f"STOP {d.get('stop_s')} s (contact to motors stopped {d.get('contact_to_mstop_s')} s)")
    st = motor_stops(r)
    print(f"motor stops (t, h): {', '.join(f'{t:.3f}@{h:.2f}m' for t, h in st)}; above 0.3 m: {sum(h > 0.3 for _, h in st)} "
          f"(per-step air_stops {d.get('air_stops')}, first {d.get('air_stop_first')})")


def check(name, what):
    r, d = rows(name), ho(name)
    armed = [x['t_s'] for x in r if x['armed'] == 1]
    run = [x['t_s'] for x in r if x['motors'] == 1]
    print(f"== {name}: {what}")
    print('state sequence: ' + ', '.join(f'{t:.3f} {NAMES[s]}({s}) h {h:.2f}' for t, s, h in states(r)))
    print(f"armed steps: {len(armed)}" + (f" ({armed[0]:.3f}-{armed[-1]:.3f} s)" if armed else '') +
          f"; motors running: {len(run)}" + (f" ({run[0]:.3f}-{run[-1]:.3f} s)" if run else ''))
    return r, d


smoke()
print()
check('ms-smoke-a', 'arm ON 60-65 s with the mission switch ON; arm OFF 65 s; mission switch OFF 66 s; arm ON 67-70 s (control); arm OFF 70 s')
print()
check('ms-smoke-b', 'FS_HOLD_EN 0 (--ho): arm ON 60-65 s with the mission switch OFF')
print()
r, d = check('ms-smoke-c', 'as ms-smoke to AUTO; arm OFF + mission switch OFF in one report (phase from 122.472 s)')
st = motor_stops(r)
k = next(x for x in r if x['state'] == 7 and x['t_s'] > 100)
print(f"STOP at {k['t_s']:.3f} s, h {k['true_h']:.2f} m, leg {int(next(x for x in reversed(r) if x['t_s'] < k['t_s'])['leg'])}; motor stops (t, h; CSV every 10 ms): "
      f"{', '.join(f'{t:.3f}@{h:.2f}m' for t, h in st)}; firmware fields mstop_s {d.get('mstop_s')} h_mstop_m {d.get('h_mstop_m')}")
