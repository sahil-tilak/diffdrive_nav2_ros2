#!/usr/bin/env python3
"""Analyse the pitch test recorded by scripts/pitch_record.py + scripts/run_batch.sh.

Usage: python3 scripts/pitch_analyze.py PREFIX N [--logdir DIR]

Reads (never writes) LOGDIR/PREFIX_KK_{pose,cmdvel,scan}.csv, _nav2.log, _goal.log.
Writes LOGDIR/PREFIX_analysis.txt with mode 'x' (never overwrites).

Definitions are fixed in the plan (set before data):
- Nose-down p = -asin(R31), R31 = z of the body +x axis in world. Positive = nose down.
- Tilt = acos(R33) = angle between body z and world z (roll and pitch together).
- Braking event: cmd_vel linear.x drops >= 0.1 m/s within 0.5 s (sim). Event time =
  first lower sample. Events closer than 0.5 s to the previous one are merged.
- Pitch around an event: max p in [t-0.2, t+1.5]. "During braking": p >= 1.5 deg
  inside [t, t+1.5] of some event.
- Floor-hit scan: all 360 beams; beam a -> v=(cos a, sin a, 0); z = R31 cos a + R32 sin a
  (third row of R, yaw-independent). If z < 0 and h/(-z) <= 10 m, predicted = h/(-z).
  Match if measured finite and within 20 %. Floor-hit if >= 8 consecutive matches
  (circular). Tested on scans with tilt >= 1.5 deg. Control: tilt < 0.5 deg (0 by
  construction). Extra control: each tilted scan's predictions applied to the nearest
  earlier scan with tilt < 0.5 deg.
"""
import argparse
import math
import os
import re
import sys

import numpy as np

H = 0.14          # lidar height from URDF: base_link_height 0.07 + lidar_z_offset 0.07
RMAX = 10.0       # URDF <range><max>
AMIN, AMAX, NBEAM = -3.14159, 3.14159, 360
ANG = AMIN + np.arange(NBEAM) * (AMAX - AMIN) / (NBEAM - 1)  # increment convention unverified (<1 deg effect)
TOL = 0.20
MIN_RUN = 8
P_HIT = 1.5
P_CTRL = 0.5
DROP = 0.1
DROP_WIN = 0.5
TS_RE = re.compile(r'\[(\d{9,}\.\d+)\]')


def load_csv(path):
    with open(path) as f:
        head = f.readline().strip().split(',')
    if os.path.getsize(path) == 0:
        return head, np.zeros((0, len(head)))
    d = np.genfromtxt(path, delimiter=',', skip_header=1)
    if d.ndim == 1:
        d = d.reshape(1, -1) if d.size else np.zeros((0, len(head)))
    return head, d


def rot_row3(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    r31 = 2 * (x * z - w * y)
    r32 = 2 * (y * z + w * x)
    r33 = 1 - 2 * (x * x + y * y)
    return r31, r32, r33


def longest_circular_run(b):
    if not b.any():
        return 0
    if b.all():
        return len(b)
    bb = np.concatenate([b, b])
    best = cur = 0
    for v in bb:
        cur = cur + 1 if v else 0
        best = max(best, cur)
    return min(best, len(b))


def floor_hit(ranges, r31, r32):
    z = r31 * np.cos(ANG) + r32 * np.sin(ANG)
    with np.errstate(divide='ignore', invalid='ignore'):
        pred = np.where(z < 0, H / -z, np.inf)
    has = pred <= RMAX
    ok = np.isfinite(ranges) & has & (np.abs(ranges - pred) <= TOL * pred)
    run = longest_circular_run(ok)
    return run >= MIN_RUN, run, int(has.sum()), pred


def match_with_pred(ranges, pred):
    has = pred <= RMAX
    ok = np.isfinite(ranges) & has & (np.abs(ranges - pred) <= TOL * pred)
    return longest_circular_run(ok) >= MIN_RUN


def braking_events(t, v):
    ev = []
    n = len(t)
    last = -1e9
    for i in range(n):
        j_end = np.searchsorted(t, t[i] + DROP_WIN, side='right')
        if j_end <= i + 1:
            continue
        w = v[i + 1:j_end]
        drop = v[i] - w.min()
        if drop >= DROP:
            lower = np.nonzero(w < v[i] - 1e-3)[0]
            te = t[i + 1 + lower[0]]
            if te - last < DROP_WIN:
                if drop > ev[-1][1]:
                    ev[-1] = (ev[-1][0], drop, ev[-1][2])
                continue
            ev.append((te, drop, v[i]))
            last = te
    return ev


def nav2_info(path):
    stops_wall, errors, sh = [], 0, False
    if not os.path.exists(path):
        return stops_wall, errors
    with open(path, errors='replace') as f:
        for line in f:
            if 'signal_handler' in line:
                break
            if 'Robot to stop' in line:
                m = TS_RE.search(line)
                stops_wall.append(float(m.group(1)) if m else float('nan'))
            if 'ERROR' in line:
                errors += 1
    return stops_wall, errors


def goal_info(path):
    status, nav_t = '-', '-'
    if not os.path.exists(path):
        return status, nav_t
    lines = open(path, errors='replace').read().splitlines()
    for ln in lines:
        if 'status:' in ln:
            status = ln.split()[-1]
    n = False
    for ln in lines:
        if 'navigation_time:' in ln:
            n = True
            continue
        if n and re.match(r'^\s*sec:', ln):
            nav_t = ln.split()[-1]
            n = False
    return status, nav_t


def analyse(logdir, prefix, k, out):
    base = os.path.join(logdir, f'{prefix}_{k:02d}')
    res = {'run': f'{k:02d}'}
    _, pose = load_csv(base + '_pose.csv')
    _, cmd = load_csv(base + '_cmdvel.csv')
    _, scan = load_csv(base + '_scan.csv')
    stops_wall, errors = nav2_info(base + '_nav2.log')
    status, nav_t = goal_info(base + '_goal.log')
    res.update(status=status, nav_t=nav_t, stops=len(stops_wall), errors=errors,
               n_pose=len(pose), n_cmd=len(cmd), n_scan=len(scan))
    if len(pose) == 0:
        out(f'Run {k:02d}: no pose data')
        return res

    ts, tw = pose[:, 0], pose[:, 1]
    r31, r32, r33 = rot_row3(pose[:, 5:9])
    p = np.degrees(-np.arcsin(np.clip(r31, -1, 1)))          # nose-down, + = nose down
    roll = np.degrees(np.arctan2(r32, r33))
    tilt = np.degrees(np.arccos(np.clip(r33, -1, 1)))
    i_max = int(np.argmax(p))
    i_min = int(np.argmin(p))
    res.update(p_max=p[i_max], p_max_t=ts[i_max], p_min=p[i_min], p_abs=np.abs(p).max(),
               tilt_max=tilt.max(), roll_absmax=np.abs(roll).max(), t0=ts[0], t1=ts[-1])

    # Braking events
    ev = []
    if len(cmd):
        m = np.isfinite(cmd[:, 0])
        tc, vc = cmd[m, 0], cmd[m, 2]
        order = np.argsort(tc, kind='stable')
        tc, vc = tc[order], vc[order]
        ev = braking_events(tc, vc)
    ev_rows = []
    during = False
    for te, drop, v0 in ev:
        win = (ts >= te - 0.2) & (ts <= te + 1.5)
        pk = p[win].max() if win.any() else float('nan')
        win2 = (ts >= te) & (ts <= te + 1.5)
        if win2.any() and p[win2].max() >= P_HIT:
            during = True
        ev_rows.append((te, drop, v0, pk))
    res.update(n_brake=len(ev), during=during)
    if ev_rows:
        hard = max(ev_rows, key=lambda r: r[1])
        res.update(hard_t=hard[0], hard_drop=hard[1], hard_p=hard[3])
    # Any p >= 1.5 at all, inside or outside braking
    res['p_hit_any'] = bool((p >= P_HIT).any())

    # Scans
    hits, tested, ctrl_tested, ctrl_hits, xctrl_hits, hit_rows = 0, 0, 0, 0, 0, []
    if len(scan):
        tsc = scan[:, 0]
        rng = scan[:, 2:2 + NBEAM]
        s31 = np.interp(tsc, ts, r31)
        s32 = np.interp(tsc, ts, r32)
        s33 = np.interp(tsc, ts, r33)
        stilt = np.degrees(np.arccos(np.clip(s33, -1, 1)))
        sp = np.degrees(-np.arcsin(np.clip(s31, -1, 1)))
        flat_idx = np.nonzero(stilt < P_CTRL)[0]
        for i in range(len(scan)):
            if stilt[i] >= P_HIT:
                tested += 1
                hit, run, npred, pred = floor_hit(rng[i], s31[i], s32[i])
                if hit:
                    hits += 1
                    hit_rows.append((tsc[i], sp[i], stilt[i], run, npred))
                prev = flat_idx[flat_idx < i]
                if len(prev) and match_with_pred(rng[prev[-1]], pred):
                    xctrl_hits += 1
            elif stilt[i] < P_CTRL:
                ctrl_tested += 1
                hit, *_ = floor_hit(rng[i], s31[i], s32[i])
                ctrl_hits += int(hit)
    res.update(tested=tested, hits=hits, ctrl_tested=ctrl_tested, ctrl_hits=ctrl_hits,
               xctrl_hits=xctrl_hits)

    # Stops -> sim time (via recorder's wall/sim pairs)
    stop_rows = []
    for w in stops_wall:
        st = float(np.interp(w, tw, ts)) if tw[0] <= w <= tw[-1] else float('nan')
        if math.isfinite(st):
            near = (ts >= st - 1.0) & (ts <= st + 0.2)
            pk = p[near].max() if near.any() else float('nan')
        else:
            pk = float('nan')
        stop_rows.append((w, st, pk))

    out(f'--- Run {k:02d}: {status}, nav time {nav_t} s, pose {len(pose)} / cmd {len(cmd)} / scan {len(scan)} rows, '
        f'sim {ts[0]:.2f}-{ts[-1]:.2f} s')
    out(f'  nose-down p: max {p[i_max]:.3f} deg at sim {ts[i_max]:.3f}; min {p[i_min]:.3f} deg; '
        f'max tilt {tilt.max():.3f} deg; max |roll| {np.abs(roll).max():.3f} deg')
    out(f'  braking events: {len(ev)}')
    for te, drop, v0, pk in ev_rows:
        out(f'    t={te:8.3f}  v0={v0:.3f}  drop={drop:.3f} m/s  max p [t-0.2,t+1.5] = {pk:.3f} deg')
    out(f'  scans tilt>=1.5: {tested}, floor-hit: {hits}; control tilt<0.5: {ctrl_hits}/{ctrl_tested}; '
        f'extra control: {xctrl_hits}/{tested}')
    for t, spv, stv, run, npred in hit_rows[:20]:
        out(f'    floor-hit scan t={t:.3f} p={spv:.3f} tilt={stv:.3f} run={run} beams-with-prediction={npred}')
    for w, st, pk in stop_rows:
        out(f'  stop: wall {w:.3f} -> sim {st:.3f}; max p in [-1.0,+0.2] s = {pk:.3f} deg')
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefix')
    ap.add_argument('n', type=int)
    ap.add_argument('--logdir', default=os.path.expanduser('~/run_logs'))
    a = ap.parse_args()
    out_path = os.path.join(a.logdir, f'{a.prefix}_analysis.txt')
    try:
        fo = open(out_path, 'x')
    except FileExistsError:
        sys.exit(f'STOP: {out_path} exists, not overwriting')

    def out(s=''):
        print(s)
        fo.write(s + '\n')

    rows = [analyse(a.logdir, a.prefix, k, out) for k in range(1, a.n + 1)]

    def f(v, fmt='{:.2f}'):
        return fmt.format(v) if isinstance(v, (int, float)) and math.isfinite(v) else '-'

    out()
    out(f'{"run":<4}{"status":<11}{"time":>5}{"stops":>6}{"max_p":>8}{"at_t":>9}{"p_hard":>8}'
        f'{"drop":>6}{"brakes":>7}{"hits/tested":>13}{"tilt":>7}{"ctrl":>8}{"xctrl":>7}')
    for r in rows:
        out(f'{r["run"]:<4}{r["status"]:<11}{r["nav_t"]:>5}{r["stops"]:>6}'
            f'{f(r.get("p_max", float("nan"))):>8}{f(r.get("p_max_t", float("nan"))):>9}'
            f'{f(r.get("hard_p", float("nan"))):>8}{f(r.get("hard_drop", float("nan"))):>6}'
            f'{r.get("n_brake", 0):>7}{str(r.get("hits", 0)) + "/" + str(r.get("tested", 0)):>13}'
            f'{f(r.get("tilt_max", float("nan"))):>7}'
            f'{str(r.get("ctrl_hits", 0)) + "/" + str(r.get("ctrl_tested", 0)):>8}'
            f'{str(r.get("xctrl_hits", 0)) + "/" + str(r.get("tested", 0)):>7}')

    supported = any(r.get('during') for r in rows)
    strong = supported and sum(r.get('hits', 0) for r in rows) >= 1
    not_sup = all(r.get('p_abs', float('inf')) < P_CTRL for r in rows)
    out()
    out(f'Supported (p >= 1.5 deg during braking in >= 1 run): {"YES" if supported else "NO"}')
    out(f'Strongly supported (also >= 1 floor-hit scan):        {"YES" if strong else "NO"}')
    out(f'Not supported (max |p| < 0.5 deg in all runs):         {"YES" if not_sup else "NO"}')
    verdict = ('STRONGLY SUPPORTED' if strong else 'SUPPORTED' if supported
               else 'NOT SUPPORTED' if not_sup else 'INCONCLUSIVE')
    out(f'Verdict: {verdict}')
    fo.close()
    print(f'(written to {out_path})')


if __name__ == '__main__':
    main()
