#!/usr/bin/env python3
"""Analyse a perception run recorded by scripts/perception_record.py.

Usage: python3 scripts/perception_analyze.py RUN_ID [--logdir DIR]

Reads (never writes) LOGDIR/RUN_ID_{box,scanstamps,det,robot,cpu}.csv.
Writes LOGDIR/RUN_ID_perception_analysis.txt with mode 'x' (never overwrites) and prints it.

Pass rule (fixed before any data; 3 min sim time, robot idle so odom = world, gz true box pose):
  a. box detected (a detection within 0.25 m of the true centre) in >= 95 % of scans
  b. scans with any detection farther than 0.5 m from the box <= 1 %
     (detections within 0.3 m of the partition end (0.0, 1.0) do not count; reported separately)
  c. detector CPU <= 10 % of one core
Window = WINDOW sim seconds from the first box movement (|dy/dt| > 0.05 m/s over >= 0.2 s).
Scans in the window = scan stamps in the window. A scan with no det row counts as a miss for (a)
(and is reported: it means the node published nothing, e.g. dropped by the TF filter).
CPU % = delta(utime+stime)/CLK_TCK / delta(wall) * 100 over the window (whole process, all
threads); ticks and wall are interpolated at the window start/end by sim time.
Collisions are not checked.
"""
import argparse
import math
import os
import sys

import numpy as np

WINDOW = 180.0
HIT_R, FAR_R, PART_R = 0.25, 0.5, 0.3
PARTITION = (0.0, 1.0)
HIT_MIN, FAR_MAX, CPU_MAX = 95.0, 1.0, 10.0
MOVE_V, MOVE_DT = 0.05, 0.2


def read_rows(path):
    invalid, rows = None, []
    with open(path) as f:
        f.readline()
        for line in f:
            if line.startswith('#INVALID'):
                invalid = line.strip()
            elif line.strip():
                rows.append(line.strip().split(','))
    return rows, invalid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_id')
    ap.add_argument('--logdir', default=os.path.expanduser('~/run_logs'))
    args = ap.parse_args()
    base = os.path.join(args.logdir, args.run_id)
    out_path = base + '_perception_analysis.txt'
    if os.path.exists(out_path):
        sys.exit(f'STOP: {out_path} exists, not overwriting')

    L = []
    say = L.append
    problems = []
    brow, binv = read_rows(base + '_box.csv')
    srow, sinv = read_rows(base + '_scanstamps.csv')
    drow, dinv = read_rows(base + '_det.csv')
    rrow, _ = read_rows(base + '_robot.csv')
    crow, cinv = read_rows(base + '_cpu.csv')
    for inv in (binv, sinv, dinv, cinv):
        if inv:
            problems.append(f'recorder marked run invalid: {inv}')

    B = np.array([[float(v) for v in r] for r in brow])
    t, wall, bx, by = B[:, 0], B[:, 1], B[:, 2], B[:, 3]
    say(f'Run {args.run_id}: {len(t)} box rows, {len(srow)} scan stamps, {len(drow)} det rows, '
        f'{len(crow)} cpu rows')

    # robot idle, odom = world
    rob = {r[0]: [float(v) for v in r[1:]] for r in rrow}
    if 'start' not in rob or 'end' not in rob or any(math.isnan(v) for v in rob['end']):
        problems.append('robot pose missing at start or end')
    else:
        s, e = rob['start'], rob['end']
        d = math.hypot(e[0] - s[0], e[1] - s[1])
        dyaw = abs(e[3] - s[3])
        say(f'Robot start ({s[0]:.4f}, {s[1]:.4f}) yaw {s[3]:.3f} deg; end moved {d:.4f} m, '
            f'yaw changed {dyaw:.3f} deg')
        if d > 0.01 or dyaw > 0.5:
            problems.append('robot moved during run')
        if math.hypot(s[0], s[1]) > 0.01 or abs(s[3]) > 0.5:
            problems.append('robot start pose is not (0, 0, 0 deg): odom != world')

    # window start (same detector as the recorder)
    ws, j = None, 0
    for i in range(len(t)):
        while j < i and t[j + 1] <= t[i] - MOVE_DT:
            j += 1
        if t[i] - t[j] >= MOVE_DT and abs(by[i] - by[j]) / (t[i] - t[j]) > MOVE_V:
            ws = t[i]
            break
    if ws is None:
        problems.append('box never moved')
    else:
        we = ws + WINDOW
        covered = t[-1] - ws
        say(f'Window: sim t {ws:.2f} to {we:.2f} (data covers {covered:.1f} s)')
        if covered < WINDOW:
            problems.append(f'window short: {covered:.1f} s of {WINDOW:.0f} s')

    res = {}
    if ws is not None:
        det = {}
        for r in drow:
            det[int(r[0])] = [(float(r[2 + 2 * k]), float(r[3 + 2 * k])) for k in range(int(r[1]))]
        n = hit = far = excl = nodet = 0
        worst_hit = 0.0
        n_det_pts = 0
        for r in srow:
            ns = int(r[0])
            ts = ns * 1e-9
            if not (ws <= ts <= we):
                continue
            n += 1
            cx, cy = np.interp(ts, t, bx), np.interp(ts, t, by)
            if ns not in det:
                nodet += 1
                continue
            pts = det[ns]
            n_det_pts += len(pts)
            dist = [math.hypot(x - cx, y - cy) for x, y in pts]
            if dist and min(dist) <= HIT_R:
                hit += 1
                worst_hit = max(worst_hit, min(dist))
            bad = [(x, y) for (x, y), dd in zip(pts, dist) if dd > FAR_R]
            if bad:
                real = [p for p in bad if math.hypot(p[0] - PARTITION[0], p[1] - PARTITION[1]) > PART_R]
                if real:
                    far += 1
                else:
                    excl += 1
        if n == 0:
            problems.append('no scans in the window')
        else:
            hp, fp = 100.0 * hit / n, 100.0 * far / n
            res['a'] = hp >= HIT_MIN
            res['b'] = fp <= FAR_MAX
            say(f'a. box detected: {hit}/{n} scans = {hp:.2f} % (need >= {HIT_MIN}) '
                f'-> {"PASS" if res["a"] else "FAIL"}   (worst hit distance {worst_hit:.3f} m; '
                f'scans with no det row: {nodet})')
            say(f'b. scans with a detection > {FAR_R} m from the box: {far}/{n} = {fp:.2f} % '
                f'(need <= {FAR_MAX}) -> {"PASS" if res["b"] else "FAIL"}')
            say(f'   not counted: {excl} scans whose only far detections were within {PART_R} m of '
                f'the partition end {PARTITION}; {n_det_pts} detections in total')

        # c. CPU
        C = np.array([[float(v) for v in r] for r in crow]) if crow else np.zeros((0, 3))
        if len(C) < 2 or C[0, 1] > ws or C[-1, 1] < we:
            problems.append('cpu rows do not cover the window')
        else:
            tk = lambda s: np.interp(s, C[:, 1], C[:, 2])
            wl = lambda s: np.interp(s, C[:, 1], C[:, 0])
            hz = os.sysconf('SC_CLK_TCK')
            cpu = 100.0 * (tk(we) - tk(ws)) / hz / (wl(we) - wl(ws))
            res['c'] = cpu <= CPU_MAX
            say(f'c. detector CPU: {cpu:.2f} % of one core over the window (need <= {CPU_MAX}) '
                f'-> {"PASS" if res["c"] else "FAIL"}   (CLK_TCK {hz}, whole process)')

        w = (t >= ws) & (t <= we)
        dsim, dwall = t[w][-1] - t[w][0], wall[w][-1] - wall[w][0]
        say(f'sim speed: {dsim:.2f} sim s in {dwall:.2f} wall s = {dsim / dwall:.3f}x')

    if problems:
        verdict = 'INVALID'
        say('Problems: ' + '; '.join(problems))
    elif len(res) == 3 and all(res.values()):
        verdict = 'PASS'
    else:
        verdict = 'FAIL'
    say(f'VERDICT: {verdict}')
    say('Collisions are not checked here.')

    text = '\n'.join(L) + '\n'
    with open(out_path, 'x') as f:
        f.write(text)
    print(text)
    print('Written:', out_path)


if __name__ == '__main__':
    main()
