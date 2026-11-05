#!/usr/bin/env python3
"""Analyse a moving-obstacle run recorded by scripts/obstacle_record.py.

Usage: python3 scripts/obstacle_analyze.py RUN_ID [--logdir DIR]

Reads (never writes) LOGDIR/RUN_ID_obstacle_{pose,scan,robot}.csv.
Writes LOGDIR/RUN_ID_obstacle_analysis.txt with mode 'x' (never overwrites) and prints it.

Pass rule (fixed before any data; 3 min sim time, world frame, gz true pose):
  1. x stays within 1.5 +/- 0.05 m
  2. each turnaround within +/- 0.1 m of y = +/-1.0
  3. roll and pitch < 1 deg
  4. box visible in /scan at the expected range +/- 0.1 m when in front of the robot
  5. sim speed >= 0.80x real time from raw timestamps
Window = WINDOW sim seconds from the first box movement (|dy/dt| > 0.05 m/s over >= 0.2 s).

Definitions:
- Turnaround: direction = sign of dy over the last >= 0.2 s (held while |v| < 0.05). At each
  direction change, turnaround y = max y (was moving +) or min y (was moving -) among samples
  within +/- 0.5 s of the change. Expected +1.0 / -1.0. The start (y = -1) and a last partial
  run are not turnarounds.
- In front: bearing of the box centre from the robot's start pose is within +/- 45 deg of its
  heading. Per scan: box x, y, measured yaw interpolated to the scan stamp; beam i has angle
  -3.14159 + i * 6.28318 / 359 (increment convention unverified, < 1 deg effect); the expected
  range is that beam's ray vs the 0.3 x 0.3 square rotated by the measured yaw, from the
  lidar xy (= robot origin xy). The beam used is the one nearest the centre bearing.
- Speed: (sim_last - sim_first) / (wall_last - wall_first) over the box pose rows in the window.
"""
import argparse
import math
import os
import sys

import numpy as np

WINDOW = 180.0
X0, X_TOL = 1.5, 0.05
Y_EDGE, Y_TOL = 1.0, 0.1
TILT_MAX = 1.0
RANGE_TOL = 0.1
RTF_MIN = 0.80
BOX_HALF = 0.15
FRONT = math.radians(45)
MOVE_V, MOVE_DT = 0.05, 0.2
AMIN, AMAX, NBEAM = -3.14159, 3.14159, 360
ANG = AMIN + np.arange(NBEAM) * (AMAX - AMIN) / (NBEAM - 1)


def read_csv(path):
    invalid = None
    rows = []
    with open(path) as f:
        f.readline()
        for line in f:
            if line.startswith('#INVALID'):
                invalid = line.strip()
            elif line.strip():
                rows.append(line.strip().split(','))
    return rows, invalid


def ray_vs_square(ox, oy, ang, cx, cy, yaw, half):
    """Distance along a ray from (ox, oy) at angle ang to a square centred (cx, cy) rotated by yaw.
    Returns nan if the ray misses."""
    c, s = math.cos(-yaw), math.sin(-yaw)
    # to box frame
    px, py = ox - cx, oy - cy
    ox_b, oy_b = c * px - s * py, s * px + c * py
    dx, dy = math.cos(ang), math.sin(ang)
    dx_b, dy_b = c * dx - s * dy, s * dx + c * dy
    tmin, tmax = -math.inf, math.inf
    for o, d in ((ox_b, dx_b), (oy_b, dy_b)):
        if abs(d) < 1e-12:
            if abs(o) > half:
                return math.nan
            continue
        t1, t2 = (-half - o) / d, (half - o) / d
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    if tmax < max(tmin, 0.0):
        return math.nan
    return tmin if tmin > 0 else tmax


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_id')
    ap.add_argument('--logdir', default=os.path.expanduser('~/run_logs'))
    args = ap.parse_args()
    base = os.path.join(args.logdir, args.run_id + '_obstacle')
    out_path = base + '_analysis.txt'
    if os.path.exists(out_path):
        sys.exit(f'STOP: {out_path} exists, not overwriting')

    L = []
    def say(s=''):
        L.append(s)

    prow, pinv = read_csv(base + '_pose.csv')
    srow, sinv = read_csv(base + '_scan.csv')
    rrow, _ = read_csv(base + '_robot.csv')
    P = np.array([[float(v) for v in r] for r in prow])
    t, wall, x, y, z = P[:, 0], P[:, 1], P[:, 2], P[:, 3], P[:, 4]
    qx, qy, qz, qw = P[:, 5], P[:, 6], P[:, 7], P[:, 8]
    roll = np.degrees(np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1)))
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    say(f'Run {args.run_id}: {len(t)} box pose rows, {len(srow)} scan rows')
    problems = []
    if pinv or sinv:
        problems.append(f'recorder marked run invalid: {pinv or sinv}')

    # robot idle check
    rob = {r[0]: [float(v) for v in r[1:]] for r in rrow}
    if 'start' not in rob or 'end' not in rob or any(math.isnan(v) for v in rob['end']):
        problems.append('robot pose missing at start or end')
        rx, ry, ryaw = 0.0, 0.0, 0.0
    else:
        s, e = rob['start'], rob['end']
        d = math.hypot(e[0] - s[0], e[1] - s[1])
        dyaw = abs(e[3] - s[3])
        say(f'Robot start ({s[0]:.4f}, {s[1]:.4f}) yaw {s[3]:.3f} deg; end moved {d:.4f} m, '
            f'yaw changed {dyaw:.3f} deg')
        if d > 0.01 or dyaw > 0.5:
            problems.append('robot moved during run')
        rx, ry, ryaw = s[0], s[1], math.radians(s[3])

    # window start: same detector as the recorder
    ws = None
    j = 0
    for i in range(len(t)):
        while j < i and t[j + 1] <= t[i] - MOVE_DT:
            j += 1
        if t[i] - t[j] >= MOVE_DT and abs(y[i] - y[j]) / (t[i] - t[j]) > MOVE_V:
            ws = t[i]
            break
    if ws is None:
        say('Box never moved. RUN INVALID.')
        verdict = 'INVALID'
        problems.append('box never moved')
        w = np.zeros(len(t), bool)
    else:
        w = (t >= ws) & (t <= ws + WINDOW)
        covered = t[-1] - ws
        say(f'Window: sim t {ws:.2f} to {ws + WINDOW:.2f} (data covers {covered:.1f} s), {w.sum()} rows')
        if covered < WINDOW:
            problems.append(f'window short: {covered:.1f} s of {WINDOW:.0f} s')

    results = {}
    if w.any():
        tw, yw, xw = t[w], y[w], x[w]
        # 1. x
        dx = np.abs(xw - X0).max()
        results[1] = dx <= X_TOL
        say(f'1. x: max |x - {X0}| = {dx:.4f} m (limit {X_TOL}) -> {"PASS" if results[1] else "FAIL"}')

        # 2. turnarounds
        direction = np.zeros(len(tw), int)
        cur = 0
        j = 0
        for i in range(len(tw)):
            while j < i and tw[j + 1] <= tw[i] - MOVE_DT:
                j += 1
            if tw[i] - tw[j] >= MOVE_DT:
                v = (yw[i] - yw[j]) / (tw[i] - tw[j])
                if abs(v) >= MOVE_V:
                    cur = 1 if v > 0 else -1
            direction[i] = cur
        turns = []   # (time, y, expected)
        prev_dir, prev_i = 0, 0
        for i in range(len(tw)):
            if direction[i] != 0 and direction[i] != prev_dir:
                if prev_dir != 0:
                    m = np.abs(tw - tw[i]) <= 0.5
                    if prev_dir > 0:
                        turns.append((tw[i], yw[m].max(), Y_EDGE))
                    else:
                        turns.append((tw[i], yw[m].min(), -Y_EDGE))
                prev_dir = direction[i]
        if turns:
            devs = [abs(ty - ex) for _, ty, ex in turns]
            worst = max(devs)
            results[2] = worst <= Y_TOL
            say(f'2. turnarounds: {len(turns)} found, worst |y - target| = {worst:.4f} m '
                f'(limit {Y_TOL}) -> {"PASS" if results[2] else "FAIL"}')
            over = [(ty - ex) * (1 if ex > 0 else -1) for _, ty, ex in turns]
            say(f'   overshoot beyond +/-1.0 (+ = past the end): min {min(over):+.4f}, max {max(over):+.4f} m')
            say('   ' + ' '.join(f'{ty:+.3f}' for _, ty, _ in turns))
        else:
            results[2] = False
            say('2. turnarounds: none found -> FAIL')

        # 3. tilt
        mr, mp = np.abs(roll[w]).max(), np.abs(pitch[w]).max()
        results[3] = mr < TILT_MAX and mp < TILT_MAX
        say(f'3. tilt: max |roll| {mr:.4f} deg, max |pitch| {mp:.4f} deg (limit {TILT_MAX}) '
            f'-> {"PASS" if results[3] else "FAIL"}')
        say(f'   max |yaw| {np.degrees(np.abs(yaw[w])).max():.4f} deg (used in the scan check)')

        # 4. scan
        n_front = n_bad = n_nan = 0
        worst_err = 0.0
        worst_info = ''
        yaw_u = np.unwrap(yaw)
        for r in srow:
            ts = float(r[0])
            if not (tw[0] <= ts <= tw[-1]):
                continue
            bx, by = np.interp(ts, t, x), np.interp(ts, t, y)
            byaw = float(np.interp(ts, t, yaw_u))
            bearing = math.atan2(by - ry, bx - rx) - ryaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if abs(bearing) > FRONT:
                continue
            n_front += 1
            i = int(np.argmin(np.abs(ANG - bearing)))
            exp = ray_vs_square(rx, ry, ANG[i] + ryaw, bx, by, byaw, BOX_HALF)
            meas = float(r[2 + i])
            if math.isnan(exp):
                n_bad += 1
                worst_info = f'beam {i} misses the box at t={ts:.2f}'
                continue
            if not math.isfinite(meas):
                n_nan += 1
                n_bad += 1
                worst_info = f'no return on beam {i} at t={ts:.2f}, expected {exp:.3f}'
                continue
            err = abs(meas - exp)
            if err > RANGE_TOL:
                n_bad += 1
            if err > worst_err:
                worst_err = err
                worst_info = f't={ts:.2f} beam {i} measured {meas:.3f} expected {exp:.3f}'
        results[4] = n_front > 0 and n_bad == 0
        say(f'4. scan: {n_front} scans with the box in front, {n_bad} outside +/-{RANGE_TOL} m '
            f'({n_nan} with no return), worst error {worst_err:.4f} m ({worst_info}) '
            f'-> {"PASS" if results[4] else "FAIL"}')

        # 5. speed
        dsim, dwall = tw[-1] - tw[0], wall[w][-1] - wall[w][0]
        rtf = dsim / dwall if dwall > 0 else float('nan')
        results[5] = rtf >= RTF_MIN
        say(f'5. sim speed: {dsim:.2f} sim s in {dwall:.2f} wall s = {rtf:.3f}x '
            f'(limit {RTF_MIN}) -> {"PASS" if results[5] else "FAIL"}')
        gaps = np.diff(tw)
        say(f'   pose rate {len(tw) / dsim:.1f} Hz, longest gap {gaps.max():.3f} sim s')

    if problems:
        verdict = 'INVALID'
        say('Problems: ' + '; '.join(problems))
    elif len(results) == 5 and all(results.values()):
        verdict = 'PASS'
    else:
        verdict = 'FAIL'
    say(f'VERDICT: {verdict}')
    say('Collisions (box vs robot) are not checked here.')

    text = '\n'.join(L) + '\n'
    with open(out_path, 'x') as f:
        f.write(text)
    print(text)
    print('Written:', out_path)


if __name__ == '__main__':
    main()
