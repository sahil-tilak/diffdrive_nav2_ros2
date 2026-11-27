#!/usr/bin/env python3
"""Numbers for one run of the with/without test, from the CSV files of ab_record.py.

Usage: python3 scripts/ab_analyze.py RUN_ID --stops N [--outdir DIR]

Writes OUTDIR/RUN_ID_summary.txt ('key=value' lines, mode 'x') and prints it.
--stops is the collision_monitor stop count that ab_batch.sh took from the launch log.

Rules (set before any run):
  window            goal accepted -> goal result (sim time)
  min distance      centre to centre, gz true poses, each robot sample paired with the box
                    sample nearest in sim time
  contact           /collision from the TouchPlugin; 'yes' only if it happened in the window
  non-box track     a track id whose MEDIAN distance to the true box position (at the track
                    stamp) is >= 0.5 m. Assumes odom = world; see odom_drift_m.
  box delay         first box movement (|dy/dt| > 0.05 m/s for >= 0.2 sim s) minus goal
                    accepted, about +-0.05 s (20 Hz pose)
  plan side         sign of the plan point with the largest |y|; 'straight' if under 0.05 m
"""
import argparse
import bisect
import collections
import csv
import json
import math
import os
import statistics
import sys

BOX_TRACK_DIST = 0.5
MOVE_V = 0.05
MOVE_DT = 0.2
STRAIGHT_Y = 0.05
CONFLICT_DIST = 0.4      # NOTES.md calibration: contact, or centre distance under this


def read(outdir, run_id, name):
    with open(os.path.join(outdir, f'{run_id}_{name}.csv'), newline='') as f:
        return list(csv.DictReader(f))


class Series:
    """Samples sorted by sim time, looked up by nearest time."""

    def __init__(self, rows):
        self.data = sorted((float(r['sim_t']), float(r['x']), float(r['y'])) for r in rows)
        self.t = [d[0] for d in self.data]

    def nearest(self, t):
        i = bisect.bisect_left(self.t, t)
        near = [j for j in (i - 1, i) if 0 <= j < len(self.t)]
        return self.data[min(near, key=lambda j: abs(self.t[j] - t))]


def first_movement(box):
    start = None
    for (t0, _, y0), (t1, _, y1) in zip(box.data, box.data[1:]):
        if t1 <= t0:
            continue
        if abs(y1 - y0) / (t1 - t0) > MOVE_V:
            start = t0 if start is None else start
            if t1 - start >= MOVE_DT:
                return start
        else:
            start = None
    return None


def side(points):
    y = max((p[1] for p in points), key=abs)
    name = 'straight' if abs(y) < STRAIGHT_Y else ('+y' if y > 0 else '-y')
    return name, y


def fmt(v, nd=3):
    return '-' if v is None else f'{v:.{nd}f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_id')
    ap.add_argument('--stops', type=int, required=True)
    ap.add_argument('--outdir', default=os.path.expanduser('~/run_logs'))
    args = ap.parse_args()
    out_path = os.path.join(args.outdir, f'{args.run_id}_summary.txt')
    if os.path.exists(out_path):
        print('STOP: already exists:', out_path)
        sys.exit(2)

    events = collections.OrderedDict()
    for r in read(args.outdir, args.run_id, 'events'):
        events.setdefault(r['event'], r)

    def detail(name):
        return json.loads(events[name]['detail']) if name in events else {}

    if 'goal_accepted' not in events or 'result' not in events:
        print('STOP: no goal_accepted or result event in this run')
        sys.exit(1)
    t_acc = float(events['goal_accepted']['sim_t'])
    t_res = float(events['result']['sim_t'])
    res = detail('result')

    # contact
    contact, contact_t = 'no', None
    if 'collision' in events:
        info = detail('collision')
        contact_t = info.get('sim_time') or float(events['collision']['sim_t'])
        contact = 'yes' if contact_t <= t_res + 0.1 else 'after_goal'

    status = res['status']
    if status == 'SUCCEEDED':
        result = 'succeeded'
    elif status == 'CANCELED' and contact == 'yes':
        result = 'cancelled_by_contact'
    elif res['timed_out']:
        result = 'timeout'
    elif status == 'ABORTED':
        result = 'aborted'
    else:
        result = f'other_{status}'

    # distance
    robot = Series(read(args.outdir, args.run_id, 'robot'))
    box = Series(read(args.outdir, args.run_id, 'box'))
    best, best_t, gap, n = None, None, 0.0, 0
    for t, x, y in robot.data:
        if not t_acc <= t <= t_res:
            continue
        bt, bx, by = box.nearest(t)
        gap = max(gap, abs(bt - t))
        n += 1
        d = math.hypot(x - bx, y - by)
        if best is None or d < best:
            best, best_t = d, t
    conflict = contact == 'yes' or (best is not None and best < CONFLICT_DIST)

    # box start delay
    moved = first_movement(box)
    meas = None if moved is None else moved - t_acc
    trig = detail('trigger_response')

    # tracks
    per_id = collections.defaultdict(list)
    lags = []
    for r in read(args.outdir, args.run_id, 'tracks'):
        recv, stamp = float(r['recv_sim_t']), float(r['stamp'])
        if not t_acc <= recv <= t_res:
            continue
        lags.append(recv - stamp)
        _, bx, by = box.nearest(stamp)
        per_id[int(r['id'])].append(math.hypot(float(r['x']) - bx, float(r['y']) - by))
    med = {i: statistics.median(v) for i, v in per_id.items()}
    non_box = sorted(i for i, d in med.items() if d >= BOX_TRACK_DIST)

    # plans
    plans = collections.OrderedDict()
    for r in read(args.outdir, args.run_id, 'plans'):
        if t_acc <= float(r['recv_sim_t']) <= t_res:
            plans.setdefault(int(r['plan_n']), []).append((float(r['x']), float(r['y'])))
    sides = [side(p) for p in plans.values()]

    odom = detail('odom_check')
    start, end = detail('start'), detail('end')
    lines = collections.OrderedDict([
        ('run', args.run_id),
        ('result', result),
        ('nav2_status', status),
        ('contact', contact),
        ('contact_sim_t_after_accept', fmt(None if contact_t is None else contact_t - t_acc)),
        ('time_to_goal_sim_s', fmt(t_res - t_acc, 2)),
        ('time_to_goal_wall_s', fmt(res['wall_s'], 2)),
        ('min_centre_dist_m', fmt(best)),
        ('min_dist_sim_t_after_accept', fmt(None if best_t is None else best_t - t_acc, 2)),
        ('conflict_contact_or_under_0.4', 'yes' if conflict else 'no'),
        ('robot_samples_in_window', n),
        ('max_pose_pair_gap_s', fmt(gap)),
        ('collision_monitor_stops', args.stops),
        ('box_delay_requested_sim_s', fmt(start.get('delay_requested'), 2)),
        ('box_delay_measured_sim_s', fmt(meas)),
        ('trigger_latency_wall_s', fmt(trig.get('latency_wall_s'))),
        ('track_ids_total', len(med)),
        ('track_ids_not_box', len(non_box)),
        ('track_median_dist_by_id',
         ' '.join(f'{i}:{d:.2f}' for i, d in sorted(med.items())) or '-'),
        ('track_recv_minus_stamp_median_s',
         fmt(statistics.median(lags) if lags else None)),
        ('global_plans', len(plans)),
        ('first_plan_side', f'{sides[0][0]} (y={sides[0][1]:.2f})' if sides else '-'),
        ('last_plan_side', f'{sides[-1][0]} (y={sides[-1][1]:.2f})' if sides else '-'),
        ('plan_side_counts',
         ' '.join(f'{k}:{v}' for k, v in collections.Counter(s for s, _ in sides).items())
         or '-'),
        ('odom_drift_m', fmt(odom.get('dist_m'))),
        ('odom_drift_yaw_deg', fmt(odom.get('yaw_diff_deg'), 2)),
        ('recorder_cpu_pct', end.get('recorder_cpu_pct', '-')),
        ('load_start', start.get('load', '-')),
        ('load_end', end.get('load', '-')),
    ])
    text = ''.join(f'{k}={v}\n' for k, v in lines.items())
    with open(out_path, 'x') as f:
        f.write(text)
    print(text, end='')


if __name__ == '__main__':
    main()
