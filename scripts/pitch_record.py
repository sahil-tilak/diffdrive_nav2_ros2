#!/usr/bin/env python3
"""Record Gazebo true pose, /cmd_vel and /scan for the pitch test.

Usage: python3 scripts/pitch_record.py PREFIX N [--outdir DIR] [--max-wall SEC]

One "session" = one Gazebo run. A session starts on the first pose message
and ends after GAP seconds (wall) without pose messages, or when sim time
goes backwards. Session k writes:
  OUTDIR/PREFIX_KK_pose.csv    sim_t, wall_t, x, y, z, qx, qy, qz, qw
  OUTDIR/PREFIX_KK_cmdvel.csv  sim_t (latest pose sim time), wall_t, linear_x, angular_z
  OUTDIR/PREFIX_KK_scan.csv    sim_t (header stamp), wall_t, r0..r359
Files are opened with mode 'x' (never overwritten). Exits after N sessions or
--max-wall seconds.

The command line of this process must not contain "gz", "rviz" or
"component_container" (scripts/run_batch.sh treats those as leftovers).
"""
import argparse
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node as RosNode
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

WORLD = 'empty_world'
MODEL = 'diffdrive_robot'
GAP = 5.0


class Session:
    def __init__(self, outdir, prefix, k):
        base = os.path.join(outdir, f'{prefix}_{k:02d}')
        self.paths = [base + s for s in ('_pose.csv', '_cmdvel.csv', '_scan.csv')]
        # 'x' = fail if the file exists; never overwrite.
        self.pose = open(self.paths[0], 'x')
        self.cmd = open(self.paths[1], 'x')
        self.scan = open(self.paths[2], 'x')
        self.pose.write('sim_t,wall_t,x,y,z,qx,qy,qz,qw\n')
        self.cmd.write('sim_t,wall_t,linear_x,angular_z\n')
        self.scan.write('sim_t,wall_t,' + ','.join(f'r{i}' for i in range(360)) + '\n')
        self.n = {'pose_msgs': 0, 'pose_rows': 0, 'cmd': 0, 'scan': 0}
        self.first_sim = None
        self.last_sim = None
        self.first_wall = time.time()
        self.last_pose_wall = time.time()

    def close(self):
        for f in (self.pose, self.cmd, self.scan):
            f.close()


class Recorder:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.session = None
        self.k = 0
        self.done = 0
        self.clock = float('nan')  # latest pose sim time; stamps cmd_vel (no /clock sub: ~1 kHz, too much CPU)
        self.failed = False
        self.warned_missing = False

        self.ros = RosNode('pitch_recorder')
        self.ros.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.ros.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.gz = GzNode()
        topic = f'/world/{WORLD}/pose/info'
        if not self.gz.subscribe(Pose_V, topic, self.on_pose):
            print(f'ERROR: cannot subscribe to {topic}', flush=True)
            self.failed = True
        else:
            print(f'Subscribed to {topic}', flush=True)

    # ---- session handling (call with lock held) ----
    def start_session(self):
        self.k += 1
        try:
            self.session = Session(self.args.outdir, self.args.prefix, self.k)
        except FileExistsError as e:
            print(f'STOP: file exists, not overwriting: {e.filename}', flush=True)
            self.failed = True
            self.session = None
            return
        print(f'Session {self.k} started', flush=True)

    def end_session(self, why):
        s = self.session
        if s is None:
            return
        s.close()
        sim_span = (s.last_sim - s.first_sim) if s.first_sim is not None else 0.0
        wall_span = s.last_pose_wall - s.first_wall
        rate_sim = s.n['pose_msgs'] / sim_span if sim_span > 0 else float('nan')
        rate_wall = s.n['pose_msgs'] / wall_span if wall_span > 0 else float('nan')
        print(f'Session {self.k} ended ({why}): {s.n}, sim span {sim_span:.2f} s, '
              f'wall span {wall_span:.2f} s, pose msgs/sim-s {rate_sim:.1f}, '
              f'pose msgs/wall-s {rate_wall:.1f}', flush=True)
        self.session = None
        self.done += 1

    # ---- callbacks ----
    def on_pose(self, msg):
        sim_t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        now = time.time()
        with self.lock:
            if self.failed or self.done >= self.args.n:
                return
            s = self.session
            if s is not None and s.last_sim is not None and sim_t < s.last_sim - 1.0:
                self.end_session('sim time went backwards')
                s = None
            if s is None:
                if self.done >= self.args.n:
                    return
                self.start_session()
                s = self.session
                if s is None:
                    return
                names = [p.name for p in msg.pose]
                print(f'  first msg: {len(names)} entries, {MODEL} present: {MODEL in names}',
                      flush=True)
            s.n['pose_msgs'] += 1
            s.last_pose_wall = now
            if s.first_sim is None:
                s.first_sim = sim_t
            s.last_sim = sim_t
            self.clock = sim_t
            for p in msg.pose:
                if p.name == MODEL:
                    o, q = p.position, p.orientation
                    s.pose.write(f'{sim_t:.6f},{now:.6f},{o.x:.6f},{o.y:.6f},{o.z:.6f},'
                                 f'{q.x:.8f},{q.y:.8f},{q.z:.8f},{q.w:.8f}\n')
                    s.n['pose_rows'] += 1
                    break
            else:
                if not self.warned_missing:
                    print(f'WARNING: {MODEL} not in pose message', flush=True)
                    self.warned_missing = True

    def on_cmd(self, msg):
        with self.lock:
            s = self.session
            if s is None:
                return
            s.cmd.write(f'{self.clock:.6f},{time.time():.6f},{msg.linear.x:.6f},{msg.angular.z:.6f}\n')
            s.n['cmd'] += 1

    def on_scan(self, msg):
        with self.lock:
            s = self.session
            if s is None:
                return
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            r = ','.join('inf' if math.isinf(v) else ('nan' if math.isnan(v) else f'{v:.4f}')
                         for v in msg.ranges)
            s.scan.write(f'{t:.6f},{time.time():.6f},{r}\n')
            s.n['scan'] += 1

    # ---- main loop ----
    def run(self):
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self.ros, timeout_sec=0.1)
            with self.lock:
                if self.failed:
                    self.end_session('failure')
                    return 1
                if self.session is not None and time.time() - self.session.last_pose_wall > GAP:
                    self.end_session(f'no pose for {GAP:.0f} s')
                if self.done >= self.args.n:
                    print(f'All {self.args.n} sessions recorded.', flush=True)
                    return 0
                if time.time() - t0 > self.args.max_wall:
                    self.end_session('max wall time')
                    print(f'Max wall time reached, {self.done} sessions recorded.', flush=True)
                    return 0 if self.done >= self.args.n else 2
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefix')
    ap.add_argument('n', type=int)
    ap.add_argument('--outdir', default=os.path.expanduser('~/run_logs'))
    ap.add_argument('--max-wall', type=float, default=1800.0)
    args = ap.parse_args()
    if not args.prefix.replace('_', '').replace('-', '').isalnum():
        sys.exit('PREFIX must be letters, digits, _ or -')
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    rec = Recorder(args)
    try:
        code = rec.run()
    except KeyboardInterrupt:
        with rec.lock:
            rec.end_session('interrupted')
        code = 130
    rec.ros.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
