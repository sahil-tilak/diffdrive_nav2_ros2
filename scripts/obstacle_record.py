#!/usr/bin/env python3
"""Record the moving box (gz true pose) and /scan for the moving-obstacle test.

Usage: python3 scripts/obstacle_record.py RUN_ID [--outdir DIR] [--max-wall SEC] [--window SEC]

ONE recording per process. No counter: the name comes from RUN_ID (e.g. mobA_01).
Writes (all opened with mode 'x', never overwritten):
  OUTDIR/RUN_ID_obstacle_pose.csv   sim_t, wall_t, x, y, z, qx, qy, qz, qw   (box)
  OUTDIR/RUN_ID_obstacle_scan.csv   sim_t (header stamp), wall_t, r0..r359
  OUTDIR/RUN_ID_obstacle_robot.csv  when, x, y, z, yaw_deg   (robot, read once at start and once at end)
At start, if any of these or RUN_ID_obstacle_analysis.txt exists, it prints STOP and exits
BEFORE subscribing to anything.

Sources: /model/moving_box/pose (gz, ~20 Hz), /scan (ROS). The robot pose comes from
/world/empty_world/pose/info, subscribed ONLY to take one message (start, end), then
unsubscribed. No pose/info subscription during the run (it cost 40-58 % of a core).

Window: starts when the box moves (|dy/dt| > 0.05 m/s over >= 0.2 s of sim time); recording
ends WINDOW + 5 sim seconds later (default WINDOW 180).
Invalid run (sim time goes backwards by > 1 s, or no box pose for > 5 s wall after the first
one): last line '#INVALID,reason' in the pose and scan files, files closed, exit code 2.
The command line of this process must not contain "gz", "rviz" or "component_container".
"""
import argparse
import collections
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node as RosNode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V

WORLD = 'empty_world'
ROBOT = 'diffdrive_robot'
BOX_TOPIC = '/model/moving_box/pose'
WORLD_POSE_TOPIC = f'/world/{WORLD}/pose/info'
MOVE_V = 0.05
MOVE_DT = 0.2
GAP = 5.0
SUFFIXES = ('_obstacle_pose.csv', '_obstacle_scan.csv', '_obstacle_robot.csv',
            '_obstacle_analysis.txt')


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


class OneShotRobotPose:
    """Subscribe to pose/info only until one message with the robot arrives."""

    def __init__(self, gz):
        self.gz = gz
        self.got = threading.Event()
        self.value = None

    def read(self, timeout):
        self.got.clear()
        self.value = None
        if not self.gz.subscribe(Pose_V, WORLD_POSE_TOPIC, self._cb):
            return None
        self.got.wait(timeout)
        self.gz.unsubscribe(WORLD_POSE_TOPIC)   # from the main thread, not the callback
        return self.value

    def _cb(self, msg):
        if self.got.is_set():
            return
        for p in msg.pose:
            if p.name == ROBOT:
                self.value = (p.position.x, p.position.y, p.position.z,
                              yaw_deg(p.orientation))
                self.got.set()
                return


class Recorder:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.pose_f = self.scan_f = None
        self.failed_reason = None
        self.finished = False
        self.n_pose = self.n_scan = 0
        self.first_sim = self.last_sim = None
        self.last_pose_wall = None
        self.hist = collections.deque()      # (sim_t, y) for the move detector
        self.move_start = None
        self.last_flush = time.time()

    def open_files(self):
        base = os.path.join(self.args.outdir, self.args.run_id)
        self.pose_f = open(base + SUFFIXES[0], 'x')
        self.scan_f = open(base + SUFFIXES[1], 'x')
        self.pose_f.write('sim_t,wall_t,x,y,z,qx,qy,qz,qw\n')
        self.scan_f.write('sim_t,wall_t,' + ','.join(f'r{i}' for i in range(360)) + '\n')

    def close(self, invalid=None):
        with self.lock:
            if self.finished:
                return
            self.finished = True
            for f in (self.pose_f, self.scan_f):
                if f is not None:
                    if invalid:
                        f.write(f'#INVALID,{invalid}\n')
                    f.close()

    def on_box(self, msg):
        sim_t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        now = time.time()
        with self.lock:
            if self.finished or self.failed_reason:
                return
            if self.last_sim is not None and sim_t < self.last_sim - 1.0:
                self.failed_reason = 'sim time went backwards'
                return
            if self.first_sim is None:
                self.first_sim = sim_t
            self.last_sim = sim_t
            self.last_pose_wall = now
            p, q = msg.position, msg.orientation
            self.pose_f.write(f'{sim_t:.6f},{now:.6f},{p.x:.6f},{p.y:.6f},{p.z:.6f},'
                              f'{q.x:.8f},{q.y:.8f},{q.z:.8f},{q.w:.8f}\n')
            self.n_pose += 1
            if now - self.last_flush > 1.0:
                self.pose_f.flush()
                self.scan_f.flush()
                self.last_flush = now
            if self.move_start is None:
                self.hist.append((sim_t, p.y))
                while len(self.hist) > 1 and self.hist[1][0] <= sim_t - MOVE_DT:
                    self.hist.popleft()
                t0, y0 = self.hist[0]
                if sim_t - t0 >= MOVE_DT and abs(p.y - y0) / (sim_t - t0) > MOVE_V:
                    self.move_start = sim_t
                    print(f'Box moving from sim t={sim_t:.2f}', flush=True)

    def on_scan(self, msg):
        with self.lock:
            if self.finished or self.failed_reason or self.first_sim is None:
                return
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            r = ','.join('inf' if math.isinf(v) else ('nan' if math.isnan(v) else f'{v:.4f}')
                         for v in msg.ranges)
            self.scan_f.write(f'{t:.6f},{time.time():.6f},{r}\n')
            self.n_scan += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_id')
    ap.add_argument('--outdir', default=os.path.expanduser('~/run_logs'))
    ap.add_argument('--max-wall', type=float, default=900.0)
    ap.add_argument('--window', type=float, default=180.0)
    args = ap.parse_args()
    if not args.run_id.replace('_', '').replace('-', '').isalnum():
        sys.exit('RUN_ID must be letters, digits, _ or -')
    low = args.run_id.lower()
    if any(w in low for w in ('gz', 'rviz', 'component_container')):
        sys.exit('RUN_ID must not contain gz, rviz or component_container')

    # 1. refuse BEFORE subscribing to anything
    existing = [os.path.join(args.outdir, args.run_id + s) for s in SUFFIXES
                if os.path.exists(os.path.join(args.outdir, args.run_id + s))]
    if existing:
        print('STOP: file exists, not overwriting:', *existing, sep='\n  ', flush=True)
        sys.exit(1)

    rclpy.init()
    ros = RosNode('obstacle_recorder')
    gz = GzNode()

    # 2. robot pose, once (idle robot)
    one = OneShotRobotPose(gz)
    start_pose = one.read(30.0)
    if start_pose is None:
        print('STOP: could not read the robot pose from pose/info in 30 s', flush=True)
        sys.exit(1)
    print('Robot at start: x=%.4f y=%.4f z=%.4f yaw=%.3f deg' % start_pose, flush=True)

    rec = Recorder(args)
    try:
        rec.open_files()
        robot_f = open(os.path.join(args.outdir, args.run_id + SUFFIXES[2]), 'x')
    except FileExistsError as e:
        print(f'STOP: file exists, not overwriting: {e.filename}', flush=True)
        sys.exit(1)
    robot_f.write('when,x,y,z,yaw_deg\n')
    robot_f.write('start,%.6f,%.6f,%.6f,%.4f\n' % start_pose)
    robot_f.flush()

    ros.create_subscription(LaserScan, '/scan', rec.on_scan, qos_profile_sensor_data)
    if not gz.subscribe(Pose, BOX_TOPIC, rec.on_box):
        print(f'ERROR: cannot subscribe to {BOX_TOPIC}', flush=True)
        rec.close('cannot subscribe')
        robot_f.close()
        sys.exit(2)
    print(f'Recording run {args.run_id}; waiting for box pose and movement...', flush=True)

    t0 = time.time()
    code = 0
    why = 'done'
    try:
        while rclpy.ok():
            rclpy.spin_once(ros, timeout_sec=0.1)
            with rec.lock:
                reason = rec.failed_reason
                if (reason is None and rec.last_pose_wall is not None
                        and time.time() - rec.last_pose_wall > GAP):
                    reason = f'no box pose for {GAP:.0f} s'
                if reason is None and time.time() - t0 > args.max_wall:
                    reason = 'max wall time'
                end_sim = (rec.move_start + args.window + 5.0) if rec.move_start else None
                done = end_sim is not None and rec.last_sim >= end_sim
            if reason:
                why, code = reason, 2
                break
            if done:
                break
    except KeyboardInterrupt:
        why, code = 'interrupted', 130

    gz.unsubscribe(BOX_TOPIC)
    n_pose, n_scan = rec.n_pose, rec.n_scan
    if code:
        rec.close(why)
    else:
        rec.close()
    # 3. robot pose again, once
    end_pose = one.read(10.0)
    if end_pose is None:
        robot_f.write('end,nan,nan,nan,nan\n')
        print('WARNING: could not read the robot pose at the end', flush=True)
    else:
        robot_f.write('end,%.6f,%.6f,%.6f,%.4f\n' % end_pose)
        d = math.hypot(end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
        dy = abs(end_pose[3] - start_pose[3])
        print(f'Robot moved {d:.4f} m, yaw changed {dy:.3f} deg '
              f'({"OK" if d <= 0.01 and dy <= 0.5 else "RUN INVALID"})', flush=True)
    robot_f.close()
    print(f'Finished ({why}): box rows {n_pose}, scan rows {n_scan}, '
          f'window start sim t={rec.move_start}', flush=True)
    ros.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
