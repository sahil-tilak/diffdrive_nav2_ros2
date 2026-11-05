#!/usr/bin/env python3
"""Record the perception run: box true pose (gz), /scan stamps, /detected_obstacles, node CPU.

Usage: python3 scripts/perception_record.py RUN_ID [--outdir DIR] [--max-wall SEC] [--window SEC]

ONE recording per process, name from RUN_ID (e.g. percA_01). All files opened with mode 'x':
  OUTDIR/RUN_ID_box.csv         sim_t, wall_t, x, y, z, qx, qy, qz, qw        (gz /model/moving_box/pose)
  OUTDIR/RUN_ID_scanstamps.csv  stamp_ns, wall_t                              (every /scan)
  OUTDIR/RUN_ID_det.csv         stamp_ns, n, x1, y1, x2, y2, ...              (every /detected_obstacles)
  OUTDIR/RUN_ID_robot.csv       when, x, y, z, yaw_deg                        (read once at start and end)
  OUTDIR/RUN_ID_cpu.csv         wall_t, sim_t, ticks                          (utime+stime of the detector, 1 Hz)
At start, if any of these or RUN_ID_perception_analysis.txt exists: prints STOP, exits BEFORE
subscribing to anything. Also STOP unless exactly one obstacle_detector_node process exists.

Pattern copied from obstacle_record.py: window starts when the box moves (|dy/dt| > 0.05 m/s over
>= 0.2 s sim), ends WINDOW + 5 sim s later. Wall time only, no /clock. Robot pose from pose/info only
for one message at start and end. Invalid run (sim time backwards > 1 s, no box pose for > 5 s wall,
detector process gone): last line '#INVALID,reason' in the csv files, exit code 2.
The command line must not contain "gz", "rviz" or "component_container".
"""
import argparse
import collections
import math
import os
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseArray
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
SUFFIXES = ('_box.csv', '_scanstamps.csv', '_det.csv', '_robot.csv', '_cpu.csv',
            '_perception_analysis.txt')
DET_PATTERN = 'lib/diffdrive_nav2_ros2/obstacle_detector_node'


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def cpu_ticks(pid):
    """utime + stime (clock ticks) of the whole process, from /proc/PID/stat."""
    with open(f'/proc/{pid}/stat') as f:
        s = f.read()
    fields = s[s.rindex(')') + 2:].split()      # fields[0] is state (field 3)
    return int(fields[11]) + int(fields[12])    # utime = field 14, stime = field 15


class OneShotRobotPose:
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
        self.gz.unsubscribe(WORLD_POSE_TOPIC)
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
        self.files = []
        self.box_f = self.scan_f = self.det_f = self.cpu_f = None
        self.failed_reason = None
        self.finished = False
        self.n_box = self.n_scan = self.n_det = 0
        self.first_sim = self.last_sim = None
        self.last_pose_wall = None
        self.hist = collections.deque()
        self.move_start = None
        self.last_flush = time.time()

    def open_files(self):
        base = os.path.join(self.args.outdir, self.args.run_id)
        self.box_f = open(base + SUFFIXES[0], 'x')
        self.scan_f = open(base + SUFFIXES[1], 'x')
        self.det_f = open(base + SUFFIXES[2], 'x')
        self.cpu_f = open(base + SUFFIXES[4], 'x')
        self.files = [self.box_f, self.scan_f, self.det_f, self.cpu_f]
        self.box_f.write('sim_t,wall_t,x,y,z,qx,qy,qz,qw\n')
        self.scan_f.write('stamp_ns,wall_t\n')
        self.det_f.write('stamp_ns,n,xy...\n')
        self.cpu_f.write('wall_t,sim_t,ticks\n')

    def close(self, invalid=None):
        with self.lock:
            if self.finished:
                return
            self.finished = True
            for f in self.files:
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
            self.box_f.write(f'{sim_t:.6f},{now:.6f},{p.x:.6f},{p.y:.6f},{p.z:.6f},'
                             f'{q.x:.8f},{q.y:.8f},{q.z:.8f},{q.w:.8f}\n')
            self.n_box += 1
            if now - self.last_flush > 1.0:
                for f in self.files:
                    f.flush()
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
            ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            self.scan_f.write(f'{ns},{time.time():.6f}\n')
            self.n_scan += 1

    def on_det(self, msg):
        with self.lock:
            if self.finished or self.failed_reason or self.first_sim is None:
                return
            ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            xy = ''.join(f',{p.position.x:.4f},{p.position.y:.4f}' for p in msg.poses)
            self.det_f.write(f'{ns},{len(msg.poses)}{xy}\n')
            self.n_det += 1


def find_detector_pid():
    out = subprocess.run(['pgrep', '-f', DET_PATTERN], capture_output=True, text=True).stdout
    pids = [int(x) for x in out.split()]
    return pids[0] if len(pids) == 1 else None, pids


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
    pid, pids = find_detector_pid()
    if pid is None:
        print(f'STOP: need exactly one obstacle_detector_node process, found {pids}', flush=True)
        sys.exit(1)
    print(f'Detector PID {pid}', flush=True)

    rclpy.init()
    ros = RosNode('perception_recorder')
    gz = GzNode()

    one = OneShotRobotPose(gz)
    start_pose = one.read(30.0)
    if start_pose is None:
        print('STOP: could not read the robot pose from pose/info in 30 s', flush=True)
        sys.exit(1)
    print('Robot at start: x=%.4f y=%.4f z=%.4f yaw=%.3f deg' % start_pose, flush=True)

    rec = Recorder(args)
    try:
        rec.open_files()
        robot_f = open(os.path.join(args.outdir, args.run_id + SUFFIXES[3]), 'x')
    except FileExistsError as e:
        print(f'STOP: file exists, not overwriting: {e.filename}', flush=True)
        sys.exit(1)
    robot_f.write('when,x,y,z,yaw_deg\n')
    robot_f.write('start,%.6f,%.6f,%.6f,%.4f\n' % start_pose)
    robot_f.flush()

    ros.create_subscription(LaserScan, '/scan', rec.on_scan, qos_profile_sensor_data)
    ros.create_subscription(PoseArray, '/detected_obstacles', rec.on_det, 100)
    if not gz.subscribe(Pose, BOX_TOPIC, rec.on_box):
        print(f'ERROR: cannot subscribe to {BOX_TOPIC}', flush=True)
        rec.close('cannot subscribe')
        robot_f.close()
        sys.exit(2)
    print(f'Recording run {args.run_id}; waiting for box pose and movement...', flush=True)

    t0 = time.time()
    next_cpu = t0
    code = 0
    why = 'done'
    try:
        while rclpy.ok():
            rclpy.spin_once(ros, timeout_sec=0.1)
            now = time.time()
            if now >= next_cpu:
                next_cpu = now + 1.0
                try:
                    ticks = cpu_ticks(pid)
                    with rec.lock:
                        if rec.last_sim is not None and not rec.finished:
                            rec.cpu_f.write(f'{now:.6f},{rec.last_sim:.6f},{ticks}\n')
                except (FileNotFoundError, ProcessLookupError):
                    with rec.lock:
                        rec.failed_reason = 'detector process gone'
            with rec.lock:
                reason = rec.failed_reason
                if (reason is None and rec.last_pose_wall is not None
                        and now - rec.last_pose_wall > GAP):
                    reason = f'no box pose for {GAP:.0f} s'
                if reason is None and now - t0 > args.max_wall:
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
    n_box, n_scan, n_det = rec.n_box, rec.n_scan, rec.n_det
    rec.close(why if code else None)
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
    print(f'Finished ({why}): box rows {n_box}, scan rows {n_scan}, det rows {n_det}, '
          f'window start sim t={rec.move_start}', flush=True)
    ros.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
