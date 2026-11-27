#!/usr/bin/env python3
"""One run of the with/without test: send the goal, start the box, record. Used by ab_batch.sh.

Usage: python3 scripts/ab_record.py RUN_ID --delay SIM_SECONDS [--outdir DIR] [--goal-timeout WALL_S]

Needs the running stack: bringup.launch.py moving_obstacle:=true, perception and tracking.

Order: TF check -> wait for box pose, robot pose, goal services and start service ->
send goal map (3.0, 0.0) heading +x -> DELAY sim seconds after the goal is accepted call
/moving_obstacle/start -> wait for the result -> record 2 s more -> compare /odom with truth.

CPU: the goal is sent through the action's own services (send_goal, get_result, cancel_goal),
not through rclpy ActionClient. ActionClient always subscribes to feedback (100 msgs/s here),
which this script never uses. The drive loop waits up to DRIVE_WAIT s per pass; a ROS message
wakes it at once, so the box start call can be up to DRIVE_WAIT late (the delay is measured).

Writes (all opened with mode 'x', never overwritten), OUTDIR/RUN_ID_*:
  robot.csv   sim_t, wall_t, x, y, z, yaw_deg           (gz true pose, throttled to 20 msgs/s)
  box.csv     sim_t, wall_t, x, y                       (gz true pose, ~20 Hz)
  tracks.csv  recv_sim_t, wall_t, stamp, frame, id, x, y, vx, vy   (/tracked_obstacles)
  plans.csv   recv_sim_t, wall_t, plan_n, frame, i, x, y           (every /plan)
  events.csv  sim_t, wall_t, event, detail
sim_t = stamp of the gz message. recv_sim_t = latest box pose stamp when a ROS message arrived.

Robot pose source: /world/empty_world/dynamic_pose/info (moving models only), throttled.
Measured with Gazebo + Nav2 idle: 3.3 % of a core, of which ~1.9 % is the idle gz node.

Exit codes: 0 goal reached a result (any result, including timeout), 2 file exists,
3 TF missing, 4 goal not accepted, 5 setup failed (pose, server, service or box start).
Runs on WALL time on purpose (use_sim_time subscribes to /clock at ~1 kHz).
The command line must not match the leftover pattern in ab_batch.sh.
"""
import argparse
import csv
import json
import math
import os
import sys
import threading
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as RosNode
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from unique_identifier_msgs.msg import UUID
from diffdrive_nav2_ros2.msg import TrackedObstacleArray

from gz.transport13 import Node as GzNode, SubscribeOptions
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V

ROBOT = 'diffdrive_robot'
BOX_TOPIC = '/model/moving_box/pose'
ROBOT_TOPIC = '/world/empty_world/dynamic_pose/info'
ROBOT_RATE = 20              # msgs/s, throttled inside gz transport
GOAL_X, GOAL_Y = 3.0, 0.0    # map frame, heading +x
TF_TIMEOUT = 15.0
SETUP_TIMEOUT = 20.0
EXTRA_S = 2.0                # keep recording after the result
DRIVE_WAIT = 0.05            # s, longest wait per pass of the loop while the goal runs
ACTION = '/navigate_to_pose/_action/'

HEADERS = {
    'robot': ['sim_t', 'wall_t', 'x', 'y', 'z', 'yaw_deg'],
    'box': ['sim_t', 'wall_t', 'x', 'y'],
    'tracks': ['recv_sim_t', 'wall_t', 'stamp', 'frame', 'id', 'x', 'y', 'vx', 'vy'],
    'plans': ['recv_sim_t', 'wall_t', 'plan_n', 'frame', 'i', 'x', 'y'],
    'events': ['sim_t', 'wall_t', 'event', 'detail'],
}

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED',
          GoalStatus.STATUS_ABORTED: 'ABORTED'}


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


class Files:
    """The CSV files of one run. Thread safe (gz callbacks run on gz threads)."""

    def __init__(self, outdir, run_id):
        paths = {k: os.path.join(outdir, f'{run_id}_{k}.csv') for k in HEADERS}
        existing = [p for p in paths.values() if os.path.exists(p)]
        if existing:
            print('STOP: already exists:', *existing)
            sys.exit(2)
        self.lock = threading.Lock()
        self.closed = False
        self.files = {k: open(p, 'x', newline='') for k, p in paths.items()}
        self.writers = {k: csv.writer(f) for k, f in self.files.items()}
        for k, w in self.writers.items():
            w.writerow(HEADERS[k])

    def row(self, name, values):
        with self.lock:
            if not self.closed:
                self.writers[name].writerow(values)

    def close(self):
        with self.lock:
            self.closed = True
            for f in self.files.values():
                f.close()


class Recorder(RosNode):
    def __init__(self, files):
        super().__init__('ab_recorder')
        self.files = files
        self.sim_t = None        # from the latest box pose
        self.robot = None        # (sim_t, x, y, yaw_deg)
        self.n_plans = 0
        self.collided = False
        self.odom = None

        self.create_subscription(TrackedObstacleArray, '/tracked_obstacles', self.on_tracks, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(
            String, '/collision', self.on_collision,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        # The goal action as three plain service clients: no feedback or status subscription.
        self.send_goal_client = self.create_client(
            NavigateToPose.Impl.SendGoalService, ACTION + 'send_goal')
        self.get_result_client = self.create_client(
            NavigateToPose.Impl.GetResultService, ACTION + 'get_result')
        self.cancel_client = self.create_client(CancelGoal, ACTION + 'cancel_goal')
        self.start_client = self.create_client(Trigger, '/moving_obstacle/start')

        self.gz = GzNode()
        if not self.gz.subscribe(Pose, BOX_TOPIC, self.on_box):
            raise RuntimeError(f'cannot subscribe to {BOX_TOPIC}')
        opts = SubscribeOptions()
        opts.msgs_per_sec = ROBOT_RATE
        if not self.gz.subscribe(Pose_V, ROBOT_TOPIC, self.on_robot, opts):
            raise RuntimeError(f'cannot subscribe to {ROBOT_TOPIC}')

    def goal_services_ready(self):
        return all(c.service_is_ready() for c in
                   (self.send_goal_client, self.get_result_client, self.cancel_client))

    def cancel(self, goal_id):
        """Cancel our own goal only."""
        req = CancelGoal.Request()
        req.goal_info.goal_id = goal_id
        return self.cancel_client.call_async(req)

    def event(self, name, detail=''):
        sim = self.sim_t
        self.files.row('events', ['' if sim is None else f'{sim:.3f}',
                                  f'{time.time():.3f}', name, detail])
        print(f'[{name}] sim={sim} {detail}', flush=True)

    # ---- gz threads ----
    def on_box(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        self.sim_t = t
        self.files.row('box', [f'{t:.3f}', f'{time.time():.3f}',
                               f'{msg.position.x:.4f}', f'{msg.position.y:.4f}'])

    def on_robot(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == ROBOT:
                yaw = yaw_deg(p.orientation)
                self.robot = (t, p.position.x, p.position.y, yaw)
                self.files.row('robot', [f'{t:.3f}', f'{time.time():.3f}',
                                         f'{p.position.x:.4f}', f'{p.position.y:.4f}',
                                         f'{p.position.z:.4f}', f'{yaw:.2f}'])
                return

    # ---- ROS (main thread) ----
    def recv(self):
        return ['' if self.sim_t is None else f'{self.sim_t:.3f}', f'{time.time():.3f}']

    def on_tracks(self, msg):
        head = self.recv() + [f'{stamp_s(msg.header.stamp):.3f}', msg.header.frame_id]
        for o in msg.obstacles:
            self.files.row('tracks', head + [o.id, f'{o.x:.4f}', f'{o.y:.4f}',
                                             f'{o.vx:.3f}', f'{o.vy:.3f}'])

    def on_plan(self, msg):
        self.n_plans += 1
        head = self.recv() + [self.n_plans, msg.header.frame_id]
        for i, ps in enumerate(msg.poses):
            self.files.row('plans', head + [i, f'{ps.pose.position.x:.3f}',
                                            f'{ps.pose.position.y:.3f}'])

    def on_collision(self, msg):
        self.collided = True
        self.event('collision', msg.data)


def run(node, ex, args):
    """Returns the exit code."""
    def spin_until(cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            ex.spin_once(timeout_sec=0.02)
        return cond()

    node.event('start', json.dumps({'delay_requested': args.delay,
                                    'load': [round(v, 2) for v in os.getloadavg()]}))

    # 1. TF check, then drop the listener so /tf costs nothing while driving.
    buf = Buffer()
    listener = TransformListener(buf, node)
    frames = (('map', 'base_footprint'), ('odom', 'base_footprint'))

    def tf_ok():
        return all(buf.can_transform(a, b, Time()) for a, b in frames)

    ok = spin_until(tf_ok, TF_TIMEOUT)
    missing = [f'{a}->{b}' for a, b in frames if not buf.can_transform(a, b, Time())]
    listener.unregister()
    if not ok:
        node.event('invalid_tf', ' '.join(missing))
        return 3
    node.event('tf_ok', 'map->base_footprint, odom->base_footprint')

    # 2. Everything needed, connected BEFORE the goal.
    for what, cond in (
            ('box pose', lambda: node.sim_t is not None),
            ('robot pose', lambda: node.robot is not None),
            ('navigate_to_pose goal services', node.goal_services_ready),
            ('/moving_obstacle/start', node.start_client.service_is_ready)):
        if not spin_until(cond, SETUP_TIMEOUT):
            node.event('invalid_setup', f'no {what}')
            return 5

    # 3. Goal
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = GOAL_X
    goal.pose.pose.position.y = GOAL_Y
    goal.pose.pose.orientation.w = 1.0
    goal_id = UUID(uuid=list(uuid.uuid4().bytes))
    node.event('goal_sent')
    send_future = node.send_goal_client.call_async(
        NavigateToPose.Impl.SendGoalService.Request(goal_id=goal_id, goal=goal))
    if not spin_until(send_future.done, SETUP_TIMEOUT) or not send_future.result().accepted:
        node.event('invalid_goal', 'not accepted')
        return 4
    accept_sim = node.sim_t
    accept_mono = time.monotonic()
    cpu0 = time.process_time()
    node.event('goal_accepted')
    result_future = node.get_result_client.call_async(
        NavigateToPose.Impl.GetResultService.Request(goal_id=goal_id))

    # 4. Start the box DELAY sim seconds after acceptance; wait for the result.
    trigger_future = None
    trigger_mono = None
    trigger_checked = False
    cancel_sent = None
    while not result_future.done():
        ex.spin_once(timeout_sec=DRIVE_WAIT)
        now = time.monotonic()
        if trigger_future is None and not node.collided and node.sim_t - accept_sim >= args.delay:
            trigger_mono = now
            node.event('trigger_sent')
            trigger_future = node.start_client.call_async(Trigger.Request())
        if trigger_future is not None and not trigger_checked and trigger_future.done():
            trigger_checked = True
            r = trigger_future.result()
            node.event('trigger_response', json.dumps({
                'success': r.success, 'message': r.message,
                'latency_wall_s': round(now - trigger_mono, 3)}))
            if not r.success and not node.collided:
                node.event('invalid_setup', 'box start refused')
                node.cancel(goal_id)
                spin_until(result_future.done, 10.0)
                return 5
        if cancel_sent is None and now - accept_mono > args.goal_timeout:
            cancel_sent = now
            node.event('goal_timeout', f'{args.goal_timeout} s wall, cancelling')
            node.cancel(goal_id)
        if cancel_sent is not None and now - cancel_sent > 10.0:
            break

    status = (STATUS.get(result_future.result().status, str(result_future.result().status))
              if result_future.done() else 'NO_RESULT')
    node.event('result', json.dumps({
        'status': status, 'timed_out': cancel_sent is not None,
        'wall_s': round(time.monotonic() - accept_mono, 3)}))

    if trigger_future is not None and not trigger_checked:
        if not spin_until(trigger_future.done, 5.0):
            node.event('invalid_setup', 'no answer from box start')
            return 5

    # 5. A little more data, then check /odom against the true pose.
    spin_until(lambda: False, EXTRA_S)
    cpu = 100.0 * (time.process_time() - cpu0) / (time.monotonic() - accept_mono)

    def on_odom(msg):
        node.odom = msg

    sub = node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)
    if spin_until(lambda: node.odom is not None, 5.0):
        p = node.odom.pose.pose
        _, rx, ry, ryaw = node.robot
        dyaw = (yaw_deg(p.orientation) - ryaw + 180.0) % 360.0 - 180.0
        node.event('odom_check', json.dumps({
            'dist_m': round(math.hypot(p.position.x - rx, p.position.y - ry), 4),
            'yaw_diff_deg': round(dyaw, 2)}))
    else:
        node.event('odom_check', json.dumps({'dist_m': None, 'yaw_diff_deg': None}))
    node.destroy_subscription(sub)
    node.event('end', json.dumps({'recorder_cpu_pct': round(cpu, 1),
                                  'load': [round(v, 2) for v in os.getloadavg()]}))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_id')
    ap.add_argument('--delay', type=float, required=True,
                    help='sim seconds from goal accepted to the box start call')
    ap.add_argument('--outdir', default=os.path.expanduser('~/run_logs'))
    ap.add_argument('--goal-timeout', type=float, default=90.0, help='wall seconds')
    args = ap.parse_args()

    files = Files(args.outdir, args.run_id)
    rclpy.init()
    node = Recorder(files)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    code = 1
    try:
        code = run(node, ex, args)
    finally:
        node.gz.unsubscribe(BOX_TOPIC)
        node.gz.unsubscribe(ROBOT_TOPIC)
        files.close()
        sys.stdout.flush()
        if sys.exc_info()[0] is None:
            # Skip interpreter teardown: live gz threads must not change the exit code.
            os._exit(code)


if __name__ == '__main__':
    main()
