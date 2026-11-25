#!/usr/bin/env python3
"""Drive the Gazebo box `moving_box` back and forth along world y (x fixed by the world pose).

Needs worlds/test_world_moving.sdf (VelocityControl + PosePublisher on the box) and the
bridge entry /model/moving_box/cmd_vel (both added by launch with moving_obstacle:=true).

- Reads the box pose from the gz topic /model/moving_box/pose (gz.msgs.Pose, ~20 Hz).
- Reverses INSIDE the pose callback when y reaches y_max (moving +) or y_min (moving -).
- A timer (param `rate`, 10 Hz) only re-sends the current command (heartbeat) and
  checks the pose watchdog.
- Starts moving only after the Trigger service /moving_obstacle/start (or `autostart`),
  then waits `start_delay` sim seconds.
- Runs on WALL time on purpose: with use_sim_time the node subscribes to /clock (~1 kHz) and
  used 60-64 % of a core (measured). Sim time comes from the stamp of the 20 Hz gz pose
  message instead (the pose stamp is sim time; checked against /clock).
- No pose for > 1 s (wall) -> sends zero velocity and warns. On shutdown -> zero.

Collision handling (needs the contact sensor on moving_box in worlds/test_world_moving.sdf):
- Subscribes to the sensor's default gz topic CONTACT_TOPIC (gz.msgs.Contacts, 50 Hz).
- Ground contacts and any shape that is not a robot part are ignored silently.
- First contact with a robot part (latched, happens once per run):
    * box velocity is set to zero and stays zero (no more reversals, start service refused)
    * logs one ERROR line and publishes /collision (std_msgs/String, JSON, transient_local)
    * cancels all goals of /navigate_to_pose
    * publishes 5 zero Twists on /cmd_vel (safety net while the cancel goes through)
  A full restart is needed to move the box again.

Parameters: speed 0.25, y_min -1.0, y_max 1.0, start_delay 0.0, autostart false, rate 10.0
Do NOT set use_sim_time:=true (see above).
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.contacts_pb2 import Contacts

POSE_TOPIC = '/model/moving_box/pose'
# Default topic of the contact sensor in worlds/test_world_moving.sdf (world empty_world, model
# moving_box, link l, sensor contact). A <topic> tag on the sensor was ignored (checked).
CONTACT_TOPIC = '/world/empty_world/model/moving_box/link/l/sensor/contact/contact'
CMD_TOPIC = '/model/moving_box/cmd_vel'
ROBOT_CMD_TOPIC = '/cmd_vel'
CANCEL_SRV = '/navigate_to_pose/_action/cancel_goal'
COLLISION_TOPIC = '/collision'
WATCHDOG_S = 1.0
CONTACT_SILENT_S = 5.0       # no contact message for this long (wall s) -> error log
SAFETY_ZEROS = 5

# Shape names that belong to the robot. The robot URDF is lumped by gz, so besides the scoped
# name (diffdrive_robot::...) the bare collision names are listed too (checked with `gz sdf -p`).
ROBOT_NAME_PARTS = ('diffdrive_robot', 'base_footprint_fixed_joint_lump__',
                    'left_wheel_collision', 'right_wheel_collision')


def is_robot(name):
    return any(p in name for p in ROBOT_NAME_PARTS)


def is_ground(name):
    return 'ground_plane' in name


class MovingObstacle(Node):
    def __init__(self):
        super().__init__('moving_obstacle')
        self.speed = self.declare_parameter('speed', 0.25).value
        self.y_min = self.declare_parameter('y_min', -1.0).value
        self.y_max = self.declare_parameter('y_max', 1.0).value
        self.start_delay = self.declare_parameter('start_delay', 0.0).value
        self.autostart = self.declare_parameter('autostart', False).value
        rate = self.declare_parameter('rate', 10.0).value

        self.lock = threading.Lock()
        self.triggered = False
        self.trigger_sim = None      # sim time (s) when trigger was received
        self.running = False
        self.direction = 0           # +1 or -1 along y
        self.x = None
        self.y = None
        self.sim_t = None            # sim time from the latest pose stamp
        self.last_pose_mono = None
        self.watchdog_zero = False
        self.n_reversals = 0
        self.collided = False        # latched by the first robot contact
        self.safety_zeros_left = 0   # zero Twists still to send on /cmd_vel
        self.logged_first_contact = False
        self.start_mono = time.monotonic()
        self.last_contact_mono = None    # wall time of the latest contact message (any kind)
        self.last_silent_log = float('-inf')
        self.contact_silent = False

        self.pub =self.create_publisher(Twist, CMD_TOPIC, 10)
        self.robot_pub = self.create_publisher(Twist, ROBOT_CMD_TOPIC, 10)
        self.collision_pub = self.create_publisher(
            String, COLLISION_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.cancel_client = self.create_client(CancelGoal, CANCEL_SRV)
        self.create_service(Trigger, '/moving_obstacle/start', self.on_start)
        self.create_timer(1.0 / rate, self.on_timer)

        self.gz = GzNode()
        if not self.gz.subscribe(Pose, POSE_TOPIC, self.on_pose):
            raise RuntimeError(f'cannot subscribe to {POSE_TOPIC}')
        if not self.gz.subscribe(Contacts, CONTACT_TOPIC, self.on_contact):
            raise RuntimeError(f'cannot subscribe to {CONTACT_TOPIC}')
        self.get_logger().info(
            f'ready: y in [{self.y_min}, {self.y_max}], speed {self.speed}, '
            f'start_delay {self.start_delay} sim s, autostart {self.autostart}')


    def send(self, vy):
        msg = Twist()
        msg.linear.y = float(vy)
        self.pub.publish(msg)

    # ---- service ----
    def on_start(self, _req, resp):
        with self.lock:
            if self.collided:
                resp.success = False
                resp.message = 'already collided, restart needed'
                return resp
            if self.triggered:
                resp.success = False
                resp.message = 'already triggered'
                return resp
            self.triggered = True
            self.trigger_sim = self.sim_t       # None if no pose yet, set in on_timer
        self.get_logger().info(f'start requested, moving after {self.start_delay} sim s')
        resp.success = True
        resp.message = 'ok'
        return resp

    # ---- gz pose callback (gz transport thread) ----
    def on_pose(self, msg):
        x = msg.position.x
        y = msg.position.y
        sim_t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with self.lock:
            self.x = x
            self.y = y
            self.sim_t = sim_t
            self.last_pose_mono = time.monotonic()
            self.watchdog_zero = False
            if not self.running:
                return
            if self.direction > 0 and y >= self.y_max:
                self.direction = -1
            elif self.direction < 0 and y <= self.y_min:
                self.direction = +1
            else:
                return
            self.n_reversals += 1
            n, d = self.n_reversals, self.direction
            self.send(d * self.speed)       # reverse right here, not on the timer
        self.get_logger().info(f'reversal {n}: y={y:.3f}, direction now {d:+d}')

    # ---- gz contact callback (gz transport thread, 50 Hz) ----
    def on_contact(self, msg):
        self.last_contact_mono = time.monotonic()   # any message counts as "sensor alive"
        hit = None
        for c in msg.contact:
            n1, n2 = c.collision1.name, c.collision2.name
            if is_ground(n1) or is_ground(n2):
                continue                        # ground contact: ignore silently
            if not self.logged_first_contact:
                self.logged_first_contact = True
                self.get_logger().info(f'first non-ground contact: "{n1}" <-> "{n2}"')
            robot_part = n1 if is_robot(n1) else (n2 if is_robot(n2) else None)
            if robot_part is not None:
                pos = c.position[0] if len(c.position) else None
                hit = (robot_part, pos)
                break
        if hit is None:
            return
        robot_part, pos = hit
        with self.lock:
            if self.collided:
                return
            self.collided = True
            self.running = False
            self.direction = 0
            self.safety_zeros_left = SAFETY_ZEROS
            self.send(0.0)                      # stop the box right here
            sim_t, bx, by = self.sim_t, self.x, self.y
        info = {
            'sim_time': sim_t,
            'box_xy': [bx, by],
            'robot_part': robot_part,
            'contact_xyz': [pos.x, pos.y, pos.z] if pos is not None else None,
        }
        self.get_logger().error(
            f'COLLISION: moving_box hit "{robot_part}" at sim t={sim_t}, box=({bx}, {by}); '
            'box stopped, cancelling navigate_to_pose goals')
        self.collision_pub.publish(String(data=json.dumps(info)))
        if self.cancel_client.service_is_ready():
            future = self.cancel_client.call_async(CancelGoal.Request())  # empty = cancel all
            future.add_done_callback(self.on_cancel_done)
        else:
            self.get_logger().error(f'{CANCEL_SRV} not available, goal NOT cancelled')

    def on_cancel_done(self, future):
        try:
            codes = {0: 'ERROR_NONE', 1: 'ERROR_REJECTED', 2: 'ERROR_UNKNOWN_GOAL_ID',
                     3: 'ERROR_GOAL_TERMINATED'}
            r = future.result()
            self.get_logger().info(
                f'cancel result: {codes.get(r.return_code, r.return_code)}, '
                f'{len(r.goals_canceling)} goal(s) canceling')
        except Exception as e:
            self.get_logger().error(f'cancel call failed: {e}')

    # ---- timer: start logic, heartbeat, watchdog ----
    def on_timer(self):
        with self.lock:
            if self.collided:
                self.send(0.0)                  # box stays stopped
                if self.safety_zeros_left > 0:
                    self.safety_zeros_left -= 1
                    self.robot_pub.publish(Twist())
                return
            # Contact sensor watchdog (log only): the resting box always touches the ground,
            # so contact messages should keep coming. Silence = collisions would go unseen.
            now = time.monotonic()
            ref = self.last_contact_mono if self.last_contact_mono is not None else self.start_mono
            if now - ref > CONTACT_SILENT_S:
                if now - self.last_silent_log >= CONTACT_SILENT_S:
                    self.last_silent_log = now
                    self.contact_silent = True
                    if self.last_pose_mono is None or now - self.last_pose_mono > WATCHDOG_S:
                        self.get_logger().warn(
                            f'no contact messages for > {CONTACT_SILENT_S} s, and no box pose '
                            'either: sim not running or paused?')
                    else:
                        self.get_logger().error(
                            f'CONTACT SENSOR SILENT for > {CONTACT_SILENT_S} s (box pose is '
                            f'arriving): collisions will NOT be detected. Topic: {CONTACT_TOPIC}')
            elif self.contact_silent:
                self.contact_silent = False
                self.get_logger().info('contact messages are back')
            if not self.running:
                if self.autostart and not self.triggered:
                    self.triggered = True
                    self.trigger_sim = self.sim_t
                if self.triggered and self.trigger_sim is None:
                    self.trigger_sim = self.sim_t
                if (self.triggered and self.y is not None and self.trigger_sim is not None
                        and self.sim_t - self.trigger_sim >= self.start_delay):
                    mid = 0.5 * (self.y_min + self.y_max)
                    self.direction = +1 if self.y < mid else -1
                    self.running = True
                    self.get_logger().info(
                        f'moving: y={self.y:.3f}, direction {self.direction:+d}')
                else:
                    return
            stale = (self.last_pose_mono is None
                     or time.monotonic() - self.last_pose_mono > WATCHDOG_S)
            if stale:
                if not self.watchdog_zero:
                    self.get_logger().warn(f'no pose for > {WATCHDOG_S} s, sending zero')
                    self.watchdog_zero = True
                self.send(0.0)
            else:
                self.send(self.direction * self.speed)   # heartbeat

    def stop(self):
        with self.lock:
            self.running = False
            for _ in range(3):
                self.send(0.0)


def main():
    rclpy.init()
    node = MovingObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
