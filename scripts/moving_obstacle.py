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

Parameters: speed 0.25, y_min -1.0, y_max 1.0, start_delay 0.0, autostart false, rate 10.0
Do NOT set use_sim_time:=true (see above).
"""
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose

POSE_TOPIC = '/model/moving_box/pose'
CMD_TOPIC = '/model/moving_box/cmd_vel'
WATCHDOG_S = 1.0


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
        self.y = None
        self.sim_t = None            # sim time from the latest pose stamp
        self.last_pose_mono = None
        self.watchdog_zero = False
        self.n_reversals = 0

        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_service(Trigger, '/moving_obstacle/start', self.on_start)
        self.create_timer(1.0 / rate, self.on_timer)

        self.gz = GzNode()
        if not self.gz.subscribe(Pose, POSE_TOPIC, self.on_pose):
            raise RuntimeError(f'cannot subscribe to {POSE_TOPIC}')
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
        y = msg.position.y
        sim_t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with self.lock:
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

    # ---- timer: start logic, heartbeat, watchdog ----
    def on_timer(self):
        with self.lock:
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
