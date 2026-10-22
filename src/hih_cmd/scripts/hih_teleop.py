#!/usr/bin/env python3
"""One keyboard, two separate position/yaw command publishers.

Each key controls just one drone. Body X/Y are rotated by that drone's
measured yaw before publishing world-frame position references. Body Z is
world Z for the level-flight teleoperation used in this experiment.
"""
import math
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from hih_interfaces.msg import AttitudeCmd, Cmd, MinitroneState


# (vehicle, body axis or yaw, direction). One key never commands both drones.
KEYS = {
    'a': ('follower', 'x', -1), 'd': ('follower', 'x', 1),
    'w': ('follower', 'y', 1), 's': ('follower', 'y', -1),
    'r': ('follower', 'z', 1),  'f': ('follower', 'z', -1),
    'q': ('follower', 'yaw', 1), 'e': ('follower', 'yaw', -1),
    'j': ('leader', 'x', 1), 'l': ('leader', 'x', -1),
    'i': ('leader', 'y', -1), 'k': ('leader', 'y', 1),
    'y': ('leader', 'z', 1), 'h': ('leader', 'z', -1),
    'u': ('leader', 'yaw', 1), 'o': ('leader', 'yaw', -1),
}

# Used before state messages arrive; commands still wait for both measured poses.
INITIAL_X_OFFSET = 0.215 + 0.15 + 0.255


def body_xy_to_world(body_x, body_y, yaw_rad):
    """Rotate horizontal body-frame motion into MuJoCo world X/Y."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return c * body_x - s * body_y, s * body_x + c * body_y


class Target:
    def __init__(self, x, yaw_deg):
        self.position = [x, 0.0, 1.05]
        self.yaw_deg = yaw_deg
        self.measured_yaw_rad = math.radians(yaw_deg)
        self.measured_position = None
        self.initialized = False
        self.motion = None
        self.last_key = 0.0

    def update_state(self, msg):
        self.measured_position = [float(value) for value in msg.pos]
        self.measured_yaw_rad = float(msg.rpy[2])
        if not self.initialized:
            # Hold each vehicle's own measured pose when teleop is first run.
            self.position = self.measured_position.copy()
            self.yaw_deg = math.degrees(self.measured_yaw_rad)
            self.initialized = True


class HIHTeleop(Node):
    def __init__(self):
        super().__init__('hih_teleop')
        self.speed_xy = float(self.declare_parameter('speed_xy', 0.15).value)
        self.speed_z = float(self.declare_parameter('speed_z', 0.15).value)
        self.speed_yaw = float(self.declare_parameter('speed_yaw_deg', 30.0).value)
        self.key_timeout = float(self.declare_parameter('key_timeout', 0.25).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)
        if min(self.speed_xy, self.speed_z, self.speed_yaw,
               self.key_timeout, self.rate_hz) <= 0:
            raise ValueError('teleop speed, timeout and rate parameters must be positive')
        self.targets = {'leader': Target(INITIAL_X_OFFSET, 180.0),
                        'follower': Target(-INITIAL_X_OFFSET, 0.0)}
        self.command_publishers = {}
        self.state_subs = []
        for label in self.targets:
            base = f'/{label}/minitrone/'
            self.command_publishers[label] = (
                self.create_publisher(Cmd, base + 'cmd', 10),
                self.create_publisher(AttitudeCmd, base + 'att_cmd', 10))
            self.state_subs.append(self.create_subscription(
                MinitroneState, base + 'state',
                lambda msg, vehicle=label: self.targets[vehicle].update_state(msg),
                10))
        self.admittance_sub = self.create_subscription(
            Bool, '/follower/minitrone/admittance_active', self.on_admittance, 10)
        self.admittance_active = False
        self.trajectory_active = False
        self.trajectory_seen = False
        self.trajectory_heartbeat = 0.0
        self.trajectory_sub = self.create_subscription(
            Bool, '/hih/trajectory_active', self.on_trajectory, 10)
        self.last_tick = time.monotonic()
        self.fd = sys.stdin.fileno()
        self.old_terminal = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            'Follower: a/d -x/+x, w/s -y/+y, r/f +z/-z, q/e +yaw/-yaw. '
            'Leader: j/l +x/-x, i/k -y/+y, y/h +z/-z, u/o +yaw/-yaw. '
            'Space: stop both; Ctrl+C: quit.')

    def destroy_node(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_terminal)
        return super().destroy_node()

    def on_admittance(self, msg):
        active = bool(msg.data)
        follower = self.targets['follower']
        if self.admittance_active and not active and follower.measured_position:
            # Match the wrench controller's pose capture when admittance exits.
            follower.position = follower.measured_position.copy()
            follower.yaw_deg = math.degrees(follower.measured_yaw_rad)
            follower.motion = None
        self.admittance_active = active

    def recapture(self):
        for target in self.targets.values():
            if target.measured_position is not None:
                target.position = target.measured_position.copy()
                target.yaw_deg = math.degrees(target.measured_yaw_rad)
            target.motion = None

    def on_trajectory(self, msg):
        self.trajectory_heartbeat = time.monotonic()
        self.trajectory_seen = True
        active = bool(msg.data)
        if self.trajectory_active and not active:
            self.recapture()
        self.trajectory_active = active

    def read_keys(self):
        # Drain buffered repeats without blocking the ROS timer or the other
        # vehicle's state callbacks.
        for _ in range(16):
            if not select.select([sys.stdin], [], [], 0)[0]:
                break
            key = sys.stdin.read(1).lower()
            if key == ' ':
                for target in self.targets.values():
                    target.motion = None
            elif key in KEYS:
                label, axis, sign = KEYS[key]
                target = self.targets[label]
                target.motion = (axis, sign)
                target.last_key = time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt = max(0.0, min(now - self.last_tick, 0.1))
        self.last_tick = now
        if self.trajectory_active and now - self.trajectory_heartbeat > 0.6:
            self.trajectory_active = False
            # A delayed trajectory heartbeat only releases teleop ownership.
            # Preserve the launch/manual Follower admittance setting.
            self.recapture()
            self.get_logger().warn('Trajectory heartbeat expired; measured poses recaptured')
        if self.trajectory_active:
            # Drain the terminal without allowing queued keys to move a target.
            while select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.read(1)
            return
        self.read_keys()
        # First publish only after BOTH initial states have arrived. This
        # prevents a startup command using stale fallback positions or yaw.
        if not all(target.initialized for target in self.targets.values()):
            return
        for label, target in self.targets.items():
            if now - target.last_key > self.key_timeout:
                target.motion = None
            if target.motion:
                axis, sign = target.motion
                if axis in ('x', 'y'):
                    bx = sign * self.speed_xy if axis == 'x' else 0.0
                    by = sign * self.speed_xy if axis == 'y' else 0.0
                    dx, dy = body_xy_to_world(bx, by, target.measured_yaw_rad)
                    target.position[0] += dx * dt
                    target.position[1] += dy * dt
                elif axis == 'z':
                    target.position[2] += sign * self.speed_z * dt
                else:
                    target.yaw_deg += sign * self.speed_yaw * dt

            pos_pub, yaw_pub = self.command_publishers[label]
            cmd = Cmd()
            cmd.pos_cmd = [float(v) for v in target.position]
            attitude = AttitudeCmd()
            attitude.roll_ref = 0.0
            attitude.pitch_ref = 0.0
            attitude.yaw_ref = float(target.yaw_deg)
            pos_pub.publish(cmd)
            yaw_pub.publish(attitude)


def main():
    if not sys.stdin.isatty():
        raise RuntimeError('Run hih_teleop in a focused interactive terminal')
    rclpy.init()
    node = None
    try:
        node = HIHTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
